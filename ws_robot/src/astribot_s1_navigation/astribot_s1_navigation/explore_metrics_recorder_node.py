# Copyright 2026 Astribot.
#
# 探索到位精度录制器：跟随自主探索运行，逐轮落盘 map 系指标。
#
# ==================== 这个节点做什么、不做什么 ====================
# 做：订阅 → 按轮切分 → 交给 explore_metrics 算 → 落盘。
# 不做：任何判据。所有"算出一个数"的逻辑都在 explore_metrics 的纯函数里，
#       因此都有单测覆盖。判据留在回调里就没有测试覆盖 —— 本包的姿态监控
#       吃过这个亏（顺序写在回调里，20 条测试全绿，把顺序调换后仍然全绿）。
#
# **只读，零发布**。评测器绝不能影响被评测的系统。
#
# ==================== 轮次怎么切 ====================
#   /exploration/current_goal 收到新目标  -> 开一轮（记下发时刻）
#   /exploration/state 离开 NAVIGATING    -> 结束该轮
#       -> ARRIVED : 成功。随后 ARRIVED 驻留窗单独采样，量静置漂移
#       -> 其它    : 失败，outcome 记成实际转到的状态名
# 协调器的 /exploration/state 是单行 key=value 串，自带
# dispatched= succeeded= nav_fail= 等计数器。我方自己数一遍、再和它对账，
# 两边不一致就报出来 —— 单方计数出错时没有任何别的办法能发现。
#
# ==================== 位姿只能来自 map 系 TF ====================
# map -> astribot_torso_base，timeout=0，在执行器线程里查。
#   · 不用 worker 线程：带超时的**动态** TF 查询在 worker 线程里会失败，
#     而静态 TF 正常，于是看着像好的。
#   · **绝不用 /odom 当 map 系位置**：它是 3-DOF 轮式里程计。
#     本项目曾把 /odom 的 y 与 map 系目标的 y 相减，造出 0.43m 的假误差，
#     然后去排查自己造出来的矛盾。真值是 0.092m，一条命令就能算。
#
# ==================== 就绪判据不是"有发布者" ====================
# 实测过 "pub=1 但 0Hz"：BEST_EFFORT 发布者 + RELIABLE 订阅者，
# 一帧都收不到、只有一条 WARNING，而 count_publishers() 恒为真。
# 所以自检报的是**实测拍率**，并且把每个话题上游的真实类型与 QoS 打出来。
# ================================================================
import csv
import gzip
import json
import math
import os

from action_msgs.msg import GoalStatus, GoalStatusArray
import rclpy
from geometry_msgs.msg import PolygonStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.srv import GetParameters
from rclpy.clock import Clock, ClockType
# 关停期要区分"信号收尾"与"真错误"，见 main() 里的说明。RCLError 只在
# 私有模块 rclpy._rclpy_pybind11 里，rclpy 顶层没有导出它。
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from .explore_metrics import geometry, plan_status, round_metrics, scan_metrics
from .explore_metrics.state_parse import coordinator_counters, parse_state_line

STATE_NAVIGATING = 'NAVIGATING'
STATE_ARRIVED = 'ARRIVED'

# 规划请求归入某一轮的时间窗左边界（相对目标下发时刻）。
# 与 plan_history 的回看窗**同一个数**：follow_path 模式下协调器是先规划、
# 校验通过才下发目标，规划必然早于 goal_stamp。两个窗不一致就会出现
# "有路径却没有对应规划请求"的自相矛盾行。
_REQ_LOOKBACK_SEC = 5.0
# 规划请求台账的上限条数。一次长跑几百轮、每轮多次重规划，不设上限就是
# 无界增长；取 400 是因为归轮只回看 5s，远超单轮所需。
# 实测 nav2 每帧 status_list 长 450 条，这里给 1000 是留一倍余量；
# 台账另有"不短于见过的最长一帧"的硬下限兜底（见 plan_status.observe_batch）
_PLAN_REQ_KEEP = 1000

# transient_local + RELIABLE。协调器的 /exploration/complete 是锁存话题，
# 用默认 QoS 订阅会收不到已经发过的那一条。
_LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


class Series:
    """一条时序的累加器。轮开始时清空。"""

    __slots__ = ('t', 'a', 'b', 'c')

    def __init__(self):
        self.t, self.a, self.b, self.c = [], [], [], []

    def add(self, t, a=0.0, b=0.0, c=0.0):
        self.t.append(float(t))
        self.a.append(float(a))
        self.b.append(float(b))
        self.c.append(float(c))

    def clear(self):
        self.t.clear()
        self.a.clear()
        self.b.clear()
        self.c.clear()

    def __len__(self):
        return len(self.t)


class ExploreMetricsRecorder(Node):
    """只读录制器。"""

    def __init__(self):
        super().__init__('explore_metrics_recorder')

        # 状态码是抄进 plan_status 的（那个包刻意 ROS 无关）。抄的值一旦与
        # 上游不符，ABORTED 会被数成 SUCCEEDED —— 那张表每一行都会看起来正常。
        # 所以启动即核对，不符就拒绝启动，绝不带着错码往下跑。
        bad = plan_status.verify_status_codes(GoalStatus)
        if bad:
            raise RuntimeError(
                'action_msgs/GoalStatus 的状态码与 plan_status 抄的常量不符：%s'
                % '; '.join(bad))

        d = self.declare_parameter
        d('output_dir', '/tmp/explore_metrics')
        d('run_label', '')
        # 请求的限速档位。**只作对账用** —— 表里的 vx_max 一律取在线查参的实测值。
        d('speed_cap_requested', -1.0)
        d('map_frame', 'map')
        d('base_frame', 'astribot_torso_base')
        # 位姿采样率。20Hz 是有算术依据的：里程的抖动门限是 2mm，
        # 限速档位最低会走到 0.1m/s -> 单步 5mm，是门限的 2.5 倍。
        # 提到 50Hz 就变成 2mm，与门限**同量级**，真实位移会被当抖动丢掉。
        # 真要改这个数，先把 min_step_m 一起改，并看 traveled_gate_suspicious 列。
        d('pose_rate_hz', 20.0)
        d('state_topic', '/exploration/state')
        d('goal_topic', '/exploration/current_goal')
        d('plan_topic', '/plan')
        # 默认 /scan_from_cloud：scan_source:=slice_scan（默认）时 nav2 订阅的就是它
        d('scan_topic', '/scan_from_cloud')
        d('cmd_raw_topic', '/cmd_vel_nav_body_raw')
        d('cmd_final_topic', '/cmd_vel')
        d('odom_topic', '/odom')
        d('footprint_topic', '/global_costmap/published_footprint')
        d('complete_topic', '/exploration/complete')
        # 规划器 action 状态。**规划耗时这一列的唯一正经口径。**
        # 它是 action 的隐藏话题：`ros2 topic list` 默认看不到（要
        # --include-hidden-topics），但 `topic info -v` 能查到，实测
        # 发布者 1 个、RELIABLE + TRANSIENT_LOCAL。
        d('plan_status_topic', '/compute_path_to_pose/_action/status')
        d('controller_node', '/controller_server')
        d('smoother_node', '/velocity_smoother')
        # 限速实参所在的**参数命名空间**。不能硬编码成 'FollowPath'：
        # 2026-09-07 起 FollowPath 是 ThreePhaseController，vx_max/vx_min 是
        # 内层 MPPI 的键、落在 <实例>.inner 下（nav2_params_mppi.yaml:447/691），
        # `FollowPath.vx_max` 这个键**根本不存在**。实测后果：controller_server
        # 每 5s 打一条 `[rclcpp] Failed to get parameters: FollowPath.vx_max`
        # （rclcpp 的 ParameterService 捕获 ParameterNotDeclaredException 后
        # 返回**空的 values 列表**），记录器把实测限速记成 None，于是
        # 「限速与请求一致」整列不可用 —— 实测 13 轮 0 轮有值。
        #
        # 取 FollowPathExplore.inner 而不是 FollowPath.inner：探索用的行为树
        # behavior_trees/navigate_to_pose_explore_three_phase.xml 里
        # <FollowPath controller_id="FollowPathExplore"/> 是硬编码的，
        # 真正在开车的是 FollowPathExplore。换行为树时这个参数要跟着换。
        #
        # !!! 一次请求里不要混入不存在的键 !!!：GetParameters 服务端是
        # `node->get_parameters(names)` 整体 try/catch，**任一个**名字未声明就
        # 抛异常、整份响应的 values 为空 —— 好的键会被坏的键一起带走。
        d('controller_speed_ns', 'FollowPathExplore.inner')
        # ARRIVED 驻留窗采样时长。协调器自己的 dwell_time_sec 是 1.5，
        # 取得比它长一点才能看到"控制器报完成后还在收敛"那一段
        d('settle_window_sec', 3.0)
        d('goal_tolerance_m', 0.18)
        # 真实碰撞只能人工填：本机没有碰撞传感器，急停按钮在操作者手里
        d('collisions_manual', -1)
        d('low_obstacle_truth_file', '')
        d('dump_samples', True)
        # 自检窗口。实机 DDS 发现实测要 30~50s 收敛，短于这个会把"还没发现"
        # 报成"话题不存在"。本项目已经因此误报过一次系统故障
        d('selfcheck_delay_sec', 45.0)

        g = self.get_parameter
        self.out_dir = str(g('output_dir').value)
        self.run_label = str(g('run_label').value)
        self.speed_cap_requested = float(g('speed_cap_requested').value)
        self.map_frame = str(g('map_frame').value)
        self.base_frame = str(g('base_frame').value)
        self.pose_rate = max(1.0, float(g('pose_rate_hz').value))
        self.settle_window = float(g('settle_window_sec').value)
        self.goal_tol = float(g('goal_tolerance_m').value)
        self.dump_samples = bool(g('dump_samples').value)
        collisions = int(g('collisions_manual').value)
        self.collisions_manual = None if collisions < 0 else collisions

        os.makedirs(self.out_dir, exist_ok=True)
        os.makedirs(os.path.join(self.out_dir, 'samples'), exist_ok=True)

        self.topics = {
            'state': str(g('state_topic').value),
            'goal': str(g('goal_topic').value),
            'plan': str(g('plan_topic').value),
            'scan': str(g('scan_topic').value),
            'cmd_raw': str(g('cmd_raw_topic').value),
            'cmd_final': str(g('cmd_final_topic').value),
            'odom': str(g('odom_topic').value),
            'footprint': str(g('footprint_topic').value),
            'complete': str(g('complete_topic').value),
            'plan_status': str(g('plan_status_topic').value),
        }

        # ---------------------------------------------------------- 状态
        self.rows = []
        self.round = None
        self.pending_settle = None
        self.state_name = None
        self.coord = {}
        self.round_index = 0
        self.goals_seen = 0
        # 对账基线。录制器几乎总是**中途挂上去**的（协调器已经派发过很多轮），
        # 所以协调器的 dispatched 是它自己开机以来的累计，而 goals_seen 是本会话的。
        # 拿绝对值互比等于保证不一致 —— 实测 coord=20,21,22,23 vs 本方=1,3,4,4，
        # 4 轮全报红，红旗因此恒亮、无信息量，还把真正的差异盖住了。
        # 只有**增量**可比，且必须在见到第一个目标时同时取两侧基线。
        self.coord_dispatched_base = None
        self.goals_seen_base = None

        self.pose = Series()
        self.pose_yaw = []
        self.settle = Series()
        self.settle_yaw = []
        self.cmd_raw = Series()
        self.cmd_final = Series()
        self.odom = Series()
        self.scan_t = []
        self.scan_frames = []
        self.plans = []
        self.plan_history = []          # 轮外也留一段，供"下发前那条路径"用
        # 规划 action 的请求台账。轮外也记，理由同 plan_history。
        self.plan_reqs = plan_status.PlanRequestLedger(keep=_PLAN_REQ_KEEP)
        # 时钟未就绪期间丢掉的状态消息数。丢是对的，但必须能看见丢了多少
        self.plan_status_dropped_no_clock = 0

        self.exploration_complete = None
        self.footprint = None
        self.fp_inscribed = None
        self.fp_circumscribed = None
        self.fp_changed_in_round = False
        # 足迹口径自检：不中心对称的帧数、以及"中心明显非零"（即拿到的是
        # 全局系坐标）的帧数。后者是常态，记数是为了让报告能说明
        # 半径是经过平移归一后算的，而不是原样直接算的
        self.fp_not_symmetric = 0
        self.fp_global_frame_frames = 0

        self.measured = {'vx_max': None, 'vx_min': None, 'smoother_max': None}
        self.msg_counts = {k: 0 for k in self.topics}

        # 在线查限速用的参数全名。在这里拼一次，_query_caps 每 5s 复用 ——
        # 报错信息里要能逐字打出问的是哪个名字，否则"查不到"这条告警
        # 本身也是静默的（旧版就是这样：只有 controller_server 那侧打了
        # 一条 `[rclcpp] Failed to get parameters:`，记录器这侧一个字都没有）。
        _ns = str(g('controller_speed_ns').value)
        self.speed_param_names = ['%s.vx_max' % _ns, '%s.vx_min' % _ns]
        # 查参失败只报一次，不要 5s 一条把日志刷满（同一个缺陷刷 72 条/小时）。
        self._caps_warned = set()

        self.low_obstacle_truth = self._load_truth(
            str(g('low_obstacle_truth_file').value))

        # ---------------------------------------------------------- 订阅
        # QoS 逐条对齐上游。单向不兼容会让整条链静默断掉：
        # BEST_EFFORT 发布者 + RELIABLE 订阅者 = 一帧都收不到、只一条 WARNING。
        self.create_subscription(String, self.topics['state'],
                                 self._on_state, 10)
        self.create_subscription(PoseStamped, self.topics['goal'],
                                 self._on_goal, 10)
        self.create_subscription(Path, self.topics['plan'], self._on_plan, 1)
        self.create_subscription(LaserScan, self.topics['scan'],
                                 self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, self.topics['cmd_raw'],
                                 self._mk_twist(self.cmd_raw, 'cmd_raw'), 10)
        self.create_subscription(Twist, self.topics['cmd_final'],
                                 self._mk_twist(self.cmd_final, 'cmd_final'), 10)
        self.create_subscription(Odometry, self.topics['odom'],
                                 self._on_odom, qos_profile_sensor_data)
        self.create_subscription(PolygonStamped, self.topics['footprint'],
                                 self._on_footprint, 1)
        self.create_subscription(Bool, self.topics['complete'],
                                 self._on_complete, _LATCHED)
        # 上游实测 RELIABLE + TRANSIENT_LOCAL。用同样的 QoS 订阅：
        # 默认 QoS（VOLATILE）在这里也兼容，但锁存那一条会随机漏掉，
        # 而漏掉的恰好是"我们连上之前最后一次规划"。
        self.create_subscription(GoalStatusArray, self.topics['plan_status'],
                                 self._on_plan_status, _LATCHED)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_ok = 0
        self.tf_fail = 0

        self.create_timer(1.0 / self.pose_rate, self._sample_pose)
        self.create_timer(0.5, self._tick)
        # 🔴 自检定时器**必须走墙钟**，不能跟着 use_sim_time。
        #
        # use_sim_time=true 时节点时钟在首条 /clock 到达前恒为 0，随后一步
        # 跳到当前仿真时刻（本机实测跳了约 1153s）。ROS 时间的定时器遇到这个
        # 跳变会**立刻**到期 —— 实测启动后 1.3s 就打了自检，于是九个话题
        # 全报"零消息"，正好是这段代码注释里说要避免的那种假故障。
        # 而 DDS 发现要 30~50s 收敛，那是墙钟现象，本来就该用墙钟计时。
        self.create_timer(float(g('selfcheck_delay_sec').value),
                          self._selfcheck, clock=Clock(clock_type=ClockType.SYSTEM_TIME))

        self.param_clients = {
            'controller': self.create_client(
                GetParameters, str(g('controller_node').value) + '/get_parameters'),
            'smoother': self.create_client(
                GetParameters, str(g('smoother_node').value) + '/get_parameters'),
        }
        self.create_timer(5.0, self._query_caps)

        self.get_logger().info(
            '探索评测录制器已启动（**只读**，不发布任何话题）。\n'
            '  输出目录: %s\n'
            '  位姿: TF %s -> %s @ %.0fHz（不用 /odom，它不是 map 系）\n'
            '  订阅: %s\n'
            '  请求限速档位: %s（表里的 vx_max 一律用在线查参的实测值）\n'
            '  自检将在 %.0fs 后打印实测拍率 —— 实机 DDS 发现要 30~50s，'
            '早于此的"话题不存在"是仪器问题不是系统问题'
            % (self.out_dir, self.map_frame, self.base_frame, self.pose_rate,
               ', '.join('%s=%s' % (k, v) for k, v in self.topics.items()),
               '未指定' if self.speed_cap_requested < 0 else
               '%.3f' % self.speed_cap_requested,
               float(g('selfcheck_delay_sec').value)))

    # ------------------------------------------------------------------ 工具

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _clock_ready(self):
        """时钟是否可用于**绝对**时刻比较。

        use_sim_time=true 下，首条 /clock 到达前 get_clock().now() 恒为 0。
        拿这个 0 去和消息自带的时间戳比，差值就是整个仿真已运行时长 ——
        实测把 107/192 条规划请求判成了"两个时间源不同轴"，而真正的原因
        只是我们自己的时钟还没起来。0 不是"很早"，是"还不知道"。
        """
        return (not self._use_sim_time()) or self._now() > 0.0

    def _load_truth(self, path):
        """低矮障碍人工真值。没有文件就是没有 —— 不编造。"""
        if not path:
            return []
        if not os.path.isfile(path):
            self.get_logger().warn(
                '低矮障碍真值文件不存在: %s -> 低矮障碍检出率本次记 n/a' % path)
            return []
        try:
            import yaml
            with open(path, encoding='utf-8') as fh:
                data = yaml.safe_load(fh) or {}
            items = data.get('low_obstacles') or []
            out = [(float(i['x']), float(i['y']), float(i.get('height', 0.0)))
                   for i in items]
            self.get_logger().info('已载入 %d 个低矮障碍真值' % len(out))
            return out
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().error('真值文件解析失败(%s) -> 本项记 n/a' % exc)
            return []

    def _mk_twist(self, series, key):
        def cb(msg):
            self.msg_counts[key] += 1
            series.add(self._now(), msg.linear.x, msg.linear.y, msg.angular.z)
        return cb

    # ------------------------------------------------------------------ 回调

    def _on_odom(self, msg):
        self.msg_counts['odom'] += 1
        # 只取 twist（车体系速度）。**位置一律不取** —— /odom 的 (x,y) 不在 map 系
        self.odom.add(self._now(), msg.twist.twist.linear.x,
                      msg.twist.twist.linear.y, msg.twist.twist.angular.z)

    def _on_complete(self, msg):
        self.msg_counts['complete'] += 1
        if self.exploration_complete != bool(msg.data):
            self.exploration_complete = bool(msg.data)
            self.get_logger().info('探索完成标志 -> %s' % self.exploration_complete)
            # 注意：空地图冷启动时"0 前沿"也会被判 COMPLETED，
            # 那不是探索完成。看到 True 时要一起看已建图面积。
            self._write_run_json()

    def _on_footprint(self, msg):
        self.msg_counts['footprint'] += 1
        raw_pts = [(p.x, p.y) for p in msg.polygon.points]
        # nav2 的 published_footprint 是**全局系绝对坐标**（实测 frame_id=map，
        # 顶点就落在机器人当前位置附近），不是车体系。直接算半径量到的是
        # "地图原点到多边形的距离"，实测随机器人开远单调增长 6.12 -> 8.81m，
        # 而底盘内切只有 0.42m；下游 min_clearance / free_width / self_residual
        # 因此全部作废。所以先平移回对称中心 —— 两个半径都绕原点旋转不变，
        # 平移够了，不需要 TF 也不需要 yaw。
        pts, center = geometry.recenter_polygon(raw_pts)
        if pts is None:
            # 不中心对称 -> 说不出中心在哪，就不能给半径。静默用质心会给出
            # 偏移的半径而每一行看着都正常
            self.fp_not_symmetric += 1
            if self.fp_not_symmetric == 1:
                self.get_logger().error(
                    '足迹多边形不中心对称（%d 顶点）-> 内切/外接半径与净空整列'
                    '记 n/a。本机两个足迹都应当中心对称，请检查上游足迹配置'
                    % len(raw_pts))
            return
        if math.hypot(center[0], center[1]) > 0.05:
            # 中心明显非零即证实拿到的是全局系坐标。记数而不是每帧告警
            self.fp_global_frame_frames += 1
        ins, circ = geometry.polygon_radii(pts)
        if ins is None:
            return
        if self.fp_inscribed is not None and abs(ins - self.fp_inscribed) > 1e-4:
            # 足迹被换过（本项目做过 36% 的临时足迹）。轮内变化必须标出来：
            # 换之后"净空"前后不是同一个口径
            self.fp_changed_in_round = True
            self.get_logger().warn(
                '足迹在录制中发生变化：内切 %.4f -> %.4f。'
                '本轮净空/侵入两列前后口径不一致，已在 '
                'footprint_changed_in_round 列标记' % (self.fp_inscribed, ins))
        self.footprint = pts
        self.fp_inscribed = ins
        self.fp_circumscribed = circ

    def _on_scan(self, msg):
        self.msg_counts['scan'] += 1
        if self.round is None:
            # 驻留窗期间不再采激光：净空/居中度这些量的窗口口径必须
            # 与轮次一致，混进驻留段会把"停下来之后"的观测算进跟踪指标
            return
        f = scan_metrics.frame_stats(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max,
            circumscribed=self.fp_circumscribed)
        self.scan_t.append(self._now())
        self.scan_frames.append(f)

    def _on_plan(self, msg):
        self.msg_counts['plan'] += 1
        entry = {'stamp': self._now(),
                 'points': [(p.pose.position.x, p.pose.position.y)
                            for p in msg.poses]}
        if not entry['points']:
            return
        self.plan_history.append(entry)
        # 只留最近 20 条，够覆盖"下发前那一条"
        del self.plan_history[:-20]
        if self.round is not None:
            self.plans.append(entry)

    def _on_plan_status(self, msg):
        """规划 action 的状态数组 -> 每次请求的 accept/end/终态。

        这个话题是**累积**的：nav2 会把最近若干个目标一直挂在 status_list 里，
        同一个 goal_id 会被反复看到。所以按 goal_id 去重，只记第一次看到
        受理、第一次看到终态。

        accept 用消息自带的 goal_info.stamp，而**不是**收报时刻：
          ① 话题是 TRANSIENT_LOCAL，我们一连上就会收到一条锁存的旧状态。
             拿收报时刻当受理时刻，那条旧请求就会被算成"刚刚发生"，
             正好落进当时那一轮 —— 这就是把陈旧读数当成当前值。
          ② 本项目已经在别处犯过同一类错（陈旧 TF / 冻结拍率）。
        end 只能用收报时刻：消息里没有终态发生时刻这个字段。两者都取自
        get_clock()/同一时间源，仿真下都是 /clock，量纲一致。
        """
        self.msg_counts['plan_status'] += 1
        if not self._clock_ready():
            # 时钟还没起来。这条锁存消息里的绝对时刻此刻**无法比较**，
            # 记数丢弃而不是拿 0 当"现在"去比 —— 后者会把整批历史请求
            # 判成不同轴，并把它们的受理时刻全改成 0。
            self.plan_status_dropped_no_clock += 1
            return
        now = self._now()
        # 整帧喂：台账的裁剪下限要跟着帧长走。实测每帧 450 条而 keep=400，
        # 逐条喂的话每帧都把上一帧的条目裁掉再重建 —— 实测 46 条消息造出
        # 20069 条"第一眼终态"，归轮的请求数会被放大数百倍
        self.plan_reqs.observe_batch(
            ((bytes(st.goal_info.goal_id.uuid), int(st.status),
              st.goal_info.stamp.sec + st.goal_info.stamp.nanosec / 1e9)
             for st in msg.status_list), now)

    def _round_plan_reqs(self, r):
        """归到这一轮的规划请求。窗口 [goal_stamp - 回看, end_stamp]。"""
        return self.plan_reqs.window(r['goal_stamp'] - _REQ_LOOKBACK_SEC,
                                     r['end_stamp'])

    def _on_goal(self, msg):
        self.msg_counts['goal'] += 1
        self.goals_seen += 1
        if self.coord_dispatched_base is None:
            # 两侧基线在同一时刻取。此刻协调器可能已经把本目标记进 dispatched，
            # 也可能还没 —— 这个 ±1 的竞态是无法消除的，判据里按容差处理。
            d = self.coord.get('coord_dispatched')
            if d is not None:
                self.coord_dispatched_base = int(d)
                self.goals_seen_base = self.goals_seen - 1
        if self.round is not None:
            # 上一轮还没被状态机关掉就来了新目标 —— 按抢占处理，旧轮以
            # PREEMPTED 收尾。不这样做的话两轮的数据会混在一起，
            # 而混在一起的那一行看起来完全正常
            self.get_logger().warn(
                '新目标在上一轮结束前到达，旧轮按 PREEMPTED 收尾')
            self._close_round('PREEMPTED', settle=False)
        self._open_round(msg)

    def _on_state(self, msg):
        self.msg_counts['state'] += 1
        fields = parse_state_line(msg.data)
        self.coord = coordinator_counters(fields)
        new = fields.get('state')
        if new is None or new == self.state_name:
            return
        prev, self.state_name = self.state_name, new
        if prev == STATE_NAVIGATING and new != STATE_NAVIGATING:
            self._close_round(new, settle=(new == STATE_ARRIVED))

    # ------------------------------------------------------------------ 采样

    def _lookup_pose(self):
        """map 系位姿。timeout=0，在执行器线程里查。

        用 timeout=0 而不是给一个超时时长：带超时的**动态** TF 查询在
        worker 线程里会失败而静态 TF 正常，那种失败看着像"TF 是空的"。
        timeout=0 从执行器线程查就完全绕开这一类问题，取不到就下一拍再来。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:                               # noqa: BLE001
            self.tf_fail += 1
            return None
        self.tf_ok += 1
        q = tf.transform.rotation
        return (tf.transform.translation.x, tf.transform.translation.y,
                geometry.yaw_from_quat(q.x, q.y, q.z, q.w))

    def _sample_pose(self):
        p = self._lookup_pose()
        if p is None:
            return
        now = self._now()
        if self.round is not None:
            self.pose.add(now, p[0], p[1])
            self.pose_yaw.append(p[2])
        elif self.pending_settle is not None:
            self.settle.add(now, p[0], p[1])
            self.settle_yaw.append(p[2])

    def _tick(self):
        """驻留窗计时。到点就把那一轮真正落盘。"""
        if self.pending_settle is None:
            return
        if self._now() - self.pending_settle['closed_at'] >= self.settle_window:
            self._flush(self.pending_settle)
            self.pending_settle = None

    # ------------------------------------------------------------------ 查参

    def _query_caps(self):
        """在线查限速实参。**表里的限速一律用这个值，不回填命令行。**

        理由：nav2_full_bringup 曾漏转发 max_linear_speed —— 不报错、不告警、
        vx_max 仍是 1.0。回填命令行会让整张表的"最大限速"这一列全是假的，
        而每一行看起来都正常。
        """
        cli = self.param_clients['controller']
        if cli.service_is_ready():
            self._get_params(cli, self.speed_param_names,
                             self._on_controller_params, 'controller')
        cli2 = self.param_clients['smoother']
        if cli2.service_is_ready():
            self._get_params(cli2, ['max_velocity'],
                             self._on_smoother_params, 'smoother')

    def _get_params(self, client, names, done, tag):
        req = GetParameters.Request()
        req.names = names
        fut = client.call_async(req)
        fut.add_done_callback(lambda f: self._param_result(f, done, names, tag))

    def _param_result(self, fut, done, names, tag):
        """查参结果分发。**禁止静默失败**：三条失败路径各自显式告警一次。

        旧版这里是 `except Exception: pass`，后果实测过：参数名的命名空间
        写错（`FollowPath.vx_max`，而三段式控制器把 MPPI 的键放在
        `<实例>.inner` 下）→ 记录器这侧一个字都不打 → 实测限速恒为 None
        → 「限速与请求一致」整列不可用，13 轮 0 轮有值。
        """
        try:
            res = fut.result()
        except Exception as exc:                          # noqa: BLE001
            self._warn_caps('%s:exc' % tag,
                            '服务调用抛异常（问的是 %s）：%r'
                            % (', '.join(names), exc))
            return
        # rclcpp 的 ParameterService 把 `node->get_parameters(names)` 整体
        # try/catch：**任一个**名字未声明就抛 ParameterNotDeclaredException，
        # 服务端只打一条 `[rclcpp] Failed to get parameters: <名字>` 并返回
        # **空的 values 列表** —— 好的键会被坏的键一起带走。
        # 所以"返回项数不等于请求项数"就等价于"至少一个名字不存在"。
        if res is None or len(res.values) != len(names):
            self._warn_caps(
                '%s:missing' % tag,
                '返回 %d 项、期望 %d 项 —— 问的是 %s。'
                'GetParameters 服务端对未声明的名字返回空列表，'
                '所以这几乎一定是**参数名不存在**（命名空间写错）。'
                '后果：实测限速记为 None，报表里「限速与请求一致」这一列不可用。'
                % (0 if res is None else len(res.values), len(names),
                   ', '.join(names)))
            return
        done(res)

    def _warn_caps(self, key, text):
        """同一个缺陷只报一次：查参是 5s 一次，不去重会 72 条/小时刷掉真告警。"""
        if key in self._caps_warned:
            return
        self._caps_warned.add(key)
        self.get_logger().warning('在线查限速参数失败：' + text)

    def _on_controller_params(self, res):
        # 项数已由 _param_result 校验过，这里只管类型。
        # type==3 是 PARAMETER_DOUBLE；类型不对同样不能静默 —— 那说明 yaml 把
        # 限速写成了整数或字符串，而现象与"查不到"完全一样。
        vals = list(res.values)
        for i, key in enumerate(('vx_max', 'vx_min')):
            if vals[i].type == 3:
                self.measured[key] = float(vals[i].double_value)
            else:
                self._warn_caps(
                    'ctrl_type:%s' % key,
                    '%s 的类型是 %d、不是 double(3)，不能当实测限速用'
                    % (self.speed_param_names[i], vals[i].type))

    def _on_smoother_params(self, res):
        vals = list(res.values) if res else []
        if vals and vals[0].type == 8 and len(vals[0].double_array_value):
            self.measured['smoother_max'] = float(vals[0].double_array_value[0])

    # ------------------------------------------------------------------ 轮次

    def _open_round(self, goal_msg):
        # 上一轮还挂在 settle 窗里就来了新目标：**先把它落盘**，再清缓冲。
        # 不这样做的话下面的 clear() 会把它的 settle 采样一起清掉，
        # 静置漂移那一列就静默变空（run11 实测 18 轮只有 7 轮有值）。
        # 截断这件事要显式记在数据里，不能让读表的人以为那是完整的驻留窗。
        if self.pending_settle is not None:
            self.pending_settle['settle_truncated'] = True
            self.get_logger().warn(
                '轮 #%s 的 %.1fs 驻留窗被新目标打断（实采 %d 个样本），'
                '已按截断落盘：静置漂移那一列偏小'
                % (self.pending_settle.get('index'), self.settle_window,
                   len(self.settle.t)))
            self._flush(self.pending_settle)
            self.pending_settle = None

        for s in (self.pose, self.cmd_raw, self.cmd_final, self.odom):
            s.clear()
        self.pose_yaw.clear()
        self.settle.clear()
        self.settle_yaw.clear()
        self.scan_t.clear()
        self.scan_frames.clear()
        self.fp_changed_in_round = False

        q = goal_msg.pose.orientation
        self.round = {
            'index': self.round_index,
            'goal': (goal_msg.pose.position.x, goal_msg.pose.position.y,
                     geometry.yaw_from_quat(q.x, q.y, q.z, q.w)),
            'goal_stamp': self._now(),
            'goal_frame': goal_msg.header.frame_id,
        }
        self.round_index += 1
        # 下发前最近那条路径也算本轮的参考：follow_path 模式下路径是协调器
        # 先校验好再交给控制器的，它的发布时刻**早于**目标下发
        self.plans = [p for p in self.plan_history
                      if p['stamp'] >= self.round['goal_stamp']
                      - _REQ_LOOKBACK_SEC]

        if goal_msg.header.frame_id and goal_msg.header.frame_id != self.map_frame:
            # 目标不在 map 系时，与 map 系位姿相减就是在造假误差。
            # 报出来而不是静默换算：换算需要一次 TF 查询，而那一步失败会
            # 静默退化，比直接说"口径不对"更糟
            self.get_logger().error(
                '目标 frame 是 %r 而不是 %r —— 到位误差这一列口径不对，'
                '请检查协调器的 map_frame 配置'
                % (goal_msg.header.frame_id, self.map_frame))

        self.get_logger().info(
            '轮 #%d 开始：目标 (%.3f, %.3f) yaw=%.3f  [map 系]'
            % (self.round['index'], self.round['goal'][0],
               self.round['goal'][1], self.round['goal'][2]))

    def _close_round(self, outcome, settle):
        if self.round is None:
            return
        # 收尾前**立刻再查一次 TF**，把这一帧作为"到位位置"。
        #
        # 为什么不能直接用最后一个采样点：位姿采样是 20Hz 定时器，
        # 轮次结束的时刻由状态话题触发，两者不同步 —— 最后一个采样点最多比
        # 结束时刻旧一个采样周期（50ms）。到位误差是这张表的头号指标，
        # 白搭进去一个采样周期的滞后没有必要。
        # 离线复核时能看出这一点：settle 窗的首帧会比 pose 末帧新。
        fresh = self._lookup_pose()
        if fresh is not None:
            self.pose.add(self._now(), fresh[0], fresh[1])
            self.pose_yaw.append(fresh[2])

        r = dict(self.round)
        r['outcome'] = outcome
        r['closed_at'] = self._now()
        r['end_stamp'] = r['closed_at']
        # 关窗这一刻就把轮内采样缓冲**值拷贝**下来。
        #
        # 为什么必须在这里拷（run11 实测的数据损坏，18 轮里毁了 9 轮）：
        # settle=True 时本轮不立刻落盘，要挂到 pending_settle 上等 settle_window
        # 秒。而 _build_raw 读的是 self.pose / self.cmd_* / self.odom 这些**活缓冲**。
        # 如果这 3 秒里下一个目标到了，_open_round 会把它们全 clear() 再重新填 ——
        # 于是 3 秒后落盘时写进"旧轮"文件里的，是**新轮**开头那几百毫秒的样本。
        # 特征是样本时间落在旧轮窗口的 107%~146% 位置（轮 0 窗口 4.08~12.28s，
        # 样本却在 14.95~15.50s），且 distance_travelled_m 恒 0.000。
        # 受影响的是一切从缓冲算出来的列（里程/横偏/净空/窄段/居中）；
        # final_pose_err_m 因为是关窗现查 TF 才幸免。
        r['_snapshot'] = self._snapshot_buffers()
        self.round = None
        if settle:
            self.settle.clear()
            self.settle_yaw.clear()
            self.pending_settle = r
            self.get_logger().info(
                '轮 #%d 结束(%s)，开始 %.1fs 驻留窗采样（量静置漂移）'
                % (r['index'], outcome, self.settle_window))
        else:
            self._flush(r)

    def _snapshot_buffers(self):
        """把轮内采样缓冲值拷贝一份。**只拷轮内的，不拷 settle**。

        settle 窗是关窗**之后**才开始采的（量到位后的静置漂移），
        落盘时它才是活的那一份，所以它必须留在 _build_raw 里现取。
        """
        return {
            'pose_t': list(self.pose.t), 'pose_x': list(self.pose.a),
            'pose_y': list(self.pose.b), 'pose_yaw': list(self.pose_yaw),
            'plans': list(self.plans),
            'cmd_raw_t': list(self.cmd_raw.t), 'cmd_raw_vx': list(self.cmd_raw.a),
            'cmd_raw_vy': list(self.cmd_raw.b), 'cmd_raw_wz': list(self.cmd_raw.c),
            'cmd_final_t': list(self.cmd_final.t),
            'cmd_final_vx': list(self.cmd_final.a),
            'cmd_final_vy': list(self.cmd_final.b),
            'cmd_final_wz': list(self.cmd_final.c),
            'odom_t': list(self.odom.t), 'odom_vx': list(self.odom.a),
            'odom_vy': list(self.odom.b),
            'scan_t': list(self.scan_t), 'scan_frames': list(self.scan_frames),
            'footprint_changed': self.fp_changed_in_round,
        }

    def _build_raw(self, r):
        # 轮内采样一律从关窗时的快照取，绝不从 self.* 活缓冲取 —— 见 _close_round
        # 里那段注释（延后落盘 + 下一轮 clear() 造成的跨轮污染）。
        snap = r.get('_snapshot') or {}
        out = {
            'index': r['index'],
            'goal': r['goal'],
            'goal_stamp': r['goal_stamp'],
            'end_stamp': r['end_stamp'],
            'outcome': r['outcome'],
            # settle 是关窗后才采的，落盘这一刻它才齐，所以这三行现取
            'settle_t': list(self.settle.t), 'settle_x': list(self.settle.a),
            'settle_y': list(self.settle.b), 'settle_yaw': list(self.settle_yaw),
            'settle_truncated': bool(r.get('settle_truncated')),
            'plan_requests': self._round_plan_reqs(r),
            'footprint_inscribed': self.fp_inscribed,
            'footprint_circumscribed': self.fp_circumscribed,
            'goal_tolerance': self.goal_tol,
            'vx_max_measured': self.measured['vx_max'],
            'vx_min_measured': self.measured['vx_min'],
            'smoother_max_velocity_measured': self.measured['smoother_max'],
            'speed_cap_requested': (None if self.speed_cap_requested < 0
                                    else self.speed_cap_requested),
            'collisions_manual': self.collisions_manual,
            'low_obstacle_truth': self.low_obstacle_truth,
            'low_obstacle_detections': [],
            # 时钟源要落盘：实机 /clock 发布者为 0，误开 use_sim_time 会让
            # 所有时间戳恒为 0 而 costmap 照发、判据照过
            'clock_source': self._clock_source(),
            'use_sim_time': self._use_sim_time(),
        }
        out.update(snap)
        if not snap:
            # 没有快照说明这一轮走了旧路径，宁可显式报错也不要静默产出
            # 一行"看起来完全正常"的假数据。
            self.get_logger().error(
                '轮 #%s 没有采样快照 —— 轮内所有派生列（里程/横偏/净空）都不可信'
                % r.get('index'))
        return out

    def _flush(self, r):
        raw = self._build_raw(r)
        try:
            row = round_metrics.compute_round(raw)
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().error(
                '轮 #%d 指标计算失败(%s) —— 该轮记为计算失败，'
                '但原始时序仍会落盘，可离线复算' % (r['index'], exc))
            row = {'round_index': r['index'], 'outcome': r['outcome'],
                   'success': False, 'compute_error': str(exc)}
        row.update(self.coord)
        # 双路对账：只比**增量**，绝不比绝对值（基线见 __init__ 处的说明）。
        row['recorder_goals_seen'] = self.goals_seen
        row['coord_dispatched_base'] = self.coord_dispatched_base
        d_now = self.coord.get('coord_dispatched')
        if self.coord_dispatched_base is None or d_now is None:
            # 没拿到基线就**不下结论**，而不是默认"一致"。
            row['coord_dispatched_delta'] = None
            row['recorder_goals_delta'] = None
            row['counter_mismatch'] = None
        else:
            cd = int(d_now) - self.coord_dispatched_base
            rd = self.goals_seen - self.goals_seen_base
            row['coord_dispatched_delta'] = cd
            row['recorder_goals_delta'] = rd
            # 容差 1：取基线那一刻谁先自增是竞态，差 1 不构成异常。
            # 差 >1 是真差异 —— 两侧数的**不是同一件事**：本方数 /goal_pose
            # 消息，协调器数它自己的派发。nav2 侧重发/抢占会让一次派发对应
            # 多条目标消息（见 [[nav2-action-resend-preempts-and-looks-like-failure]]）。
            row['counter_mismatch'] = bool(abs(cd - rd) > 1)
        row['speed_cap_matches_request'] = self._cap_matches()
        row['tf_lookup_ok'] = self.tf_ok
        row['tf_lookup_fail'] = self.tf_fail

        self.rows.append(row)
        self._write_rounds_csv()
        self._write_run_json()
        if self.dump_samples:
            self._dump_samples(raw)

        self.get_logger().info(
            '轮 #%d 落盘：%s  到位误差 %s m  航向误差 %s rad  '
            '静置漂移 %s m  实测限速 %s  耦合衰减 p50=%s'
            % (row.get('round_index'), row.get('outcome'),
               _fmt(row.get('arrival_error_xy_m')),
               _fmt(row.get('arrival_error_yaw_rad')),
               _fmt(row.get('settle_drift_xy_m')),
               _fmt(row.get('vx_max_measured')),
               _fmt(row.get('coupling_atten_p50'))))
        if row['counter_mismatch']:
            self.get_logger().warn(
                '计数对账不一致（比的是增量）：协调器派发 +%s，本录制器看到 +%s 个目标'
                '（基线 dispatched=%s）。两侧数的不是同一件事：本方数 /goal_pose '
                '消息，协调器数它自己的派发，一次派发被重发/抢占会对应多条消息。'
                '差值即漏录/多录的轮数，这一列不查清楚，成功率就不可信'
                % (row['coord_dispatched_delta'], row['recorder_goals_delta'],
                   self.coord_dispatched_base))
        elif row['counter_mismatch'] is None:
            self.get_logger().warn(
                '计数对账**无法进行**：协调器 dispatched 没读到（coord=%s）。'
                '这不等于一致 —— 本轮的成功率没有第二路证据'
                % self.coord.get('coord_dispatched'))
        if row['speed_cap_matches_request'] is False:
            self.get_logger().error(
                '实测 vx_max=%s 与请求档位 %.3f 不一致 —— 本档位数据不可用。'
                '最常见原因是外层 launch 没有把 max_linear_speed 转发下去'
                % (_fmt(self.measured['vx_max']), self.speed_cap_requested))

    def _use_sim_time(self):
        try:
            return bool(self.get_parameter('use_sim_time').value)
        except Exception:                               # noqa: BLE001
            return False

    def _clock_source(self):
        if not self._use_sim_time():
            # 对称的另一半：use_sim_time=false 而 /clock **有**发布者，
            # 说明整条栈在仿真时间上跑，而我们在拿墙钟给它计时。
            # 后果不是"差一点"：Gazebo 的 RTF 不是 1，所有时长/速度/拍率
            # 都会被系统性乘上一个未知因子，而每一行看起来都正常。
            # run1（18 轮）就是这么跑的 —— 落盘的 clock_source 只写了
            # 'system'，没人会从这三个字里看出整表的时间轴是错的。
            try:
                n = self.count_publishers('/clock')
            except Exception:                           # noqa: BLE001
                return 'system'
            return 'system' if n == 0 else 'system_but_clock_publisher_exists'
        try:
            n = self.count_publishers('/clock')
        except Exception:                               # noqa: BLE001
            # SIGINT 后 rclpy 先关 context，而 finish() 还要写最后一份 run.json。
            # 这里不吞掉会让整份 run.json 写不出去（实测 RCLError
            # "rcl node's context is invalid"），把一次完整的跑机数据丢掉。
            # 不静默：返回一个能在报告里看见的枚举值，而不是假装是 'sim'。
            return 'sim_context_invalid_at_shutdown'
        # use_sim_time=true 而 /clock 无发布者 = 时钟恒为 0，永不前进。
        # 实机踩过：六个 nav2 节点时钟全冻在 0，而 costmap 照发、判据照过
        return 'sim' if n > 0 else 'sim_but_no_clock_publisher'

    def _cap_matches(self):
        if self.speed_cap_requested < 0 or self.measured['vx_max'] is None:
            return None
        return abs(self.measured['vx_max'] - self.speed_cap_requested) < 1e-6

    # ------------------------------------------------------------------ 落盘

    def _path(self, name):
        stem = ('%s_' % self.run_label) if self.run_label else ''
        return os.path.join(self.out_dir, stem + name)

    def _write_rounds_csv(self):
        """整表重写而不是追加。

        轮数是几十量级，重写的开销可以忽略；换来的是**列集合永远自洽** ——
        追加时一旦某轮多出/少掉一列，表头就和后面的行错位，
        而错位后的 CSV 用 pandas 读进来完全不报错，只是列的含义全变了。
        """
        cols = []
        for row in self.rows:
            for k in row:
                if k not in cols:
                    cols.append(k)
        path = self._path('rounds.csv')
        tmp = path + '.tmp'
        with open(tmp, 'w', encoding='utf-8', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=cols, extrasaction='ignore')
            w.writeheader()
            for row in self.rows:
                w.writerow({k: row.get(k) for k in cols})
        os.replace(tmp, path)       # 原子替换：中途被 kill 不会留半张表

    def _write_run_json(self):
        summary = round_metrics.summarize(self.rows)
        doc = {
            'run_label': self.run_label,
            'speed_cap_requested': (None if self.speed_cap_requested < 0
                                    else self.speed_cap_requested),
            'speed_cap_measured': self.measured,
            'speed_cap_matches_request': self._cap_matches(),
            'footprint': {
                'vertices': self.footprint,
                'inscribed_m': self.fp_inscribed,
                'circumscribed_m': self.fp_circumscribed,
                # published_footprint 是全局系绝对坐标，半径是平移到对称中心后
                # 算的。这两个计数让报告能自证口径，而不是让人假定它是车体系
                'recentered_from_global_frames': self.fp_global_frame_frames,
                'not_symmetric_frames': self.fp_not_symmetric,
            },
            'topics': self.topics,
            'message_counts': dict(self.msg_counts),
            'tf': {'frame': '%s -> %s' % (self.map_frame, self.base_frame),
                   'ok': self.tf_ok, 'fail': self.tf_fail},
            'use_sim_time': self._use_sim_time(),
            'clock_source': self._clock_source(),
            'pose_rate_hz': self.pose_rate,
            'thresholds': dict(round_metrics.DEFAULTS),
            # 代理量的定义必须跟着数据走 —— 只写在代码注释里，
            # 半年后没人能复核这张表
            'proxy_definitions': dict(round_metrics.PROXY_DEFINITIONS),
            'coordinator_counters': dict(self.coord),
            'recorder_goals_seen': self.goals_seen,
            # 规划耗时的口径证据。请求数为 0 时表里那一列是退化口径，
            # 不能与有请求的跑次放在一起比较
            'plan_requests': {
                'ledger_entries': len(self.plan_reqs),
                'messages': self.msg_counts.get('plan_status', 0),
                'stamp_fallbacks': self.plan_reqs.stamp_fallbacks,
                'stamp_in_future': self.plan_reqs.stamp_in_future,
                'terminal_on_first_sight': self.plan_reqs.terminal_on_first_sight,
                'keep_raised_to': self.plan_reqs.keep_raised_to,
                'newest_accept_stamp': self.plan_reqs.newest_accept(),
                'dropped_before_clock_ready': self.plan_status_dropped_no_clock,
            },
            'exploration_complete_flag': self.exploration_complete,
            'summary': summary,
            'caveats': [
                '到位位置来自 map 系 TF，不是 /odom（/odom 是 3-DOF 轮式里程计）',
                'yaw_goal_tolerance 在本机配置里≈3.15（等于不约束朝向），'
                '航向到位误差这一列仅供观察，不是收敛判据',
                '碰撞次数无传感器，只有几何侵入代理量与人工填写的 collisions_manual',
                '低矮障碍检出率需 --low-obstacle-truth 人工真值，否则为 n/a；'
                '且 /scan 是 z 切片 [0.05,1.63]，低于 0.05m 的障碍按构造不可见',
                '限速档位一律取在线查参实测值；与请求值不符时该档位数据不可用',
            ],
        }
        path = self._path('run.json')
        tmp = path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, ensure_ascii=False, indent=2, default=str)
        os.replace(tmp, path)

    def _dump_samples(self, raw):
        """原始时序压缩落盘，供事后离线复算。

        指标算错过、口径改过，都需要能拿原始数据重算而不必重跑机器人 ——
        实机跑一轮的成本远高于存几 MB。
        """
        name = os.path.join(self.out_dir, 'samples',
                            '%sround_%03d.csv.gz'
                            % (('%s_' % self.run_label) if self.run_label else '',
                               raw['index']))
        try:
            with gzip.open(name, 'wt', encoding='utf-8', newline='') as fh:
                w = csv.writer(fh)
                w.writerow(['channel', 't', 'a', 'b', 'c'])
                for ch, keys in (
                        ('pose', ('pose_t', 'pose_x', 'pose_y', 'pose_yaw')),
                        ('settle', ('settle_t', 'settle_x', 'settle_y',
                                    'settle_yaw')),
                        ('cmd_raw', ('cmd_raw_t', 'cmd_raw_vx', 'cmd_raw_vy',
                                     'cmd_raw_wz')),
                        ('cmd_final', ('cmd_final_t', 'cmd_final_vx',
                                       'cmd_final_vy', 'cmd_final_wz')),
                        ('odom', ('odom_t', 'odom_vx', 'odom_vy', None))):
                    t = raw.get(keys[0]) or []
                    for i in range(len(t)):
                        vals = []
                        for k in keys[1:]:
                            seq = raw.get(k) if k else None
                            vals.append(seq[i] if seq is not None
                                        and i < len(seq) else '')
                        w.writerow([ch, t[i]] + vals)
                for i, f in enumerate(raw.get('scan_frames') or []):
                    st = raw['scan_t'][i] if i < len(raw['scan_t']) else ''
                    w.writerow(['scan', st, f['min_range'], f['left_min'],
                                f['right_min']])
                for p in raw.get('plans') or []:
                    w.writerow(['plan', p['stamp'], len(p['points']), '', ''])
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn('原始时序落盘失败(%s)，指标行不受影响' % exc)

    # ------------------------------------------------------------------ 自检

    def _describe(self, topic):
        """报出上游话题**实际的**类型与 QoS，不让人靠猜。"""
        try:
            infos = self.get_publishers_info_by_topic(topic)
        except Exception as exc:                        # noqa: BLE001
            return '查询失败(%s)' % exc
        if not infos:
            return '**没有任何发布者**（上游没起来，或 domain/DDS profile 不一致）'
        return '; '.join(
            '%s 类型=%s reliability=%s durability=%s'
            % (i.node_name, i.topic_type,
               str(i.qos_profile.reliability).split('.')[-1],
               str(i.qos_profile.durability).split('.')[-1])
            for i in infos)

    def _selfcheck(self):
        """一次性自检：按**实测消息数**判就绪，不按"有没有发布者"。

        实测过 "pub=1 但 0Hz"：BEST_EFFORT 发布者 + RELIABLE 订阅者，
        一帧都收不到、只有一条 WARNING，而 count_publishers() 恒为真。
        """
        lines = ['==================== 录制器自检 ====================']
        dead = []
        for key, topic in sorted(self.topics.items()):
            n = self.msg_counts[key]
            mark = 'OK' if n > 0 else '**零消息**'
            lines.append('  %-10s %-34s 收到 %6d 条  %s' % (key, topic, n, mark))
            if n == 0:
                dead.append((key, topic))
        lines.append('  %-10s %-34s 成功 %6d / 失败 %d'
                     % ('tf', '%s->%s' % (self.map_frame, self.base_frame),
                        self.tf_ok, self.tf_fail))
        lines.append('  实测限速 vx_max=%s vx_min=%s smoother_max=%s'
                     % (_fmt(self.measured['vx_max']),
                        _fmt(self.measured['vx_min']),
                        _fmt(self.measured['smoother_max'])))
        # 查参没拿到值必须在自检里直说。它是"报表某一列整列不可用"，
        # 而不是"某一行缺一个数" —— 只打一个 None 会被当成无关紧要的空位。
        if self.measured['vx_max'] is None:
            lines.append(
                '  🔴 实测限速查不到（问的是 %s，节点 %s）—— '
                '「限速与请求一致」这一列**整列不可用**。'
                '最常见原因是参数命名空间写错：三段式控制器把内层 MPPI 的键放在'
                ' <实例>.inner 下，`FollowPath.vx_max` 这个键并不存在。'
                '用 `ros2 param list %s | grep vx_max` 核对真实名字，'
                '再改 -p controller_speed_ns:=<命名空间>'
                % (', '.join(self.speed_param_names),
                   self.get_parameter('controller_node').value,
                   self.get_parameter('controller_node').value))
        lines.append('  时钟源 %s（use_sim_time=%s）'
                     % (self._clock_source(), self._use_sim_time()))
        if self._clock_source() == 'system_but_clock_publisher_exists':
            lines.append(
                '  🔴 /clock 有发布者而本节点用墙钟计时 —— **整表时间轴不可用**。'
                'Gazebo 的 RTF 不是 1，所有时长/速度/拍率都被乘了一个未知因子。'
                '重启时加 -p use_sim_time:=true')
        lines.append('  规划请求台账 %d 条（其中 %d 条 goal_info.stamp 为 0，'
                     '只能退回收报时刻 —— 这些条目的受理时刻偏晚）'
                     % (len(self.plan_reqs), self.plan_reqs.stamp_fallbacks))
        if self.plan_status_dropped_no_clock:
            lines.append(
                '  规划状态消息有 %d 条在时钟就绪前到达、已丢弃（锁存的历史'
                '状态，绝对时刻此刻无法比较）'
                % self.plan_status_dropped_no_clock)
        newest = self.plan_reqs.newest_accept()
        if newest is not None:
            # 「台账里全是历史」和「时间轴不对」长得一样，必须把龄期显式印出来
            # 见 [[frozen-counter-read-as-current-value]]
            lines.append('  最近一次规划受理于 %.1fs，距今 %.1fs'
                         % (newest, self._now() - newest))
        if self.plan_reqs.keep_raised_to:
            lines.append(
                '  台账裁剪下限已按实测帧长抬到 %d 条（配置值 %d）——'
                '下限低于帧长会让每帧条目全部重建、请求数被放大数百倍'
                % (self.plan_reqs.keep_raised_to, _PLAN_REQ_KEEP))
        if self.plan_reqs.terminal_on_first_sight:
            lines.append(
                '  %d 条规划请求第一眼就是终态（锁存话题订阅上来的历史）——'
                '状态码计入失败率，但耗时不可知已留空'
                % self.plan_reqs.terminal_on_first_sight)
        if self.plan_reqs.stamp_in_future:
            lines.append(
                '  🔴 %d 条 goal_info.stamp 落在收报时刻**之后**超过 %.1fs —— '
                '受理时刻不可能在未来，两个时间源不同轴（最常见就是 '
                'use_sim_time 配错）。这些条目已退回收报时刻，规划耗时会偏小'
                % (self.plan_reqs.stamp_in_future,
                   self.plan_reqs.max_future_skew_sec))
        if self.msg_counts['plan_status'] == 0:
            lines.append(
                '  规划 action 状态零消息 -> planning_time 整列会退化成'
                '「目标下发 → 首条 /plan」，那不是规划耗时。'
                '这个话题是 action 隐藏话题，`ros2 topic list` 默认不显示，'
                '要用 `ros2 topic info -v %s` 查' % self.topics['plan_status'])
        for key, topic in dead:
            lines.append('  %s 零消息 -> 上游实况：%s' % (topic, self._describe(topic)))
        if dead:
            lines.append(
                '  排查顺序：① 上游话题类型/QoS 是否与本节点不匹配'
                '（单向不兼容会一帧都收不到、只一条 WARNING）'
                ' ② ROS_DOMAIN_ID 是否一致（实机是 25，查询 shell 不带就是'
                '"节点全都 Node not found"）'
                ' ③ 话题名是否被 remap')
        if self.tf_ok == 0:
            lines.append(
                '  TF 一次都没取到 -> 到位精度整列不可用。'
                '注意根 frame 是 astribot_torso_base（不是 base_link），'
                '且实机 /tf 的发布者本来就只有我们自己的节点')
        lines.append('====================================================')
        self.get_logger().info('\n'.join(lines))

    def finish(self):
        """收工：把还没落盘的轮次写出去。"""
        if self.round is not None:
            self._close_round('INTERRUPTED', settle=False)
        if self.pending_settle is not None:
            self._flush(self.pending_settle)
            self.pending_settle = None
        self._write_rounds_csv()
        self._write_run_json()
        self.get_logger().info(
            '录制结束：%d 轮已落盘 -> %s'
            % (len(self.rows), self._path('rounds.csv')))


def _fmt(v):
    if v is None:
        return 'n/a'
    if isinstance(v, bool):
        return str(v)
    try:
        f = float(v)
    except (TypeError, ValueError):
        return str(v)
    return 'n/a' if not math.isfinite(f) else '%.4f' % f


def main(args=None):
    rclpy.init(args=args)
    node = ExploreMetricsRecorder()
    # 结束原因要**显式区分**并落到日志里，不能一律吞掉：
    #   · SIGINT/SIGTERM（Ctrl-C、脚本 kill -INT、timeout 到点）是评测会话的
    #     正常收尾 —— rclpy 的信号处理器会先把 context 关掉，于是 spin() 抛的
    #     是 ExternalShutdownException 或 RCLError("context is not valid")，
    #     **不是** KeyboardInterrupt。原来只接 KeyboardInterrupt，实机上
    #     kill -TERM 就会在 finish() 正常写完 CSV 之后再吐一整段 traceback，
    #     看日志的人会以为录制崩了、CSV 不可信。
    #   · 其它异常必须原样抛出去、退出码非 0 —— 静默成功是本项目的红线。
    ended_by = None
    try:
        rclpy.spin(node)
        ended_by = '节点自行退出'
    except KeyboardInterrupt:
        ended_by = 'Ctrl-C (SIGINT)'
    except ExternalShutdownException:
        ended_by = '外部关停信号 (SIGINT/SIGTERM)'
    except RCLError as exc:
        # context 已失效 == 信号处理器已经关停过，同上；其余 RCL 错误照抛。
        if rclpy.ok():
            raise
        ended_by = '外部关停信号（context 已失效: %s）' % exc
    finally:
        try:
            node.finish()
            if ended_by:
                # 用 print 而不是 node.get_logger()：此刻 context 可能已经失效，
                # 走 rosout 会再打一条 "Failed to publish log message to rosout"。
                print('[explore_metrics_recorder] 结束原因：%s' % ended_by, flush=True)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
