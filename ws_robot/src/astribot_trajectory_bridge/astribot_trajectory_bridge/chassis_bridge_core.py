#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘桥接的控制核心（状态机）。**不依赖 rclpy、不依赖厂商 SDK。**

依赖全部通过端口注入（见 ports.py），所以使能/leash/闭环/异常这些分支可以在
离线环境里全部跑通 —— 这是本模块不必等真机就能被验证的前提。

状态机
======
::

    DISABLED ──enable()──▶ ENABLED ──leash 触发──▶ LEASH_TRIPPED
       ▲                    │  ▲                        │
       │                    │  └───reset_leash()────────┘
       │                    │
       │                    └──位姿丢失且 require_slam──▶ STOPPED_NO_POSE
       │                                                     │
       └──────────── disable() / 需重新 enable ───────────────┘

设计要点（每一条都对应一个已知风险）
==================================
* **积分种子每次使能重取**（202:44 / 203:43）。复用旧 pos_cmd 的后果：两次使能
  之间机器人被推动或自行漂移，积分起点与真实位置有偏差，一使能就是一个**阶跃
  位置指令** → 底盘猛冲。
* **leash 触发时必须同时冻结积分与校正**。只冻结积分而让外环继续推，校正会持续
  把指令往外拽，等于 leash 没起作用。
* **异常绝不吞**。任何 SDK 调用失败都转成状态位，由调用方上报；桥接不因单次
  失败退出（退出会让 /cmd_vel 彻底断流，比继续上报更糟）。
* **看门狗只把速度置零，不改状态**。cmd_vel 短暂中断是正常工况（Nav2 到点后
  就不发了），不该因此进入故障态。
"""

import collections
import math

from astribot_trajectory_bridge.chassis_integrator import (
    ChassisConfigError,
    IDX_THETA,
    clamp_velocity,
    integrate_step_dt,
    measure_tick_dt,
    slew_limit_velocity,
    to_local_velocity,
    wrap_angle,
)
from astribot_trajectory_bridge.chassis_feedback import (
    POSE_SOURCE_GROUND_TRUTH,
    advance_desired_pose,
    check_leash,
    compute_correction,
    detect_pose_jump,
    effective_thresholds,
    is_correction_degenerate,
    leash_recover_command,
    odom_drift,
    slice_correction,
    validate_correction_config,
    validate_leash_config,
)

# 状态
ST_DISABLED = 'DISABLED'
ST_ENABLED = 'ENABLED'
ST_LEASH_TRIPPED = 'LEASH_TRIPPED'
ST_STOPPED_NO_POSE = 'STOPPED_NO_POSE'
#: 因 /scan 持续陈旧而闩锁停车。
#: 刻意**不复用** ST_STOPPED_NO_POSE：本文件里已有一条同样的教训 ——
#: 位姿查询失败报 POSE_PORT_FAILED 而不是 SDK_CALL_FAILED，因为"报错类别错了
#: 会把诊断引向错的子系统"。感知瞎了和定位丢了是两个不同的子系统，混成一个
#: 状态就等于在最需要分辨的时候丢掉了分辨能力。
ST_STOPPED_STALE_SCAN = 'STOPPED_STALE_SCAN'

# 上报用的状态位名（与 BridgeStatus.msg 的枚举同名，由节点层映射成数字）
S_OK = 'OK'
S_NOT_ENABLED = 'NOT_ENABLED'
S_SDK_CALL_FAILED = 'SDK_CALL_FAILED'
S_CMD_VEL_TIMEOUT = 'CMD_VEL_TIMEOUT'
S_LEASH_TRIPPED = 'LEASH_TRIPPED'
S_SLAM_UNAVAILABLE_OPEN_LOOP = 'SLAM_UNAVAILABLE_OPEN_LOOP'
S_SLAM_STALE = 'SLAM_STALE'
S_SLAM_RELOCALIZED = 'SLAM_RELOCALIZED'
S_ODOM_DRIFT_HIGH = 'ODOM_DRIFT_HIGH'
S_CORRECTION_DEGENERATE = 'CORRECTION_DEGENERATE'
S_SLAM_LOST_STOPPED = 'SLAM_LOST_STOPPED'
S_POSE_PORT_FAILED = 'POSE_PORT_FAILED'
#: /scan 陈旧，本拍速度已置零（可自动恢复）。metric_1=龄期(s)，metric_2=阈值(s)。
S_SCAN_STALE = 'SCAN_STALE'
#: /scan 持续陈旧超过宽限期，已闩锁停车（要人介入）。
S_SCAN_LOST_STOPPED = 'SCAN_LOST_STOPPED'
#: 从未收到过 /scan。与"收到过但变旧了"分开报 —— 前者通常是话题名/QoS 配错，
#: 后者是上游故障，两者的排查方向完全不同。本项目已因 QoS 单向不兼容
#: （BEST_EFFORT 发布 + RELIABLE 订阅，一帧都收不到、只有一条 WARNING）
#: 浪费过一轮排查，这个区分是为那个场景留的。
S_SCAN_NEVER_RECEIVED = 'SCAN_NEVER_RECEIVED'
#: 内环步长被钳位。metric_1=钳位前的实测步长(s)，metric_2=上限(s)。
#: 偶发说明调度抖动，持续出现说明内环真的跟不上，两种都必须可见 ——
#: 步长直接乘在速度上，静默钳位等于静默改变底盘速度。
S_TICK_DT_CLAMPED = 'TICK_DT_CLAMPED'

#: :meth:`ChassisBridgeCore.tick_stats` 的返回值。
#: ``rate_hz`` 是 count / 墙钟时长，**无偏** —— 与只在超阈时上报的
#: LOOP_OVERRUN 不同，后者的周期均值是截尾样本，不能当平均周期用。
#: ``live``：统计窗口是否**正在累积**（核心处于使能态）。停用后 count/rate 会
#: 停在上一段使能期间的值上，若不带这个标志，陈旧读数与当前读数长得一模一样。
#: ``max_dt``：本段内**未钳位**的最大拍间隔（秒）。均值查不出瞬时停顿 ——
#: 2026-09-04 桥接被 EtherCAT 判「电机长时间没有收到指令」而退出时，最后一个
#: 10s 窗口的均值是 228.7Hz（完全正常），塌陷若发生只能发生在均值抹平的尺度上。
TickStats = collections.namedtuple(
    'TickStats', 'count mean_dt rate_hz clamp_count clamp_ratio live max_dt')

#: :meth:`ChassisBridgeCore.consume_tick_window_gap` 的返回值。
#: ``max_dt`` 本窗最大未钳位拍间隔，``at`` 它发生的时刻（秒，单调钟），
#: ``over_count`` 本窗超过 2× 标称步长的拍数，``ticks`` 本窗拍数。
TickWindowGap = collections.namedtuple('TickWindowGap', 'max_dt at over_count ticks')

#: :meth:`ChassisBridgeCore.consume_vel_trace` 的返回值 —— 桥接**内部**速度链路。
#:
#: 为什么需要它：桥接外面那四段（raw→smooth→preCpl→cmd）已有诊断器在看，而
#: 进了桥接之后还有五段变换，其中**两段不体现在任何速度话题上**：
#:
#:   /cmd_vel ─[看门狗/scan 联锁 置零]→ in ─[clamp 模长限幅]→ clamped
#:            ─[slew 加速度限幅]→ slewed ─[世界→本体]→ local
#:            ─[×dt 积分]→ pos_cmd 增量 ─[+外环校正]→ 实发位置 ─[SDK]→ actual
#:
#: 「联锁把速度扔了」和「上游根本没发速度」在 /cmd_vel 上长得一样；外环校正是
#: 直接加在位置上的，它贡献的那部分位移在任何速度量里都查不到。这两件事只能在
#: 这里看见。
#:
#: 口径警告（对比时必须遵守）：``cmd_path`` 是**路径长**（Σ|v|·dt），
#: ``cmd_net``/``act_net`` 是**净位移**（窗口首末两点直线距离）。拿路径长和净
#: 位移相比是错的 —— 只有当 ``cmd_path ≈ cmd_net``（即这一窗基本走直线）时，
#: ``act_net`` 和 ``cmd_net`` 的比较才成立。actual 侧刻意**不**累加逐拍 |δ| 求
#: 路径长：250Hz 下每拍取绝对值会把编码器抖动整流成单向偏置（±0.1mm 的抖动就是
#: 0.025m/s 的假速度），那个数会稳定地大于指令值，看着像"底盘超速"。
VelTrace = collections.namedtuple(
    'VelTrace',
    'ticks wall in_peak clamped_peak slewed_peak local_peak '
    'clamp_bit slew_bit zeroed_ticks '
    'cmd_path cmd_net act_net dtheta_cmd dtheta_act corr_path')

#: 判"某一段有没有真的改动过速度"的死区。这些量都过了浮点乘除，不能用精确相等。
#: 取 1e-9：比任何真实速度小若干个数量级，又远大于双精度在 O(1) 量级上的舍入。
VEL_BIT_EPS = 1e-9


class StatusEvent:
    """一条待上报的状态。核心不直接发话题，只产出事件，由节点层转成消息。"""

    def __init__(self, code, detail='', metric_1=0.0, metric_2=0.0):
        self.code = code
        self.detail = detail
        self.metric_1 = float(metric_1)
        self.metric_2 = float(metric_2)

    def __repr__(self):
        return 'StatusEvent(%s, %r, %.4f, %.4f)' % (
            self.code, self.detail, self.metric_1, self.metric_2)


class ChassisBridgeConfig:
    """底盘桥接配置。构造时即做合法性校验 —— 非法参数不允许被带进运行期。"""

    def __init__(self, part_name='astribot_chassis', freq=250.0,
                 input_frame='body', theta_reference='at_enable',
                 cmd_vel_timeout_sec=0.3,
                 max_vel_xy=1.0, max_vel_theta=2.0,
                 max_accel_xy=2.5, max_accel_theta=3.2,
                 max_accel_xy_up=None,
                 leash_xy_m=0.25, leash_theta_rad=0.35,
                 require_manual_reset=True,
                 enable_slam_correction=True, pose_source='slam',
                 map_frame='map', base_frame='astribot_torso_base',
                 outer_rate=10.0, slam_max_age_sec=0.5,
                 slam_jump_threshold_m=0.30,
                 kp_xy=0.35, kp_theta=0.40,
                 max_corr_vel_xy=0.10, max_corr_vel_theta=0.20,
                 require_slam_to_enable=False, slam_loss_grace_sec=2.0,
                 odom_drift_window_sec=2.0, odom_drift_warn_m=0.15,
                 max_tick_dt_sec=0.04,
                 require_fresh_scan=True, scan_max_age_sec=0.5,
                 scan_loss_grace_sec=2.0):
        self.part_name = part_name
        self.freq = float(freq)
        self.input_frame = input_frame
        self.theta_reference = theta_reference
        self.cmd_vel_timeout_sec = float(cmd_vel_timeout_sec)
        self.max_vel_xy = float(max_vel_xy)
        self.max_vel_theta = float(max_vel_theta)
        self.max_accel_xy = float(max_accel_xy)
        self.max_accel_theta = float(max_accel_theta)
        #: xy **加速**方向的加速度上限（m/s²）。减速仍走 max_accel_xy。
        #: None = 退化为对称限幅（旧行为），这是**默认值**。
        #:
        #: !!! 代码默认必须是 None，不能是那个实测值 !!!
        #: 0.39m/s² 是**实机**底盘测出来的数。把它设成构造函数默认值，等于让
        #: 每一个不读 yaml 的调用方（全部单元测试在内）都吃这个实机数字 ——
        #: 实测过：默认给 0.35 会让 11 条既有测试变红，其中多数测的是积分器
        #: 本身（"250 拍走 1 米"之类），它们红了不是发现了缺陷，是前提被这个
        #: 默认值悄悄改掉了。值在 config/chassis_bridge.yaml 里给
        #: （仿真与实机**共用同一份**：同一条代码路径两边都在跑，比各给一份
        #: 更不容易出现"两边前提不同"那类缺陷；代价是仿真加速也变慢，而那
        #: 反而更接近实机）。
        #:
        #: 为什么需要它、0.35 怎么来的，见
        #: :func:`chassis_integrator.slew_limit_velocity` 的完整推导。
        #: 一句话：底盘真实加速度实测 ~0.39m/s²，而桥接原来按 2.5 发指令，
        #: 开环位置链在加速段积下的欠账永不归还，v=0.5 时暂态欠账 0.272m
        #: **必然**撞开 leash_xy_m=0.25 —— 实机 2026-09-08 就是这么停的
        #: （实测 0.2509m 对 0.250m，吻合 0.4%）。0.35 = 实测 0.39 留余量。
        #: theta 刻意不做非对称：同一次日志按**带符号**累加，跳闸时 theta
        #: 误差只有 0.054rad（预算 0.350，占 15%），不是瓶颈；而压慢旋转
        #: 会挤压三段式 ALIGN_START 的 15s 对齐预算。
        self.max_accel_xy_up = (None if max_accel_xy_up is None
                                else float(max_accel_xy_up))
        # 内环步长上限。默认 0.04s = 标称 4ms 的 10 倍。
        #
        # 怎么定的：实测超时周期最大 14.9ms（≈3.7 倍），取 10 倍留足余量，
        # 正常抖动不会被钳（钳了就等于没修这个缺陷）。上界的物理含义是
        # **单拍最大位移** = max_vel_xy * max_tick_dt_sec = 1.0 * 0.04 = 0.04m，
        # 必须远小于 leash_xy_m=0.25 —— 否则一次调度停顿就能把 leash 撞开。
        self.max_tick_dt_sec = float(max_tick_dt_sec)
        self.leash_xy_m = float(leash_xy_m)
        self.leash_theta_rad = float(leash_theta_rad)
        self.require_manual_reset = bool(require_manual_reset)
        self.enable_slam_correction = bool(enable_slam_correction)
        self.pose_source = pose_source
        # 节点层建 TfPosePort 要用；核心自身不查 TF（由 PosePort 注入）
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.outer_rate = float(outer_rate)
        self.slam_max_age_sec = float(slam_max_age_sec)
        self.kp_xy = float(kp_xy)
        self.kp_theta = float(kp_theta)
        self.max_corr_vel_xy = float(max_corr_vel_xy)
        self.max_corr_vel_theta = float(max_corr_vel_theta)
        self.require_slam_to_enable = bool(require_slam_to_enable)
        self.slam_loss_grace_sec = float(slam_loss_grace_sec)
        self.odom_drift_window_sec = float(odom_drift_window_sec)

        # ═══════════════ /scan 时效性联锁 ═══════════════
        # 为什么必须加在这里，而不是靠 nav2：
        # nav2 的 obstacle_layer 设了 expected_update_rate 之后**只会告警**。
        # 实测 controller_server 二进制里没有任何检查 costmap currency 的字符串，
        # 陈旧时它照样继续发 /cmd_vel。所以"拿着陈旧障碍数据继续走"这件事，
        # 只有写通路自己能拒绝 —— 这一层是链路上最后一个能说不的地方。
        #
        # 为什么现有的两个机制都盖不住这个场景（都实测过）：
        #   · leash：指令与实测都在动、偏差正常，**不会** trip
        #   · cmd_vel 看门狗：nav2 一直在发指令，`_last_twist_time` 不为 None，
        #     走不到"无输入置零"那条分支
        # 也就是说：上游感知已经瞎了，而这两道保护看到的一切都正常。
        #
        # scan_max_age_sec=0.5 的依据（不是拍的）：
        #   · 健康态实测 /scan 是 9.88~10.04Hz，周期 ~0.1s
        #   · 上游 pointcloud_slice_scan_node 的 hold_last_max_frames=5，
        #     也就是它**保证**最多连续重发 5 帧(=0.5s)后就停止输出
        #   · 两者取同一个值不是巧合：0.5s 正是上游自己放弃的时刻，
        #     也就是"最长可能隐身时间"。设得比它小会在上游正常保持时误触发，
        #     设得比它大则这段时间内谁都不管
        # scan_loss_grace_sec=2.0 与 slam_loss_grace_sec 对齐：短暂陈旧只置零
        # （可自动恢复），持续陈旧才闩锁停车（要人介入）。
        self.require_fresh_scan = bool(require_fresh_scan)
        self.scan_max_age_sec = float(scan_max_age_sec)
        self.scan_loss_grace_sec = float(scan_loss_grace_sec)
        if self.require_fresh_scan and self.scan_max_age_sec <= 0.0:
            raise ChassisConfigError(
                'scan_max_age_sec=%r 必须为正。要关闭这道联锁请显式设 '
                'require_fresh_scan=False —— 用非正阈值"顺便"关掉它，'
                '会让配置读起来像开着。' % (self.scan_max_age_sec,))
        if self.require_fresh_scan and self.scan_loss_grace_sec < self.scan_max_age_sec:
            raise ChassisConfigError(
                'scan_loss_grace_sec=%r 小于 scan_max_age_sec=%r：宽限期比判陈旧的'
                '阈值还短，等于跳过"置零"直接闩锁，短暂抖动就要人工复位。'
                % (self.scan_loss_grace_sec, self.scan_max_age_sec))

        if self.cmd_vel_timeout_sec <= 0.0:
            raise ChassisConfigError(
                'cmd_vel_timeout_sec=%r 必须为正：看门狗是 cmd_vel 断流时的'
                '唯一止损，不允许关闭。' % (self.cmd_vel_timeout_sec,))
        # 步长上限的两条硬约束，任一不满足都直接拒绝启动：
        #   ① 不能小于标称步长 —— 否则连正常拍都被钳，积分恒等于钳位值；
        #   ② 单拍最大位移必须远小于 leash —— 否则一次停顿就能把 leash 撞开，
        #      而 leash 是这条开环位置链路上唯一的硬保护。
        nominal_dt = 1.0 / self.freq if self.freq > 0.0 else float('inf')
        if self.max_tick_dt_sec < nominal_dt:
            raise ChassisConfigError(
                'max_tick_dt_sec=%r 小于标称步长 1/freq=%r：那样每一拍都会被钳位，'
                '积分恒等于钳位值，等于没修"按标称频率积分"这个缺陷。'
                % (self.max_tick_dt_sec, nominal_dt))
        max_tick_disp = self.max_vel_xy * self.max_tick_dt_sec
        if max_tick_disp >= self.leash_xy_m:
            raise ChassisConfigError(
                'max_vel_xy=%r × max_tick_dt_sec=%r = %.4fm，已达到 leash_xy_m=%r。'
                '单拍位移必须远小于 leash，否则一次调度停顿就能把 leash 撞开 —— '
                'leash 是开环位置链路上唯一的硬保护。请减小 max_tick_dt_sec。'
                % (self.max_vel_xy, self.max_tick_dt_sec, max_tick_disp,
                   self.leash_xy_m))
        # 非对称加速度限幅的自洽性。给成 > max_accel_xy 就不是"更紧的加速限"
        # 而是悄悄放宽了加速，与这个参数存在的理由正相反 —— 必须拒绝启动，
        # 不能留一个"读起来像限了速、实际放宽了"的配置。
        if (self.max_accel_xy_up is not None
                and self.max_accel_xy_up > self.max_accel_xy):
            raise ChassisConfigError(
                'max_accel_xy_up=%r 大于 max_accel_xy=%r：这个参数的用途是把'
                '**加速**方向限得比减速更紧（底盘真实加速度只有 ~0.39m/s²，'
                '开环位置链在加速段积下的欠账永不归还）。给成更大的值等于'
                '悄悄放宽加速，与它存在的理由正相反。'
                % (self.max_accel_xy_up, self.max_accel_xy))
        validate_leash_config(self.leash_xy_m, self.leash_theta_rad)
        validate_correction_config(self.kp_xy, self.kp_theta,
                                   self.max_corr_vel_xy, self.max_corr_vel_theta,
                                   self.outer_rate, self.freq)
        # 按位姿源自动失效没有意义的阈值（见 chassis_feedback.effective_thresholds）
        self.slam_jump_threshold_m, self.odom_drift_warn_m = effective_thresholds(
            self.pose_source, float(slam_jump_threshold_m), float(odom_drift_warn_m))
        # 提前算一次，避免每帧重复校验 input_frame
        to_local_velocity((0.0, 0.0, 0.0), self.input_frame, 0.0)


class ChassisBridgeCore:
    """底盘控制核心。节点层只负责：喂 twist、按频率调 tick、把事件转成话题。"""

    def __init__(self, cfg, session, pose_port, clock):
        self.cfg = cfg
        self.session = session
        self.pose = pose_port
        self.clock = clock

        self.state = ST_DISABLED
        #: 最近一次"从使能态掉出去"的原因，供周期日志把状态说清楚。
        #: !!! 为什么必须有这个字段 !!!（2026-09-08 实机排查代价换来的）
        #: 节点层的周期日志判据是 `TickStats.live`，而 live 的定义是
        #: `state == ST_ENABLED` —— 于是 DISABLED / LEASH_TRIPPED /
        #: STOPPED_NO_POSE / STOPPED_STALE_SCAN 四种状态打出**逐字相同**的
        #: "内环已停用"。真正的原因和 err_xy/err_theta 只进
        #: /astribot/bridge/status，而实机跑车时没人在录那个话题，
        #: 于是"机器人为什么不动了"在日志里无从分辨（实机那次是
        #: leash 跳闸，xy 与 theta 两条都逼近阈值，日志无法区分是哪条）。
        #: 形如 (state, reason, metric_1, metric_2, stamp)；从未停过则为 None。
        self.last_stop = None
        self.pos_cmd = None
        self.theta_ref = 0.0
        self.corr_per_tick = (0.0, 0.0, 0.0)
        self.corr_frozen = True

        self._last_twist = (0.0, 0.0, 0.0)
        self._last_twist_time = None
        self._prev_vel_out = (0.0, 0.0, 0.0)
        #: 最近一次收到 /scan 的时刻。None = 从未收到过。
        #: **刻意不在 enable() 里清空** —— /scan 是外部持续流，与使能周期无关。
        #: 清了会导致每次 enable 后头 0.5s 都被判成"从未收到"而拒绝下发。
        self._last_scan_time = None
        #: /scan 连续陈旧的起始时刻，用于判宽限期。恢复新鲜时归零。
        self._scan_stale_since = None

        # ---- 内环步长与拍率统计 ----
        # `_tick_count` / `_tick_dt_sum` 是**无偏**计数：每一拍都计，不像
        # LOOP_OVERRUN 只在超阈时才报。LOOP_OVERRUN 的周期均值是截尾样本的均值，
        # 拿它反推拍率会算出 91Hz，而时间账反解的下界是 ≥157Hz —— 差 1.7 倍。
        # 真实拍率至今没有无偏测量值，这两个字段就是为了补上它。
        self._prev_tick_time = None
        self._tick_first_time = None
        self._tick_count = 0
        self._tick_dt_sum = 0.0
        self._tick_clamp_count = 0
        # 均值查不出**瞬时**停顿：2026-09-04 桥接被 EtherCAT 判「电机长时间没有
        # 收到指令」而退出时，最后一个 10s 窗口的均值是 228.7Hz —— 完全正常。
        # 所以另记最大拍间隔：`_tick_max_raw` 跟整段（进 TickStats），
        # `_tick_win_*` 每次上报后由 consume_tick_window_gap 取走并清零，
        # 于是每条日志印的是**本窗**极值，不会被历史极值盖住。
        self._tick_max_raw = 0.0
        self._tick_win_max_raw = 0.0
        self._tick_win_max_at = None
        self._tick_win_over_count = 0
        self._tick_win_ticks = 0

        # ---- 桥接内部速度链路的本窗统计（见 VelTrace 的口径说明）----
        # 全部按窗累积、由 consume_vel_trace 取走并清零。绝不逐拍打日志：
        # 内环 250Hz，逐拍打等于把日志刷死，诊断器本身会变成被诊断系统的负担。
        self._vel_win = None
        self._reset_vel_window()

        self._p_des_map = None
        self._p_slam_prev = None
        self._pose_lost_since = None
        # 外环用：累积"上一次外环以来的本体位移"，供 advance_desired_pose 推进
        self._body_disp_accum = [0.0, 0.0]
        self._dtheta_accum = 0.0
        # 漂移诊断窗口：(时刻, sdk_xy, slam_xy)
        self._drift_window = []

        self.events = []

    # ---------------- 事件 ----------------

    def _emit(self, code, detail='', m1=0.0, m2=0.0):
        self.events.append(StatusEvent(code, detail, m1, m2))

    def _note_stop(self, state, reason, m1=0.0, m2=0.0):
        """记下"因何掉出使能态"，见 ``self.last_stop`` 的说明。

        刻意**不**复用 ``_emit``：事件会被 ``drain_events()`` 取空，
        而周期日志要的是一个**可反复读**的最近原因。同一个事实需要两种
        生命周期的载体，合并成一个就等于周期日志永远读到 None。
        """
        self.last_stop = (state, reason, m1, m2, self.clock.now())

    def drain_events(self):
        out = self.events
        self.events = []
        return out

    # ---------------- 使能/停用 ----------------

    def enable(self):
        """使能。返回 (ok, detail)。

        每次使能都**重取积分种子**，绝不复用旧 pos_cmd（202:44 / 203:43）。
        """
        try:
            seed = self.session.get_desired_joints_position([self.cfg.part_name])[0]
        except Exception as exc:      # noqa: BLE001 —— 禁止吞异常，转状态位
            self._emit(S_SDK_CALL_FAILED, '使能时取积分种子失败：%s' % exc)
            return (False, 'get_desired_joints_position 失败：%s' % exc)

        if len(seed) != 3:
            self._emit(S_SDK_CALL_FAILED,
                       '底盘自由度=%d，期望 3。ROBOT_TYPE 未设为 S1 时是 2'
                       '（astribot_base.py:36-38）' % len(seed))
            return (False, '底盘自由度不是 3')

        pose, stamp = self._lookup_pose_checked()
        if pose is None and self.cfg.require_slam_to_enable:
            self._emit(S_SLAM_UNAVAILABLE_OPEN_LOOP,
                       'require_slam_to_enable=true 且位姿源不可用，拒绝使能')
            return (False, '位姿源不可用且 require_slam_to_enable=true')

        self.pos_cmd = [seed[0], seed[1], wrap_angle(seed[2])]
        self.theta_ref = (self.pos_cmd[IDX_THETA]
                          if self.cfg.theta_reference == 'at_enable' else 0.0)
        self._prev_vel_out = (0.0, 0.0, 0.0)
        self._last_twist = (0.0, 0.0, 0.0)
        self._last_twist_time = None
        # !!! 必须清掉上一次使能留下的时间戳 !!!
        # 不清的话，disable 停了一段时间再 enable，第一拍算出来的 dt 就是那整段
        # 停机时长；乘上速度就是一次巨大的位置阶跃。虽然会被 max_tick_dt_sec
        # 钳住，但那属于"靠钳位兜住逻辑错误"，不是设计意图。置 None 走首拍分支。
        self._prev_tick_time = None
        # 拍率统计窗口也必须一起重开 —— 2026-09-01 实机踩到的：
        # 原来只清 _prev_tick_time 而留着 _tick_first_time，于是
        # rate = count / (_prev_tick_time - _tick_first_time) 的**分母**把
        # disabled 的那段时长也算进去了。实测表现是 65 拍报成 165.4Hz
        # （真值 ~238Hz），停得越久报得越低，而且看不出异常。
        self._reset_tick_window()
        # 速度链路窗口同理必须重开：不清的话，act_net 会拿**停机前**的实际位置
        # 当窗口起点，而 cmd_net 拿的是刚重取的积分种子 —— 两个口径跨越了停机期，
        # 报出来就是一个凭空出现的"跟踪误差"。与上面 _tick_first_time 那个坑同型。
        self._reset_vel_window()
        self.corr_per_tick = (0.0, 0.0, 0.0)
        self._p_slam_prev = None
        self._pose_lost_since = None
        # /scan 陈旧计时也要重开，否则 disable 那段时长会被算进"已持续陈旧"，
        # 报出来的数字跨越了停机期，与上面 _tick_first_time 那个坑同型。
        # 清它是安全的：真正阻止运动的是"龄期超阈 → vel_in 置零"那一步，它每拍
        # 独立判定、与本计时器无关；这个计时器只决定**何时闩锁**。
        self._scan_stale_since = None
        self._body_disp_accum = [0.0, 0.0]
        self._dtheta_accum = 0.0
        self._drift_window = []

        if pose is not None and self.cfg.enable_slam_correction:
            self._p_des_map = list(pose)
            self.corr_frozen = False
        else:
            self._p_des_map = None
            self.corr_frozen = True
            if pose is None:
                self._emit(S_SLAM_UNAVAILABLE_OPEN_LOOP,
                           '位姿源不可用，退化为纯开环 + leash')

        if is_correction_degenerate(self.cfg.pose_source,
                                    self.cfg.enable_slam_correction):
            # 防止把 ground_truth 下的漂亮数字误当成"闭环有效性已验收"
            self._emit(S_CORRECTION_DEGENERATE,
                       'pose_source=ground_truth：map->odom 是恒等静态 TF，'
                       '外环误差恒≈0，闭环不产生实际校正。仅可用于验证代码路径。')

        self.state = ST_ENABLED
        return (True, 'enabled')

    def disable(self):
        self.state = ST_DISABLED
        self.corr_frozen = True
        self.corr_per_tick = (0.0, 0.0, 0.0)
        self._emit(S_NOT_ENABLED, '已停用')
        self._note_stop(ST_DISABLED, '外部调用 ~/disable 主动停用')
        return (True, 'disabled')

    def reset_leash(self):
        """leash 复位。只在 LEASH_TRIPPED 态有效，且会重取种子。"""
        if self.state != ST_LEASH_TRIPPED:
            return (False, '当前状态 %s 不是 LEASH_TRIPPED，无需复位' % self.state)
        return self.enable()

    # ---------------- 输入 ----------------

    def submit_twist(self, vx, vy, wz):
        self._last_twist = (float(vx), float(vy), float(wz))
        self._last_twist_time = self.clock.now()

    def submit_scan_seen(self, stamp=None):
        """上报"刚收到一帧 /scan"。节点层在 /scan 订阅回调里调用。

        :param stamp: 该帧的 header 时间戳（秒）。None 表示用当前时刻。

        !!! 为什么默认用**接收时刻**而不是 header.stamp !!!
        上游 pointcloud_slice_scan_node 的 hold_last 策略会把上一帧的几何
        **配上 now() 的新时间戳**重发。也就是说 header.stamp 在保持期间是"新"的，
        拿它判龄期会被骗过去 —— 这正是这道联锁要防的那件事。
        接收时刻至少能反映"话题还在动"，配合上游的 hold_last_max_frames=5
        上限（超过就真的停止输出），两者合起来才封住整个窗口：
            · 上游保持 <=0.5s：header 是新的、接收也在动 → 本联锁不触发（有意）
            · 上游超过上限停发：接收时刻不再更新 → 本联锁在 0.5s 后触发
        传 stamp 的用法留给"就是要按 header 判"的场景，但要清楚它可以被骗。
        """
        self._last_scan_time = self.clock.now() if stamp is None else float(stamp)

    def _scan_age(self):
        """返回 (龄期秒, 是否从未收到过)。"""
        if self._last_scan_time is None:
            return (float('inf'), True)
        return (self.clock.now() - self._last_scan_time, False)

    # ---------------- 位姿 ----------------

    def _lookup_pose_checked(self):
        """查位姿并做龄期检查。返回 (pose_or_None, stamp_or_None)。

        龄期检查在两种 pose_source 下都生效：虽然 ground_truth 的 map->odom 是
        静态 TF，但 odom->base 始终动态，合成变换的时间戳由动态那段决定。
        """
        try:
            pose, stamp = self.pose.lookup()
        except Exception as exc:      # noqa: BLE001
            # !!! 这里报 POSE_PORT_FAILED，不是 SDK_CALL_FAILED !!!
            # 位姿查询不是 SDK 调用；报成 SDK 故障会把诊断引向机器人/SDK，
            # 而真实故障在 TF/位姿源。也不折进 SLAM_UNAVAILABLE_OPEN_LOOP ——
            # lookup() 的契约是"查不到返回 (None, None) 而不抛"，抛了就是
            # 端口实现有缺陷，不能伪装成正常降级工况。
            self._emit(S_POSE_PORT_FAILED,
                       '位姿源实现抛异常（契约要求返回 (None, None) 而非抛）：%s' % exc)
            return (None, None)
        if pose is None:
            return (None, None)
        age = self.clock.now() - stamp
        if age > self.cfg.slam_max_age_sec:
            self._emit(S_SLAM_STALE,
                       '位姿龄期 %.3fs 超过阈值 %.3fs' % (age, self.cfg.slam_max_age_sec),
                       age, self.cfg.slam_max_age_sec)
            return (None, None)
        return (pose, stamp)

    # ---------------- 内环 ----------------

    def inner_tick(self):
        """内环一拍（按 cfg.freq 调用）。返回本拍是否真的下发了指令。"""
        if self.state in (ST_DISABLED, ST_LEASH_TRIPPED, ST_STOPPED_NO_POSE,
                          ST_STOPPED_STALE_SCAN):
            return False

        # ---- 步长用**实测**值，不用 1/freq ----
        # 底盘是位置接口：物理速度 = 每拍增量 × 墙钟拍率。按 1/freq 积分而实际
        # 拍率更低时，底盘只有指令速度的 (实际拍率/freq)。实测拍率下界 ≥157Hz，
        # 即最多只跑到指令的 63%。改用实测 dt 后拍率快慢不再影响速度。
        # 钳位是必需的：dt 直接乘在速度上，一次停顿就是一次位置阶跃。
        tick = measure_tick_dt(self.clock.now(), self._prev_tick_time,
                               1.0 / self.cfg.freq, self.cfg.max_tick_dt_sec)
        now_tick = self.clock.now()
        if self._tick_first_time is None:
            self._tick_first_time = now_tick
        self._prev_tick_time = now_tick
        dt = tick.dt
        self._tick_count += 1
        self._tick_dt_sum += dt
        # 最大间隔必须用**未钳位**的 tick.raw：dt 已被 max_tick_dt_sec 削平，
        # 拿它求最大值永远只能得到钳位上限本身，看不见真实停顿有多长。
        # 首拍 raw 是 None（没有参照），跳过 —— 不是 0 间隔。
        if tick.raw is not None and tick.raw > 0.0:
            self._tick_win_ticks += 1
            if tick.raw > self._tick_max_raw:
                self._tick_max_raw = tick.raw
            if tick.raw > self._tick_win_max_raw:
                self._tick_win_max_raw = tick.raw
                self._tick_win_max_at = now_tick
            if tick.raw > 2.0 / self.cfg.freq:
                self._tick_win_over_count += 1
        if tick.clamped:
            self._tick_clamp_count += 1
            self._emit(S_TICK_DT_CLAMPED, tick.reason,
                       tick.raw if tick.raw is not None else 0.0,
                       self.cfg.max_tick_dt_sec)

        # 看门狗：只把速度置零，不改状态（cmd_vel 短暂中断是正常工况）
        # raw_in 是**上游真给过的**最后一帧，vel_in 是被看门狗/联锁处理过之后的。
        # 两者都留着才分得清「上游没发」和「我们自己扔了」—— 在 /cmd_vel 上这两
        # 件事长得一模一样，而处置完全不同。
        raw_in = self._last_twist
        vel_in = self._last_twist
        if self._last_twist_time is None:
            vel_in = (0.0, 0.0, 0.0)
        else:
            idle = self.clock.now() - self._last_twist_time
            if idle > self.cfg.cmd_vel_timeout_sec:
                vel_in = (0.0, 0.0, 0.0)
                self._emit(S_CMD_VEL_TIMEOUT,
                           '/cmd_vel 已 %.3fs 无输入，速度置零' % idle,
                           idle, self.cfg.cmd_vel_timeout_sec)

        # ---- /scan 时效性联锁 ----
        # 放在看门狗**之后、限幅之前**：置零要能覆盖 cmd_vel 给的值，
        # 又要让后面的 slew_limit 把这个零按加速度限幅平滑收下去
        # （直接把 _prev_vel_out 打成 0 会变成速度阶跃）。
        if self.cfg.require_fresh_scan:
            age, never = self._scan_age()
            if age > self.cfg.scan_max_age_sec:
                vel_in = (0.0, 0.0, 0.0)
                if self._scan_stale_since is None:
                    self._scan_stale_since = self.clock.now()
                stale_for = self.clock.now() - self._scan_stale_since
                if never:
                    self._emit(S_SCAN_NEVER_RECEIVED,
                               '从未收到 /scan（已等 %.2fs），速度置零。'
                               '先查话题名与 QoS —— BEST_EFFORT 发布配 RELIABLE '
                               '订阅会一帧都收不到且只有一条 WARNING'
                               % stale_for,
                               stale_for, self.cfg.scan_max_age_sec)
                else:
                    self._emit(S_SCAN_STALE,
                               '/scan 龄期 %.3fs 超过 %.3fs，速度置零'
                               % (age, self.cfg.scan_max_age_sec),
                               age, self.cfg.scan_max_age_sec)
                # 持续陈旧超过宽限期 → 闩锁停车，要人介入。
                # 与 SLAM 丢失那条同构：短暂异常自动恢复，持续异常必须有人看到。
                if stale_for > self.cfg.scan_loss_grace_sec:
                    self.state = ST_STOPPED_STALE_SCAN
                    self._prev_vel_out = (0.0, 0.0, 0.0)
                    self._emit(S_SCAN_LOST_STOPPED,
                               '/scan 已持续陈旧 %.2fs 超过宽限 %.2fs，已停车。'
                               '感知失效期间继续行走等于拿旧障碍图开车'
                               % (stale_for, self.cfg.scan_loss_grace_sec),
                               stale_for, self.cfg.scan_loss_grace_sec)
                    self._note_stop(
                        ST_STOPPED_STALE_SCAN,
                        '/scan 已持续陈旧 %.2fs 超过宽限 %.2fs'
                        % (stale_for, self.cfg.scan_loss_grace_sec),
                        stale_for, self.cfg.scan_loss_grace_sec)
                    return False
            else:
                self._scan_stale_since = None

        vel = clamp_velocity(vel_in, self.cfg.max_vel_xy, self.cfg.max_vel_theta)
        vel_clamped = vel
        # !!! 行为变化 !!! 用实测 dt 后，加速度限幅按 a*dt 放行，每秒允许的
        # 速度变化回到配置的 max_accel；此前按 1/250 计算，实际只放行了
        # (实际拍率/250) 倍，也就是一直比配置**更保守**。这是有意修正，
        # 但它会让加速更快 —— 属运动行为变化，必须上机验。
        vel = slew_limit_velocity(vel, self._prev_vel_out,
                                  self.cfg.max_accel_xy, self.cfg.max_accel_theta, dt,
                                  self.cfg.max_accel_xy_up)
        self._prev_vel_out = vel

        v_local = to_local_velocity(vel, self.cfg.input_frame,
                                    self.pos_cmd[IDX_THETA] - self.theta_ref)
        # 窗口起点必须是**积分之前**的 pos_cmd。取积分之后的话，本拍的增量会被
        # 算进 cmd_path 却不算进 cmd_net，两个累加器之间就差了一拍 ——
        # 250 拍的窗口里表现为直度恒为 0.996 而非 1.000，看着像轨迹有点弯。
        pos_before = list(self.pos_cmd)
        self.pos_cmd = integrate_step_dt(self.pos_cmd, v_local, dt)

        # 供外环推进 p_des_map 用的本体位移累积
        self._body_disp_accum[0] += v_local[0] * dt
        self._body_disp_accum[1] += v_local[1] * dt
        self._dtheta_accum += v_local[2] * dt

        # 叠加外环校正（已被切成每拍小量）
        if not self.corr_frozen:
            self.pos_cmd = [self.pos_cmd[0] + self.corr_per_tick[0],
                            self.pos_cmd[1] + self.corr_per_tick[1],
                            wrap_angle(self.pos_cmd[2] + self.corr_per_tick[2])]

        # leash（快环，判据 = SDK 实际位置）
        try:
            actual = self.session.get_current_joints_position([self.cfg.part_name])[0]
        except Exception as exc:      # noqa: BLE001
            self._emit(S_SDK_CALL_FAILED, '读实际位置失败：%s' % exc)
            return False

        leash = check_leash(self.pos_cmd, actual,
                            self.cfg.leash_xy_m, self.cfg.leash_theta_rad)

        # 本窗速度链路取样。放在**这里**的两个理由：
        #   · actual 已经读到了（leash 本来就要读），不为诊断多打一次 SDK。
        #   · 放在 leash 判定**之前** —— leash 跳闸那一拍也要计入，那正是最需要
        #     知道"当时链路上各段是多少"的一拍。
        self._record_vel(raw_in, vel_in, vel_clamped, vel, v_local, dt,
                         actual, pos_before)

        if leash.tripped:
            # 冻结积分 **和** 校正 —— 只冻结积分等于 leash 没起作用
            self.state = ST_LEASH_TRIPPED
            self.corr_frozen = True
            self.corr_per_tick = (0.0, 0.0, 0.0)
            self.pos_cmd = leash_recover_command(actual)
            self._prev_vel_out = (0.0, 0.0, 0.0)
            self._emit(S_LEASH_TRIPPED, leash.reason, leash.err_xy, leash.err_theta)
            self._note_stop(ST_LEASH_TRIPPED, leash.reason,
                            leash.err_xy, leash.err_theta)
            return False

        try:
            self.session.set_joints_position(
                [self.cfg.part_name], [list(self.pos_cmd)])
        except Exception as exc:      # noqa: BLE001
            self._emit(S_SDK_CALL_FAILED, '下发位置失败：%s' % exc)
            return False
        return True

    def tick_stats(self):
        """内环拍率的**无偏**统计。

        ════════════════ 为什么必须是无偏的 ════════════════
        此前唯一能看到拍率的东西是 `LOOP_OVERRUN`，而它**只在周期 > 阈值时上报** ——
        是截尾样本。拿它的周期均值（9.26ms）反推拍率会得到 91Hz，而由超时**发生率**
        （11.4~18.0 次/秒）做时间账反解，下界是 **≥157Hz** —— 差 1.7 倍。
        据前者算出的"底盘只有指令速度的 36%"是错的。

        这里每一拍都计入，所以 `count / 墙钟时长` 就是真实平均拍率，无需推断。

        ════════════════ live 字段：为什么读数必须自带有效性 ════════════════
        `inner_tick` 在 `ST_DISABLED` 下提前返回，所以停用期间计数**不再增长**
        —— 这是对的。但那意味着停用后再读到的 count/rate 全是**上一段使能期间的
        陈旧值**。2026-09-01 我就被这个骗了两次：先据此断言"内环停了"，
        又据此断言"桥接仍是 enabled"，两次都错。

        这与 map_odom_tf 那个"陈旧 TF 被 lookup_transform(Time()) 当成最新返回"
        是**同一类缺陷**：一个不再更新的量，被下游当成当前值。
        所以这里把"窗口是否还在累积"做成返回值的一部分，调用方拿不到一个
        看起来正常、实则过期的数字。

        Returns:
            :class:`TickStats`，含 ``live``：True 表示统计窗口正在累积
            （核心处于使能态），False 表示读到的是陈旧值。
            未跑过任何一拍时全 0 且 ``live=False``。
        """
        live = self.state == ST_ENABLED
        if self._tick_count == 0 or self._tick_first_time is None:
            return TickStats(0, 0.0, 0.0, self._tick_clamp_count, 0.0, live, 0.0)
        elapsed = self._prev_tick_time - self._tick_first_time
        # 只有一拍时 elapsed=0，此时算不出速率，报 0 而不是除零。
        rate = (self._tick_count / elapsed) if elapsed > 0.0 else 0.0
        return TickStats(
            self._tick_count,
            self._tick_dt_sum / self._tick_count,
            rate,
            self._tick_clamp_count,
            self._tick_clamp_count / self._tick_count,
            live,
            self._tick_max_raw)

    def consume_tick_window_gap(self):
        """取走并清零**本窗**的拍间隔极值。

        为什么是 consume 而不是只读：这个量的意义就是"上一条日志到现在"，
        只读会让每条日志都印同一个历史最大值 —— 那正是均值查不出瞬时停顿的
        同一类错误（一个不再更新的量被当成当前值，见 :meth:`tick_stats` 的
        ``live`` 那段）。所以调用方每印一次就必须清一次，名字里写明是 consume。

        Returns:
            :class:`TickWindowGap`。本窗一拍都没跑时全 0 / ``at=None``。
        """
        gap = TickWindowGap(self._tick_win_max_raw, self._tick_win_max_at,
                            self._tick_win_over_count, self._tick_win_ticks)
        self._tick_win_max_raw = 0.0
        self._tick_win_max_at = None
        self._tick_win_over_count = 0
        self._tick_win_ticks = 0
        return gap

    def _reset_tick_window(self):
        """重开拍率统计窗口。``enable()`` 必须调 —— 否则速率分母会含停机时长。"""
        self._prev_tick_time = None
        self._tick_first_time = None
        self._tick_count = 0
        self._tick_dt_sum = 0.0
        self._tick_clamp_count = 0
        self._tick_max_raw = 0.0
        self._tick_win_max_raw = 0.0
        self._tick_win_max_at = None
        self._tick_win_over_count = 0
        self._tick_win_ticks = 0

    def reset_tick_stats(self):
        """重置拍率统计窗口。诊断时想看"当前"拍率而非整段使能的均值时用。"""
        self._reset_tick_window()

    # ---------------- 内部速度链路取样 ----------------

    def _reset_vel_window(self):
        """重开速度链路统计窗口。"""
        self._vel_win = {
            'ticks': 0, 't0': None, 't1': None,
            'in_peak': 0.0, 'clamped_peak': 0.0, 'slewed_peak': 0.0,
            'local_peak': 0.0,
            'clamp_bit': 0, 'slew_bit': 0, 'zeroed': 0,
            'cmd_path': 0.0, 'corr_path': 0.0,
            'cmd_xy0': None, 'cmd_th0': None, 'cmd_xy1': None, 'cmd_th1': None,
            'act_xy0': None, 'act_th0': None, 'act_xy1': None, 'act_th1': None,
        }

    def _record_vel(self, raw_in, vel_in, vel_clamped, vel_slewed,
                    v_local, dt, actual, pos_before):
        """记一拍速度链路。纯累加，不打日志（内环 250Hz）。"""
        w = self._vel_win
        now = self.clock.now()
        w['ticks'] += 1
        if w['t0'] is None:
            w['t0'] = now
        w['t1'] = now

        raw_n = math.hypot(raw_in[0], raw_in[1])
        in_n = math.hypot(vel_in[0], vel_in[1])
        cl_n = math.hypot(vel_clamped[0], vel_clamped[1])
        sl_n = math.hypot(vel_slewed[0], vel_slewed[1])
        lo_n = math.hypot(v_local[0], v_local[1])
        w['in_peak'] = max(w['in_peak'], in_n)
        w['clamped_peak'] = max(w['clamped_peak'], cl_n)
        w['slewed_peak'] = max(w['slewed_peak'], sl_n)
        w['local_peak'] = max(w['local_peak'], lo_n)

        # 「这一段有没有真的咬住」按**逐轴**比较，不按模长：clamp 是等比缩放
        # xy，模长会变；而 slew 可能只限住了 wz，模长一点没动 —— 只看模长会漏。
        # 阈值用 VEL_BIT_EPS 而不是精确相等：这些量都过了浮点乘除。
        if raw_n > VEL_BIT_EPS and in_n <= VEL_BIT_EPS:
            w['zeroed'] += 1
        if any(abs(a - b) > VEL_BIT_EPS
               for a, b in zip(vel_in, vel_clamped)):
            w['clamp_bit'] += 1
        if any(abs(a - b) > VEL_BIT_EPS
               for a, b in zip(vel_clamped, vel_slewed)):
            w['slew_bit'] += 1

        # 指令路径长（Σ|v|·dt）。与净位移是两个口径，见 VelTrace 文档。
        w['cmd_path'] += lo_n * dt
        # 外环校正贡献的位移：它直接加在位置上，任何速度量里都看不到它。
        if not self.corr_frozen:
            w['corr_path'] += math.hypot(self.corr_per_tick[0],
                                         self.corr_per_tick[1])

        if w['cmd_xy0'] is None:
            # pos_before = 本拍积分**之前**的指令位置，见 inner_tick 里那段注释。
            w['cmd_xy0'] = (pos_before[0], pos_before[1])
            w['cmd_th0'] = pos_before[IDX_THETA]
            w['act_xy0'] = (actual[0], actual[1])
            w['act_th0'] = actual[IDX_THETA]
        w['cmd_xy1'] = (self.pos_cmd[0], self.pos_cmd[1])
        w['cmd_th1'] = self.pos_cmd[IDX_THETA]
        w['act_xy1'] = (actual[0], actual[1])
        w['act_th1'] = actual[IDX_THETA]

    def consume_vel_trace(self):
        """取走并清零**本窗**的速度链路取样。

        和 :meth:`consume_tick_window_gap` 同一个理由必须是 consume：这些量的
        意义是"上一条日志到现在"。只读不清会让每条日志印同一段历史 —— 一个不再
        更新的读数被当成当前值，是本项目反复踩到的那一类错误。

        Returns:
            :class:`VelTrace`。本窗一拍都没跑时 ``ticks=0``，调用方应据此跳过打印
            （而不是打一行全 0 —— 全 0 会被读成"链路上确实全是零"）。
        """
        w = self._vel_win
        wall = 0.0
        if w['t0'] is not None and w['t1'] is not None:
            wall = max(0.0, w['t1'] - w['t0'])

        def _net(p0, p1):
            if p0 is None or p1 is None:
                return 0.0
            return math.hypot(p1[0] - p0[0], p1[1] - p0[1])

        def _dth(a0, a1):
            if a0 is None or a1 is None:
                return 0.0
            return wrap_angle(a1 - a0)

        trace = VelTrace(
            ticks=w['ticks'], wall=wall,
            in_peak=w['in_peak'], clamped_peak=w['clamped_peak'],
            slewed_peak=w['slewed_peak'], local_peak=w['local_peak'],
            clamp_bit=w['clamp_bit'], slew_bit=w['slew_bit'],
            zeroed_ticks=w['zeroed'],
            cmd_path=w['cmd_path'],
            cmd_net=_net(w['cmd_xy0'], w['cmd_xy1']),
            act_net=_net(w['act_xy0'], w['act_xy1']),
            dtheta_cmd=_dth(w['cmd_th0'], w['cmd_th1']),
            dtheta_act=_dth(w['act_th0'], w['act_th1']),
            corr_path=w['corr_path'])
        self._reset_vel_window()
        return trace

    # ---------------- 外环 ----------------

    def outer_tick(self):
        """外环一拍（按 cfg.outer_rate 调用）。"""
        if self.state != ST_ENABLED:
            return
        if not self.cfg.enable_slam_correction:
            return

        pose, stamp = self._lookup_pose_checked()
        if pose is None:
            self.corr_frozen = True
            self.corr_per_tick = (0.0, 0.0, 0.0)
            if self._pose_lost_since is None:
                self._pose_lost_since = self.clock.now()
            lost = self.clock.now() - self._pose_lost_since
            if (self.cfg.require_slam_to_enable
                    and lost > self.cfg.slam_loss_grace_sec):
                # 商业化配置：丢位姿超宽限期 → 主动停车，但不冻结 leash、不杀节点
                self.state = ST_STOPPED_NO_POSE
                self._prev_vel_out = (0.0, 0.0, 0.0)
                self._emit(S_SLAM_LOST_STOPPED,
                           '位姿源丢失 %.2fs 超过宽限 %.2fs，已停车'
                           % (lost, self.cfg.slam_loss_grace_sec),
                           lost, self.cfg.slam_loss_grace_sec)
                self._note_stop(
                    ST_STOPPED_NO_POSE,
                    '位姿源丢失 %.2fs 超过宽限 %.2fs'
                    % (lost, self.cfg.slam_loss_grace_sec),
                    lost, self.cfg.slam_loss_grace_sec)
            return

        self._pose_lost_since = None

        # 跳变检测：重定位事件必须重新对齐，不能当误差施加
        jumped, jump = detect_pose_jump(pose, self._p_slam_prev,
                                        self.cfg.slam_jump_threshold_m)
        self._p_slam_prev = list(pose)
        if jumped:
            self._p_des_map = list(pose)
            self.corr_per_tick = (0.0, 0.0, 0.0)
            self._reset_accum()
            self._emit(S_SLAM_RELOCALIZED,
                       '位姿跳变 %.4fm 超过阈值 %.4fm，已重新对齐期望位姿'
                       % (jump, self.cfg.slam_jump_threshold_m),
                       jump, self.cfg.slam_jump_threshold_m)
            return

        if self._p_des_map is None:
            # 之前一直没有位姿（开环），现在恢复了 -> 以当前位姿重新对齐
            self._p_des_map = list(pose)
            self._reset_accum()
            self.corr_frozen = False
            return

        # 用 SLAM 的绝对朝向把本体位移推进到 map 系
        self._p_des_map = advance_desired_pose(
            self._p_des_map, tuple(self._body_disp_accum),
            self._dtheta_accum, pose[IDX_THETA])
        self._reset_accum()

        corr = compute_correction(self._p_des_map, pose,
                                  self.cfg.kp_xy, self.cfg.kp_theta,
                                  self.cfg.max_corr_vel_xy,
                                  self.cfg.max_corr_vel_theta,
                                  self.cfg.outer_rate)
        self.corr_per_tick = slice_correction(corr, self.cfg.freq, self.cfg.outer_rate)
        self.corr_frozen = False

        self._update_drift(pose)

    def _reset_accum(self):
        self._body_disp_accum = [0.0, 0.0]
        self._dtheta_accum = 0.0

    def _update_drift(self, pose):
        """打滑诊断：窗口内两个位移源的模长差，frame 无关（见 odom_drift 说明）。"""
        try:
            sdk = self.session.get_current_joints_position([self.cfg.part_name])[0]
        except Exception as exc:      # noqa: BLE001
            self._emit(S_SDK_CALL_FAILED, '漂移诊断读实际位置失败：%s' % exc)
            return
        now = self.clock.now()
        self._drift_window.append((now, (sdk[0], sdk[1]), (pose[0], pose[1])))
        cutoff = now - self.cfg.odom_drift_window_sec
        while len(self._drift_window) > 1 and self._drift_window[0][0] < cutoff:
            self._drift_window.pop(0)
        if len(self._drift_window) < 2:
            return
        t0, sdk0, slam0 = self._drift_window[0]
        _, sdk1, slam1 = self._drift_window[-1]
        drift = odom_drift((sdk1[0] - sdk0[0], sdk1[1] - sdk0[1]),
                           (slam1[0] - slam0[0], slam1[1] - slam0[1]))
        if drift > self.cfg.odom_drift_warn_m:
            moved = ((slam1[0] - slam0[0]) ** 2 + (slam1[1] - slam0[1]) ** 2) ** 0.5
            self._emit(S_ODOM_DRIFT_HIGH,
                       '%.2fs 窗口内里程计与位姿源位移差 %.4fm 超过阈值 %.4fm'
                       % (now - t0, drift, self.cfg.odom_drift_warn_m),
                       drift, moved)
