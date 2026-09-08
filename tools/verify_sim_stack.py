#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分层验证仿真全链路：上一层不过就不查下一层。

被 tools/launch_sim_stack.sh 调用，也可以单独跑（栈已经在跑的时候）：
    python3 tools/verify_sim_stack.py --mode explore --tracker mppi

════════ 判据的选取原则（每一条都是踩过的坑换来的）════════

1) 不用 count_publishers 当"这条链通了"。实测过五条话题全 pub=1 而拍率 0Hz：
   BEST_EFFORT 发布者 + RELIABLE 订阅者，一帧都收不到，只有一条 WARNING。
   所以每条话题都按**窗口内实收帧数**判，并且订阅一律用 sensor_data
   (BEST_EFFORT/VOLATILE) —— 它对 RELIABLE 与 TRANSIENT_LOCAL 的发布者都兼容，
   反过来不兼容。

2) 不用进程数当"nav2 起来了"。实测过 9 个进程全活而 lifecycle_manager 报
   Aborting bringup、整套 nav2 从未 activate。所以逐个节点调 get_state。

3) 计时窗口一律用**墙钟**，不用仿真时间。理由是失败模式：/clock 冻住时
   仿真时间永不前进，用仿真时间做窗口的循环会永久挂住，表现为"验证脚本卡了"
   而不是"仿真冻了"。仿真时间是否前进单独作为一条判据（看 /clock 消息内容）。

4) 到位判据用**实测位姿**，不用 action 的状态码。状态码 4=SUCCEEDED 只说明
   nav2 认为自己完成了；本仓库实测过状态码 6=ABORTED 被误读、也实测过
   "要求半开而 actual=99.998"这类只比对指令的假验证。这里两个都报，
   但**通过与否只看实测距离**。

5) explore 档刻意**不**发自己的目标。navigate_to_pose 是单目标 action，
   外部再发一个会抢占协调器正在飞的目标：实测过客户端周期重发导致自激
   （66 个终止结果 vs 真正 abort 仅 2 次）。所以 explore 档改为观察
   协调器自己的派发 + 用 /odom 实测位移证明机器人真的动了。
"""
import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

BASE_FRAME = 'astribot_torso_base'   # 本机器人的根 frame。**没有** base_link
MAP_FRAME = 'map'

NAV2_LIFECYCLE_NODES = [
    'controller_server', 'smoother_server', 'planner_server',
    'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother',
]

CLOCK_QOS = QoSProfile(depth=10,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST)

# 报告行。verdict 只有 PASS / FAIL / INFO 三种；INFO 是"记录下来但不参与判定"。
ROWS = []


def row(layer, item, measured, verdict, note=''):
    ROWS.append((layer, item, str(measured), verdict, note))
    mark = {'PASS': '✅', 'FAIL': '🔴', 'INFO': '  '}[verdict]
    print('  %s %-34s %s%s' % (mark, item, measured, ('   ' + note) if note else ''),
          flush=True)
    return verdict != 'FAIL'


def spin_for(node, seconds):
    """墙钟窗口内持续 spin。不用仿真时间，理由见文件头第 3 条。"""
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)


def call_service(node, cli, req, timeout=10.0):
    """同步调服务。返回 None 表示服务不可达或超时。"""
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result() if fut.done() else None


# ══════════════════════════════════════════════════════════════════════════
# L1 · 时钟：仿真时间必须**在前进**
#
# 这一层是最省时的分水岭。use_sim_time=true 而 /clock 不推进时，所有节点的
# 时钟恒为 0：RCLCPP_*_THROTTLE 全被节流掉（节点活着但一条日志不打），而
# create_wall_timer 不受影响仍在跑、状态话题照发。这个组合极像 DDS/QoS 故障。
# ══════════════════════════════════════════════════════════════════════════
def layer_clock(node, window=6.0):
    got = {'n': 0, 'first': None, 'last': None}

    def on_clock(msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        got['n'] += 1
        if got['first'] is None:
            got['first'] = t
        got['last'] = t

    # ⚠️ 这里**刻意不调** destroy_subscription，见文件末尾那段注释：
    # rclpy 的拆除路径在这台机器上会挂死。实测 2026-09-08：第一版就是在这一行
    # 卡住的 —— verify.log 里只有一行"【L1 时钟】"表头，进程 1792s 只烧掉
    # 0.7s CPU，发现套接字 Recv-Q 堆到 213120 字节（连 DDS 线程都停了）。
    # 表象是"验证脚本卡在 L1"，极容易被读成"栈没起来"。
    # 订阅留着不销毁的代价只是多几个 KB 与几个空回调；os._exit() 会一起带走。
    node.create_subscription(Clock, '/clock', on_clock, CLOCK_QOS)
    spin_for(node, window)

    n = got['n']
    advanced = (got['last'] or 0.0) - (got['first'] or 0.0)
    ok = row('L1 时钟', '/clock 实收帧数（%.0fs 窗口）' % window,
             '%d 帧' % n, 'PASS' if n >= 10 else 'FAIL',
             '' if n >= 10 else '发布者=%d —— 有发布者而 0 帧就是加载期死锁的特征'
                               % node.count_publishers('/clock'))
    ok = row('L1 时钟', '仿真时间前进量', '%.2f s' % advanced,
             'PASS' if advanced > 0.1 else 'FAIL',
             '' if advanced > 0.1 else '收到了帧但时间不走 —— 物理没步进') and ok
    return ok, got['last']


# ══════════════════════════════════════════════════════════════════════════
# L2 · TF：map -> astribot_torso_base
#
# 根 frame 是 astribot_torso_base，不是 base_link（本机器人没有 base_link），
# 也不是任何传感器 frame。查不到这条链时 nav2 的表现是 planner 反复 abort。
#
# ⚠️ 这里刻意用 timeout=0 的轮询而不是给 lookup_transform 一个 timeout：
# 带 timeout 的查询需要有别的线程在 spin，从单线程/工作线程里调用时对
# **动态** TF 会失败而静态 TF 照常成功 —— 那个组合看起来像"TF 没问题"。
# ══════════════════════════════════════════════════════════════════════════
def layer_tf(node, buf, sim_now, deadline=20.0):
    t0 = time.time()
    tr = None
    while time.time() - t0 < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            tr = buf.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
            break
        except Exception:
            continue
    if tr is None:
        row('L2 TF', '%s -> %s' % (MAP_FRAME, BASE_FRAME), '查不到',
            'FAIL', '%.0fs 内一次都没查到 —— SLAM/定位没起来，或地图没加载' % deadline)
        return False, None

    stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
    age = (sim_now - stamp) if sim_now else float('nan')
    x = tr.transform.translation.x
    y = tr.transform.translation.y
    ok = row('L2 TF', '%s -> %s' % (MAP_FRAME, BASE_FRAME),
             'x=%.3f y=%.3f' % (x, y), 'PASS')
    # 龄期单独判：查到了但是**陈旧**的读数会被当成当前值用（本仓库一天内犯过三次）。
    if age == age:   # 非 NaN
        ok = row('L2 TF', 'TF 龄期（仿真时钟）', '%.2f s' % age,
                 'PASS' if age < 2.0 else 'FAIL',
                 '' if age < 2.0 else '查到的是陈旧变换，不能当当前位姿用') and ok
    return ok, (x, y)


# ══════════════════════════════════════════════════════════════════════════
# L3 · 生命周期：7 个 nav2 节点必须真的 active
# ══════════════════════════════════════════════════════════════════════════
LABELS = {0: 'unknown', 1: 'unconfigured', 2: 'inactive', 3: 'active', 4: 'finalized'}


def layer_lifecycle(node):
    ok = True
    for name in NAV2_LIFECYCLE_NODES:
        cli = node.create_client(GetState, '/%s/get_state' % name)
        res = call_service(node, cli, GetState.Request(), timeout=8.0)
        # 不 destroy_client：拆除路径挂死，见 layer_clock 里的说明。
        if res is None:
            ok = row('L3 生命周期', name, '服务不可达', 'FAIL',
                     '进程可能活着但从未 activate，或参数配置期就挂了') and ok
            continue
        label = res.current_state.label or LABELS.get(res.current_state.id, '?')
        ok = row('L3 生命周期', name, label,
                 'PASS' if label == 'active' else 'FAIL') and ok
    return ok


# ══════════════════════════════════════════════════════════════════════════
# L4 · 实测拍率：每条话题在窗口内真的出帧
#
# /map 只在 SLAM 有新扫描时才更新，拍率天生低（0.2~1Hz），所以它的门限是
# "窗口内 >= 1 帧"而不是一个 Hz 数。门限按话题分别给，不用一个统一的 Hz。
# ══════════════════════════════════════════════════════════════════════════
def layer_rates(node, scan_topic, window=15.0):
    need = [
        (scan_topic, LaserScan, 5, '感知切片输出；0 帧 = costmap 一个障碍都收不到'),
        ('/map', OccupancyGrid, 1, 'SLAM 输出；0 帧 = 没有地图，前沿/规划全不可用'),
        ('/global_costmap/costmap_raw', Costmap, 1,
         '原始代价 0~255（/global_costmap/costmap 那条是 OccupancyGrid，被压成 0~100，'
         '253 在那里显示成 99）'),
        ('/odom', Odometry, 10, '里程计；到位判据和位移判据都依赖它'),
    ]
    cnt = {t: 0 for t, _, _, _ in need}
    subs = []
    for topic, typ, _, _ in need:
        subs.append(node.create_subscription(
            typ, topic,
            (lambda k: (lambda _m: cnt.__setitem__(k, cnt[k] + 1)))(topic),
            qos_profile_sensor_data))
    spin_for(node, window)
    # 不 destroy_subscription：拆除路径挂死，见 layer_clock 里的说明。

    ok = True
    for topic, _, min_frames, why in need:
        n = cnt[topic]
        good = n >= min_frames
        ok = row('L4 拍率', topic, '%d 帧 / %.0fs (%.2f Hz)' % (n, window, n / window),
                 'PASS' if good else 'FAIL',
                 '' if good else '要求 >=%d 帧。%s（pub=%d，pub>0 并不代表出帧）'
                                 % (min_frames, why, node.count_publishers(topic))) and ok
    return ok


# ══════════════════════════════════════════════════════════════════════════
# L5 · 跟踪器参数：从**活着的** controller_server 回读
#
# 为什么必须回读而不是读 yaml：本仓库两个实测缺陷都只有回读能抓到 ——
#   · IncludeLaunchDescription 的 launch_arguments 是白名单，漏项不报错、
#     子 launch 用自己的默认值。实测过 max_linear_speed:=0.2 完全没生效、
#     限速扫描产出若干档位数字完全相同的表，而每张表看起来都正常。
#   · RewrittenYaml.substitute_params 只改**已存在**的键。vx_max 是 MPPI 的键，
#     RPP 那份 yaml 里根本没有它（RPP 叫 desired_linear_vel），
#     所以在 rpp 上只写 vx_max 是**静默 no-op**。
# 所以这里按 --tracker 选**那条路径真正在用的键**去比对。
# ══════════════════════════════════════════════════════════════════════════
def pv_to_py(pv):
    """ParameterValue -> python。自己写而不用 rclpy 的私有辅助，避免版本差异。"""
    t = pv.type
    return {1: pv.bool_value, 2: pv.integer_value, 3: pv.double_value,
            4: pv.string_value, 5: list(pv.byte_array_value),
            6: list(pv.bool_array_value), 7: list(pv.integer_array_value),
            8: list(pv.double_array_value), 9: list(pv.string_array_value),
            }.get(t, None)


def read_params(node, node_name, prefixes=None, depth=0):
    """列出并读回某节点的参数，返回 {名: 值}。服务不可达时返回 None。"""
    cli = node.create_client(ListParameters, '/%s/list_parameters' % node_name)
    req = ListParameters.Request()
    if prefixes:
        req.prefixes = prefixes
    req.depth = depth
    res = call_service(node, cli, req, timeout=10.0)
    # 不 destroy_client：拆除路径挂死，见 layer_clock 里的说明。
    if res is None:
        return None
    names = list(res.result.names)
    if not names:
        return {}
    out = {}
    # 分批取，别一次几百个。
    gcli = node.create_client(GetParameters, '/%s/get_parameters' % node_name)
    for i in range(0, len(names), 40):
        chunk = names[i:i + 40]
        greq = GetParameters.Request()
        greq.names = chunk
        gres = call_service(node, gcli, greq, timeout=10.0)
        if gres is None:
            continue
        for name, pv in zip(chunk, gres.values):
            out[name] = pv_to_py(pv)
    # 不 destroy_client：拆除路径挂死，见 layer_clock 里的说明。
    return out


# 屏幕上展示的跟踪器参数。全部参数都会写进 TSV，这里只挑三段式行为的关键项。
SHOW_KEYS = [
    'FollowPath.plugin', 'FollowPath.inner.plugin',
    'FollowPath.align_start_enabled', 'FollowPath.align_goal_enabled',
    'FollowPath.align_kp', 'FollowPath.align_max_vel', 'FollowPath.align_tolerance',
    'FollowPath.start_min_angle', 'FollowPath.align_timeout',
    'FollowPath.approach_enabled', 'FollowPath.approach_dist',
    'FollowPath.zero_vy_in_follow',
]


def layer_tracker_params(node, tracker, max_v):
    params = read_params(node, 'controller_server', prefixes=['FollowPath'])
    if params is None:
        row('L5 跟踪器', 'controller_server 参数服务', '不可达', 'FAIL',
            'nav2 可能没 activate（L3 应该已经报了）')
        return False, {}
    if not params:
        row('L5 跟踪器', 'FollowPath.* 参数', '一个都没有', 'FAIL',
            '控制器实例名对不上，或 yaml 根本没被加载')
        return False, {}

    ok = True
    # ---- 1) 内层控制器家族必须与 --tracker 一致。
    #      这一条抓的是"以为在跑 MPPI，其实 yaml 选的是另一份"。
    fam = (str(params.get('FollowPath.plugin', '')) + ' '
           + str(params.get('FollowPath.inner.plugin', ''))).lower()
    want = 'mppi' if tracker == 'mppi' else 'pure_pursuit'
    ok = row('L5 跟踪器', '控制器家族与 --tracker 一致',
             '%s（期望含 %s）' % (fam.strip() or '空', want),
             'PASS' if want in fam else 'FAIL') and ok

    # ---- 2) 限速键。**按路径选键**，选错键就是一条恒为真的假判据。
    if tracker == 'mppi':
        cap_keys = [('FollowPath.inner.vx_max', float(max_v)),
                    ('FollowPath.inner.vx_min', -abs(float(max_v)))]
    else:
        cap_keys = [('FollowPath.desired_linear_vel', float(max_v))]
    for key, want_val in cap_keys:
        got = params.get(key)
        if got is None:
            ok = row('L5 跟踪器', key, '这个键不存在', 'FAIL',
                     '--max-linear-speed 在这条控制器路径上没有落点 —— '
                     '限速是静默 no-op') and ok
            continue
        good = abs(float(got) - want_val) < 1e-6
        ok = row('L5 跟踪器', key, '%.4f（期望 %.4f）' % (float(got), want_val),
                 'PASS' if good else 'FAIL',
                 '' if good else '--max-linear-speed 没有传到这一层 —— '
                                 'launch_arguments 白名单漏项的典型表现') and ok

    # ---- 3) 其余关键项只回读展示，不覆盖（见 launch_sim_stack.sh 头部第 3 条）。
    for key in SHOW_KEYS:
        if key in params and not key.endswith('.plugin'):
            row('L5 跟踪器', key + '（只读）', params[key], 'INFO')

    # ---- 4) 到位容差：L6 的判据要用它，所以在这里回读，不在代码里写死。
    gc = read_params(node, 'controller_server', prefixes=['general_goal_checker'])
    tol = (gc or {}).get('general_goal_checker.xy_goal_tolerance')
    if tol is None:
        row('L5 跟踪器', 'general_goal_checker.xy_goal_tolerance', '读不到', 'INFO',
            'L6 将退回用 0.25m 判到位')
        tol = 0.25
    else:
        row('L5 跟踪器', 'general_goal_checker.xy_goal_tolerance（L6 判据）',
            '%.3f m' % float(tol), 'INFO')
    params['__xy_tol__'] = float(tol)
    return ok, params


# ══════════════════════════════════════════════════════════════════════════
# L6 · 真实导航
#
# 两档走**完全不同**的通路，这不是偷懒，是被 action 语义逼出来的：
#
# explore 档：**一个目标都不发**。navigate_to_pose 是单目标 action，外部再发
#   一个就会抢占协调器正在飞的那个目标，旧目标以"失败"回来 —— 而失败会触发
#   协调器重试、重试再被下一次抢占，自激。实测过 66 个终止结果里真正的 abort
#   只有 2 次，剩下全是自己造的。所以这里只**旁观**：数协调器自己派发了几个
#   互不相同的目标，并用 /odom 实测位移证明机器人真的动了。
#
# localize 档：发**一个**目标，绝不重发。发之前先用 /compute_path_to_pose
#   证明它可达 —— 判据与真跑共用同一个 planner、同一份 costmap。
#   （外部探针与真跑判据不一致时，会在缺的那一项上系统性假阳性，本仓库白扫过两轮。）
#   到位判据只看**实测位姿到目标的距离** vs 从 controller_server 回读的
#   xy_goal_tolerance；action 状态码只作 INFO 一起报。
# ══════════════════════════════════════════════════════════════════════════
def _pose_from_tf(buf):
    try:
        tr = buf.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
    except Exception:
        return None
    return (tr.transform.translation.x, tr.transform.translation.y)


def layer_nav_explore(node, window):
    """explore 档：旁观协调器的派发 + 实测位移。"""
    seen_goals = []
    states = {}
    complete = {'v': None}
    odom = {'last': None, 'dist': 0.0, 'n': 0}

    def on_goal(msg):
        p = (msg.pose.position.x, msg.pose.position.y)
        # 同一个目标会被重复发布（重试/重规划），按 5cm 去重才是"派发了几个不同的点"。
        for q in seen_goals:
            if math.hypot(p[0] - q[0], p[1] - q[1]) < 0.05:
                return
        seen_goals.append(p)
        print('     协调器派发第 %d 个目标: (%.2f, %.2f)' % (len(seen_goals), p[0], p[1]),
              flush=True)

    def on_state(msg):
        states[msg.data] = states.get(msg.data, 0) + 1

    def on_odom(msg):
        p = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        odom['n'] += 1
        if odom['last'] is not None:
            d = math.hypot(p[0] - odom['last'][0], p[1] - odom['last'][1])
            # 1cm 死区：里程计噪声逐帧累加会把"静止"积成几十厘米的假位移。
            if d > 0.01:
                odom['dist'] += d
        odom['last'] = p

    subs = [
        node.create_subscription(PoseStamped, '/exploration/current_goal',
                                 on_goal, qos_profile_sensor_data),
        node.create_subscription(String, '/exploration/state',
                                 on_state, qos_profile_sensor_data),
        node.create_subscription(Bool, '/exploration/complete',
                                 lambda m: complete.__setitem__('v', m.data),
                                 qos_profile_sensor_data),
        node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data),
    ]
    print('     旁观 %ds（不发任何目标，避免抢占协调器在飞的目标）...' % window, flush=True)
    spin_for(node, window)
    # 不 destroy_subscription：拆除路径挂死，见 layer_clock 里的说明。

    ok = row('L6 导航(explore)', '协调器派发的不同目标数',
             '%d 个' % len(seen_goals),
             'PASS' if len(seen_goals) >= 1 else 'FAIL',
             '' if seen_goals else '0 次派发。已知成因：目标格从可站退化成致命、'
                                   '前沿净空半径 >= 地图分辨率会否掉全部候选、'
                                   '或起点被自身点云判为致命')
    # 位移门限 0.30m：既明显大于里程计噪声，也大于"原地对齐"能造出的位移。
    ok = row('L6 导航(explore)', '实测行走里程（/odom，1cm 死区）',
             '%.3f m（%d 帧）' % (odom['dist'], odom['n']),
             'PASS' if odom['dist'] > 0.30 else 'FAIL',
             '' if odom['dist'] > 0.30 else '派发了目标但机器人没动 —— '
                                            '限速被压到 0、cmd_vel 被姿态止损归零、'
                                            '或控制器一直在原地对齐') and ok
    if states:
        row('L6 导航(explore)', '/exploration/state 直方图',
            ' '.join('%s×%d' % (k, v) for k, v in sorted(states.items())), 'INFO')
    else:
        ok = row('L6 导航(explore)', '/exploration/state', '窗口内 0 帧', 'FAIL',
                 '协调器进程可能没起来（exploration:=true 传到了吗）') and ok
    row('L6 导航(explore)', '/exploration/complete', complete['v'], 'INFO',
        '冷启动全未知地图上 true 是**假**完成 —— 空地图 0 前沿不等于探索完成')
    return ok


def _candidate_goals(cur, explicit):
    if explicit:
        return [explicit]
    # 由近到远、绕一圈。每个候选都要先过 planner 才会被发出去，所以多给几个不亏。
    out = []
    for dist in (1.5, 2.5, 1.0, 3.5):
        for deg in (0, 45, -45, 90, -90, 135, -135, 180):
            a = math.radians(deg)
            out.append((cur[0] + dist * math.cos(a), cur[1] + dist * math.sin(a)))
    return out


def _mk_pose(node, xy):
    p = PoseStamped()
    p.header.frame_id = MAP_FRAME
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x = float(xy[0])
    p.pose.position.y = float(xy[1])
    p.pose.orientation.w = 1.0
    return p


def layer_nav_localize(node, buf, explicit_goal, xy_tol, timeout):
    cur = _pose_from_tf(buf)
    if cur is None:
        row('L6 导航(localize)', '当前位姿', '查不到', 'FAIL', 'L2 应该已经报了')
        return False
    row('L6 导航(localize)', '起点（实测 TF）', '(%.2f, %.2f)' % cur, 'INFO')

    # ---- 1) 先用 planner 证明目标可达。与真跑同一个 planner、同一份 costmap。
    pc = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    if not pc.wait_for_server(timeout_sec=15.0):
        row('L6 导航(localize)', '/compute_path_to_pose', '服务器不可达', 'FAIL')
        return False

    goal_xy = None
    plan_len = 0.0
    tried = 0
    for cand in _candidate_goals(cur, explicit_goal):
        tried += 1
        g = ComputePathToPose.Goal()
        g.goal = _mk_pose(node, cand)
        g.use_start = False
        g.planner_id = 'GridBased'
        fut = pc.send_goal_async(g)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted:
            continue
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(node, rf, timeout_sec=20.0)
        if not rf.done():
            continue
        path = rf.result().result.path
        if len(path.poses) < 2:
            continue
        goal_xy = cand
        plan_len = sum(
            math.hypot(path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x,
                       path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y)
            for i in range(len(path.poses) - 1))
        break

    if goal_xy is None:
        row('L6 导航(localize)', 'planner 可达性证明',
            '试了 %d 个候选，一个都规划不出路径' % tried, 'FAIL',
            '这是 planner/costmap 的问题，不是跟踪器的。'
            + ('给定的 --goal 不可达' if explicit_goal else
               '起点可能就在致命区（自身点云、或地图与足迹不相容）'))
        return False
    row('L6 导航(localize)', 'planner 可达性证明',
        '目标 (%.2f, %.2f)  路径长 %.2f m（第 %d 个候选）'
        % (goal_xy[0], goal_xy[1], plan_len, tried), 'PASS')

    # ---- 2) 发**一个**目标。绝不重发（重发=抢占，旧目标以失败回来）。
    nc = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    if not nc.wait_for_server(timeout_sec=15.0):
        row('L6 导航(localize)', '/navigate_to_pose', '服务器不可达', 'FAIL')
        return False
    ng = NavigateToPose.Goal()
    ng.pose = _mk_pose(node, goal_xy)
    fut = nc.send_goal_async(ng)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    gh = fut.result() if fut.done() else None
    if gh is None or not gh.accepted:
        row('L6 导航(localize)', '目标被接受', '否', 'FAIL',
            'bt_navigator 拒了目标')
        return False

    rf = gh.get_result_async()
    t0 = time.time()
    best = float('inf')
    travel = 0.0
    last = cur
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        p = _pose_from_tf(buf)
        if p is not None:
            d = math.hypot(p[0] - last[0], p[1] - last[1])
            if d > 0.01:
                travel += d
                last = p
            best = min(best, math.hypot(p[0] - goal_xy[0], p[1] - goal_xy[1]))
        if rf.done():
            break

    elapsed = time.time() - t0
    status = rf.result().status if rf.done() else None
    label = {1: 'ACCEPTED', 2: 'EXECUTING', 4: 'SUCCEEDED', 5: 'CANCELED',
             6: 'ABORTED'}.get(status, '未返回(超时)')
    # 状态码只报不判：4=SUCCEEDED 只说明 nav2 认为自己完成了。
    row('L6 导航(localize)', 'action 状态码（只报不判）',
        '%s (%s)' % (label, status), 'INFO')
    row('L6 导航(localize)', '耗时 / 实测里程',
        '%.1f s / %.2f m（规划长度 %.2f m）' % (elapsed, travel, plan_len), 'INFO')

    final = _pose_from_tf(buf)
    if final is None:
        row('L6 导航(localize)', '终点位姿', '查不到', 'FAIL')
        return False
    dist = math.hypot(final[0] - goal_xy[0], final[1] - goal_xy[1])
    # 唯一的通过判据：**实测**距离 <= 目标检查器自己的容差。
    ok = row('L6 导航(localize)', '实测终点到目标距离',
             '%.3f m（容差 %.3f m）' % (dist, xy_tol),
             'PASS' if dist <= xy_tol else 'FAIL',
             '' if dist <= xy_tol
             else ('最近逼近到 %.3f m —— 到过附近又走开了' % best
                   if best <= xy_tol else '始终没进容差圈'))
    return ok


# ══════════════════════════════════════════════════════════════════════════
# 主流程：逐级往上，上一层不过就**不查**下一层
#
# 为什么要短路：把上游故障读成下游缺陷是本仓库最常见的错误来源。/clock 不推进时
# 去查跟踪器参数，会得到一堆"服务不可达"，据此写下的根因全在错的那一层。
# 退出码按层给（10+层号），调用方能直接知道断在哪一层。
# ══════════════════════════════════════════════════════════════════════════
SCAN_TOPIC = {'slice_scan': '/scan_from_cloud', 'laserscan': '/scan'}


def write_report(path):
    if not path:
        return
    try:
        with open(path, 'w') as f:
            f.write('层\t项\t实测\t判定\t备注\n')
            for r in ROWS:
                f.write('\t'.join(x.replace('\t', ' ').replace('\n', ' ') for x in r) + '\n')
        print('\n  逐层判据已写入 %s' % path, flush=True)
    except Exception as e:
        print('  ⚠️ 报告写不出去: %s' % e, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', required=True, choices=['explore', 'localize'])
    ap.add_argument('--tracker', default='mppi', choices=['mppi', 'rpp'])
    ap.add_argument('--max-linear-speed', type=float, default=1.0)
    ap.add_argument('--scan-source', default='slice_scan',
                    choices=['slice_scan', 'laserscan'])
    ap.add_argument('--goal', default='', help='localize 档的目标 "x,y"（map 系）')
    ap.add_argument('--skip-nav-goal', action='store_true')
    ap.add_argument('--nav-timeout', type=float, default=120.0)
    ap.add_argument('--report', default='')
    args = ap.parse_args()

    explicit_goal = None
    if args.goal:
        try:
            gx, gy = (float(v) for v in args.goal.split(','))
            explicit_goal = (gx, gy)
        except Exception:
            print('🔴 --goal 要写成 x,y 两个数，收到 %r' % args.goal)
            sys.exit(2)

    rclpy.init()
    # use_sim_time=true 是必须的：目标位姿的时间戳要用**仿真**时钟，
    # 用墙钟盖的戳在仿真时间轴上是遥远的未来，TF 变换会直接失败。
    node = Node('verify_sim_stack',
                parameter_overrides=[
                    Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    buf = Buffer()
    TransformListener(buf, node)

    rc = 0
    try:
        print('\n【L1 时钟】', flush=True)
        ok, sim_now = layer_clock(node)
        if not ok:
            rc = 11
            raise SystemExit

        print('\n【L2 TF】', flush=True)
        ok, _cur = layer_tf(node, buf, sim_now)
        if not ok:
            rc = 12
            raise SystemExit

        print('\n【L3 生命周期】', flush=True)
        if not layer_lifecycle(node):
            rc = 13
            raise SystemExit

        print('\n【L4 实测拍率】', flush=True)
        rates_ok = layer_rates(node, SCAN_TOPIC[args.scan_source])

        print('\n【L5 跟踪器参数（回读，不覆盖）】', flush=True)
        params_ok, params = layer_tracker_params(node, args.tracker, args.max_linear_speed)
        xy_tol = params.get('__xy_tol__', 0.25)

        # L4/L5 不过时**仍然**跑 L6：这两层的失败（少一条话题、限速没传到）
        # 与"机器人能不能走到"是两个独立的事实，而后者是用户明确要的那一项。
        # 但退出码按最靠下的失败层给。
        if args.skip_nav_goal:
            row('L6 导航', '真实导航验证', '按 --skip-nav-goal 跳过', 'INFO')
            nav_ok = True
        else:
            print('\n【L6 真实导航】', flush=True)
            if args.mode == 'explore':
                nav_ok = layer_nav_explore(node, int(args.nav_timeout))
            else:
                nav_ok = layer_nav_localize(node, buf, explicit_goal, xy_tol,
                                            args.nav_timeout)

        if not rates_ok:
            rc = 14
        elif not params_ok:
            rc = 15
        elif not nav_ok:
            rc = 16
    except SystemExit:
        pass
    except Exception as e:
        row('异常', type(e).__name__, str(e)[:200], 'FAIL')
        rc = 20

    n_pass = sum(1 for r in ROWS if r[3] == 'PASS')
    n_fail = sum(1 for r in ROWS if r[3] == 'FAIL')
    print('\n════════════════════════════════════════════════════════════')
    print(' 判据 %d 条通过 / %d 条失败（INFO 行不参与判定）' % (n_pass, n_fail))
    if n_fail:
        print(' 失败的是：')
        for r in ROWS:
            if r[3] == 'FAIL':
                print('   🔴 [%s] %s = %s   %s' % (r[0], r[1], r[2], r[4]))
    print('════════════════════════════════════════════════════════════')
    write_report(args.report)

    # 绝不走 destroy_node()/rclpy.shutdown()：实测它会挂死（一次 78 分钟只烧掉
    # 1s CPU）。判据已经全部打印并落盘，此处直接退进程，不给关闭路径任何机会。
    sys.stdout.flush()
    os._exit(rc)


if __name__ == '__main__':
    main()
