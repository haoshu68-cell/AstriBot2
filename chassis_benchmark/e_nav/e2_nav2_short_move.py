#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E2-short · Nav2 走 30 cm 短距离位移测试。

================================================================================
先算清楚：30 cm 在 Nav2 这条链上测得到什么、测不到什么
================================================================================
生效的到位判据（nav2_params_{rpp,mppi}.yaml，两份一致）::

    goal_checker_plugins: ["general_goal_checker"]      # 只列一个
    general_goal_checker.xy_goal_tolerance:  0.18
    general_goal_checker.yaw_goal_tolerance: 3.15       # ≈±180°，等于不约束朝向

`SimpleGoalChecker` 每周期查「当前位姿到目标距离 < 0.18」。于是：

    单轴 0.30 m  -> 实际只需走 0.30 - 0.18 = 0.12 m 就判到达（60% 被容差吃掉）
    斜向 (0.30, 0.30) = 0.4243 m -> 需走 0.2443 m（42% 被容差吃掉）

**容差不能为这个测试调小。** ``xy_goal_tolerance: 0.10`` 已被实测否决并写进 yaml：
「派发 10 次、ARRIVED **0** 次、恢复行为 30 次 —— Nav2 从不报成功，
因为滑行量(0.069~0.100 m)与容差同量级」。0.18 = 实测最大滑行 0.100 + 0.08 余量。
所以 0.18 是这台底盘的**容差地板**，不是可调参数。

结论：**30 cm 测不出"路径跟踪精度"，但能测出两件更有价值的事。**

--------------------------------------------------------------------------------
能测到的第一件：到点精度（这才是 30 cm 该测的量）
--------------------------------------------------------------------------------
主指标不是"走了多少"，而是**停下来时离目标多远**。判据就是 0.18。

--------------------------------------------------------------------------------
能测到的第二件：RPP 与 MPPI 的结构性差异，30 cm 就够看
--------------------------------------------------------------------------------
RPP 的参数在 30 cm 尺度上全部退化，而且是可预测的：

* ``lookahead_dist: 0.6`` **比整条路径还长**（0.30 / 0.4243 m）。
  前视圆与路径不相交时 RPP 取路径末点当 carrot —— 全程只有一个"直奔终点"，
  **根本没有路径跟踪发生**。
* ``approach_velocity_scaling_dist: 0.6`` 同样 >= 全程长度 ->
  RPP 从第一步就在做接近减速，一路压向 ``min_approach_linear_velocity: 0.05``。
* ``use_rotate_to_heading: true`` + ``rotate_to_heading_min_angle: 0.785``(45°)：
  斜向目标 (0.30, 0.30) 的方向角正好是 45° ≈ 0.785 ->
  **RPP 会先原地转约 45°，再直线开过去**。

MPPI 是 ``motion_model: "Omni"``，会直接斜向平移、不先转向。

于是本测试的核心产出是一个**行为判别器**：

    起步阶段的原地转角  +  全程 |vy| 峰值

RPP 应该是「转角大、|vy| ≈ 0」，MPPI 应该是「转角小、|vy| 明显非零」。
这两个数在 30 cm 上就能分开，不需要长距离场地。

================================================================================
用法
================================================================================
    # 先起完整 Nav2（要 controller_server / bt_navigator / 全局代价地图）
    python3 e_nav/e2_nav2_short_move.py --env sim --n 5                  # 斜向
    python3 e_nav/e2_nav2_short_move.py --env sim --n 5 --mode sequential  # 先 x 再 y
    python3 e_nav/e2_nav2_short_move.py --env sim --n 5 --dx 0.3 --dy 0.0
"""

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bench import stats as st                                     # noqa: E402
from bench.session import (Session, base_arg_parser, results_dir,  # noqa: E402
                           run_main)
from bench.truth import (Pose2D, SimOdomTruth, displacement,       # noqa: E402
                         unwrap_delta, wrap_angle, yaw_from_quaternion)

TEST_ID = 'e2_nav2_short_move'

ROBOT_BASE_FRAME = 'astribot_torso_base'    # 不是 base_link
GLOBAL_FRAME_CANDIDATES = ('map', 'odom')

# 生效判据，逐字来自 yaml。改 yaml 必须同步改这里。
XY_GOAL_TOLERANCE = 0.18
YAW_GOAL_TOLERANCE = 3.15

# 判别器阈值：起步原地转角。RPP 的 rotate_to_heading_min_angle 是 0.785 rad。
# 取它的一半做分界：> 0.39 rad 判"先转向"，是差速式行为。
ROTATE_FIRST_RAD = 0.39
# "开始移动"的判据：净位移超过 2 cm。小于它的位移可能只是原地转时的几何漂移。
MOVE_ONSET_M = 0.02
# MPPI 应该用得上横移。|vy| 峰值超过这个值算"确实在用全向能力"。
VY_USED_MPS = 0.05

GOAL_TIMEOUT_SEC = 60.0


def lookup_pose(node, tf_buffer, target_frame, source_frame, timeout_sec=5.0):
    """查 TF 得到 source 在 target 里的 2D 位姿。

    **刻意不用 lookup_transform 的阻塞 timeout**：带 timeout 的查询在非主
    spin 线程上对**动态** TF 会失败（静态 TF 反而正常，所以看着像没问题）。
    这里改成主线程轮询 can_transform + spin_once，行为可预期。
    """
    end = time.monotonic() + timeout_sec
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        if tf_buffer.can_transform(target_frame, source_frame,
                                   rclpy.time.Time(), Duration(seconds=0.0)):
            t = tf_buffer.lookup_transform(target_frame, source_frame,
                                           rclpy.time.Time())
            tr = t.transform.translation
            return Pose2D(tr.x, tr.y, yaw_from_quaternion(t.transform.rotation),
                          time.monotonic())
    raise TimeoutError(
        f'{timeout_sec}s 内查不到 {target_frame} -> {source_frame}。'
        f'确认 SLAM/定位在跑（map->odom 由 slam_toolbox 发布），'
        f'且根 frame 是 {ROBOT_BASE_FRAME} 而不是 base_link。')


def pick_global_frame(node, tf_buffer):
    for f in GLOBAL_FRAME_CANDIDATES:
        try:
            lookup_pose(node, tf_buffer, f, ROBOT_BASE_FRAME, timeout_sec=3.0)
            return f
        except TimeoutError:
            continue
    raise TimeoutError(
        f'{GLOBAL_FRAME_CANDIDATES} 都查不到 -> {ROBOT_BASE_FRAME}。Nav2 没起或 TF 断了。')


def body_offset_goal(current, dx_body, dy_body, global_frame):
    """当前位姿 + 本体系偏移 -> 全局系绝对目标。

    「x/y 方向各走 30 cm」是本体系语义，所以偏移要按当前 yaw 旋进全局系。
    朝向保持不变（yaw_goal_tolerance 3.15 等于不约束，这里只是别乱给）。
    """
    c, s = math.cos(current.yaw), math.sin(current.yaw)
    gx = current.x + c * dx_body - s * dy_body
    gy = current.y + s * dx_body + c * dy_body
    p = PoseStamped()
    p.header.frame_id = global_frame
    p.pose.position.x = gx
    p.pose.position.y = gy
    p.pose.orientation.z = math.sin(current.yaw / 2.0)
    p.pose.orientation.w = math.cos(current.yaw / 2.0)
    return p, (gx, gy)


def read_loaded_controller(node):
    """读 controller_server 实际加载的插件名，让结果自描述是哪个控制器跑的。"""
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, '/controller_server/get_parameters')
    if not cli.wait_for_service(timeout_sec=3.0):
        return None
    req = GetParameters.Request()
    req.names = ['controller_plugins']
    fut = cli.call_async(req)
    end = time.monotonic() + 3.0
    while not fut.done() and time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
    if not fut.done() or not fut.result().values:
        return None
    return list(fut.result().values[0].string_array_value)


class CmdVelWatcher:
    """记录控制器输出，用来判断它是否真的用了横移能力。

    订阅的是 Nav2 最终输出那一级（cmd_vel_nav_body），不是 /cmd_vel ——
    /cmd_vel 上还叠了臂-底盘耦合限速，看不出控制器本来想发什么。
    """

    def __init__(self, node, topic='/cmd_vel_nav_body'):
        self.topic = topic
        self.samples = []          # [(t, vx, vy, wz)]
        self._sub = node.create_subscription(Twist, topic, self._cb, 50)

    def _cb(self, m):
        self.samples.append((time.monotonic(), m.linear.x, m.linear.y, m.angular.z))

    def reset(self):
        self.samples.clear()

    @property
    def peak_abs_vy(self):
        return max((abs(s[2]) for s in self.samples), default=0.0)

    @property
    def peak_abs_vx(self):
        return max((abs(s[1]) for s in self.samples), default=0.0)

    @property
    def peak_abs_wz(self):
        return max((abs(s[3]) for s in self.samples), default=0.0)


def run_one_goal(node, truth, watcher, ac, goal_pose, goal_xy, label):
    """派发一个目标并采集全过程。返回度量字典。"""
    p_start = truth.pose()
    watcher.reset()
    traj = [p_start]

    goal = NavigateToPose.Goal()
    goal.pose = goal_pose
    send = ac.send_goal_async(goal)
    end = time.monotonic() + 10.0
    while not send.done() and time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
    if not send.done():
        raise TimeoutError('派发目标 10s 无响应，bt_navigator 没起？')
    handle = send.result()
    if not handle.accepted:
        return {'label': label, 'accepted': False, 'status': None}

    res_fut = handle.get_result_async()
    t0 = time.monotonic()
    rotate_first = 0.0          # 净位移超过 MOVE_ONSET_M 之前的最大原地转角
    moved_onset = False
    path_len = 0.0
    while not res_fut.done() and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        p = truth.pose()
        if p.stamp != traj[-1].stamp:
            path_len += math.hypot(p.x - traj[-1].x, p.y - traj[-1].y)
            traj.append(p)
            net = math.hypot(p.x - p_start.x, p.y - p_start.y)
            if not moved_onset:
                rotate_first = max(rotate_first,
                                   abs(unwrap_delta(p.yaw, p_start.yaw)))
                if net > MOVE_ONSET_M:
                    moved_onset = True
        if time.monotonic() - t0 > GOAL_TIMEOUT_SEC:
            handle.cancel_goal_async()
            break

    status = res_fut.result().status if res_fut.done() else None
    # 收尾：让惯性和收敛尾巴走完再取终点
    settle_end = time.monotonic() + 2.0
    while time.monotonic() < settle_end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
    p_end = truth.pose()

    fwd, lat, dyaw = displacement(p_start, p_end)
    return {
        'label': label,
        'accepted': True,
        'status': status,
        'status_name': {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED(4)',
                        GoalStatus.STATUS_ABORTED: 'ABORTED(6)',
                        GoalStatus.STATUS_CANCELED: 'CANCELED(5)'}.get(status,
                                                                      str(status)),
        'goal_error_m': math.hypot(p_end.x - goal_xy[0], p_end.y - goal_xy[1]),
        'net_displacement_m': math.hypot(p_end.x - p_start.x, p_end.y - p_start.y),
        'path_length_m': path_len,
        'body_fwd_m': fwd,
        'body_lat_m': lat,
        'dyaw_rad': dyaw,
        'rotate_first_rad': rotate_first,
        'peak_abs_vy': watcher.peak_abs_vy,
        'peak_abs_vx': watcher.peak_abs_vx,
        'peak_abs_wz': watcher.peak_abs_wz,
        'duration_sec': time.monotonic() - t0,
        'n_cmd_samples': len(watcher.samples),
    }


def main(argv=None):
    p = base_arg_parser(TEST_ID, 'E2-short Nav2 走 30 cm（到点精度 + 控制器行为判别）')
    p.add_argument('--dx', type=float, default=0.30, help='本体系 x 位移 (m)')
    p.add_argument('--dy', type=float, default=0.30, help='本体系 y 位移 (m)')
    p.add_argument('--mode', choices=('diagonal', 'sequential'), default='diagonal')
    args = p.parse_args(argv)

    if args.env != 'sim':
        print('真机版未实现：真机没有独立真值源，自动跑只能读 SDK 航迹推算，'
              '那正是本方案禁止的自我验证。真机应人工量终点到目标点的距离。')
        return 2

    dist = math.hypot(args.dx, args.dy) if args.mode == 'diagonal' else max(
        abs(args.dx), abs(args.dy))
    consumed = XY_GOAL_TOLERANCE / dist if dist > 0 else float('inf')
    print('=' * 76)
    print(f'目标距离 {dist:.4f} m，生效容差 {XY_GOAL_TOLERANCE} m')
    print(f'-> 只需实际移动 {max(0.0, dist - XY_GOAL_TOLERANCE):.4f} m 即判到达'
          f'（{consumed * 100:.0f}% 被容差吃掉）')
    print('-> 所以主指标是「停下来离目标多远」，不是「走了多少」')
    if consumed >= 1.0:
        print('!! 目标距离 <= 容差，goal checker 第一次检查就判到达，机器人不会动。')
        return 2
    print('=' * 76)

    with Session(TEST_ID, args.env, force=args.force,
                 need_open_loop=False,        # 本项**需要** Nav2 在跑
                 extra_context={
                     'dx_body_m': args.dx, 'dy_body_m': args.dy, 'mode': args.mode,
                     'xy_goal_tolerance': XY_GOAL_TOLERANCE,
                     'yaw_goal_tolerance': YAW_GOAL_TOLERANCE,
                     'tolerance_consumed_fraction': consumed,
                     'robot_base_frame': ROBOT_BASE_FRAME,
                 }) as s:

        truth = SimOdomTruth(s.node)
        truth.wait_ready()
        s.context['truth_source'] = truth.name

        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer, s.node)
        global_frame = pick_global_frame(s.node, tf_buffer)
        s.context['global_frame'] = global_frame
        print(f'全局系用 {global_frame}')

        loaded = read_loaded_controller(s.node)
        s.context['controller_plugins'] = loaded
        print(f'controller_server 加载的插件：{loaded}')
        if not loaded:
            print('!! 读不到 controller_plugins。Nav2 没起，或名字被 remap 过。')

        watcher = CmdVelWatcher(s.node)
        s.context['cmd_vel_topic_watched'] = watcher.topic

        ac = ActionClient(s.node, NavigateToPose, '/navigate_to_pose')
        if not ac.wait_for_server(timeout_sec=10.0):
            print('!! /navigate_to_pose 不可用。bt_navigator 没起。')
            return 2

        result = st.Result(TEST_ID, args.env,
                           description=f'Nav2 短距离位移 {args.mode} '
                                       f'({args.dx:+.2f}, {args.dy:+.2f}) m × {args.n}',
                           context=s.context)
        result.note(f'容差 {XY_GOAL_TOLERANCE} m 是这台底盘的**地板**不是可调参数：'
                    f'0.10 已被实测否决（派发 10 次 ARRIVED 0 次），'
                    f'因为滑行量 0.069~0.100 m 与容差同量级。')
        result.note('RPP 在 30 cm 尺度上全部退化：lookahead_dist 0.6 与 '
                    'approach_velocity_scaling_dist 0.6 都 >= 全程长度，'
                    '所以没有路径跟踪发生，只有"直奔终点 + 全程接近减速"。')

        err_m = result.metric('终点到目标距离', 'm', threshold=XY_GOAL_TOLERANCE,
                              min_n=args.n,
                              threshold_basis='与生效的 general_goal_checker.'
                                              'xy_goal_tolerance 同口径 0.18')
        rot_m = result.metric('起步原地转角', 'rad', min_n=args.n,
                              threshold_basis='只记录、不设判据。它是 RPP/MPPI 的'
                                              '行为判别器，不是好坏指标')
        vy_m = result.metric('|vy| 峰值', 'm/s', min_n=args.n,
                             threshold_basis='只记录。RPP 结构上无 vy 应≈0；'
                                             'MPPI(Omni) 应明显非零')
        net_m = result.metric('净位移', 'm', min_n=args.n,
                              threshold_basis='只记录。它会显著小于目标距离，'
                                              '差额就是容差吃掉的部分')
        ok_m = result.metric('非 SUCCEEDED 次数', '次', threshold=0, min_n=1,
                             threshold_basis='状态码 6=ABORTED 不是成功')

        raws = []
        bad = 0
        legs = ([('xy 斜向', args.dx, args.dy)] if args.mode == 'diagonal'
                else [('x', args.dx, 0.0), ('y', 0.0, args.dy)])
        try:
            for i in range(args.n):
                for label, dx, dy in legs:
                    cur = lookup_pose(s.node, tf_buffer, global_frame, ROBOT_BASE_FRAME)
                    goal_pose, goal_xy = body_offset_goal(cur, dx, dy, global_frame)
                    print(f'\n[{i + 1}/{args.n} {label}] 目标 '
                          f'({goal_xy[0]:+.3f}, {goal_xy[1]:+.3f}) @ {global_frame}')
                    r = run_one_goal(s.node, truth, watcher, ac, goal_pose, goal_xy,
                                     f'{label}#{i}')
                    raws.append(r)
                    if not r.get('accepted'):
                        print('  目标被拒')
                        bad += 1
                        continue
                    print(f"  {r['status_name']} · 终点误差 {r['goal_error_m']:.4f} m"
                          f" · 净位移 {r['net_displacement_m']:.4f} m"
                          f" · 路径长 {r['path_length_m']:.4f} m")
                    print(f"  起步原地转角 {r['rotate_first_rad']:.4f} rad"
                          f" ({math.degrees(r['rotate_first_rad']):.1f}°)"
                          f" · |vy|峰值 {r['peak_abs_vy']:.4f}"
                          f" · |wz|峰值 {r['peak_abs_wz']:.4f}"
                          f" · {r['duration_sec']:.1f}s")
                    err_m.add(r['goal_error_m'], **{k: r[k] for k in
                                                    ('label', 'status_name',
                                                     'net_displacement_m')})
                    rot_m.add(r['rotate_first_rad'], label=r['label'])
                    vy_m.add(r['peak_abs_vy'], label=r['label'],
                             peak_abs_vx=r['peak_abs_vx'])
                    net_m.add(r['net_displacement_m'], label=r['label'])
                    if r['status'] != GoalStatus.STATUS_SUCCEEDED:
                        bad += 1
        except (KeyboardInterrupt, TimeoutError) as exc:
            result.note(f'⚠️ 提前中止：{exc}。已采集样本仍写盘。')

        ok_m.add(bad, total=len(raws))

        # 行为判别：把两个记录量合成一个结论
        if rot_m.values and vy_m.values:
            mean_rot = sum(rot_m.values) / len(rot_m.values)
            mean_vy = sum(vy_m.values) / len(vy_m.values)
            if mean_rot > ROTATE_FIRST_RAD and mean_vy < VY_USED_MPS:
                verdict = ('差速式：先原地转向再直行，全程不用横移 —— '
                           '与 RPP 的结构性限制一致（头文件里只有 linear/angular，无 vy）')
            elif mean_rot <= ROTATE_FIRST_RAD and mean_vy >= VY_USED_MPS:
                verdict = '全向式：直接斜向平移、几乎不先转向 —— 与 MPPI motion_model:Omni 一致'
            else:
                verdict = (f'混合/异常：转角 {mean_rot:.3f} rad 与 |vy| {mean_vy:.3f} '
                           f'不构成上面任一种典型模式，需要看原始样本')
            result.note(f'行为判别（起步转角均值 {mean_rot:.4f} rad / '
                        f'|vy| 峰值均值 {mean_vy:.4f} m/s）：{verdict}')
            print(f'\n行为判别：{verdict}')

        result.context['raw_runs'] = raws
        print()
        print(result.to_markdown())
        paths = result.save(results_dir(args.results_dir))
        print(f'结果已写入：\n  {paths[0]}\n  {paths[1]}')
        return 0 if result.verdict == st.PASS else 1


if __name__ == '__main__':
    sys.exit(run_main(main))
