#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Copyright (c) 2026, Astribot Co., Ltd.
# All rights reserved.
# License: BSD 3-Clause License
# -----------------------------------------------------------------------------

"""
File: 214-chassis_track_rpp.py
Brief: Regulated Pure Pursuit 局部路径跟踪律的**离线验证**（只算不动）。

================================================================================
这个脚本不会让机器人动
================================================================================
它只做三件事：
  1. 读机器人当前底盘位姿（`get_current_joints_position`，只读，不下发）；
  2. 在内部用**理想运动学积分器**把整趟跟踪跑完；
  3. 打印每一步的指令、以及收敛/过冲/横向误差/终态航向等指标。

**全程一次 `set_joints_position` 都不发。** 什么时候可以真动由人决定，
到时候再单独加执行路径 —— 现在故意连开关都没有，避免误触。

不加任何依赖：只用 Python 标准库 + 机器人上已有的 SDK。numpy 都不需要。

================================================================================
为什么要在真机上"只算"
================================================================================
机器人上没有 `/cmd_vel`、没有 `/odom`、没有 `/joint_states`（实测 42 个话题里全无），
`/tf` 与 `/tf_static` 的 **Publisher count 都是 0**。所以仓库里那批基于仿真 IO 的
跟踪脚本在真机上一条都跑不了。

但**控制律是纯函数**，与 IO 无关。在真机上离线跑它有两个真实价值：
  · 用**真机的实际起始位姿**做初值，而不是仿真里的理想 (0,0,0)；
  · 验证 aarch64 上的数值行为与 x86 一致（同一套算术，但值得确认）。

================================================================================
参数全部沿用仓库现值（nav2_params_rpp.yaml 的 FollowPath 段）
================================================================================
    controller_frequency                20.0
    desired_linear_vel                  0.5
    lookahead_dist                      0.6
    min_lookahead_dist                  0.3
    max_lookahead_dist                  1.2
    min_approach_linear_velocity        0.05
    approach_velocity_scaling_dist      0.6
    regulated_linear_scaling_min_radius 0.9
    regulated_linear_scaling_min_speed  0.25
    rotate_to_heading_angular_vel       1.0
    rotate_to_heading_min_angle         0.785

================================================================================
两个从配置里读出来的结构性问题，脚本显式暴露
================================================================================
**问题 1：前视距离比整条路径还长。**
30 cm 级任务的路径只有 0.30（单轴）/ 0.4243 m（斜向），而 `lookahead_dist` 是 0.6，
连 `min_lookahead_dist` 都**恰好等于 0.30**。纯追踪的前视圆与路径不相交时只能取
路径末点当 carrot —— **全程没有任何路径跟踪发生**。
`--shrink-lookahead`（默认开）让前视随剩余距离收缩：`L = clamp(0.5*剩余, 0.08, 0.6)`。
用 `--no-shrink-lookahead` 可以看到不修时的退化行为，两者对比就是这个问题的证据。

**问题 2：接近速度有下限，永远刹不到零。**
`min_approach_linear_velocity: 0.05` 意味着到点判据满足前速度恒 >= 0.05 m/s。
这是 RPP 必须搭配较宽到位容差的结构性原因之一。本脚本会打印"到位那一刻的速度"，
把这件事变成一个可看见的数。

================================================================================
用法
================================================================================
    # 机器人上（两套 ROS 都要 source，顺序不能反）
    source /opt/ros/humble/setup.bash
    cd ~/Downloads/astribot_sdk_aarch64 && source ./env.sh
    python3 examples/214-chassis_track_rpp.py                    # 斜向 30cm
    python3 examples/214-chassis_track_rpp.py --dx 0.3 --dy 0.0  # 单轴
    python3 examples/214-chassis_track_rpp.py --no-shrink-lookahead
    python3 examples/214-chassis_track_rpp.py --no-robot         # 不连机器人，纯离线
"""

import argparse
import math
import sys

# ---- 逐字沿用 nav2_params_rpp.yaml 的 FollowPath 段 ----
CONTROLLER_FREQUENCY_HZ = 20.0
DESIRED_LINEAR_VEL = 0.5
LOOKAHEAD_DIST = 0.6
MIN_LOOKAHEAD_DIST = 0.3
MAX_LOOKAHEAD_DIST = 1.2
MIN_APPROACH_LINEAR_VELOCITY = 0.05
APPROACH_VELOCITY_SCALING_DIST = 0.6
REGULATED_MIN_RADIUS = 0.9
REGULATED_MIN_SPEED = 0.25
ROTATE_TO_HEADING_ANGULAR_VEL = 1.0
ROTATE_TO_HEADING_MIN_ANGLE = 0.785

# ---- 底盘物理上限（astribot_chassis.yaml / velocity_smoother 同口径）----
MAX_VEL_XY = 1.0
MAX_VEL_THETA = 2.0
MAX_ACCEL_XY = 2.5
MAX_ACCEL_THETA = 3.2

# ---- 短路径前视收缩（本脚本的参数，见文件头"问题 1"）----
LOOKAHEAD_SHRINK_K = 0.5
LOOKAHEAD_FLOOR = 0.08

PATH_STEP = 0.02          # 路径采样步长 (m)，代价地图分辨率 0.05 的 40%
IDX_X, IDX_Y, IDX_THETA = 0, 1, 2


# ============================== 纯几何 ==============================

def wrap_angle(a):
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def straight_path(x0, y0, x1, y1, step=PATH_STEP):
    d = math.hypot(x1 - x0, y1 - y0)
    if d < 1e-12:
        return [(x0, y0)]
    n = max(1, int(math.ceil(d / step)))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


def to_body(px, py, rx, ry, ryaw):
    dx, dy = px - rx, py - ry
    c, s = math.cos(-ryaw), math.sin(-ryaw)
    return (c * dx - s * dy, s * dx + c * dy)


def circle_segment_intersection(p1, p2, center, radius):
    """线段与圆的交点，取沿 p1->p2 最远的那个。纯追踪取 carrot 的标准做法。"""
    x1, y1 = p1[0] - center[0], p1[1] - center[1]
    x2, y2 = p2[0] - center[0], p2[1] - center[1]
    dx, dy = x2 - x1, y2 - y1
    a = dx * dx + dy * dy
    if a < 1e-18:
        return None
    b = 2.0 * (x1 * dx + y1 * dy)
    c = x1 * x1 + y1 * y1 - radius * radius
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    best = None
    for t in ((-b + sq) / (2.0 * a), (-b - sq) / (2.0 * a)):
        if -1e-9 <= t <= 1.0 + 1e-9:
            t = min(1.0, max(0.0, t))
            if best is None or t > best:
                best = t
    if best is None:
        return None
    return (p1[0] + best * (p2[0] - p1[0]), p1[1] + best * (p2[1] - p1[1]))


def effective_lookahead(remaining, shrink=True):
    if not shrink:
        return max(MIN_LOOKAHEAD_DIST, min(MAX_LOOKAHEAD_DIST, LOOKAHEAD_DIST))
    if remaining <= 0.0:
        return LOOKAHEAD_FLOOR
    return max(LOOKAHEAD_FLOOR, min(LOOKAHEAD_DIST, LOOKAHEAD_SHRINK_K * remaining))


def find_carrot(path, robot_xy, lookahead, from_index=0):
    if not path:
        raise ValueError('path 为空')
    if len(path) == 1:
        return path[0], 0
    nearest_i, nearest_d = from_index, float('inf')
    for i in range(from_index, len(path)):
        d = math.hypot(path[i][0] - robot_xy[0], path[i][1] - robot_xy[1])
        if d < nearest_d:
            nearest_d, nearest_i = d, i
    for i in range(nearest_i, len(path) - 1):
        hit = circle_segment_intersection(path[i], path[i + 1], robot_xy, lookahead)
        if hit is not None:
            return hit, i
    return path[-1], len(path) - 1


def cross_track_error(robot_xy, path):
    if len(path) == 1:
        return math.hypot(robot_xy[0] - path[0][0], robot_xy[1] - path[0][1])
    best = float('inf')
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        t = 0.0 if seg2 < 1e-18 else max(0.0, min(
            1.0, ((robot_xy[0] - ax) * dx + (robot_xy[1] - ay) * dy) / seg2))
        px, py = ax + t * dx, ay + t * dy
        best = min(best, math.hypot(robot_xy[0] - px, robot_xy[1] - py))
    return best


# ============================== RPP 控制律 ==============================

def regulate_by_curvature(v, curvature):
    if abs(curvature) < 1e-9:
        return v
    radius = 1.0 / abs(curvature)
    if radius >= REGULATED_MIN_RADIUS:
        return v
    return max(REGULATED_MIN_SPEED, v * radius / REGULATED_MIN_RADIUS)


def regulate_by_approach(v, dist_to_goal):
    """接近限速。**注意有下限 min_approach_linear_velocity，刹不到零。**"""
    if dist_to_goal >= APPROACH_VELOCITY_SCALING_DIST:
        return v
    return max(MIN_APPROACH_LINEAR_VELOCITY,
               v * dist_to_goal / APPROACH_VELOCITY_SCALING_DIST)


def rpp_step(robot, path, from_index=0, shrink=True,
             desired_linear_vel=DESIRED_LINEAR_VEL):
    """RPP 一步。robot=(x,y,yaw)，返回 ((vx, vy=0, wz), info)。

    **输出恒无 vy** —— RPP 是为差速/阿克曼设计的，这是它的结构性限制，
    不是参数没调好（nav2 的头文件里只有 linear_vel / angular_vel）。
    """
    goal = path[-1]
    dist_to_goal = math.hypot(goal[0] - robot[0], goal[1] - robot[1])
    L = effective_lookahead(dist_to_goal, shrink)
    carrot, idx = find_carrot(path, (robot[0], robot[1]), L, from_index)
    cb = to_body(carrot[0], carrot[1], robot[0], robot[1], robot[2])
    angle = math.atan2(cb[1], cb[0])
    info = {'lookahead': L, 'carrot': carrot, 'carrot_body': cb, 'path_index': idx,
            'dist_to_goal': dist_to_goal, 'angle_to_carrot': angle,
            'carrot_at_path_end': carrot == path[-1], 'rotating_in_place': False,
            'curvature': 0.0}

    # 角差过大时先原地转向。斜向目标 (0.3,0.3) 的方向角 45°=0.785 恰好等于阈值，
    # 这就是"RPP 在斜向目标上先转 45°"的来源。
    if abs(angle) > ROTATE_TO_HEADING_MIN_ANGLE:
        info['rotating_in_place'] = True
        return (0.0, 0.0, math.copysign(ROTATE_TO_HEADING_ANGULAR_VEL, angle)), info

    k = 0.0 if L <= 1e-9 else 2.0 * cb[1] / (L * L)
    v = min(regulate_by_approach(regulate_by_curvature(desired_linear_vel, k),
                                 dist_to_goal), MAX_VEL_XY)
    wz = max(-MAX_VEL_THETA, min(MAX_VEL_THETA, v * k))
    info['curvature'] = k
    return (v, 0.0, wz), info


def slew(target, prev, dt):
    """加速度限幅，与 velocity_smoother 的 max_accel 同口径。"""
    limits = (MAX_ACCEL_XY, MAX_ACCEL_XY, MAX_ACCEL_THETA)
    out = []
    for i in range(3):
        md = limits[i] * dt
        out.append(prev[i] + max(-md, min(md, target[i] - prev[i])))
    return tuple(out)


def clamp_xy(twist):
    """xy 按**合成模长**等比缩放，不是逐轴裁剪（逐轴裁会改变运动方向）。"""
    vx, vy, wz = twist
    n = math.hypot(vx, vy)
    if n > MAX_VEL_XY:
        vx, vy = vx * MAX_VEL_XY / n, vy * MAX_VEL_XY / n
    return (vx, vy, max(-MAX_VEL_THETA, min(MAX_VEL_THETA, wz)))


# ============================== 离线闭环 ==============================

def simulate(start, dx_body, dy_body, shrink=True, stop_tol=0.05,
             cruise=DESIRED_LINEAR_VEL, max_steps=4000, verbose_every=0):
    """理想运动学闭环：下发的 twist 直接成为真实速度。

    **不含任何动力学**（无质量、无摩擦、无打滑），所以它测的是控制律本身。
    真机上的表现一定比这个差，差多少就是底盘的动力学代价。
    """
    dt = 1.0 / CONTROLLER_FREQUENCY_HZ
    x, y, yaw = start
    c, s = math.cos(yaw), math.sin(yaw)
    gx = x + c * dx_body - s * dy_body
    gy = y + s * dx_body + c * dy_body
    path = straight_path(x, y, gx, gy)
    total = math.hypot(gx - x, gy - y)

    idx = 0
    prev = (0.0, 0.0, 0.0)
    xte_max = 0.0
    overshoot = 0.0
    rotate_first = 0.0
    moved = False
    rotating_steps = 0
    end_at_path_end = 0
    peak_vy = 0.0
    speed_at_arrival = None
    trace = []

    for step in range(max_steps):
        d = math.hypot(gx - x, gy - y)
        if d <= stop_tol:
            speed_at_arrival = math.hypot(prev[0], prev[1])
            return {'converged': True, 'steps': step, 'final_dist': d,
                    'xte_max': xte_max, 'overshoot': overshoot,
                    'final_yaw': yaw, 'rotate_first': rotate_first,
                    'rotating_steps': rotating_steps, 'peak_vy': peak_vy,
                    'speed_at_arrival': speed_at_arrival,
                    'carrot_at_end_steps': end_at_path_end,
                    'goal': (gx, gy), 'total': total, 'path': path,
                    'trace': trace, 'end_pose': (x, y, yaw)}

        twist, info = rpp_step((x, y, yaw), path, idx, shrink, cruise)
        idx = info['path_index']
        if info['rotating_in_place']:
            rotating_steps += 1
        if info['carrot_at_path_end']:
            end_at_path_end += 1

        vel = slew(clamp_xy(twist), prev, dt)
        prev = vel
        peak_vy = max(peak_vy, abs(vel[1]))

        cy, sy = math.cos(yaw), math.sin(yaw)
        x += (cy * vel[0] - sy * vel[1]) * dt
        y += (sy * vel[0] + cy * vel[1]) * dt
        yaw = wrap_angle(yaw + vel[2] * dt)

        xte_max = max(xte_max, cross_track_error((x, y), path))
        overshoot = max(overshoot, math.hypot(x - start[0], y - start[1]) - total)
        if not moved:
            rotate_first = max(rotate_first, abs(wrap_angle(yaw - start[2])))
            if math.hypot(x - start[0], y - start[1]) > 0.02:
                moved = True

        rec = {'step': step, 'dist': d, 'L': info['lookahead'],
               'vx': vel[0], 'vy': vel[1], 'wz': vel[2],
               'rot': info['rotating_in_place'],
               'carrot_end': info['carrot_at_path_end'],
               'xte': cross_track_error((x, y), path)}
        trace.append(rec)
        if verbose_every and step % verbose_every == 0:
            print(f"    step {step:4d} d={d:.4f} L={info['lookahead']:.3f} "
                  f"v=({vel[0]:+.3f},{vel[1]:+.3f}) wz={vel[2]:+.3f} "
                  f"{'原地转' if info['rotating_in_place'] else '     '} "
                  f"{'carrot=末点' if info['carrot_at_path_end'] else ''}")

    return {'converged': False, 'steps': max_steps,
            'final_dist': math.hypot(gx - x, gy - y), 'xte_max': xte_max,
            'overshoot': overshoot, 'final_yaw': yaw,
            'rotate_first': rotate_first, 'rotating_steps': rotating_steps,
            'peak_vy': peak_vy, 'speed_at_arrival': None,
            'carrot_at_end_steps': end_at_path_end,
            'goal': (gx, gy), 'total': total, 'path': path,
            'trace': trace, 'end_pose': (x, y, yaw)}


def read_robot_pose():
    """只读当前底盘位姿。失败返回 None（不阻断，退化成从原点算）。

    **只调 get_current_joints_position，不发任何指令。**
    """
    try:
        from astribot_sdk.core.astribot_api.astribot_client import Astribot
    except Exception as exc:                                      # noqa: BLE001
        print(f'  SDK import 失败（{type(exc).__name__}），改用原点起算。'
              f'两套 ROS 都 source 了吗？先 /opt/ros/humble 再 env.sh')
        return None
    try:
        bot = Astribot()
        q = bot.get_current_joints_position([bot.chassis_name])[0]
        if len(q) < 3:
            print(f'  底盘自由度 {len(q)} < 3（ROBOT_TYPE 未设为 S1？），改用原点起算')
            return None
        return (q[0], q[1], q[2])
    except Exception as exc:                                      # noqa: BLE001
        print(f'  读位姿失败（{type(exc).__name__}: {exc}），改用原点起算')
        return None


def report(tag, r, stop_tol):
    print(f'\n--- {tag} ---')
    if not r['converged']:
        print(f'  !! 未收敛（{r["steps"]} 步用尽），终点还差 {r["final_dist"]:.4f} m')
    print(f'  收敛步数        {r["steps"]}  ({r["steps"] / CONTROLLER_FREQUENCY_HZ:.2f} s @ '
          f'{CONTROLLER_FREQUENCY_HZ:g} Hz)')
    print(f'  终点误差        {r["final_dist"]:.4f} m   (到位判据 {stop_tol:g})')
    print(f'  最大横向误差    {r["xte_max"]:.4f} m')
    print(f'  过冲            {r["overshoot"]:+.4f} m')
    print(f'  起步原地转角    {r["rotate_first"]:.4f} rad '
          f'({math.degrees(r["rotate_first"]):.1f}°)')
    print(f'  原地转向步数    {r["rotating_steps"]} / {r["steps"]}')
    print(f'  |vy| 峰值       {r["peak_vy"]:.4f} m/s   <- RPP 结构上恒 0')
    print(f'  终态航向变化    {math.degrees(wrap_angle(r["final_yaw"])):+.1f}°')
    if r['speed_at_arrival'] is not None:
        print(f'  到位那刻的速度  {r["speed_at_arrival"]:.4f} m/s'
              f'   <- 接近速度下限 {MIN_APPROACH_LINEAR_VELOCITY:g}，刹不到零')
    print(f'  carrot 退化为末点的步数  {r["carrot_at_end_steps"]} / {r["steps"]}'
          f'   <- 这些步没有真正的路径跟踪')


def main():
    ap = argparse.ArgumentParser(
        description='RPP 跟踪律离线验证（只算不动，不发任何指令）')
    ap.add_argument('--dx', type=float, default=0.30, help='本体系 x 位移 (m)')
    ap.add_argument('--dy', type=float, default=0.30, help='本体系 y 位移 (m)')
    ap.add_argument('--cruise', type=float, default=DESIRED_LINEAR_VEL,
                    help=f'巡航速度 (m/s)，默认 desired_linear_vel={DESIRED_LINEAR_VEL}')
    ap.add_argument('--stop-tol', type=float, default=0.05, help='到位判据 (m)')
    ap.add_argument('--no-shrink-lookahead', action='store_true',
                    help='用标称前视 0.6（展示"前视比路径长"的退化行为）')
    ap.add_argument('--both', action='store_true',
                    help='收缩前视 / 不收缩 两种都跑，直接对比')
    ap.add_argument('--no-robot', action='store_true', help='不连机器人，从原点起算')
    ap.add_argument('--trace-every', type=int, default=0,
                    help='每 N 步打印一行明细，0=不打印')
    args = ap.parse_args()

    print('=' * 78)
    print('RPP 跟踪律离线验证 —— 只算不动，全程不发 set_joints_position')
    print('=' * 78)

    start = None if args.no_robot else read_robot_pose()
    if start is None:
        start = (0.0, 0.0, 0.0)
        print(f'起始位姿：原点 {start}（未连机器人或读取失败）')
    else:
        print(f'起始位姿：机器人实测 x={start[0]:+.4f} y={start[1]:+.4f} '
              f'theta={start[2]:+.4f}')

    leg = math.hypot(args.dx, args.dy)
    print(f'\n任务：本体系位移 ({args.dx:+.3f}, {args.dy:+.3f}) m，'
          f'合成 {leg:.4f} m，方向 {math.degrees(math.atan2(args.dy, args.dx)):+.1f}°')
    print(f'控制频率 {CONTROLLER_FREQUENCY_HZ:g} Hz，巡航上限 {args.cruise:g} m/s')
    print(f'\n配置里的两个结构性问题：')
    print(f'  1) lookahead_dist {LOOKAHEAD_DIST:g} m'
          f'{" >" if LOOKAHEAD_DIST > leg else " <="} 全程 {leg:.4f} m；'
          f'min_lookahead_dist {MIN_LOOKAHEAD_DIST:g} m')
    print(f'  2) min_approach_linear_velocity {MIN_APPROACH_LINEAR_VELOCITY:g} m/s '
          f'-> 到位前速度恒 >= 此值')

    if args.both:
        for tag, shrink in (('收缩前视（L=0.5×剩余, 下限 0.08）', True),
                            (f'标称前视（L={LOOKAHEAD_DIST:g}，不收缩）', False)):
            r = simulate(start, args.dx, args.dy, shrink, args.stop_tol,
                         args.cruise, verbose_every=args.trace_every)
            report(tag, r, args.stop_tol)
        print('\n对比要点：不收缩时 "carrot 退化为末点的步数" 应接近总步数，'
              '\n            也就是全程都在直奔终点，没有路径跟踪。')
    else:
        shrink = not args.no_shrink_lookahead
        tag = ('收缩前视' if shrink else f'标称前视 {LOOKAHEAD_DIST:g}（不收缩）')
        r = simulate(start, args.dx, args.dy, shrink, args.stop_tol,
                     args.cruise, verbose_every=args.trace_every)
        report(tag, r, args.stop_tol)

    print('\n' + '=' * 78)
    print('这是理想运动学结果：无质量、无摩擦、无打滑。')
    print('真机表现一定更差，差值就是底盘的动力学代价。')
    print('本脚本没有执行路径 —— 要让底盘真动，由人决定后再单独加。')
    print('=' * 78)
    return 0


if __name__ == '__main__':
    sys.exit(main())
