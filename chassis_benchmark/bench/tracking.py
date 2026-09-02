#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自己实现的局部路径跟踪控制律。**纯函数，无 ROS 依赖**，可离线单测。

不调用 Nav2 的任何接口，从控制原理写起，参数值全部沿用仓库现有配置。
两套控制律并列实现，是为了让「同一条路径 + 同一组参数」下的差异可测：

* ``rpp_step``  —— 忠实复刻 Regulated Pure Pursuit 的**差速**律：
  前视点 -> 曲率 -> 曲率/接近双重限速 -> (v, wz)，**没有 vy**。
  用来在离线和仿真里复现 nav2 那个控制器的行为（含"先原地转向"）。
* ``omni_step`` —— 面向本机 X 构型全向底盘的**完整全向**律：
  前视点方向直接分解成 (vx, vy)，航向单独用 P 律保持，不需要先转向。

================================================================================
两个从配置里读出来的结构性问题，本模块显式修掉
================================================================================
**问题 1：前视距离比整条路径还长。**
``lookahead_dist: 0.6``、``min_lookahead_dist: 0.3``，而 30 cm 级任务的整条路径只有
0.30~0.42 m。纯追踪的前视圆与路径不相交时只能退化成"取末点当 carrot"，
于是全程没有任何路径跟踪发生。
修法（``effective_lookahead``）：让前视距离随**剩余距离**收缩，
``L = clamp(k * remaining, L_floor, L_nominal)``。这样 carrot 始终落在路径上，
短路径也有真正的跟踪行为。k 与 L_floor 是显式参数，不是魔数。

**问题 2：RPP 的接近速度有下限，永远刹不到零。**
``min_approach_linear_velocity: 0.05`` 意味着 RPP 在到点判据满足之前速度**恒 >= 0.05 m/s**。
这是它必须搭配一个较宽到位容差的结构性原因之一。
``omni_step`` 不设这个下限：它用"刹得住"曲线把速度收到 0
（``brake_speed_cap``，与 bench/driver.py 同一条离散安全公式）。
**能不能因此换来更紧的到位容差，是 e3 脚本要实测的问题，不是本模块的断言。**
"""

import math

# ---- 逐字沿用 nav2_params_rpp.yaml 的 FollowPath 段（改 yaml 要同步改这里）----
CONTROLLER_FREQUENCY_HZ = 20.0          # controller_server.controller_frequency
DESIRED_LINEAR_VEL = 0.5                # desired_linear_vel
LOOKAHEAD_DIST = 0.6                    # lookahead_dist
MIN_LOOKAHEAD_DIST = 0.3                # min_lookahead_dist
MAX_LOOKAHEAD_DIST = 1.2                # max_lookahead_dist
MIN_APPROACH_LINEAR_VELOCITY = 0.05     # min_approach_linear_velocity
APPROACH_VELOCITY_SCALING_DIST = 0.6    # approach_velocity_scaling_dist
REGULATED_MIN_RADIUS = 0.9              # regulated_linear_scaling_min_radius
REGULATED_MIN_SPEED = 0.25              # regulated_linear_scaling_min_speed
ROTATE_TO_HEADING_ANGULAR_VEL = 1.0     # rotate_to_heading_angular_vel
ROTATE_TO_HEADING_MIN_ANGLE = 0.785     # rotate_to_heading_min_angle
MAX_ANGULAR_ACCEL = 3.2                 # max_angular_accel

# ---- 底盘物理上限（astribot_chassis.yaml / velocity_smoother 同口径）----
MAX_VEL_XY = 1.0
MAX_VEL_THETA = 2.0
MAX_ACCEL_XY = 2.5
MAX_ACCEL_THETA = 3.2

# ---- 短路径前视收缩（本模块自己的参数，见文件头"问题 1"）----
LOOKAHEAD_SHRINK_K = 0.5     # L = k * remaining
LOOKAHEAD_FLOOR = 0.08       # 前视下限 (m)。太小会让 carrot 抖，太大会退化成取末点

# ---- omni 律的航向保持 ----
YAW_HOLD_KP = 1.5            # rad/s per rad


def wrap_angle(a):
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def straight_path(x0, y0, x1, y1, step=0.02):
    """两点间的直线路径，等间距采样。

    这是"全局规划器"在本任务里该产出的东西 —— 30 cm 的空旷直线位移不需要搜索。
    step 取 0.02 m（代价地图分辨率 0.05 的 40%），保证 carrot 插值足够细。
    """
    if step <= 0.0:
        raise ValueError('step 必须 > 0')
    d = math.hypot(x1 - x0, y1 - y0)
    if d < 1e-12:
        return [(x0, y0)]
    n = max(1, int(math.ceil(d / step)))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


def to_body(px, py, robot_x, robot_y, robot_yaw):
    """世界系点 -> 机器人本体系。"""
    dx, dy = px - robot_x, py - robot_y
    c, s = math.cos(-robot_yaw), math.sin(-robot_yaw)
    return (c * dx - s * dy, s * dx + c * dy)


def circle_segment_intersection(p1, p2, center, radius):
    """线段 p1->p2 与以 center 为心、radius 为半径的圆的交点。

    返回沿 p1->p2 方向**最远**的那个交点，无交点返回 None。
    纯追踪取 carrot 的标准做法；与 RPP 的 circleSegmentIntersection 同一件事。

    解 |p1 + t*(p2-p1) - center|^2 = r^2，取 t in [0,1] 中较大的根。
    """
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


def effective_lookahead(remaining_dist, nominal=LOOKAHEAD_DIST,
                        k=LOOKAHEAD_SHRINK_K, floor=LOOKAHEAD_FLOOR):
    """随剩余距离收缩的前视距离。见文件头"问题 1"。

    remaining 很大时取 nominal；很小时取 k*remaining，但不低于 floor。
    """
    if remaining_dist <= 0.0:
        return floor
    return max(floor, min(nominal, k * remaining_dist))


def find_carrot(path, robot_xy, lookahead, from_index=0):
    """在 path 上找距 robot 恰好 lookahead 的前视点。

    返回 (carrot_xy, 已推进到的路径下标)。
    没有交点时返回路径末点 —— 但那意味着已经进入"直奔终点"模式，
    调用方可以用返回的 carrot == path[-1] 来识别。
    """
    if not path:
        raise ValueError('path 为空')
    if len(path) == 1:
        return path[0], 0
    # 先把起始下标推进到离机器人最近的那一段，避免回头找到旧的交点
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


def brake_speed_cap(remaining, accel, dt):
    """离散安全的"刹得住"速度上限（与 bench/driver.py 同一条公式）。

        v^2/(2a) + v*dt <= |s|   ->   v <= -a*dt + sqrt((a*dt)^2 + 2*a*|s|)

    天真的 sqrt(2*a*s) 会过冲：那条曲线要求的减速度恰好是 a，一步都不能落后。
    """
    if accel <= 0.0 or dt <= 0.0:
        raise ValueError('accel/dt 必须 > 0')
    adt = accel * dt
    return max(0.0, -adt + math.sqrt(adt * adt + 2.0 * accel * abs(remaining)))


def curvature_from_carrot(carrot_body, lookahead):
    """纯追踪曲率 k = 2*y_c / L^2（y_c 为 carrot 在本体系的横向坐标）。"""
    if lookahead <= 1e-9:
        return 0.0
    return 2.0 * carrot_body[1] / (lookahead * lookahead)


def regulate_speed_by_curvature(v, curvature,
                                min_radius=REGULATED_MIN_RADIUS,
                                min_speed=REGULATED_MIN_SPEED):
    """曲率限速：转弯半径小于 min_radius 时按比例降速，但不低于 min_speed。"""
    if abs(curvature) < 1e-9:
        return v
    radius = 1.0 / abs(curvature)
    if radius >= min_radius:
        return v
    return max(min_speed, v * radius / min_radius)


def regulate_speed_by_approach(v, dist_to_goal,
                               scaling_dist=APPROACH_VELOCITY_SCALING_DIST,
                               min_approach=MIN_APPROACH_LINEAR_VELOCITY):
    """接近限速：距终点小于 scaling_dist 时线性降速，**但有下限 min_approach**。

    这个下限是 RPP 的结构特征（min_approach_linear_velocity: 0.05）：
    它在到点判据满足之前速度恒 >= 0.05 m/s，刹不到零。omni_step 不用这条。
    """
    if scaling_dist <= 0.0 or dist_to_goal >= scaling_dist:
        return v
    return max(min_approach, v * dist_to_goal / scaling_dist)


def rpp_step(robot, path, dt, from_index=0,
             desired_linear_vel=DESIRED_LINEAR_VEL,
             nominal_lookahead=LOOKAHEAD_DIST,
             shrink_lookahead=True):
    """Regulated Pure Pursuit 的一步（**差速律，输出没有 vy**）。

    robot: (x, y, yaw)
    返回 (twist, info)；twist = (vx, vy=0.0, wz)。
    """
    goal = path[-1]
    dist_to_goal = math.hypot(goal[0] - robot[0], goal[1] - robot[1])
    L = (effective_lookahead(dist_to_goal, nominal_lookahead) if shrink_lookahead
         else max(MIN_LOOKAHEAD_DIST, min(MAX_LOOKAHEAD_DIST, nominal_lookahead)))
    carrot, idx = find_carrot(path, (robot[0], robot[1]), L, from_index)
    cb = to_body(carrot[0], carrot[1], robot[0], robot[1], robot[2])
    angle_to_carrot = math.atan2(cb[1], cb[0])

    info = {'lookahead': L, 'carrot': carrot, 'carrot_body': cb,
            'path_index': idx, 'dist_to_goal': dist_to_goal,
            'angle_to_carrot': angle_to_carrot, 'rotating_in_place': False,
            'carrot_at_path_end': carrot == path[-1]}

    # 先原地转向：|角差| 超过 rotate_to_heading_min_angle 时只转不走。
    # 这是差速载体的必然行为，也是 RPP 在斜向目标上"先转 45°"的来源。
    if abs(angle_to_carrot) > ROTATE_TO_HEADING_MIN_ANGLE:
        wz = math.copysign(ROTATE_TO_HEADING_ANGULAR_VEL, angle_to_carrot)
        info['rotating_in_place'] = True
        info['curvature'] = 0.0
        return (0.0, 0.0, wz), info

    k = curvature_from_carrot(cb, L)
    v = desired_linear_vel
    v = regulate_speed_by_curvature(v, k)
    v = regulate_speed_by_approach(v, dist_to_goal)
    # 别超过底盘上限
    v = min(v, MAX_VEL_XY)
    wz = max(-MAX_VEL_THETA, min(MAX_VEL_THETA, v * k))
    info['curvature'] = k
    return (v, 0.0, wz), info


def omni_step(robot, path, dt, from_index=0, reference_yaw=None,
              desired_linear_vel=DESIRED_LINEAR_VEL,
              accel=MAX_ACCEL_XY, nominal_lookahead=LOOKAHEAD_DIST,
              yaw_kp=YAW_HOLD_KP):
    """全向律的一步：前视点方向直接分解成 (vx, vy)，航向单独 P 律保持。

    与 rpp_step 的三个本质区别：
      1. **不先转向**：方向由 (vx, vy) 表达，车头不必对准运动方向。
      2. **没有接近速度下限**：用 brake_speed_cap 把速度收到 0。
      3. **航向是独立自由度**：默认保持 reference_yaw（缺省用当前 yaw 冻结），
         而不是让它跟随路径切线。

    robot: (x, y, yaw)；reference_yaw: 要保持的航向，None 表示保持当前。
    返回 (twist, info)；twist = (vx, vy, wz)，本体系。
    """
    goal = path[-1]
    dist_to_goal = math.hypot(goal[0] - robot[0], goal[1] - robot[1])
    L = effective_lookahead(dist_to_goal, nominal_lookahead)
    carrot, idx = find_carrot(path, (robot[0], robot[1]), L, from_index)
    cb = to_body(carrot[0], carrot[1], robot[0], robot[1], robot[2])
    norm = math.hypot(cb[0], cb[1])

    # 速度：巡航上限与"刹得住"上限取小者。后者保证能停在终点而不过冲。
    speed = min(desired_linear_vel, brake_speed_cap(dist_to_goal, accel, dt))
    speed = min(speed, MAX_VEL_XY)

    if norm < 1e-9:
        vx = vy = 0.0
    else:
        vx = speed * cb[0] / norm
        vy = speed * cb[1] / norm

    ref = robot[2] if reference_yaw is None else reference_yaw
    yaw_err = wrap_angle(ref - robot[2])
    wz = max(-MAX_VEL_THETA, min(MAX_VEL_THETA, yaw_kp * yaw_err))

    return (vx, vy, wz), {
        'lookahead': L, 'carrot': carrot, 'carrot_body': cb, 'path_index': idx,
        'dist_to_goal': dist_to_goal, 'speed': speed, 'yaw_error': yaw_err,
        'carrot_at_path_end': carrot == path[-1], 'rotating_in_place': False,
    }


def cross_track_error(robot_xy, path):
    """机器人到路径的最短距离（横向跟踪误差）。逐段求点到线段距离取最小。"""
    if not path:
        raise ValueError('path 为空')
    if len(path) == 1:
        return math.hypot(robot_xy[0] - path[0][0], robot_xy[1] - path[0][1])
    best = float('inf')
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-18:
            t = 0.0
        else:
            t = ((robot_xy[0] - ax) * dx + (robot_xy[1] - ay) * dy) / seg2
            t = max(0.0, min(1.0, t))
        px, py = ax + t * dx, ay + t * dy
        best = min(best, math.hypot(robot_xy[0] - px, robot_xy[1] - py))
    return best
