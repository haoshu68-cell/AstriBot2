#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""全向底盘运动学：系数矩阵与由它导出的量。

**纯函数，无 ROS 依赖**，可以离线单测。这一层存在的理由是：底盘的四个已知根因里
有一个就是"运动学矩阵写错"，而矩阵错的表现是"机器人动了、方向不对"——不报错。
所以矩阵本身以及由它导出的预测必须能在不开机器人的情况下被断言住。

系数来源
========
``omni_effort_drive_node.py:123-125`` 的三个参数默认值，逐字抄下来：

    wheel_coeff_vx = [-0.7071,  0.7071, -0.7071,  0.7071]
    wheel_coeff_vy = [-0.7071, -0.7071,  0.7071,  0.7071]
    wheel_coeff_wz = [-0.3060, -0.3060, -0.3024, -0.3024]

含义：``omega_i = (a_i * vx + b_i * vy + c_i * wz) / wheel_radius``。
``(a_i, b_i)`` 就是第 i 个轮的滚动方向单位向量（四个的模都是 1）。

这是 **X 构型 OMNI**，不是麦轮（仓库命名已于 2026-08-21 清理过）：
轮 0/3 共用 +45° 轴，轮 1/2 共用 −45° 轴，两组正交。

!!! 若哪天改了节点里的系数，必须同步改这里，并让 tests/test_kinematics.py 重新通过 !!!
两处漂移的后果是：测试脚本按旧矩阵算判据，节点按新矩阵驱动，谁都不报错。
"""

import math

# ---- 逐字抄自 omni_effort_drive_node.py 的默认参数 ----
WHEEL_COEFF_VX = (-0.7071, 0.7071, -0.7071, 0.7071)
WHEEL_COEFF_VY = (-0.7071, -0.7071, 0.7071, 0.7071)
WHEEL_COEFF_WZ = (-0.3060, -0.3060, -0.3024, -0.3024)

WHEEL_RADIUS_M = 0.08                # declare_parameter('wheel_radius', 0.08)
WHEEL_EFFORT_LIMIT_NM = 15.0         # declare_parameter('wheel_effort_limit_nm', 15.0)
WHEEL_VELOCITY_LIMIT_RAD_S = 40.0    # declare_parameter('wheel_velocity_limit_rad_s', 40.0)
CONTROL_PERIOD_SEC = 0.01            # declare_parameter('control_period_sec', 0.01) → 100 Hz
FRICTION_DEADBAND_RAD_S = 0.05       # declare_parameter('friction_deadband_rad_s', 0.05)

N_WHEELS = 4

# ---- 轮序。系数数组的下标 0..3 就是这个顺序，诊断话题名也按它拼 ----
# 抄自 omni_effort_drive_node.py:51-52
#   WHEEL_ORDER = ('RF', 'LF', 'RR', 'LR')
#   JOINT_NAMES = tuple(f'wheel_{w}_Joint' for w in WHEEL_ORDER)
# 诊断话题：/wheel_effort/<RF|LF|RR|LR>、/wheel_velocity_setpoint/<...>
#
# !!! 轮序错了 F2 的净偏航力矩会算成别的东西且不报错 !!!
# 由系数可读出：前对 RF/LF 的偏航力臂 0.3060，后对 RR/LR 是 0.3024（差 1.2%）。
WHEEL_ORDER = ('RF', 'LF', 'RR', 'LR')
JOINT_NAMES = tuple(f'wheel_{w}_Joint' for w in WHEEL_ORDER)
EFFORT_DEBUG_TOPICS = tuple(f'/wheel_effort/{w}' for w in WHEEL_ORDER)
SETPOINT_DEBUG_TOPICS = tuple(f'/wheel_velocity_setpoint/{w}' for w in WHEEL_ORDER)

# ---- 底盘物理上限，来自 astribot_chassis.yaml 的 joint_max_velocities ----
# nav2 侧 velocity_smoother 的 max_velocity 与它对齐（[1.0, 1.0, 2.0]）。
CHASSIS_MAX_VX = 1.0
CHASSIS_MAX_VY = 1.0
CHASSIS_MAX_WZ = 2.0

# ---- nav2 侧加速度箱式约束，来自两份 nav2_params 的 velocity_smoother ----
NAV2_MAX_ACCEL = (2.5, 2.5, 3.2)     # (ax, ay, az)


def wheel_speeds(vx, vy, wz, wheel_radius=WHEEL_RADIUS_M):
    """车体系速度 -> 四个轮的角速度 (rad/s)。逆运动学，与节点里同一套系数。"""
    if wheel_radius <= 0.0:
        raise ValueError('wheel_radius 必须 > 0')
    return tuple(
        (WHEEL_COEFF_VX[i] * vx + WHEEL_COEFF_VY[i] * vy + WHEEL_COEFF_WZ[i] * wz)
        / wheel_radius
        for i in range(N_WHEELS)
    )


def wheel_rolling_directions():
    """四个轮的滚动方向单位向量 [(ax, ay), ...]。"""
    return tuple((WHEEL_COEFF_VX[i], WHEEL_COEFF_VY[i]) for i in range(N_WHEELS))


def driving_wheel_count(heading_rad, vel_eps=1e-6):
    """沿 heading_rad 方向平移时，**实际在出力**的轮子个数。

    这是 45° 各向同性测试的理论依据：X 构型下沿 ±45° 走时有两个轮完全静止。
    """
    vx = math.cos(heading_rad)
    vy = math.sin(heading_rad)
    return sum(1 for w in wheel_speeds(vx, vy, 0.0) if abs(w) > vel_eps)


def max_wheel_speed_for_unit_translation(heading_rad):
    """沿 heading_rad 走 1 m/s 时的最大单轮角速度 (rad/s)。

    0°/90° 时是 0.7071/r；±45° 时是 1.0/r（高 41%），因为只有两个轮承担全部运动。
    """
    vx = math.cos(heading_rad)
    vy = math.sin(heading_rad)
    return max(abs(w) for w in wheel_speeds(vx, vy, 0.0))


def max_force_along(heading_rad,
                    effort_limit_nm=WHEEL_EFFORT_LIMIT_NM,
                    wheel_radius=WHEEL_RADIUS_M):
    """力矩饱和时，沿 heading_rad 方向能产生的最大合力 (N)。

    每个轮沿自己的滚动方向产生 ``effort_limit_nm / wheel_radius`` 的力，
    取它在目标方向上投影的绝对值之和（每个轮的转向可以自由选，所以取绝对值）。

    0°/90°: 530.3 N    ±45°: 375.0 N   -> 比值恰好 1/sqrt(2)
    """
    f_per_wheel = effort_limit_nm / wheel_radius
    ux, uy = math.cos(heading_rad), math.sin(heading_rad)
    return sum(abs(ax * ux + ay * uy) for ax, ay in wheel_rolling_directions()) * f_per_wheel


def force_anisotropy_ratio(heading_rad):
    """沿 heading_rad 的可用驱动力 / 沿 0° 的可用驱动力。±45° 处为 1/sqrt(2)。"""
    return max_force_along(heading_rad) / max_force_along(0.0)


def nav2_commanded_accel_magnitude(heading_rad, max_accel=NAV2_MAX_ACCEL):
    """nav2 的**箱式**加速度约束在 heading_rad 方向上允许的加速度幅值 (m/s^2)。

    ax_max / ay_max 是各轴独立的上限，所以斜向允许的幅值可以大于任一轴的上限：
    45° 处为 sqrt(ax^2 + ay^2) = 3.54，而平台在该方向的能力只有轴向的 0.707 倍。
    """
    ax, ay = max_accel[0], max_accel[1]
    c, s = abs(math.cos(heading_rad)), abs(math.sin(heading_rad))
    # 箱式约束下沿该方向能走多远：受更早触顶的那个轴限制
    scale = min(ax / c if c > 1e-12 else float('inf'),
                ay / s if s > 1e-12 else float('inf'))
    return scale


def accel_demand_over_capability(heading_rad, axis_capability_m_s2=None):
    """nav2 允许的加速度幅值 / 平台在该方向的能力，>1 表示超发。

    axis_capability_m_s2 = 轴向(0°)实测可达加速度。留空时用 ax_max=2.5 当代理值——
    那只是"配置声称的能力"，**真实值必须由 b3_slip_threshold.py 测出来再回填**。
    """
    if axis_capability_m_s2 is None:
        axis_capability_m_s2 = NAV2_MAX_ACCEL[0]
    capability = axis_capability_m_s2 * force_anisotropy_ratio(heading_rad)
    if capability <= 0.0:
        return float('inf')
    return nav2_commanded_accel_magnitude(heading_rad) / capability


def wheel_speed_headroom(vx, vy, wz):
    """最大单轮转速 / 转速上限。>1 表示会被 wheel_velocity_limit 截断。"""
    return max(abs(w) for w in wheel_speeds(vx, vy, wz)) / WHEEL_VELOCITY_LIMIT_RAD_S


def net_yaw_moment(wheel_efforts):
    """由四个轮力矩算净偏航力矩的**代理量** (N·m)。

    用 wz 列做投影：wz 列本身就是每个轮对偏航的力臂系数。
    F2（静止净偏航力矩）用这个量，判据 |值| <= 0.5
    （修复后实测 −0.098；故障时 60，让车体自转 3.3 rad/s）。
    """
    if len(wheel_efforts) != N_WHEELS:
        raise ValueError(f'需要 {N_WHEELS} 个轮力矩，收到 {len(wheel_efforts)}')
    return sum(WHEEL_COEFF_WZ[i] * wheel_efforts[i] for i in range(N_WHEELS))


def theoretical_stop_distance(v0, decel):
    """理论最短停车距离 v^2/(2a)。0.5 m/s @ 2.5 m/s^2 -> 0.05 m。"""
    if decel <= 0.0:
        raise ValueError('decel 必须 > 0')
    return v0 * v0 / (2.0 * decel)


def theoretical_rise_time(v_target, accel):
    """理论最短加速时间。0 -> 0.5 m/s @ 2.5 m/s^2 -> 0.2 s。"""
    if accel <= 0.0:
        raise ValueError('accel 必须 > 0')
    return abs(v_target) / accel
