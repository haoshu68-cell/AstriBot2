#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘位置积分的纯函数核心（不依赖 rclpy / 不依赖厂商 SDK，可离线单测）。

接口事实来源
============
本文件所有下发语义都来自 examples 源码，不来自任何文档：

* ``examples/202-chassis_joy_control_local.py:44-52``（本体系）::

      pos_cmd = astribot.get_desired_joints_position([chassis_name])[0]   # 种子
      pos_cmd[i] += vel_cmd[i] / freq                                     # 直接积分
      astribot.set_joints_position([chassis_name], [pos_cmd])

* ``examples/203-chassis_joy_control_global.py:54-66``（世界系）::

      rot_mat = [[cos, -sin, 0], [sin, cos, 0], [0, 0, 1]]   # local -> global
      vel_cmd = rot_mat.T @ vel                              # global -> local
      pos_cmd[i] += vel_cmd[i] / freq

!!! 旋转方向以源码为准 !!!
203 第 61 行是 ``rot_mat.T @ vel``，注释写的是 "rotate global velocity command to
local coordinate" —— 方向是**世界系 → 本体系**。由此推断 ``set_joints_position``
累加的 ``pos_cmd`` 是**本体系口径**。所以：

* Nav2 的 ``/cmd_vel`` 本身就是本体系 → 对应 202 模式，**直接积分，不旋转**；
* 只有上游给世界系速度时，才需要 203 的 ``rot_mat.T`` 先转回本体系。

搞反符号的后果是底盘反向跑，所以 ``input_frame`` 做成显式配置项而不是猜。

为什么这一层要做成纯函数
======================
底盘是**位置指令开环积分**：``set_joints_position`` 收的是位置，若轮子打滑，
指令位置会持续超前实际位置，而且没有任何反馈会阻止它 —— 误差单调累积，一旦
恢复附着力，底盘会以最大能力冲向那个跑飞的位置。这条链路上每一个数值都必须
可离线复现、可故障注入，所以积分/限幅/leash/校正全部剥离成无副作用函数。
"""

import collections
import math

IDX_X = 0
IDX_Y = 1
IDX_THETA = 2
CHASSIS_DOF_S1 = 3

FRAME_BODY = 'body'      # 202 模式：直接积分（Nav2 默认口径）
FRAME_WORLD = 'world'    # 203 模式：先 rot_mat.T 转回本体系
VALID_INPUT_FRAMES = (FRAME_BODY, FRAME_WORLD)


class ChassisConfigError(ValueError):
    """底盘配置非法。显式抛出，由调用方决定"拒绝启动"还是"回退保守值"，
    本模块不自行决定——静默回退会让一个配错的阈值表现成"保护好像没生效"。"""


def wrap_angle(theta):
    """把角度归一到 (-pi, pi]。

    theta 是累积积分量，不归一会让 ``theta`` 无界增长；而 leash 与外环校正都要
    算角度差，不归一时 179° 与 -179° 的差会算成 358° 而不是 2°，直接误触发。
    """
    return math.atan2(math.sin(theta), math.cos(theta))


def rot_z(theta):
    """绕 z 的 2x2 旋转矩阵（本体系 -> 世界系），以嵌套 tuple 返回。

    只返回 xy 的 2x2 部分：203 的 3x3 里第三行/列是 theta 的恒等映射，
    角速度不需要旋转，单独处理更不容易写错。
    """
    c, s = math.cos(theta), math.sin(theta)
    return ((c, -s), (s, c))


def rotate_vec2(mat2, vec2):
    """2x2 矩阵乘 2 向量。"""
    return (mat2[0][0] * vec2[0] + mat2[0][1] * vec2[1],
            mat2[1][0] * vec2[0] + mat2[1][1] * vec2[1])


def rotate_vec2_transposed(mat2, vec2):
    """用 mat2 的**转置**乘 2 向量，即 203:61 的 ``rot_mat.T @ vel``。

    单独提供这个函数而不是让调用方自己转置，是为了让"用了转置"这件事在调用点
    可见 —— 这个转置一旦漏掉或多加，底盘就反向跑，而代码看起来完全正常。
    """
    return (mat2[0][0] * vec2[0] + mat2[1][0] * vec2[1],
            mat2[0][1] * vec2[0] + mat2[1][1] * vec2[1])


def to_local_velocity(twist_xy_wz, input_frame, theta):
    """把上游速度指令换算成**本体系**速度（pos_cmd 的积分口径）。

    Args:
        twist_xy_wz: (vx, vy, wz)。含义取决于 input_frame。
        input_frame: FRAME_BODY -> 原样返回（202:47-49 直接积分）；
                     FRAME_WORLD -> 用 rot_mat.T 转回本体系（203:61）。
        theta: 当前底盘朝向（rad）。FRAME_BODY 时不使用。

    Returns:
        (vx_local, vy_local, wz)
    """
    if input_frame not in VALID_INPUT_FRAMES:
        raise ChassisConfigError(
            'input_frame=%r 非法，只能是 %s' % (input_frame, list(VALID_INPUT_FRAMES)))
    vx, vy, wz = twist_xy_wz
    if input_frame == FRAME_BODY:
        return (vx, vy, wz)
    local_xy = rotate_vec2_transposed(rot_z(theta), (vx, vy))
    return (local_xy[0], local_xy[1], wz)


def clamp_velocity(twist_xy_wz, max_vel_xy, max_vel_theta):
    """速度限幅。xy 按**合成模长**等比缩放，不是逐轴裁剪。

    逐轴裁剪会改变运动方向：(1.0, 1.0) 逐轴裁到 (1.0, 1.0) 模长是 1.41 超限，
    而裁成 (0.707, 0.707) 才是既满足限幅又保持方向。方向被改会让 Nav2 的
    路径跟踪出现它无法解释的横向偏差。
    """
    if max_vel_xy <= 0.0 or max_vel_theta <= 0.0:
        raise ChassisConfigError(
            'max_vel_xy=%r / max_vel_theta=%r 必须为正' % (max_vel_xy, max_vel_theta))
    vx, vy, wz = twist_xy_wz
    norm_xy = math.hypot(vx, vy)
    if norm_xy > max_vel_xy:
        scale = max_vel_xy / norm_xy
        vx, vy = vx * scale, vy * scale
    wz = max(-max_vel_theta, min(max_vel_theta, wz))
    return (vx, vy, wz)


def slew_limit_velocity(target, previous, max_accel_xy, max_accel_theta, dt,
                        max_accel_xy_up=None):
    """速度斜率限制（加速度限幅）。与 nav2 velocity_smoother 的 max_accel 取齐。

    没有这一层时，Nav2 一个突变的 Twist 会在一个积分周期内变成位置阶跃，
    底盘会猛冲。

    ═══════════ max_accel_xy_up：非对称限幅，只限**加速** ═══════════
    2026-09-08 实机：RPP 一进 FOLLOW 就要 0.5m/s，桥接按 max_accel_xy=2.5
    在 0.20s 内把指令拉到 0.5，而底盘真实加速度实测只有 ~0.39m/s²（由
    "0.0872m / 0.67s 从静止起"反解），要 1.29s 才到 0.5。
    **底盘是位置指令开环积分链**：加速段指令跑在实际前面积下的位置欠账，
    在随后的匀速段**永不归还**（指令与实际同步前进，差值不变），而 leash
    预算是总量制。纯加速暂态的欠账：

        Δ = v²/2 × (1/a_实际 − 1/a_指令) ≈ 1.09·v²
        v=0.5 → 0.272m  >  leash_xy_m 0.250m   ← 每次从静止起步都必然跳闸
        v=0.3 → 0.098m  =  39% 预算
        v=0.2 → 0.044m  =  17% 预算（与 MPPI 那次 leash 从未跳吻合）

    实测跳闸值 0.2509m（0.0124 + 0.2385 两窗累加）对 0.250m 阈值，吻合 0.4%。
    所以瓶颈**不是速度上限**，是指令加速度比底盘快 6.4 倍。

    **减速方向刻意不限**：减速时底盘因惯性反超指令，欠账是**缩小**的 ——
    同一份日志里实测过（旋转段 `指令 -0.4158 / 实际 -0.5484`）。而
    max_accel_xy 同时管刹车，一起压下去会把停车距离从实测的 0.069~0.100m
    拉长到 0.36m，那是拿一个真实的安全裕度换另一个。

    ``max_accel_xy_up=None`` 时退化为对称限幅（与本函数原行为一致）。

    ⚠ xy 按**二维矢量**限幅，不是逐轴。逐轴符号判据不是旋转不变的
    （本项目已在"越界判据"上踩过一次：横向分量能合法顶到
    sqrt(0.10²+0.05²)）。逐轴限幅在斜向上放行 sqrt(2) 倍的加速度，
    而"加速还是减速"只有对速度**模长**才有定义。
    模长不减（含等模长的方向变化）一律走 up 限幅，取保守侧。
    theta 是标量，仍按原样逐轴处理。
    """
    if dt <= 0.0:
        raise ChassisConfigError('dt=%r 必须为正' % (dt,))
    if max_accel_xy <= 0.0 or max_accel_theta <= 0.0:
        raise ChassisConfigError(
            'max_accel_xy=%r / max_accel_theta=%r 必须为正'
            % (max_accel_xy, max_accel_theta))
    if max_accel_xy_up is not None and max_accel_xy_up <= 0.0:
        raise ChassisConfigError(
            'max_accel_xy_up=%r 必须为正或 None' % (max_accel_xy_up,))

    dx = target[0] - previous[0]
    dy = target[1] - previous[1]
    a_xy = max_accel_xy
    if max_accel_xy_up is not None:
        speed_t = math.hypot(target[0], target[1])
        speed_p = math.hypot(previous[0], previous[1])
        if speed_t >= speed_p:
            a_xy = max_accel_xy_up
    max_delta_xy = a_xy * dt
    delta_norm = math.hypot(dx, dy)
    if delta_norm > max_delta_xy:
        scale = max_delta_xy / delta_norm
        dx *= scale
        dy *= scale

    dth = target[2] - previous[2]
    max_delta_th = max_accel_theta * dt
    if dth > max_delta_th:
        dth = max_delta_th
    elif dth < -max_delta_th:
        dth = -max_delta_th

    return (previous[0] + dx, previous[1] + dy, previous[2] + dth)


TickDt = collections.namedtuple('TickDt', 'dt clamped reason raw')


def measure_tick_dt(now, prev, nominal_dt, max_dt):
    """由相邻两拍的时间戳算积分步长，并做钳位。

    ════════════════ 为什么不能直接用 1/freq ════════════════
    底盘是**位置**接口：``pos_cmd`` 推进多快决定物理速度，而"推进多快"是
    每拍增量 × **墙钟**拍率。若每拍加 ``v/250`` 而实际只跑到 157Hz，
    底盘就只有指令速度的 157/250 = 63%。用实测 dt 后拍率快慢不再影响速度。

    ════════════════ 为什么必须钳位 ════════════════
    dt 直接乘在速度上，所以一次调度停顿会变成一次**位置阶跃**：
    1 m/s 下停顿 100ms 就是 0.1m 的跳变，底盘会以最大能力冲向那个位置。
    钳位把单拍位移的上界锁死在 ``max_vel * max_dt``，这个上界必须远小于
    leash 阈值，否则一次停顿就能把 leash 撞开。

    Args:
        now: 本拍时间戳（秒，单调）。
        prev: 上一拍时间戳（秒）；首拍传 ``None``。
        nominal_dt: 标称步长（``1/freq``），首拍与时钟异常时的兜底值。
        max_dt: 步长上限（秒）。

    Returns:
        :class:`TickDt`。

    Raises:
        ChassisConfigError: ``nominal_dt`` 或 ``max_dt`` 非正，或
            ``max_dt < nominal_dt``（那样连正常拍都会被钳，等于没修）。
    """
    if nominal_dt <= 0.0:
        raise ChassisConfigError('nominal_dt=%r 必须为正' % (nominal_dt,))
    if max_dt <= 0.0:
        raise ChassisConfigError('max_dt=%r 必须为正' % (max_dt,))
    if max_dt < nominal_dt:
        raise ChassisConfigError(
            'max_dt=%r 小于 nominal_dt=%r —— 那样连按标称频率跑的拍都会被钳位，'
            '积分恒等于钳位值，等于没修这个缺陷' % (max_dt, nominal_dt))

    if prev is None:
        return TickDt(nominal_dt, False, '', None)

    raw = now - prev
    if raw <= 0.0:
        return TickDt(nominal_dt, True,
                      '时钟未前进（raw=%.6fs），退回标称步长' % raw, raw)
    if raw > max_dt:
        return TickDt(max_dt, True,
                      '实测步长 %.4fs 超过上限 %.4fs，已钳位。'
                      '未钳位的话本拍会积出一次位置阶跃' % (raw, max_dt), raw)
    return TickDt(raw, False, '', raw)


def integrate_step_dt(pos_cmd, local_velocity, dt):
    """按**实测**步长积分一步：``pos_cmd[i] += v[i] * dt``。

    Args:
        pos_cmd: 当前指令位置 [x, y, theta]（会被复制，不原地修改）。
        local_velocity: 本体系速度 (vx, vy, wz)。
        dt: 步长（秒），必须为正。调用方应已用 :func:`measure_tick_dt` 钳位。

    Returns:
        新的 [x, y, theta]，theta 已归一。
    """
    if dt <= 0.0:
        raise ChassisConfigError('dt=%r 必须为正' % (dt,))
    if len(pos_cmd) != CHASSIS_DOF_S1:
        raise ChassisConfigError(
            'pos_cmd 长度=%d，期望 %d。注意 chassis_dof 由 ROBOT_TYPE 决定，'
            '未设置为 S1 时是 2（见 astribot_base.py:36-38）'
            % (len(pos_cmd), CHASSIS_DOF_S1))
    return [
        pos_cmd[IDX_X] + local_velocity[0] * dt,
        pos_cmd[IDX_Y] + local_velocity[1] * dt,
        wrap_angle(pos_cmd[IDX_THETA] + local_velocity[2] * dt),
    ]


def integrate_step(pos_cmd, local_velocity, freq):
    """一个积分步：``pos_cmd[i] += v[i] / freq``（202:47-49 / 203:63-65）。

    这是**标称频率**口径，等价于 ``integrate_step_dt(..., 1/freq)``。
    内环已改用 :func:`integrate_step_dt` + 实测 dt；本函数保留给
    "确知拍率就是标称值"的场合（examples 对齐、离线推演）。

    Args:
        pos_cmd: 当前指令位置 [x, y, theta]（会被复制，不原地修改）。
        local_velocity: 本体系速度 (vx, vy, wz)。
        freq: 控制频率（Hz）。examples 全部循环样例用 250.0。

    Returns:
        新的 [x, y, theta]，theta 已归一。
    """
    if freq <= 0.0:
        raise ChassisConfigError('freq=%r 必须为正' % (freq,))
    return integrate_step_dt(pos_cmd, local_velocity, 1.0 / freq)


def pose_error(pos_a, pos_b):
    """位姿差 a - b，返回 (dx, dy, dtheta)。dtheta 已按最短弧归一。"""
    return (pos_a[IDX_X] - pos_b[IDX_X],
            pos_a[IDX_Y] - pos_b[IDX_Y],
            wrap_angle(pos_a[IDX_THETA] - pos_b[IDX_THETA]))


def error_magnitude(err_xy_theta):
    """把位姿差拆成 (xy 模长, |dtheta|)。

    xy 与 theta **不合成成一个标量** —— 它们量纲不同（m vs rad），
    合成需要一个人为的权重，那个权重会变成一个说不清依据的魔数。
    分开返回，让 leash 用两个独立阈值判断。
    """
    return (math.hypot(err_xy_theta[0], err_xy_theta[1]), abs(err_xy_theta[2]))
