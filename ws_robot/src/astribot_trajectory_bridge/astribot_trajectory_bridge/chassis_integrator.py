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

# 三个自由度的下标（ROBOT_TYPE=S1 时 chassis_dof=3，见 astribot_base.py:36-38）
IDX_X = 0
IDX_Y = 1
IDX_THETA = 2
CHASSIS_DOF_S1 = 3

# input_frame 合法取值
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
    # FRAME_WORLD：203:58-61 —— rot_mat 是 local->global，用它的转置把
    # 世界系速度转成本体系速度。角速度不受平面旋转影响，原样透传。
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


def slew_limit_velocity(target, previous, max_accel_xy, max_accel_theta, dt):
    """速度斜率限制（加速度限幅）。与 nav2 velocity_smoother 的 max_accel 取齐。

    没有这一层时，Nav2 一个突变的 Twist 会在一个积分周期内变成位置阶跃，
    底盘会猛冲。
    """
    if dt <= 0.0:
        raise ChassisConfigError('dt=%r 必须为正' % (dt,))
    if max_accel_xy <= 0.0 or max_accel_theta <= 0.0:
        raise ChassisConfigError(
            'max_accel_xy=%r / max_accel_theta=%r 必须为正'
            % (max_accel_xy, max_accel_theta))
    out = []
    limits = (max_accel_xy, max_accel_xy, max_accel_theta)
    for i in range(3):
        delta = target[i] - previous[i]
        max_delta = limits[i] * dt
        if delta > max_delta:
            delta = max_delta
        elif delta < -max_delta:
            delta = -max_delta
        out.append(previous[i] + delta)
    return tuple(out)


#: :func:`measure_tick_dt` 的返回值。
#:
#: - ``dt``：本拍实际该用的积分步长（秒），已钳位。
#: - ``clamped``：是否被钳位。钳位说明调度出了状况，必须上报而不是静默吞掉。
#: - ``reason``：钳位原因，空串表示未钳位。
#: - ``raw``：钳位前的原始测量值，用于诊断（首拍为 ``None``）。
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
        # 首拍没有参照，用标称值。不算钳位（不是异常）。
        return TickDt(nominal_dt, False, '', None)

    raw = now - prev
    if raw <= 0.0:
        # 时钟没前进或倒退。rclpy 定时器积压时会背靠背触发，dt≈0；
        # 用 0 积分等于这一拍白丢，用标称值至少保持速度连续。
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
