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
``set_joints_position`` 接收位置目标。PoseFrameIntegrator 将定位反馈映射到 SDK
增量口径，在每个新 pose 到来时同时清零 x/y/yaw 的指令积分。转换函数便于离线复现；
SDK 位置误差检查由控制核心执行，速度与加减速限制由上层导航负责。
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
    """定位朝向和几何误差取最短角；SDK 位置目标不得调用此函数。"""
    return math.atan2(math.sin(theta), math.cos(theta))


class PoseFrameIntegrator:
    """SDK 目标 = 最新反馈基准 + 本帧指令积分，三个轴同时换帧。

    无前瞻模式累加 SLAM 的局部测量增量；有限前瞻模式由控制核心按 SLAM
    时间戳查询 SDK 历史位置，重设接口基准。两种模式均不保留上一帧未完成的
    指令积分。SDK 基准只服务于位置接口，导航到位判断仍使用定位源。
    """

    def __init__(self, pose, stamp, sdk_pose):
        self.stamp = stamp
        self.pose = list(pose)
        self.anchor = list(sdk_pose)
        self.integral = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]

    def observe(self, pose, stamp):
        if stamp <= self.stamp:
            return False
        dx, dy = local_pose_displacement(self.pose, pose)
        self.anchor = [self.anchor[IDX_X] + dx, self.anchor[IDX_Y] + dy,
                       self.anchor[IDX_THETA] + wrap_angle(pose[IDX_THETA] - self.pose[IDX_THETA])]
        self.stamp = stamp
        self.pose = list(pose)
        self.integral = [0.0, 0.0, 0.0]
        return True

    def reanchor_axis(self, axis, actual):
        """Set one axis' SDK interface reference and discard its frame integral."""
        self.anchor[axis] = actual
        self.integral[axis] = 0.0

    def target(self, velocity, dt):
        base = [a + delta for a, delta in zip(self.anchor, self.integral)]
        return integrate_step_dt(base, velocity, dt)

    def commit(self, velocity, dt):
        """只提交实际写入 SDK 的本拍增量。"""
        self.integral = [delta + v * dt for delta, v in zip(self.integral, velocity)]

    def preview_target(self, velocity, dt, times, xy_limit, theta_limit):
        """有限位置前瞻；限的是目标提前量，不改变上游速度指令。

        原地等待时不累积，换向时丢弃本帧同轴旧方向积分。
        上层负责在零速边沿以 SDK 实测位置撤掉前瞻，并保持该位置。
        """
        delta = [((old if v * prev > 0.0 else 0.0) + v * dt + v * h)
                 if v != 0.0 else 0.0
                 for old, v, prev, h in zip(self.integral, velocity, self.velocity, times)]
        xy = math.hypot(delta[0], delta[1])
        if xy > xy_limit:
            delta[0] *= xy_limit / xy
            delta[1] *= xy_limit / xy
        delta[2] = max(-theta_limit, min(theta_limit, delta[2]))
        result = [base + d for base, d in zip(self.anchor, delta)]
        return result

    def commit_preview(self, velocity, dt):
        self.integral = [(old if v * prev > 0.0 else 0.0) + v * dt
                         for old, v, prev in zip(self.integral, velocity, self.velocity)]
        self.velocity = list(velocity)


class SdkPoseHistory:
    """SDK read-time samples aligned to localization time, without accumulating frame bias."""

    def __init__(self, duration):
        self.duration = duration
        self.samples = collections.deque()

    def append(self, stamp, pose):
        if self.samples and stamp < self.samples[-1][0]:
            self.samples.clear()
        if self.samples and stamp == self.samples[-1][0]:
            self.samples.pop()
        self.samples.append((stamp, list(pose)))
        while len(self.samples) > 2 and self.samples[1][0] < stamp-self.duration:
            self.samples.popleft()

    def at(self, stamp):
        if not self.samples:
            raise ValueError('SDK pose history is empty')
        if stamp <= self.samples[0][0]:
            return list(self.samples[0][1])
        for (ta, a), (tb, b) in zip(self.samples, list(self.samples)[1:]):
            if stamp <= tb:
                ratio = (stamp-ta)/(tb-ta)
                # SDK theta is a continuous position coordinate, including its turn count.
                return [a[i]+ratio*(b[i]-a[i]) for i in range(CHASSIS_DOF_S1)]
        return list(self.samples[-1][1])


def local_pose_displacement(previous, current):
    """SLAM 帧间位移转成 SDK 示例使用的本体行程；转弯按 SE(2) 对数映射。

    使用最短 yaw 差，假定相邻定位帧的真实转角小于 pi。sinc 补偿避免将圆弧
    的弦长当作本体行程；不能恢复两帧间未被测量的复杂运动。
    """
    turn = wrap_angle(current[IDX_THETA] - previous[IDX_THETA])
    mid_yaw = previous[IDX_THETA] + 0.5 * turn
    chord = (current[IDX_X] - previous[IDX_X], current[IDX_Y] - previous[IDX_Y])
    dx, dy = rotate_vec2_transposed(rot_z(mid_yaw), chord)
    half = 0.5 * turn
    scale = half / math.sin(half) if abs(half) > 1e-6 else 1.0 + half * half / 6.0
    return dx * scale, dy * scale


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
        新的 [x, y, theta]，theta 保留 SDK 连续角度分支。
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
        pos_cmd[IDX_THETA] + local_velocity[2] * dt,
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
