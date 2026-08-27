#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂桥接的纯函数核心：限位自检、轨迹插值、收敛判据、路点整形。

接口事实来源
============
* ``astribot_client.py:132-152`` —— ``get_joints_position_limit()`` 的 Returns
  明确写着 *"the first list is the lower limit and the second list is the upper
  limit"*，即返回顺序是 **(lower, upper)**。

  !!! ``examples/100-get_robot_properties.py:49`` 是厂商样例的 bug !!!
  它写的是 ``upper_limit, lower_limit = astribot.get_joints_position_limit()``，
  顺序反了。``examples/103-move_to_joint_position.py:46`` 才是对的
  （``lower_limit, upper_limit = ...``）。本模块以 client 的 docstring +103 为准。

  同一段 docstring 的 Example 还佐证了夹爪量纲：limit 是 ``[0.0] ~ [100.0]``，
  与 103:34 / 106:38-39 的"100=完全闭合，0=完全张开"一致。

* ``examples/105-set_joint_position_direct.py:53`` —— 流式下发（方案 B，默认）::

      astribot.set_joints_position(names, command_list,
                                  control_way="direct", use_wbc=False)

  105 头部说明 ``control_way="direct", use_wbc=False`` 时
  *"the user's desired value will be directly sent to the robot joints without
  any processing"*。

* ``examples/206-move_joints_waypoints.py`` —— 整段下发（方案 A，保留接口）。
  头部说明 *"No need to give a point at time 0, the default time is 0 at the
  current position"*，所以 t<=0 的点必须丢弃。

* ``astribot_client.py:704`` —— ``set_joints_position(..., add_default_torso=True)``
  **默认值是 True**。只给 arm_left 也会隐式附加躯干默认位姿 → 躯干会动。
  MoveIt 规划的是 arm_left 组、躯干由规划器另行管理，所以桥接必须显式传 False。
"""

import math


class ArmConfigError(ValueError):
    """机械臂桥接配置或输入非法。"""


class LimitCheckResult:
    """限位自检结果。"""

    def __init__(self, ok, lower, upper, reason=''):
        self.ok = ok
        self.lower = lower
        self.upper = upper
        self.reason = reason


def assert_limit_order(lower, upper):
    """启动期断言：逐关节 ``lower <= upper``。

    这里刻意**不做** min/max 归一容错。归一虽然能兜住返回顺序变更，但会**掩盖**
    SDK 行为变化 —— 而"限位方向反了"的后果是把所有合法目标判为越界（或反之把
    越界判为合法），必须当场失败而不是悄悄工作。

    返回 LimitCheckResult 而不是直接抛异常，让调用方决定是"拒绝启动"还是"上报状态"。
    """
    if len(lower) != len(upper):
        return LimitCheckResult(
            False, lower, upper,
            'lower 长度 %d != upper 长度 %d' % (len(lower), len(upper)))
    for j, (lo, hi) in enumerate(zip(lower, upper)):
        if lo > hi:
            return LimitCheckResult(
                False, lower, upper,
                '关节 %d 的 lower(%.6f) > upper(%.6f)，SDK 返回顺序与 '
                'astribot_client.py:141 的约定 (lower, upper) 不符。'
                '注意 examples/100:49 的解包顺序是反的，不要以它为准。'
                % (j, lo, hi))
    return LimitCheckResult(True, lower, upper)


def cross_check_limits(sdk_lower, sdk_upper, urdf_lower, urdf_upper, tol):
    """SDK 运行时限位与 URDF 派生限位的交叉核对。

    为什么要做这一步：本项目已实测"厂商 ship 了四份互不一致的模型"，真值源定为
    per-part yaml 的 model 字段、URDF 由它派生。SDK 运行时限位与 URDF 对不上，
    说明真值源没贯通到某一环，应当当场发现而不是等到规划出越界轨迹。

    返回 (是否一致, 最大偏差, 说明)。
    """
    if tol < 0.0:
        raise ArmConfigError('tol=%r 不能为负' % (tol,))
    n = min(len(sdk_lower), len(urdf_lower))
    worst = 0.0
    worst_desc = ''
    for j in range(n):
        for kind, a, b in (('lower', sdk_lower[j], urdf_lower[j]),
                           ('upper', sdk_upper[j], urdf_upper[j])):
            d = abs(a - b)
            if d > worst:
                worst = d
                worst_desc = ('关节 %d 的 %s 限位：SDK=%.6f URDF=%.6f 偏差=%.6f'
                              % (j, kind, a, b, d))
    if worst > tol:
        return (False, worst, worst_desc + '（超过容差 %.6f）' % tol)
    return (True, worst, '')


def check_within_limits(positions, lower, upper, margin=0.0):
    """逐关节限位校验。返回 (是否合法, 首个越限的描述)。

    margin 让调用方可以留安全余量：margin>0 时要求 ``lower+margin <= q <=
    upper-margin``，避免规划到限位边缘（本项目已知 joint_4=0 是奇异位形，
    且 SDK docstring 的示例里 joint_4 下限就是 0.001 这种贴边值）。
    """
    if len(positions) != len(lower) or len(positions) != len(upper):
        return (False, '位置维度 %d 与限位维度 (%d, %d) 不符'
                % (len(positions), len(lower), len(upper)))
    for j, q in enumerate(positions):
        lo = lower[j] + margin
        hi = upper[j] - margin
        if lo > hi:
            return (False, '关节 %d 的 margin=%.4f 过大，可行区间为空' % (j, margin))
        if q < lo or q > hi:
            return (False, '关节 %d 目标 %.6f 越界 [%.6f, %.6f]（含 margin %.4f）'
                    % (j, q, lo, hi, margin))
    return (True, '')


# ---------------------------------------------------------------------------
# 轨迹插值（方案 B 流式下发用）
# ---------------------------------------------------------------------------

INTERP_LINEAR = 'linear'
INTERP_CUBIC = 'cubic'
VALID_INTERP = (INTERP_LINEAR, INTERP_CUBIC)


def _lerp(a, b, s):
    return a + (b - a) * s


def _cubic_hermite(p0, v0, p1, v1, s, seg_dt):
    """三次 Hermite 插值。s in [0,1]，seg_dt 是该段时长（秒）。

    用 Hermite 而不是简单三次样条：MoveIt 的 JointTrajectoryPoint 自带
    velocities，直接用它做端点切线即可，不需要再解一遍样条系数。
    velocities 为空时退化成线性（由调用方传 v=0 实现）。
    """
    h00 = 2 * s ** 3 - 3 * s ** 2 + 1
    h10 = s ** 3 - 2 * s ** 2 + s
    h01 = -2 * s ** 3 + 3 * s ** 2
    h11 = s ** 3 - s ** 2
    return (h00 * p0 + h10 * seg_dt * v0 + h01 * p1 + h11 * seg_dt * v1)


def interpolate_trajectory(times, positions, velocities, t, mode=INTERP_CUBIC):
    """按时刻 t 在轨迹上取一个关节向量。

    Args:
        times: 各点的 time_from_start（秒），必须单调递增。
        positions: List[List[float]]，与 times 等长。
        velocities: List[List[float]] 或 None。None 时按线性处理。
        t: 查询时刻（秒），相对轨迹起点。
        mode: linear | cubic。

    Returns:
        插值后的关节向量（list）。

    边界行为：t<=times[0] 返回首点，t>=times[-1] 返回末点（钳位，不外推）。
    外推会在轨迹末尾产生超出规划范围的指令，这是执行期最危险的一类越界。
    """
    if mode not in VALID_INTERP:
        raise ArmConfigError('interp mode=%r 非法，只能是 %s' % (mode, list(VALID_INTERP)))
    if not times or not positions:
        raise ArmConfigError('空轨迹无法插值')
    if len(times) != len(positions):
        raise ArmConfigError('times 长度 %d != positions 长度 %d'
                             % (len(times), len(positions)))
    for i in range(1, len(times)):
        if times[i] <= times[i - 1]:
            raise ArmConfigError(
                'times 非单调递增：times[%d]=%.6f <= times[%d]=%.6f'
                % (i, times[i], i - 1, times[i - 1]))

    if t <= times[0]:
        return list(positions[0])
    if t >= times[-1]:
        return list(positions[-1])

    # 定位区间
    hi = 0
    for i in range(1, len(times)):
        if t <= times[i]:
            hi = i
            break
    lo = hi - 1
    seg_dt = times[hi] - times[lo]
    s = (t - times[lo]) / seg_dt

    out = []
    use_cubic = (mode == INTERP_CUBIC and velocities is not None
                 and len(velocities) == len(positions)
                 and velocities[lo] and velocities[hi])
    for j in range(len(positions[lo])):
        if use_cubic:
            out.append(_cubic_hermite(positions[lo][j], velocities[lo][j],
                                      positions[hi][j], velocities[hi][j],
                                      s, seg_dt))
        else:
            out.append(_lerp(positions[lo][j], positions[hi][j], s))
    return out


# ---------------------------------------------------------------------------
# 收敛判据
# ---------------------------------------------------------------------------

def max_abs_error(a, b):
    """两个关节向量的最大逐元素绝对差。"""
    if len(a) != len(b):
        raise ArmConfigError('维度不符：%d vs %d' % (len(a), len(b)))
    worst = 0.0
    for x, y in zip(a, b):
        worst = max(worst, abs(x - y))
    return worst


def is_settled(actual, target, tolerance):
    """是否已收敛到目标容差内。

    为什么需要这个判据而不是"发完最后一点就算成功"
    ------------------------------------------
    本项目已实测：控制器报完成时手臂还在收敛，下一步规划拿到移动中的起点，
    0.53s 后下发就报 ``start point deviates``（0.066 > 0.05），
    而上层只看到 ``MoveItErrorCode=-4``，完全看不出真实原因。
    所以流式下发发完末点后必须再等实际位置收敛。
    """
    if tolerance <= 0.0:
        raise ArmConfigError('tolerance=%r 必须为正' % (tolerance,))
    return max_abs_error(actual, target) <= tolerance


# ---------------------------------------------------------------------------
# 方案 A 的路点整形（保留接口）
# ---------------------------------------------------------------------------

def reshape_waypoints(flat, dof):
    """把展平数组还原成 List[List[float]]。ROS2 IDL 不支持变长嵌套序列，
    所以 DispatchWaypoints.srv 里用展平传输。"""
    if dof <= 0:
        raise ArmConfigError('dof=%r 必须为正' % (dof,))
    if len(flat) % dof != 0:
        raise ArmConfigError(
            'flat_waypoints 长度 %d 不是 dof=%d 的整数倍' % (len(flat), dof))
    return [list(flat[i:i + dof]) for i in range(0, len(flat), dof)]


def drop_non_positive_time_points(waypoints, time_list):
    """丢弃 t<=0 的路点，返回 (剩余路点, 剩余时刻, 丢弃数量)。

    ``examples/206`` 头部：*"No need to give a point at time 0, the default time
    is 0 at the current position"* —— t=0 由 SDK 用当前位置隐含。
    MoveIt 的轨迹首点 time_from_start 通常正好是 0，直接透传会与 SDK 的隐含点
    冲突（同一时刻两个不同位置）。
    """
    if len(waypoints) != len(time_list):
        raise ArmConfigError(
            '路点数 %d 与 time_list 长度 %d 不符' % (len(waypoints), len(time_list)))
    kept_wp, kept_t, dropped = [], [], 0
    for wp, t in zip(waypoints, time_list):
        if t <= 0.0:
            dropped += 1
            continue
        kept_wp.append(wp)
        kept_t.append(t)
    return (kept_wp, kept_t, dropped)


def check_time_monotonic(time_list):
    """time_list 必须严格单调递增。返回 (是否合法, 说明)。"""
    for i in range(1, len(time_list)):
        if time_list[i] <= time_list[i - 1]:
            return (False, 'time_list[%d]=%.6f <= time_list[%d]=%.6f'
                    % (i, time_list[i], i - 1, time_list[i - 1]))
    return (True, '')
