#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂桥接纯函数的单元测试。

限位顺序那组测试锁住的是一条**厂商样例互相矛盾**的事实：
``examples/100:49`` 与 ``examples/103:46`` 对 ``get_joints_position_limit()``
的返回值按相反顺序解包，而 ``astribot_client.py:141`` 的 Returns 明确是
(lower, upper) —— 也就是 **100 是样例 bug**。
"""

import math

import pytest

from astribot_trajectory_bridge.arm_traj_math import (
    ArmConfigError,
    INTERP_CUBIC,
    INTERP_LINEAR,
    assert_limit_order,
    check_time_monotonic,
    check_within_limits,
    cross_check_limits,
    drop_non_positive_time_points,
    interpolate_trajectory,
    is_settled,
    max_abs_error,
    reshape_waypoints,
)

# astribot_client.py:141 docstring 里 arm_left 的示例限位
LOWER_7 = [-3.01, -1.5, -3.14, 0.001, -2.4, -0.75, -1.57]
UPPER_7 = [3.01, 0.25, 3.14, 2.618, 2.4, 0.75, 1.57]


class TestLimitOrder:

    def test_correct_order_accepted(self):
        r = assert_limit_order(LOWER_7, UPPER_7)
        assert r.ok

    def test_reversed_order_rejected(self):
        # 把 100:49 的错误解包顺序喂进来，必须当场失败而不是静默工作
        r = assert_limit_order(UPPER_7, LOWER_7)
        assert not r.ok
        assert 'examples/100' in r.reason

    def test_single_reversed_joint_caught(self):
        lo = list(LOWER_7)
        up = list(UPPER_7)
        lo[3], up[3] = up[3], lo[3]     # 只反一个关节
        r = assert_limit_order(lo, up)
        assert not r.ok
        assert '关节 3' in r.reason

    def test_length_mismatch_rejected(self):
        r = assert_limit_order(LOWER_7, UPPER_7[:5])
        assert not r.ok

    def test_equal_bounds_allowed(self):
        # lower == upper 是"锁死的关节"，合法（不是错误顺序）
        r = assert_limit_order([1.0], [1.0])
        assert r.ok


class TestCrossCheckLimits:

    def test_identical_passes(self):
        ok, worst, _ = cross_check_limits(LOWER_7, UPPER_7, LOWER_7, UPPER_7, 1e-6)
        assert ok and worst == pytest.approx(0.0)

    def test_mismatch_detected(self):
        urdf_lower = list(LOWER_7)
        urdf_lower[2] -= 0.2
        ok, worst, desc = cross_check_limits(LOWER_7, UPPER_7,
                                             urdf_lower, UPPER_7, tol=0.01)
        assert not ok
        assert worst == pytest.approx(0.2)
        assert '关节 2' in desc

    def test_within_tolerance_passes(self):
        urdf_upper = list(UPPER_7)
        urdf_upper[0] += 0.005
        ok, _, _ = cross_check_limits(LOWER_7, UPPER_7, LOWER_7, urdf_upper, tol=0.01)
        assert ok

    def test_negative_tol_rejected(self):
        with pytest.raises(ArmConfigError):
            cross_check_limits(LOWER_7, UPPER_7, LOWER_7, UPPER_7, -0.1)


class TestWithinLimits:

    def test_inside_passes(self):
        q = [0.0, -0.5, 0.0, 1.0, 0.0, 0.0, 0.0]
        ok, _ = check_within_limits(q, LOWER_7, UPPER_7)
        assert ok

    def test_outside_rejected(self):
        q = [0.0, -0.5, 0.0, 3.0, 0.0, 0.0, 0.0]   # joint_4 上限 2.618
        ok, desc = check_within_limits(q, LOWER_7, UPPER_7)
        assert not ok and '关节 3' in desc

    def test_margin_shrinks_range(self):
        # joint_4 下限 0.001，给 0.05 margin 后 0.01 应被拒
        q = [0.0, -0.5, 0.0, 0.01, 0.0, 0.0, 0.0]
        ok, _ = check_within_limits(q, LOWER_7, UPPER_7, margin=0.0)
        assert ok
        ok, _ = check_within_limits(q, LOWER_7, UPPER_7, margin=0.05)
        assert not ok

    def test_excessive_margin_reported(self):
        ok, desc = check_within_limits([0.0], [0.0], [0.1], margin=1.0)
        assert not ok and 'margin' in desc

    def test_dim_mismatch_rejected(self):
        ok, _ = check_within_limits([0.0], LOWER_7, UPPER_7)
        assert not ok


class TestInterpolation:

    TIMES = [1.0, 2.0, 3.0]
    POS = [[0.0, 0.0], [1.0, -1.0], [2.0, -2.0]]

    def test_clamps_before_start(self):
        # 不外推：外推会在轨迹端点产生超出规划范围的指令
        out = interpolate_trajectory(self.TIMES, self.POS, None, 0.0, INTERP_LINEAR)
        assert out == pytest.approx(self.POS[0])

    def test_clamps_after_end(self):
        out = interpolate_trajectory(self.TIMES, self.POS, None, 99.0, INTERP_LINEAR)
        assert out == pytest.approx(self.POS[-1])

    def test_linear_midpoint(self):
        out = interpolate_trajectory(self.TIMES, self.POS, None, 1.5, INTERP_LINEAR)
        assert out == pytest.approx([0.5, -0.5])

    def test_hits_knot_points_exactly(self):
        out = interpolate_trajectory(self.TIMES, self.POS, None, 2.0, INTERP_LINEAR)
        assert out == pytest.approx(self.POS[1])

    def test_cubic_matches_endpoints(self):
        vel = [[0.0, 0.0], [1.0, -1.0], [0.0, 0.0]]
        a = interpolate_trajectory(self.TIMES, self.POS, vel, 1.0, INTERP_CUBIC)
        b = interpolate_trajectory(self.TIMES, self.POS, vel, 2.0, INTERP_CUBIC)
        assert a == pytest.approx(self.POS[0])
        assert b == pytest.approx(self.POS[1])

    def test_cubic_falls_back_without_velocities(self):
        out = interpolate_trajectory(self.TIMES, self.POS, None, 1.5, INTERP_CUBIC)
        assert out == pytest.approx([0.5, -0.5])   # 退化成线性

    def test_non_monotonic_times_rejected(self):
        with pytest.raises(ArmConfigError) as e:
            interpolate_trajectory([1.0, 1.0], [[0.0], [1.0]], None, 1.0)
        assert '单调' in str(e.value)

    def test_empty_rejected(self):
        with pytest.raises(ArmConfigError):
            interpolate_trajectory([], [], None, 0.0)

    def test_invalid_mode_rejected(self):
        with pytest.raises(ArmConfigError):
            interpolate_trajectory(self.TIMES, self.POS, None, 1.5, 'spline')


class TestSettle:

    def test_within_tolerance(self):
        assert is_settled([0.0, 0.0], [0.005, -0.005], 0.02)

    def test_outside_tolerance(self):
        assert not is_settled([0.0, 0.0], [0.05, 0.0], 0.02)

    def test_max_abs_error(self):
        assert max_abs_error([0.0, 0.0, 0.0], [0.1, -0.3, 0.05]) == pytest.approx(0.3)

    def test_zero_tolerance_rejected(self):
        with pytest.raises(ArmConfigError):
            is_settled([0.0], [0.0], 0.0)

    def test_dim_mismatch_rejected(self):
        with pytest.raises(ArmConfigError):
            max_abs_error([0.0], [0.0, 0.0])


class TestWaypointReshape:

    def test_reshape(self):
        out = reshape_waypoints([1, 2, 3, 4, 5, 6], dof=3)
        assert out == [[1, 2, 3], [4, 5, 6]]

    def test_bad_length_rejected(self):
        with pytest.raises(ArmConfigError):
            reshape_waypoints([1, 2, 3, 4], dof=3)

    def test_bad_dof_rejected(self):
        with pytest.raises(ArmConfigError):
            reshape_waypoints([1, 2], dof=0)


class TestDropT0:
    """examples/206 头部：t=0 由 SDK 用当前位置隐含，不要给 t=0 的点。"""

    def test_drops_zero_time_point(self):
        wp = [[0.0], [1.0], [2.0]]
        tl = [0.0, 1.0, 2.0]
        kept_wp, kept_t, dropped = drop_non_positive_time_points(wp, tl)
        assert dropped == 1
        assert kept_t == [1.0, 2.0]
        assert kept_wp == [[1.0], [2.0]]

    def test_drops_negative_time(self):
        _, _, dropped = drop_non_positive_time_points([[0.0]], [-0.5])
        assert dropped == 1

    def test_keeps_all_positive(self):
        wp = [[1.0], [2.0]]
        kept_wp, kept_t, dropped = drop_non_positive_time_points(wp, [1.0, 2.0])
        assert dropped == 0 and kept_wp == wp and kept_t == [1.0, 2.0]

    def test_length_mismatch_rejected(self):
        with pytest.raises(ArmConfigError):
            drop_non_positive_time_points([[0.0]], [1.0, 2.0])

    def test_all_dropped_leaves_empty(self):
        kept_wp, kept_t, dropped = drop_non_positive_time_points([[0.0]], [0.0])
        assert kept_wp == [] and kept_t == [] and dropped == 1


class TestTimeMonotonic:

    def test_increasing_ok(self):
        ok, _ = check_time_monotonic([1.0, 2.0, 3.0])
        assert ok

    def test_equal_rejected(self):
        ok, desc = check_time_monotonic([1.0, 1.0])
        assert not ok and 'time_list[1]' in desc

    def test_decreasing_rejected(self):
        ok, _ = check_time_monotonic([2.0, 1.0])
        assert not ok

    def test_single_point_ok(self):
        ok, _ = check_time_monotonic([1.0])
        assert ok
