#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘积分核心的单元测试（不需要 SDK、不需要 rclpy）。

这些测试锁住的是**接口事实**，不只是代码行为。凡断言里出现具体数值的，
都能回溯到 examples 源码的某一行，注释里标了出处。
"""

import math

import pytest

from astribot_trajectory_bridge.chassis_integrator import (
    CHASSIS_DOF_S1,
    ChassisConfigError,
    FRAME_BODY,
    FRAME_WORLD,
    clamp_velocity,
    error_magnitude,
    integrate_step,
    pose_error,
    rot_z,
    rotate_vec2,
    rotate_vec2_transposed,
    slew_limit_velocity,
    to_local_velocity,
    wrap_angle,
)


class TestWrapAngle:

    def test_identity_in_range(self):
        assert wrap_angle(0.0) == pytest.approx(0.0)
        assert wrap_angle(1.0) == pytest.approx(1.0)

    def test_wraps_beyond_pi(self):
        assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
        assert wrap_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)

    def test_large_accumulation(self):
        # theta 是累积积分量，不归一会无界增长
        assert wrap_angle(10 * math.pi + 0.3) == pytest.approx(0.3, abs=1e-9)

    def test_near_180_difference_is_small(self):
        # 不归一时 179° 与 -179° 的差会算成 358°，直接误触发 leash
        d = wrap_angle(math.radians(179) - math.radians(-179))
        assert abs(d) == pytest.approx(math.radians(2), abs=1e-9)


class TestRotation:

    def test_rot_z_90deg(self):
        m = rot_z(math.pi / 2)
        # 本体系 +x 转到世界系 +y
        out = rotate_vec2(m, (1.0, 0.0))
        assert out[0] == pytest.approx(0.0, abs=1e-12)
        assert out[1] == pytest.approx(1.0)

    def test_transpose_is_inverse(self):
        m = rot_z(0.7)
        v = (0.3, -0.8)
        back = rotate_vec2_transposed(m, rotate_vec2(m, v))
        assert back[0] == pytest.approx(v[0])
        assert back[1] == pytest.approx(v[1])

    def test_transposed_differs_from_plain(self):
        # 这两个搞混就是底盘反向跑的直接原因，必须能区分
        m = rot_z(math.pi / 2)
        v = (1.0, 0.0)
        assert rotate_vec2(m, v) != pytest.approx(rotate_vec2_transposed(m, v))


class TestToLocalVelocity:
    """锁死 202/203 两种口径。这是最容易搞反、搞反后底盘反向跑的一处。"""

    def test_body_frame_passthrough(self):
        # 202:47-49 —— 直接积分，不旋转
        v = (0.3, -0.2, 0.5)
        out = to_local_velocity(v, FRAME_BODY, theta=1.234)
        assert out == pytest.approx(v)

    def test_body_frame_ignores_theta(self):
        v = (0.3, -0.2, 0.5)
        a = to_local_velocity(v, FRAME_BODY, theta=0.0)
        b = to_local_velocity(v, FRAME_BODY, theta=2.0)
        assert a == pytest.approx(b)

    def test_world_frame_uses_transpose(self):
        # 203:61 —— rot_mat.T @ vel，方向是 global -> local
        # theta=90°：世界系 +x 的速度，在本体系里应该是 -y
        out = to_local_velocity((1.0, 0.0, 0.0), FRAME_WORLD, theta=math.pi / 2)
        assert out[0] == pytest.approx(0.0, abs=1e-12)
        assert out[1] == pytest.approx(-1.0)
        assert out[2] == pytest.approx(0.0)

    def test_world_frame_theta_passthrough(self):
        # 角速度不受平面旋转影响
        out = to_local_velocity((0.0, 0.0, 0.7), FRAME_WORLD, theta=1.1)
        assert out[2] == pytest.approx(0.7)

    def test_world_at_zero_theta_equals_body(self):
        v = (0.4, 0.1, -0.2)
        assert to_local_velocity(v, FRAME_WORLD, 0.0) == pytest.approx(
            to_local_velocity(v, FRAME_BODY, 0.0))

    def test_invalid_frame_rejected(self):
        with pytest.raises(ChassisConfigError):
            to_local_velocity((0, 0, 0), 'global', 0.0)


class TestClampVelocity:

    def test_within_limit_untouched(self):
        v = (0.3, 0.4, 0.5)
        assert clamp_velocity(v, 1.0, 2.0) == pytest.approx(v)

    def test_xy_scaled_by_norm_not_per_axis(self):
        # 逐轴裁剪会改变运动方向，让 Nav2 看到无法解释的横向偏差
        out = clamp_velocity((1.0, 1.0, 0.0), max_vel_xy=1.0, max_vel_theta=2.0)
        assert math.hypot(out[0], out[1]) == pytest.approx(1.0)
        # 方向必须保持 45°
        assert out[0] == pytest.approx(out[1])

    def test_theta_clamped(self):
        out = clamp_velocity((0.0, 0.0, 5.0), 1.0, 2.0)
        assert out[2] == pytest.approx(2.0)
        out = clamp_velocity((0.0, 0.0, -5.0), 1.0, 2.0)
        assert out[2] == pytest.approx(-2.0)

    def test_non_positive_limits_rejected(self):
        with pytest.raises(ChassisConfigError):
            clamp_velocity((0, 0, 0), 0.0, 2.0)
        with pytest.raises(ChassisConfigError):
            clamp_velocity((0, 0, 0), 1.0, -1.0)


class TestSlewLimit:

    def test_step_is_limited(self):
        # 0 -> 1.0 m/s，加速度上限 2.5，dt=0.004 -> 单步最多 0.01
        out = slew_limit_velocity((1.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                  max_accel_xy=2.5, max_accel_theta=3.2, dt=0.004)
        assert out[0] == pytest.approx(0.01)

    def test_deceleration_also_limited(self):
        out = slew_limit_velocity((0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
                                  2.5, 3.2, 0.004)
        assert out[0] == pytest.approx(1.0 - 0.01)

    def test_small_change_passes_through(self):
        out = slew_limit_velocity((0.001, 0.0, 0.0), (0.0, 0.0, 0.0),
                                  2.5, 3.2, 0.004)
        assert out[0] == pytest.approx(0.001)

    def test_bad_dt_rejected(self):
        with pytest.raises(ChassisConfigError):
            slew_limit_velocity((0, 0, 0), (0, 0, 0), 2.5, 3.2, 0.0)


class TestIntegrateStep:
    """锁死 202:47-49 的积分语义：pos_cmd[i] += v[i] / freq"""

    def test_single_step(self):
        out = integrate_step([0.0, 0.0, 0.0], (1.0, 0.0, 0.0), freq=250.0)
        assert out[0] == pytest.approx(1.0 / 250.0)

    def test_250_steps_equals_one_meter(self):
        # 1 m/s @ 250Hz，250 步后位移 1.000 m
        pos = [0.0, 0.0, 0.0]
        for _ in range(250):
            pos = integrate_step(pos, (1.0, 0.0, 0.0), 250.0)
        assert pos[0] == pytest.approx(1.0, abs=1e-9)

    def test_does_not_mutate_input(self):
        pos = [0.0, 0.0, 0.0]
        integrate_step(pos, (1.0, 1.0, 1.0), 250.0)
        assert pos == [0.0, 0.0, 0.0]

    def test_theta_is_wrapped(self):
        pos = [0.0, 0.0, math.pi - 0.001]
        out = integrate_step(pos, (0.0, 0.0, 1.0), freq=250.0)
        assert out[2] <= math.pi

    def test_wrong_dof_rejected(self):
        # chassis_dof 由 ROBOT_TYPE 决定，未设 S1 时是 2（astribot_base.py:36-38）
        with pytest.raises(ChassisConfigError) as e:
            integrate_step([0.0, 0.0], (1.0, 0.0, 0.0), 250.0)
        assert 'ROBOT_TYPE' in str(e.value)

    def test_bad_freq_rejected(self):
        with pytest.raises(ChassisConfigError):
            integrate_step([0.0, 0.0, 0.0], (1.0, 0.0, 0.0), 0.0)

    def test_dof_constant_is_three_for_s1(self):
        assert CHASSIS_DOF_S1 == 3


class TestPoseError:

    def test_plain_difference(self):
        err = pose_error([1.0, 2.0, 0.5], [0.4, 0.5, 0.2])
        assert err[0] == pytest.approx(0.6)
        assert err[1] == pytest.approx(1.5)
        assert err[2] == pytest.approx(0.3)

    def test_theta_shortest_arc(self):
        err = pose_error([0.0, 0.0, math.radians(179)],
                         [0.0, 0.0, math.radians(-179)])
        assert abs(err[2]) == pytest.approx(math.radians(2), abs=1e-9)

    def test_magnitude_keeps_units_separate(self):
        # xy(m) 与 theta(rad) 不合成成一个标量：合成需要一个说不清依据的权重
        xy, th = error_magnitude((3.0, 4.0, 0.7))
        assert xy == pytest.approx(5.0)
        assert th == pytest.approx(0.7)
