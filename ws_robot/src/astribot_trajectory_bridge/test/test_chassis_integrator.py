#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘积分核心的单元测试（不需要 SDK、不需要 rclpy）。

这些测试锁住的是**接口事实**，不只是代码行为。凡断言里出现具体数值的，
都能回溯到 examples 源码的某一行，注释里标了出处。
"""

import math
import os

import pytest

from astribot_trajectory_bridge.chassis_bridge_core import ChassisBridgeConfig
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


class TestAsymmetricSlewLimit:
    """`max_accel_xy_up`：只限加速、不限减速。

    ═══════════ 这一组守卫的是什么（2026-09-08 实机停车）═══════════
    底盘是位置指令开环积分链，加速段"指令跑在实际前面"积下的位置欠账在
    随后的匀速段**永不归还**（指令与实际同步前进，差值不变），而 leash
    预算是总量制。实机实测：桥接按 max_accel_xy=2.5 在 0.20s 把指令拉到
    0.5m/s，底盘真实加速度只有 ~0.39m/s²（需 1.29s），暂态欠账 0.272m
    **必然**撞开 leash_xy_m=0.25 —— 实测 0.2509m，机器人闩锁不动。

    减速方向刻意不限：减速时底盘因惯性反超指令，欠账是**缩小**的
    （同一份日志实测 `指令 -0.4158 / 实际 -0.5484`）。而 max_accel_xy
    同时管刹车，一起压下去会把停车距离从实测 0.069~0.100m 拉长到 0.36m,
    那是拿一个真实的安全裕度换另一个。
    """

    def test_acceleration_uses_the_tighter_limit(self):
        out = slew_limit_velocity((0.5, 0.0, 0.0), (0.0, 0.0, 0.0),
                                  2.5, 3.2, 0.1, 0.35)
        # 0.35 * 0.1 = 0.035，而对称限幅会放行 0.25
        assert out[0] == pytest.approx(0.035)

    def test_deceleration_is_not_limited_by_the_up_value(self):
        """最关键的一条：刹车必须仍按 max_accel_xy 放行。"""
        out = slew_limit_velocity((0.0, 0.0, 0.0), (0.5, 0.0, 0.0),
                                  2.5, 3.2, 0.1, 0.35)
        # 2.5 * 0.1 = 0.25，够从 0.5 减到 0.25；若误用 0.35 则只能减到 0.465
        assert out[0] == pytest.approx(0.25)

    def test_none_means_symmetric_old_behavior(self):
        accel = slew_limit_velocity((1.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                    2.5, 3.2, 0.1, None)
        decel = slew_limit_velocity((0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
                                    2.5, 3.2, 0.1, None)
        assert accel[0] == pytest.approx(0.25)
        assert decel[0] == pytest.approx(0.75)

    def test_up_limit_is_rotation_invariant(self):
        """斜向加速不得放行 sqrt(2) 倍 —— 逐轴限幅会。

        本项目已在"越界判据"上踩过一次逐轴符号判据的坑：
        "加速还是减速"只对速度**模长**才有定义。
        """
        out = slew_limit_velocity((1.0, 1.0, 0.0), (0.0, 0.0, 0.0),
                                  2.5, 3.2, 0.1, 0.35)
        assert math.hypot(out[0], out[1]) == pytest.approx(0.035)

    def test_direction_change_at_constant_speed_takes_the_up_limit(self):
        """等模长的方向变化走保守侧（模长不减即用 up 限幅）。"""
        out = slew_limit_velocity((0.0, 0.5, 0.0), (0.5, 0.0, 0.0),
                                  2.5, 3.2, 0.1, 0.35)
        dx = out[0] - 0.5
        dy = out[1] - 0.0
        assert math.hypot(dx, dy) == pytest.approx(0.035)

    def test_theta_is_untouched_by_the_xy_up_limit(self):
        """theta 刻意不做非对称：实机跳闸时它只用掉 15% 预算。"""
        out = slew_limit_velocity((0.0, 0.0, 2.0), (0.0, 0.0, 0.0),
                                  2.5, 3.2, 0.1, 0.35)
        assert out[2] == pytest.approx(0.32)

    def test_nonpositive_up_value_rejected(self):
        with pytest.raises(ChassisConfigError):
            slew_limit_velocity((0, 0, 0), (0, 0, 0), 2.5, 3.2, 0.1, 0.0)

    def test_transient_deficit_arithmetic_matches_the_hardware_trip(self):
        """把实机那次的算术钉住：0.5m/s 在 2.5m/s² 下必然撞开 0.25m 预算。

        欠账 = v²/2 × (1/a_实际 − 1/a_指令)。这条不是在测代码，是在钉住
        "为什么必须有这个参数"的定量依据 —— 数字被人改动时它会红。
        """
        v, a_real, a_cmd, budget = 0.5, 0.39, 2.5, 0.25
        deficit = v * v / 2.0 * (1.0 / a_real - 1.0 / a_cmd)
        assert deficit > budget          # 0.272 > 0.250 → 必然跳闸
        # 换成 up 限幅后指令不再快于底盘，暂态欠账消失
        deficit_fixed = v * v / 2.0 * max(0.0, 1.0 / a_real - 1.0 / 0.35)
        assert deficit_fixed == pytest.approx(0.0)


class TestAsymmetricSlewWiring:
    """yaml -> 节点声明 -> 核心层，这条链任一处断掉都要红。

    本项目反复吃过"yaml 键静默失效"的亏（RewrittenYaml 只改已存在的键、
    节点名 remap 让整份 yaml 匹配不上、launch_arguments 白名单漏项……），
    共同点都是**不报错、不告警、回落到代码默认值**。所以"值在 yaml 里"
    和"值真的到了用它的那一层"必须分开验。
    """

    YAML = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'config', 'chassis_bridge.yaml')

    def _yaml_value(self):
        import re
        with open(self.YAML, encoding='utf-8') as fh:
            for line in fh:
                stripped = line.strip()
                if stripped.startswith('#'):
                    continue          # 必须先剥注释，否则会命中说明文字
                m = re.match(r'^max_accel_xy_up:\s*([0-9.]+)', stripped)
                if m:
                    return float(m.group(1))
        return None

    def test_yaml_declares_the_measured_value(self):
        val = self._yaml_value()
        assert val is not None, 'chassis_bridge.yaml 里找不到 max_accel_xy_up'
        # 必须严格小于 max_accel_xy=2.5，否则这个参数没有意义
        assert 0.0 < val < 2.5
        # 且必须不高于实测的底盘真实加速度 0.39 —— 高于它就等于没限住
        assert val <= 0.39

    def test_node_declares_the_parameter(self):
        """节点没声明这个参数时，yaml 里那一行会被 rclcpp/rclpy 静默忽略。"""
        src = os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), 'astribot_trajectory_bridge',
            'chassis_cmd_bridge_node.py')
        with open(src, encoding='utf-8') as fh:
            text = fh.read()
        assert "d('max_accel_xy_up'" in text, '节点未声明该参数'
        assert 'max_accel_xy_up=' in text, '节点未把该参数传给核心层'

    def test_core_actually_applies_the_yaml_value(self):
        """端到端：把 yaml 的值喂进 config，核心层的限幅必须真的按它走。

        只断言"字段等于那个值"是在验证我自己抄对了赋值语句；
        必须比对**限幅结果**。
        """
        val = self._yaml_value()
        cfg = ChassisBridgeConfig(max_accel_xy_up=val)
        out = slew_limit_velocity((0.5, 0.0, 0.0), (0.0, 0.0, 0.0),
                                  cfg.max_accel_xy, cfg.max_accel_theta,
                                  0.1, cfg.max_accel_xy_up)
        assert out[0] == pytest.approx(val * 0.1)
        # 对照：不给这个值时会放行 6 倍以上
        loose = slew_limit_velocity((0.5, 0.0, 0.0), (0.0, 0.0, 0.0),
                                    cfg.max_accel_xy, cfg.max_accel_theta,
                                    0.1, None)
        assert loose[0] > out[0] * 6.0

    def test_config_rejects_up_greater_than_down(self):
        with pytest.raises(ChassisConfigError) as e:
            ChassisBridgeConfig(max_accel_xy=2.5, max_accel_xy_up=3.0)
        assert 'max_accel_xy_up' in str(e.value)
