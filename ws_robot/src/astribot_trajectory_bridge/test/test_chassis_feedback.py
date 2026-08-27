#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""leash 与 SLAM 位姿反馈外环的单元测试。

这组测试是本模块**安全性的主要证据**：底盘是位置指令开环积分，leash 是唯一硬保护，
外环校正是唯一能纠正累积漂移的机制，而两者都必须在位姿源发疯时不放大故障。
"""

import math

import pytest

from astribot_trajectory_bridge.chassis_integrator import ChassisConfigError
from astribot_trajectory_bridge.chassis_feedback import (
    KP_HARD_MAX,
    POSE_SOURCE_GROUND_TRUTH,
    POSE_SOURCE_SLAM,
    advance_desired_pose,
    check_leash,
    compute_correction,
    detect_pose_jump,
    effective_thresholds,
    is_correction_degenerate,
    leash_recover_command,
    odom_drift,
    slice_correction,
    validate_correction_config,
    validate_leash_config,
)

LEASH_XY = 0.25
LEASH_TH = 0.35
OUTER = 10.0
INNER = 250.0


class TestLeashConfig:

    def test_accepts_defaults(self):
        validate_leash_config(LEASH_XY, LEASH_TH)

    def test_zero_xy_rejected(self):
        # <=0 等于关闭保护，而 leash 是开环链路上唯一的硬保护，不允许关闭
        with pytest.raises(ChassisConfigError) as e:
            validate_leash_config(0.0, LEASH_TH)
        assert '唯一' in str(e.value)

    def test_negative_theta_rejected(self):
        with pytest.raises(ChassisConfigError):
            validate_leash_config(LEASH_XY, -0.1)


class TestLeashTrip:

    def test_within_leash_not_tripped(self):
        st = check_leash([0.1, 0.1, 0.0], [0.0, 0.0, 0.0], LEASH_XY, LEASH_TH)
        assert not st.tripped
        assert st.err_xy == pytest.approx(math.hypot(0.1, 0.1))

    def test_xy_trips(self):
        st = check_leash([0.3, 0.0, 0.0], [0.0, 0.0, 0.0], LEASH_XY, LEASH_TH)
        assert st.tripped
        assert 'xy' in st.reason

    def test_theta_trips_independently(self):
        # xy 完全没问题，只有朝向偏了 -> 也必须触发
        st = check_leash([0.0, 0.0, 0.5], [0.0, 0.0, 0.0], LEASH_XY, LEASH_TH)
        assert st.tripped
        assert 'theta' in st.reason

    def test_boundary_is_strict_greater(self):
        st = check_leash([LEASH_XY, 0.0, 0.0], [0.0, 0.0, 0.0], LEASH_XY, LEASH_TH)
        assert not st.tripped

    def test_theta_wrap_does_not_false_trip(self):
        # 179° vs -179° 实际只差 2°，不归一会算成 358° 直接误触发
        st = check_leash([0.0, 0.0, math.radians(179)],
                         [0.0, 0.0, math.radians(-179)], LEASH_XY, LEASH_TH)
        assert not st.tripped

    def test_recover_pulls_command_to_actual(self):
        actual = [1.5, -2.5, 0.9]
        out = leash_recover_command(actual)
        assert out == pytest.approx(actual)

    def test_recover_is_a_copy(self):
        actual = [1.0, 2.0, 0.0]
        out = leash_recover_command(actual)
        out[0] = 99.0
        assert actual[0] == pytest.approx(1.0)


class TestCorrectionConfig:

    def test_accepts_defaults(self):
        validate_correction_config(0.35, 0.40, 0.10, 0.20, OUTER, INNER)

    def test_kp_above_hard_max_rejected(self):
        # 外环 10Hz 下增益 1.0 意味着一个周期吃掉全部误差 -> 必然过冲
        with pytest.raises(ChassisConfigError) as e:
            validate_correction_config(1.0, 0.4, 0.10, 0.20, OUTER, INNER)
        assert '硬上限' in str(e.value)
        assert KP_HARD_MAX == 0.5

    def test_zero_corr_vel_rejected(self):
        # 这是位姿源发疯时的最后防线，不允许关闭
        with pytest.raises(ChassisConfigError) as e:
            validate_correction_config(0.35, 0.4, 0.0, 0.20, OUTER, INNER)
        assert '最后防线' in str(e.value)

    def test_outer_faster_than_inner_rejected(self):
        with pytest.raises(ChassisConfigError) as e:
            validate_correction_config(0.35, 0.4, 0.1, 0.2,
                                       outer_rate=300.0, inner_freq=250.0)
        assert '切片' in str(e.value)


class TestPoseSourceSemantics:
    """ground_truth 下闭环退化 —— 这组测试防止把它误当成"闭环已验收"。"""

    def test_slam_keeps_thresholds(self):
        jump, drift = effective_thresholds(POSE_SOURCE_SLAM, 0.30, 0.15)
        assert jump == pytest.approx(0.30)
        assert drift == pytest.approx(0.15)

    def test_ground_truth_disables_jump_and_drift(self):
        jump, drift = effective_thresholds(POSE_SOURCE_GROUND_TRUTH, 0.30, 0.15)
        assert jump == float('inf')     # 静态 TF 不会跳，检测无意义
        assert drift == float('inf')    # 两个位移源同源，差恒为 0

    def test_invalid_pose_source_rejected(self):
        with pytest.raises(ChassisConfigError):
            effective_thresholds('amcl', 0.30, 0.15)

    def test_ground_truth_is_degenerate(self):
        assert is_correction_degenerate(POSE_SOURCE_GROUND_TRUTH, True) is True

    def test_slam_is_not_degenerate(self):
        assert is_correction_degenerate(POSE_SOURCE_SLAM, True) is False

    def test_correction_off_is_not_degenerate(self):
        # 闭环没开就不该报"退化"，那会掩盖"根本没开闭环"这个事实
        assert is_correction_degenerate(POSE_SOURCE_GROUND_TRUTH, False) is False


class TestJumpDetection:

    def test_first_call_no_jump(self):
        tripped, dist = detect_pose_jump([1.0, 1.0, 0.0], None, 0.30)
        assert not tripped and dist == pytest.approx(0.0)

    def test_small_move_not_jump(self):
        tripped, dist = detect_pose_jump([0.1, 0.0, 0.0], [0.0, 0.0, 0.0], 0.30)
        assert not tripped and dist == pytest.approx(0.1)

    def test_relocalization_detected(self):
        tripped, dist = detect_pose_jump([0.5, 0.0, 0.0], [0.0, 0.0, 0.0], 0.30)
        assert tripped and dist == pytest.approx(0.5)

    def test_inf_threshold_never_trips(self):
        # ground_truth 模式下阈值被置 inf
        tripped, _ = detect_pose_jump([100.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                                      float('inf'))
        assert not tripped


class TestComputeCorrection:

    def test_zero_error_zero_correction(self):
        c = compute_correction([1.0, 2.0, 0.3], [1.0, 2.0, 0.3],
                               0.35, 0.40, 0.10, 0.20, OUTER)
        assert c[0] == pytest.approx(0.0)
        assert c[1] == pytest.approx(0.0)
        assert c[2] == pytest.approx(0.0)

    def test_small_error_scaled_by_kp(self):
        # theta=0 时 map 系与本体系一致；误差 0.01m * kp 0.35 = 0.0035
        c = compute_correction([0.01, 0.0, 0.0], [0.0, 0.0, 0.0],
                               0.35, 0.40, 0.10, 0.20, OUTER)
        assert c[0] == pytest.approx(0.0035)

    def test_error_rotated_into_body_frame(self):
        # 底盘朝 90°，map 系 +x 的误差在本体系里是 -y
        c = compute_correction([1.0, 0.0, math.pi / 2], [0.0, 0.0, math.pi / 2],
                               kp_xy=0.35, kp_theta=0.4,
                               max_corr_vel_xy=100.0, max_corr_vel_theta=100.0,
                               outer_rate=OUTER)
        assert c[0] == pytest.approx(0.0, abs=1e-12)
        assert c[1] == pytest.approx(-0.35)

    def test_correction_velocity_is_capped(self):
        # 注入 10m 巨大误差 -> 单个外环周期的校正位移 <= max_corr_vel/outer_rate
        c = compute_correction([10.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                               0.35, 0.40, max_corr_vel_xy=0.10,
                               max_corr_vel_theta=0.20, outer_rate=OUTER)
        step = math.hypot(c[0], c[1])
        assert step <= 0.10 / OUTER + 1e-12
        # 换算成速度必须 <= 0.10 m/s
        assert step * OUTER <= 0.10 + 1e-12

    def test_cap_preserves_direction(self):
        c = compute_correction([10.0, 10.0, 0.0], [0.0, 0.0, 0.0],
                               0.35, 0.40, 0.10, 0.20, OUTER)
        assert c[0] == pytest.approx(c[1])

    def test_theta_correction_capped(self):
        c = compute_correction([0.0, 0.0, 3.0], [0.0, 0.0, 0.0],
                               0.35, 0.40, 0.10, max_corr_vel_theta=0.20,
                               outer_rate=OUTER)
        assert abs(c[2]) <= 0.20 / OUTER + 1e-12


class TestSliceCorrection:

    def test_sliced_into_inner_ticks(self):
        # 外环 10Hz、内环 250Hz -> 25 个内环周期
        c = slice_correction((0.025, 0.0, 0.0), INNER, OUTER)
        assert c[0] == pytest.approx(0.001)

    def test_sum_of_slices_equals_original(self):
        orig = (0.025, -0.01, 0.004)
        ticks = int(round(INNER / OUTER))
        s = slice_correction(orig, INNER, OUTER)
        for i in range(3):
            assert s[i] * ticks == pytest.approx(orig[i])

    def test_never_divides_by_zero_ticks(self):
        s = slice_correction((1.0, 0.0, 0.0), inner_freq=5.0, outer_rate=10.0)
        assert s[0] == pytest.approx(1.0)   # ticks 下限为 1


class TestOdomDrift:
    """frame 无关性是这组测试的核心 —— 它让 T_map_sdk 从设计里彻底消失。"""

    def test_equal_displacement_zero_drift(self):
        assert odom_drift((1.0, 0.0), (1.0, 0.0)) == pytest.approx(0.0)

    def test_slip_detected(self):
        # 轮子以为走了 1.0m，世界里只走了 0.6m -> 打滑 0.4m
        assert odom_drift((1.0, 0.0), (0.6, 0.0)) == pytest.approx(0.4)

    def test_frame_invariant_under_rotation(self):
        # 两个位移源各自处于不同 frame（差一个旋转），drift 必须不变
        theta = 0.9
        c, s = math.cos(theta), math.sin(theta)
        d_sdk = (1.0, 0.3)
        d_slam = (0.7, -0.2)
        rot_slam = (c * d_slam[0] - s * d_slam[1], s * d_slam[0] + c * d_slam[1])
        assert odom_drift(d_sdk, d_slam) == pytest.approx(
            odom_drift(d_sdk, rot_slam))

    def test_frame_invariant_under_translation(self):
        # 只看增量，所以平移偏移天然不影响（这正是不需要 T_map_sdk 的原因）
        assert odom_drift((1.0, 0.0), (0.6, 0.0)) == pytest.approx(
            odom_drift((1.0, 0.0), (0.6, 0.0)))

    def test_direction_difference_not_captured(self):
        # 如实记录本算法的边界：模长相等但方向相反时 drift=0。
        # 这是刻意的取舍 —— 换取 frame 无关性。方向类故障由 leash 与外环校正覆盖。
        assert odom_drift((1.0, 0.0), (-1.0, 0.0)) == pytest.approx(0.0)


class TestAdvanceDesiredPose:

    def test_uses_slam_theta_for_rotation(self):
        # 用 SLAM 的绝对朝向旋转本体位移；theta_slam=90° 时本体 +x -> map +y
        out = advance_desired_pose([0.0, 0.0, 0.0], (1.0, 0.0), 0.0,
                                   theta_slam=math.pi / 2)
        assert out[0] == pytest.approx(0.0, abs=1e-12)
        assert out[1] == pytest.approx(1.0)

    def test_theta_accumulates_and_wraps(self):
        out = advance_desired_pose([0.0, 0.0, math.pi - 0.01], (0.0, 0.0), 0.02,
                                   theta_slam=0.0)
        assert out[2] <= math.pi
