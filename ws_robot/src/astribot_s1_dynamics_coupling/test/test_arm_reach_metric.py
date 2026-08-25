#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arm_reach_metric 纯函数单测。

覆盖 C1 修正的核心算术：伸展→活跃度斜坡、阈值配置校验、旧度量回归路径、
以及"新度量必须能把收拢姿态和伸出姿态分开"这条本质要求。

用到的伸展数值全部是实测值（由活的 URDF 采样得到，见包 README 的取证表）：
  0.3532 = 肘部折回的真实收纳姿态
  0.4205 = 全 0 姿态（原来被当成"收纳基准"，其实肘部完全伸直）
  0.4698 = 实测左臂作业中
  0.4790 = ready
  0.8865 = 全工作空间最大水平伸展
"""

import math

import pytest

from astribot_s1_dynamics_coupling.arm_reach_metric import (
    METRIC_HORIZONTAL_REACH,
    METRIC_JOINT_DEVIATION,
    VALID_METRICS,
    ReachMetricConfigError,
    horizontal_reach,
    joint_deviation_activity,
    reach_activity,
    scale_from_activity,
    validate_reach_thresholds,
    velocity_activity,
)

FOLDED = 0.42
FULL = 0.8865
MIN_SCALE = 0.15

# 实测姿态样本：(标签, 水平伸展 m, 该姿态下的最大关节偏差 rad)
MEASURED_POSES = [
    ('候选收纳', 0.3532, 2.4000),
    ('全0', 0.4205, 0.0000),
    ('实测作业', 0.4698, 2.1104),
    ('ready', 0.4790, 1.0000),
    ('工作空间极限', 0.8865, 3.0620),
]

ARM_JOINTS = [f'astribot_arm_{s}_joint_{i}' for s in ('left', 'right') for i in range(1, 8)]


def scale_for_reach(reach_m):
    return scale_from_activity(reach_activity(reach_m, FOLDED, FULL), MIN_SCALE)


class TestHorizontalReach:

    def test_is_xy_norm_and_ignores_z(self):
        # 只取 xy：竖直抬高不增加倾覆力臂，把 z 算进去会把"举高但贴着身体"误判成展开
        assert horizontal_reach(0.3, 0.4) == pytest.approx(0.5)
        assert horizontal_reach(-0.3, 0.4) == pytest.approx(0.5)
        assert horizontal_reach(0.0, 0.0) == pytest.approx(0.0)


class TestReachThresholdValidation:

    def test_accepts_shipped_defaults(self):
        validate_reach_thresholds(FOLDED, FULL)

    def test_rejects_full_not_greater_than_folded(self):
        # 这是最容易配错的一种：区间为空会导致除零或量程反向，必须显式报错而不是静默夹紧
        with pytest.raises(ReachMetricConfigError):
            validate_reach_thresholds(0.5, 0.5)
        with pytest.raises(ReachMetricConfigError):
            validate_reach_thresholds(0.9, 0.42)

    def test_rejects_negative_folded(self):
        with pytest.raises(ReachMetricConfigError):
            validate_reach_thresholds(-0.1, 0.8)

    def test_reach_activity_raises_on_empty_span(self):
        # 即使绕过 validate 直接调用，也不能静默返回一个看起来正常的数
        with pytest.raises(ReachMetricConfigError):
            reach_activity(0.5, 0.5, 0.5)


class TestReachActivityRamp:

    def test_below_folded_is_zero(self):
        # 机械臂还在底盘足迹以内 → 规划器已经算进去了，不额外限速
        assert reach_activity(0.30, FOLDED, FULL) == pytest.approx(0.0)
        assert reach_activity(FOLDED, FOLDED, FULL) == pytest.approx(0.0)

    def test_at_and_above_full_is_clamped_to_one(self):
        assert reach_activity(FULL, FOLDED, FULL) == pytest.approx(1.0)
        assert reach_activity(2.0, FOLDED, FULL) == pytest.approx(1.0)

    def test_midpoint_is_half(self):
        mid = FOLDED + (FULL - FOLDED) / 2.0
        assert reach_activity(mid, FOLDED, FULL) == pytest.approx(0.5)

    def test_monotonic_non_decreasing_in_reach(self):
        prev = -1.0
        for i in range(0, 121):
            reach = 0.2 + i * 0.01
            act = reach_activity(reach, FOLDED, FULL)
            assert act >= prev - 1e-12, f'活跃度在 reach={reach:.3f} 处非单调'
            prev = act


class TestScaleFromActivity:

    def test_endpoints(self):
        assert scale_from_activity(0.0, MIN_SCALE) == pytest.approx(1.0)
        assert scale_from_activity(1.0, MIN_SCALE) == pytest.approx(MIN_SCALE)

    def test_never_below_floor_even_if_activity_overshoots(self):
        # 系数下限是安全底线：不能因为活跃度算出 >1 就把底盘限到 0(那样反而更危险)
        assert scale_from_activity(1.5, MIN_SCALE) == pytest.approx(MIN_SCALE)

    def test_never_above_one_if_activity_negative(self):
        assert scale_from_activity(-0.5, MIN_SCALE) == pytest.approx(1.0)

    def test_scale_monotonic_non_increasing_in_reach(self):
        prev = 2.0
        for i in range(0, 121):
            reach = 0.2 + i * 0.01
            s = scale_for_reach(reach)
            assert s <= prev + 1e-12, f'限速系数在 reach={reach:.3f} 处非单调'
            prev = s


class TestC1RegressionOnMeasuredPoses:
    """这一组是 C1 的本质回归：新度量必须修掉"与物理量反相关"这个错误。"""

    def test_new_metric_is_monotonic_in_measured_reach(self):
        # 按实测伸展排序后，限速系数必须单调不增
        by_reach = sorted(MEASURED_POSES, key=lambda p: p[1])
        scales = [scale_for_reach(p[1]) for p in by_reach]
        assert scales == sorted(scales, reverse=True), (
            '新度量在实测姿态上不单调：%s' % list(zip([p[0] for p in by_reach], scales)))

    def test_old_metric_was_anti_correlated_on_measured_poses(self):
        # 反过来证明旧度量确实是坏的（这条测试锁住"为什么要改"这个前提，
        # 如果哪天有人把默认度量改回去，这条会提醒他旧度量的问题依然存在）
        by_reach = sorted(MEASURED_POSES, key=lambda p: p[1])
        old_scales = [
            scale_from_activity(
                joint_deviation_activity(
                    {ARM_JOINTS[0]: dev}, ARM_JOINTS, [0.0] * len(ARM_JOINTS), 1.2),
                MIN_SCALE)
            for _, _, dev in by_reach
        ]
        assert old_scales != sorted(old_scales, reverse=True), (
            '旧度量竟然单调了，说明这组实测样本没能复现 C1 的反相关问题')
        # 最收拢的姿态在旧度量下反而比 ready 限得更狠——这就是那个反相关
        tucked_old = old_scales[0]
        ready_old = old_scales[3]
        assert tucked_old < ready_old, '旧度量的反相关未复现'

    def test_tucked_pose_is_unlimited_under_new_metric(self):
        # 真正收拢的姿态(0.3532m)在新度量下应该完全不限速
        assert scale_for_reach(0.3532) == pytest.approx(1.0)

    def test_workspace_extreme_still_hits_the_floor(self):
        # 修正不能变成"把保护关掉"：真正伸到极限时必须还是限到下限
        assert scale_for_reach(0.8865) == pytest.approx(MIN_SCALE)

    def test_working_poses_get_meaningful_speedup(self):
        # ready / 实测作业姿态应该显著放开(旧度量在这两个姿态上是 0.292 和 0.150)
        assert scale_for_reach(0.4790) > 0.85
        assert scale_for_reach(0.4698) > 0.85


class TestJointDeviationLegacyPath:

    def test_missing_joints_are_skipped_not_treated_as_zero(self):
        # 关节缺失当成 0 偏差会伪造"手臂已收拢"的假象 → 静默解除限速。
        # !!! 参考姿态必须取非零，否则这条测试分辨不出来 !!!
        # 参考全 0 时，"跳过缺失关节"和"把缺失关节当 0"算出的偏差都是 0，测试会
        # 假通过——故障注入(把 .get(name) 改成 .get(name, 0.0))时正是这条没抓住。
        ref = [0.8] * len(ARM_JOINTS)
        # 全部缺失：跳过 → 活跃度 0；若当成 0 处理则偏差 0.8、活跃度 0.667
        assert joint_deviation_activity({}, ARM_JOINTS, ref, 1.2) == pytest.approx(0.0)
        # 只有一个关节在位且恰好等于参考值：仍应是 0，其余缺失的不许贡献偏差
        assert joint_deviation_activity(
            {ARM_JOINTS[3]: 0.8}, ARM_JOINTS, ref, 1.2) == pytest.approx(0.0)
        # 在位的那个关节偏离参考值时，必须按它算
        assert joint_deviation_activity(
            {ARM_JOINTS[3]: 2.0}, ARM_JOINTS, ref, 1.2) == pytest.approx(1.0)

    def test_zero_full_scale_does_not_divide_by_zero(self):
        assert joint_deviation_activity(
            {ARM_JOINTS[0]: 5.0}, ARM_JOINTS, [0.0] * len(ARM_JOINTS), 0.0) == 0.0

    def test_reference_offset_is_honoured(self):
        ref = [0.5] * len(ARM_JOINTS)
        assert joint_deviation_activity(
            {ARM_JOINTS[0]: 0.5}, ARM_JOINTS, ref, 1.2) == pytest.approx(0.0)


class TestVelocityActivityUntouchedByC1:
    """速率维度是 C1 明确不动的部分，这里锁住它的行为不被顺手改坏。"""

    def test_endpoints_and_clamp(self):
        assert velocity_activity({}, ARM_JOINTS, 2.0) == pytest.approx(0.0)
        assert velocity_activity(
            {ARM_JOINTS[0]: 1.0}, ARM_JOINTS, 2.0) == pytest.approx(0.5)
        assert velocity_activity(
            {ARM_JOINTS[0]: 99.0}, ARM_JOINTS, 2.0) == pytest.approx(1.0)

    def test_uses_absolute_value(self):
        assert velocity_activity(
            {ARM_JOINTS[0]: -1.0}, ARM_JOINTS, 2.0) == pytest.approx(0.5)

    def test_zero_full_scale_does_not_divide_by_zero(self):
        assert velocity_activity({ARM_JOINTS[0]: 5.0}, ARM_JOINTS, 0.0) == 0.0

    def test_takes_max_over_joints_not_sum(self):
        vels = {ARM_JOINTS[0]: 0.4, ARM_JOINTS[1]: 0.4, ARM_JOINTS[2]: 0.4}
        assert velocity_activity(vels, ARM_JOINTS, 2.0) == pytest.approx(0.2)


class TestMetricConstants:

    def test_valid_metrics_contains_both_and_nothing_else(self):
        assert set(VALID_METRICS) == {METRIC_HORIZONTAL_REACH, METRIC_JOINT_DEVIATION}

    def test_no_nan_leaks_from_activity(self):
        for _, reach, _ in MEASURED_POSES:
            assert not math.isnan(scale_for_reach(reach))
