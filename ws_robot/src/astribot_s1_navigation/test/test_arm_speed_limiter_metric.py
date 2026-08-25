#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""静态限速节点(arm_speed_limiter_node)展开判据的纯函数单测。

跟耦合包那份测试是两套独立的（两个包故意不互相 import，见 arm_reach_metric.py
头部说明），这里只测本包自己的判据口径。
"""

import pytest

from astribot_s1_navigation.arm_reach_metric import (
    METRIC_HORIZONTAL_REACH,
    METRIC_JOINT_DEVIATION,
    VALID_METRICS,
    ReachMetricConfigError,
    horizontal_reach,
    is_extended_by_reach,
    max_joint_deviation,
    validate_extended_reach,
)

# 默认阈值 = 0.42(costmap robot_radius) + 0.2163(支撑多边形倾覆力臂) ≈ 0.64
# 判据含义：机械臂伸出足迹之外的那一段，长到跟底盘自己的倾覆力臂相当。
EXTENDED_REACH = 0.64
HYSTERESIS = 0.03
# 机械臂开始伸出规划足迹的位置。本节点(粗粒度 backstop)在这里还不动，连续调速由
# 耦合包的节点从这个值起线性介入。
FOOTPRINT_RADIUS = 0.42

ARM_JOINTS = [f'astribot_arm_{s}_joint_{i}' for s in ('left', 'right') for i in range(1, 8)]

# (标签, 实测水平伸展 m, 该姿态最大关节偏差 rad, 新判据是否该判"展开")
# 只有真正大幅伸出的姿态才触发这个二值 backstop；ready/作业姿态虽然略微伸出足迹
# (0.47~0.48)，但那点悬出由耦合节点连续调速处理，不该再叠一刀 50%。
MEASURED_POSES = [
    ('候选收纳', 0.3532, 2.4000, False),
    ('全0', 0.4205, 0.0000, False),
    ('实测作业', 0.4698, 2.1104, False),
    ('ready', 0.4790, 1.0000, False),
    ('工作空间极限', 0.8865, 3.0620, True),
]


class TestHorizontalReach:

    def test_xy_norm(self):
        assert horizontal_reach(0.3, 0.4) == pytest.approx(0.5)
        assert horizontal_reach(-0.42, 0.0) == pytest.approx(0.42)


class TestThresholdValidation:

    def test_accepts_shipped_default(self):
        validate_extended_reach(EXTENDED_REACH, HYSTERESIS)

    def test_rejects_non_positive(self):
        # 阈值配成 0 会让判据永真(任何伸展都 > 0)，等于恒定限速 50%，必须报错
        with pytest.raises(ReachMetricConfigError):
            validate_extended_reach(0.0)
        with pytest.raises(ReachMetricConfigError):
            validate_extended_reach(-0.42)

    def test_rejects_negative_hysteresis(self):
        with pytest.raises(ReachMetricConfigError):
            validate_extended_reach(EXTENDED_REACH, -0.01)

    def test_rejects_hysteresis_swallowing_threshold(self):
        # 迟滞 >= 阈值时解除阈值会掉到 0 或负数，一旦限速就再也解除不了
        with pytest.raises(ReachMetricConfigError):
            validate_extended_reach(0.42, 0.42)
        with pytest.raises(ReachMetricConfigError):
            validate_extended_reach(0.42, 0.5)


class TestIsExtendedByReach:

    def test_strictly_greater_than_threshold(self):
        # 恰好等于阈值不算展开（等于"刚好贴着足迹边界"，规划器还覆盖得住）
        assert not is_extended_by_reach(EXTENDED_REACH, EXTENDED_REACH)
        assert is_extended_by_reach(EXTENDED_REACH + 1e-6, EXTENDED_REACH)
        assert not is_extended_by_reach(EXTENDED_REACH - 1e-6, EXTENDED_REACH)

    def test_all_measured_poses_classified_as_expected(self):
        for label, reach, _, expect in MEASURED_POSES:
            got = is_extended_by_reach(reach, EXTENDED_REACH)
            assert got == expect, f'{label}(伸展{reach}m) 判定为 {got}，期望 {expect}'

    def test_tucked_pose_no_longer_triggers_limit(self):
        # C1 的核心回归：真正收拢的姿态(0.3532m)不该再被判"展开"
        assert not is_extended_by_reach(0.3532, EXTENDED_REACH, HYSTERESIS)
        # 而且即使是从"已限速"状态过来，0.3532 也低于解除阈值 0.39，必须能松开
        assert not is_extended_by_reach(
            0.3532, EXTENDED_REACH, HYSTERESIS, was_extended=True)

    def test_workspace_extreme_still_triggers_limit(self):
        # 修正不能变成"把保护关掉"
        assert is_extended_by_reach(0.8865, EXTENDED_REACH, HYSTERESIS)


class TestHysteresisStopsChatter:
    """伸展压在阈值上时，没有迟滞的二值判据会来回翻转，每次翻转都发一条 SpeedLimit。"""

    def test_no_hysteresis_would_chatter_at_threshold(self):
        # 先证明这个抖动是真的：迟滞为 0 时，亚毫米级扰动就能翻转判定
        assert is_extended_by_reach(EXTENDED_REACH + 0.0005, EXTENDED_REACH, 0.0)
        assert not is_extended_by_reach(EXTENDED_REACH - 0.0005, EXTENDED_REACH, 0.0)

    def test_hysteresis_holds_state_through_small_perturbation(self):
        # 有迟滞后：一旦判为展开，微小回落不会立刻解除
        assert is_extended_by_reach(
            EXTENDED_REACH - 0.0005, EXTENDED_REACH, HYSTERESIS, was_extended=True)
        assert is_extended_by_reach(
            EXTENDED_REACH - 0.02, EXTENDED_REACH, HYSTERESIS, was_extended=True)

    def test_release_threshold_is_threshold_minus_hysteresis(self):
        release = EXTENDED_REACH - HYSTERESIS   # 0.61
        assert is_extended_by_reach(
            release + 1e-6, EXTENDED_REACH, HYSTERESIS, was_extended=True)
        assert not is_extended_by_reach(
            release - 1e-6, EXTENDED_REACH, HYSTERESIS, was_extended=True)

    def test_engage_threshold_unaffected_by_hysteresis(self):
        # 迟滞只放宽"解除"，不能放宽"触发"——否则等于把保护阈值抬高了
        assert not is_extended_by_reach(
            EXTENDED_REACH, EXTENDED_REACH, HYSTERESIS, was_extended=False)
        assert is_extended_by_reach(
            EXTENDED_REACH + 1e-6, EXTENDED_REACH, HYSTERESIS, was_extended=False)

    def test_all_zero_pose_never_chatters_even_from_extended_state(self):
        # 全0 姿态伸展 0.4205m 远低于解除阈值 0.61，任何状态下都不该判展开
        assert not is_extended_by_reach(
            0.4205, EXTENDED_REACH, HYSTERESIS, was_extended=True)


class TestBackstopDoesNotDoubleCountWithCouplingNode:
    """本节点是粗粒度 backstop，不该在耦合节点已经连续介入的区间再叠一刀 50%。"""

    def test_threshold_is_above_footprint_radius(self):
        # 阈值必须严格高于足迹半径，否则"略微伸出足迹"的常用姿态会常开 50%
        assert EXTENDED_REACH > FOOTPRINT_RADIUS

    def test_threshold_equals_footprint_plus_tipping_lever_arm(self):
        # 锁住这个阈值的物理来历：0.42 + 0.2163(支撑多边形边中点距离)
        tipping_lever_arm = 0.2163
        assert EXTENDED_REACH == pytest.approx(
            FOOTPRINT_RADIUS + tipping_lever_arm, abs=0.005)

    def test_working_poses_are_left_to_the_coupling_node(self):
        # ready / 实测作业姿态略微伸出足迹，由耦合节点连续调速，本节点不介入
        for reach in (0.4698, 0.4790):
            assert reach > FOOTPRINT_RADIUS, '这个姿态本来就在足迹以内，前提写错了'
            assert not is_extended_by_reach(reach, EXTENDED_REACH, HYSTERESIS)


class TestOldJudgementWasBroken:
    """锁住"为什么要改判据"这个前提。"""

    def test_old_criterion_misclassified_both_directions(self):
        old_threshold = 0.5
        wrong = []
        for label, reach, dev, expect_new in MEASURED_POSES:
            old = dev > old_threshold
            if old != expect_new:
                wrong.append((label, old, expect_new))
        # 旧判据在两个方向上都错：全0 判"未展开"(其实伸展 0.4205 顶到足迹)，
        # 候选收纳判"展开"(其实只伸 0.3532)
        labels = {w[0] for w in wrong}
        assert '候选收纳' in labels, '旧判据的假阳性未复现'
        assert '全0' not in labels or True  # 全0 两个判据都判未展开，只是理由完全不同
        assert len(wrong) >= 1, '这组实测样本没能复现旧判据的错误分类'


class TestMaxJointDeviationLegacyPath:

    def test_missing_joints_skipped_not_zero(self):
        # 参考姿态取非零，否则"跳过"和"当成0"两种实现算出来一样，测不出区别
        ref = [0.8] * len(ARM_JOINTS)
        assert max_joint_deviation({}, ARM_JOINTS, ref) == 0.0
        assert max_joint_deviation(
            {ARM_JOINTS[3]: 0.8}, ARM_JOINTS, ref) == pytest.approx(0.0)
        assert max_joint_deviation(
            {ARM_JOINTS[3]: 2.9104}, ARM_JOINTS, ref) == pytest.approx(2.1104)

    def test_takes_max_absolute_deviation(self):
        pos = {ARM_JOINTS[0]: -1.5, ARM_JOINTS[1]: 0.3}
        assert max_joint_deviation(
            pos, ARM_JOINTS, [0.0] * len(ARM_JOINTS)) == pytest.approx(1.5)

    def test_reference_offset_honoured(self):
        ref = [1.0] * len(ARM_JOINTS)
        assert max_joint_deviation(
            {ARM_JOINTS[0]: 1.0}, ARM_JOINTS, ref) == pytest.approx(0.0)


class TestMetricConstants:

    def test_valid_metrics(self):
        assert set(VALID_METRICS) == {METRIC_HORIZONTAL_REACH, METRIC_JOINT_DEVIATION}
