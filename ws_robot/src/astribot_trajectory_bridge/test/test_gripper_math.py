#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪换算的测试。**离线，不需要 SDK/仿真。**

这组测试的重点是**极性**
====================
夹爪命令空间 0=张开、100=闭合，与直觉相反。极性错的后果是抓取时张开、
放置时闭合，而**两边都不报错** —— 本项目已经在夹爪上吃过一次这种静默错误
（张口算成一半、抓取角错一倍且零报错）。所以下面把极性正反两个方向都钉住，
而不是只测"换算数值对不对"。
"""

import pytest

from astribot_trajectory_bridge.gripper_math import (
    CMD_CLOSED,
    CMD_OPEN,
    RAD_CLOSED,
    RAD_PER_CMD,
    GripperConfigError,
    clamp_cmd,
    cmd_to_opening_fraction,
    cmd_to_rad,
    describe_cmd,
    is_cmd_in_range,
    opening_fraction_to_cmd,
    rad_to_cmd,
    validate_grasp_cmd,
)


class TestPolarity:
    """!!! 极性 !!! 0=张开、100=闭合。这几条错了整个抓取都是反的。"""

    def test_zero_is_open(self):
        """astribot_client.py:813 —— open_effector 下发 0.0。"""
        assert CMD_OPEN == 0.0

    def test_hundred_is_closed(self):
        """astribot_client.py:836 —— close_effector 下发 100.0。"""
        assert CMD_CLOSED == 100.0

    def test_rad_increases_with_closing(self):
        """弧度是**闭合角**：命令越大（越闭合）弧度越大。"""
        assert cmd_to_rad(0) < cmd_to_rad(50) < cmd_to_rad(100)

    def test_opening_fraction_is_inverted_vs_cmd(self):
        """"张开程度"与命令**反向** —— 这就是提供该函数的理由。"""
        assert opening_fraction_to_cmd(1.0) == pytest.approx(CMD_OPEN)
        assert opening_fraction_to_cmd(0.0) == pytest.approx(CMD_CLOSED)

    def test_full_open_is_not_cmd_100(self):
        """反向锁：谁把"全张开"写成 100 就会失败。"""
        assert opening_fraction_to_cmd(1.0) != CMD_CLOSED


class TestConversion:

    def test_cmd_100_equals_joint_upper_limit(self):
        """cmd=100 必须正好落在 MuJoCo 关节上限 0.93 上。

        这个自洽关系是换算系数取理论值 4.65/500 而非实测 0.009298 的理由。
        """
        assert cmd_to_rad(100) == pytest.approx(0.93)
        assert RAD_CLOSED == pytest.approx(0.93)

    def test_coefficient_matches_actuator_params(self):
        """gainprm=4.65, biasprm[1]=-500 -> 稳态 4.65/500。"""
        assert RAD_PER_CMD == pytest.approx(4.65 / 500.0)

    def test_measured_coefficient_within_tolerance(self):
        """实测 0.009298 与理论 0.0093 差 0.02% —— 锁住这个吻合度。

        若哪天模型改了执行器参数，这条会失败并提醒重新标定。
        """
        measured = 0.009298
        assert abs(RAD_PER_CMD - measured) / RAD_PER_CMD < 0.001

    @pytest.mark.parametrize('cmd', [0, 20, 40, 60, 80, 100])
    def test_roundtrip_cmd_rad(self, cmd):
        assert rad_to_cmd(cmd_to_rad(cmd)) == pytest.approx(cmd)

    @pytest.mark.parametrize('frac', [0.0, 0.25, 0.5, 0.75, 1.0])
    def test_roundtrip_fraction(self, frac):
        assert cmd_to_opening_fraction(opening_fraction_to_cmd(frac)) == \
            pytest.approx(frac)

    def test_midpoint(self):
        assert cmd_to_rad(50) == pytest.approx(0.465)
        assert cmd_to_opening_fraction(50) == pytest.approx(0.5)


class TestClamping:

    def test_clamp_below(self):
        assert clamp_cmd(-20) == CMD_OPEN

    def test_clamp_above(self):
        assert clamp_cmd(150) == CMD_CLOSED

    def test_clamp_inside_unchanged(self):
        assert clamp_cmd(37.5) == pytest.approx(37.5)

    def test_cmd_to_rad_clamps_by_default(self):
        """默认夹：不许算出超过关节上限的弧度。"""
        assert cmd_to_rad(150) == pytest.approx(0.93)
        assert cmd_to_rad(-50) == pytest.approx(0.0)

    def test_cmd_to_rad_can_skip_clamp(self):
        """显式关掉夹用于诊断（比如想看"要求的值有多离谱"）。"""
        assert cmd_to_rad(200, clamp=False) == pytest.approx(1.86)

    def test_fraction_clamps(self):
        assert opening_fraction_to_cmd(1.5) == pytest.approx(CMD_OPEN)
        assert opening_fraction_to_cmd(-0.5) == pytest.approx(CMD_CLOSED)

    def test_in_range_predicate(self):
        assert is_cmd_in_range(0) and is_cmd_in_range(100)
        assert not is_cmd_in_range(-0.1)
        assert not is_cmd_in_range(100.1)


class TestValidation:
    """参数入口必须**显式报错**，不静默夹 —— 否则"我要求 150"会变成"实际 100"。"""

    def test_accepts_in_range(self):
        assert validate_grasp_cmd(0) == 0.0
        assert validate_grasp_cmd(100) == 100.0
        assert validate_grasp_cmd('42.5') == pytest.approx(42.5)

    def test_rejects_out_of_range(self):
        with pytest.raises(GripperConfigError):
            validate_grasp_cmd(150)
        with pytest.raises(GripperConfigError):
            validate_grasp_cmd(-1)

    def test_rejects_non_numeric(self):
        with pytest.raises(GripperConfigError):
            validate_grasp_cmd('open')
        with pytest.raises(GripperConfigError):
            validate_grasp_cmd(None)

    def test_rejects_nan(self):
        with pytest.raises(GripperConfigError):
            validate_grasp_cmd(float('nan'))

    def test_error_message_explains_polarity(self):
        """报错必须**提醒极性是反的** —— 越界往往正是极性搞错的症状。"""
        with pytest.raises(GripperConfigError) as ei:
            validate_grasp_cmd(150)
        msg = str(ei.value)
        assert '0=张开' in msg and 'opening_fraction_to_cmd' in msg


class TestDescribe:
    """日志里必须带文字说明：看到裸的 100 很容易误读成"全开"。"""

    def test_open(self):
        assert '张开' in describe_cmd(0)

    def test_closed(self):
        assert '闭合' in describe_cmd(100)

    def test_half(self):
        d = describe_cmd(50)
        assert '半开' in d and '50%' in d

    def test_includes_rad(self):
        assert '0.9300' in describe_cmd(100)

    def test_100_is_not_described_as_open(self):
        """反向锁：描述 100 时不许出现"全开"这类字样。"""
        d = describe_cmd(100)
        assert '张开' not in d.replace('半开', '')
