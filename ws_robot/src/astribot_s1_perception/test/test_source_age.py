#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`check_source_age` 的离线测试 —— A2（陈旧 TF 被当成当前值）的回归网。

════════════════ 这组测试为什么存在 ════════════════
tf2 的 Buffer 只在某个 frame pair **有新数据进来时**才修剪它。停更的 pair
把最后一条记录永久保留，而 `lookup_transform(..., Time())` 的语义是
"最新可用的" —— 于是它一直成功、一直返回陈旧值、一直不报错。

本仓库在"冻结读数被当成当前值"上已经吃过多次亏（陈旧 TF、冻结拍率、
脉冲当速率），所以这里把判据钉死在纯函数上，不依赖 rclpy、不读时钟。
时间一律由测试传入。
"""

import math

import pytest

from astribot_s1_perception.map_odom_decompose import (
    CLOCK_SKEW_TOLERANCE_SEC,
    DEFAULT_MAX_SOURCE_AGE_SEC,
    DecompositionError,
    check_source_age,
)


class TestFresh:
    """新鲜数据必须原样通过，且不能有误报 —— 否则整条 TF 链会被自己掐断。"""

    def test_same_instant_is_fresh(self):
        r = check_source_age(100.0, 100.0, 1.0)
        assert not r.stale
        assert r.age_sec == pytest.approx(0.0)
        assert r.reason == ''

    def test_just_under_limit_is_fresh(self):
        r = check_source_age(100.0, 99.01, 1.0)
        assert not r.stale
        assert r.age_sec == pytest.approx(0.99)

    def test_exactly_at_limit_is_fresh(self):
        # 边界取"不算陈旧"：判据是 age > max，不是 >=。
        # 20Hz 发布 + 1.0s 上限时，恰好等于上限是正常抖动，不该丢。
        r = check_source_age(100.0, 99.0, 1.0)
        assert not r.stale

    def test_age_is_reported_even_when_fresh(self):
        # 龄期本身是可观测量，新鲜时也要能读出来（用于上报，而非只在出错时才有）
        r = check_source_age(100.0, 99.5, 1.0)
        assert r.age_sec == pytest.approx(0.5)


class TestStale:
    """陈旧必须被拦住，且 reason 要指向**发布方**而不是本节点。"""

    def test_just_over_limit_is_stale(self):
        r = check_source_age(100.0, 98.99, 1.0)
        assert r.stale
        assert r.age_sec == pytest.approx(1.01)

    def test_slam_died_ten_seconds_ago(self):
        r = check_source_age(100.0, 90.0, 1.0, 'camera_init→aft_mapped')
        assert r.stale
        assert 'camera_init→aft_mapped' in r.reason
        assert '10.00s' in r.reason

    def test_reason_points_at_the_publisher_not_at_us(self):
        # 这条断言是防"报错把人引错方向"的。陈旧的根因永远在发布方。
        r = check_source_age(100.0, 50.0, 1.0, 'odom→astribot_torso_base')
        assert r.stale
        assert '查它' in r.reason
        assert '不要查本节点' in r.reason

    def test_reason_explains_why_lookup_did_not_fail(self):
        # 排查者的第一反应是"取到了就说明是好的"。reason 必须先破这个假设，
        # 否则他会去怀疑本节点的判据写错了。
        r = check_source_age(100.0, 80.0, 1.0, 'x→y')
        assert 'tf2' in r.reason
        assert '不报任何错' in r.reason


class TestUnsetStamp:
    """戳是 0 的情况必须单独识别，不能算成"很旧"含混过去。"""

    @pytest.mark.parametrize('stamp', [0.0, -1.0, -1e9])
    def test_nonpositive_stamp_is_stale(self, stamp):
        r = check_source_age(100.0, stamp, 1.0, 'a→b')
        assert r.stale

    def test_unset_stamp_says_unset_not_old(self):
        r = check_source_age(100.0, 0.0, 1.0, 'a→b')
        assert '未填' in r.reason
        assert r.age_sec == math.inf

    def test_unset_stamp_is_stale_even_with_huge_limit(self):
        # "戳没填"不是龄期问题，放宽上限不该让它通过。
        r = check_source_age(100.0, 0.0, 1e9, 'a→b')
        assert r.stale


class TestClockSkew:
    """未来戳要与"新鲜"区分开 —— 本机开机时钟是 1970，靠 PTP 追上来。"""

    def test_small_negative_age_is_tolerated(self):
        # 发布方按采集时刻打戳、我们在它到达前就查，会让龄期略负。
        r = check_source_age(100.0, 100.0 + CLOCK_SKEW_TOLERANCE_SEC / 2, 1.0)
        assert not r.stale

    def test_at_tolerance_boundary_is_tolerated(self):
        r = check_source_age(100.0, 100.0 + CLOCK_SKEW_TOLERANCE_SEC, 1.0)
        assert not r.stale

    def test_far_future_stamp_is_rejected(self):
        r = check_source_age(100.0, 130.0, 1.0, 'a→b')
        assert r.stale
        assert r.age_sec == pytest.approx(-30.0)

    def test_future_stamp_reason_mentions_ptp_not_staleness(self):
        # 时钟不一致与"数据旧"是两个完全不同的故障，指错方向就白查一轮。
        r = check_source_age(100.0, 130.0, 1.0, 'a→b')
        assert 'ptp' in r.reason.lower()
        assert '不是数据新' in r.reason

    def test_future_stamp_is_not_reported_as_positive_age(self):
        r = check_source_age(100.0, 130.0, 1.0)
        assert r.age_sec < 0


class TestDisabled:
    """逃生口：max<=0 关闭检查。必须真的完全关掉，含戳未填的情形。"""

    @pytest.mark.parametrize('max_age', [0.0, -1.0])
    def test_nonpositive_max_disables_check(self, max_age):
        r = check_source_age(100.0, 0.0, max_age, 'a→b')
        assert not r.stale
        assert r.reason == ''

    def test_disabled_also_lets_ancient_data_through(self):
        r = check_source_age(1e9, 1.0, 0.0)
        assert not r.stale


class TestBadInput:
    """NaN/Inf 必须当场炸，不能悄悄算出一个 bool。"""

    @pytest.mark.parametrize('now,stamp', [
        (float('nan'), 100.0),
        (100.0, float('nan')),
        (float('inf'), 100.0),
        (100.0, float('-inf')),
    ])
    def test_non_finite_raises(self, now, stamp):
        with pytest.raises(DecompositionError):
            check_source_age(now, stamp, 1.0)

    def test_non_finite_raises_before_the_disabled_shortcut(self):
        # 顺序很重要：先校验输入，再看是否关闭检查。反过来会让
        # "检查关着"时的 NaN 静默流进 Pose2D，而那里的报错离根因更远。
        with pytest.raises(DecompositionError):
            check_source_age(float('nan'), 100.0, 0.0)


class TestDefaultLimit:
    """默认上限必须容得下真实发布率，又能在进程死掉后及时发现。"""

    def test_default_tolerates_slam_at_ten_hz(self):
        # SLAM 出图约 10Hz（实测雷达 99.8ms 周期）。一两帧抖动不该被判陈旧。
        r = check_source_age(100.0, 100.0 - 0.3, DEFAULT_MAX_SOURCE_AGE_SEC)
        assert not r.stale

    def test_default_catches_a_dead_publisher_within_seconds(self):
        r = check_source_age(100.0, 100.0 - 2.0, DEFAULT_MAX_SOURCE_AGE_SEC)
        assert r.stale
