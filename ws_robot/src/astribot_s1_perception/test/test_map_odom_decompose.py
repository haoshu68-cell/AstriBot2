#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`map→odom` 分解的离线测试。不需要 ROS / SLAM / 真机。

这一层的错误后果是**地图与机器人系统性偏移**，而症状看起来像"定位漂移"，
极难归因。所以核心不变量（分解后 base 必须落在 SLAM 说的位置）用随机化
穷举来验，而不是只测几个手算的例子。
"""

import math

import pytest

from astribot_s1_perception.map_odom_decompose import (
    DecompositionError,
    MapOdomDecomposer,
    Pose2D,
    quaternion_from_yaw,
    wrap_angle,
    yaw_from_quaternion,
)


# ---------------------------------------------------------------------------
# 一、Pose2D 的代数性质
# ---------------------------------------------------------------------------
def test_identity_compose_is_noop():
    p = Pose2D(1.5, -2.5, 0.7)
    assert_pose_close(Pose2D().compose(p), p)
    assert_pose_close(p.compose(Pose2D()), p)


def test_inverse_then_compose_gives_identity():
    for p in (Pose2D(1.0, 2.0, 0.5), Pose2D(-3.0, 0.5, -2.1), Pose2D(0.0, 0.0, math.pi)):
        assert_pose_close(p.compose(p.inverse()), Pose2D())
        assert_pose_close(p.inverse().compose(p), Pose2D())


def test_compose_is_not_commutative():
    """左乘右乘不同 —— 写反了会让地图绕机器人转，这条钉住顺序。"""
    a, b = Pose2D(1.0, 0.0, 0.0), Pose2D(0.0, 0.0, math.pi / 2)
    assert a.compose(b).translation_distance_to(b.compose(a)) > 0.5


def test_pure_rotation_moves_translation():
    """先转 90° 再前进 1m，应落在 +y。"""
    result = Pose2D(0.0, 0.0, math.pi / 2).compose(Pose2D(1.0, 0.0, 0.0))
    assert result.x == pytest.approx(0.0, abs=1e-9)
    assert result.y == pytest.approx(1.0)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf')])
def test_non_finite_rejected(bad):
    """NaN 会让整条 TF 链变 NaN，而 tf2 不报错，只是所有查询都失败。"""
    with pytest.raises(DecompositionError) as excinfo:
        Pose2D(bad, 0.0, 0.0)
    assert 'tf2 不会报错' in str(excinfo.value)


# ---------------------------------------------------------------------------
# 二、四元数 <-> yaw
# ---------------------------------------------------------------------------
@pytest.mark.parametrize('theta', [0.0, 0.5, -0.5, 1.7, -2.9, math.pi - 1e-9])
def test_quaternion_yaw_roundtrip(theta):
    z, w = quaternion_from_yaw(theta)
    assert yaw_from_quaternion(z, w) == pytest.approx(wrap_angle(theta))
    assert z * z + w * w == pytest.approx(1.0)


def test_yaw_wrapped_into_range():
    assert -math.pi <= yaw_from_quaternion(*quaternion_from_yaw(7.0)) <= math.pi


# ---------------------------------------------------------------------------
# 三、核心不变量：分解后 base 必须落在 SLAM 说的位置
#
# 这是整个模块存在的理由。"以 aft_mapped 为准"这个决策的含义就是：
# 无论 SDK 里程计漂到哪里，(map→odom) ∘ (odom→base) 必须等于 SLAM 的 (map→base)。
# ---------------------------------------------------------------------------
def test_decomposition_puts_base_where_slam_says():
    d = MapOdomDecomposer()
    map_to_base = Pose2D(3.0, -1.0, 0.8)
    odom_to_base = Pose2D(2.5, -0.5, 0.6)      # SDK 已经漂了
    map_to_odom, _, _ = d.update(map_to_base, odom_to_base)
    # 串起来必须精确回到 SLAM 的位姿
    assert_pose_close(map_to_odom.compose(odom_to_base), map_to_base)


def test_invariant_holds_over_many_random_cases():
    """随机穷举。手算几个例子挡不住符号写反、转置写错这类问题。"""
    rnd = _Lcg(seed=20260828)
    for _ in range(400):
        map_to_base = Pose2D(rnd.uniform(-50, 50), rnd.uniform(-50, 50),
                             rnd.uniform(-math.pi, math.pi))
        odom_to_base = Pose2D(rnd.uniform(-50, 50), rnd.uniform(-50, 50),
                              rnd.uniform(-math.pi, math.pi))
        d = MapOdomDecomposer()
        map_to_odom, _, _ = d.update(map_to_base, odom_to_base)
        assert_pose_close(map_to_odom.compose(odom_to_base), map_to_base, tol=1e-7)


def test_no_odom_drift_gives_identity_map_to_odom():
    """SDK 与 SLAM 完全一致时，map→odom 应是恒等 —— 否则说明有多余偏移。"""
    p = Pose2D(1.0, 2.0, 0.3)
    map_to_odom, _, _ = MapOdomDecomposer().update(p, p)
    assert_pose_close(map_to_odom, Pose2D())


def test_pure_odom_drift_shows_up_entirely_in_map_to_odom():
    """odom 漂移必须**全部**体现在 map→odom 上，odom→base 不该被改。

    这正是 REP-105 分解的目的：跳变/漂移留在 map→odom，
    odom→base 保持局部连续。
    """
    map_to_base = Pose2D(5.0, 0.0, 0.0)
    odom_to_base = Pose2D(4.8, 0.1, 0.0)       # 漂了 0.2m / 0.1m
    map_to_odom, _, _ = MapOdomDecomposer().update(map_to_base, odom_to_base)
    assert map_to_odom.x == pytest.approx(0.2)
    assert map_to_odom.y == pytest.approx(-0.1)


# ---------------------------------------------------------------------------
# 四、回环跳变的上报
#
# 跳变**不是故障** —— 回环修正本来就该体现在 map→odom 上。但要上报，
# 因为它会让 global_costmap 整体平移，那对上层是可见事件。
# ---------------------------------------------------------------------------
def test_first_update_is_never_a_jump():
    d = MapOdomDecomposer()
    _, jumped, jump_m = d.update(Pose2D(99.0, 99.0, 0.0), Pose2D())
    assert jumped is False
    assert jump_m == 0.0
    assert d.stats.jumps == 0


def test_small_change_is_not_a_jump():
    d = MapOdomDecomposer(jump_report_m=0.30)
    d.update(Pose2D(1.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0))
    _, jumped, _ = d.update(Pose2D(1.2, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0))
    assert jumped is False


def test_loop_closure_is_reported_but_still_returned():
    """跳变照常返回结果 —— 藏起来会让下游拿不到修正后的位姿。"""
    d = MapOdomDecomposer(jump_report_m=0.30)
    d.update(Pose2D(1.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0))
    result, jumped, jump_m = d.update(Pose2D(6.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0))
    assert jumped is True
    assert jump_m == pytest.approx(5.0)
    assert result.x == pytest.approx(5.0)      # 结果没被夹住/平滑
    assert d.stats.jumps == 1
    assert d.stats.max_jump_m == pytest.approx(5.0)


@pytest.mark.parametrize('bad', [0.0, -0.1])
def test_non_positive_jump_threshold_rejected(bad):
    with pytest.raises(DecompositionError) as excinfo:
        MapOdomDecomposer(jump_report_m=bad)
    assert '刷满' in str(excinfo.value)


# ---------------------------------------------------------------------------
# 五、平面性校验
# ---------------------------------------------------------------------------
def test_planar_quaternion_accepted():
    d = MapOdomDecomposer()
    assert d.check_planar(0.0, 0.0, 'SLAM') is None
    assert d.check_planar(1e-4, -1e-4, 'SLAM') is None


def test_tilted_quaternion_rejected_with_consequence():
    d = MapOdomDecomposer(max_tilt_rad=0.10)
    reason = d.check_planar(0.3, 0.2, 'SLAM')
    assert reason is not None
    # 必须说出后果，否则排查会停在"它报了个警告"
    assert '定位精度' in reason
    assert d.stats.rejected_tilt == 1


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------
def assert_pose_close(a, b, tol=1e-9):
    assert a.x == pytest.approx(b.x, abs=tol)
    assert a.y == pytest.approx(b.y, abs=tol)
    # 角度比较要考虑 ±pi 环绕
    assert abs(wrap_angle(a.theta - b.theta)) < max(tol, 1e-9)


class _Lcg:
    """自带线性同余发生器。

    刻意不用 random 模块：本仓库要求测试可复现，而一个固定 seed 的
    自实现发生器在任何 Python 版本上都给出同一串数，`random` 不保证。
    """

    def __init__(self, seed):
        self._s = seed & 0xFFFFFFFF

    def _next(self):
        self._s = (1103515245 * self._s + 12345) & 0x7FFFFFFF
        return self._s

    def uniform(self, lo, hi):
        return lo + (hi - lo) * (self._next() / 0x7FFFFFFF)
