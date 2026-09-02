#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench/driver.py 与 bench/truth.py 里纯函数部分的离线单测（不需要 ROS 运行时）。"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# driver/truth 顶部 import 了 geometry_msgs / nav_msgs，需要 source ROS 环境。
# 没有 ROS 时整个文件跳过，而不是让它红成"代码有错"。
pytest.importorskip('geometry_msgs', reason='需要 source /opt/ros/humble/setup.bash')
pytest.importorskip('nav_msgs', reason='需要 source /opt/ros/humble/setup.bash')

from bench.driver import brake_in_time_profile          # noqa: E402
from bench.truth import (Pose2D, displacement, unwrap_delta,   # noqa: E402
                         wrap_angle, yaw_from_quaternion)


# ---------------- brake_in_time_profile ----------------

def test_profile_accelerates_from_rest():
    v = brake_in_time_profile(remaining=10.0, v_max=0.5, accel=2.5, v_now=0.0, dt=0.02)
    assert math.isclose(v, 0.05, rel_tol=1e-9)          # accel*dt


def test_profile_respects_v_max():
    v = 0.0
    for _ in range(1000):
        v = brake_in_time_profile(10.0, 0.5, 2.5, v, 0.02)
    assert math.isclose(v, 0.5, rel_tol=1e-9)


def test_profile_brakes_so_it_can_stop_in_the_remaining_distance():
    """核心性质：本周期走完之后仍然刹得住（离散安全，不是连续曲线那条）。"""
    v, traveled, amount = 0.0, 0.0, 1.0
    dt, accel, v_max = 0.01, 2.5, 0.5
    adt = accel * dt
    steps = 0
    while steps < 100000:
        remaining = amount - traveled
        v = brake_in_time_profile(remaining, v_max, accel, v, dt)
        bound = -adt + math.sqrt(adt * adt + 2.0 * accel * abs(remaining))
        assert abs(v) <= bound + 1e-9, (
            f'剩余 {remaining:.6f} 时速度 {v:.6f} 超过离散安全上限 {bound:.6f}')
        traveled += v * dt
        if abs(remaining) < 1e-4 and abs(v) < 1e-4:
            break
        steps += 1
    assert steps < 100000, '规划没有收敛'
    assert math.isclose(traveled, amount, abs_tol=2e-3)


def test_profile_never_overshoots():
    """过冲对开环积分位置的底盘意味着指令要往回收，是漂移累积的来源。"""
    for amount in (0.02, 0.1, 1.0, 5.0):
        for dt in (0.004, 0.01, 0.02):
            v, traveled = 0.0, 0.0
            for _ in range(200000):
                remaining = amount - traveled
                v = brake_in_time_profile(remaining, 0.5, 2.5, v, dt)
                traveled += v * dt
                if abs(remaining) < 1e-4 and abs(v) < 1e-4:
                    break
            assert traveled <= amount + 1e-3, (
                f'amount={amount} dt={dt} 过冲到 {traveled:.6f}')


def test_profile_rejects_bad_dt():
    with pytest.raises(ValueError):
        brake_in_time_profile(1.0, 0.5, 2.5, 0.0, 0.0)


def test_profile_short_distance_degenerates_to_triangle():
    """距离短到跑不满 v_max 时，峰值速度应显著小于 v_max。"""
    v, traveled, amount = 0.0, 0.0, 0.02
    peak = 0.0
    for _ in range(10000):
        remaining = amount - traveled
        v = brake_in_time_profile(remaining, 0.5, 2.5, v, 0.01)
        peak = max(peak, abs(v))
        traveled += v * dt if (dt := 0.01) else 0.0
        if abs(remaining) < 1e-4 and abs(v) < 1e-4:
            break
    assert peak < 0.5
    assert math.isclose(traveled, amount, abs_tol=2e-3)


def test_profile_handles_negative_direction():
    v = brake_in_time_profile(-10.0, 0.5, 2.5, 0.0, 0.02)
    assert v < 0.0


def test_profile_freeze_only_decelerates():
    v = brake_in_time_profile(10.0, 0.5, 2.5, 0.4, 0.02, freeze=True)
    assert v < 0.4


def test_profile_rejects_bad_accel():
    with pytest.raises(ValueError):
        brake_in_time_profile(1.0, 0.5, 0.0, 0.0, 0.02)


# ---------------- 角度与位移 ----------------

def test_wrap_angle_range():
    assert math.isclose(wrap_angle(0.0), 0.0)
    assert math.isclose(wrap_angle(math.pi), math.pi)
    assert math.isclose(wrap_angle(-math.pi), math.pi)      # 归到 (-pi, pi]
    assert math.isclose(wrap_angle(3.0 * math.pi), math.pi)
    assert math.isclose(wrap_angle(1.5 * math.pi), -0.5 * math.pi)


def test_unwrap_delta_takes_short_path():
    assert math.isclose(unwrap_delta(0.1, -0.1), 0.2, abs_tol=1e-12)
    # 跨 ±pi：从 3.1 到 -3.1 的最短路径是 +0.083，不是 -6.2
    d = unwrap_delta(-3.1, 3.1)
    assert 0.0 < d < 0.1


def test_displacement_is_expressed_in_start_body_frame():
    """起点朝 +y（yaw=90°），世界系里往 +y 走 1 m，应读成"前向 1 m、侧向 0"。"""
    p0 = Pose2D(0.0, 0.0, math.pi / 2, 0.0)
    p1 = Pose2D(0.0, 1.0, math.pi / 2, 1.0)
    fwd, lat, dyaw = displacement(p0, p1)
    assert math.isclose(fwd, 1.0, abs_tol=1e-9)
    assert math.isclose(lat, 0.0, abs_tol=1e-9)
    assert math.isclose(dyaw, 0.0, abs_tol=1e-9)


def test_displacement_detects_lateral_drift():
    p0 = Pose2D(0.0, 0.0, 0.0, 0.0)
    p1 = Pose2D(1.0, 0.05, 0.0, 1.0)
    fwd, lat, _ = displacement(p0, p1)
    assert math.isclose(fwd, 1.0, abs_tol=1e-9)
    assert math.isclose(lat, 0.05, abs_tol=1e-9)


def test_yaw_from_quaternion_identity_and_90deg():
    class Q:
        def __init__(self, x, y, z, w):
            self.x, self.y, self.z, self.w = x, y, z, w

    assert math.isclose(yaw_from_quaternion(Q(0, 0, 0, 1)), 0.0, abs_tol=1e-12)
    h = math.sqrt(0.5)
    assert math.isclose(yaw_from_quaternion(Q(0, 0, h, h)), math.pi / 2, abs_tol=1e-9)
