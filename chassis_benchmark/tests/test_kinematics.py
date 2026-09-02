#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench/kinematics.py 的离线单测。不需要机器人、不需要 ROS。

这里断言的是**测试方案 §1.5 的三个预测**。如果哪天有人改了
omni_effort_drive_node 的系数矩阵却没同步这里，这些用例会红——
那正是本文件存在的理由（两处漂移谁都不报错）。
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bench import kinematics as k    # noqa: E402

D45 = math.radians(45.0)
D90 = math.radians(90.0)


def test_rolling_directions_are_unit_vectors():
    for ax, ay in k.wheel_rolling_directions():
        assert math.isclose(math.hypot(ax, ay), 1.0, abs_tol=1e-4)


def test_x_translation_drives_all_four_wheels_equally():
    w = k.wheel_speeds(1.0, 0.0, 0.0)
    assert k.driving_wheel_count(0.0) == 4
    for wi in w:
        assert math.isclose(abs(wi), 0.7071 / k.WHEEL_RADIUS_M, rel_tol=1e-3)


def test_prediction_1_at_45deg_only_two_wheels_turn():
    """§1.5 预测 1：沿 45° 走时轮 1、2 完全静止。"""
    w = k.wheel_speeds(math.cos(D45), math.sin(D45), 0.0)
    assert k.driving_wheel_count(D45) == 2
    assert math.isclose(w[1], 0.0, abs_tol=1e-9)
    assert math.isclose(w[2], 0.0, abs_tol=1e-9)
    assert math.isclose(abs(w[0]), 1.0 / k.WHEEL_RADIUS_M, rel_tol=1e-3)
    assert math.isclose(abs(w[3]), 1.0 / k.WHEEL_RADIUS_M, rel_tol=1e-3)


def test_prediction_1_wheel_speed_is_41pct_higher_at_45deg():
    ratio = (k.max_wheel_speed_for_unit_translation(D45)
             / k.max_wheel_speed_for_unit_translation(0.0))
    assert math.isclose(ratio, math.sqrt(2.0), rel_tol=1e-3)


def test_prediction_1_force_anisotropy_is_one_over_sqrt2():
    """§1.5 预测 1：45° 可用驱动力只有 0° 的 1/sqrt(2)。"""
    assert math.isclose(k.max_force_along(0.0), 530.33, rel_tol=1e-3)
    assert math.isclose(k.max_force_along(D45), 375.0, rel_tol=1e-3)
    assert math.isclose(k.max_force_along(D90), 530.33, rel_tol=1e-3)
    assert math.isclose(k.force_anisotropy_ratio(D45), 1.0 / math.sqrt(2.0), rel_tol=1e-3)
    assert math.isclose(k.force_anisotropy_ratio(D90), 1.0, rel_tol=1e-3)


def test_prediction_2_nav2_box_constraint_allows_more_where_platform_is_weaker():
    """§1.5 预测 2：箱式约束在最弱方向上允许最大的幅值。"""
    assert math.isclose(k.nav2_commanded_accel_magnitude(0.0), 2.5, rel_tol=1e-6)
    assert math.isclose(k.nav2_commanded_accel_magnitude(D45),
                        math.sqrt(2.5 ** 2 * 2), rel_tol=1e-3)
    # 需求/能力：0° 处应为 1（配置恰好等于代理能力），45° 处应为 2
    assert math.isclose(k.accel_demand_over_capability(0.0), 1.0, rel_tol=1e-3)
    assert math.isclose(k.accel_demand_over_capability(D45), 2.0, rel_tol=1e-2)


def test_prediction_3_pure_rotation_is_self_consistent():
    """§1.5 预测 3：正逆运动学同矩阵 -> 纯 wz 指令不产生寄生平移。

    做法：把纯 wz 的轮速用最小二乘反解回车体速度，vx/vy 必须为 0。
    """
    wz = 1.0
    w = k.wheel_speeds(0.0, 0.0, wz)
    # 最小二乘反解 A v = w * r
    a = [[k.WHEEL_COEFF_VX[i], k.WHEEL_COEFF_VY[i], k.WHEEL_COEFF_WZ[i]]
         for i in range(k.N_WHEELS)]
    rhs = [wi * k.WHEEL_RADIUS_M for wi in w]
    # 正规方程 (A^T A) v = A^T b，手写 3x3 消元，避免引入 numpy 依赖
    ata = [[sum(a[m][i] * a[m][j] for m in range(4)) for j in range(3)] for i in range(3)]
    atb = [sum(a[m][i] * rhs[m] for m in range(4)) for i in range(3)]
    v = _solve3(ata, atb)
    assert math.isclose(v[0], 0.0, abs_tol=1e-9), f'寄生 vx = {v[0]}'
    assert math.isclose(v[1], 0.0, abs_tol=1e-9), f'寄生 vy = {v[1]}'
    assert math.isclose(v[2], wz, rel_tol=1e-9)


def test_wheel_speed_limit_is_not_the_binding_constraint():
    """满速满转的组合仍应远离 wheel_velocity_limit_rad_s，说明瓶颈是力矩不是转速。

    这一条支撑"45° 的问题是力/加速度而不是转速"这个判断。
    """
    h = k.wheel_speed_headroom(k.CHASSIS_MAX_VX, k.CHASSIS_MAX_VY, k.CHASSIS_MAX_WZ)
    assert h < 0.8, f'转速余量比 {h:.3f}，转速已成瓶颈，§1.5 的推论需要重写'


def test_net_yaw_moment_zero_for_balanced_efforts():
    # 前后两对力臂不同(0.3060/0.3024)，所以"四个相等"并不是零净偏航
    assert not math.isclose(k.net_yaw_moment([1.0, 1.0, 1.0, 1.0]), 0.0, abs_tol=1e-6)
    # 前对与后对反向、按力臂配比才是零
    e = [k.WHEEL_COEFF_WZ[2], k.WHEEL_COEFF_WZ[2],
         -k.WHEEL_COEFF_WZ[0], -k.WHEEL_COEFF_WZ[0]]
    assert math.isclose(k.net_yaw_moment(e), 0.0, abs_tol=1e-9)


def test_net_yaw_moment_rejects_wrong_length():
    with pytest.raises(ValueError):
        k.net_yaw_moment([1.0, 2.0, 3.0])


def test_wheel_order_and_topic_names_match_the_node():
    """轮序错了 F2 的净偏航力矩会算成别的东西且不报错。"""
    assert k.WHEEL_ORDER == ('RF', 'LF', 'RR', 'LR')
    assert k.JOINT_NAMES == ('wheel_RF_Joint', 'wheel_LF_Joint',
                             'wheel_RR_Joint', 'wheel_LR_Joint')
    assert k.EFFORT_DEBUG_TOPICS[0] == '/wheel_effort/RF'
    assert k.SETPOINT_DEBUG_TOPICS[3] == '/wheel_velocity_setpoint/LR'
    assert len(k.EFFORT_DEBUG_TOPICS) == k.N_WHEELS


def test_front_pair_has_longer_yaw_arm_than_rear_pair():
    """前对 RF/LF = 0.3060，后对 RR/LR = 0.3024。这个 1.2% 不对称是预测 3 的来源。"""
    rf, lf, rr, lr = k.WHEEL_COEFF_WZ
    assert math.isclose(rf, lf)
    assert math.isclose(rr, lr)
    assert abs(rf) > abs(rr)
    assert math.isclose(abs(rf) / abs(rr) - 1.0, 0.0119, abs_tol=2e-3)


def test_theoretical_numbers_match_the_plan():
    """方案里引用的两个数：停车 0.05 m、上升 0.2 s。"""
    assert math.isclose(k.theoretical_stop_distance(0.5, 2.5), 0.05, rel_tol=1e-9)
    assert math.isclose(k.theoretical_rise_time(0.5, 2.5), 0.2, rel_tol=1e-9)
    with pytest.raises(ValueError):
        k.theoretical_stop_distance(0.5, 0.0)


def _solve3(m, b):
    """3x3 高斯消元（带部分选主元）。只为避免引入 numpy 依赖。"""
    a = [row[:] + [b[i]] for i, row in enumerate(m)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(a[r][col]))
        if abs(a[piv][col]) < 1e-15:
            raise ValueError('奇异矩阵')
        a[col], a[piv] = a[piv], a[col]
        for r in range(3):
            if r == col:
                continue
            f = a[r][col] / a[col][col]
            for c in range(col, 4):
                a[r][c] -= f * a[col][c]
    return [a[i][3] / a[i][i] for i in range(3)]
