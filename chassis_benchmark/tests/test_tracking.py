#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench/tracking.py 的离线单测。不需要机器人、不需要 ROS。

**这一层的价值**：控制律的正确性在这里被证明，所以到机器人上出问题时
可以直接排除"律写错了"，剩下的就是底盘的事。仿真里分不清这两者。

除了逐个函数的性质，还有两个**闭环仿真**用例：用理想运动学积分器跑完整任务，
断言收敛、不过冲、横向误差有界。理想积分器不含动力学，所以它测的正是控制律本身。
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bench import tracking as tr    # noqa: E402


# ---------------------------- 几何基础 ----------------------------

def test_straight_path_endpoints_and_spacing():
    p = tr.straight_path(0.0, 0.0, 0.3, 0.3, step=0.02)
    assert p[0] == (0.0, 0.0)
    assert math.isclose(p[-1][0], 0.3, abs_tol=1e-12)
    assert math.isclose(p[-1][1], 0.3, abs_tol=1e-12)
    gaps = [math.hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1])
            for i in range(len(p) - 1)]
    assert max(gaps) <= 0.02 + 1e-9


def test_straight_path_degenerate_and_bad_step():
    assert tr.straight_path(1.0, 2.0, 1.0, 2.0) == [(1.0, 2.0)]
    with pytest.raises(ValueError):
        tr.straight_path(0, 0, 1, 1, step=0.0)


def test_to_body_rotates_correctly():
    # 机器人在原点朝 +y（yaw=90°），世界系 (0,1) 应读成本体系 (1,0)
    bx, by = tr.to_body(0.0, 1.0, 0.0, 0.0, math.pi / 2)
    assert math.isclose(bx, 1.0, abs_tol=1e-9)
    assert math.isclose(by, 0.0, abs_tol=1e-9)


def test_circle_segment_intersection_picks_far_root():
    # 线段 (0,0)->(2,0)，圆心原点半径 1 -> 交点应是 (1,0)
    hit = tr.circle_segment_intersection((0.0, 0.0), (2.0, 0.0), (0.0, 0.0), 1.0)
    assert hit is not None
    assert math.isclose(hit[0], 1.0, abs_tol=1e-9)


def test_circle_segment_intersection_none_when_too_far():
    assert tr.circle_segment_intersection((5.0, 5.0), (6.0, 5.0),
                                          (0.0, 0.0), 1.0) is None


def test_circle_segment_intersection_degenerate_segment():
    assert tr.circle_segment_intersection((1.0, 0.0), (1.0, 0.0),
                                          (0.0, 0.0), 1.0) is None


# ---------------- 问题 1：前视距离比路径还长 ----------------

def test_nominal_lookahead_exceeds_short_path_length():
    """先把问题本身钉住：前视距离比 30 cm 任务的整条路径还长/等长。"""
    diag = math.hypot(0.3, 0.3)
    assert tr.LOOKAHEAD_DIST > diag                       # 0.6 > 0.4243，斜向全程
    # min_lookahead_dist **恰好等于** 单轴 30 cm 全程 —— 连前视下限都吃掉整条路径
    assert math.isclose(tr.MIN_LOOKAHEAD_DIST, 0.30, abs_tol=1e-12)
    assert tr.MIN_LOOKAHEAD_DIST >= 0.30


def test_effective_lookahead_shrinks_with_remaining():
    assert math.isclose(tr.effective_lookahead(10.0), tr.LOOKAHEAD_DIST)
    assert math.isclose(tr.effective_lookahead(0.4), 0.2, abs_tol=1e-9)   # 0.5*0.4
    assert math.isclose(tr.effective_lookahead(0.02), tr.LOOKAHEAD_FLOOR)
    assert math.isclose(tr.effective_lookahead(0.0), tr.LOOKAHEAD_FLOOR)


def test_shrunk_lookahead_keeps_carrot_off_the_path_end():
    """收缩前视后，carrot 在中途不应该退化成"路径末点"。"""
    path = tr.straight_path(0.0, 0.0, 0.3, 0.3, step=0.02)
    robot = (0.0, 0.0)
    # 标称前视 0.6 > 全程 0.424 -> 必然取末点（问题本身）
    c_nom, _ = tr.find_carrot(path, robot, tr.LOOKAHEAD_DIST)
    assert c_nom == path[-1]
    # 收缩后的前视 -> carrot 落在路径中间
    L = tr.effective_lookahead(math.hypot(0.3, 0.3))
    c_eff, _ = tr.find_carrot(path, robot, L)
    assert c_eff != path[-1]
    assert math.isclose(math.hypot(*c_eff), L, abs_tol=1e-6)


def test_find_carrot_falls_back_to_end_point():
    path = tr.straight_path(0.0, 0.0, 0.1, 0.0, step=0.02)
    c, i = tr.find_carrot(path, (0.0, 0.0), 5.0)
    assert c == path[-1] and i == len(path) - 1


def test_find_carrot_rejects_empty_path():
    with pytest.raises(ValueError):
        tr.find_carrot([], (0.0, 0.0), 0.1)


# ---------------- 限速律 ----------------

def test_curvature_zero_when_carrot_straight_ahead():
    assert math.isclose(tr.curvature_from_carrot((0.5, 0.0), 0.5), 0.0)


def test_curvature_sign_follows_lateral_offset():
    assert tr.curvature_from_carrot((0.5, 0.1), 0.5) > 0.0
    assert tr.curvature_from_carrot((0.5, -0.1), 0.5) < 0.0


def test_curvature_regulation_floors_at_min_speed():
    # 半径 0.1 << min_radius 0.9 -> 应被压到 min_speed 0.25
    v = tr.regulate_speed_by_curvature(0.5, 1.0 / 0.1)
    assert math.isclose(v, tr.REGULATED_MIN_SPEED, abs_tol=1e-9)


def test_curvature_regulation_no_op_on_gentle_curve():
    v = tr.regulate_speed_by_curvature(0.5, 1.0 / 2.0)   # 半径 2 > 0.9
    assert math.isclose(v, 0.5)


def test_approach_regulation_has_a_floor_that_never_reaches_zero():
    """RPP 的结构特征：到点前速度恒 >= min_approach_linear_velocity。"""
    for d in (0.6, 0.3, 0.1, 0.01, 0.0):
        v = tr.regulate_speed_by_approach(0.5, d)
        assert v >= tr.MIN_APPROACH_LINEAR_VELOCITY - 1e-12
    assert math.isclose(tr.regulate_speed_by_approach(0.5, 0.0),
                        tr.MIN_APPROACH_LINEAR_VELOCITY)


def test_brake_speed_cap_reaches_zero_unlike_approach_regulation():
    """omni 律用的这条能收到 0 —— 与上一个用例正好对照。"""
    assert math.isclose(tr.brake_speed_cap(0.0, 2.5, 0.05), 0.0, abs_tol=1e-12)
    assert tr.brake_speed_cap(0.001, 2.5, 0.05) < tr.MIN_APPROACH_LINEAR_VELOCITY


def test_brake_speed_cap_rejects_bad_args():
    with pytest.raises(ValueError):
        tr.brake_speed_cap(1.0, 0.0, 0.05)
    with pytest.raises(ValueError):
        tr.brake_speed_cap(1.0, 2.5, 0.0)


# ---------------- 两套律的结构性差异 ----------------

def test_rpp_rotates_in_place_for_a_45deg_diagonal_goal():
    """(0.3, 0.3) 的方向角 45° = 0.785 rad，正好等于 rotate_to_heading_min_angle。

    这就是"RPP 在斜向目标上先原地转 45°"的来源。
    """
    path = tr.straight_path(0.0, 0.0, 0.3, 0.3, step=0.02)
    twist, info = tr.rpp_step((0.0, 0.0, 0.0), path, dt=0.05)
    assert info['rotating_in_place'] is True
    assert math.isclose(twist[0], 0.0)          # 不前进
    assert twist[1] == 0.0                      # 结构上没有 vy
    assert twist[2] > 0.0                       # 朝 +yaw 转


def test_rpp_never_commands_lateral_velocity():
    path = tr.straight_path(0.0, 0.0, 1.0, 0.2, step=0.02)
    for yaw in (0.0, 0.3, -0.3, 1.0):
        twist, _ = tr.rpp_step((0.0, 0.0, yaw), path, dt=0.05)
        assert twist[1] == 0.0


def test_omni_goes_diagonally_without_rotating():
    """同样的斜向目标，全向律应直接斜着走、几乎不转。"""
    path = tr.straight_path(0.0, 0.0, 0.3, 0.3, step=0.02)
    twist, info = tr.omni_step((0.0, 0.0, 0.0), path, dt=0.05)
    assert info['rotating_in_place'] is False
    assert twist[0] > 0.0 and twist[1] > 0.0
    assert math.isclose(twist[0], twist[1], rel_tol=1e-6)     # 45° -> vx == vy
    assert math.isclose(twist[2], 0.0, abs_tol=1e-9)          # 航向已在参考值上


def test_omni_holds_reference_yaw():
    path = tr.straight_path(0.0, 0.0, 0.3, 0.0, step=0.02)
    twist, info = tr.omni_step((0.0, 0.0, 0.2), path, dt=0.05, reference_yaw=0.0)
    assert info['yaw_error'] < 0.0            # 当前 yaw 偏正 -> 误差为负
    assert twist[2] < 0.0                     # 往回转


def test_omni_respects_chassis_limits():
    path = tr.straight_path(0.0, 0.0, 50.0, 0.0, step=0.5)
    twist, _ = tr.omni_step((0.0, 0.0, 0.0), path, dt=0.05,
                            desired_linear_vel=10.0)
    assert math.hypot(twist[0], twist[1]) <= tr.MAX_VEL_XY + 1e-9


# ---------------- 横向误差 ----------------

def test_cross_track_error_zero_on_path():
    path = tr.straight_path(0.0, 0.0, 1.0, 0.0, step=0.05)
    assert math.isclose(tr.cross_track_error((0.5, 0.0), path), 0.0, abs_tol=1e-12)


def test_cross_track_error_perpendicular_offset():
    path = tr.straight_path(0.0, 0.0, 1.0, 0.0, step=0.05)
    assert math.isclose(tr.cross_track_error((0.5, 0.07), path), 0.07, abs_tol=1e-9)


def test_cross_track_error_beyond_endpoint_uses_endpoint():
    path = tr.straight_path(0.0, 0.0, 1.0, 0.0, step=0.05)
    assert math.isclose(tr.cross_track_error((1.3, 0.0), path), 0.3, abs_tol=1e-9)


# ---------------- 闭环：理想运动学积分（不含动力学，测的是控制律本身）----------------

def _simulate(law, start, goal, dt=1.0 / tr.CONTROLLER_FREQUENCY_HZ,
              max_steps=4000, stop_tol=0.02, desired_linear_vel=0.5):
    """完美执行假设下的闭环仿真：下发的 twist 直接成为真实速度。"""
    path = tr.straight_path(start[0], start[1], goal[0], goal[1], step=0.02)
    x, y, yaw = start
    ref_yaw = start[2]
    idx = 0
    xte_max = 0.0
    max_dist_beyond = 0.0
    total = math.hypot(goal[0] - start[0], goal[1] - start[1])
    for step in range(max_steps):
        d = math.hypot(goal[0] - x, goal[1] - y)
        if d <= stop_tol:
            return {'converged': True, 'steps': step, 'final_dist': d,
                    'xte_max': xte_max, 'overshoot': max_dist_beyond,
                    'final_yaw': yaw, 'path': path}
        if law is tr.rpp_step:
            twist, info = law((x, y, yaw), path, dt, from_index=idx,
                              desired_linear_vel=desired_linear_vel)
        else:
            twist, info = law((x, y, yaw), path, dt, from_index=idx,
                              reference_yaw=ref_yaw,
                              desired_linear_vel=desired_linear_vel)
        idx = info['path_index']
        vx, vy, wz = twist
        # 本体系速度 -> 世界系位移
        c, s = math.cos(yaw), math.sin(yaw)
        x += (c * vx - s * vy) * dt
        y += (s * vx + c * vy) * dt
        yaw = tr.wrap_angle(yaw + wz * dt)
        xte_max = max(xte_max, tr.cross_track_error((x, y), path))
        travelled = math.hypot(x - start[0], y - start[1])
        max_dist_beyond = max(max_dist_beyond, travelled - total)
    return {'converged': False, 'steps': max_steps,
            'final_dist': math.hypot(goal[0] - x, goal[1] - y),
            'xte_max': xte_max, 'overshoot': max_dist_beyond,
            'final_yaw': yaw, 'path': path}


def test_omni_closed_loop_converges_on_30cm_diagonal():
    r = _simulate(tr.omni_step, (0.0, 0.0, 0.0), (0.3, 0.3))
    assert r['converged'], r
    assert r['xte_max'] < 0.01, f"横向误差 {r['xte_max']:.4f} 太大"
    assert math.isclose(r['final_yaw'], 0.0, abs_tol=1e-3)   # 航向被保持住


def test_omni_closed_loop_does_not_overshoot():
    """刹得住曲线的目的就是这个：不过冲。"""
    for goal in ((0.3, 0.0), (0.3, 0.3), (0.0, 0.3), (1.0, 0.0)):
        r = _simulate(tr.omni_step, (0.0, 0.0, 0.0), goal, stop_tol=0.005)
        assert r['converged'], (goal, r)
        assert r['overshoot'] < 0.01, f"{goal} 过冲 {r['overshoot']:.4f}"


def test_omni_closed_loop_rejects_initial_cross_track_offset():
    """起点偏离路径 5 cm，闭环应把它拉回来。"""
    path_goal = (0.5, 0.0)
    r = _simulate(tr.omni_step, (0.0, 0.05, 0.0), path_goal, stop_tol=0.01)
    assert r['converged'], r
    assert r['final_dist'] <= 0.01


def test_rpp_closed_loop_converges_but_rotates_first():
    r = _simulate(tr.rpp_step, (0.0, 0.0, 0.0), (0.3, 0.3), stop_tol=0.05,
                  desired_linear_vel=0.3)
    assert r['converged'], r
    # 差速律必须把车头转到运动方向上：终态 yaw 应接近 45°
    assert abs(tr.wrap_angle(r['final_yaw'] - math.pi / 4)) < 0.35, r['final_yaw']


def test_rpp_and_omni_differ_in_final_heading():
    """同一条路径、同一组参数，两套律的终态航向应显著不同。

    这是 e3 脚本要在真机/仿真上复现的判别量。
    """
    ro = _simulate(tr.omni_step, (0.0, 0.0, 0.0), (0.3, 0.3), stop_tol=0.02)
    rr = _simulate(tr.rpp_step, (0.0, 0.0, 0.0), (0.3, 0.3), stop_tol=0.05,
                   desired_linear_vel=0.3)
    assert ro['converged'] and rr['converged']
    assert abs(ro['final_yaw']) < 0.05
    assert abs(rr['final_yaw']) > 0.5
