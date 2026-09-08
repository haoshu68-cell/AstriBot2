# Copyright 2026 Astribot.
#
# path_tracking_diagnostics_node.classify 的判定表测试。
#
# 为什么这些用例值得写：classify 的结论会直接被人拿去决定"下一步查哪一层"，
# 判错一次就是一整轮排查方向错。而它是纯函数，判定表能被穷举。
#
# 最重要的一条是 wheel_known=False 那组：实机的 /joint_states 由
# joint_state_publisher 发固定姿态，**velocity 数组是空的** —— 于是 wheel_peak
# 恒为 0。如果照常判 `wheel_peak > wheel_epsilon`，那条分支在实机上恒真，
# 每一拍都报"底盘不响应"，这个"判据"永远不会说别的话。
# 恒真的判据没有信息量，而且它会盖住真正的原因。
import pytest

from astribot_s1_navigation.path_tracking_diagnostics_node import (
    ARM_COUPLING_BLOCKED,
    BODY_TO_WORLD_BLOCKED,
    CHASSIS_NOT_RESPONDING,
    CMD_BUT_NO_MOTION,
    HEALTHY,
    IDLE_NO_GOAL,
    PLANNER_NO_OUTPUT,
    SLIPPING_OR_BLOCKED,
    SMOOTHER_BLOCKED,
    STUCK_VERDICTS,
    classify,
)

# 一条"全链路健康"的基线快照。每个用例只改它关心的那几项，
# 这样断言失败时，改动的那一项就是原因，不用再去分辨是哪个字段带来的。
BASE = {
    'has_path': True,
    'raw_rate': 20.0, 'raw_peak': 0.15,
    'smoothed_rate': 20.0, 'smoothed_peak': 0.14,
    'pre_rate': 20.0, 'pre_peak': 0.14,
    'cmd_rate': 20.0, 'cmd_peak': 0.12,
    'wheel_peak': 1.5, 'robot_speed': 0.11,
    'wheel_known': True,
    'vel_epsilon': 1e-3, 'wheel_epsilon': 0.05, 'motion_epsilon': 0.02,
}


def snap(**over):
    s = dict(BASE)
    s.update(over)
    return s


def test_healthy_baseline():
    assert classify(snap()) == HEALTHY


def test_idle_without_path_is_not_a_fault():
    # 这条最容易误判：没有在途目标时全链路静默是正常的。
    assert classify(snap(has_path=False, raw_rate=0.0, raw_peak=0.0)) == IDLE_NO_GOAL
    assert IDLE_NO_GOAL not in STUCK_VERDICTS


def test_path_present_but_planner_silent():
    assert classify(snap(raw_rate=0.0, raw_peak=0.0)) == PLANNER_NO_OUTPUT


def test_stale_path_with_live_velocity_is_not_idle():
    # 有速度但 /plan 陈旧：不是空闲，链路确实在动，不能报成"无目标"。
    assert classify(snap(has_path=False)) == HEALTHY


@pytest.mark.parametrize('rate_key,peak_key,expected', [
    ('smoothed_rate', 'smoothed_peak', SMOOTHER_BLOCKED),
    ('pre_rate', 'pre_peak', BODY_TO_WORLD_BLOCKED),
    ('cmd_rate', 'cmd_peak', ARM_COUPLING_BLOCKED),
])
def test_each_downstream_stage_reports_its_own_break(rate_key, peak_key, expected):
    assert classify(snap(**{rate_key: 0.0, peak_key: 0.0})) == expected


def test_upstream_break_wins_over_downstream():
    # 下游没输出多半是上游没给，所以必须按顺序判 —— 三段同时哑掉时
    # 结论应该是最上游那一段，而不是最下游那一段。
    s = snap(smoothed_rate=0.0, smoothed_peak=0.0,
             pre_rate=0.0, pre_peak=0.0, cmd_rate=0.0, cmd_peak=0.0)
    assert classify(s) == SMOOTHER_BLOCKED


def test_rate_alone_is_not_enough():
    # 话题在流但幅值全在死区内（例如残留的 0 帧）等于没输出。
    assert classify(snap(cmd_peak=0.0)) == ARM_COUPLING_BLOCKED
    # 反过来幅值够大但一帧没来过也不算。
    assert classify(snap(cmd_rate=0.0)) == ARM_COUPLING_BLOCKED


def test_wheels_stopped_with_command_is_chassis_fault():
    assert classify(snap(wheel_peak=0.0, robot_speed=0.0)) == CHASSIS_NOT_RESPONDING


def test_wheels_turning_without_motion_is_slipping():
    assert classify(snap(robot_speed=0.0)) == SLIPPING_OR_BLOCKED


# ---- 轮速不可测（实机常态）-------------------------------------------------
def test_unknown_wheels_must_not_report_chassis_fault():
    # 这是本文件存在的主要理由。wheel_peak=0 只是因为没数据源，
    # 此时**绝不能**报"底盘不响应" —— 那个结论会把排查带到底盘层去。
    s = snap(wheel_known=False, wheel_peak=0.0, robot_speed=0.0)
    assert classify(s) == CMD_BUT_NO_MOTION
    assert classify(s) != CHASSIS_NOT_RESPONDING


def test_unknown_wheels_but_moving_is_healthy():
    # 位移是能测的，测到了就不必管轮速。
    assert classify(snap(wheel_known=False, wheel_peak=0.0)) == HEALTHY


def test_unknown_wheels_still_reports_upstream_breaks():
    # 轮速不可测只影响最后两段，不能顺带把上游的判定也一起绕过。
    s = snap(wheel_known=False, wheel_peak=0.0,
             pre_rate=0.0, pre_peak=0.0, cmd_rate=0.0, cmd_peak=0.0)
    assert classify(s) == BODY_TO_WORLD_BLOCKED


def test_cmd_but_no_motion_counts_as_stuck():
    # 它必须进 STUCK_VERDICTS，否则 only_when_stuck=True 时这条结论
    # 一次都不会打印 —— 而它恰好是实机上最可能出现的那一条。
    assert CMD_BUT_NO_MOTION in STUCK_VERDICTS


def test_epsilons_are_honoured_not_hardcoded():
    # motion_epsilon 抬高之后，原来算"在动"的速度应当变成"没动"。
    assert classify(snap(robot_speed=0.05, motion_epsilon=0.02)) == HEALTHY
    assert classify(snap(robot_speed=0.05, motion_epsilon=0.20)) == SLIPPING_OR_BLOCKED
