#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂轨迹执行核心的分支测试。全部离线（无 SDK、无 rclpy、无仿真）。

覆盖目标：start() 的每条拒绝分支、STREAMING/SETTLING 的每条退出路径、
cancel、SDK 异常转错误码、方案 A 的每个错误码。
"""

import pytest

from astribot_trajectory_bridge.arm_traj_math import ArmConfigError
from astribot_trajectory_bridge.arm_bridge_core import (
    ArmBridgeConfig,
    ArmConfigError,
    ArmTrajExecutor,
    EC_GOAL_TOLERANCE_VIOLATED,
    EC_INVALID_GOAL,
    EC_INVALID_JOINTS,
    EC_PATH_TOLERANCE_VIOLATED,
    EC_SUCCESSFUL,
    PH_ABORTED,
    PH_CANCELED,
    PH_DONE,
    PH_HOLDING,
    PH_SETTLING,
    PH_STREAMING,
    S_LIMIT_SOURCE_MISMATCH,
    S_LIMIT_VIOLATION,
    S_SDK_CALL_FAILED,
    S_SETTLE_TIMEOUT,
    S_TRACKING_ERROR_EXCEEDED,
    WaypointDispatcher,
)
from astribot_trajectory_bridge.ports import FakeClock, FakeSession

PART = 'astribot_arm_left'
JOINTS = ['astribot_arm_left_joint_%d' % i for i in range(1, 8)]
LOWER = [-3.01, -1.5, -3.14, 0.001, -2.4, -0.75, -1.57]
UPPER = [3.01, 0.25, 3.14, 2.618, 2.4, 0.75, 1.57]

Q0 = [0.0, -0.5, 0.0, 1.0, 0.0, 0.0, 0.0]
Q1 = [0.2, -0.5, 0.0, 1.2, 0.0, 0.0, 0.0]


def build(follow_ratio=1.0, **overrides):
    clock = FakeClock(50.0)
    session = FakeSession(desired={PART: list(Q0)}, current={PART: list(Q0)},
                          limits={PART: (LOWER, UPPER)},
                          follow_ratio=follow_ratio)
    kw = dict(part_name=PART, joint_names=JOINTS)
    kw.update(overrides)
    cfg = ArmBridgeConfig(**kw)
    ex = ArmTrajExecutor(cfg, session, clock)
    return (ex, session, clock)


def codes(obj):
    return [e.code for e in obj.drain_events()]


def run_to_end(ex, clock, max_ticks=5000, dt=None):
    dt = dt if dt is not None else 1.0 / ex.cfg.stream_freq
    for _ in range(max_ticks):
        if ex.phase not in (PH_STREAMING, PH_SETTLING):
            break
        ex.step()
        clock.advance(dt)
    return ex.phase


class TestConfigValidation:

    def test_defaults_accepted(self):
        ArmBridgeConfig(part_name=PART, joint_names=JOINTS)

    def test_add_default_torso_true_rejected(self):
        # SDK 默认 True 会隐式动躯干（astribot_client.py:704）
        with pytest.raises(ArmConfigError) as e:
            ArmBridgeConfig(part_name=PART, add_default_torso=True)
        assert '躯干' in str(e.value)

    def test_zero_settle_tolerance_rejected(self):
        with pytest.raises(ArmConfigError) as e:
            ArmBridgeConfig(part_name=PART, settle_tolerance_rad=0.0)
        assert '收敛判据' in str(e.value)

    def test_zero_freq_rejected(self):
        with pytest.raises(ArmConfigError):
            ArmBridgeConfig(part_name=PART, stream_freq=0.0)

    def test_zero_tracking_error_rejected(self):
        with pytest.raises(ArmConfigError):
            ArmBridgeConfig(part_name=PART, max_tracking_error_rad=0.0)


class TestLoadLimits:

    def test_loads_lower_upper_order(self):
        ex, _, _ = build()
        ok, _ = ex.load_limits()
        assert ok
        assert ex._lower == pytest.approx(LOWER)
        assert ex._upper == pytest.approx(UPPER)

    def test_reversed_limits_rejected(self):
        # 把 examples/100:49 的错误解包顺序喂进来
        ex, session, _ = build()
        session._limits[PART] = (UPPER, LOWER)
        ok, detail = ex.load_limits()
        assert not ok
        assert S_LIMIT_SOURCE_MISMATCH in codes(ex)
        assert 'examples/100' in detail

    def test_sdk_exception_reported(self):
        ex, session, _ = build()
        session.fail_after('get_joints_position_limit', 0)
        ok, _ = ex.load_limits()
        assert not ok and S_SDK_CALL_FAILED in codes(ex)

    def test_urdf_mismatch_reported_but_not_fatal_by_default(self):
        ex, _, _ = build(cross_check_urdf=True, strict_limit_check=False)
        bad = list(LOWER)
        bad[2] -= 0.5
        ok, _ = ex.load_limits(urdf_lower=bad, urdf_upper=UPPER)
        assert ok                                    # 非严格模式下继续
        assert S_LIMIT_SOURCE_MISMATCH in codes(ex)

    def test_urdf_mismatch_fatal_in_strict_mode(self):
        ex, _, _ = build(cross_check_urdf=True, strict_limit_check=True)
        bad = list(LOWER)
        bad[2] -= 0.5
        ok, _ = ex.load_limits(urdf_lower=bad, urdf_upper=UPPER)
        assert not ok


class TestStartRejections:

    def test_requires_limits_loaded(self):
        ex, _, _ = build()
        ok, ec, detail = ex.start(JOINTS, [1.0], [Q0])
        assert not ok and ec == EC_INVALID_GOAL and '限位未加载' in detail

    def test_empty_trajectory_rejected(self):
        ex, _, _ = build()
        ex.load_limits()
        ok, ec, _ = ex.start(JOINTS, [], [])
        assert not ok and ec == EC_INVALID_GOAL

    def test_length_mismatch_rejected(self):
        ex, _, _ = build()
        ex.load_limits()
        ok, ec, _ = ex.start(JOINTS, [1.0, 2.0], [Q0])
        assert not ok and ec == EC_INVALID_GOAL

    def test_wrong_joint_names_rejected(self):
        ex, _, _ = build()
        ex.load_limits()
        ok, ec, _ = ex.start(['a', 'b'], [1.0], [Q0])
        assert not ok and ec == EC_INVALID_JOINTS

    def test_reordered_joint_names_rejected(self):
        # 顺序错会让姿态全错而话题格式完全正常 —— 必须拦
        ex, _, _ = build()
        ex.load_limits()
        swapped = list(JOINTS)
        swapped[0], swapped[1] = swapped[1], swapped[0]
        ok, ec, _ = ex.start(swapped, [1.0], [Q0])
        assert not ok and ec == EC_INVALID_JOINTS

    def test_non_monotonic_times_rejected(self):
        ex, _, _ = build()
        ex.load_limits()
        ok, ec, detail = ex.start(JOINTS, [1.0, 1.0], [Q0, Q1])
        assert not ok and ec == EC_INVALID_GOAL and '非单调' in detail

    def test_too_long_trajectory_rejected(self):
        ex, _, _ = build(max_traj_duration_sec=5.0)
        ex.load_limits()
        ok, ec, _ = ex.start(JOINTS, [10.0], [Q0])
        assert not ok and ec == EC_INVALID_GOAL

    def test_out_of_limit_waypoint_rejected(self):
        ex, _, _ = build()
        ex.load_limits()
        bad = list(Q0)
        bad[3] = 5.0                     # joint_4 上限 2.618
        ok, ec, _ = ex.start(JOINTS, [1.0], [bad])
        assert not ok and ec == EC_INVALID_GOAL
        assert S_LIMIT_VIOLATION in codes(ex)

    def test_margin_makes_edge_waypoint_illegal(self):
        # joint_4 下限 0.001（SDK docstring 示例就是这个贴边值）
        ex, _, _ = build(limit_margin_rad=0.05)
        ex.load_limits()
        edge = list(Q0)
        edge[3] = 0.01
        ok, _, _ = ex.start(JOINTS, [1.0], [edge])
        assert not ok

    def test_valid_goal_enters_streaming(self):
        ex, _, _ = build()
        ex.load_limits()
        ok, ec, _ = ex.start(JOINTS, [1.0, 2.0], [Q0, Q1])
        assert ok and ec == EC_SUCCESSFUL and ex.phase == PH_STREAMING


class TestStreamingHappyPath:

    def test_completes_and_settles(self):
        ex, _, clock = build()
        ex.load_limits()
        ex.start(JOINTS, [0.1, 0.2], [Q0, Q1])
        assert run_to_end(ex, clock) == PH_DONE
        assert ex.error_code == EC_SUCCESSFUL

    def test_dispatch_params_are_correct(self):
        # 锁死三个容易回归的参数：direct / use_wbc=False / add_default_torso=False
        ex, session, clock = build()
        ex.load_limits()
        ex.start(JOINTS, [0.05], [Q1])
        run_to_end(ex, clock)
        _, _, control_way, use_wbc, add_torso = session.set_position_calls[0]
        assert control_way == 'direct'
        assert use_wbc is False
        assert add_torso is False

    def test_feedback_emitted_each_tick(self):
        ex, _, clock = build()
        ex.load_limits()
        ex.start(JOINTS, [0.04], [Q1])
        run_to_end(ex, clock)
        fb = ex.drain_feedbacks()
        assert len(fb) >= 2
        assert all(f.error >= 0.0 for f in fb)

    def test_final_position_is_last_waypoint(self):
        ex, session, clock = build()
        ex.load_limits()
        ex.start(JOINTS, [0.05], [Q1])
        run_to_end(ex, clock)
        last = session.set_position_calls[-1][1][0]
        assert last == pytest.approx(Q1)


class TestCancel:
    """方案 B 的核心优势：天然可取消。方案 A 做不到这一点。

    !!! 这组测试曾经把缺陷本身写成了断言 !!!
    旧版有一条 ``test_no_further_dispatch_after_cancel``，断言"取消后不再下发"。
    Gate 1-f 在真实 MuJoCo 后端上实测证明这个要求**是错的**：

      * 取消后只 dispatch 一次 -> 实际位置继续漂 **0.045 ~ 0.37 rad** 才停下，
        且 SDK 侧 desired 明明已经等于我们发的值 —— 指令登记上了，但
        **一次指令抓不住一个正在运动的关节**；
      * 以 stream_freq 持续重发 -> 漂移 **0.0000 rad**。

    对照组：正常完成（DONE）之后停发只掉 **0.0047 rad**，因为
    ``_step_settling`` 本来就在持续重发末点，到 DONE 时关节已静止。
    也就是说同一个 SDK 特性在正常路径上无害、在**安全路径**上有害 ——
    而取消抓不住等于取消无效。

    所以取消现在走 STREAMING/SETTLING -> **HOLDING** -> CANCELED，
    与 settling 对称；下面的断言按这个语义重写。
    """

    def test_cancel_enters_holding_not_terminal(self):
        """取消的下一拍进 HOLDING，**不是**直接落终态。"""
        ex, _, clock = build()
        ex.load_limits()
        ex.start(JOINTS, [10.0], [Q1])
        ex.step()
        clock.advance(0.004)
        ex.request_cancel()
        assert ex.step() == PH_HOLDING

    def test_cancel_holds_current_position(self):
        # 取消不能松手 —— 松手会让手臂受重力下坠
        ex, session, clock = build()
        ex.load_limits()
        ex.start(JOINTS, [10.0], [Q1])
        for _ in range(5):
            ex.step()
            clock.advance(0.004)
        session.set_current(PART, [0.11, -0.5, 0.0, 1.05, 0.0, 0.0, 0.0])
        ex.request_cancel()
        ex.step()
        held = session.set_position_calls[-1][1][0]
        assert held == pytest.approx([0.11, -0.5, 0.0, 1.05, 0.0, 0.0, 0.0])

    def test_holding_keeps_redispatching(self):
        """!!! 与旧断言完全相反 !!! HOLDING 期必须**持续**下发。

        这是修掉"取消抓不住"的关键，所以要正面锁住：多推几拍，
        下发次数必须跟着增加。
        """
        ex, session, clock = build()
        ex.load_limits()
        ex.start(JOINTS, [10.0], [Q1])
        ex.step()
        ex.request_cancel()
        ex.step()                       # -> HOLDING
        assert ex.phase == PH_HOLDING
        n = len(session.set_position_calls)
        for _ in range(3):
            clock.advance(0.004)
            ex.step()
        assert len(session.set_position_calls) > n, (
            'HOLDING 期没有继续下发 —— 取消会抓不住正在运动的关节')

    def test_holding_always_dispatches_same_target(self):
        """保持点必须钉死在取消瞬间的位置，不许跟着实际位置漂。

        若每拍都重新读实际位置当目标，手臂会被自己的漂移一路"带着走"。
        """
        ex, session, clock = build(follow_ratio=0.0)
        ex.load_limits()
        ex.start(JOINTS, [10.0], [Q1])
        ex.step()
        ex.request_cancel()
        ex.step()
        first = list(session.set_position_calls[-1][1][0])
        for _ in range(4):
            clock.advance(0.004)
            session.set_current(PART, [v + 0.05 for v in first])   # 实际在漂
            ex.step()
        last = list(session.set_position_calls[-1][1][0])
        assert last == pytest.approx(first)

    def test_holding_ends_when_still(self):
        """实际位置连续多拍不动 -> 落 CANCELED。"""
        ex, session, clock = build(hold_still_ticks_required=3)
        ex.load_limits()
        ex.start(JOINTS, [10.0], [Q1])
        ex.step()
        ex.request_cancel()
        ex.step()
        assert ex.phase == PH_HOLDING
        # FakeSession follow_ratio=1.0 -> 实际立刻等于指令，之后就不动了
        for _ in range(6):
            clock.advance(0.004)
            ph = ex.step()
            if ph == PH_CANCELED:
                break
        assert ex.phase == PH_CANCELED

    def test_holding_times_out_and_reports(self):
        """一直不停 -> 超时落 CANCELED **并上报**，不许静默。"""
        ex, session, clock = build(hold_timeout_sec=0.05,
                                   hold_still_ticks_required=1000)
        ex.load_limits()
        ex.start(JOINTS, [10.0], [Q1])
        ex.step()
        ex.request_cancel()
        ex.step()
        moving = 0.0
        for _ in range(40):
            clock.advance(0.004)
            moving += 0.02
            session.set_current(PART, [0.11 + moving] * 7)   # 持续在动
            if ex.step() == PH_CANCELED:
                break
        assert ex.phase == PH_CANCELED
        assert S_SETTLE_TIMEOUT in codes(ex), '保持超时没有上报'

    def test_cancel_during_settling(self):
        # max_tracking_error_rad 放大：follow_ratio=0.0 时跟踪误差必然超默认阈值，
        # 会先被跟踪误差 abort，测不到 SETTLING 期的取消分支
        ex, _, clock = build(follow_ratio=0.0, max_tracking_error_rad=99.0,
                             settle_timeout_sec=100.0)
        ex.load_limits()
        ex.start(JOINTS, [0.01], [Q1])
        ex.step()
        clock.advance(0.02)
        ex.step()
        assert ex.phase == PH_SETTLING
        ex.request_cancel()
        assert ex.step() == PH_HOLDING

    def test_cancel_falls_terminal_when_actual_unreadable(self):
        """读不到实际位置就没法保持：必须上报并落终态，不许假装保持住了。"""
        ex, session, clock = build()
        ex.load_limits()
        ex.start(JOINTS, [10.0], [Q1])
        ex.step()
        session.fail_after('get_current_joints_position', 0)
        ex.request_cancel()
        assert ex.step() == PH_CANCELED
        assert S_SDK_CALL_FAILED in codes(ex)

    def test_start_resets_holding_state(self):
        """新轨迹必须清掉上一次的 HOLDING 计数，否则会提前判定停稳。"""
        ex, session, clock = build(hold_still_ticks_required=3)
        ex.load_limits()
        ex.start(JOINTS, [10.0], [Q1])
        ex.step(); ex.request_cancel(); ex.step()
        for _ in range(5):
            clock.advance(0.004); ex.step()
        assert ex.phase == PH_CANCELED
        ok, _, _ = ex.start(JOINTS, [10.0], [Q1])
        assert ok and ex.phase == PH_STREAMING
        assert ex._hold_still_ticks == 0      # noqa: SLF001
        assert ex._hold_target is None        # noqa: SLF001


class TestTrackingError:

    def test_aborts_when_exceeded(self):
        ex, session, clock = build(follow_ratio=0.0, max_tracking_error_rad=0.05,
                                   abort_on_tracking_error=True)
        ex.load_limits()
        ex.start(JOINTS, [1.0], [Q1])       # 目标偏离 0.2rad，实际不跟
        run_to_end(ex, clock)
        assert ex.phase == PH_ABORTED
        assert ex.error_code == EC_PATH_TOLERANCE_VIOLATED
        assert S_TRACKING_ERROR_EXCEEDED in codes(ex)

    def test_only_warns_when_abort_disabled(self):
        ex, _, clock = build(follow_ratio=0.0, max_tracking_error_rad=0.05,
                             abort_on_tracking_error=False,
                             settle_timeout_sec=0.05)
        ex.load_limits()
        ex.start(JOINTS, [0.02], [Q1])
        phase = run_to_end(ex, clock)
        # 不因跟踪误差 abort，但最终会因收敛超时 abort（实际位置根本没动）
        assert phase == PH_ABORTED
        ev = codes(ex)
        assert S_TRACKING_ERROR_EXCEEDED in ev
        assert S_SETTLE_TIMEOUT in ev


class TestSettling:
    """锁死"控制器报完成但手臂还在收敛"这个已实测的坑。"""

    def test_not_done_before_settled(self):
        ex, _, clock = build(follow_ratio=0.0, max_tracking_error_rad=99.0,
                             settle_timeout_sec=100.0)
        ex.load_limits()
        ex.start(JOINTS, [0.01], [Q1])
        ex.step()
        clock.advance(0.02)
        ex.step()
        # 末点已发完但实际没到 -> 必须是 SETTLING，不能是 DONE
        assert ex.phase == PH_SETTLING

    def test_settle_timeout_aborts(self):
        ex, _, clock = build(follow_ratio=0.0, max_tracking_error_rad=99.0,
                             settle_timeout_sec=0.1)
        ex.load_limits()
        ex.start(JOINTS, [0.01], [Q1])
        assert run_to_end(ex, clock) == PH_ABORTED
        assert ex.error_code == EC_GOAL_TOLERANCE_VIOLATED
        assert S_SETTLE_TIMEOUT in codes(ex)

    def test_settles_when_actual_arrives(self):
        ex, session, clock = build(follow_ratio=0.0, max_tracking_error_rad=99.0,
                                   settle_timeout_sec=100.0)
        ex.load_limits()
        ex.start(JOINTS, [0.01], [Q1])
        ex.step()
        clock.advance(0.02)
        ex.step()
        assert ex.phase == PH_SETTLING
        session.set_current(PART, list(Q1))       # 手臂终于到位
        assert ex.step() == PH_DONE

    def test_keeps_dispatching_last_point_while_settling(self):
        # 收敛期不继续发末点，控制器可能停在中途
        ex, session, clock = build(follow_ratio=0.0, max_tracking_error_rad=99.0,
                                   settle_timeout_sec=100.0)
        ex.load_limits()
        ex.start(JOINTS, [0.01], [Q1])
        ex.step()
        clock.advance(0.02)
        ex.step()
        n = len(session.set_position_calls)
        ex.step()
        assert len(session.set_position_calls) == n + 1
        assert session.set_position_calls[-1][1][0] == pytest.approx(Q1)


class TestSdkFailureDuringExecution:

    def test_dispatch_failure_aborts_with_code(self):
        ex, session, clock = build()
        ex.load_limits()
        ex.start(JOINTS, [1.0], [Q1])
        session.fail_after('set_joints_position', 0)
        ex.step()
        assert ex.phase == PH_ABORTED
        assert ex.error_code == EC_PATH_TOLERANCE_VIOLATED
        assert S_SDK_CALL_FAILED in codes(ex)

    def test_read_failure_aborts_with_code(self):
        ex, session, clock = build()
        ex.load_limits()
        ex.start(JOINTS, [1.0], [Q1])
        session.fail_after('get_current_joints_position', 0)
        ex.step()
        assert ex.phase == PH_ABORTED
        assert S_SDK_CALL_FAILED in codes(ex)

    def test_failure_during_settling_aborts(self):
        ex, session, clock = build(follow_ratio=0.0, max_tracking_error_rad=99.0,
                                   settle_timeout_sec=100.0)
        ex.load_limits()
        ex.start(JOINTS, [0.01], [Q1])
        ex.step()
        clock.advance(0.02)
        ex.step()
        assert ex.phase == PH_SETTLING
        session.fail_after('get_current_joints_position', 0)
        ex.step()
        assert ex.phase == PH_ABORTED
        assert ex.error_code == EC_GOAL_TOLERANCE_VIOLATED


class TestWaypointDispatcherPlanA:
    """方案 A（保留接口）。每个错误码都要有测试。"""

    def make(self, enabled=True, **kw):
        clock = FakeClock(0.0)
        session = FakeSession(desired={PART: list(Q0)}, current={PART: list(Q0)},
                              limits={PART: (LOWER, UPPER)})
        cfg = ArmBridgeConfig(part_name=PART, joint_names=JOINTS, **kw)
        return (WaypointDispatcher(cfg, session, enabled=enabled), session)

    def test_disabled_by_default_is_explicit_refusal(self):
        # 静默成功会让调用方以为机器人动了
        d, session = self.make(enabled=False)
        ok, ec, detail, disp, dropped = d.dispatch([Q1], [1.0])
        assert not ok and ec == 'DISABLED_BY_CONFIG'
        assert session.waypoints_calls == []

    def test_drops_t0_point(self):
        # examples/206：t=0 由当前位置隐含，不要给 t=0 的点
        d, session = self.make()
        ok, ec, _, disp, dropped = d.dispatch([Q0, Q1], [0.0, 1.0])
        assert ok and ec == 'SUCCESS'
        assert dropped == 1 and disp == 1
        _, wps, tl, _, _ = session.waypoints_calls[0]
        assert tl == [1.0]
        assert wps[0] == [list(Q1)]

    def test_all_dropped_reported(self):
        d, _ = self.make()
        ok, ec, _, _, dropped = d.dispatch([Q0], [0.0])
        assert not ok and ec == 'NO_POINTS_AFTER_DROP' and dropped == 1

    def test_shape_mismatch(self):
        d, _ = self.make()
        ok, ec, _, _, _ = d.dispatch([Q0], [1.0, 2.0])
        assert not ok and ec == 'SHAPE_MISMATCH'

    def test_time_not_monotonic(self):
        d, _ = self.make()
        ok, ec, _, _, _ = d.dispatch([Q0, Q1], [1.0, 1.0])
        assert not ok and ec == 'TIME_NOT_MONOTONIC'

    def test_limit_violation(self):
        d, _ = self.make()
        bad = list(Q1)
        bad[3] = 9.0
        ok, ec, _, _, _ = d.dispatch([bad], [1.0], LOWER, UPPER)
        assert not ok and ec == 'LIMIT_VIOLATION'
        assert S_LIMIT_VIOLATION in codes(d)

    def test_sdk_failure(self):
        d, session = self.make()
        session.fail_after('move_joints_waypoints', 0)
        ok, ec, _, _, _ = d.dispatch([Q1], [1.0])
        assert not ok and ec == 'SDK_CALL_FAILED'
        assert S_SDK_CALL_FAILED in codes(d)

    def test_add_default_torso_false_passed(self):
        d, session = self.make()
        d.dispatch([Q1], [1.0])
        _, _, _, use_wbc, add_torso = session.waypoints_calls[0]
        assert use_wbc is False and add_torso is False


class TestRecoveryFromOutOfLimits:
    """机器人**已经越限**时必须还能被开回来。

    机器人一旦处于越限状态（外力碰撞、编码器异常、上电初始位姿越界、物理异常），
    上层给任何轨迹，路点 0（= 当前位置）都越限 -> 每条轨迹都被拒；
    而"把手臂开回合法区间"本身也需要一条轨迹 -> **死锁**。
    一个用来防止坏指令的保护，把恢复通路也一起堵死了。

    判据：路点 0 是"从哪儿出发"（测量值），不是我们挑的目标。只要它确实等于
    当前实测位置、其余路点合法、且终点的越限量不比起点更糟，就放行并上报。

    注：最初发现它的现场其实是仿真进程退化造成的假象（读数超出 MuJoCo 自己的
    硬限位，物理不可能），但**缺陷与成因无关** —— 真机上外力/编码器/初始位姿
    都会造成同样的越限状态。已在真的越限的机器人上验证过恢复通路。
    """

    OOB = [0.0, 0.0, 0.0, 1.0, 0.0, 1.04, 0.0]      # j6=1.04 越出 [-0.76, 0.76]
    LEGAL = [0.0, 0.0, 0.0, 1.0, 0.0, 0.5, 0.0]

    def _build_oob(self, **kw):
        ex, session, clock = build(**kw)
        ex.load_limits()
        session.set_current(PART, list(self.OOB))
        ex.drain_events()
        return ex, session, clock

    def test_recovery_trajectory_accepted(self):
        ex, _, _ = self._build_oob()
        ok, ec, detail = ex.start(JOINTS, [0.0, 1.0],
                                  [list(self.OOB), list(self.LEGAL)])
        assert ok, '越限状态下无法启动恢复轨迹 —— 机器人被永久锁死：%s' % detail

    def test_recovery_is_reported_not_silent(self):
        """从非法状态出发不是正常工况，必须上报。"""
        ex, _, _ = self._build_oob()
        ex.start(JOINTS, [0.0, 1.0], [list(self.OOB), list(self.LEGAL)])
        assert S_LIMIT_VIOLATION in codes(ex)

    def test_start_not_matching_actual_still_rejected(self):
        """路点 0 越限**且**不等于当前位置：那是个真的坏目标，必须拒。"""
        ex, session, _ = self._build_oob()
        far = list(self.OOB)
        far[5] = 2.0          # 越限，且离实测的 1.04 很远
        ok, ec, _ = ex.start(JOINTS, [0.0, 1.0], [far, list(self.LEGAL)])
        assert not ok and ec == EC_INVALID_GOAL

    def test_later_waypoint_oob_still_rejected(self):
        """只有路点 0 享受这个例外；中途/终点越限照拒。"""
        ex, _, _ = self._build_oob()
        bad_end = list(self.LEGAL)
        bad_end[5] = 1.5
        ok, ec, _ = ex.start(JOINTS, [0.0, 1.0], [list(self.OOB), bad_end])
        assert not ok and ec == EC_INVALID_GOAL

    def test_trajectory_making_violation_worse_rejected(self):
        """起点越限、终点越限更严重 -> 不是恢复，是继续往外走。"""
        ex, _, _ = self._build_oob()
        # 终点在限内但另一个关节越限更多：构造"总越限量变大"
        worse = list(self.OOB)
        worse[5] = 1.04
        worse[3] = 3.0        # j4 上限 2.61 -> 越限 0.39，总量变大
        ok, ec, _ = ex.start(JOINTS, [0.0, 1.0], [list(self.OOB), worse])
        assert not ok and ec == EC_INVALID_GOAL

    def test_can_be_disabled_by_config(self):
        """保留一键关掉的开关（回归/严格模式用）。"""
        ex, _, _ = self._build_oob(allow_recovery_from_oob=False)
        ok, ec, _ = ex.start(JOINTS, [0.0, 1.0],
                             [list(self.OOB), list(self.LEGAL)])
        assert not ok and ec == EC_INVALID_GOAL

    def test_legal_start_unaffected(self):
        """正常情况（起点合法）行为完全不变。"""
        ex, _, _ = build()
        ex.load_limits()
        ok, _, _ = ex.start(JOINTS, [0.0, 1.0], [list(self.LEGAL), list(self.LEGAL)])
        assert ok

    def test_tolerance_validated(self):
        with pytest.raises(ArmConfigError):
            ArmBridgeConfig(oob_start_tolerance_rad=0.0)
        with pytest.raises(ArmConfigError):
            ArmBridgeConfig(oob_start_tolerance_rad=1.0)
