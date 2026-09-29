"""Fault-injection and boundary replay for the native chassis state machine."""

import math

from astribot_trajectory_bridge import chassis_bridge_core as bridge_core
from astribot_trajectory_bridge import chassis_integrator as integrator
from astribot_trajectory_bridge.chassis_bridge_core import (
    ChassisBridgeConfig,
    ChassisBridgeCore,
    ST_LEASH_TRIPPED,
    ST_STOPPED_NO_POSE,
    ST_STOPPED_STALE_SCAN,
)
from astribot_trajectory_bridge.ports import FakeClock, FakePose, FakeSession
from astribot_trajectory_bridge_native import _chassis_math_native as native


def _cfg(**kwargs):
    values = dict(
        freq=250.0,
        outer_rate=10.0,
        require_fresh_scan=False,
        pose_source='ground_truth',
        leash_xy_m=0.02,
        leash_theta_rad=0.08,
    )
    values.update(kwargs)
    return ChassisBridgeConfig(**values)


def _make(enabled, *, session=None, pose=None, clock=None, **cfg_kwargs):
    old_core = bridge_core._native
    old_math = integrator._native
    bridge_core._native = native if enabled else None
    integrator._native = native if enabled else None
    clock = clock or FakeClock()
    pose = pose or FakePose(clock, [0.0, 0.0, 0.0])
    session = session or FakeSession(
        desired={'astribot_chassis': [0.0, 0.0, 0.0]},
        current={'astribot_chassis': [0.0, 0.0, 0.0]},
        follow_ratio=0.87)
    core = ChassisBridgeCore(_cfg(**cfg_kwargs), session, pose, clock)
    return core, session, pose, clock, old_core, old_math


def _restore(old_core, old_math):
    bridge_core._native = old_core
    integrator._native = old_math


def _events(core):
    return [(event.code, event.metric_1, event.metric_2)
            for event in core.drain_events()]


def test_cmd_timeout_and_leash_replay_match():
    # 看门狗只置零，不应改变状态；先把这一条独立锁住。
    timeout_outputs = []
    for enabled in (False, True):
        core, session, pose, clock, old_core, old_math = _make(
            enabled, cmd_vel_timeout_sec=0.01)
        try:
            assert core.enable()[0]
            core.submit_twist(0.1, 0.0, 0.0)
            clock.advance(0.02)
            assert core.inner_tick()
            timeout_outputs.append((core.state, _events(core)))
        finally:
            _restore(old_core, old_math)
    assert timeout_outputs[0][0] == timeout_outputs[1][0] == 'ENABLED'
    assert timeout_outputs[0][1][-1][0] == timeout_outputs[1][1][-1][0] == 'CMD_VEL_TIMEOUT'
    # 新输入必须解除超时事件抑制；否则恢复后首个再次超时会被吞掉。
    timeout_recovery = []
    for enabled in (False, True):
        core, session, pose, clock, old_core, old_math = _make(
            enabled, cmd_vel_timeout_sec=0.01)
        try:
            assert core.enable()[0]
            core.submit_twist(0.1, 0.0, 0.0)
            clock.advance(0.02)
            assert core.inner_tick()
            _events(core)
            core.submit_twist(0.1, 0.0, 0.0)
            clock.advance(0.02)
            assert core.inner_tick()
            timeout_recovery.append(_events(core))
        finally:
            _restore(old_core, old_math)
    assert timeout_recovery[0][-1][0] == timeout_recovery[1][-1][0] == 'CMD_VEL_TIMEOUT'

    # 再用持续新输入覆盖 leash 路径；这样不会因看门狗先置零而掩盖 leash。
    outputs = []
    for enabled in (False, True):
        core, session, pose, clock, old_core, old_math = _make(
            enabled, session=FakeSession(
                desired={'astribot_chassis': [0.0, 0.0, 0.0]},
                current={'astribot_chassis': [0.0, 0.0, 0.0]},
                follow_ratio=0.0),
            cmd_vel_timeout_sec=1.0, leash_xy_m=0.005)
        try:
            # 无位姿时桥接仍允许开环使能；外力造成的实际位置偏差必须闩锁。
            pose.set_pose(None)
            assert core.enable()[0]
            core.submit_twist(1.0, 0.0, 0.0)
            clock.advance(0.004)
            session.set_current('astribot_chassis', [0.02, 0.0, 0.0])
            assert not core.inner_tick()
            assert core.state == ST_LEASH_TRIPPED
            clock.advance(0.1)
            core.submit_twist(0.1, 0.0, 0.0)
            core.inner_tick()
            outputs.append((core.state, tuple(core.pos_cmd), _events(core),
                            len(session.set_position_calls)))
        finally:
            _restore(old_core, old_math)
    assert outputs[0] == outputs[1]


def test_scan_stale_grace_stop_replay_match():
    outputs = []
    for enabled in (False, True):
        core, session, pose, clock, old_core, old_math = _make(
            enabled, require_fresh_scan=True, scan_max_age_sec=0.02,
            scan_loss_grace_sec=0.05)
        try:
            assert core.enable()[0]
            core.submit_scan_seen()
            core.submit_twist(0.1, 0.0, 0.0)
            for _ in range(30):
                clock.advance(0.01)
                core.inner_tick()
                if core.state == ST_STOPPED_STALE_SCAN:
                    break
            outputs.append((core.state, _events(core), len(session.set_position_calls)))
        finally:
            _restore(old_core, old_math)
    assert outputs[0] == outputs[1]
    assert outputs[0][0] == ST_STOPPED_STALE_SCAN


def test_pose_loss_grace_stop_replay_match():
    outputs = []
    for enabled in (False, True):
        clock = FakeClock()
        pose = FakePose(clock, [0.0, 0.0, 0.0])
        core, session, pose, clock, old_core, old_math = _make(
            enabled, clock=clock, pose=pose, require_slam_to_enable=True,
            slam_loss_grace_sec=0.05)
        try:
            assert core.enable()[0]
            pose.set_pose(None)
            for _ in range(20):
                clock.advance(0.01)
                core.outer_tick()
                if core.state == ST_STOPPED_NO_POSE:
                    break
            outputs.append((core.state, _events(core)))
        finally:
            _restore(old_core, old_math)
    assert outputs[0] == outputs[1]
    assert outputs[0][0] == ST_STOPPED_NO_POSE


def test_sdk_read_and_write_failures_replay_error_events():
    for failure in ('read', 'write'):
        outputs = []
        for enabled in (False, True):
            session = FakeSession(
                desired={'astribot_chassis': [0.0, 0.0, 0.0]},
                current={'astribot_chassis': [0.0, 0.0, 0.0]},
                follow_ratio=0.87)
            session.fail_after(
                'get_current_joints_position' if failure == 'read' else
                'set_joints_position', ok_calls=1 if failure == 'read' else 0)
            core, session, pose, clock, old_core, old_math = _make(
                enabled, session=session)
            try:
                assert core.enable()[0]
                core.submit_twist(0.1, 0.0, 0.0)
                clock.advance(0.004)
                core.inner_tick()
                outputs.append((core.state, _events(core),
                                len(session.set_position_calls)))
            finally:
                _restore(old_core, old_math)
        assert outputs[0] == outputs[1]
        assert outputs[0][1] and outputs[0][1][-1][0] == 'SDK_CALL_FAILED'


def test_native_tick_stats_and_pose_view_remain_finite():
    core, session, pose, clock, old_core, old_math = _make(True)
    try:
        assert core.enable()[0]
        core.submit_twist(0.05, -0.01, 0.02)
        for _ in range(20):
            clock.advance(0.004)
            core.inner_tick()
        stats = core.tick_stats()
        assert stats.count == 20
        assert math.isfinite(stats.mean_dt)
        assert core._pose_integrator is not None
        assert all(math.isfinite(v) for v in core._pose_integrator.integral)
        trace = core.consume_vel_trace()
        assert trace.ticks == 20
    finally:
        _restore(old_core, old_math)
