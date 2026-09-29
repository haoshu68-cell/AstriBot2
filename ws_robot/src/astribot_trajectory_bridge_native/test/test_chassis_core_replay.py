"""700-tick Python/native bridge-core replay with fake ports only."""

from astribot_trajectory_bridge import chassis_integrator as integrator
from astribot_trajectory_bridge import chassis_bridge_core as bridge_core
from astribot_trajectory_bridge.chassis_bridge_core import ChassisBridgeConfig, ChassisBridgeCore
from astribot_trajectory_bridge.ports import FakeClock, FakePose, FakeSession
from astribot_trajectory_bridge_native import _chassis_math_native as native


def _run(use_native):
    integrator._native = native if use_native else None
    bridge_core._native = native if use_native else None
    clock = FakeClock(0.0)
    pose = FakePose(clock, [0.0, 0.0, 0.0])
    session = FakeSession(
        desired={"astribot_chassis": [0.0, 0.0, 0.0]},
        current={"astribot_chassis": [0.0, 0.0, 0.0]},
        follow_ratio=0.87,
    )
    config = ChassisBridgeConfig(
        require_fresh_scan=False, pose_source="ground_truth", freq=250.0, outer_rate=10.0
    )
    core = ChassisBridgeCore(config, session, pose, clock)
    assert core.enable()[0]
    trace = []
    for tick in range(700):
        clock.advance(0.008 if tick % 113 == 0 else 0.004)
        if tick in (100, 101):
            pose.set_pose([0.002 * tick, 0.0005 * tick, 0.0002 * tick])
        if tick == 300:
            core.submit_twist(0.0, 0.0, 0.0)
        elif tick == 301:
            core.submit_twist(-0.08, 0.02, 0.03)
        else:
            core.submit_twist(0.12, -0.03, 0.04)
        ok = core.inner_tick()
        trace.append(
            (
                ok,
                core.state,
                tuple(core.pos_cmd),
                tuple(core._pose_integrator.integral),
                tuple((event.code, event.metric_1, event.metric_2) for event in core.drain_events()),
            )
        )
    return trace, session.set_position_calls


def test_chassis_core_native_replay_matches_python():
    python_trace, python_calls = _run(False)
    native_trace, native_calls = _run(True)
    assert len(python_trace) == len(native_trace) == 700
    assert len(python_calls) == len(native_calls)
    for python_tick, native_tick in zip(python_trace, native_trace):
        assert python_tick[0] == native_tick[0]
        assert python_tick[1] == native_tick[1]
        assert python_tick[2] == native_tick[2]
        assert python_tick[3] == native_tick[3]
        assert python_tick[4] == native_tick[4]
    for python_call, native_call in zip(python_calls, native_calls):
        assert python_call == native_call
