"""Fake-session differential replay for the native gripper controller.

The Python controller remains the oracle.  The native candidate is exercised
through the public facade, so this covers the same write-gate and error-code
boundary that the ROS service uses without starting a robot or a ROS graph.
"""

import math
import threading

from astribot_trajectory_bridge import gripper_core
from astribot_trajectory_bridge.ports import FakeClock, FakeSession, SdkCallFailure
from astribot_trajectory_bridge_native import _chassis_math_native as native


def _cfg(**kwargs):
    values = dict(
        gripper_names=['left_gripper', 'right_gripper'],
        enable_service=True,
        default_duration_sec=0.2,
        default_max_force_n=0.0,
        settle_extra_sec=0.0,
        stream_freq=20.0,
        mid_stream_tolerance=0.5,
        mid_stream_timeout_sec=0.25,
    )
    values.update(kwargs)
    return gripper_core.GripperConfig(**values)


def _controller(use_native, session=None, clock=None, sleep=None, **cfg_kwargs):
    old = gripper_core._native
    gripper_core._native = native if use_native else None
    try:
        clock = clock or FakeClock()
        session = session or FakeSession(
            current={'left_gripper': [0.0], 'right_gripper': [0.0]},
            follow_ratio=1.0, in_simulation=True)
        controller = gripper_core.GripperController(
            _cfg(**cfg_kwargs), session, sleep_fn=sleep,
            clock_fn=clock.now, in_simulation=session.in_simulation)
        return controller, session, clock, old
    except Exception:
        gripper_core._native = old
        raise


def _finish(old):
    gripper_core._native = old


def _result(result):
    return (
        result.ok, result.error_code, result.dispatched_cmd,
        result.dispatched_rad, result.actual_cmd, result.force_applied,
    )


def _events(controller):
    return [(event.code, event.detail) for event in controller.drain_events()]


def _run_case(use_native, request, session=None, **cfg_kwargs):
    controller, session, clock, old = _controller(
        use_native, session=session, **cfg_kwargs)
    try:
        result = controller.execute(**request)
        return _result(result), _events(controller), list(session.set_position_calls), session
    finally:
        _finish(old)


def test_successful_open_close_raw_and_mid_open_are_equivalent():
    requests = [
        dict(name='', opening_fraction=1.0, duration=0.0,
             use_raw_cmd=False, raw_cmd=0.0, max_force=0.0,
             write_allowed=True),
        dict(name='left_gripper', opening_fraction=0.0, duration=0.3,
             use_raw_cmd=False, raw_cmd=0.0, max_force=0.0,
             write_allowed=True),
        dict(name='right_gripper', opening_fraction=0.0, duration=0.0,
             use_raw_cmd=True, raw_cmd=25.0, max_force=0.0,
             write_allowed=True),
    ]
    for request in requests:
        py, py_events, py_calls, _ = _run_case(False, request)
        cpp, cpp_events, cpp_calls, _ = _run_case(True, request)
        assert py == cpp
        assert [code for code, _ in py_events] == [code for code, _ in cpp_events]
        assert len(py_calls) == len(cpp_calls)
        if py_calls:
            assert py_calls[-1][2:] == cpp_calls[-1][2:]


def test_rejection_paths_preserve_codes_and_do_not_write():
    requests = [
        dict(name='', opening_fraction=1.0, duration=0.0,
             use_raw_cmd=False, raw_cmd=0.0, max_force=0.0,
             write_allowed=False),
        dict(name='unknown', opening_fraction=1.0, duration=0.0,
             use_raw_cmd=False, raw_cmd=0.0, max_force=0.0,
             write_allowed=True),
        dict(name='left_gripper', opening_fraction=1.2, duration=0.0,
             use_raw_cmd=False, raw_cmd=0.0, max_force=0.0,
             write_allowed=True),
        dict(name='left_gripper', opening_fraction=1.0, duration=0.0,
             use_raw_cmd=True, raw_cmd=101.0, max_force=0.0,
             write_allowed=True),
        dict(name=None, opening_fraction=None, duration=0.0,
             use_raw_cmd=False, raw_cmd=0.0, max_force=0.0,
             write_allowed=True),
        dict(name='left_gripper', opening_fraction=1.0, duration=0.0,
             use_raw_cmd=True, raw_cmd=None, max_force=0.0,
             write_allowed=True),
    ]
    for request in requests:
        py, py_events, py_calls, _ = _run_case(False, request)
        cpp, cpp_events, cpp_calls, _ = _run_case(True, request)
        assert py[:2] == cpp[:2]
        assert py_calls == cpp_calls == []
        assert [code for code, _ in py_events] == [code for code, _ in cpp_events]


def test_force_and_sdk_failures_keep_safety_result_semantics():
    for use_native in (False, True):
        session = FakeSession(
            current={'left_gripper': [0.0], 'right_gripper': [0.0]},
            follow_ratio=1.0, in_simulation=False)
        session.fail_after('set_effector_max_force', ok_calls=0)
        result, events, calls, _ = _run_case(
            use_native,
            dict(name='left_gripper', opening_fraction=1.0, duration=0.0,
                 use_raw_cmd=False, raw_cmd=0.0, max_force=3.0,
                 write_allowed=True), session=session)
        assert result[:2] == (False, gripper_core.EC_SDK_CALL_FAILED)
        assert calls == []
        assert events and events[-1][0] == gripper_core.S_SDK_CALL_FAILED

        session = FakeSession(
            current={'left_gripper': [0.0], 'right_gripper': [0.0]},
            follow_ratio=1.0, in_simulation=False)
        session.fail_after('open_effector', ok_calls=0)
        result, events, calls, _ = _run_case(
            use_native,
            dict(name='left_gripper', opening_fraction=1.0, duration=0.0,
                 use_raw_cmd=False, raw_cmd=0.0, max_force=0.0,
                 write_allowed=True), session=session)
        assert result[:2] == (False, gripper_core.EC_SDK_CALL_FAILED)
        assert events and events[-1][0] == gripper_core.S_SDK_CALL_FAILED


class _StalledSession(FakeSession):
    def set_joints_position(self, names, position, control_way='filter',
                            use_wbc=False, add_default_torso=True):
        self._maybe_fail('set_joints_position')
        self.set_position_calls.append(
            (list(names), [list(p) for p in position], control_way,
             use_wbc, add_default_torso))
        # Keep current values away from the target to force the bounded stream
        # timeout.  This is a fault-injection session, not a production port.
        return True


def test_mid_open_timeout_is_bounded_and_reports_the_same_code():
    outputs = []
    for use_native in (False, True):
        clock = FakeClock()
        session = _StalledSession(
            current={'left_gripper': [0.0], 'right_gripper': [0.0]},
            follow_ratio=0.0, in_simulation=True)

        def advance(seconds, c=clock):
            c.advance(seconds)

        result, events, calls, _ = _run_case(
            use_native,
            dict(name='left_gripper', opening_fraction=0.5, duration=0.0,
                 use_raw_cmd=False, raw_cmd=0.0, max_force=0.0,
                 write_allowed=True), session=session, clock=clock,
            sleep=advance)
        outputs.append((result, [code for code, _ in events], len(calls)))
    assert outputs[0][0][:2] == outputs[1][0][:2] == (True, gripper_core.EC_SUCCESS)
    assert outputs[0][1] == outputs[1][1] == ['SETTLE_TIMEOUT']
    assert outputs[0][2] == outputs[1][2] > 1


def test_readback_failure_reports_success_with_nan_actual_in_both_paths():
    for use_native in (False, True):
        session = FakeSession(
            current={'left_gripper': [0.0], 'right_gripper': [0.0]},
            follow_ratio=1.0, in_simulation=True)
        session.fail_after('get_current_joints_position', ok_calls=0)
        result, events, _, _ = _run_case(
            use_native,
            dict(name='left_gripper', opening_fraction=1.0, duration=0.0,
                 use_raw_cmd=False, raw_cmd=0.0, max_force=0.0,
                 write_allowed=True), session=session)
        assert result[:2] == (True, gripper_core.EC_SUCCESS)
        assert math.isnan(result[4])
        assert events and events[-1][0] == gripper_core.S_SDK_CALL_FAILED


def test_busy_is_rejected_without_queueing():
    class _BlockingSession(FakeSession):
        def __init__(self, *args, started, release, **kwargs):
            super().__init__(*args, **kwargs)
            self._started = started
            self._release = release

        def open_effector(self, names=None, duration=1.0):
            self._started.set()
            self._release.wait(timeout=2.0)

    for use_native in (False, True):
        started = threading.Event()
        release = threading.Event()
        session = _BlockingSession(
            current={'left_gripper': [0.0], 'right_gripper': [0.0]},
            follow_ratio=1.0, in_simulation=True,
            started=started, release=release)
        controller, _, _, old = _controller(use_native, session=session)
        try:
            request = dict(name='left_gripper', opening_fraction=1.0,
                           duration=0.0, use_raw_cmd=False, raw_cmd=0.0,
                           max_force=0.0, write_allowed=True)
            result_holder = []
            worker = threading.Thread(
                target=lambda: result_holder.append(controller.execute(**request)))
            worker.start()
            assert started.wait(timeout=1.0)
            second = controller.execute(**request)
            assert second.error_code == gripper_core.EC_BUSY
            release.set()
            worker.join(timeout=2.0)
            assert result_holder and result_holder[0].ok
        finally:
            release.set()
            _finish(old)
