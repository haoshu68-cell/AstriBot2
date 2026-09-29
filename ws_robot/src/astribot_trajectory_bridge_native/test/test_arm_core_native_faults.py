"""Boundary and fault-injection replay for the native ArmTrajExecutor."""

from astribot_trajectory_bridge import arm_bridge_core, arm_traj_math
from astribot_trajectory_bridge_native import _chassis_math_native as native


JOINTS = ['j0', 'j1', 'j2']


class Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def advance(self, seconds):
        self.t += seconds


class FaultSession:
    def __init__(self, *, limits=None, mode='follow', fail_limits=False,
                 fail_read_at=None, fail_write_at=None):
        self.limits = limits or ([[-2.0] * 3], [[2.0] * 3])
        self.mode = mode
        self.fail_limits = fail_limits
        self.fail_read_at = fail_read_at
        self.fail_write_at = fail_write_at
        self.reads = 0
        self.writes = 0
        self.actual = [0.0, 0.0, 0.0]

    def get_joints_position_limit(self, _parts):
        if self.fail_limits:
            raise RuntimeError('limit read injected')
        return self.limits

    def get_current_joints_position(self, _parts):
        self.reads += 1
        if self.fail_read_at is not None and self.reads >= self.fail_read_at:
            raise RuntimeError('position read injected')
        return [list(self.actual)]

    def set_joints_position(self, _parts, positions, **_kwargs):
        self.writes += 1
        if self.fail_write_at is not None and self.writes >= self.fail_write_at:
            raise RuntimeError('position write injected')
        target = list(positions[0])
        if self.mode == 'follow':
            self.actual = [a + 0.7 * (q - a)
                           for a, q in zip(self.actual, target)]
        elif self.mode == 'fixed':
            pass


def _cfg(**kwargs):
    values = dict(
        joint_names=JOINTS,
        stream_freq=50.0,
        settle_tolerance_rad=0.02,
        settle_timeout_sec=0.2,
        max_tracking_error_rad=10.0,
        hold_still_epsilon_rad=0.001,
        hold_still_ticks_required=3,
        hold_timeout_sec=0.2,
    )
    values.update(kwargs)
    return arm_bridge_core.ArmBridgeConfig(**values)


def _make(enabled, session, clock, **cfg_kwargs):
    old_core = arm_bridge_core._native
    old_math = arm_traj_math._native
    arm_bridge_core._native = native if enabled else None
    arm_traj_math._native = native if enabled else None
    executor = arm_bridge_core.ArmTrajExecutor(
        _cfg(**cfg_kwargs), session, clock)
    return executor, old_core, old_math


def _restore(old_core, old_math):
    arm_bridge_core._native = old_core
    arm_traj_math._native = old_math


def _event_codes(executor):
    return [event.code for event in executor.events]


def test_limit_load_failures_and_reversed_limits_match():
    for limits, fail in (
        (([[-2.0, -2.0, -2.0]], [[2.0, 2.0, 2.0]]), True),
        (([[2.0, -2.0, -2.0]], [[-2.0, 2.0, 2.0]]), False),
    ):
        outputs = []
        for enabled in (False, True):
            session = FaultSession(limits=limits, fail_limits=fail)
            clock = Clock()
            executor, old_core, old_math = _make(enabled, session, clock)
            try:
                result = executor.load_limits()
                outputs.append((result, executor.phase, executor.error_code,
                                _event_codes(executor)))
            finally:
                _restore(old_core, old_math)
        assert outputs[0] == outputs[1]
        assert outputs[0][0][0] is False


def test_start_rejects_missing_limits_shape_joint_time_and_duration():
    cases = [
        lambda e, c: e.start(JOINTS, [0.0, 0.1], [[0.0] * 3]),
        lambda e, c: e.start(JOINTS, [0.0, 0.1], [[0.0] * 3, [0.1] * 3], None),
        lambda e, c: e.start(['wrong'] * 3, [0.0, 0.1],
                              [[0.0] * 3, [0.1] * 3]),
        lambda e, c: e.start(JOINTS, [0.1, 0.1],
                              [[0.0] * 3, [0.1] * 3]),
    ]
    for make_case in cases:
        outputs = []
        for enabled in (False, True):
            session = FaultSession()
            clock = Clock()
            executor, old_core, old_math = _make(
                enabled, session, clock, max_traj_duration_sec=0.05)
            try:
                if make_case.__code__.co_firstlineno == cases[0].__code__.co_firstlineno:
                    # First case deliberately omits a second position row.
                    pass
                output = make_case(executor, clock)
                outputs.append((output, executor.phase, executor.error_code,
                                executor.detail, _event_codes(executor)))
            finally:
                _restore(old_core, old_math)
        assert outputs[0] == outputs[1]
        assert outputs[0][0][0] is False
        assert outputs[0][2] in (arm_bridge_core.EC_INVALID_GOAL,
                                 arm_bridge_core.EC_INVALID_JOINTS)


def test_stream_write_and_read_faults_preserve_abort_semantics():
    for fault in ('write', 'read'):
        outputs = []
        for enabled in (False, True):
            session = FaultSession(
                fail_write_at=1 if fault == 'write' else None,
                fail_read_at=2 if fault == 'read' else None)
            clock = Clock()
            executor, old_core, old_math = _make(enabled, session, clock)
            try:
                assert executor.load_limits()[0]
                assert executor.start(
                    JOINTS, [0.0, 0.2],
                    [[0.0] * 3, [0.4, -0.2, 0.1]],
                    [[0.0] * 3, [0.0] * 3])[0]
                clock.advance(0.02)
                executor.step()
                outputs.append((executor.phase, executor.error_code,
                                executor.detail, _event_codes(executor)))
            finally:
                _restore(old_core, old_math)
        assert outputs[0] == outputs[1]
        assert outputs[0][0] == arm_bridge_core.PH_ABORTED
        assert outputs[0][1] == arm_bridge_core.EC_PATH_TOLERANCE_VIOLATED
        assert outputs[0][3] == [arm_bridge_core.S_SDK_CALL_FAILED]


def test_cancel_hold_read_failure_preserves_cancel_terminal_state():
    outputs = []
    for enabled in (False, True):
        session = FaultSession(fail_read_at=3)
        clock = Clock()
        executor, old_core, old_math = _make(enabled, session, clock)
        try:
            assert executor.load_limits()[0]
            assert executor.start(
                JOINTS, [0.0, 0.5],
                [[0.0] * 3, [0.5, -0.2, 0.1]],
                [[0.0] * 3, [0.0] * 3])[0]
            clock.advance(0.02)
            executor.request_cancel()
            executor.step()
            outputs.append((executor.phase, executor.error_code,
                            executor.detail, _event_codes(executor)))
        finally:
            _restore(old_core, old_math)
    assert outputs[0] == outputs[1]
    assert outputs[0][0] == arm_bridge_core.PH_CANCELED
