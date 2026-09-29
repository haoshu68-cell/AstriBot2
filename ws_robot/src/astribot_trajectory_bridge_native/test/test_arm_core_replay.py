import math

from astribot_trajectory_bridge import arm_bridge_core
from astribot_trajectory_bridge import arm_traj_math
from astribot_trajectory_bridge_native import _chassis_math_native as native


JOINTS = ["j0", "j1", "j2"]
LOWER = [-2.0] * len(JOINTS)
UPPER = [2.0] * len(JOINTS)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class FakeSession:
    """Small deterministic SDK double used for before/after replay."""

    def __init__(self, mode="lag"):
        self.mode = mode
        self.actual = [0.0] * len(JOINTS)
        self.calls = []

    def get_joints_position_limit(self, _parts):
        return [list(LOWER)], [list(UPPER)]

    def get_current_joints_position(self, _parts):
        return [list(self.actual)]

    def set_joints_position(self, _parts, positions, **kwargs):
        target = list(positions[0])
        self.calls.append((target, tuple(sorted(kwargs.items()))))
        if self.mode == "lag":
            self.actual = [a + 0.65 * (q - a) for a, q in zip(self.actual, target)]
        elif self.mode == "fixed":
            pass
        elif self.mode == "hold":
            self.actual = [a + 0.75 * (q - a) for a, q in zip(self.actual, target)]
        else:
            raise AssertionError(self.mode)


def _cfg(**kwargs):
    values = dict(
        joint_names=JOINTS,
        stream_freq=50.0,
        settle_tolerance_rad=0.02,
        settle_timeout_sec=0.8,
        hold_still_epsilon_rad=0.001,
        hold_still_ticks_required=4,
        hold_timeout_sec=0.8,
        max_tracking_error_rad=0.10,
    )
    values.update(kwargs)
    return arm_bridge_core.ArmBridgeConfig(**values)


def _run(mode, *, cancel_at=None, **cfg_kwargs):
    clock = FakeClock()
    session = FakeSession(mode)
    ex = arm_bridge_core.ArmTrajExecutor(_cfg(**cfg_kwargs), session, clock)
    assert ex.load_limits()[0]
    ok, code, detail = ex.start(
        JOINTS,
        [0.0, 0.20, 0.50],
        [[0.0, 0.0, 0.0], [0.35, -0.20, 0.15], [0.55, -0.30, 0.25]],
        [[0.0, 0.0, 0.0], [0.2, -0.1, 0.1], [0.0, 0.0, 0.0]],
    )
    assert ok, (code, detail)
    for tick in range(300):
        clock.advance(0.02)
        if cancel_at is not None and clock.now() >= cancel_at:
            ex.request_cancel()
        ex.step()
        if ex.phase in (arm_bridge_core.PH_DONE,
                        arm_bridge_core.PH_CANCELED,
                        arm_bridge_core.PH_ABORTED):
            break
    else:
        raise AssertionError("executor did not reach a terminal phase")
    return ex, session, clock, tick + 1


def _snapshot(ex, session, clock, ticks):
    return {
        "phase": ex.phase,
        "error_code": ex.error_code,
        "detail": ex.detail,
        "events": [(e.code, e.detail, e.metric_1, e.metric_2) for e in ex.events],
        "feedbacks": [
            (f.t, tuple(f.desired), tuple(f.actual), f.error)
            for f in ex.feedbacks
        ],
        "calls": session.calls,
        "actual": tuple(session.actual),
        "clock": clock.now(),
        "ticks": ticks,
    }


def _assert_replay_equal(before, after, tol=3e-12):
    assert before["phase"] == after["phase"]
    assert before["error_code"] == after["error_code"]
    assert before["detail"] == after["detail"]
    assert [(a[0], a[1]) for a in before["events"]] == [
        (a[0], a[1]) for a in after["events"]
    ]
    assert len(before["events"]) == len(after["events"])
    for lhs, rhs in zip(before["events"], after["events"]):
        assert math.isclose(lhs[2], rhs[2], rel_tol=tol, abs_tol=tol)
        assert math.isclose(lhs[3], rhs[3], rel_tol=tol, abs_tol=tol)
    assert len(before["feedbacks"]) == len(after["feedbacks"])
    for lhs, rhs in zip(before["feedbacks"], after["feedbacks"]):
        assert math.isclose(lhs[0], rhs[0], rel_tol=tol, abs_tol=tol)
        for lv, rv in zip(lhs[1], rhs[1]):
            assert math.isclose(lv, rv, rel_tol=tol, abs_tol=tol)
        for lv, rv in zip(lhs[2], rhs[2]):
            assert math.isclose(lv, rv, rel_tol=tol, abs_tol=tol)
        assert math.isclose(lhs[3], rhs[3], rel_tol=tol, abs_tol=tol)
    assert len(before["calls"]) == len(after["calls"])
    for (lq, lmeta), (rq, rmeta) in zip(before["calls"], after["calls"]):
        for lv, rv in zip(lq, rq):
            assert math.isclose(lv, rv, rel_tol=tol, abs_tol=tol)
        assert lmeta == rmeta
    for lv, rv in zip(before["actual"], after["actual"]):
        assert math.isclose(lv, rv, rel_tol=tol, abs_tol=tol)
    assert before["clock"] == after["clock"]
    assert before["ticks"] == after["ticks"]


def _run_pair(**kwargs):
    old = arm_traj_math._native
    old_core = arm_bridge_core._native
    try:
        arm_traj_math._native = None
        arm_bridge_core._native = None
        before = _snapshot(*_run("lag", **kwargs))
        arm_traj_math._native = native
        arm_bridge_core._native = native
        after = _snapshot(*_run("lag", **kwargs))
    finally:
        arm_traj_math._native = old
        arm_bridge_core._native = old_core
    _assert_replay_equal(before, after)
    return before


def test_executor_success_replay():
    result = _run_pair()
    assert result["phase"] == arm_bridge_core.PH_DONE
    assert result["error_code"] == arm_bridge_core.EC_SUCCESSFUL


def test_executor_cancel_replay():
    # The hold path is deliberately exercised while the fake plant is moving.
    old = arm_traj_math._native
    old_core = arm_bridge_core._native
    try:
        arm_traj_math._native = None
        arm_bridge_core._native = None
        before = _snapshot(*_run("hold", cancel_at=0.22))
        arm_traj_math._native = native
        arm_bridge_core._native = native
        after = _snapshot(*_run("hold", cancel_at=0.22))
    finally:
        arm_traj_math._native = old
        arm_bridge_core._native = old_core
    _assert_replay_equal(before, after)
    assert before["phase"] == arm_bridge_core.PH_CANCELED
    assert "保持并停稳" in before["detail"]


def test_executor_tracking_abort_and_settle_timeout_replay():
    old = arm_traj_math._native
    old_core = arm_bridge_core._native
    try:
        for kwargs in (
            dict(mode="fixed", max_tracking_error_rad=0.05),
            dict(mode="fixed", max_tracking_error_rad=10.0,
                 abort_on_tracking_error=False, settle_timeout_sec=0.05),
        ):
            mode = kwargs.pop("mode")
            arm_traj_math._native = None
            arm_bridge_core._native = None
            before = _snapshot(*_run(mode, **kwargs))
            arm_traj_math._native = native
            arm_bridge_core._native = native
            after = _snapshot(*_run(mode, **kwargs))
            _assert_replay_equal(before, after)
            assert before["phase"] == arm_bridge_core.PH_ABORTED
    finally:
        arm_traj_math._native = old
        arm_bridge_core._native = old_core
