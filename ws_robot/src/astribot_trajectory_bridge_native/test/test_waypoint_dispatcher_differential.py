"""Differential tests for the optional native WaypointDispatcher."""

from astribot_trajectory_bridge import arm_bridge_core
from astribot_trajectory_bridge_native import _chassis_math_native as native


class Session:
    def __init__(self, fail=False):
        self.fail = fail
        self.calls = []

    def move_joints_waypoints(self, names, waypoints, time_list, **kwargs):
        if self.fail:
            raise RuntimeError('waypoint write injected')
        self.calls.append((list(names), waypoints, list(time_list), kwargs))


def _run(enabled, request, *, sdk_fail=False, dispatcher_enabled=True):
    old = arm_bridge_core._native
    arm_bridge_core._native = native if enabled else None
    session = Session(fail=sdk_fail)
    cfg = arm_bridge_core.ArmBridgeConfig(
        part_name='arm', joint_names=['j0', 'j1'], limit_margin_rad=0.1)
    dispatcher = arm_bridge_core.WaypointDispatcher(
        cfg, session, enabled=dispatcher_enabled)
    try:
        result = dispatcher.dispatch(**request)
        events = [(event.code, event.detail, event.metric_1, event.metric_2)
                  for event in dispatcher.events]
        return result, events, session.calls
    finally:
        arm_bridge_core._native = old


def test_waypoint_success_and_drop_are_equivalent():
    request = dict(
        waypoints=[[0.0, 0.0], [0.1, 0.2], [0.3, 0.4]],
        time_list=[0.0, 0.2, 0.5],
        lower=[-1.0, -1.0], upper=[1.0, 1.0])
    before = _run(False, request)
    after = _run(True, request)
    assert before == after
    assert before[0] == (True, 'SUCCESS', '', 2, 1)
    assert before[2][0][2] == [0.2, 0.5]


def test_waypoint_rejection_matrix_preserves_code_and_event():
    requests = [
        (dict(waypoints=[[0.0, 0.0]], time_list=[0.1], lower=None, upper=None),
         dict(dispatcher_enabled=False)),
        (dict(waypoints=[[0.0, 0.0]], time_list=[0.1, 0.2], lower=None, upper=None),
         {}),
        (dict(waypoints=[[0.0, 0.0]], time_list=[0.0], lower=None, upper=None),
         {}),
        (dict(waypoints=[[0.0, 0.0], [0.1, 0.1]], time_list=[0.2, 0.1],
              lower=None, upper=None), {}),
        (dict(waypoints=[[2.0, 0.0]], time_list=[0.1], lower=[-1.0, -1.0],
              upper=[1.0, 1.0]), {}),
    ]
    for request, options in requests:
        before = _run(False, request, **options)
        after = _run(True, request, **options)
        assert before == after
        assert before[0][0] is False
        assert before[2] == []


def test_waypoint_sdk_failure_is_reported_without_success():
    request = dict(waypoints=[[0.0, 0.0]], time_list=[0.1],
                   lower=None, upper=None)
    before = _run(False, request, sdk_fail=True)
    after = _run(True, request, sdk_fail=True)
    assert before == after
    assert before[0][:2] == (False, 'SDK_CALL_FAILED')
    assert before[1][0][0] == arm_bridge_core.S_SDK_CALL_FAILED
