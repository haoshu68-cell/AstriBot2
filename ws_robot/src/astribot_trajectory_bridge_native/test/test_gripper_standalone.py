"""Compare the Python reference to a separate C++ process, without a binding or ROS."""
import math
import os
import subprocess
import pytest
from astribot_trajectory_bridge import gripper_core
from astribot_trajectory_bridge.ports import FakeClock, FakeSession

PROBE = os.environ.get('ASTRIBOT_GRIPPER_PROBE', '/tmp/astribot_gripper_core_probe')
SCENARIOS = ('mid', 'open', 'close', 'stalled', 'read_failure', 'write_failure',
             'open_failure', 'force_failure', 'force_real', 'force_sim', 'denied',
             'disabled', 'unknown', 'invalid')

@pytest.mark.parametrize('scenario', SCENARIOS)
def test_standalone_replay_matches_python(scenario, monkeypatch):
    from astribot_trajectory_bridge import gripper_math
    monkeypatch.setattr(gripper_math, "_native", None)
    # Wrong gate, polarity, deadline, force result, error branch, or SDK metadata
    # changes the result or the per-tick device trace captured here.
    clock = FakeClock()
    trace = []
    class Session(FakeSession):
        def get_current_joints_position(self, names):
            if scenario == 'read_failure':
                raise RuntimeError('read failure')
            return super().get_current_joints_position(names)
        def set_joints_position(self, names, position, control_way='filter',
                                use_wbc=False, add_default_torso=True):
            if scenario == 'write_failure':
                raise RuntimeError('write failure')
            for name, value in zip(names, position):
                trace.append(('WRITE', clock.now(), name, value[0], control_way,
                              int(use_wbc), int(add_default_torso)))
            if scenario != 'stalled':
                return super().set_joints_position(names, position, control_way,
                                                  use_wbc, add_default_torso)
        def open_effector(self, names, duration=1.):
            if scenario == 'open_failure':
                raise RuntimeError('open failure')
            for name in names:
                trace.append(('OPEN', clock.now(), name, duration))
            return super().open_effector(names, duration)
        def close_effector(self, names, duration=1.):
            for name in names:
                trace.append(('CLOSE', clock.now(), name, duration))
            return super().close_effector(names, duration)
        def set_effector_max_force(self, names, force):
            if scenario == 'force_failure':
                raise RuntimeError('force failure')
            for name, value in zip(names, force):
                trace.append(('FORCE', clock.now(), name, value))
            return super().set_effector_max_force(names, force)
    session = Session(current={'left_gripper':[0.], 'right_gripper':[0.]})
    config = gripper_core.GripperConfig(
        gripper_names=['left_gripper','right_gripper'], enable_service=scenario!='disabled',
        default_duration_sec=.2, settle_extra_sec=0, stream_freq=20,
        mid_stream_tolerance=.5, mid_stream_timeout_sec=.25)
    reference = gripper_core._PythonGripperController(config, session, sleep_fn=clock.advance,
        clock_fn=clock.now, in_simulation=scenario!='force_real')
    fraction = {'open':1, 'open_failure':1, 'close':0, 'invalid':1.2}.get(scenario,.5)
    result = reference.execute(name='unknown' if scenario=='unknown' else 'left_gripper',
        opening_fraction=fraction, duration=0, use_raw_cmd=False, raw_cmd=0,
        max_force=30 if scenario.startswith('force_') else 0, write_allowed=scenario!='denied')
    actual_trace, events = [], []
    for line in subprocess.check_output([PROBE, scenario], text=True).splitlines():
        fields = line.split()
        if fields[0] == 'RESULT':
            actual_result = fields[1:]
        elif fields[0] == 'EVENT':
            events.append(fields[1])
        else:
            row = (fields[0], float(fields[1]), fields[2], float(fields[3]))
            if fields[0] == 'WRITE':
                row += (fields[4], int(fields[5]), int(fields[6]))
            actual_trace.append(row)
    assert actual_trace == trace
    assert events == [e.code for e in reference.drain_events()]
    assert (bool(int(actual_result[0])), actual_result[1]) == (result.ok,result.error_code)
    expected = (result.dispatched_cmd,result.dispatched_rad,result.actual_cmd,
                int(result.force_applied),clock.now())
    for actual, wanted in zip(map(float,actual_result[2:]),expected):
        assert math.isnan(actual) if math.isnan(wanted) else actual == pytest.approx(wanted,abs=1e-14)
