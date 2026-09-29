"""Per-tick arm replay through an independent C++ process, no Python binding."""
import os
import subprocess
import pytest
from astribot_trajectory_bridge import arm_bridge_core as arm
from astribot_trajectory_bridge.ports import FakeClock

@pytest.mark.parametrize('scenario', ['success','cancel','tracking','settle_timeout',
    'limit_failure','read_failure','write_failure','hold_read_failure'])
def test_arm_standalone_trace(scenario, monkeypatch):
    from astribot_trajectory_bridge import arm_traj_math
    monkeypatch.setattr(arm_traj_math, "_native", None)
    clock=FakeClock()
    trace=[]
    class Session:
        def __init__(self): self.actual=[0.,0.,0.]; self.reads=0
        def get_joints_position_limit(self,names):
            if scenario=='limit_failure': raise RuntimeError('limit read injected')
            return [[-2.]*3],[[2.]*3]
        def get_current_joints_position(self,names):
            self.reads+=1
            if ((scenario=='read_failure' and self.reads>=2) or
                    (scenario=='hold_read_failure' and self.reads>=3)):
                raise RuntimeError('position read injected')
            return [list(self.actual)]
        def set_joints_position(self,names,p,control_way='filter',use_wbc=False,add_default_torso=True):
            if scenario=='write_failure': raise RuntimeError('position write injected')
            trace.append(['WRITE',clock.now(),names[0],control_way,int(use_wbc),int(add_default_torso),*p[0]])
            if scenario not in ('tracking','settle_timeout'):
                self.actual=[a+.65*(q-a) for a,q in zip(self.actual,p[0])]
    c=arm.ArmBridgeConfig(joint_names=['j0','j1','j2'],stream_freq=50,
        settle_timeout_sec=.05 if scenario=='settle_timeout' else .8,
        hold_still_ticks_required=4,hold_timeout_sec=.8,
        max_tracking_error_rad=.05 if scenario=='tracking' else (10 if scenario=='settle_timeout' else .10),
        abort_on_tracking_error=scenario!='settle_timeout')
    ex=arm._PythonArmTrajExecutor(c,Session(),clock)
    trace.append(['LOAD',int(ex.load_limits()[0])])
    started=ex.start(c.joint_names,[0,.2,.5],[[0,0,0],[.35,-.20,.15],[.55,-.30,.25]],[[0,0,0],[.2,-.1,.1],[0,0,0]])
    trace.append(['START',int(started[0]),started[1]])
    if started[0]:
        for _ in range(300):
            clock.advance(.02)
            if (scenario=='cancel' and clock.now()>=.22) or scenario=='hold_read_failure':ex.request_cancel()
            ex.step()
            trace.append(['TICK',clock.now(),ex.phase,ex.error_code])
            if ex.phase in ('DONE','CANCELED','ABORTED'):break
    for e in ex.events:trace.append(['EVENT',e.code,e.metric_1,e.metric_2])
    for f in ex.feedbacks:trace.append(['FB',f.t,f.error,*f.desired,*f.actual])
    actual=subprocess.check_output([os.environ.get('ASTRIBOT_ARM_PROBE','/tmp/astribot_arm_core_probe'),scenario],text=True).splitlines()
    assert len(actual)==len(trace)
    for actual_line,expected_line in zip(actual,trace):
        fields=actual_line.split()
        assert len(fields)==len(expected_line)
        for value,expected in zip(fields,expected_line):
            if isinstance(expected,str): assert value==expected
            else: assert float(value)==pytest.approx(expected,abs=3e-12)
