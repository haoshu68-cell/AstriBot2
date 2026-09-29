"""Chassis replay against the Python reference through a standalone C++ process."""
import os
import subprocess
import pytest
from astribot_trajectory_bridge import chassis_bridge_core as bridge
from astribot_trajectory_bridge.ports import FakeClock, FakePose, FakeSession

@pytest.mark.parametrize('scenario',['replay','timeout','leash','scan','pose','read_failure','write_failure'])
def test_chassis_standalone_trace(scenario, monkeypatch):
    from astribot_trajectory_bridge import chassis_integrator, chassis_feedback
    monkeypatch.setattr(chassis_integrator, "_native", None)
    monkeypatch.setattr(chassis_feedback, "_native", None)
    clock=FakeClock();pose=FakePose(clock,[0.,0.,0.]);trace=[]
    class Session(FakeSession):
        def __init__(self):super().__init__(current={'astribot_chassis':[0.,0.,0.]},follow_ratio=.87);self.reads=0
        def get_current_joints_position(self,n):
            self.reads+=1
            if scenario=='read_failure' and self.reads>1:raise RuntimeError('position read injected')
            return super().get_current_joints_position(n)
        def set_joints_position(self,n,q,control_way='filter',use_wbc=False,add_default_torso=True):
            if scenario=='write_failure':raise RuntimeError('position write injected')
            trace.append(['WRITE',clock.now(),n[0],control_way,int(use_wbc),int(add_default_torso),*q[0]])
            return super().set_joints_position(n,q,control_way,use_wbc,add_default_torso)
    session=Session()
    cfg=bridge.ChassisBridgeConfig(require_fresh_scan=scenario=='scan',
        require_slam_to_enable=scenario=='pose',scan_max_age_sec=.02,
        scan_loss_grace_sec=.05,slam_loss_grace_sec=.05,
        cmd_vel_timeout_sec=.01 if scenario=='timeout' else .3,
        leash_xy_m=.005 if scenario=='leash' else .25)
    if scenario=='leash':pose.set_pose(None)
    core=bridge._PythonChassisBridgeCore(cfg,session,pose,clock)
    trace.append(['ENABLE',int(core.enable()[0])]);core.submit_scan_seen()
    if scenario=='pose':pose.set_pose(None)
    for tick in range(700 if scenario=='replay' else 30):
        clock.advance((.008 if tick%113==0 else .004) if scenario=='replay' else .01)
        if scenario=='replay' and tick in (100,101):pose.set_pose([.002*tick,.0005*tick,.0002*tick])
        if scenario!='timeout' or tick==0:
            core.submit_twist(*( (0,0,0) if tick==300 else (-.08,.02,.03) if tick==301 else (.12,-.03,.04)))
        if scenario=='leash':session.set_current('astribot_chassis',[.02,0,0])
        ok=core.inner_tick()
        if scenario=='pose':core.outer_tick()
        trace.append(['TICK',clock.now(),int(ok),core.state,*core.pos_cmd])
        for e in core.drain_events():trace.append(['EVENT',e.code,e.metric_1,e.metric_2])
    actual=subprocess.check_output([os.environ.get('ASTRIBOT_CHASSIS_PROBE','/tmp/astribot_chassis_core_probe'),scenario],text=True).splitlines()
    assert len(actual)==len(trace)
    for actual_line,expected_line in zip(actual,trace):
        fields=actual_line.split();assert len(fields)==len(expected_line)
        for value,expected in zip(fields,expected_line):
            if isinstance(expected,str):assert value==expected
            else:assert float(value)==pytest.approx(expected,abs=3e-12)
