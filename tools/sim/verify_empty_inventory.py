#!/usr/bin/env python3
"""Owned simulation EMPTY-inventory fault checks. Never sends motion goals.

Mutations: one named temporary Gazebo model, one named temporary PlanningScene
attachment, and a bounded world pause. Each is restored in finally. Hardware and
other sessions are rejected by exact supervisor identity + environment checks.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import subprocess
import time
import uuid


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--owner',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--session',required=True)
    p.add_argument('--source',default='gazebo_empty_v1')
    a=p.parse_args()
    identity=json.loads(a.owner.read_text());process=Path('/proc')/str(identity['pid'])
    assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()==identity['boot_id']
    assert process.joinpath('stat').read_text().rsplit(') ',1)[1].split()[19]==identity['start_ticks']
    assert str(process.joinpath('exe').resolve())==identity['exe']
    argv=[v.decode() for v in process.joinpath('cmdline').read_bytes().split(b'\0')[:-1]]
    assert argv==identity['cmdline']
    assert argv[argv.index('--instance')+1]==a.session
    assert argv[argv.index('--ros-domain-id')+1]==os.environ['ROS_DOMAIN_ID']!='25'
    assert os.environ['IGN_PARTITION']=='astribot_'+a.session
    a.output.mkdir(parents=True,exist_ok=False)

    import rclpy
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Twist,Pose
    from shape_msgs.msg import SolidPrimitive
    from moveit_msgs.msg import AttachedCollisionObject
    from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene
    from std_msgs.msg import String
    from astribot_payload_msgs.msg import AttachmentState,AttachmentObservation
    from astribot_navigation_msgs.msg import RobotGeometryState
    from rosidl_runtime_py.convert import message_to_ordereddict
    rclpy.init();node=rclpy.create_node('verify_empty_inventory',parameter_overrides=[Parameter('use_sim_time',value=True)])
    latest={};received={};trace=[];phases=[];paused=False;created=False;phantom=False
    probe='inventory_probe_'+uuid.uuid4().hex[:12]
    def record(key,value):
        data=message_to_ordereddict(value)
        if key=='diagnostic':data=json.loads(value.data)
        latest[key]=value if key!='diagnostic' else data
        received[key]=time.monotonic()
        trace.append(dict(kind=key,wall=time.monotonic(),ros_ns=node.get_clock().now().nanoseconds,message=data))
    subs=[node.create_subscription(AttachmentState,'/payload/attachment_state',lambda v:record('ledger',v),10),
          node.create_subscription(AttachmentObservation,'/payload/attachment_observation',lambda v:record('source',v),10),
          node.create_subscription(RobotGeometryState,'/navigation/geometry_state',lambda v:record('geometry',v),10),
          node.create_subscription(String,'/payload/simulation_inventory_diagnostics',lambda v:record('diagnostic',v),10),
          node.create_subscription(Twist,'/cmd_vel',lambda v:record('command',v),10)]
    apply=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
    scene=node.create_client(GetPlanningScene,'/get_planning_scene')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    def wait(predicate,timeout=8.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return
        raise AssertionError({k:(v.reason if hasattr(v,'reason') else v.get('reason') if isinstance(v,dict) else 'received') for k,v in latest.items()})
    def ready():
        s,g=latest.get('ledger'),latest.get('geometry')
        fresh=all(time.monotonic()-received.get(k,0)<.3 for k in ('ledger','geometry','diagnostic'))
        return fresh and s and g and s.confirmed and g.complete and g.attachment_revision==s.attachment_revision and s.observation.status==AttachmentObservation.EMPTY
    def stable_ready(seconds=2.,timeout=12.):
        # Receiving one healthy sample does not prove the subsequent interval.
        # Reset on every observed invalid state, including startup reconciliation.
        start=None;end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            now=time.monotonic()
            if not ready():start=None
            elif start is None:start=now
            elif now-start>=seconds:return
        raise AssertionError('EMPTY readiness was not continuously observed for the required interval')
    def rejected():
        s,g=latest.get('ledger'),latest.get('geometry')
        return s and g and not s.confirmed and not g.complete
    def stationary():
        command=latest.get('command')
        assert command is not None and time.monotonic()-received['command']<.3,'missing fresh stationary command'
        assert max(abs(getattr(getattr(command,field),axis)) for field in ('linear','angular') for axis in ('x','y','z'))<1e-6
    def mark(name,**extra):
        s=latest.get('ledger');g=latest.get('geometry');d=latest.get('diagnostic',{})
        phases.append(dict(name=name,wall=time.monotonic(),ros_ns=node.get_clock().now().nanoseconds,
            ledger_reason=s.reason if s else None,geometry_reason=g.reason if g else None,
            revision=s.attachment_revision if s else None,source_revision=d.get('revision'),**extra))
    def ign(service,kind,request):
        result=subprocess.run(['ign','service','-s','/world/default/'+service,'--reqtype','ignition.msgs.'+kind,
            '--reptype','ignition.msgs.Boolean','--timeout','5000','--req',request],capture_output=True,text=True,timeout=8)
        assert result.returncode==0 and 'data: true' in result.stdout,(service,result.stdout,result.stderr)
    def call(client,request):
        assert client.wait_for_service(timeout_sec=4),client.srv_name
        future=client.call_async(request);wait(future.done,4)
        return future.result()
    def scene_attachment(remove=False):
        obj=AttachedCollisionObject(link_name='astribot_arm_left_tcp_link');obj.object.id=probe+'_phantom'
        obj.object.header.frame_id=obj.link_name;obj.object.pose.orientation.w=1.
        obj.object.operation=obj.object.REMOVE if remove else obj.object.ADD
        if not remove:
            obj.object.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.02,.02,.02])]
            pose=Pose();pose.orientation.w=1.;obj.object.primitive_poses=[pose]
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True
        req.scene.robot_state.attached_collision_objects=[obj]
        assert call(apply,req).success
    success=False
    try:
        stable_ready()
        assert latest['ledger'].observation.session_id==a.session
        assert latest['ledger'].observation.source_id==a.source
        assert latest['ledger'].observation.environment=='simulation'
        assert latest['diagnostic']['policy']=='static_world_empty_only_v1'
        mark('real_empty_confirmed');original=latest['ledger'].attachment_revision
        request=GetPlanningScene.Request();request.components.components=4
        actual=call(scene,request).scene
        assert not actual.is_diff and not actual.robot_state.is_diff and not actual.robot_state.attached_collision_objects
        assert ready(),'readiness lost during independent readback'
        mark('independent_full_moveit_readback')

        stationary()
        sdf='<sdf version="1.7"><model name="'+probe+'"><static>true</static><pose>-18 -18 1 0 0 0</pose><link name="body"><visual name="v"><geometry><box><size>0.05 0.05 0.05</size></box></geometry></visual></link></model></sdf>'
        ign('create','EntityFactory','sdf: '+json.dumps(sdf)+' allow_renaming: false');created=True
        wait(lambda:rejected() and latest.get('diagnostic',{}).get('reason')=='BACKGROUND_MANIFEST_MISMATCH')
        mark('unregistered_static_payload_revoked')
        ign('remove','Entity','name: '+json.dumps(probe)+' type: MODEL');created=False
        wait(ready);assert latest['ledger'].attachment_revision!=original
        mark('removal_restored_new_empty_version')

        stationary()
        scene_attachment();phantom=True
        wait(rejected);spin(.5)
        actual=call(scene,request).scene
        assert any(v.object.id==probe+'_phantom' for v in actual.robot_state.attached_collision_objects)
        assert latest['diagnostic']['reason']=='EMPTY_INVENTORY_OBSERVED'
        mark('physical_scene_mismatch_revoked')
        scene_attachment(True);phantom=False;wait(ready)
        mark('scene_restored_reconciled')

        stationary()
        ign('control','WorldControl','pause: true');paused=True
        spin(.9);assert rejected();frozen=node.get_clock().now().nanoseconds
        spin(.3);assert node.get_clock().now().nanoseconds==frozen
        mark('paused_physics_wall_expiry')
        ign('control','WorldControl','pause: false');paused=False;wait(ready)
        mark('resumed_fresh_capture_reconciled')
        stable_ready(seconds=3.)
        commands=[v['message'] for v in trace if v['kind']=='command']
        assert commands,'missing final command evidence'
        maximum=max(abs(axis) for cmd in commands for field in ('linear','angular') for axis in cmd[field].values())
        assert maximum<1e-6,maximum
        mark('final_stationary_ready',max_command=maximum)
        success=True
    finally:
        cleanup=[]
        for needed,fn,label in [(paused,lambda:ign('control','WorldControl','pause: false'),'resume_world'),
                                 (phantom,lambda:scene_attachment(True),'remove_phantom'),
                                 (created,lambda:ign('remove','Entity','name: '+json.dumps(probe)+' type: MODEL'),'remove_probe')]:
            if needed:
                try:fn();cleanup.append(dict(action=label,ok=True))
                except Exception as error:cleanup.append(dict(action=label,ok=False,error=str(error)))
        (a.output/'trace.json').write_text(json.dumps(trace,indent=2))
        report=dict(passed=success,phases=phases,cleanup=cleanup,owner=str(a.owner.resolve()),
                    evidence='actual canonical Gazebo + C++ inventory/ledger/geometry + independent MoveIt; EMPTY only',
                    reasons=dict(Counter(v['message'].get('reason') for v in trace if v['kind']=='ledger')))
        (a.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
        node.destroy_node();rclpy.shutdown()
        assert all(v['ok'] for v in cleanup),cleanup

if __name__=='__main__':main()
