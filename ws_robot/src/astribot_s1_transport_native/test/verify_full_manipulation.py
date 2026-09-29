#!/usr/bin/env python3
"""Owned ROS protocol fixture. Synthetic controllers; no Gazebo/robot motion."""
import argparse
import copy
import hashlib
from collections import deque
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from action_msgs.msg import GoalStatusArray
from builtin_interfaces.msg import Time, Duration
from control_msgs.action import FollowJointTrajectory as FJT
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, TransformStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from astribot_payload_msgs.msg import AttachmentObservation, AttachmentState
from trajectory_msgs.msg import JointTrajectoryPoint
from astribot_navigation_msgs.msg import RobotGeometryState, ArmHoldStatus, NavigationEnvelopeV2
from astribot_navigation_msgs.srv import SetRobotEnvelope
from astribot_transport_msgs.action import PlanManipulation
from astribot_transport_msgs.msg import ManipulationStage, ExecutionGuardStatus
from astribot_transport_msgs.srv import SetExecutionGuard, RevalidateManipulation, RevalidatePayloadTransition
from astribot_s1_transport_native.action import ManipulationToHold as PlanToHold
from astribot_s1_transport_native.srv import RenewHold


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['normal','delayed_ledger','revision_changed','late_invalid','geometry_model_changed'],required=True)
    p.add_argument('--observation-offset-s',type=float,default=0.,help='Offset observation stamps from local ROS clock without changing task timers')
    p.add_argument('--late-kind',choices=['revision','unknown'],default='revision')
    p.add_argument('--expect-unsafe',action='store_true',help='Diagnostic RED: require old executor to submit after known invalid raw')
    p.add_argument('--operation',choices=['PICK','PLACE'],required=True)
    p.add_argument('--physical-fixture',type=Path,required=True)
    p.add_argument('--ledger-executable',type=Path,required=True)
    p.add_argument('--domain',type=int,required=True);p.add_argument('--executable',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    assert not args.output.exists(), 'Evidence output must be new'
    assert os.environ.get('IGN_PARTITION','').startswith('m1_full_'), 'Owned unique Ignition partition required'
    assert os.environ.get('ROS_DOMAIN_ID')==str(args.domain)
    journal=Path.home()/'.local/state/astribot/transport'/f'domain_{args.domain}.jsonl'
    assert not journal.exists() and not Path(f'/tmp/astribot_transport_domain_{args.domain}.lock').exists(), 'Fresh owned domain required; do not remove unresolved records'
    groups={}
    for side in ('left','right'):
        groups[f'arm_{side}_controller']=[f'astribot_arm_{side}_joint_{i}' for i in range(1,8)]
        groups[f'gripper_{side}_controller']=[f'astribot_gripper_{side}_joint_L1']
    groups['head_controller']=[f'astribot_head_joint_{i}' for i in range(1,3)]
    groups['torso_controller']=[f'astribot_torso_joint_{i}' for i in range(1,5)]
    positions={j:0. for names in groups.values() for j in names}
    events=[];statuses=[];holds=[];goals={};canceled=set();entities=[];geometry_samples=[]
    controller_stage={};current_ros_ns=0;controller_responses=[];status_receipts=[]
    ledger_states=[];delayed=deque();physical=dict(command=7,attached=args.operation=='PLACE',revision=1)
    scene_state=PlanningScene();scene_state.world.octomap.header.frame_id='astribot_torso_base';scene_state.world.octomap.origin.orientation.w=1.
    tcp='astribot_arm_left_tcp_link';base='astribot_torso_base'
    def body():
        b=AttachedCollisionObject(link_name=tcp,touch_links=[tcp],weight=.2)
        b.object=CollisionObject(id='box',operation=CollisionObject.ADD);b.object.header.frame_id=tcp;b.object.pose.orientation.w=1.
        b.object.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.071,.071,.131])]
        b.object.primitive_poses=[Pose()];b.object.primitive_poses[0].orientation.w=1.;return b
    if physical['attached']:scene_state.robot_state.attached_collision_objects=[body()]
    else:
        o=body().object;o.header.frame_id=base;o.pose.position.x=.1;o.pose.position.z=1.;o.primitives[0].dimensions=[.06,.06,.12];scene_state.world.collision_objects=[o]
    scene_applied_at=None;revision_injected=False;late_injected=False;late_raw_sent=False;geometry_model_injected=False
    operation_order=[('PREGRASP','ARM'),('GRASP_APPROACH','ARM'),('GRASP_CONFIRM','GRIPPER'),('ATTACH_CONFIRM','ATTACH'),('LIFT','ARM'),('TRANSPORT_POSTURE','ARM')] if args.operation=='PICK' else [('PREPLACE','ARM'),('PLACE_APPROACH','ARM'),('RELEASE','GRIPPER'),('DETACH_CONFIRM','DETACH'),('RETREAT','ARM'),('STOW','ARM')]

    motion_indices=[i for i,(_,kind) in enumerate(operation_order) if kind in ('ARM','GRIPPER')]
    expected_endpoints=[];planned_endpoint=dict(positions)
    for index in motion_indices:
        active='arm_left_controller' if operation_order[index][1]=='ARM' else 'gripper_left_controller'
        for joint in groups[active]:planned_endpoint[joint]+=.005
        expected_endpoints.append(dict(planned_endpoint))
    origin=time.monotonic();stop=threading.Event();guard=dict(active=False,context='');sequence=0;envelope_epoch=1;navigation=False;scene_calls=0
    def stamp(offset=0.):
        ns=int((100.+time.monotonic()-origin+offset)*1e9);return Time(sec=ns//10**9,nanosec=ns%10**9)
    def event(kind,**values):events.append(dict(event=kind,wall=time.monotonic(),**values))
    rclpy.init()
    probe=Node('m1_domain_probe_'+uuid.uuid4().hex)
    discovery_until=time.monotonic()+2.
    while time.monotonic()<discovery_until:rclpy.spin_once(probe,timeout_sec=.1)
    foreign=[n for n in probe.get_node_names() if n!=probe.get_name()]
    assert not foreign, ('domain already has nodes',foreign)
    event('isolated_domain_checked',domain=args.domain);probe.destroy_node()
    fixture=Node('m1_synthetic_controller_fixture');client=Node('m1_protocol_client');group=ReentrantCallbackGroup()
    executor=MultiThreadedExecutor(num_threads=16);executor.add_node(fixture);executor.add_node(client)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    def send(name):
        def callback(req,res):
            key=tuple(req.goal_id.uuid);stage='plan'
            if name!='mtc':
                ordinal=controller_stage.get(name,0);assert ordinal<5
                controller_stage[name]=ordinal+1;stage=operation_order[motion_indices[ordinal]][0]
                assert list(req.goal.trajectory.joint_names)==groups[name]
                assert list(req.goal.trajectory.points[-1].positions)==[expected_endpoints[ordinal][j] for j in groups[name]]
            goals[key]=(name,req.goal,stage)
            event('send',controller=name,uuid=bytes(key).hex(),stage=stage,ros_ns=current_ros_ns)
            res.accepted=True;res.stamp=stamp();event('accepted',controller=name,uuid=bytes(key).hex(),stage=stage);return res
        return callback
    def cancel(name):
        def callback(req,res):
            canceled.add(tuple(req.goal_info.goal_id.uuid));res.return_code=0;res.goals_canceling=[req.goal_info];event('cancel_ack',controller=name,uuid=bytes(req.goal_info.goal_id.uuid).hex());return res
        return callback
    def result(name):
        def callback(req,res):
            key=tuple(req.goal_id.uuid);goal=goals[key][1]
            if name=='mtc':
                res.status=5 if key in canceled else 4;res.result.success=res.status==4
                res.result.context_id=goal.context_id
                planned=dict(positions)
                for index,(stage_id,kind) in enumerate(operation_order):
                    s=ManipulationStage(stage_id=stage_id,kind=kind)
                    s.expected_start.joint_state.name=list(planned);s.expected_start.joint_state.position=list(planned.values())
                    attached_before=args.operation=='PLACE' if index<=3 else args.operation=='PICK'
                    s.expected_start.attached_collision_objects=[body()] if attached_before else []
                    if kind in ('ARM','GRIPPER'):
                        active=groups['arm_left_controller'] if kind=='ARM' else groups['gripper_left_controller']
                        before=[planned[j] for j in active]
                        for j in active:planned[j]+=.005
                        s.trajectory.joint_trajectory.joint_names=active
                        s.trajectory.joint_trajectory.points=[JointTrajectoryPoint(positions=before,time_from_start=Duration()),JointTrajectoryPoint(positions=[planned[j] for j in active],time_from_start=Duration(nanosec=250000000))]
                    res.result.stages.append(s)
            else:
                stop.wait(.12)
                res.status=4
                res.result.error_code=0
                for joint,value in zip(goal.trajectory.joint_names,goal.trajectory.points[-1].positions):positions[joint]=value
            event('terminal',controller=name,uuid=bytes(key).hex(),stage=goals[key][2],status=res.status,ros_ns=current_ros_ns);return res
        return callback
    action_status_publishers={}
    for name,action,endpoint in [(n,FJT,'/'+n+'/follow_joint_trajectory') for n in groups]+[('mtc',PlanManipulation,'/transport/plan_manipulation')]:
        action_status_publishers[name]=fixture.create_publisher(GoalStatusArray,endpoint+'/_action/status',qos_profile_action_status_default)
        entities.extend([fixture.create_service(action.Impl.SendGoalService,endpoint+'/_action/send_goal',send(name),callback_group=group),fixture.create_service(action.Impl.GetResultService,endpoint+'/_action/get_result',result(name),callback_group=group),fixture.create_service(action.Impl.CancelGoalService,endpoint+'/_action/cancel_goal',cancel(name),callback_group=group),action_status_publishers[name],fixture.create_publisher(action.Impl.FeedbackMessage,endpoint+'/_action/feedback',10)])
    def controllers(req,res):
        event('controllers_read')
        res.controller=[ControllerState(name=n,state='active',type='joint_trajectory_controller/JointTrajectoryController',claimed_interfaces=[j+'/position' for j in js]) for n,js in groups.items()]
        controller_responses.append(time.monotonic())
        event('controllers_response_ready',controllers=list(groups));return res
    def scene(req,res):
        nonlocal scene_calls,late_injected
        if args.mode=='late_invalid' and req.components.components==1023 and any(e['event']=='payload_revalidate' for e in events) and not late_injected:
            late_injected=True
            if args.late_kind=='revision':physical['revision']=3
            event('late_invalid_injected',invalid_kind=args.late_kind)
            until=time.monotonic()+.12
            while not late_raw_sent and time.monotonic()<until:stop.wait(.005)
            assert late_raw_sent
            stop.wait(.04) # deliver the unpaired raw before returning this full readback
            assert ledger_states[-1].confirmed and ledger_states[-1].observation.revision==2
            event('late_readback_replied',raw_kind=args.late_kind,ledger_revision=2)
        scene_calls+=1;res.scene=copy.deepcopy(scene_state)
        res.scene.robot_state.joint_state.name=list(positions);res.scene.robot_state.joint_state.position=list(positions.values())
        event('scene_read',components=req.components.components,attached=[o.object.id for o in res.scene.robot_state.attached_collision_objects])
        return res
    def apply_scene(req,res):
        nonlocal scene_applied_at
        assert req.scene.is_diff and req.scene.robot_state.is_diff
        assert len([e for e in events if e['event']=='physical_command'])==1
        for o in req.scene.world.collision_objects:
            scene_state.world.collision_objects=[v for v in scene_state.world.collision_objects if v.id!=o.id]
            if o.operation==o.ADD:scene_state.world.collision_objects.append(copy.deepcopy(o))
        for o in req.scene.robot_state.attached_collision_objects:
            scene_state.robot_state.attached_collision_objects=[v for v in scene_state.robot_state.attached_collision_objects if v.object.id!=o.object.id]
            if o.object.operation==o.object.ADD:scene_state.robot_state.attached_collision_objects.append(copy.deepcopy(o))
        scene_applied_at=time.monotonic();event('scene_applied');res.success=True;return res
    def revalidate(req,res):
        raise AssertionError('no occupancy revalidation expected')
    def revalidate_payload(req,res):
        assert req.context_id=='context' and req.start_index==4 and req.transaction_id
        assert ledger_states and ledger_states[-1].confirmed
        assert ledger_states[-1].observation.revision==physical['revision']
        assert len([e for e in events if e['event']=='send' and e['controller']!='mtc'])==18
        assert bool(req.scene.robot_state.attached_collision_objects)==(args.operation=='PICK')
        event('payload_revalidate',source_revision=ledger_states[-1].observation.revision,transaction=req.transaction_id)
        res.context_id=req.context_id;res.transaction_id=req.transaction_id;res.success=True;res.reason='fixture remaining payload trajectory check';return res
    def physical_command(message):
        value=json.loads(message.data)
        assert value['command']==8 and value['attached']==(args.operation=='PICK')
        assert len([e for e in events if e['event']=='send' and e['controller']!='mtc'])==18
        expected=[0.,0.,0.] if value['attached'] else [.1,0.,1.13]
        assert all(abs(a-b)<1e-9 for a,b in zip(value['position'],expected))
        physical.update(command=value['command'],attached=value['attached'],revision=2)
        event('physical_command',**value)
    def receive_ledger(value):
        if not ledger_states or value.confirmed!=ledger_states[-1].confirmed or value.observation.revision!=ledger_states[-1].observation.revision:
            event('ledger',confirmed=value.confirmed,revision=value.observation.revision,reason=value.reason)
        ledger_states.append(value)
    def set_guard(req,res):guard.update(active=req.enable,context=req.context_id);res.accepted=True;event('guard',active=req.enable,context=req.context_id);return res
    def revoke(req,res):
        nonlocal envelope_epoch,navigation
        assert not req.envelope.transport_ready
        navigation=False;res.accepted=True;res.epoch=envelope_epoch;event('navigation_revoked' if res.accepted else 'navigation_revoke_rejected');return res
    entities.extend([fixture.create_service(ListControllers,'/controller_manager/list_controllers',controllers,callback_group=group),fixture.create_service(GetPlanningScene,'/get_planning_scene',scene,callback_group=group),fixture.create_service(SetExecutionGuard,'/transport/execution_guard/set',set_guard,callback_group=group),fixture.create_service(SetRobotEnvelope,'/navigation/set_robot_envelope',revoke,callback_group=group)])
    entities.append(fixture.create_service(RevalidateManipulation,'/transport/revalidate_manipulation',revalidate,callback_group=group))
    entities.append(fixture.create_service(ApplyPlanningScene,'/apply_planning_scene',apply_scene,callback_group=group))
    entities.append(fixture.create_service(RevalidatePayloadTransition,'/transport/revalidate_payload_transition',revalidate_payload,callback_group=group))
    entities.append(fixture.create_subscription(String,'/fixture/physical_command',physical_command,10,callback_group=group))
    entities.append(fixture.create_subscription(AttachmentState,'/payload/attachment_state',receive_ledger,10,callback_group=group))
    observations=fixture.create_publisher(AttachmentObservation,'/payload/attachment_observation',32)
    ledger_input=fixture.create_publisher(AttachmentObservation,'/fixture/ledger_observation',32)
    diagnostics=fixture.create_publisher(String,'/payload/simulation_inventory_diagnostics',32)
    physical_input=fixture.create_publisher(String,'/fixture/physical_input',10)
    joint_pub=fixture.create_publisher(JointState,'/joint_states',qos_profile_sensor_data)
    tf_pub=fixture.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    transform=TransformStamped();transform.header.frame_id=base;transform.child_frame_id='wrist';transform.transform.translation.x=.1;transform.transform.translation.z=1.;transform.transform.rotation.w=1.
    tf_pub.publish(TFMessage(transforms=[transform]))
    clock=fixture.create_publisher(Clock,'/clock',10);geometry=fixture.create_publisher(RobotGeometryState,'/navigation/geometry_state',10)
    slam_pose=fixture.create_publisher(PoseWithCovarianceStamped,'/slam/pose',qos_profile_sensor_data);envelopes=fixture.create_publisher(NavigationEnvelopeV2,'/navigation/envelope_v2',10)
    guard_pub=fixture.create_publisher(ExecutionGuardStatus,'/transport/execution_guard/status',10)
    def publish():
        nonlocal sequence,revision_injected,late_raw_sent,geometry_model_injected
        sequence+=1
        at=stamp(-.02+args.observation_offset_s);at_ns=at.sec*10**9+at.nanosec
        until_ns=at_ns+300000000;until=Time(sec=until_ns//10**9,nanosec=until_ns%10**9)
        if args.mode=='revision_changed' and scene_applied_at and not revision_injected and time.monotonic()-scene_applied_at>.03:
            physical['revision']=3;revision_injected=True;event('unexpected_revision')
        snapshot=dict(physical);joint_snapshot=dict(positions)
        o=AttachmentObservation(environment='simulation',session_id='fixture_session',source_id='fixture_source',source_epoch='fixture_epoch',clock_epoch=1,sequence=sequence,revision=snapshot['revision'],observed_at=at,valid_until=until,full_inventory=True,transaction_id='fixture_'+str(snapshot['revision']))
        o.status=o.ATTACHED if snapshot['attached'] else o.EMPTY;o.objects=[body()] if snapshot['attached'] else []
        if args.mode=='late_invalid' and late_injected and args.late_kind=='unknown':o.status=o.UNKNOWN;o.full_inventory=False;o.objects=[]
        observations.publish(o)
        if args.mode=='late_invalid' and late_injected and not late_raw_sent:
            late_raw_sent=True;event('late_invalid_published',revision=o.revision,full_inventory=o.full_inventory,status=o.status,capture_ns=at_ns)
        delayed.append((time.monotonic(),copy.deepcopy(o)))
        delay=.15 if args.mode in ('delayed_ledger','revision_changed') and snapshot['command']==8 else 0.
        if args.mode=='late_invalid' and late_injected:delay=.25
        while delayed and time.monotonic()-delayed[0][0]>=delay:ledger_input.publish(delayed.popleft()[1])
        detail=dict(world='fixture_world',robot_model='fixture_robot',policy='kinematic_inventory_v1',source_epoch='fixture_epoch',clock_epoch=1,revision=o.revision,sequence=sequence,stamp_ns=at_ns,models=[[136,'fixture_robot'],[41,'fixture']],reason='ATTACHED_INVENTORY_OBSERVED' if snapshot['attached'] else 'EMPTY_INVENTORY_OBSERVED',execution=[dict(entity=41,epoch='fixture_plugin',clock_epoch=1,accepted=snapshot['command'],applied=snapshot['command'],attached=snapshot['attached'],parent=187)])
        if not (args.mode=='late_invalid' and late_injected):diagnostics.publish(String(data=json.dumps(detail)))
        physical_input.publish(String(data=json.dumps(dict(stamp_ns=at_ns,initial_attached=args.operation=='PLACE'))))
        g=RobotGeometryState();g.header.stamp=at;g.header.frame_id=base;g.valid_until=until;g.source_id='m1_fixture';g.sequence=sequence;g.model_revision='model';g.clock_epoch=1
        state=ledger_states[-1] if ledger_states else None
        g.complete=g.attachment_state_confirmed=bool(state and state.confirmed)
        g.attachment_revision=state.attachment_revision if state else ''
        g.attachment_ids=[v.object.id for v in state.observation.objects] if state and state.confirmed else []
        g.joints.header.stamp=at;g.joints.name=list(joint_snapshot);g.joints.position=list(joint_snapshot.values());g.joint_position_error_bounds=[.004]*22;g.joint_source_stamps=[at]*22
        joint_pub.publish(g.joints) # Actual joint evidence remains available during ledger reconciliation.
        if not g.complete:
            # Match the real geometry publisher's error message: identity fields
            # not established by a complete snapshot remain unset, not invented.
            g=RobotGeometryState();g.header.frame_id=base;g.published_at=stamp();g.source_id='m1_fixture';g.sequence=sequence;g.reason='ATTACHMENT_UNCONFIRMED'
        elif args.mode=='geometry_model_changed' and state.observation.revision==2:
            g.model_revision='different_model'
            if not geometry_model_injected:
                geometry_model_injected=True;event('complete_geometry_model_changed',source_id=g.source_id,clock_epoch=g.clock_epoch,model_revision=g.model_revision,attachment_revision=g.attachment_revision)
        geometry.publish(g)
        geometry_samples.append(dict(wall=time.monotonic(),capture_ns=at_ns,confirmed=g.complete,positions=joint_snapshot,source_id=g.source_id,clock_epoch=g.clock_epoch,model_revision=g.model_revision,attachment_revision=g.attachment_revision))
        pose=PoseWithCovarianceStamped();pose.header.stamp=at;pose.header.frame_id='map';pose.pose.pose.orientation.w=1.;slam_pose.publish(pose)
        e=NavigationEnvelopeV2();e.header.stamp=at;e.valid_until=stamp(.28);e.coordinator_session_id='m1_coordinator';e.epoch=envelope_epoch;e.mode=e.FIXED_POSTURE if navigation else e.HOLD;e.navigation_allowed=navigation;envelopes.publish(e)
        guard_pub.publish(ExecutionGuardStatus(stamp=at,joint_stamp=at,context_id=guard['context'],active=guard['active'],healthy=guard['active'],reason='EXECUTION_WITHIN_BOUNDS'))
    def publish_clock():
        nonlocal current_ros_ns
        value=stamp();current_ros_ns=value.sec*10**9+value.nanosec;clock.publish(Clock(clock=value))
    # Independent clock delivery precedes the 20ms-old source capture. Publishing
    # /clock and source together on separate DDS topics has no ordering guarantee.
    entities.append(fixture.create_timer(.005,publish_clock,callback_group=group))
    entities.append(fixture.create_timer(.04,publish,callback_group=group))
    def record_status(message):
        statuses.append(json.loads(message.data));status_receipts.append(time.monotonic())
    entities.append(client.create_subscription(String,'/transport/hold_executor/status',record_status,10))
    entities.append(client.create_subscription(ArmHoldStatus,'/navigation/arm_hold',lambda m:holds.append((time.monotonic(),m.hold_confirmed)),10))
    action=ActionClient(client,PlanToHold,'/transport/manipulate_to_hold');renew=client.create_client(RenewHold,'/transport/hold_executor/renew')
    args.output.parent.mkdir(exist_ok=True,parents=True);log=args.output.with_suffix('.log').open('w');process=None;handle=None;terminal=None;auxiliaries=[];aux_logs=[];renew_seq=0;renew_last=0.;renew_future=None;outcome={}
    def wait(predicate,seconds,label):
        nonlocal renew_seq,renew_last,renew_future
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if predicate():return
            for aux in auxiliaries:
                if aux.poll() is not None:raise AssertionError("fixture/ledger exited "+str(aux.returncode))
            if process is not None and process.poll() is not None:raise AssertionError('executor exited '+str(process.returncode))
            if renew_future is not None and renew_future.done():renew_future=None
            if statuses and statuses[-1]['phase'] in ('1','2','3') and renew_future is None and time.monotonic()-renew_last>.2:
                s=statuses[-1];renew_seq+=1;renew_future=renew.call_async(RenewHold.Request(lease_id=s['lease_id'],resource_epoch=s['epoch'],sequence=renew_seq));renew_last=time.monotonic()
            stop.wait(.01)
        raise AssertionError(label)
    try:
        urdf='<robot name="fixture"><link name="astribot_torso_base"/><link name="wrist"/><link name="astribot_arm_left_tcp_link"/><joint name="fixture_arm" type="continuous"><parent link="astribot_torso_base"/><child link="wrist"/><origin xyz=".1 0 1"/><axis xyz="0 0 1"/></joint><joint name="fixture_tcp" type="fixed"><parent link="wrist"/><child link="astribot_arm_left_tcp_link"/></joint></robot>'
        physical_log=args.output.with_suffix('.physical.log').open('w');aux_logs.append(physical_log)
        auxiliaries.append(subprocess.Popen([str(args.physical_fixture.resolve())],stdout=physical_log,stderr=subprocess.STDOUT))
        ledger_log=args.output.with_suffix('.ledger.log').open('w');aux_logs.append(ledger_log)
        auxiliaries.append(subprocess.Popen([str(args.ledger_executable.resolve()),'--ros-args','-p','use_sim_time:=true','-p','session_id:=fixture_session','-p','source_id:=fixture_source','-p','journal_path:='+str(args.output.with_suffix('.ledger.jsonl').resolve()),'-r','/payload/attachment_observation:=/fixture/ledger_observation'],stdout=ledger_log,stderr=subprocess.STDOUT))
        command=[str(args.executable.resolve()),'--ros-args','-p','use_sim_time:=true','-p','simulation_commissioning:=true']
        params={'payload_model':'fixture','payload_object_id':'box','payload_world':'fixture_world','payload_robot_model':'fixture_robot','payload_session_id':'fixture_session','payload_source_id':'fixture_source','payload_size_xyz':'[0.06, 0.06, 0.12]','payload_model_from_root':'[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]','robot_description':urdf}
        for key,value in params.items():command+=['-p',key+':='+value]
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        owned=[process]+auxiliaries
        event('owned_processes',partition=os.environ['IGN_PARTITION'],boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),processes=[dict(pid=v.pid,start_ticks=Path('/proc/'+str(v.pid)+'/stat').read_text().split(') ',1)[1].split()[19],argv=v.args) for v in owned])
        event('artifact_identity',hashes={str(v.resolve()):hashlib.sha256(v.read_bytes()).hexdigest() for v in [Path(__file__),args.executable,args.physical_fixture,args.ledger_executable]},environment={k:os.environ.get(k,'') for k in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','IGN_PARTITION','LD_LIBRARY_PATH','AMENT_PREFIX_PATH']})
        wait(lambda:action.server_is_ready() and len(statuses)>8 and ledger_states and ledger_states[-1].confirmed,10,'startup')
        # A second query well before the native 500ms pending timeout, followed
        # by native status, establishes the normal request/response cadence.
        # Never retry the task goal or extend the product claims lifetime.
        wait(lambda:len(controller_responses)>=2 and
             0.<controller_responses[-1]-controller_responses[-2]<.2 and
             status_receipts[-1]>controller_responses[-1] and
             time.monotonic()-controller_responses[-2]<.3,
             10,'controller query/response cadence not ready')
        event('controller_handshake',response_wall=controller_responses[-2:],native_status_wall=status_receipts[-1])
        wait(lambda:slam_pose.get_subscription_count()>0,3,'SLAM stop-window subscriber')
        stop_window_end=time.monotonic()+.7
        wait(lambda:time.monotonic()>=stop_window_end,2,'initial SLAM stop window')
        goal=PlanToHold.Goal(task_id='test_'+uuid.uuid4().hex,request_id='request',context_id='context',operation=args.operation,object_id='box',grasp_width_m=.04)
        goal.pre_target.header.frame_id=goal.target.header.frame_id=base;goal.pre_target.pose.orientation.w=goal.target.pose.orientation.w=1.;goal.exit_targets=[goal.pre_target];goal.touch_links=[tcp]
        goal.target.pose.position.x=.1;goal.target.pose.position.z=1.
        event('admission_snapshot',action_status_subscribers={name:pub.get_subscription_count() for name,pub in action_status_publishers.items()},nodes=client.get_node_names(),observation_subscribers=observations.get_subscription_count(),diagnostic_subscribers=diagnostics.get_subscription_count(),geometry_subscribers=geometry.get_subscription_count(),clock_subscribers=clock.get_subscription_count(),last_status=statuses[-1],ledger_confirmed=ledger_states[-1].confirmed)
        sent=action.send_goal_async(goal);wait(sent.done,4,'accept');handle=sent.result()
        event('admission_result',accepted=handle.accepted)
        if not handle.accepted:
            before=len(statuses);wait(lambda:len(statuses)>=before+3,2,'admission rejection status')
            raise AssertionError(('goal rejected',statuses[-3:]))
        terminal=handle.get_result_async()
        if args.mode in ('normal','delayed_ledger'):
            wait(lambda:any(h for _,h in holds) or terminal.done(),35,'no full operation HOLD')
            assert any(h for _,h in holds), statuses[-10:]
            assert len([e for e in events if e['event']=='send' and e['controller']!='mtc'])==30
            assert len([e for e in events if e['event']=='physical_command'])==1
            assert len([e for e in events if e['event']=='payload_revalidate'])==1
            assert not terminal.done() and not guard['active']
            revalidation=next(i for i,e in enumerate(events) if e['event']=='payload_revalidate')
            later=[e for e in events[revalidation+1:] if e['event']=='send' and e['controller']!='mtc']
            assert len(later)==12
            first_later=next(i for i in range(revalidation+1,len(events)) if events[i]['event']=='send' and events[i]['controller']!='mtc')
            assert any(e['event']=='scene_read' and e['components']==1023 for e in events[revalidation+1:first_later])
            if args.mode=='delayed_ledger':
                applied=next(e['wall'] for e in events if e['event']=='scene_applied')
                confirmed=next(e['wall'] for e in events if e['event']=='ledger' and e['confirmed'] and e['revision']==2)
                assert confirmed>applied
                physical_at=next(e['wall'] for e in events if e['event']=='physical_command')
                revalidated_at=events[revalidation]['wall']
                unknown=[v for v in geometry_samples if physical_at<v['wall']<revalidated_at and not v['confirmed']]
                assert unknown and all(v['model_revision']==v['attachment_revision']=='' and v['clock_epoch']==0 for v in unknown)
                assert not any(e['event']=='send' and e['controller']!='mtc' and physical_at<e['wall']<revalidated_at for e in events)
                assert not any(s['reason']=='MTC_CONTEXT_CHANGED' for s in statuses)
                event('incomplete_geometry_wait_verified',samples=len(unknown),first_wall=unknown[0]['wall'],last_wall=unknown[-1]['wall'])
            cancel_future=handle.cancel_goal_async();wait(cancel_future.done,3,'cancel');assert cancel_future.result().goals_canceling
        wait(terminal.done,15,'terminal');result_value=terminal.result();outcome=dict(status=result_value.status,reason=result_value.result.reason,resources_released=result_value.result.resources_released)
        if args.mode=='late_invalid':
            assert late_raw_sent and not any(h for _,h in holds)
            injected=next(e['wall'] for e in events if e['event']=='late_invalid_published')
            subsequent=[e for e in events if e['event']=='send' and e['controller']!='mtc' and e['wall']>injected]
            assert bool(subsequent)==args.expect_unsafe, ('late invalid barrier',subsequent,outcome)
            if not args.expect_unsafe:
                assert len([e for e in events if e['event']=='send' and e['controller']!='mtc'])==18
                assert not outcome['resources_released'] and 'PAYLOAD_RAW_' in outcome['reason']
            event('late_invalid_result',unsafe_submissions=len(subsequent),expected_old_defect=args.expect_unsafe)
        elif args.mode=='geometry_model_changed':
            assert geometry_model_injected and not any(h for _,h in holds)
            assert len([e for e in events if e['event']=='send' and e['controller']!='mtc'])==18
            assert not any(e['event']=='payload_revalidate' for e in events)
            assert not outcome['resources_released'] and outcome['reason']=='RESOURCE_RECOVERY_REQUIRED:MTC_CONTEXT_CHANGED'
        elif args.mode=='revision_changed':
            assert not any(h for _,h in holds)
            assert len([e for e in events if e['event']=='send' and e['controller']!='mtc'])==18
            assert not any(e['event']=='payload_revalidate' for e in events)
            assert not outcome['resources_released'] and 'RECONCILIATION_CONTEXT_CHANGED' in outcome['reason']
        else:assert outcome['resources_released'] and outcome['reason']=='TASK_CANCELED'
        # Verify every actual child UUID, endpoint and inter-stage settling
        # window independently of the executor's own stage_confirmed journal.
        completed_count=3 if args.mode in ('revision_changed','late_invalid','geometry_model_changed') else 5
        for ordinal,index in enumerate(motion_indices[:completed_count]):
            stage=operation_order[index][0]
            sends=[e for e in events if e['event']=='send' and e['stage']==stage]
            terminals=[e for e in events if e['event']=='terminal' and e['stage']==stage]
            assert len(sends)==len(terminals)==6
            assert {e['controller'] for e in sends}==set(groups)
            assert {e['uuid'] for e in sends}=={e['uuid'] for e in terminals}
            assert all(e['status']==4 for e in terminals)
            end=max(e['wall'] for e in terminals);end_ros=max(e['ros_ns'] for e in terminals)
            if ordinal+1<completed_count:
                next_stage=operation_order[motion_indices[ordinal+1]][0]
                cutoff=min(e['wall'] for e in events if e['event']=='send' and e['stage']==next_stage)
            elif args.mode in ('revision_changed','late_invalid','geometry_model_changed'):
                cutoff=next(e['wall'] for e in events if e['event']=='physical_command')
            else:cutoff=next(at for at,confirmed in holds if confirmed)
            # The payload transaction may sit between motion segments, so also
            # bound GRIPPER settling by the first physical command itself.
            if ordinal==2:cutoff=min(cutoff,next(e['wall'] for e in events if e['event']=='physical_command'))
            stable=[v for v in geometry_samples if end<v['wall']<cutoff and v['capture_ns']>end_ros and v['confirmed'] and v['positions']==expected_endpoints[ordinal]]
            assert len(stable)>=3, ('missing independent settled samples',stage)
            assert stable[-1]['wall']-stable[0]['wall']>=.5, ('steady settling',stage)
            assert stable[-1]['capture_ns']-stable[0]['capture_ns']>=500000000, ('source settling',stage)
            event('settling_verified',stage=stage,uuids=sorted(e['uuid'] for e in sends),source_span_ns=stable[-1]['capture_ns']-stable[0]['capture_ns'],steady_span=stable[-1]['wall']-stable[0]['wall'])
        records=[json.loads(line) for line in journal.read_text().splitlines()]
        stages=[v['details']['stage_id'] for v in records if v['event']=='stage_confirmed']
        expected=[name for name,kind in operation_order if kind in ('ARM','GRIPPER')]
        assert stages[:3]==expected[:3]
        if args.mode not in ('revision_changed','late_invalid','geometry_model_changed'):assert stages==expected
        payload_confirmed=[v for v in records if v['event']=='payload_transaction_confirmed']
        assert len(payload_confirmed)==(0 if args.mode in ('revision_changed','geometry_model_changed') or (args.mode=='late_invalid' and not args.expect_unsafe) else 1)
        if args.mode in ('revision_changed','late_invalid','geometry_model_changed') and not args.expect_unsafe:assert records[-1]['phase']==5
        if args.mode=='geometry_model_changed':
            assert next(v['reason'] for v in records if v['event']=='stop_requested')=='MTC_CONTEXT_CHANGED'
        if args.mode=='late_invalid' and not args.expect_unsafe:
            initial=next(v['reason'] for v in records if v['event']=='stop_requested')
            assert initial.startswith('PAYLOAD_RAW_')
            assert outcome['reason']==records[-1]['reason']=='RESOURCE_RECOVERY_REQUIRED:'+initial
            assert any(s['reason']==initial and s.get('cleanup_reason')=='GEOMETRY_UNCONFIRMED' for s in statuses)
        assert not any(v.get('reason')=='RESOURCE_CLOCK_RESET' for v in records)
        event('assertions_passed',confirmed_stages=stages,payload_confirmed=len(payload_confirmed))
    finally:
        # Keep the evidence producers alive while bounded action cancellation
        # collects terminal state. Unknown resources stay in the journal.
        if process is not None and process.poll() is None and handle is not None and handle.accepted and terminal is not None and not terminal.done():
            try:
                pending_cancel=handle.cancel_goal_async()
                wait(pending_cancel.done,2,'cleanup cancel acknowledgment')
                wait(terminal.done,11,'cleanup terminal barrier')
                event('cleanup_terminal',released=terminal.result().result.resources_released,reason=terminal.result().result.reason)
            except Exception as error:
                event('cleanup_unconfirmed',error=repr(error));outcome['cleanup_unconfirmed']=repr(error)
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait();outcome['forced_cleanup']=True
        for owned in auxiliaries:
            if owned.poll() is None:
                owned.send_signal(signal.SIGINT)
                try:owned.wait(timeout=5)
                except subprocess.TimeoutExpired:owned.kill();owned.wait();outcome['forced_aux_cleanup']=True
        event('owned_process_exit',processes=[dict(pid=v.pid,returncode=v.returncode) for v in ([process] if process else [])+auxiliaries])
        for file in aux_logs:file.close()
        log.close();stop.set();executor.shutdown(timeout_sec=3);thread.join(timeout=3);fixture.destroy_node();client.destroy_node();rclpy.shutdown()
        args.output.write_text(json.dumps(dict(mode=args.mode,observation_offset_s=args.observation_offset_s,late_kind=args.late_kind,expect_unsafe=args.expect_unsafe,operation=args.operation,domain=args.domain,events=events,statuses=statuses,holds=holds,geometry_samples=geometry_samples,outcome=outcome,journal=str(journal),evidence='synthetic six-stage controllers and Ignition endpoint; real C++ ledger; no Gazebo, real MTC planning or hardware acceptance'),indent=2))

if __name__=='__main__':main()
