#!/usr/bin/env python3
"""Owned ROS protocol fixture. Synthetic controllers; no Gazebo/robot motion."""
import argparse
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
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from moveit_msgs.srv import GetPlanningScene
from trajectory_msgs.msg import JointTrajectoryPoint
from astribot_navigation_msgs.msg import RobotGeometryState, ArmHoldStatus, NavigationEnvelopeV2
from astribot_navigation_msgs.srv import SetRobotEnvelope
from astribot_transport_msgs.action import PlanManipulation
from astribot_transport_msgs.msg import ManipulationStage, ExecutionGuardStatus
from astribot_transport_msgs.srv import SetExecutionGuard
from astribot_s1_transport_native.action import PlanToHold
from astribot_s1_transport_native.srv import RenewHold


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['normal','wrong_context','start_changed','scene_changed','pending_cancel','unknown_result','no_endpoint'],required=True)
    p.add_argument('--domain',type=int,required=True);p.add_argument('--executable',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();assert os.environ.get('ROS_DOMAIN_ID')==str(args.domain)
    journal=Path.home()/'.local/state/astribot/transport'/f'domain_{args.domain}.jsonl'
    assert not journal.exists() and not Path(f'/tmp/astribot_transport_domain_{args.domain}.lock').exists(), 'Fresh owned domain required; do not remove unresolved records'
    groups={}
    for side in ('left','right'):
        groups[f'arm_{side}_controller']=[f'astribot_arm_{side}_joint_{i}' for i in range(1,8)]
        groups[f'gripper_{side}_controller']=[f'astribot_gripper_{side}_joint_L1']
    groups['head_controller']=[f'astribot_head_joint_{i}' for i in range(1,3)]
    groups['torso_controller']=[f'astribot_torso_joint_{i}' for i in range(1,5)]
    positions={j:0. for names in groups.values() for j in names}
    events=[];statuses=[];holds=[];goals={};canceled=set();entities=[]
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
            key=tuple(req.goal_id.uuid);goals[key]=(name,req.goal);event('send',controller=name)
            if name=='mtc' and args.mode=='pending_cancel':stop.wait(1.2)
            res.accepted=True;res.stamp=stamp();event('accepted',controller=name);return res
        return callback
    def cancel(name):
        def callback(req,res):
            canceled.add(tuple(req.goal_info.goal_id.uuid));res.return_code=0;res.goals_canceling=[req.goal_info];event('cancel_ack',controller=name);return res
        return callback
    def result(name):
        def callback(req,res):
            key=tuple(req.goal_id.uuid);goal=goals[key][1]
            if name=='mtc':
                if args.mode=='pending_cancel':
                    deadline=time.monotonic()+5
                    while key not in canceled and time.monotonic()<deadline and not stop.is_set():stop.wait(.01)
                res.status=5 if key in canceled else 4;res.result.success=res.status==4
                res.result.context_id='wrong' if args.mode=='wrong_context' else goal.context_id
                order=[('PREGRASP','ARM'),('GRASP_APPROACH','ARM'),('GRASP_CONFIRM','GRIPPER'),('ATTACH_CONFIRM','ATTACH'),('LIFT','ARM'),('TRANSPORT_POSTURE','ARM')]
                for stage_id,kind in order:
                    s=ManipulationStage(stage_id=stage_id,kind=kind);s.expected_start=goal.scene.robot_state
                    s.trajectory.joint_trajectory.joint_names=groups['arm_left_controller']
                    s.trajectory.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.]*7,time_from_start=Duration()),JointTrajectoryPoint(positions=[.1]*7,time_from_start=Duration(nanosec=250000000))]
                    res.result.stages.append(s)
                if args.mode=='start_changed':positions[groups['arm_left_controller'][0]]=.04;stop.wait(.15)
            else:
                stop.wait(.12)
                res.status=0 if args.mode=='unknown_result' and name=='arm_left_controller' else 4
                res.result.error_code=0
                if args.mode!='no_endpoint':
                    for joint,value in zip(goal.trajectory.joint_names,goal.trajectory.points[-1].positions):positions[joint]=value
            event('terminal',controller=name,status=res.status);return res
        return callback
    for name,action,endpoint in [(n,FJT,'/'+n+'/follow_joint_trajectory') for n in groups]+[('mtc',PlanManipulation,'/transport/plan_manipulation')]:
        entities.extend([fixture.create_service(action.Impl.SendGoalService,endpoint+'/_action/send_goal',send(name),callback_group=group),fixture.create_service(action.Impl.GetResultService,endpoint+'/_action/get_result',result(name),callback_group=group),fixture.create_service(action.Impl.CancelGoalService,endpoint+'/_action/cancel_goal',cancel(name),callback_group=group),fixture.create_publisher(GoalStatusArray,endpoint+'/_action/status',qos_profile_action_status_default),fixture.create_publisher(action.Impl.FeedbackMessage,endpoint+'/_action/feedback',10)])
    def controllers(req,res):
        res.controller=[ControllerState(name=n,state='active',type='joint_trajectory_controller/JointTrajectoryController',claimed_interfaces=[j+'/position' for j in js]) for n,js in groups.items()];return res
    def scene(req,res):
        nonlocal scene_calls
        scene_calls+=1;res.scene.robot_state.joint_state.name=list(positions);res.scene.robot_state.joint_state.position=list(positions.values())
        if args.mode=='scene_changed' and scene_calls>1:res.scene.allowed_collision_matrix.default_entry_names=['obstacle'];res.scene.allowed_collision_matrix.default_entry_values=[True]
        return res
    def set_guard(req,res):guard.update(active=req.enable,context=req.context_id);res.accepted=True;event('guard',active=req.enable);return res
    def revoke(req,res):
        nonlocal envelope_epoch,navigation
        assert not req.envelope.transport_ready;navigation=False;envelope_epoch+=1;res.accepted=True;res.epoch=envelope_epoch;event('navigation_revoked');return res
    entities.extend([fixture.create_service(ListControllers,'/controller_manager/list_controllers',controllers,callback_group=group),fixture.create_service(GetPlanningScene,'/get_planning_scene',scene,callback_group=group),fixture.create_service(SetExecutionGuard,'/transport/execution_guard/set',set_guard,callback_group=group),fixture.create_service(SetRobotEnvelope,'/navigation/set_robot_envelope',revoke,callback_group=group)])
    clock=fixture.create_publisher(Clock,'/clock',10);geometry=fixture.create_publisher(RobotGeometryState,'/navigation/geometry_state',10)
    odom=fixture.create_publisher(Odometry,'/odom',qos_profile_sensor_data);envelopes=fixture.create_publisher(NavigationEnvelopeV2,'/navigation/envelope_v2',10)
    guard_pub=fixture.create_publisher(ExecutionGuardStatus,'/transport/execution_guard/status',10)
    def publish():
        nonlocal sequence
        at=stamp();clock.publish(Clock(clock=at));sequence+=1
        g=RobotGeometryState();g.header.stamp=at;g.header.frame_id='astribot_torso_base';g.valid_until=stamp(.3);g.source_id='m1_fixture';g.sequence=sequence;g.model_revision='model';g.attachment_revision='empty'
        g.complete=g.attachment_state_confirmed=True;g.joints.name=list(positions);g.joints.position=list(positions.values());g.joint_position_error_bounds=[.004]*22;g.joint_source_stamps=[at]*22;geometry.publish(g)
        o=Odometry();o.header.stamp=at;o.header.frame_id='odom';o.child_frame_id='astribot_torso_base';o.pose.pose.orientation.w=1.;odom.publish(o)
        e=NavigationEnvelopeV2();e.header.stamp=at;e.valid_until=stamp(.3);e.coordinator_session_id='m1_coordinator';e.epoch=envelope_epoch;e.mode=e.FIXED_POSTURE if navigation else e.HOLD;e.navigation_allowed=navigation;envelopes.publish(e)
        guard_pub.publish(ExecutionGuardStatus(stamp=at,joint_stamp=at,base_stamp=at,context_id=guard['context'],active=guard['active'],healthy=guard['active'],reason='EXECUTION_WITHIN_BOUNDS'))
    entities.append(fixture.create_timer(.04,publish,callback_group=group))
    entities.append(client.create_subscription(String,'/transport/hold_executor/status',lambda m:statuses.append(json.loads(m.data)),10))
    entities.append(client.create_subscription(ArmHoldStatus,'/navigation/arm_hold',lambda m:holds.append((time.monotonic(),m.hold_confirmed)),10))
    action=ActionClient(client,PlanToHold,'/transport/plan_to_hold');renew=client.create_client(RenewHold,'/transport/hold_executor/renew')
    args.output.parent.mkdir(exist_ok=True,parents=True);log=args.output.with_suffix('.log').open('w');process=None;renew_seq=0;renew_last=0.;renew_future=None;outcome={}
    def wait(predicate,seconds,label):
        nonlocal renew_seq,renew_last,renew_future
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if predicate():return
            if process is not None and process.poll() is not None:raise AssertionError('executor exited '+str(process.returncode))
            if renew_future is not None and renew_future.done():renew_future=None
            if statuses and statuses[-1]['phase'] in ('1','2','3') and renew_future is None and time.monotonic()-renew_last>.2:
                s=statuses[-1];renew_seq+=1;renew_future=renew.call_async(RenewHold.Request(lease_id=s['lease_id'],resource_epoch=s['epoch'],sequence=renew_seq));renew_last=time.monotonic()
            stop.wait(.01)
        raise AssertionError(label)
    try:
        process=subprocess.Popen([str(args.executable.resolve()),'--ros-args','-p','use_sim_time:=true','-p','simulation_commissioning:=true'],stdout=log,stderr=subprocess.STDOUT)
        wait(lambda:action.server_is_ready() and len(statuses)>8,8,'startup')
        goal=PlanToHold.Goal(task_id='test_'+uuid.uuid4().hex,request_id='request',context_id='context',operation='PICK',object_id='box',grasp_width_m=.04)
        goal.pre_target.header.frame_id=goal.target.header.frame_id='astribot_torso_base';goal.pre_target.pose.orientation.w=goal.target.pose.orientation.w=1.;goal.exit_targets=[goal.pre_target]
        sent=action.send_goal_async(goal);wait(sent.done,4,'accept');handle=sent.result();assert handle.accepted
        terminal=handle.get_result_async()
        if args.mode=='normal':
            wait(lambda:any(h for _,h in holds),8,'no first-stage HOLD')
            assert len([e for e in events if e['event']=='send' and e['controller']!='mtc'])==6
            assert not terminal.done();assert not guard['active'];assert all(positions[j]==.1 for j in groups['arm_left_controller'])
            navigation=True;until=time.monotonic()+.3;wait(lambda:time.monotonic()>=until,1,'hold handoff');assert statuses[-1]['hold_confirmed']
            cancel_future=handle.cancel_goal_async();wait(cancel_future.done,3,'cancel');assert cancel_future.result().goals_canceling
        elif args.mode=='pending_cancel':
            wait(lambda:any(e['event']=='send' and e['controller']=='mtc' for e in events),3,'planner not requested')
            cancel_future=handle.cancel_goal_async();wait(cancel_future.done,3,'cancel');assert cancel_future.result().goals_canceling
            assert not terminal.done(), 'cancel ACK is not resource release'
        wait(terminal.done,15,'terminal');result_value=terminal.result();outcome=dict(status=result_value.status,reason=result_value.result.reason,resources_released=result_value.result.resources_released)
        if args.mode=='unknown_result':assert not outcome['resources_released'] and 'RESOURCE_RECOVERY_REQUIRED' in outcome['reason']
        else:assert outcome['resources_released']
        if args.mode!='normal':assert not any(h for _,h in holds)
        if args.mode in ('wrong_context','start_changed','scene_changed','pending_cancel'):
            assert not any(e['event']=='send' and e['controller']!='mtc' for e in events)
        if args.mode=='pending_cancel':
            assert any(e['event']=='cancel_ack' and e['controller']=='mtc' for e in events)
        if args.mode=='no_endpoint':assert 'ENDPOINT_NOT_REACHED' in outcome['reason']
        event('assertions_passed')
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait();outcome['forced_cleanup']=True
        log.close();stop.set();executor.shutdown(timeout_sec=3);thread.join(timeout=3);fixture.destroy_node();client.destroy_node();rclpy.shutdown()
        args.output.write_text(json.dumps(dict(mode=args.mode,domain=args.domain,events=events,statuses=statuses,holds=holds,outcome=outcome,journal=str(journal),evidence='isolated synthetic ROS protocol, not motion acceptance'),indent=2))

if __name__=='__main__':main()
