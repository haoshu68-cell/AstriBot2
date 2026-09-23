#!/usr/bin/env python3
"""Isolated ROS protocol fixture; never an actual robot/Gazebo acceptance claim.

Exercises delayed SendGoal acceptance, rejected goals, and UNKNOWN GetResult.
Requires a fresh dedicated ROS domain with no pre-existing resource journal.
"""
import argparse
import concurrent.futures
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
from rclpy.qos import qos_profile_action_status_default
from action_msgs.msg import GoalInfo, GoalStatusArray
from control_msgs.action import FollowJointTrajectory as FJT
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from astribot_navigation_msgs.msg import RobotGeometryState, ArmHoldStatus
from astribot_s1_transport_native.action import HoldResources
from astribot_s1_transport_native.srv import RenewHold


def names():
    result = {}
    for side in ('left', 'right'):
        result['arm_'+side+'_controller'] = ['astribot_arm_'+side+'_joint_'+str(i) for i in range(1, 8)]
        result['gripper_'+side+'_controller'] = ['astribot_gripper_'+side+'_joint_L1']
    result['head_controller'] = ['astribot_head_joint_'+str(i) for i in range(1, 3)]
    result['torso_controller'] = ['astribot_torso_joint_'+str(i) for i in range(1, 5)]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('pending_cancel', 'unknown', 'reject', 'aborted'), required=True)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert 0 <= args.domain <= 232 and os.environ.get('ROS_DOMAIN_ID') == str(args.domain)
    state = Path.home()/'.local/state/astribot/transport'/('domain_'+str(args.domain)+'.jsonl')
    lock = Path('/tmp')/('astribot_transport_domain_'+str(args.domain)+'.lock')
    assert not state.exists() and not lock.exists(), 'fresh dedicated domain required; never erase unresolved state'
    rclpy.init()
    fixture = Node('isolated_hold_controller_fixture')
    client = Node('isolated_hold_test_client')
    group = ReentrantCallbackGroup()
    executor = MultiThreadedExecutor(num_threads=20)
    executor.add_node(fixture); executor.add_node(client)
    stop = threading.Event()
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
    origin = time.monotonic()
    events, statuses, holds = [], [], []
    data_lock = threading.Lock()
    canceled = set()
    started = {}
    entities = []

    def stamp():
        from builtin_interfaces.msg import Time
        value = int((100.+time.monotonic()-origin)*1e9)
        return Time(sec=value//10**9, nanosec=value % 10**9)

    def event(kind, name, goal):
        with data_lock:
            events.append(dict(wall=time.monotonic(), event=kind, controller=name, uuid=[int(v) for v in goal]))

    def send(name):
        def callback(request, response):
            goal = tuple(request.goal_id.uuid)
            event('send_received', name, goal)
            started[goal] = time.monotonic()
            if args.mode == 'pending_cancel':
                stop.wait(1.2)
            response.accepted = not (args.mode == 'reject' and name == 'arm_left_controller')
            response.stamp = stamp()
            event('accepted' if response.accepted else 'rejected', name, goal)
            return response
        return callback

    def cancel(name):
        def callback(request, response):
            goal = tuple(request.goal_info.goal_id.uuid)
            with data_lock:
                canceled.add(goal)
            event('cancel_ack', name, goal)
            response.return_code = 0
            response.goals_canceling = [request.goal_info]
            return response
        return callback

    def result(name):
        def callback(request, response):
            goal = tuple(request.goal_id.uuid)
            if args.mode == 'pending_cancel':
                deadline = time.monotonic()+5
                while goal not in canceled and time.monotonic() < deadline and not stop.is_set():
                    stop.wait(.01)
                response.status = 5 if goal in canceled else 4
            elif args.mode == 'unknown' and name == 'arm_left_controller':
                response.status = 0
            elif args.mode == 'aborted' and name == 'arm_left_controller':
                response.status = 6
            else:
                response.status = 4
            response.result.error_code = 0
            event('result_'+str(response.status), name, goal)
            return response
        return callback

    for name in names():
        endpoint = '/'+name+'/follow_joint_trajectory'
        entities += [fixture.create_service(FJT.Impl.SendGoalService, endpoint+'/_action/send_goal', send(name), callback_group=group),
            fixture.create_service(FJT.Impl.GetResultService, endpoint+'/_action/get_result', result(name), callback_group=group),
            fixture.create_service(FJT.Impl.CancelGoalService, endpoint+'/_action/cancel_goal', cancel(name), callback_group=group),
            fixture.create_publisher(GoalStatusArray, endpoint+'/_action/status', qos_profile_action_status_default),
            fixture.create_publisher(FJT.Impl.FeedbackMessage, endpoint+'/_action/feedback', 10)]

    def controllers(request, response):
        response.controller = [ControllerState(name=name, state='active',
            type='joint_trajectory_controller/JointTrajectoryController',
            claimed_interfaces=[n+'/position' for n in joints]) for name,joints in names().items()]
        return response
    entities.append(fixture.create_service(ListControllers, '/controller_manager/list_controllers', controllers, callback_group=group))
    clock = fixture.create_publisher(Clock, '/clock', 10)
    geometry = fixture.create_publisher(RobotGeometryState, '/navigation/geometry_state', 10)
    sequence = 0
    def publish():
        nonlocal sequence
        at = stamp(); clock.publish(Clock(clock=at)); sequence += 1
        message = RobotGeometryState(); message.header.stamp=at; message.header.frame_id='base_link'
        message.source_id='isolated_fixture'; message.sequence=sequence; message.model_revision='fixture_model'; message.attachment_revision='fixture_empty'
        message.complete=message.attachment_state_confirmed=True
        message.valid_until.sec=at.sec; message.valid_until.nanosec=at.nanosec+300000000
        if message.valid_until.nanosec>=10**9:message.valid_until.sec+=1;message.valid_until.nanosec-=10**9
        message.joints.name=sorted(n for group_names in names().values() for n in group_names)
        message.joints.position=[0.]*22; message.joint_position_error_bounds=[.004]*22; message.joint_source_stamps=[at]*22
        geometry.publish(message)
    entities.append(fixture.create_timer(.05, publish, callback_group=group))
    def receive_status(message):
        value=json.loads(message.data);value['_received_wall']=time.monotonic();statuses.append(value)
    entities.append(client.create_subscription(String, '/transport/hold_executor/status', receive_status, 10))
    entities.append(client.create_subscription(ArmHoldStatus, '/navigation/arm_hold', lambda m:holds.append(bool(m.hold_confirmed)), 10))
    action=ActionClient(client, HoldResources, '/transport/hold_resources')
    renew=client.create_client(RenewHold, '/transport/hold_executor/renew')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log=args.output.with_suffix('.log').open('w')
    process=subprocess.Popen([str(args.executable.resolve()), '--ros-args', '-p', 'use_sim_time:=true', '-p', 'simulation_commissioning:=true'], stdout=log, stderr=subprocess.STDOUT)
    sequence_renew=0; last_renew=0.; pending=None
    def wait(condition, seconds, description):
        nonlocal sequence_renew,last_renew,pending
        deadline=time.monotonic()+seconds
        while not condition():
            if process.poll() is not None:raise RuntimeError('executor exited: '+args.output.with_suffix('.log').read_text())
            if time.monotonic()>deadline:raise RuntimeError(description+': '+str(statuses[-1:]));
            if pending is not None and pending.done():pending=None
            if statuses and statuses[-1].get('phase') in ('1','2','3') and pending is None and time.monotonic()-last_renew>.25:
                s=statuses[-1];sequence_renew+=1;pending=renew.call_async(RenewHold.Request(lease_id=s['lease_id'], resource_epoch=s['epoch'],sequence=sequence_renew));last_renew=time.monotonic()
            time.sleep(.01)
    outcome={'pass':False,'mode':args.mode,'domain':args.domain,'evidence':'isolated_ros_protocol_fixture'}
    try:
        wait(lambda:action.server_is_ready() and statuses,8,'entry timeout')
        time.sleep(.5)
        sent=action.send_goal_async(HoldResources.Goal(task_id='fixture',request_id=uuid.uuid4().hex));wait(sent.done,3,'admission timeout')
        goal=sent.result()
        if not goal.accepted:
            time.sleep(.5)
            raise AssertionError(statuses[-1:])
        result_future=goal.get_result_async()
        if args.mode=='pending_cancel':
            canceled_future=goal.cancel_goal_async();wait(canceled_future.done,3,'cancel acknowledgement timeout');assert canceled_future.result().goals_canceling
        wait(result_future.done,16,'terminal timeout');value=result_future.result()
        outcome.update(status=value.status,resources_released=value.result.resources_released,reason=value.result.reason)
        assert not any(holds)
        if args.mode=='unknown':
            assert not value.result.resources_released and value.status==6
            assert 'CHILD_RESULT_UNKNOWN' in value.result.reason
            assert json.loads(state.read_text().splitlines()[-1])['phase'] == 5
            before=len([e for e in events if e['event']=='send_received'])
            prior_epoch=statuses[-1]['epoch']
            process.send_signal(signal.SIGINT);process.wait(timeout=5)
            process=subprocess.Popen([str(args.executable.resolve()), '--ros-args', '-p', 'use_sim_time:=true', '-p', 'simulation_commissioning:=true'], stdout=log, stderr=subprocess.STDOUT)
            wait(lambda:statuses and statuses[-1]['epoch']!=prior_epoch and statuses[-1]['phase']=='5',8,'restart quarantine missing')
            time.sleep(.6)
            retry=action.send_goal_async(HoldResources.Goal(task_id='fixture',request_id=uuid.uuid4().hex));wait(retry.done,3,'restart rejection timeout')
            assert not retry.result().accepted
            time.sleep(.2)
            assert len([e for e in events if e['event']=='send_received'])==before
            outcome['restart_quarantined_without_child_replay']=True
        else:
            assert value.result.resources_released
            assert value.status==(5 if args.mode=='pending_cancel' else 6)
        if args.mode=='pending_cancel':
            assert len([e for e in events if e['event']=='cancel_ack'])==6
            assert len([e for e in events if e['event']=='result_5'])==6
            wait(lambda:any(s['phase']=='0' and s['lease_id']==value.result.lease_id for s in statuses),2,'handoff not observed')
            release_at=min(s['_received_wall'] for s in statuses if s['phase']=='0' and s['lease_id']==value.result.lease_id)
            terminal_at=max(e['wall'] for e in events if e['event']=='result_5')
            outcome['terminal_to_release_s']=release_at-terminal_at
            assert release_at-terminal_at>=.5
        outcome['pass']=True
    except Exception as error:
        outcome['error']=repr(error)
    finally:
        # Only this Popen child; leave unresolved durable state for inspection.
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            outcome['pass']=False;outcome['cleanup_error']='executor did not stop on SIGINT'
            process.terminate()
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
        stop.set();executor.shutdown(timeout_sec=2);thread.join(timeout=2)
        outcome.update(events=events,statuses=statuses,holds=holds,process_exit=process.returncode)
        if state.exists():args.output.with_suffix('.journal.jsonl').write_text(state.read_text())
        args.output.write_text(json.dumps(outcome,indent=2));log.close()
        print(json.dumps({k:v for k,v in outcome.items() if k not in ('events','statuses','holds')},indent=2))
        fixture.destroy_node();client.destroy_node();rclpy.shutdown()
    return 0 if outcome['pass'] else 1


if __name__=='__main__':raise SystemExit(main())
