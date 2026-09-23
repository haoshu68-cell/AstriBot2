#!/usr/bin/env python3
"""Owned simulation only: request current-pose holds, renew/cancel, retain evidence.

No geometry, ownership, or successful child-result fixture is published here.
The controller actions and measurements come from the running C++ executor.
"""
import argparse
import json
import time
import uuid
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from astribot_navigation_msgs.msg import ArmHoldStatus, RobotGeometryState
from astribot_s1_transport_native.action import HoldResources
from astribot_s1_transport_native.srv import RenewHold


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=('cancel', 'expiry', 'early_cancel', 'foreign_client'), default='cancel')
    args = parser.parse_args()
    rclpy.init()
    node = Node('task_hold_acceptance_' + uuid.uuid4().hex[:8], parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    observations = []
    latest = {}
    holds = []
    commands = []
    geometry = None
    geometry_received = 0.
    geometry_ready_since = None
    def receive_geometry(message):
        nonlocal geometry,geometry_received,geometry_ready_since
        stamp=time.monotonic()
        identity=lambda value:(value.source_id,value.clock_epoch,value.model_revision,value.attachment_revision)
        if not message.complete or not message.attachment_state_confirmed:
            geometry_ready_since=None
        elif geometry is None or not geometry.complete or identity(message)!=identity(geometry) or stamp-geometry_received>.3:
            geometry_ready_since=stamp
        geometry=message;geometry_received=stamp
    node.create_subscription(RobotGeometryState,'/navigation/geometry_state',receive_geometry,10)
    def geometry_ready():
        if geometry is None or geometry_ready_since is None or not geometry.complete or not geometry.attachment_state_confirmed:
            return False
        ros=node.get_clock().now().nanoseconds
        at=geometry.header.stamp.sec*10**9+geometry.header.stamp.nanosec
        until=geometry.valid_until.sec*10**9+geometry.valid_until.nanosec
        return at<=ros<until and time.monotonic()-geometry_received<.3 and time.monotonic()-geometry_ready_since>=.5
    node.create_subscription(String, '/transport/hold_executor/status',
        lambda message: (latest.update(json.loads(message.data)), observations.append(
            {'wall': time.time(), 'state': json.loads(message.data)})), 10)
    node.create_subscription(ArmHoldStatus, '/navigation/arm_hold',
        lambda message: holds.append({'at': time.monotonic(), 'confirmed': message.hold_confirmed,
            'hold_id': message.hold_id, 'owner': message.owner_id, 'lease': message.lease_s}), 10)
    node.create_subscription(Twist, '/cmd_vel', lambda message: commands.append(
        max(abs(message.linear.x), abs(message.linear.y), abs(message.angular.z))), qos_profile_sensor_data)
    action = ActionClient(node, HoldResources, '/transport/hold_resources')
    renew = node.create_client(RenewHold, '/transport/hold_executor/renew')
    sequence = 0
    renew_enabled = True
    last_renew = 0.
    pending = None
    renewal_results = []
    foreign = None

    def spin(predicate, timeout, description):
        nonlocal sequence, last_renew, pending
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError(description + ': ' + json.dumps(latest))
            rclpy.spin_once(node, timeout_sec=.02)
            if pending is not None and pending.done():
                renewal_results.append(bool(pending.result().accepted))
                pending = None
            if renew_enabled and latest.get('lease_id') and latest.get('phase') in ('1', '2', '3') and pending is None and time.monotonic()-last_renew > .25:
                sequence += 1
                pending = renew.call_async(RenewHold.Request(lease_id=latest['lease_id'],
                    resource_epoch=latest['epoch'], sequence=sequence))
                last_renew = time.monotonic()

    goal = None
    outcome = {'mode': args.mode, 'pass': False, 'evidence': 'owned_navigation_simulation_current_pose_hold'}
    try:
        spin(lambda: action.server_is_ready() and renew.service_is_ready() and latest.get('phase') == '0' and len(commands) >= 5 and geometry_ready(), 15, 'entry, geometry or stationary observation unavailable')
        outcome['admission_geometry']={'source_id':geometry.source_id,'sequence':geometry.sequence,'model_revision':geometry.model_revision,'attachment_revision':geometry.attachment_revision}
        request = HoldResources.Goal(task_id='acceptance', request_id='probe_' + uuid.uuid4().hex)
        sent = action.send_goal_async(request)
        spin(sent.done, 5, 'goal admission timeout')
        goal = sent.result()
        if not goal.accepted:
            spin(lambda: False, 1, 'goal rejected')
        result = goal.get_result_async()
        if args.mode != 'early_cancel':
            spin(lambda: latest.get('hold_confirmed') is True or result.done(), 15, 'hold did not confirm')
            if result.done():
                raise RuntimeError('task ended before hold: '+str(result.result().result))
            hold_id = latest['hold_id']
            deadline = time.monotonic()+2
            spin(lambda: time.monotonic() >= deadline, 3, 'confirmed window')
            assert latest.get('hold_confirmed') and latest['hold_id'] == hold_id
            duplicate = action.send_goal_async(request)
            spin(duplicate.done, 3, 'duplicate admission timeout')
            assert not duplicate.result().accepted
        else:
            hold_id = latest.get('hold_id')
        if args.mode in ('cancel', 'early_cancel'):
            canceled = goal.cancel_goal_async()
            spin(canceled.done, 3, 'cancel acknowledgement timeout')
            assert canceled.result().goals_canceling
        elif args.mode == 'expiry':
            renew_enabled = False
        elif args.mode == 'foreign_client':
            # Discovery only: no unauthorized trajectory is transmitted.
            from control_msgs.action import FollowJointTrajectory
            foreign = node.create_client(FollowJointTrajectory.Impl.SendGoalService,
                '/arm_left_controller/follow_joint_trajectory/_action/send_goal')
        spin(result.done, 15, 'terminal/release timeout')
        value = result.result()
        outcome.update(status=value.status, resources_released=value.result.resources_released,
                       reason=value.result.reason, lease_id=value.result.lease_id, hold_id=hold_id)
        assert value.result.resources_released
        assert value.status == (5 if args.mode in ('cancel', 'early_cancel') else 6)
        if args.mode == 'early_cancel':
            assert not any(sample['confirmed'] for sample in holds)
        if args.mode == 'foreign_client':
            assert 'FOREIGN_CONTROLLER_SERVICE_CLIENT' in value.result.reason
        spin(lambda: latest.get('phase') == '0' and not latest.get('hold_confirmed'), 3, 'release not observed')
        outcome['hold_id'] = latest.get('hold_id', hold_id)
        assert commands and max(commands) <= 1e-8
        if args.mode != 'early_cancel':
            assert any(renewal_results)
        if foreign is not None:
            node.destroy_client(foreign)
            foreign = None
        duplicate = action.send_goal_async(request)
        spin(duplicate.done, 3, 'released duplicate timeout')
        assert not duplicate.result().accepted
        outcome['pass'] = True
    except Exception as error:
        outcome['error'] = str(error)
    finally:
        if goal is not None and goal.accepted and not outcome['pass']:
            try:
                canceled = goal.cancel_goal_async()
                renew_enabled = False
                spin(canceled.done, 3, 'cleanup cancel timeout')
                result = goal.get_result_async()
                spin(result.done, 12, 'cleanup terminal timeout')
                outcome['cleanup_result'] = str(result.result().result)
            except Exception as error:
                outcome['cleanup_error'] = str(error)
        outcome.update(observations=observations, holds=holds, renewal_results=renewal_results,
                       cmd_vel_samples=len(commands), max_cmd_vel=max(commands, default=None))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(outcome, indent=2))
        print(json.dumps({key:value for key,value in outcome.items() if key not in ('observations', 'holds', 'renewal_results')}, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if outcome['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
