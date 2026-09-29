#!/usr/bin/env python3
"""Stationary owned simulation: real task hold -> six real consumers -> revoke/recover.

Uses no fabricated positive hold or ACK. No chassis command is published.
"""
import argparse
import os
import subprocess
import signal
import sys
import json
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Polygon, Point32, Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from astribot_payload_msgs.msg import AttachmentState
from astribot_navigation_msgs.msg import ArmHoldStatus, RobotGeometryState, NavigationEnvelopeV2, EnvelopeApplyStatus, RobotEnvelope
from astribot_navigation_msgs.srv import SetFixedEnvelope
from astribot_s1_transport_native.action import HoldResources
from astribot_s1_transport_native.srv import RenewHold

CONSUMERS = {'global_costmap', 'local_costmap', 'planner', 'controller', 'policy'}


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--owner', type=Path, required=True, help='owned simulation supervisor identity')
    parser.add_argument('--profile', type=Path, default=Path('ws_robot/src/astribot_s1_navigation_policy/config/simulation.json'))
    args = parser.parse_args()
    owner = json.loads(args.owner.read_text())
    process = Path('/proc') / str(owner['pid'])
    assert Path('/proc/sys/kernel/random/boot_id').read_text().strip() == owner['boot_id']
    assert process.joinpath('stat').read_text().rsplit(') ', 1)[1].split()[19] == str(owner['start_ticks'])
    assert str(process.joinpath('exe').resolve()) == owner['exe']
    argv = process.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
    assert argv == owner['cmdline']
    assert argv[argv.index('--ros-domain-id') + 1] == os.environ['ROS_DOMAIN_ID'] != '25'
    assert os.environ['IGN_PARTITION'] == 'astribot_' + argv[argv.index('--instance') + 1]
    profile = json.loads(args.profile.read_text())
    if profile['environment'] != 'simulation':
        raise RuntimeError('simulation profile required')
    rclpy.init()
    node = Node('fixed_hold_acceptance_' + uuid.uuid4().hex[:8], parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    latest = {}
    geom = None
    geom_wall = 0.
    hold = None
    envelope = None
    envelopes = []
    acks = []
    holds = []
    commands = []
    outcomes = []
    renew_enabled = True
    renew_sequence = 0
    renew_last = 0.
    renew_pending = None
    renew_lease = None
    goal = None
    result = None
    phantom_id = None
    paused = False
    suspended_consumer = None
    ledger = None
    all_data = {'pass': False, 'evidence_layer': 'owned_navigation_simulation_stationary', 'started_wall': time.time(), 'scenarios': outcomes}
    action = ActionClient(node, HoldResources, '/transport/hold_resources')
    renew = node.create_client(RenewHold, '/transport/hold_executor/renew')
    fixed = node.create_client(SetFixedEnvelope, '/navigation/set_fixed_envelope')
    footprint = node.create_publisher(Polygon, '/local_costmap/footprint', 1)
    apply_scene = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    def receive_ledger(message):
        nonlocal ledger
        ledger = message
    node.create_subscription(AttachmentState, '/payload/attachment_state', receive_ledger, 10)

    def geometry(message):
        nonlocal geom, geom_wall
        geom = message
        geom_wall = time.monotonic()

    def receive_hold(message):
        nonlocal hold
        hold = message
        holds.append({'wall': time.monotonic(), 'ros_ns': node.get_clock().now().nanoseconds,
                      'id': message.hold_id, 'confirmed': message.hold_confirmed,
                      'owner': message.owner_id, 'attachment': message.attachment_revision})

    def receive_envelope(message):
        nonlocal envelope
        envelope = message
        envelopes.append({'wall': time.monotonic(), 'ros_ns': node.get_clock().now().nanoseconds,
                          'stamp_ns': ns(message.header.stamp), 'until_ns': ns(message.valid_until),
                          'epoch': message.epoch, 'session': message.coordinator_session_id,
                          'hash': message.installed_geometry_hash, 'hold_id': message.hold_id, 'request_id': message.request_id,
                          'reference_sequence': message.reference_state_sequence,
                          'source_sequence': message.source_state_sequence,
                          'model_revision': message.model_revision, 'attachment_revision': message.attachment_revision,
                          'allowed': message.navigation_allowed, 'reason': message.reason})

    def receive_ack(message):
        acks.append({'wall': time.monotonic(), 'ros_ns': node.get_clock().now().nanoseconds,
                     'stamp_ns': ns(message.header.stamp), 'consumer': message.consumer_id,
                     'session': message.coordinator_session_id, 'epoch': message.envelope_epoch,
                     'hash': message.installed_geometry_hash, 'applied': message.applied, 'reason': message.reason})

    node.create_subscription(RobotGeometryState, '/navigation/geometry_state', geometry, 10)
    node.create_subscription(ArmHoldStatus, '/navigation/arm_hold', receive_hold, 10)
    node.create_subscription(NavigationEnvelopeV2, '/navigation/envelope_v2', receive_envelope, 30)
    node.create_subscription(EnvelopeApplyStatus, '/navigation/envelope_applied', receive_ack, 50)
    node.create_subscription(String, '/transport/hold_executor/status', lambda m: latest.update(json.loads(m.data)), 10)
    node.create_subscription(Twist, '/cmd_vel', lambda m: commands.append({'wall': time.monotonic(), 'max': max(abs(m.linear.x), abs(m.linear.y), abs(m.angular.z))}), qos_profile_sensor_data)

    def spin(predicate, timeout, reason):
        nonlocal renew_sequence, renew_last, renew_pending, renew_lease
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError(reason + ': ' + json.dumps({'hold': latest, 'envelope': envelopes[-1] if envelopes else None, 'recent_acks': acks[-8:]}))
            rclpy.spin_once(node, timeout_sec=.01)
            if renew_pending is not None and renew_pending.done():
                answer = renew_pending.result()
                if renew_enabled and latest.get('phase') in ('1', '2', '3') and not answer.accepted:
                    raise RuntimeError('renew rejected: ' + answer.reason)
                renew_pending = None
            if renew_enabled and latest.get('phase') in ('1', '2', '3') and renew_pending is None and time.monotonic() - renew_last > .2:
                if latest['lease_id'] != renew_lease:
                    renew_sequence = 0
                    renew_lease = latest['lease_id']
                renew_sequence += 1
                renew_pending = renew.call_async(RenewHold.Request(lease_id=renew_lease, resource_epoch=latest['epoch'], sequence=renew_sequence))
                renew_last = time.monotonic()

    def consumer_pause(value):
        nonlocal suspended_consumer
        if value:
            sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
            from sim_stack_supervisor import owned_descendants, process_identity
            candidates = []
            for pid, started in owned_descendants([owner['pid']]).items():
                try:
                    executable = Path(f'/proc/{pid}/exe').resolve()
                    if executable.name == 'planner_server':
                        environment = dict(x.split('=', 1) for x in Path(f'/proc/{pid}/environ').read_bytes().decode().split('\0') if '=' in x)
                        assert environment['ROS_DOMAIN_ID'] == os.environ['ROS_DOMAIN_ID']
                        candidates.append((pid, started, str(executable)))
                except (FileNotFoundError, ProcessLookupError):
                    continue
            assert len(candidates) == 1, candidates
            pid, started, executable = candidates[0]
            fd = os.pidfd_open(pid)
            try:
                assert process_identity(pid)[1] == started
                assert str(Path(f'/proc/{pid}/exe').resolve()) == executable
                suspended_consumer = {'pid': pid, 'start_ticks': started, 'exe': executable, 'fd': fd}
                signal.pidfd_send_signal(fd, signal.SIGSTOP)
            except Exception:
                os.close(fd);suspended_consumer = None
                raise
        else:
            assert suspended_consumer is not None
            # Retained pidfd refers to exactly the process we suspended.
            signal.pidfd_send_signal(suspended_consumer['fd'], signal.SIGCONT)
            os.close(suspended_consumer['fd'])
            suspended_consumer = None

    def world_pause(value):
        nonlocal paused
        # Only the verified supervisor's partition; no motion goal is sent.
        command = ['ign', 'service', '-s', '/world/default/control', '--reqtype', 'ignition.msgs.WorldControl', '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', 'pause: ' + ('true' if value else 'false')]
        if value:
            paused = True  # uncertain service outcome must still trigger resume cleanup
        output = subprocess.run(command, capture_output=True, text=True, timeout=4)
        assert output.returncode == 0 and 'data: true' in output.stdout, output.stderr
        paused = value

    def scene_mismatch(remove=False):
        nonlocal phantom_id
        if not remove:
            assert phantom_id is None
            phantom_id = 'a3_mismatch_' + uuid.uuid4().hex
        assert phantom_id
        item = AttachedCollisionObject(link_name='astribot_arm_left_tcp_link')
        item.object.id = phantom_id
        item.object.header.frame_id = item.link_name
        item.object.pose.orientation.w = 1.
        item.object.operation = item.object.REMOVE if remove else item.object.ADD
        if not remove:
            item.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.02, .02, .02])]
            pose = Pose();pose.orientation.w = 1.;item.object.primitive_poses = [pose]
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True;request.scene.robot_state.is_diff = True
        request.scene.robot_state.attached_collision_objects = [item]
        spin(apply_scene.service_is_ready, 3, 'PlanningScene unavailable')
        future = apply_scene.call_async(request)
        spin(future.done, 3, 'PlanningScene mutation timeout')
        assert future.result().success
        if remove:
            phantom_id = None

    def duration(seconds):
        end = time.monotonic() + seconds
        spin(lambda: time.monotonic() >= end, seconds + 1, 'observation window')

    def valid_geometry():
        return geom is not None and geom.complete and geom.attachment_state_confirmed and time.monotonic()-geom_wall < .25 and ns(geom.header.stamp) <= node.get_clock().now().nanoseconds < ns(geom.valid_until)

    def matching_positive():
        if envelope is None:
            return set()
        now = node.get_clock().now().nanoseconds
        latest_ack = {}
        for sample in acks:
            if (sample['session'], sample['epoch'], sample['hash']) == (envelope.coordinator_session_id, envelope.epoch, envelope.installed_geometry_hash) and sample['stamp_ns'] <= now:
                latest_ack[sample['consumer']] = sample
        return {name for name, sample in latest_ack.items() if name in CONSUMERS and sample['applied'] and 0 <= now-sample['stamp_ns'] < 500_000_000}

    try:
        spin(lambda: action.server_is_ready() and renew.service_is_ready() and fixed.service_is_ready() and latest.get('phase') == '0' and valid_geometry() and len(commands) >= 10, 20, 'entry unavailable')
        duration(.6)
        assert valid_geometry() and max(c['max'] for c in commands) <= 1e-8
        assert not geom.attachment_ids, 'this initial matrix requires confirmed versioned empty payload'
        all_data['ack_publishers'] = [{'node': i.node_name, 'namespace': i.node_namespace} for i in node.get_publishers_info_by_topic('/navigation/envelope_applied')]
        previous_epoch = None
        for mode in ('cancel', 'expiry', 'cancel_recovery', 'footprint_mismatch', 'consumer_pause', 'scene_mismatch', 'scene_recovery', 'world_pause', 'pause_recovery'):
            renew_enabled = True
            requested = action.send_goal_async(HoldResources.Goal(task_id='A3_acceptance', request_id='a3_' + uuid.uuid4().hex))
            spin(requested.done, 5, 'hold admission')
            goal = requested.result()
            assert goal.accepted, 'hold request rejected'
            result = goal.get_result_async()
            spin(lambda: (hold is not None and hold.hold_confirmed and hold.hold_id == latest.get('hold_id')) or result.done(), 18, 'formal hold confirmation')
            assert not result.done(), str(result.result().result) if result.done() else ''
            spin(valid_geometry, 3, 'fresh geometry for request')
            limits = RobotEnvelope(frame_id=profile['base_frame'], posture_id='a3_current_pose', lease_s=.3)
            for name in ('half_length_m', 'half_width_m', 'height_m', 'payload_mass_kg', 'max_speed_m_s', 'max_angular_speed_rad_s', 'max_acceleration_m_s2', 'brake_deceleration_m_s2'):
                setattr(limits, name, float(profile[name]))
            request = SetFixedEnvelope.Request(request_id='a3_fixed_' + uuid.uuid4().hex, hold_id=hold.hold_id, geometry_sequence=geom.sequence, limits=limits)
            future = fixed.call_async(request)
            spin(future.done, 3, 'fixed request timeout')
            response = future.result()
            assert response.accepted, response.reason
            phase = {'mode': mode, 'request_id': request.request_id, 'hold_id': request.hold_id, 'epoch': response.epoch, 'initial_reason': response.reason}
            outcomes.append(phase)
            if previous_epoch is not None:
                assert response.epoch > previous_epoch
            previous_epoch = response.epoch
            spin(lambda: envelope is not None and envelope.epoch == response.epoch and envelope.navigation_allowed and matching_positive() == CONSUMERS, 15, 'five actual geometry consumers did not confirm')
            begin = len(envelopes)
            duration(2.)
            window = envelopes[begin:]
            assert len(window) >= 10 and all(e['allowed'] and e['epoch'] == response.epoch and e['hold_id'] == request.hold_id and e['request_id'] == request.request_id for e in window), 'unstable or mismatched positive envelope'
            assert latest.get('hold_confirmed'), 'formal hold lost during confirmation'
            phase.update(positive_samples=len(window), positive_publication_hz=len(window)/(window[-1]['wall']-window[0]['wall']), consumers=sorted(matching_positive()), session=envelope.coordinator_session_id, geometry_hash=envelope.installed_geometry_hash, attachment_revision=envelope.attachment_revision, model_revision=envelope.model_revision)
            assert phase['positive_publication_hz'] < 120., 'ACK_FEEDBACK_AMPLIFICATION'
            if mode == 'footprint_mismatch':
                mismatch_start = len(acks)
                wrong = Polygon(points=[Point32(x=p.x*1.5, y=p.y*1.5, z=0.) for p in envelope.installed_footprint.points])
                began = time.monotonic()
                while time.monotonic()-began < 1.:
                    footprint.publish(wrong)
                    duration(.04)
                bad = [a for a in acks[mismatch_start:] if a['epoch'] == response.epoch and not a['applied'] and a['reason'].startswith('FOOTPRINT_')]
                assert {a['consumer'] for a in bad} == {'controller', 'local_costmap'}, 'actual footprint mismatch not observed'
                assert any(not e['allowed'] and e['reason'].startswith('WAITING_FOR:') for e in envelopes[begin:]), 'mismatch did not remove permission'
                phase['actual_footprint_negative_ack_samples'] = len(bad)
                spin(lambda: envelope.navigation_allowed and matching_positive() == CONSUMERS, 5, 'actual footprint did not recover')
                phase['footprint_recovered_same_epoch'] = envelope.epoch == response.epoch
            if mode == 'consumer_pause':
                before = time.monotonic()
                consumer_pause(True)
                phase['suspended_consumer'] = {k:v for k,v in suspended_consumer.items() if k != 'fd'}
                spin(lambda: envelope is not None and envelope.epoch == response.epoch and not envelope.navigation_allowed and envelope.reason.startswith('WAITING_FOR:') and 'planner' in envelope.reason, 2, 'missing real consumer did not revoke')
                phase['consumer_timeout_after_suspend_s'] = time.monotonic()-before
                phase['consumer_timeout_reason'] = envelope.reason
                assert latest.get('hold_confirmed') and valid_geometry(), 'source failed instead of ACK timeout'
                consumer_pause(False)
                spin(lambda: envelope.epoch == response.epoch and envelope.navigation_allowed and matching_positive() == CONSUMERS, 5, 'consumer did not recover same epoch')
                phase['consumer_recovered_same_epoch'] = True
            trigger_index = len(envelopes)
            trigger = time.monotonic()
            if mode == 'expiry':
                renew_enabled = False
            elif mode == 'world_pause':
                renew_enabled = False
                phase['clock_before_pause_ns'] = node.get_clock().now().nanoseconds
                world_pause(True)
                duration(.15)
                phase['clock_frozen_start_ns'] = node.get_clock().now().nanoseconds
            elif mode == 'scene_mismatch':
                renew_enabled = False
                scene_mismatch()
                spin(lambda: ledger is not None and not ledger.confirmed and ledger.reason == 'PHYSICAL_SCENE_MISMATCH', 3, 'independent scene mismatch not observed')
                phase['ledger_rejection'] = ledger.reason
            else:
                renew_enabled = False
                canceled = goal.cancel_goal_async()
                spin(canceled.done, 3, 'cancel ack')
                assert canceled.result().goals_canceling
            spin(lambda: envelope is not None and not envelope.navigation_allowed and envelope.epoch == response.epoch, 5, 'envelope did not revoke')
            revoked_index = next(i for i in range(trigger_index, len(envelopes)) if envelopes[i]['epoch'] == response.epoch and not envelopes[i]['allowed'])
            phase.update(revocation_reason=envelope.reason, revoke_after_trigger_s=time.monotonic()-trigger)
            if mode == 'world_pause':
                renew_enabled = False
                phase['clock_at_revoke_ns'] = node.get_clock().now().nanoseconds
                assert phase['clock_frozen_start_ns'] == phase['clock_at_revoke_ns'], 'ROS clock continued during pause'
                world_pause(False)
            if mode == 'scene_mismatch':
                scene_mismatch(remove=True)
            spin(result.done, 15, 'resource handoff terminal timeout')
            terminal = result.result()
            assert terminal.result.resources_released
            if mode == 'world_pause':
                assert 'GEOMETRY' in terminal.result.reason, terminal.result.reason
            assert terminal.status == (6 if mode in ('expiry', 'scene_mismatch', 'world_pause') else 5)
            phase.update(terminal_status=terminal.status, terminal_reason=terminal.result.reason, released=terminal.result.resources_released)
            duration(.7)
            spin(lambda: latest.get('phase') == '0' and valid_geometry(), 5, 'ready for next request')
            revoked_window = envelopes[revoked_index:]
            assert revoked_window and all(not e['allowed'] and e['epoch'] == response.epoch and e['hold_id'] == request.hold_id and e['session'] == phase['session'] for e in revoked_window), 'old permission restored or authority changed during revoke/handoff'
            phase['revoked_through_handoff_samples'] = len(revoked_window)
            phase['pass'] = True
            goal = None
        assert commands and max(c['max'] for c in commands) <= 1e-8
        all_data['pass'] = True
    except Exception as error:
        all_data['error'] = str(error)
    finally:
        if suspended_consumer is not None:
            try:
                consumer_pause(False)
            except Exception as error:
                all_data['consumer_cleanup_error'] = str(error)
        if paused:
            try:
                world_pause(False)
            except Exception as error:
                all_data['pause_cleanup_error'] = str(error)
        if phantom_id is not None:
            try:
                scene_mismatch(remove=True)
            except Exception as error:
                all_data['scene_cleanup_error'] = str(error)
        if goal is not None and goal.accepted:
            try:
                renew_enabled = False
                canceled = goal.cancel_goal_async()
                spin(canceled.done, 3, 'cleanup cancel')
                terminal = result or goal.get_result_async()
                spin(terminal.done, 15, 'cleanup release')
                all_data['cleanup_result'] = str(terminal.result().result)
            except Exception as error:
                all_data['cleanup_error'] = str(error)
        all_data.update(envelopes=envelopes, acks=acks, holds=holds, cmd_vel_samples=len(commands), max_cmd_vel=max((c['max'] for c in commands), default=None), finished_wall=time.time())
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(all_data, indent=2) + '\n')
        print(json.dumps({k:v for k,v in all_data.items() if k not in ('envelopes','acks','holds')}, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if all_data['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
