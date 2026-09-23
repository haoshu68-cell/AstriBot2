"""Real C++ adapters with deterministic ROS fixtures, never physical inventory proof.

Explicit opt-in, domain 116 only. No simulator, controller, action or motion client.
Run with PAYLOAD_STATE_CPP and GEOMETRY_STATE_CPP from the candidate build.
"""
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('PAYLOAD_ROS_PROTOCOL') != '1',
                               reason='explicit isolated ROS protocol opt-in required')

URDF = '''<robot name="fixture"><link name="base"><collision><geometry><box size="0.62 0.62 0.2"/></geometry></collision></link>
<joint name="arm" type="revolute"><parent link="base"/><child link="arm_link"/><origin xyz="0 0.3 0.5"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
<link name="arm_link"><collision><origin xyz="0.3 0 0"/><geometry><box size="0.6 0.06 0.06"/></geometry></collision></link></robot>'''


def test_ledger_geometry_ros_state_version_loss_and_restart(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '116', 'never run fixtures in the navigation domain'
    executables = {k: Path(os.environ[k]).resolve() for k in ('PAYLOAD_STATE_CPP', 'GEOMETRY_STATE_CPP')}
    assert all(p.is_file() for p in executables.values())
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from rcl_interfaces.srv import GetParameters
    from moveit_msgs.srv import GetPlanningScene
    from moveit_msgs.msg import PlanningScene, AttachedCollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    from astribot_payload_msgs.msg import AttachmentObservation, AttachmentState
    from astribot_navigation_msgs.msg import RobotGeometryState
    from rosidl_runtime_py.convert import message_to_ordereddict

    rclpy.init()
    node = rclpy.create_node('payload_protocol_fixture', parameter_overrides=[Parameter('use_sim_time', value=True)])
    clock = node.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    source = node.create_publisher(AttachmentObservation, '/payload/attachment_observation', 32)
    joints = node.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
    filter_ack = node.create_publisher(String, '/navigation/attachment_filter_applied', 10)
    states, geometry, trace, processes, logs, phases = [], [], [], [], [], []
    ros_ns, sequence, revision, source_on, scene_diff = 10_000_000_000, 0, 1, False, False
    objects = []
    coverage = dict(input_cloud_topic='/map_scan', base_frame='base', enable_outlier_filter=False)
    for i, name in enumerate(('low_obstacle', 'main_nav', 'torso_high', 'overhead')):
        for field, value in dict(enabled=True, z_min=-.03 if i == 0 else i*.6, z_max=(i+1)*.6, min_points=1).items():
            coverage[f'slices.{name}.{field}'] = value

    def ns(stamp):
        return stamp.sec*10**9 + stamp.nanosec

    def stamp(value):
        return Time(sec=value//10**9, nanosec=value % 10**9)

    def parameters(request, response):
        response.values = [Parameter(name, value=URDF if name == 'robot_description' else coverage[name]).to_parameter_msg().value
                           for name in request.names]
        return response

    def scene(request, response):
        response.scene.is_diff = scene_diff
        response.scene.robot_state.attached_collision_objects = copy.deepcopy(objects)
        return response

    services = [node.create_service(GetParameters, '/robot_state_publisher/get_parameters', parameters),
                node.create_service(GetParameters, '/pointcloud_slice_scan_node/get_parameters', parameters),
                node.create_service(GetPlanningScene, '/get_planning_scene', scene)]

    def record(kind, message):
        (states if kind == 'ledger' else geometry).append(message)
        trace.append(dict(kind=kind, receive_wall=time.monotonic(), receive_ros_ns=ros_ns,
                          message=message_to_ordereddict(message)))

    subs = [node.create_subscription(AttachmentState, '/payload/attachment_state', lambda m: record('ledger', m), 10),
            node.create_subscription(RobotGeometryState, '/navigation/geometry_state', lambda m: record('geometry', m), 10),
            node.create_subscription(PlanningScene, '/navigation/attached_geometry',
                                     lambda m: filter_ack.publish(String(data=m.name)), 10)]

    def spawn(kind, extra):
        stream = (tmp_path / (kind.lower() + '_' + str(len(processes)) + '.log')).open('w')
        command = [str(executables[kind]), '--ros-args', '-p', 'use_sim_time:=true', *extra]
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
        processes.append(process);logs.append(stream)
        return process

    ledger_args = ['-p', 'session_id:=protocol', '-p', 'source_id:=fixture', '-p',
                   'allowed_attachment_links:=[arm_link]', '-p', 'journal_path:=' + str(tmp_path / 'ledger.jsonl')]
    ledger = spawn('PAYLOAD_STATE_CPP', ledger_args)
    spawn('GEOMETRY_STATE_CPP', ['-p', 'base_frame:=base', '-p', 'payload_session_id:=protocol',
                               '-p', 'payload_source_id:=fixture', '-p', 'allowed_attachment_links:=[arm_link]'])

    def spin(duration, advance_clock=True):
        nonlocal ros_ns, sequence
        end, previous, last_publish = time.monotonic()+duration, time.monotonic(), -1.
        while time.monotonic() < end:
            wall = time.monotonic()
            if advance_clock:
                ros_ns += int((wall-previous)*1e9)
            previous = wall
            clock.publish(Clock(clock=stamp(ros_ns)))
            if wall-last_publish >= .04:
                j = JointState();j.header.stamp = stamp(ros_ns);j.name = ['arm'];j.position = [.2];joints.publish(j)
                if source_on:
                    sequence += 1
                    o = AttachmentObservation(environment='simulation', session_id='protocol', source_id='fixture',
                        source_epoch='fixture_boot', clock_epoch=0, sequence=sequence, revision=revision,
                        # Real sources have capture/transport delay; recovery must not chase receipt time.
                        observed_at=stamp(ros_ns-50_000_000), valid_until=stamp(ros_ns+250_000_000), full_inventory=True,
                        status=AttachmentObservation.ATTACHED if objects else AttachmentObservation.EMPTY,
                        transaction_id='fixture_'+str(revision), objects=copy.deepcopy(objects))
                    source.publish(o)
                last_publish = wall
            rclpy.spin_once(node, timeout_sec=.005)
            for p in processes:
                assert p.poll() is None, f'owned adapter exited: {p.args}'

    def phase(name):
        phases.append(dict(name=name, ros_ns=ros_ns, wall=time.monotonic(),
                           ledger_reason=states[-1].reason if states else None,
                           geometry_reason=geometry[-1].reason if geometry else None))
        states.clear();geometry.clear()

    def wait_ready(timeout=4.):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            spin(.1)
            if states and states[-1].confirmed and geometry and geometry[-1].complete:
                assert geometry[-1].attachment_revision == states[-1].attachment_revision
                assert ns(geometry[-1].valid_until) <= ns(states[-1].observation.valid_until)
                return
        raise AssertionError([(p['kind'], p['message'].get('reason')) for p in trace[-12:]])

    def stop(process):
        process.send_signal(signal.SIGINT)
        try:
            code = process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill();process.wait(timeout=5)
            raise AssertionError('owned adapter failed graceful exit')
        processes.remove(process)
        assert code == 0, f'non-graceful shutdown: {code}'

    try:
        spin(1.2)
        assert states and not any(s.confirmed for s in states)
        assert geometry and not any(g.complete for g in geometry)
        phase('missing_inventory_rejected')
        source_on = True;wait_ready();empty = states[-1].attachment_revision
        assert not geometry[-1].attachment_ids
        phase('explicit_empty_confirmed')

        a = AttachedCollisionObject(link_name='arm_link');a.object.id = 'payload';a.object.header.frame_id = 'arm_link'
        a.object.pose.orientation.w = 1.;a.object.pose.position.x = .3
        a.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.2, .1, .1])]
        pose = Pose();pose.orientation.w = 1.;a.object.primitive_poses = [pose]
        objects = [a];revision += 1;wait_ready();loaded = states[-1].attachment_revision
        assert loaded != empty and geometry[-1].attachment_ids == ['payload']
        phase('loaded_version_confirmed')
        objects = [];revision += 1;wait_ready();new_empty = states[-1].attachment_revision
        assert new_empty not in (empty, loaded) and not geometry[-1].attachment_ids
        phase('empty_loaded_empty_no_version_reuse')

        scene_diff = True;spin(.7)
        assert not states[-1].confirmed and not geometry[-1].complete
        phase('incomplete_scene_revoked')
        scene_diff = False;wait_ready();phase('fresh_scene_recovered')
        source_on = False;spin(.8)
        assert not states[-1].confirmed and not geometry[-1].complete
        phase('source_loss_revoked')
        source_on = True;wait_ready();phase('fresh_source_recovered')
        source_on = False;frozen = ros_ns;spin(.8, advance_clock=False)
        assert ros_ns == frozen and not states[-1].confirmed and not geometry[-1].complete
        phase('frozen_ros_clock_wall_expiry')
        source_on = True;wait_ready();old_epoch = states[-1].ledger_epoch
        source_on = False;stop(ledger);spin(.4)
        ledger = spawn('PAYLOAD_STATE_CPP', ledger_args);spin(.8)
        assert states and states[-1].ledger_epoch != old_epoch and not states[-1].confirmed
        assert geometry and not geometry[-1].complete
        phase('ledger_restart_unknown_revoked')
        source_on = True;wait_ready()
        assert states[-1].attachment_revision != new_empty
        phase('new_epoch_reconciled')
        for p in list(processes):
            stop(p)
    finally:
        for p in list(processes):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:p.kill();p.wait(timeout=5)
        for stream in logs:stream.close()
        (tmp_path/'protocol_trace.json').write_text(json.dumps(trace, indent=2))
        (tmp_path/'phases.json').write_text(json.dumps(phases, indent=2))
        node.destroy_node();rclpy.shutdown()
