#!/usr/bin/env python3
"""One-shot, owned-simulation EMPTY inventory bootstrap and read-only admission check.

This operations helper loads the existing C++ ECM observer. It never publishes
attachment/hold/ACK messages, modifies PlanningScene, or sends motion goals.
Failure leaves any loaded observer in place and supplies no admission proof;
this helper never grants or revokes navigation permission, clears a journal,
or starts a second source. Recheck with --observe-existing.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET


def read_model_parameters(client, request, spin, timeout=10., *, clock=time.monotonic, events=None):
    """Only model GetParameters reads may retry; discovery shares the budget."""
    require(client.srv_name.endswith('/get_parameters'), 'READ_ONLY_PARAMETER_SERVICE_REQUIRED')
    import sys
    tools_directory = str(Path(__file__).resolve().parents[1])
    if tools_directory not in sys.path:
        sys.path.insert(0, tools_directory)
    from run_waypoint_route import read_service
    started = clock()
    deadline = started + timeout
    discovery_deadline = started + min(5., timeout)
    require(timeout > 0, 'SERVICE_UNAVAILABLE:' + client.srv_name)
    while not client.service_is_ready():
        require(clock() < discovery_deadline, 'SERVICE_UNAVAILABLE:' + client.srv_name)
        spin()
    require(clock() <= discovery_deadline, 'SERVICE_UNAVAILABLE:' + client.srv_name)
    remaining = deadline-clock()
    require(remaining > 0, 'SERVICE_TIMEOUT:' + client.srv_name)
    return read_service(client, request, spin, remaining, clock=clock, events=events)


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def option(argv, name):
    require(argv.count(name) == 1, 'MISSING_OR_DUPLICATE_OPTION:' + name)
    index = argv.index(name)
    require(index + 1 < len(argv), 'MISSING_VALUE:' + name)
    return argv[index + 1]


def verify_owner(owner, session, source, environ=None, proc=Path('/proc')):
    environ = os.environ if environ is None else environ
    require((proc/'sys/kernel/random/boot_id').read_text().strip() == owner['boot_id'], 'BOOT_CHANGED')
    process = proc/str(int(owner['pid']))
    require(process.joinpath('stat').read_text().rsplit(') ', 1)[1].split()[19] == str(owner['start_ticks']), 'PID_REUSED')
    require(str(process.joinpath('exe').resolve()) == owner['exe'], 'EXECUTABLE_CHANGED')
    argv = process.joinpath('cmdline').read_bytes().decode().rstrip('\0').split('\0')
    require(argv == owner['cmdline'], 'COMMAND_CHANGED')
    require(any(Path(v).name == 'sim_stack_supervisor.py' for v in argv[:3]), 'SUPERVISOR_REQUIRED')
    domain = option(argv, '--ros-domain-id')
    require(domain.isdigit() and 1 <= int(domain) <= 101 and int(domain) != 25, 'ISOLATED_DOMAIN_REQUIRED')
    require(domain == environ.get('ROS_DOMAIN_ID'), 'DOMAIN_MISMATCH')
    require(option(argv, '--instance') == session and environ.get('IGN_PARTITION') == 'astribot_' + session, 'SESSION_MISMATCH')
    require(environ.get('ASTRIBOT_SIM_INSTANCE') == session, 'INSTANCE_ENV_MISMATCH')
    require(option(argv, '--navigation-geometry-mode') == 'fixed_v2', 'FIXED_V2_REQUIRED')
    require(option(argv, '--payload-source-id') == source, 'PAYLOAD_SOURCE_MISMATCH')
    require(option(argv, '--navigation-policy') in ('p4', 'p5'), 'P4_OR_P5_REQUIRED')


CAMERAS = {'head_rgbd', 'torso_rgbd', 'left_wrist_rgbd', 'right_wrist_rgbd',
           'head_stereo_left', 'head_stereo_right'}


def camera_structure(xml):
    root = ET.fromstring(xml)
    def normalize(e):
        return (e.tag, sorted(e.attrib.items()), (e.text or '').strip(), [normalize(c) for c in e])
    result = {e.get('name'): normalize(e) for e in root if e.tag in ('link', 'joint')
              and ('rgbd' in e.get('name', '') or 'stereo' in e.get('name', ''))}
    require({name + '_camera_link' for name in CAMERAS}.issubset(result), 'SIX_PHYSICAL_CAMERAS_REQUIRED')
    return result


def ns(stamp):
    return stamp['sec'] * 1_000_000_000 + stamp['nanosec']


def capture_key(kind, value):
    if kind == 'diagnostic':
        return (value['source_epoch'], value['clock_epoch'], value['stamp_ns'])
    if kind == 'ledger':
        value = value['observation']
    if kind in ('source', 'ledger'):
        return (value['source_epoch'], value['clock_epoch'], ns(value['observed_at']))
    return (value['source_id'], value['clock_epoch'], ns(value['header']['stamp']))


class CaptureReceipts:
    """A repeated capture cannot extend the verification observation window."""
    def __init__(self):
        self.keys, self.received = {}, {}

    def observe(self, kind, value, wall):
        key = capture_key(kind, value)
        previous = self.keys.get(kind)
        if previous is not None and key[:2] == previous[:2] and key[2] <= previous[2]:
            return  # duplicate or reordered source does not refresh wall freshness
        self.keys[kind], self.received[kind] = key, wall


def proof_key(latest):
    observation = latest['ledger']['observation']
    return (observation['source_epoch'], observation['clock_epoch'], observation['revision'],
            latest['ledger']['ledger_epoch'], latest['ledger']['attachment_revision'], latest['geometry']['model_revision'],
            latest['geometry']['source_id'], latest['geometry']['clock_epoch'], latest['geometry']['header']['frame_id'])


class ReadbackBarrier:
    def __init__(self, key, wall, ros):
        self.key, self.wall, self.ros, self.valid = key, wall, ros, True

    def observe(self, ready, key, wall, ros):
        self.valid = self.valid and ready and key == self.key and 0 <= wall-self.wall < .3 and 0 <= ros-self.ros < 300_000_000
        return self.valid


def empty_ready(latest, received, wall, ros_ns, session, source, policy='static_world_empty_only_v1'):
    if policy not in ('static_world_empty_only_v1', 'kinematic_inventory_v1'):
        return False
    if ros_ns <= 0 or any(wall - received.get(k, float('-inf')) >= .3 for k in ('source', 'ledger', 'geometry', 'diagnostic')):
        return False
    s, state, g, d = (latest[k] for k in ('source', 'ledger', 'geometry', 'diagnostic'))
    observation = state['observation']
    for value in (s, observation):
        if not (value['environment'] == 'simulation' and value['session_id'] == session and value['source_id'] == source
                and value['source_epoch'] and value['revision'] > 0 and value['sequence'] > 0 and value['full_inventory']
                and value['status'] == 1 and not value['objects'] and 0 < ns(value['observed_at']) <= ros_ns < ns(value['valid_until'])
                and 0 < ns(value['valid_until'])-ns(value['observed_at']) <= 300_000_000):
            return False
    key = lambda value: (value['source_epoch'], value['clock_epoch'], value['revision'])
    if key(s) != key(observation):
        return False
    return bool(state['confirmed'] and state['attachment_revision'] and state['ledger_epoch']
                and 0 < ns(state['published_at']) <= ros_ns < ns(state['valid_until'])
                and g['complete'] and g['attachment_state_confirmed'] and not g['attachment_ids']
                and g['attachment_revision'] == state['attachment_revision'] and g['model_revision']
                and 0 < ns(g['header']['stamp']) <= ros_ns < ns(g['valid_until'])
                and d['policy'] == policy and d['reason'] == 'EMPTY_INVENTORY_OBSERVED'
                and d['source_epoch'] == s['source_epoch'] and d['revision'] == s['revision'] and d['clock_epoch'] == s['clock_epoch']
                and 0 < d['stamp_ns'] <= ros_ns < d['stamp_ns'] + 300_000_000)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--owner', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='new evidence directory')
    parser.add_argument('--session', required=True)
    parser.add_argument('--source', required=True)
    parser.add_argument('--plugin-directory', type=Path, required=True, help='installed lib directory containing both inventory libraries and package executable subdirectory')
    parser.add_argument('--world-reference', type=Path, required=True)
    parser.add_argument('--observe-existing', action='store_true', help='read-only retry; never try loading the source twice')
    args = parser.parse_args()
    owner = json.loads(args.owner.read_text())
    verify_owner(owner, args.session, args.source)
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'passed': False, 'kind': 'empty_bootstrap_observation', 'owner': owner,
              'started_wall': time.time(), 'observer_load_attempted': False, 'observe_existing': args.observe_existing}
    import rclpy
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import GetParameters
    from moveit_msgs.srv import GetPlanningScene
    from std_msgs.msg import String
    from astribot_payload_msgs.msg import AttachmentState, AttachmentObservation
    from astribot_navigation_msgs.msg import RobotGeometryState
    from rosidl_runtime_py.convert import message_to_ordereddict
    rclpy.init()
    node = rclpy.create_node('prepare_empty_' + uuid.uuid4().hex[:8], parameter_overrides=[Parameter('use_sim_time', value=True)])
    latest, samples = {}, []
    receipts = CaptureReceipts()
    readback = None
    def ready():
        return empty_ready(latest, receipts.received, time.monotonic(), node.get_clock().now().nanoseconds, args.session, args.source)
    def receive(key, value):
        latest[key] = json.loads(value.data) if key == 'diagnostic' else message_to_ordereddict(value)
        receipts.observe(key, latest[key], time.monotonic())
        if readback is not None:
            readback.observe(ready(), proof_key(latest), time.monotonic(), node.get_clock().now().nanoseconds)
    for typ, topic, key in ((AttachmentObservation, '/payload/attachment_observation', 'source'),
                            (AttachmentState, '/payload/attachment_state', 'ledger'),
                            (RobotGeometryState, '/navigation/geometry_state', 'geometry'),
                            (String, '/payload/simulation_inventory_diagnostics', 'diagnostic')):
        node.create_subscription(typ, topic, lambda v, k=key: receive(k, v), 10)
    try:
        urdfs = {}
        for target in ('robot_state_publisher', 'move_group'):
            client = node.create_client(GetParameters, '/' + target + '/get_parameters')
            result = read_model_parameters(client, GetParameters.Request(names=['robot_description', 'use_sim_time']),
                lambda: rclpy.spin_once(node, timeout_sec=.01), events=report.setdefault('parameter_read_attempts', []))
            require(len(result.values) == 2 and result.values[1].bool_value, 'SIMULATION_MODEL_REQUIRED:' + target)
            urdfs[target] = result.values[0].string_value
            (args.output/(target + '.urdf')).write_text(urdfs[target])
        require(camera_structure(urdfs['robot_state_publisher']) == camera_structure(urdfs['move_group']), 'CAMERA_MODEL_MISMATCH')
        report['six_camera_models_equal'] = True
        robot_path = args.output/'robot_state_publisher.urdf'
        converted = subprocess.run(['ign', 'sdf', '-p', str(robot_path)], capture_output=True, text=True, timeout=20)
        require(converted.returncode == 0, 'SDF_CONVERSION_FAILED:' + converted.stderr)
        ET.fromstring(converted.stdout)
        reference = args.output/'robot_reference.sdf'
        reference.write_text(converted.stdout)
        report['references'] = {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in (robot_path, reference, args.world_reference)}
        if not args.observe_existing:
            verify_owner(owner, args.session, args.source)
            loader = args.plugin_directory/'astribot_s1_gazebo_bringup/load_empty_inventory'
            command = [str(loader.resolve()), '--world', 'default', '--robot', 'astribot_s1', '--session', args.session,
                       '--source', args.source, '--world-reference', str(args.world_reference.resolve()),
                       '--robot-reference', str(reference.resolve()), '--plugin', str((args.plugin_directory/'libastribot_empty_inventory.so').resolve())]
            report['observer_load_attempted'] = True
            report['loader_command'] = command
            result = subprocess.run(command, capture_output=True, text=True, timeout=15)
            report['loader_result'] = {'exit_code': result.returncode, 'stdout': result.stdout, 'stderr': result.stderr}
            require(result.returncode == 0, 'CPP_OBSERVER_LOAD_FAILED')
        client = node.create_client(GetPlanningScene, '/get_planning_scene')
        require(client.wait_for_service(timeout_sec=5), 'FULL_SCENE_SERVICE_UNAVAILABLE')
        stable, first_key = None, None
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            wall, ros = time.monotonic(), node.get_clock().now().nanoseconds
            current_ready = ready()
            samples.append({'wall': wall, 'ros_ns': ros, 'ready': current_ready})
            key = proof_key(latest) if current_ready else None
            if not current_ready or key != first_key:
                stable, first_key = None, key
            elif stable is None:
                stable = wall
            elif wall-stable >= 2:
                readback = ReadbackBarrier(key, wall, ros)
                request = GetPlanningScene.Request()
                request.components.components = 4
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=.25)
                valid = readback.observe(ready(), proof_key(latest), time.monotonic(), node.get_clock().now().nanoseconds)
                report.setdefault('readback_attempts', []).append({'key': key, 'request_wall': wall, 'request_ros_ns': ros,
                                                                 'valid': valid, 'completed': future.done()})
                readback = None
                if not future.done():
                    client.remove_pending_request(future)
                if not valid or not future.done():
                    stable, first_key = None, None
                    continue
                full_scene = future.result().scene
                report['independent_full_scene'] = message_to_ordereddict(full_scene)
                require(not full_scene.is_diff and not full_scene.robot_state.is_diff and not full_scene.robot_state.attached_collision_objects, 'FULL_SCENE_NOT_EMPTY')
                break
        else:
            raise RuntimeError('CONTINUOUS_EMPTY_READINESS_TIMEOUT')
        verify_owner(owner, args.session, args.source)
        report['passed'] = True
    except Exception as error:
        report['error'] = str(error)
    finally:
        report.update(latest=latest, samples=samples, finished_wall=time.time())
        (args.output/'result.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({k: v for k, v in report.items() if k not in ('samples', 'latest', 'independent_full_scene')}, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
