#!/usr/bin/env python3
"""Exercise task-owned C++ wrist sessions against actual owned Gazebo images."""
import argparse
import json
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import time

from owned_process_cleanup import cleanup
from camera_clock_retry import call_with_clock_retry


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--hold-seconds', type=int, default=0)
    parser.add_argument('--scenario', default='camera_slam_navigation_idle')
    parser.add_argument('--dds-profile', type=Path)
    parser.add_argument('--launch-file', type=Path,
        help='Optional explicit launch candidate; copied into evidence before use')
    args = parser.parse_args()
    if not 0 <= args.hold_seconds <= 1800:
        raise ValueError('hold-seconds must be 0..1800')
    if args.dds_profile and not args.dds_profile.is_file():
        raise ValueError('DDS profile does not exist')
    session = json.loads(args.session.read_text())
    identity = session.get('env', session.get('isolation'))
    for key in ('ROS_DOMAIN_ID', 'IGN_PARTITION', 'ASTRIBOT_SIM_INSTANCE'):
        if identity[key] != os.environ.get(key):
            raise RuntimeError('Query environment does not match owned session')
    launch_pid = session.get('launch_pid', session.get('children', [{}])[0].get('pid'))
    proc = Path('/proc') / str(launch_pid)
    stat = (proc/'stat').read_text().split(') ', 1)[1].split()
    live = dict(x.split('=', 1) for x in (proc/'environ').read_bytes().decode().split('\0') if '=' in x)
    recorded_tick = session.get('launch_start_ticks')
    expected_log = args.session.parent.resolve()
    if 'children' in session:
        expected_log /= session['children'][0]['name']
    group_identity = live.get('ASTRIBOT_LOG_DIR') == str(expected_log)
    if (recorded_tick is not None and stat[19] != recorded_tick) or not group_identity or any(live.get(k) != identity[k] for k in ('ROS_DOMAIN_ID', 'IGN_PARTITION')):
        raise RuntimeError('Session process identity changed')
    args.output.mkdir(parents=True, exist_ok=False)
    launch_command=['astribot_s1_perception_components','wrist_camera_session.launch.py']
    launch_hash=None
    if args.launch_file:
        launch_text=args.launch_file.read_bytes()
        snapshot=args.output.resolve()/'wrist_camera_session.launch.py'
        snapshot.write_bytes(launch_text);launch_hash=hashlib.sha256(launch_text).hexdigest()
        launch_command=[str(snapshot)]
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from astribot_perception_msgs.srv import SetCameraSession
    from astribot_perception_msgs.msg import CameraHealth
    from sensor_msgs.msg import PointCloud2
    rclpy.init()
    node = rclpy.create_node('wrist_session_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
    health, clouds, records, children, logs, subscriptions = {}, {}, [], [], [], []
    report = {'scope': 'C++ task lease with Gazebo wrist RGB-D, no arm/base execution', 'checks': {},
              'probe_pid':os.getpid(), 'dds_profile':str(args.dds_profile) if args.dds_profile else None,
              'launch_sha256':launch_hash,'started_wall':time.time(),
              'validation_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'retry_sha256':hashlib.sha256(Path(__file__).with_name('camera_clock_retry.py').read_bytes()).hexdigest()}
    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
    def check(name, ok):
        report['checks'][name] = bool(ok)
        if not ok:
            raise AssertionError(name)
    try:
        for side in ('left', 'right'):
            camera = side + '_wrist_rgbd'
            clouds[camera] = 0
            def receive_health(msg, camera=camera):
                health[camera] = msg
                records.append({'wall': time.monotonic(), 'camera': camera, 'valid': msg.valid,
                                'state': msg.state, 'epoch': msg.source_epoch, 'age_sec': msg.age_sec,
                                'valid_until_ns':msg.valid_until.sec*10**9+msg.valid_until.nanosec})
            def receive_cloud(msg, camera=camera):
                clouds[camera] += 1
            subscriptions.append(node.create_subscription(CameraHealth, '/perception/camera_health/'+camera,
                receive_health, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
            subscriptions.append(node.create_subscription(PointCloud2, '/manipulation/camera/'+camera+'/points',
                receive_cloud, qos_profile_sensor_data))
            log = (args.output/(camera+'.log')).open('w'); logs.append(log)
            env = dict(os.environ, ASTRIBOT_LOG_DIR=str(args.output.resolve()))
            children.append(subprocess.Popen(['ros2', 'launch', *launch_command,
                'camera_id:='+camera, 'calibration_revision:=2026092101',
                *(['dds_profile:='+str(args.dds_profile.resolve())] if args.dds_profile else [])],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        pump(3.)
        for side in ('left', 'right'):
            camera = side + '_wrist_rgbd'
            client = node.create_client(SetCameraSession, '/perception/camera_session/'+camera+'/set')
            check(camera+'/service', client.wait_for_service(timeout_sec=3.))
            check(camera+'/inactive', camera in health and not health[camera].valid and clouds[camera] == 0)
            sequence = 0
            def request(operation, token='', owner='probe', lease=2.):
                nonlocal sequence
                sequence += 1
                req = SetCameraSession.Request()
                req.header.stamp = node.get_clock().now().to_msg()
                req.camera_id = camera; req.owner_id = owner; req.execution_id = 'w1_lease_validation'
                req.request_id = camera+str(sequence); req.operation = operation
                req.session_token = token; req.lease_sec = lease
                result = call_with_clock_retry(client, req, pump, service_timeout=3.)
                records.append({'camera': camera, 'operation': operation, 'accepted': result.accepted,
                                'reason': result.reason_code, 'token': result.state.session_token,
                                'request_stamp_ns': req.header.stamp.sec*10**9+req.header.stamp.nanosec,
                                'server_stamp_ns': result.state.header.stamp.sec*10**9+result.state.header.stamp.nanosec})
                return result
            activation_begin = len(records)
            first = request(0); check(camera+'/acquire', first.accepted); token = first.state.session_token
            pump(.8)
            accepted_frames = [e for e in records[activation_begin:] if e.get('camera')==camera and e.get('valid')]
            check(camera+'/fresh_cloud', bool(accepted_frames) and clouds[camera] >= 3)
            first_epoch = accepted_frames[-1]['epoch']
            check(camera+'/busy_owner', not request(0, owner='other').accepted)
            check(camera+'/renew', request(1, token).accepted)
            pump(.4)
            check(camera+'/release', request(2, token).accepted)
            pump(.3); stopped = clouds[camera]; pump(.3)
            check(camera+'/release_stops_cloud', clouds[camera] == stopped and not health[camera].valid)
            check(camera+'/old_token', not request(1, token).accepted)
            activation_begin = len(records)
            second = request(0); check(camera+'/new_token', second.accepted and second.state.session_token != token)
            pump(.8)
            check(camera+'/new_epoch', any(e.get('camera')==camera and e.get('valid') and
                e['epoch']!=first_epoch for e in records[activation_begin:]))
            pump(1.5); stopped = clouds[camera]; pump(.3)
            check(camera+'/expiry_stops_cloud', clouds[camera] == stopped and not health[camera].valid)
            lease_deadline=second.state.valid_until.sec*10**9+second.state.valid_until.nanosec
            lease_health=[e for e in records[activation_begin:] if e.get('camera')==camera and
                e.get('valid') and second.state.session_token in e['epoch']]
            check(camera+'/health_lifetime_within_lease', bool(lease_health) and all(
                e['valid_until_ns']<=lease_deadline for e in lease_health))
        if args.hold_seconds:
            clients, tokens = {}, {}
            hold_sequence = 0
            def hold_request(camera, operation):
                nonlocal hold_sequence
                hold_sequence += 1
                req = SetCameraSession.Request()
                req.header.stamp = node.get_clock().now().to_msg()
                req.camera_id = camera; req.owner_id = 'workload_probe'; req.execution_id = 'w1_workload'
                req.request_id = 'hold_'+camera+str(hold_sequence); req.operation = operation
                req.session_token = tokens.get(camera, ''); req.lease_sec = 2.
                result = call_with_clock_retry(clients[camera], req, pump,
                    on_retry=lambda request_stamp, server_stamp: records.append({
                        'camera':camera, 'clock_order_retry':True,
                        'request_stamp_ns':request_stamp, 'server_stamp_ns':server_stamp}))
                records.append({'camera': camera, 'hold_operation': operation,
                    'request_stamp_ns': req.header.stamp.sec*10**9+req.header.stamp.nanosec,
                    'accepted': result.accepted, 'reason': result.reason_code,
                    'server_stamp_ns': result.state.header.stamp.sec*10**9+result.state.header.stamp.nanosec})
                if not result.accepted:
                    raise RuntimeError('Workload lease renewal failed: '+camera+' '+result.reason_code)
                tokens[camera] = result.state.session_token
            for camera in clouds:
                clients[camera] = node.create_client(SetCameraSession, '/perception/camera_session/'+camera+'/set')
                if not clients[camera].wait_for_service(timeout_sec=1.): raise RuntimeError('Service vanished')
                hold_request(camera, 0)
            log = (args.output/'capture.log').open('w'); logs.append(log)
            capture = subprocess.Popen(['python3', str(Path(__file__).with_name('capture_camera_reference.py')),
                '--seconds', str(args.hold_seconds), '--scenario', args.scenario,
                '--output', str(args.output/'capture')],
                stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
            children.append(capture)
            report['hold_started_wall']=time.time()
            (args.output/'report.json').write_text(json.dumps(report,indent=2))
            deadline = time.monotonic() + args.hold_seconds + 20
            while capture.poll() is None and time.monotonic() < deadline:
                for camera in clouds: hold_request(camera, 1)
                pump(.5)
            check('workload/capture_completed', capture.poll() == 0)
            for camera in clouds: hold_request(camera, 2)
            # Capture completeness is separate from sustained camera health acceptance.
            report['workload_health_verdict'] = 'requires_capture_analysis'
        report['passed'] = True
    except Exception as error:
        report.update(passed=False, error=str(error))
    finally:
        for child in children:
            if child.poll() is None: child.send_signal(signal.SIGINT)
        report['cleanup'] = []
        for child in children:
            try: child.wait(timeout=5.)
            except subprocess.TimeoutExpired: pass
            report['cleanup'].append(cleanup(child.pid, args.output, identity['ASTRIBOT_SIM_INSTANCE']))
        for log in logs: log.close()
        report['cloud_counts'] = clouds
        (args.output/'report.json').write_text(json.dumps(report, indent=2))
        (args.output/'events.json').write_text(json.dumps(records, indent=2))
        node.destroy_node(); rclpy.shutdown()
    return 0 if report.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
