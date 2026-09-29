#!/usr/bin/env python3
"""Observe processing/raw health and pause only identified, owned CUDA children.

Validation only. The parent projector remains the owner of termination/restart.
No robot command, Gazebo pause, device reset or VLA inference is performed.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time
import traceback


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def worker_identity(pid, camera, directory, executable, identity):
    proc = Path('/proc') / str(pid)
    fields = (proc / 'stat').read_text().rsplit(') ', 1)[1].split()
    parent = Path('/proc') / fields[1]
    env = dict(x.split('=', 1) for x in (proc / 'environ').read_bytes().decode().split('\0') if '=' in x)
    argv = (proc / 'cmdline').read_bytes().decode().split('\0')
    parent_argv = (parent / 'cmdline').read_bytes().decode().split('\0')
    if (proc / 'exe').resolve() != executable or argv[1:3] != ['--backend', 'cuda']:
        raise RuntimeError('Unexpected worker executable/backend')
    if (parent / 'exe').resolve() != executable.with_name('rgbd_pointcloud_node'):
        raise RuntimeError('Unexpected projection owner')
    if '__node:=' + camera + '_manipulation_pointcloud' not in parent_argv:
        raise RuntimeError('Unexpected camera owner')
    if env.get('ASTRIBOT_LOG_DIR') != str(directory):
        raise RuntimeError('Worker not owned by this wrist validation directory')
    for key in ('ROS_DOMAIN_ID', 'ASTRIBOT_SIM_INSTANCE', 'IGN_PARTITION'):
        if env.get(key) != identity[key]:
            raise RuntimeError('Worker isolation identity mismatch: ' + key)
    fd = os.pidfd_open(pid)
    if (proc / 'stat').read_text().rsplit(') ', 1)[1].split()[19] != fields[19]:
        os.close(fd)
        raise RuntimeError('Worker identity changed before injection')
    return fd, dict(pid=pid, parent=int(fields[1]), start_ticks=fields[19], argv=argv,
                    executable=str(executable), executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('session', 'wrist-owner-dir', 'worker', 'output'):
        p.add_argument('--' + name, type=Path, required=True)
    p.add_argument('--seconds', type=float, default=1790.)
    a = p.parse_args()
    if not 330 <= a.seconds <= 1800:
        p.error('seconds must be within [330, 1800]')
    session = json.loads(a.session.read_text())
    identity = session['isolation']
    if session['state'] != 'ready' or any(os.environ.get(k) != identity[k] for k in
            ('ROS_DOMAIN_ID', 'ASTRIBOT_SIM_INSTANCE', 'IGN_PARTITION')):
        raise RuntimeError('Not querying the owned ready simulation')
    a.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from astribot_perception_msgs.msg import ProjectionHealth, CameraHealth
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    rclpy.init()
    node = rclpy.create_node('projection_recovery_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    cameras = ('head_rgbd', 'torso_rgbd', 'left_wrist_rgbd', 'right_wrist_rgbd')
    latest, raw, rows, faults, subscriptions, fds = {}, {}, [], [], [], []
    odom = None
    stable_since = None
    started = time.monotonic()
    stream = (a.output / 'events.jsonl').open('x')
    report = dict(scope='stationary warehouse + SLAM + activated Nav2 + actual CUDA wrist projection',
                  hardware=False, vla=False, session=str(a.session.resolve()), started_wall=time.time(),
                  validation_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  thresholds=dict(publication_age_sec=.25, invalid_detection_sec=.5, recovery_sec=4.5), faults=faults)
    def emit(row):
        row['elapsed'] = time.monotonic() - started
        rows.append(row)
        stream.write(json.dumps(row) + '\n')
    def health(camera):
        def callback(h):
            row = dict(kind='projection', camera=camera, valid=h.valid, state=h.state, reason=h.reason_code,
                       epoch=h.processing_epoch, worker_epoch=h.worker_epoch, worker_pid=h.worker_pid,
                       backend=h.backend, capture_ns=ns(h.capture_stamp), published_ns=ns(h.published_stamp),
                       first_capture_ns=ns(h.epoch_first_capture_stamp), header_ns=ns(h.header.stamp),
                       until_ns=ns(h.valid_until), sequence=h.sequence, errors=h.errors,
                       pending=h.pending_depth, discarded=h.discarded, processed=h.processed, restarts=h.restarts)
            latest[camera] = row
            emit(row)
        return callback
    def raw_health(camera):
        def callback(h):
            row = dict(kind='raw', camera=camera, valid=h.valid, epoch=h.source_epoch, reason=h.reason_code)
            raw[camera] = row
            emit(row)
        return callback
    def cloud(camera):
        def callback(h):
            emit(dict(kind='cloud', camera=camera, capture_ns=ns(h.header.stamp), frame=h.header.frame_id))
        return callback
    def on_odom(message):
        nonlocal odom
        t = message.twist.twist
        odom = (time.monotonic(), [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z])
    qos = QoSProfile(depth=4, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for c in cameras:
        subscriptions.append(node.create_subscription(ProjectionHealth, '/perception/projection_health/' + c, health(c), qos))
        subscriptions.append(node.create_subscription(CameraHealth, '/perception/camera_health/' + c, raw_health(c), qos))
        topic = '/manipulation/camera/' + c + '/points' if 'wrist' in c else '/camera/' + c + '/points_raw'
        subscriptions.append(node.create_subscription(PointCloud2, topic, cloud(c), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data))
    schedule = [(45., 'left_wrist_rgbd', signal.SIGSTOP), (150., 'right_wrist_rgbd', signal.SIGKILL),
                (300., 'left_wrist_rgbd', signal.SIGSTOP)]
    if a.seconds > 960:
        schedule.append((900., 'right_wrist_rgbd', signal.SIGSTOP))
    next_fault = 0
    try:
        while time.monotonic() - started < a.seconds:
            rclpy.spin_once(node, timeout_sec=.01)
            current = time.monotonic()
            stationary = odom and current - odom[0] < .2 and all(math.isfinite(x) and abs(x) < .005 for x in odom[1])
            if not stationary:
                stable_since = None
            elif stable_since is None:
                stable_since = current
            if next_fault < len(schedule) and current - started >= schedule[next_fault][0]:
                _, camera, sig = schedule[next_fault]
                if not stable_since or current - stable_since < 2.:
                    raise RuntimeError('No two-second fresh stationary feedback before injection')
                if not all(c in latest and latest[c]['valid'] and c in raw and raw[c]['valid'] and
                           current-started-latest[c]['elapsed'] < .15 and current-started-raw[c]['elapsed'] < .15 for c in cameras):
                    raise RuntimeError('Raw and processing health are not all valid before injection')
                before = latest[camera]
                fd, owned = worker_identity(before['worker_pid'], camera, a.wrist_owner_dir.resolve(), a.worker.resolve(), identity)
                fds.append(fd)
                event = dict(camera=camera, signal=sig.name, elapsed=current-started,
                             ros_ns=node.get_clock().now().nanoseconds, before_epoch=before['epoch'],
                             before_worker=before['worker_epoch'], owned=owned, stationary_velocities=odom[1],
                             peer_workers={c: latest[c]['worker_epoch'] for c in cameras if c != camera})
                signal.pidfd_send_signal(fd, sig)
                faults.append(event)
                (a.output / 'injections.json').write_text(json.dumps(faults, indent=2))
                stream.flush()
                next_fault += 1
        report['completed'] = True
    except (Exception, KeyboardInterrupt) as error:
        report.update(completed=False, error=str(error), traceback=traceback.format_exc())
    finally:
        # The parent may already have reaped this exact child. A still-owned
        # stopped child is resumed; the validator never leaves it suspended.
        for fd in fds:
            try:
                signal.pidfd_send_signal(fd, signal.SIGCONT)
            except ProcessLookupError:
                pass
            os.close(fd)
        report['duration_sec'] = time.monotonic() - started
        stream.close()
        node.destroy_node()
        rclpy.shutdown()
    windows = []
    for f in faults:
        after = [r for r in rows if r['kind'] == 'projection' and r['camera'] == f['camera'] and r['elapsed'] >= f['elapsed']]
        invalid = next((r for r in after if not r['valid']), None)
        recovered = next((r for r in after if r['valid'] and r['epoch'] != f['before_epoch'] and
                          r['worker_epoch'] > f['before_worker'] and r['capture_ns'] > f['ros_ns']), None)
        f['invalid_after_sec'] = invalid['elapsed'] - f['elapsed'] if invalid else None
        f['recovered_after_sec'] = recovered['elapsed'] - f['elapsed'] if recovered else None
        end = recovered['elapsed'] if recovered else f['elapsed'] + 4.5
        interval = [r for r in rows if f['elapsed'] <= r['elapsed'] <= end]
        raw_interval = [r for r in interval if r['kind'] == 'raw' and r['camera'] == f['camera']]
        peer_interval = [r for r in interval if r['kind'] == 'projection' and r['camera'] != f['camera']]
        f['raw_samples'] = len(raw_interval)
        f['peer_samples'] = {c: sum(r['camera'] == c for r in peer_interval) for c in f['peer_workers']}
        f['raw_remained_valid'] = bool(raw_interval) and all(r['valid'] for r in raw_interval)
        f['peers_unchanged_and_valid'] = all(f['peer_samples'].values()) and all(
            r['valid'] and r['worker_epoch'] == f['peer_workers'][r['camera']] for r in peer_interval)
        f['passed'] = bool(invalid and recovered and f['invalid_after_sec'] <= .5 and f['recovered_after_sec'] <= 4.5
                           and f['raw_remained_valid'] and f['peers_unchanged_and_valid'])
        windows.append((f['elapsed'], end + 1.))
    report['cameras'] = {}
    for c in cameras:
        health_rows = [r for r in rows if r['kind'] == 'projection' and r['camera'] == c]
        outside = [r for r in health_rows if r['elapsed'] > 10 and not any(begin <= r['elapsed'] <= end for begin, end in windows)]
        valid = [r for r in health_rows if r['valid']]
        report['cameras'][c] = dict(health_samples=len(health_rows), cloud_samples=sum(r['kind']=='cloud' and r['camera']==c for r in rows),
            outside_fault_samples=len(outside), outside_fault_invalid=sum(not r['valid'] for r in outside),
            max_publication_age_sec=max(((r['published_ns']-r['capture_ns'])*1.e-9 for r in valid), default=None),
            all_publications_within_budget=bool(valid) and all(0 <= r['published_ns']-r['capture_ns'] <= 250000000 for r in valid),
            queue_bounded=bool(health_rows) and all(r['pending'] <= 1 for r in health_rows))
    report['passed'] = bool(report.get('completed') and len(faults)==len(schedule) and all(f['passed'] for f in faults) and
        all(c['outside_fault_samples'] and not c['outside_fault_invalid'] and c['all_publications_within_budget'] and c['queue_bounded']
            for c in report['cameras'].values()))
    (a.output / 'report.json').write_text(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
