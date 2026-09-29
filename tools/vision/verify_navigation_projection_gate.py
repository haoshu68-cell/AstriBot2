#!/usr/bin/env python3
"""Stationary warehouse validation of raw/processing capability at final protection.

Only pidfd-identified projection nodes owned by the supplied session are paused.
This script publishes no commands and always resumes each suspended process.
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


def owned_process(executable, node_name, session, directory):
    matches = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if (proc / 'exe').resolve() != executable:
                continue
            argv = (proc / 'cmdline').read_bytes().decode().split('\0')
            if node_name and '__node:=' + node_name not in argv:
                continue
            env = dict(x.split('=', 1) for x in (proc / 'environ').read_bytes().decode().split('\0') if '=' in x)
            if env.get('ASTRIBOT_LOG_DIR') != str(directory):
                continue
            if any(env.get(k) != session['isolation'][k] for k in ('ROS_DOMAIN_ID', 'ASTRIBOT_SIM_INSTANCE', 'IGN_PARTITION')):
                continue
            before = (proc / 'stat').read_text().rsplit(') ', 1)[1].split()
            roots = [c['pid'] for c in session['children'] if c['name'] == directory.name]
            if len(roots) != 1:
                raise RuntimeError('Missing session launch owner')
            ancestor, seen = int(before[1]), set()
            while ancestor > 1 and ancestor not in seen and ancestor != roots[0]:
                seen.add(ancestor)
                ancestor = int(Path(f'/proc/{ancestor}/stat').read_text().rsplit(') ', 1)[1].split()[1])
            if ancestor != roots[0]:
                continue
            fd = os.pidfd_open(int(proc.name))
            if (proc / 'stat').read_text().rsplit(') ', 1)[1].split()[19] != before[19]:
                os.close(fd)
                continue
            matches.append((fd, dict(pid=int(proc.name), start_ticks=before[19], argv=argv,
                executable=str(executable), sha256=hashlib.sha256(executable.read_bytes()).hexdigest())))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    if len(matches) != 1:
        for fd, _ in matches:
            os.close(fd)
        raise RuntimeError(f'Expected one owned {node_name or executable.name}, found {len(matches)}')
    return matches[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('session', 'projector', 'protection', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=360.)
    a = parser.parse_args()
    if not 210 <= a.seconds <= 600:
        parser.error('seconds must be within [210,600]')
    session = json.loads(a.session.read_text())
    if session['state'] != 'ready' or any(os.environ.get(k) != session['isolation'][k]
            for k in ('ROS_DOMAIN_ID', 'ASTRIBOT_SIM_INSTANCE', 'IGN_PARTITION')):
        raise RuntimeError('Query environment is not the supplied ready session')
    a.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rcl_interfaces.srv import GetParameters
    from astribot_perception_msgs.msg import CameraHealth, ProjectionHealth
    from astribot_navigation_msgs.msg import MotionConstraint
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('navigation_projection_gate_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    started = time.monotonic()
    rows, faults, latest, raw, projection = [], [], {}, {}, {}
    odom, stable_since = None, None
    suspended = None
    stream = (a.output / 'events.jsonl').open('x')
    report = dict(scope='stationary actual warehouse navigation execution protection; no motion goal',
        session=str(a.session.resolve()), faults=faults, started_wall=time.time(), hardware=False, vla=False,
        validation_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        thresholds=dict(invalid_sec=.5, recovery_sec=3., pause_sec=.6))
    def emit(row):
        row['elapsed'] = time.monotonic() - started
        rows.append(row)
        stream.write(json.dumps(row) + '\n')
        return row
    def raw_cb(camera):
        def cb(h):
            raw[camera] = emit(dict(kind='raw', camera=camera, valid=h.valid, capture_ns=ns(h.capture_stamp),
                header_ns=ns(h.header.stamp), epoch=h.source_epoch, reason=h.reason_code, until_ns=ns(h.valid_until)))
        return cb
    def projection_cb(camera):
        def cb(h):
            projection[camera] = emit(dict(kind='projection', camera=camera, valid=h.valid, capture_ns=ns(h.capture_stamp),
                header_ns=ns(h.header.stamp), epoch=h.processing_epoch, reason=h.reason_code))
        return cb
    def diagnostic(h):
        latest['state'] = emit(dict(kind='state', **json.loads(h.data)))
    def command(h):
        latest['command'] = emit(dict(kind='command', velocity=[h.linear.x,h.linear.y,h.linear.z,h.angular.x,h.angular.y,h.angular.z]))
    def constraint(h):
        latest['constraint'] = emit(dict(kind='constraint', hold=h.hold, reason=h.reason, stamp_ns=ns(h.stamp)))
    def odometry(h):
        nonlocal odom
        t = h.twist.twist
        odom = (time.monotonic(), [t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z])
    qos = QoSProfile(depth=4, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for camera in ('head_rgbd', 'torso_rgbd'):
        node.create_subscription(CameraHealth, '/perception/camera_health/' + camera, raw_cb(camera), qos)
        node.create_subscription(ProjectionHealth, '/perception/projection_health/' + camera, projection_cb(camera), qos)
    node.create_subscription(String, '/navigation_policy/protection_state', diagnostic, 20)
    node.create_subscription(Twist, '/cmd_vel', command, 20)
    node.create_subscription(MotionConstraint, '/navigation_policy/constraint', constraint, 20)
    node.create_subscription(Odometry, '/odom', odometry, qos_profile_sensor_data)
    try:
        fd, report['protection_owner'] = owned_process(a.protection.resolve(), None, session, a.session.parent.resolve() / 'navigation_1')
        os.close(fd)
        client = node.create_client(GetParameters, '/navigation_final_protection/get_parameters')
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('Final protection parameter service unavailable')
        future = client.call_async(GetParameters.Request(names=['required_projection_cameras']))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done() or future.result() is None:
            raise RuntimeError('Final protection parameter query failed')
        report['required_cameras'] = json.loads(future.result().values[0].string_value)
        if report['required_cameras'] != ['head_rgbd', 'torso_rgbd']:
            raise RuntimeError('Head and torso processing capability was not enabled')
        schedule = [(30., 'head_rgbd'), (90., 'torso_rgbd'), (180., 'head_rgbd')]
        next_fault = 0
        while time.monotonic() - started < a.seconds:
            rclpy.spin_once(node, timeout_sec=.005)
            current = time.monotonic()
            stationary = odom and current-odom[0] < .2 and all(math.isfinite(v) and abs(v) < .005 for v in odom[1])
            if not stationary:
                stable_since = None
            elif stable_since is None:
                stable_since = current
            if suspended and current >= suspended[1]:
                signal.pidfd_send_signal(suspended[0], signal.SIGCONT)
                os.close(suspended[0])
                faults[-1]['resumed_elapsed'] = current-started
                suspended = None
            if next_fault < len(schedule) and current-started >= schedule[next_fault][0]:
                _, camera = schedule[next_fault]
                state = latest.get('state', {})
                if not stable_since or current-stable_since < 2. or not state.get('projection_ready') or current-started-state['elapsed'] > .2:
                    raise RuntimeError('Missing fresh stationary and projection-ready precondition')
                if not all(c in raw and c in projection and raw[c]['valid'] and projection[c]['valid'] and
                    current-started-raw[c]['elapsed'] < .15 and current-started-projection[c]['elapsed'] < .15 for c in ('head_rgbd','torso_rgbd')):
                    raise RuntimeError('Missing healthy source precondition')
                publishers = node.get_publishers_info_by_topic('/cmd_vel')
                owners = [x.node_namespace.rstrip('/') + '/' + x.node_name for x in publishers]
                if owners != ['/navigation_final_protection']:
                    raise RuntimeError('Unexpected command ownership: ' + repr(owners))
                fd, owner = owned_process(a.projector.resolve(), camera + '_pointcloud', session, a.session.parent.resolve() / 'simulation')
                fault = dict(camera=camera, elapsed=current-started, ros_ns=node.get_clock().now().nanoseconds,
                    before_revision=state['projection_revision'], owner=owner, cmd_vel_owners=owners, stationary_velocity=odom[1])
                suspended = (fd, current+.6)
                signal.pidfd_send_signal(fd, signal.SIGSTOP)
                faults.append(fault)
                stream.flush()
                next_fault += 1
        report['completed'] = True
    except (Exception, KeyboardInterrupt) as error:
        report.update(completed=False, error=str(error), traceback=traceback.format_exc())
    finally:
        if suspended:
            try:
                signal.pidfd_send_signal(suspended[0], signal.SIGCONT)
            except ProcessLookupError:
                pass
            os.close(suspended[0])
        report['duration_sec'] = time.monotonic()-started
        stream.close()
        node.destroy_node()
        rclpy.shutdown()
    for f in faults:
        states = [r for r in rows if r['kind']=='state' and r['elapsed'] >= f['elapsed']]
        invalid = next((r for r in states if not r.get('projection_ready', True)), None)
        resumed = f.get('resumed_elapsed', float('inf'))
        recovered = next((r for r in states if r['elapsed'] >= resumed and r.get('projection_ready') and r['projection_revision'] > f['before_revision']), None)
        raw_samples = [r for r in rows if r['kind']=='raw' and r['camera']==f['camera'] and f['elapsed'] <= r['elapsed'] <= resumed]
        constraint_samples = [r for r in rows if r['kind']=='constraint' and f['elapsed']+.35 <= r['elapsed'] < resumed]
        captures = [r for r in rows if r['kind']=='projection' and r['camera']==f['camera'] and r['elapsed'] >= resumed and r['valid'] and r['capture_ns'] > f['ros_ns']]
        f.update(invalid_sec=None if invalid is None else invalid['elapsed']-f['elapsed'],
            recovery_after_resume_sec=None if recovered is None else recovered['elapsed']-resumed,
            raw_count=len(raw_samples), raw_stayed_valid=bool(raw_samples) and all(r['valid'] for r in raw_samples),
            hold_count=len(constraint_samples), held_during_fault=bool(constraint_samples) and all(r['hold'] and 'PROJECTION' in r['reason'] for r in constraint_samples),
            recovered_with_new_capture=bool(captures), invalid_state=invalid, recovered_state=recovered)
        f['pass'] = bool(invalid and recovered and f['invalid_sec'] <= .5 and f['recovery_after_resume_sec'] <= 3. and
            f['raw_stayed_valid'] and f['held_during_fault'] and f['recovered_with_new_capture'])
    commands = [r for r in rows if r['kind']=='command']
    states = [r for r in rows if r['kind']=='state']
    stable = [r for r in states if r['elapsed'] > 5. and not any(f['elapsed'] <= r['elapsed'] <= f.get('resumed_elapsed', f['elapsed'])+3. for f in faults)]
    unexpected_revisions = []
    for before, after in zip(states, states[1:]):
        if after['elapsed'] > 5. and after['projection_revision'] != before['projection_revision'] and not any(
                f['elapsed'] <= after['elapsed'] <= f.get('resumed_elapsed', f['elapsed'])+3. for f in faults):
            unexpected_revisions.append(dict(elapsed=after['elapsed'], before=before['projection_revision'], after=after['projection_revision']))
    unexpected_constraints = [r for r in rows if r['kind']=='constraint' and r['elapsed'] > 5. and
        ('PROJECTION' in r['reason'] or r['reason'].startswith('CAMERA_HEALTH_INVALID')) and not any(
            f['elapsed'] <= r['elapsed'] <= f.get('resumed_elapsed', f['elapsed'])+3. for f in faults)]
    report.update(command_samples=len(commands), state_samples=len(states), stable_state_samples=len(stable),
        commands_zero=bool(commands) and all(all(math.isfinite(x) and abs(x)<1e-9 for x in r['velocity']) for r in commands),
        stable_projection_invalid=sum(not r.get('projection_ready',False) for r in stable),
        unexpected_revisions=unexpected_revisions, unexpected_constraint_samples=len(unexpected_constraints))
    report['pass'] = bool(report.get('completed') and len(faults)==3 and all(f['pass'] for f in faults) and
        report['commands_zero'] and len(stable)>100 and report['stable_projection_invalid']==0 and
        not unexpected_revisions and not unexpected_constraints)
    (a.output/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)
    return 0 if report['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
