#!/usr/bin/env python3
"""Read-only, time-aligned HuNav/Gazebo scene acceptance recorder."""
import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from hunav_msgs.msg import Agents
from astribot_navigation_msgs.msg import SocialObservationStatus


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 2:
        parser.error('--seconds must be finite and greater than the 2 s warmup')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = rclpy.create_node('social_scene_acceptance')
    tf = Buffer(); listener = TransformListener(tf, node)
    history = deque(maxlen=300); scans = deque(maxlen=20); poses = deque(maxlen=40)
    records = []; counts = Counter(); start = time.monotonic()

    def record(kind, data):
        counts[kind] += 1
        records.append(dict(kind=kind, wall_s=time.monotonic()-start, **data))

    def state(message):
        data = json.loads(message.data)
        if history and data['stamp_ns'] < history[-1]['stamp_ns']:
            history.clear(); scans.clear(); poses.clear()
        history.append(data); poses.append(data); record('geometry', data)

    def pose_check():
        while poses:
            data = poses[0]
            try:
                transform = tf.lookup_transform('social_sim_world', 'astribot_torso_base',
                                                Time(nanoseconds=data['stamp_ns']))
            except TransformException:
                if history[-1]['stamp_ns']-data['stamp_ns'] < 300_000_000:
                    break
                poses.popleft(); counts['pose_tf_unavailable'] += 1; continue
            poses.popleft()
            p = transform.transform.translation; q = transform.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            record('robot_alignment', {'stamp_ns': data['stamp_ns'],
                   'tf_xyz': [p.x, p.y, p.z], 'gazebo_xy_yaw': data['robot'],
                   'xy_error_m': math.hypot(p.x-data['robot'][0], p.y-data['robot'][1]),
                   'yaw_error_rad': abs(math.remainder(yaw-data['robot'][2], 2*math.pi))})

    def people(message):
        record('people', {'t': seconds(message.header.stamp), 'agents': [
            {'id': a.id, 'x': a.position.position.x, 'y': a.position.position.y,
             'yaw': a.yaw, 'vx': a.velocity.linear.x, 'vy': a.velocity.linear.y,
             'wz': a.velocity.angular.z, 'radius': a.radius} for a in message.agents]})

    def status(message):
        record('status', {'t': seconds(message.header.stamp), 'valid': message.input_valid,
                          'age_s': message.sample_age_s, 'reason': message.reason,
                          'accepted': message.accepted_frames, 'rejected': message.rejected_frames})

    def scan_check():
        while scans and history:
            scan = scans[0]; t = seconds(scan.header.stamp)
            if history[-1]['stamp_ns'] * 1e-9 < t + .05:
                break
            scans.popleft()
            candidates = list(history)
            before = [p for p in candidates if p['stamp_ns'] * 1e-9 <= t]
            after = [p for p in candidates if p['stamp_ns'] * 1e-9 >= t]
            if not before or not after:
                continue
            a, b = before[-1], after[0]
            ta, tb = a['stamp_ns']*1e-9, b['stamp_ns']*1e-9
            if tb-ta > .11:
                continue
            try:
                transform = tf.lookup_transform('social_sim_world', scan.header.frame_id,
                                                Time.from_msg(scan.header.stamp))
            except TransformException:
                counts['scan_tf_unavailable'] += 1; continue
            q = transform.transform.rotation; p = transform.transform.translation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            angles = scan.angle_min + np.arange(len(scan.ranges))*scan.angle_increment + yaw
            ux, uy = np.cos(angles), np.sin(angles); ranges = np.asarray(scan.ranges)
            ratio = 0 if tb == ta else (t-ta)/(tb-ta)
            later = {v['name']: v for v in b['people']}
            for person in a['people']:
                if person['name'] not in later:
                    continue
                other = later[person['name']]
                dx = person['x']+(other['x']-person['x'])*ratio-p.x
                dy = person['y']+(other['y']-person['y'])*ratio-p.y
                along = ux*dx+uy*dy; side2 = dx*dx+dy*dy-along*along
                intersects = (along > 0) & (side2 < person['radius']**2)
                expected = along - np.sqrt(np.maximum(0.,person['radius']**2-side2))
                matches = intersects & np.isfinite(ranges) & (np.abs(ranges-expected) < .08)
                record('scan', {'t':t, 'person':person['name'], 'matching_beams':int(matches.sum()),
                                'expected_beams':int(intersects.sum()), 'distance_m':math.hypot(dx,dy)})

    subscriptions = [
        node.create_subscription(String,'/social_sim/state',state,qos_profile_sensor_data),
        node.create_subscription(Agents,'/simulation/hunav_actor_states',people,qos_profile_sensor_data),
        node.create_subscription(SocialObservationStatus,'/social_navigation/observation_status',status,qos_profile_sensor_data),
        node.create_subscription(LaserScan,'/scan_from_cloud',scans.append,qos_profile_sensor_data),
        node.create_subscription(Clock,'/clock',lambda m: counts.update(clock=1),qos_profile_sensor_data)]
    try:
        while time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.01); scan_check(); pose_check()
    finally:
        (args.output/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
        geom = [r for r in records if r['kind']=='geometry' and r['wall_s']>2]
        status_rows = [r for r in records if r['kind']=='status' and r['wall_s']>2]
        people_rows = [a for r in records if r['kind']=='people' and r['wall_s']>2 for a in r['agents']]
        scan_rows = [r for r in records if r['kind']=='scan' and r['wall_s']>2]
        pose_rows = [r for r in records if r['kind']=='robot_alignment' and r['wall_s']>2]
        alignment = [math.hypot(p['x']-p['visual_root'][0],p['y']-p['visual_root'][1]) for r in geom for p in r['people']]
        summary = {'wall_seconds':time.monotonic()-start, 'counts':dict(counts),
                   'geometry_valid':bool(geom) and all(r['geometry_valid'] for r in geom),
                   'minimum_clearance_lower_bound_m':min((r['clearance_lower_bound_m'] for r in geom),default=None),
                   'conservative_overlap_steps':max((r['conservative_overlap_steps'] for r in geom),default=None),
                   'visual_proxy_xy_max_m':max(alignment,default=None),
                   'visual_root_z_range_m':[min((p['visual_root'][2] for r in geom for p in r['people']),default=None),max((p['visual_root'][2] for r in geom for p in r['people']),default=None)],
                   'observations_valid_fraction':sum(r['valid'] for r in status_rows)/max(1,len(status_rows)),
                   'observation_reasons':dict(Counter(r['reason'] for r in status_rows if r['reason'])),
                   'person_y_range_m':[min((r['y'] for r in people_rows),default=None),max((r['y'] for r in people_rows),default=None)],
                   'max_abs_yaw_rate_rad_s':max((abs(r['wz']) for r in people_rows),default=None),
                   'scan_visible_fraction':sum(r['matching_beams']>=3 for r in scan_rows)/max(1,len(scan_rows)),
                   'scan_samples':len(scan_rows),
                   'robot_tf_gazebo_samples':len(pose_rows),
                   'robot_tf_gazebo_xy_max_m':max((r['xy_error_m'] for r in pose_rows),default=None),
                   'robot_tf_gazebo_yaw_max_deg':max((math.degrees(r['yaw_error_rad']) for r in pose_rows),default=None),
                   'boundary':'Read-only H1 observation evidence; not robot avoidance acceptance'}
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps(summary,indent=2));node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    main()
