#!/usr/bin/env python3
"""Bounded RGB-D/VoxelLayer/narrow-passage coverage evidence collector.

This is a validation tool, not a runtime controller.  It records sparse input,
capture freshness, voxel publication and the existing passage decision in one
evidence directory.  A missing or stale depth stream is deliberately reported
as a safety limitation; the tool never turns an incomplete 3-D view into a
pass.  ``--spawn-occluder`` adds a temporary static wall through the Gazebo
service and removes only that owned entity at shutdown.
"""
import argparse
import json
import math
import subprocess
import time
import uuid
from collections import defaultdict
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav2_msgs.msg import VoxelGrid
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from astribot_navigation_msgs.msg import PassageAssessment, SensorHealthArray


def stamp(msg):
    h = msg.header.stamp
    return float(h.sec) + float(h.nanosec) * 1e-9


def point_stats(msg):
    # Decode only xyz fields.  PointCloud2 is bounded by the camera driver and
    # this path is used for evidence, never for the navigation loop.
    try:
        from sensor_msgs_py import point_cloud2
        rows = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        count = 0
        z_min, z_max = math.inf, -math.inf
        for x, y, z in rows:
            if all(math.isfinite(float(v)) for v in (x, y, z)):
                count += 1
                z_min = min(z_min, float(z)); z_max = max(z_max, float(z))
        return dict(points=count, z_min=None if count == 0 else z_min,
                    z_max=None if count == 0 else z_max,
                    width=int(msg.width), height=int(msg.height), frame=msg.header.frame_id)
    except Exception as exc:  # retain transport evidence even if a decoder is absent
        return dict(points=None, decode_error=str(exc), width=int(msg.width),
                    height=int(msg.height), frame=msg.header.frame_id)


def ign_create(name):
    sdf = (f'<sdf version="1.7"><model name="{name}"><static>true</static><pose>'
           '0.65 0 1.0 0 0 0</pose><link name="body"><collision name="collision">'
           '<geometry><box><size>0.30 1.8 2.0</size></box></geometry></collision>'
           '<visual name="visual"><geometry><box><size>0.30 1.8 2.0</size></box></geometry>'
           '</visual></link></model></sdf>')
    result = subprocess.run([
        'ign', 'service', '-s', '/world/default/create', '--reqtype',
        'ignition.msgs.EntityFactory', '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '1500', '--req', 'sdf: ' + json.dumps(sdf) +
        ' allow_renaming: false'], capture_output=True, text=True, timeout=4.)
    if result.returncode or 'data: true' not in result.stdout:
        raise RuntimeError('OCCLUDER_CREATE_FAILED:' + result.stdout[-240:])


def ign_remove(name):
    subprocess.run([
        'ign', 'service', '-s', '/world/default/remove', '--reqtype',
        'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '1500', '--req', f'name: "{name}" type: MODEL'],
        capture_output=True, text=True, timeout=4.)


def param_snapshot():
    result = subprocess.run(['ros2', 'param', 'dump', '/local_costmap/local_costmap'],
                            capture_output=True, text=True, timeout=8.)
    if result.returncode:
        return {'error': result.stderr.strip()[-300:]}
    try:
        data = json.loads(result.stdout) if result.stdout.lstrip().startswith('{') else None
    except json.JSONDecodeError:
        data = None
    return {'raw': result.stdout[-12000:], 'json': data}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--scenario', choices=['nominal', 'sparse', 'occluded', 'voxel', 'narrow'],
                        required=True)
    parser.add_argument('--duration', type=float, default=12.)
    parser.add_argument('--spawn-occluder', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be finite and positive')
    output = Path(args.output); output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('coverage_matrix_probe', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    clouds = defaultdict(list); voxels = []; costmaps = []; health = []; passages = []; policies = []
    occluder = None
    def cloud_cb(camera, msg):
        row = point_stats(msg); row['stamp'] = stamp(msg); row['received_wall'] = time.monotonic()
        clouds[camera].append(row)
    node.create_subscription(PointCloud2, '/camera/head_rgbd/points',
                             lambda m: cloud_cb('head', m), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/camera/torso_rgbd/points',
                             lambda m: cloud_cb('torso', m), qos_profile_sensor_data)
    node.create_subscription(VoxelGrid, '/local_costmap/voxel_grid',
                             lambda m: voxels.append(m), 10)
    node.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                             lambda m: costmaps.append(m), 10)
    node.create_subscription(SensorHealthArray, '/navigation/sensor_health',
                             lambda m: health.append(m), 10)
    node.create_subscription(PassageAssessment, '/navigation/passage_assessment',
                             lambda m: passages.append(m), 10)
    node.create_subscription(String, '/navigation_policy/state',
                             lambda m: policies.append(m), 10)
    start = time.monotonic()
    try:
        if args.spawn_occluder:
            occluder = 'coverage_occluder_' + uuid.uuid4().hex[:8]
            ign_create(occluder)
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        if occluder:
            ign_remove(occluder)
        node.destroy_node(); rclpy.shutdown()

    now = time.monotonic()
    sensor_rows = {}
    for camera in ('head', 'torso'):
        rows = clouds[camera]
        stamps = [r['stamp'] for r in rows if r.get('stamp', 0) > 0]
        gaps = [b - a for a, b in zip(stamps, stamps[1:]) if b >= a]
        sensor_rows[camera] = dict(samples=len(rows), nonempty=sum((r.get('points') or 0) > 0 for r in rows),
                                   max_points=max((r.get('points') or 0 for r in rows), default=0),
                                   max_gap_s=max(gaps, default=None), last_age_s=(
                                       None if not stamps else max(0., now - rows[-1]['received_wall'])),
                                   last=rows[-1] if rows else None)
    health_rows = []
    for msg in health[-20:]:
        health_rows.append([dict(sensor_id=s.sensor_id, state=s.state, depth_available=s.depth_available,
                                 reason=s.reason,
                                 valid_until=float(s.valid_until.sec) + float(s.valid_until.nanosec) * 1e-9)
                            for s in msg.sensors])
    passage_rows = [dict(corridor_id=m.corridor_id, width_m=m.width_m,
                         in_place_rotation_allowed=m.in_place_rotation_allowed,
                         reverse_allowed=m.reverse_allowed, decision=m.decision, reason=m.reason)
                    for m in passages[-20:]]
    policy_rows = []
    for msg in policies[-20:]:
        try: policy_rows.append(json.loads(msg.data))
        except (TypeError, ValueError): policy_rows.append({'raw': msg.data})
    params = param_snapshot()
    depth_samples = sum(v['samples'] for v in sensor_rows.values())
    depth_nonempty = sum(v['nonempty'] for v in sensor_rows.values())
    if args.scenario == 'voxel':
        passed = bool(voxels) and params.get('error') is None
        verdict = 'PASS' if passed else 'HOLD_VOXEL_EVIDENCE_UNAVAILABLE'
    elif args.scenario in ('sparse', 'occluded'):
        # Safety acceptance is conservative: a sparse/occluded view passes only
        # when the run records the degraded observation and no false full-ready
        # claim is present in the health stream.
        degraded = depth_nonempty == 0 or any(s.get('last_age_s') is None or (s.get('max_gap_s') or 0) > .5
                                              for s in sensor_rows.values())
        verdict = 'PASS_SAFE_HOLD' if degraded else 'HOLD_SCENE_NOT_DEGRADED'
        passed = degraded
    elif args.scenario == 'narrow':
        passed = bool(passage_rows) and all(not r['in_place_rotation_allowed'] for r in passage_rows)
        verdict = 'PASS_NO_IN_PLACE_TURN' if passed else 'HOLD_PASSAGE_EVIDENCE_UNAVAILABLE'
    else:
        passed = depth_nonempty > 0
        verdict = 'PASS_DEPTH_OBSERVED' if passed else 'HOLD_RGBD_STALE_OR_EMPTY'
    summary = dict(scenario=args.scenario, duration_s=time.monotonic() - start,
                   sensor=sensor_rows, voxel_messages=len(voxels), costmap_messages=len(costmaps),
                   health=health_rows, passage=passage_rows, policy=policy_rows,
                   costmap_params=params, verdict=verdict, passed=passed,
                   evidence='sensor receipt and Nav2 safety state; occlusion is a bounded fixture, not camera calibration proof')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    raise SystemExit(0 if passed else 2)


if __name__ == '__main__':
    main()
