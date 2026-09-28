#!/usr/bin/env python3
"""Synthetic depth -> actual installed C++ projector; no robot commands.

The cylinder is 20 mm in diameter and 160 mm long, perpendicular to the
optical axis. This checks loss during pixel decimation, not camera accuracy,
reflectance, occlusion, scene coverage or physical protective performance.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from astribot_perception_msgs.msg import ProjectionHealth
import yaml

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--binary', type=Path, required=True)
p.add_argument('--head-profile', type=Path, required=True)
p.add_argument('--torso-profile', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '25')
assert not a.output.exists()
a.output.mkdir(parents=True)
report = {'scope': __doc__, 'domain': os.environ['ROS_DOMAIN_ID'], 'cases': [],
          'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          'binary': str(a.binary.resolve()),
          'binary_sha256': hashlib.sha256(a.binary.read_bytes()).hexdigest(),
          'profiles': {}, 'completed': False}
report['transport_environment'] = {k: os.environ.get(k) for k in
    ('ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION', 'FASTRTPS_DEFAULT_PROFILES_FILE')}
profile = os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
if profile:
    report['transport_profile_sha256'] = hashlib.sha256(Path(profile).read_bytes()).hexdigest()
rclpy.init()
node = Node('arm_depth_decimation_observer')
children, streams, subscriptions, clouds, health = [], [], [], {}, {}
try:
    depth_pub = node.create_publisher(Image, '/arm_depth_probe/depth', qos_profile_sensor_data)
    info_pub = node.create_publisher(CameraInfo, '/arm_depth_probe/info', qos_profile_sensor_data)
    for stride in (4, 1):
        topic = f'/arm_depth_probe/points_{stride}'
        subscriptions.append(node.create_subscription(PointCloud2, topic,
            lambda msg, s=stride: clouds.__setitem__((s, msg.header.stamp.sec, msg.header.stamp.nanosec), msg),
            qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(ProjectionHealth, f'/arm_depth_probe/health_{stride}',
            lambda msg, s=stride: health.__setitem__(s, {'processed': msg.processed, 'errors': msg.errors,
                                                      'reason': msg.reason_code}), 10))
        command = [str(a.binary), '--ros-args', '-r', f'__node:=arm_depth_probe_{stride}',
                   '-p', 'use_sim_time:=false', '-p', 'depth_topic:=/arm_depth_probe/depth',
                   '-p', 'camera_info_topic:=/arm_depth_probe/info', '-p', f'output_topic:={topic}',
                   '-p', f'decimation:={stride}', '-p', 'projection_backend:=cpu',
                   '-p', f'processing_health_topic:=/arm_depth_probe/health_{stride}',
                   '-p', 'min_depth:=0.2', '-p', 'max_depth:=5.0']
        report.setdefault('commands', []).append(command)
        stream = (a.output / f'projector_{stride}.log').open('w')
        streams.append(stream)
        children.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT))
    deadline = time.monotonic() + 10
    while (depth_pub.get_subscription_count() != 2 or info_pub.get_subscription_count() != 2 or
           any(node.count_publishers(f'/arm_depth_probe/points_{s}') != 1 for s in (4, 1))):
        assert all(c.poll() is None for c in children), 'PROJECTOR_EXITED'
        assert time.monotonic() < deadline, 'PROJECTOR_DISCOVERY_TIMEOUT'
        rclpy.spin_once(node, timeout_sec=.05)
    report['loaded_libraries'] = {
        str(c.pid): sorted({line.split()[-1] for line in Path(f'/proc/{c.pid}/maps').read_text().splitlines()
                           if '.so' in line and line.split()[-1].startswith('/')}) for c in children}
    report['input_endpoints'] = {topic: [str(v) for v in node.get_subscriptions_info_by_topic(topic)]
                                 for topic in ('/arm_depth_probe/depth', '/arm_depth_probe/info')}
    for camera, profile, distances in (
            ('head', a.head_profile, (1., 2., 4.)),
            ('torso', a.torso_profile, (1., 2., 3.))):
        raw = profile.read_bytes()
        config = yaml.safe_load(raw)
        report['profiles'][camera] = {'path': str(profile.resolve()),
                                      'sha256': hashlib.sha256(raw).hexdigest()}
        width, height = config['width'], config['height']
        k = list(map(float, config['intrinsics']['depth']))
        v, u = np.mgrid[:height, :width]
        ray_x, ray_y = (u-k[2])/k[0], (v-k[5])/k[4]
        for distance in distances:
            for phase in (0, 2):
                # Cylinder axis parallel to optical y. Analytic ray/cylinder
                # intersection supplies independent input geometry.
                center_z = distance + .010
                center_x = (width // 2 + phase - k[2]) * center_z / k[0]
                aa = ray_x**2 + 1
                bb = ray_x * center_x + center_z
                cc = center_x**2 + center_z**2 - .010**2
                disc = bb**2 - aa*cc
                side_z = (bb - np.sqrt(np.maximum(disc, 0))) / aa
                hit = (disc >= 0) & (side_z > 0) & (np.abs(ray_y*side_z) <= .080)
                # Include the two flat ends of the finite cylinder too.
                near_z = np.where(hit, side_z, np.inf)
                for cap_y in (-.080, .080):
                    cap_z = np.divide(cap_y, ray_y, out=np.full_like(ray_y, np.inf), where=ray_y != 0)
                    cap_hit = (cap_z > 0) & ((ray_x*cap_z-center_x)**2 + (cap_z-center_z)**2 <= .010**2)
                    near_z = np.minimum(near_z, np.where(cap_hit, cap_z, np.inf))
                hit = np.isfinite(near_z)
                depth = np.where(hit, near_z, 5.).astype('<f4')
                assert np.count_nonzero(hit) > 0, 'SYNTHETIC_TARGET_HAS_NO_PIXELS'
                image = Image(height=height, width=width, encoding='32FC1', step=width*4,
                              is_bigendian=0, data=depth.tobytes())
                image.header.frame_id = 'arm_depth_probe_optical'
                info = CameraInfo(header=image.header, height=height, width=width, k=k)
                # These actual sensor subscriptions are best effort. Supply a
                # bounded stream of distinct captures, retaining every missing
                # output. Compare both projectors on the SAME capture only.
                clouds.clear()
                sent, stamp, next_capture = [], None, 0.
                deadline = time.monotonic() + 6
                while stamp is None:
                    assert all(c.poll() is None for c in children), 'PROJECTOR_EXITED'
                    assert time.monotonic() < deadline, f'OUTPUT_TIMEOUT {camera} {distance} {phase}; received={list(clouds)} health={health}'
                    if time.monotonic() >= next_capture:
                        image.header.stamp = node.get_clock().now().to_msg()
                        info.header = image.header
                        sent.append((image.header.stamp.sec, image.header.stamp.nanosec))
                        info_pub.publish(info)
                        depth_pub.publish(image)
                        next_capture = time.monotonic() + .2
                    rclpy.spin_once(node, timeout_sec=.02)
                    stamp = next((t for t in sent if all((s, *t) in clouds for s in (4, 1))), None)
                row = {'camera': camera, 'front_distance_m': distance, 'pixel_phase': phase,
                       'diameter_m': .020, 'length_m': .160,
                       'source_target_pixels': int(hit.sum()),
                       'matched_stamp': stamp,
                       'captures': [{'stamp': t, 'received_decimations': [s for s in (4, 1) if (s, *t) in clouds]}
                                    for t in sent],
                       'depth_sha256': hashlib.sha256(image.data).hexdigest(), 'outputs': {}}
                for stride in (4, 1):
                    cloud = clouds.pop((stride, *stamp))
                    assert cloud.header.frame_id == image.header.frame_id and not cloud.is_bigendian
                    offsets = {f.name: f.offset for f in cloud.fields}
                    zs = np.ndarray((cloud.height, cloud.width), '<f4', buffer=bytes(cloud.data),
                                    offset=offsets['z'], strides=(cloud.row_step, cloud.point_step))
                    count = int(np.count_nonzero(np.isfinite(zs) & (zs < 4.5)))
                    expected = int(hit[::stride, ::stride].sum())
                    assert count == expected, f'PROJECTION_MISMATCH {stride}: {count} != {expected}'
                    row['outputs'][str(stride)] = {'target_points': count, 'total_points': int(zs.size)}
                report['cases'].append(row)
    report['lost_with_decimation_4'] = sum(c['outputs']['4']['target_points'] == 0 for c in report['cases'])
    report['lost_with_decimation_1'] = sum(c['outputs']['1']['target_points'] == 0 for c in report['cases'])
    report['completed'] = True
finally:
    report['last_health'] = health
    for child in children:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
    report['child_exit_codes'] = [c.wait(timeout=10) for c in children]
    for stream in streams:
        stream.close()
    (a.output / 'result.json').write_text(json.dumps(report, indent=2)+'\n')
    node.destroy_node()
    rclpy.shutdown()
print(json.dumps({k: report[k] for k in ('completed', 'lost_with_decimation_4', 'lost_with_decimation_1')}))
