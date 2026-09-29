#!/usr/bin/env python3
"""Read-only six-camera imagery, depth, TF, point-cloud and health evidence."""
import argparse
import signal
from rclpy.signals import SignalHandlerOptions
import hashlib
from collections import Counter, defaultdict
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from tf2_ros import Buffer, TransformListener
from astribot_perception_msgs.msg import CameraHealth
from rosgraph_msgs.msg import Clock
from rcl_interfaces.srv import GetParameters
from sim_pose_capture import image_data, stamp, transform_dict
from clock_receive_statistics import ClockReceiveStatistics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--scenario', default='stationary_camera_only')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 1800:
        raise ValueError('duration must be in [1,1800] seconds')
    args.output.mkdir(parents=True, exist_ok=True)
    ids = ('head_rgbd', 'head_stereo_left', 'head_stereo_right', 'torso_rgbd',
           'left_wrist_rgbd', 'right_wrist_rgbd')
    interrupted = False
    def stop(_signal, _frame):
        nonlocal interrupted
        interrupted = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('camera_reference_readonly_capture')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    description_client = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
    if not description_client.wait_for_service(timeout_sec=3):
        raise RuntimeError('No robot_state_publisher parameter service in this session')
    description_future = description_client.call_async(GetParameters.Request(names=['robot_description']))
    messages = defaultdict(dict)
    counts, stamps, health, clouds = Counter(), defaultdict(list), defaultdict(list), defaultdict(list)
    receive_ages = defaultdict(list)
    receive_wall_times = defaultdict(list)
    subscriptions, clocks = [], ClockReceiveStatistics()
    def on_image(camera, kind):
        def callback(message):
            ts = stamp(message)
            counts[(camera, kind)] += 1
            receive_wall_times[(camera, kind)].append(time.monotonic())
            stamps[(camera, kind)].append(ts)
            receive_ages[(camera, kind)].append((node.get_clock().now().nanoseconds-ts)*1e-9)
            cache = messages[(camera, kind)]
            cache[ts] = message
            while len(cache) > 8:
                del cache[min(cache)]
        return callback
    def on_health(camera):
        def callback(message):
            health[camera].append(dict(valid=message.valid, state=message.state,
                                       source_epoch=message.source_epoch,
                                       age_sec=message.age_sec, frequency_hz=message.frequency_hz,
                                       max_interval_sec=message.max_interval_sec, reason=message.reason_code,
                                       wall_elapsed=time.monotonic()-start,
                                       calibration_revision=message.calibration_revision,
                                       frame_id=message.frame_id, header_stamp_ns=stamp(message),
                                       capture_stamp_ns=message.capture_stamp.sec*10**9 +
                                                        message.capture_stamp.nanosec))
        return callback
    for camera in ids:
        stereo = 'stereo' in camera
        kinds = [('rgb', 'image_raw' if stereo else 'image', Image),
                 ('info', 'camera_info', CameraInfo)]
        if not stereo:
            kinds.append(('depth', 'depth_image', Image))
        for kind, suffix, typ in kinds:
            subscriptions.append(node.create_subscription(typ, f'/camera/raw/{camera}/{suffix}',
                                                           on_image(camera, kind), qos_profile_sensor_data))
        if not stereo:
            subscriptions.append(node.create_subscription(CameraHealth, f'/perception/camera_health/{camera}',
                                                          on_health(camera), 10))
            cloud_topic = f'/manipulation/camera/{camera}/points' if 'wrist' in camera else f'/camera/{camera}/points'
            subscriptions.append(node.create_subscription(PointCloud2, cloud_topic,
                                 lambda msg, c=camera: clouds[c].append(dict(stamp_ns=stamp(msg),
                                     points=msg.width*msg.height, frame=msg.header.frame_id)), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(Clock, '/clock',
        lambda msg: clocks.observe(msg.clock.sec*10**9+msg.clock.nanosec, time.monotonic()), qos_profile_sensor_data))
    start = time.monotonic()
    capture_started_wall = time.time()
    while not interrupted and time.monotonic()-start < args.seconds:
        rclpy.spin_once(node, timeout_sec=.02)
    clock_timing = clocks.summary(at_monotonic=time.monotonic())
    report = {'scope': 'rendered camera data in canonical warehouse', 'scenario': args.scenario,
              'ROS_DOMAIN_ID': os.getenv('ROS_DOMAIN_ID'), 'IGN_PARTITION': os.getenv('IGN_PARTITION'),
              'duration_wall_sec': time.monotonic()-start, 'requested_duration_sec': args.seconds,
              'capture_started_wall': capture_started_wall,
              'capture_started_monotonic': start,
              'interrupted': interrupted, 'capture_completed': not interrupted, 'clock_samples': clocks.samples,
              'clock_advanced_sec': clock_timing['advanced_sec'],
              'clock_receive_timing': clock_timing,
              'cameras': {}}
    # Save raw evidence before optional image/TF exports, including interrupted runs.
    (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    (args.output/'health_events.json').write_text(json.dumps(dict(health), indent=2)+'\n')
    def statistics(values):
        if not values: return {'samples': 0}
        return dict(samples=len(values), min=float(min(values)), max=float(max(values)),
            p50=float(np.percentile(values,50)), p95=float(np.percentile(values,95)),
            p99=float(np.percentile(values,99)))
    streams = {}
    for (camera,kind), values in stamps.items():
        intervals=np.diff(values)*1e-9
        streams[camera+'/'+kind] = dict(
            capture_to_observer_ros_age_sec=statistics(receive_ages[camera,kind]),
            capture_interval_sec=statistics(list(intervals)),
            receive_interval_wall_sec=statistics(list(np.diff(receive_wall_times[camera,kind]))),
            receive_gaps_over_250ms=int(np.sum(np.diff(receive_wall_times[camera,kind])>.25)),
            nonpositive_intervals=int(np.sum(intervals<=0)))
    (args.output/'stream_timing.json').write_text(json.dumps(dict(
        scope='Observer receive timing; not internal bridge/TF/inference stage latency or queue occupancy',
        scenario=args.scenario,streams=streams),indent=2))
    try:
        if not description_future.done():
            raise RuntimeError('Could not read the actual live robot model')
        description = description_future.result().values[0].string_value
        if not description.strip().startswith('<?xml'):
            raise RuntimeError('Live robot_description is empty or invalid')
        (args.output/'live_robot_description.urdf').write_text(description)
        report['live_robot_description_sha256'] = hashlib.sha256(description.encode()).hexdigest()
        for camera in ids:
            kinds = ['rgb', 'info'] + ([] if 'stereo' in camera else ['depth'])
            common = set.intersection(*(set(messages[(camera, kind)]) for kind in kinds))
            row = {'counts': {kind: counts[(camera, kind)] for kind in kinds},
                   'exact_stamp_groups': len(set.intersection(*(set(stamps[(camera, kind)]) for kind in kinds))),
                   'health_states': dict(Counter(h['state'] for h in health[camera])),
                   'cloud_messages': len(clouds[camera])}
            if not common:
                raise RuntimeError(f'No synchronized frame: {camera}')
            ts = max(common)
            rgb, info = (messages[(camera, kind)][ts] for kind in ('rgb', 'info'))
            assert rgb.header.frame_id == info.header.frame_id
            color = image_data(rgb)
            tf = buffer.lookup_transform('astribot_torso_base', rgb.header.frame_id, Time.from_msg(rgb.header.stamp))
            row.update(capture_stamp_ns=ts, frame=rgb.header.frame_id, width=rgb.width,
                       height=rgb.height, rgb_std=float(color.std()), base_from_optical=transform_dict(tf),
                       intrinsics=list(info.k))
            cv2.imwrite(str(args.output/(camera+'_rgb.png')), cv2.cvtColor(color[:,:,:3], cv2.COLOR_RGB2BGR))
            if 'depth' in kinds:
                msg = messages[(camera, 'depth')][ts]
                assert msg.header.frame_id == rgb.header.frame_id
                depth = image_data(msg)
                finite = np.isfinite(depth) & (depth > 0)
                y, x = depth.shape[0]//2, depth.shape[1]//2
                center = finite[y-5:y+6,x-5:x+6]
                row.update(finite_depth_ratio=float(finite.mean()), central_finite_depth_ratio=float(center.mean()),
                           depth_min_m=float(depth[finite].min()) if finite.any() else None,
                           depth_max_m=float(depth[finite].max()) if finite.any() else None)
                np.savez_compressed(args.output/(camera+'_depth.npz'), depth=depth)
            if health[camera]:
                row['latest_health'] = health[camera][-1]
            if clouds[camera]:
                row['latest_cloud'] = clouds[camera][-1]
            report['cameras'][camera] = row
        (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
        (args.output/'health_events.json').write_text(json.dumps(dict(health), indent=2)+'\n')
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if interrupted:
        raise SystemExit(130)


if __name__ == '__main__':
    main()
