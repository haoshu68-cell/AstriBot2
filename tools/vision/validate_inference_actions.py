#!/usr/bin/env python3
"""Live RGB-D -> inference Action validation. No simulator truth is read.

The magenta instance mask is a declared validation fixture, not a YOLO result.
Use the isolated simulation domain/overlay from its session manifest.
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from rosidl_runtime_py.convert import message_to_ordereddict
from astribot_perception_msgs.action import ComputeGrasps, EstimateObjectPose
from sim_pose_capture import Capture, image_data, stamp, transform_dict
from tf2_ros import TransformException


def live_snapshot(node, timeout=15, include_tf=False):
    deadline = time.monotonic() + timeout
    tf_error = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
        common = set(node.msgs['rgb']) & set(node.msgs['depth']) & set(node.msgs['info'])
        for ts in sorted(common, reverse=True):
            health = node.health.get(ts)
            if not health or not health['valid']:
                continue
            age = (node.get_clock().now().nanoseconds - ts) / 1e9
            if not 0 <= age <= .2:
                continue
            rgb, dep, info = (node.msgs[k][ts] for k in ('rgb', 'depth', 'info'))
            if len({rgb.header.frame_id, dep.header.frame_id, info.header.frame_id, health['frame_id']}) != 1:
                continue
            if np.any(np.abs(info.d) > 1e-9):
                raise RuntimeError('Distorted images require rectification')
            if rgb.width != dep.width or rgb.height != dep.height or info.width != rgb.width or info.height != rgb.height:
                raise RuntimeError('RGB/depth/info dimensions mismatch')
            metadata = None
            if include_tf:
                # An explicit image stamp is essential: latest TF cannot support
                # independent accuracy scoring of a moving camera.
                try:
                    query_time = Time.from_msg(info.header.stamp)
                    odom_camera = node.buffer.lookup_transform('odom', info.header.frame_id, query_time)
                    odom_base = node.buffer.lookup_transform('odom', 'astribot_torso_base', query_time)
                except TransformException as error:
                    tf_error = str(error)
                    continue
                metadata = {
                    'source': 'ROS /tf and /tf_static, lookup at RGB-D capture stamp',
                    'requested_stamp_ns': ts, 'latest_tf_fallback': False,
                    'odom_from_camera': transform_dict(odom_camera),
                    'odom_from_robot_base': transform_dict(odom_base),
                    'robot_base_frame': 'astribot_torso_base',
                    'exact_sync_stamps': {key: stamp(msg) for key, msg in
                                          (('rgb', rgb), ('depth', dep), ('info', info))},
                    'camera_info': message_to_ordereddict(info),
                }
            color, depth = image_data(rgb), image_data(dep)
            hsv = cv2.cvtColor(color[:, :, :3], cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv, np.array([135, 80, 55]), np.array([175, 255, 255])) > 0
            mask &= np.isfinite(depth) & (depth > .15) & (depth < 4.5)
            mask = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
            v, u = np.nonzero(mask)
            k = np.array(info.k).reshape(3, 3)
            xyz = np.column_stack(((u-k[0, 2])*depth[v, u]/k[0, 0], (v-k[1, 2])*depth[v, u]/k[1, 1], depth[v, u])).astype('<f4')
            if len(xyz) < 64:
                continue
            cloud = PointCloud2()
            cloud.header = info.header
            cloud.width, cloud.height = len(xyz), 1
            cloud.point_step, cloud.row_step = 12, len(xyz)*12
            cloud.is_dense = True
            cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, name in enumerate('xyz')]
            cloud.data = xyz.tobytes()
            return (cloud, health, age, metadata) if include_tf else (cloud, health, age)
    raise RuntimeError('No fresh synchronized target snapshot and matching camera health/TF' +
                       (f'; last TF error: {tf_error}' if tf_error else ''))


def await_future(node, future, timeout):
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
    if not future.done():
        raise TimeoutError('Action response deadline exceeded')
    return future.result()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--kind', choices=('grasp', 'pose'), required=True)
    p.add_argument('--camera', default='torso_rgbd')
    p.add_argument('--model-id', default='asymmetric_union')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--repeat', type=int, default=3)
    p.add_argument('--timeout', type=float, default=15.0)
    args = p.parse_args()
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = Capture(args.camera)
    action = ComputeGrasps if args.kind == 'grasp' else EstimateObjectPose
    endpoint = '/perception/compute_grasps' if args.kind == 'grasp' else '/perception/estimate_object_pose'
    client = ActionClient(node, action, endpoint)
    if not client.wait_for_server(timeout_sec=10):
        raise RuntimeError('Action server unavailable in selected ROS domain')
    report = {'schema': 'astribot.live_inference_validation/2', 'source': 'live Gazebo raw RGB-D', 'truth_used_as_input': False,
              'pose_source': endpoint, 'model_id': args.model_id if args.kind == 'pose' else None,
              'segmentation': 'HSV magenta fixture, not YOLO', 'execution_permission': False, 'trials': []}
    try:
        for index in range(args.repeat):
            cloud, health, age, capture_tf = live_snapshot(node, include_tf=True)
            inputs = args.output.parent / (args.output.stem + '_inputs')
            inputs.mkdir(parents=True, exist_ok=True)
            cloud_path = inputs / f'{stamp(cloud)}.bin'
            cloud_path.write_bytes(bytes(cloud.data))
            goal = action.Goal()
            goal.header = cloud.header
            goal.task_id = f'live_{args.kind}_{index}_{stamp(cloud)}'
            goal.object_id = 'asymmetric_union'
            goal.camera_id, goal.source_epoch = args.camera, health['source_epoch']
            goal.calibration_revision = health['calibration_revision']
            # Fixed inference test context, never claims a live MoveIt scene/envelope.
            goal.planning_scene_revision, goal.envelope_epoch = 1, 1
            goal.object_cloud = cloud
            goal.timeout_sec = args.timeout
            goal.valid_until = (Time.from_msg(cloud.header.stamp) + Duration(seconds=5)).to_msg()
            if args.kind == 'grasp':
                goal.arm_id, goal.max_candidates = 'right', 32
            else:
                goal.model_id = args.model_id
            start = time.monotonic()
            handle = await_future(node, client.send_goal_async(goal), 3)
            trial = {'capture_stamp_ns': stamp(cloud), 'frame': cloud.header.frame_id, 'points': cloud.width,
                     'camera_id': args.camera, 'task_id': goal.task_id,
                     'capture_tf': capture_tf,
                     'input_cloud_file': str(cloud_path),
                     'capture_age_before_send_sec': age, 'source_epoch': health['source_epoch'], 'calibration_revision': health['calibration_revision'],
                     'camera_health_at_capture': health, 'accepted': handle.accepted}
            if handle.accepted:
                try:
                    result = await_future(node, handle.get_result_async(), args.timeout+5)
                except TimeoutError:
                    await_future(node, handle.cancel_goal_async(), 3)
                    raise
                trial.update(status=result.status, result=message_to_ordereddict(result.result))
            trial['wall_latency_sec'] = time.monotonic()-start
            report['trials'].append(trial)
            print(json.dumps({k: v for k, v in trial.items() if k not in ('result', 'camera_health_at_capture', 'capture_tf')}), flush=True)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
