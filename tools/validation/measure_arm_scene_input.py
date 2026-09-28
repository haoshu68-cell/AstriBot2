#!/usr/bin/env python3
"""Read-only capture of the real arm scene input; never submits robot motion."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from astribot_transport_msgs.msg import ObservedOctomap
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=20.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('arm_scene_input_measurement',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    result = {'started_wall': time.time(), 'duration_requested': args.seconds,
              'samples': {}, 'parameters': {}, 'transforms': {}}
    last = {}

    def record(topic, msg):
        sample = {'receive_monotonic': time.monotonic(),
                  'receive_ros': node.get_clock().now().nanoseconds * 1e-9}
        if hasattr(msg, 'header'):
            sample.update(stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                          frame=msg.header.frame_id)
        if isinstance(msg, PointCloud2):
            sample.update(width=msg.width, height=msg.height, point_step=msg.point_step)
        elif isinstance(msg, ObservedOctomap):
            sample.update(source=msg.source_id, epoch=msg.map_epoch, revision=msg.map_revision,
                          callback=msg.callback_stamp.sec + msg.callback_stamp.nanosec * 1e-9,
                          integrated=msg.integrated_stamp.sec + msg.integrated_stamp.nanosec * 1e-9,
                          processing_seconds=msg.processing_seconds, bytes=len(msg.octomap.data),
                          free_keys=len(msg.ray_free_keys)//3)
            topic += '/' + msg.source_id
        elif isinstance(msg, PoseWithCovarianceStamped):
            sample['pose'] = message_to_ordereddict(msg.pose.pose)
        elif isinstance(msg, JointState):
            sample.update(names=list(msg.name), positions=list(msg.position), velocities=list(msg.velocity))
        elif isinstance(msg, Clock):
            sample['clock'] = msg.clock.sec + msg.clock.nanosec * 1e-9
        result['samples'].setdefault(topic, []).append(sample)
        last[topic] = msg

    topics = [('/clock', Clock), ('/slam/pose', PoseWithCovarianceStamped),
              ('/joint_states', JointState), ('/moveit/observed_octomap', ObservedOctomap)]
    for camera in ('head_rgbd', 'torso_rgbd'):
        topics += [(f'/camera/raw/{camera}/depth_image', Image),
                   (f'/camera/raw/{camera}/camera_info', CameraInfo),
                   (f'/camera/{camera}/points_raw', PointCloud2),
                   (f'/camera/{camera}/points', PointCloud2)]
    subs = [node.create_subscription(kind, topic, lambda msg, t=topic: record(t, msg),
                                    10 if kind is ObservedOctomap else qos_profile_sensor_data)
            for topic, kind in topics]
    names = {'head_rgbd_pointcloud': ['decimation', 'use_sim_time'],
             'torso_rgbd_pointcloud': ['decimation', 'use_sim_time'],
             'omni_effort_drive_node': ['idle_position_hold', 'idle_position_kp'],
             'move_group': ['allow_trajectory_execution', 'use_sim_time',
                            'head_rgbd.point_subsample', 'torso_rgbd.point_subsample']}
    clients = []
    futures = []
    for name, params in names.items():
        client = node.create_client(GetParameters, '/' + name + '/get_parameters')
        clients.append(client)
        futures.append((name, params, client.call_async(GetParameters.Request(names=params))))
    until = time.monotonic() + args.seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.05)
    for name, params, future in futures:
        result['parameters'][name] = (dict(zip(params, [message_to_ordereddict(v)
            for v in future.result().values])) if future.done() else {'error': 'RESPONSE_TIMEOUT'})
    for camera in ('head_rgbd', 'torso_rgbd'):
        frame = camera + '_camera_optical_frame'
        if buffer.can_transform('astribot_torso_base', frame, rclpy.time.Time()):
            result['transforms'][frame] = message_to_ordereddict(
                buffer.lookup_transform('astribot_torso_base', frame, rclpy.time.Time()))
        else:
            result['transforms'][frame] = {'error': 'TF_UNAVAILABLE'}
    result['last'] = {}
    for topic, msg in last.items():
        filename = topic.strip('/').replace('/', '_')
        if isinstance(msg, Image):
            assert msg.encoding == '32FC1' and not msg.is_bigendian
            np.save(args.output/(filename+'.npy'), np.frombuffer(msg.data, dtype='<f4').reshape(msg.height, msg.step//4))
            result['last'][topic] = {'width': msg.width, 'height': msg.height, 'step': msg.step,
                                    'header': message_to_ordereddict(msg.header)}
        elif isinstance(msg, PointCloud2):
            (args.output/(filename+'.bin')).write_bytes(bytes(msg.data))
            msg.data = []
            result['last'][topic] = message_to_ordereddict(msg)
        elif isinstance(msg, ObservedOctomap):
            (args.output/(filename+'.octree')).write_bytes(msg.octomap.data.tobytes())
            msg.octomap.data = []
            msg.ray_free_keys = []
            result['last'][topic] = message_to_ordereddict(msg)
        elif isinstance(msg, CameraInfo):
            result['last'][topic] = message_to_ordereddict(msg)
    result['ended_wall'] = time.time()
    (args.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: len(v) for k, v in result['samples'].items()}, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
