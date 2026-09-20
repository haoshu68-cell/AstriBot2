#!/usr/bin/env python3
"""Read-only camera audit. No SDK object, activation service, or motion publisher.

Run on the robot in its existing ROS environment; emits JSON to stdout.
CameraInfo provides intrinsics, not robot-to-camera extrinsics. TF transforms are
reported with parent/child and timestamps, never guessed from topic names.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=10.)
    args = parser.parse_args()
    if not 1 <= args.duration <= 60:
        parser.error('duration must be 1..60 seconds')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image, CompressedImage
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener
    from rclpy.time import Time
    from rosidl_runtime_py.convert import message_to_ordereddict
    result = dict(schema_version=1, captured_unix=time.time(), hostname=os.uname().nodename,
                  ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0'), camera_info={}, images={},
                  tf_static=[], extrinsics=[], errors=[])
    try:
        usb = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=5.)
        result['usb_inventory'] = usb.stdout.splitlines()
    except (OSError, subprocess.TimeoutExpired) as error:
        result['errors'].append(str(error))
    result['video_devices'] = {str(p.parent): p.read_text().strip()
                               for p in Path('/sys/class/video4linux').glob('*/name')}
    rclpy.init()
    node = rclpy.create_node('read_only_camera_audit')
    buffer = Buffer(); listener = TransformListener(buffer, node)
    subscriptions = []
    known = set()
    static_edges = {}
    def static(message):
        for transform in message.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            static_edges[key] = message_to_ordereddict(transform)
    subscriptions.append(node.create_subscription(TFMessage, '/tf_static', static,
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    def info(message, topic):
        result['camera_info'][topic] = message_to_ordereddict(message)
    def image(message, topic):
        entry = result['images'].setdefault(topic, dict(samples=0))
        entry.update(samples=entry['samples']+1, frame_id=message.header.frame_id,
                     stamp=message_to_ordereddict(message.header.stamp), bytes=len(message.data))
        for field in ('width', 'height', 'encoding', 'format'):
            if hasattr(message, field):entry[field] = getattr(message, field)
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        for topic, types in node.get_topic_names_and_types():
            if topic in known:
                continue
            kind = None
            if 'sensor_msgs/msg/CameraInfo' in types:
                kind, callback = CameraInfo, info
            elif any(word in topic.lower() for word in ('camera', 'image', 'rgbd', 'stereo')):
                if 'sensor_msgs/msg/Image' in types:kind, callback = Image, image
                elif 'sensor_msgs/msg/CompressedImage' in types:kind, callback = CompressedImage, image
            if kind:
                subscriptions.append(node.create_subscription(kind, topic,
                    lambda msg, t=topic, cb=callback: cb(msg, t), qos_profile_sensor_data))
                known.add(topic)
        rclpy.spin_once(node, timeout_sec=.1)
    result['topics'] = [dict(name=name, types=types) for name, types in node.get_topic_names_and_types()
                        if any(w in name.lower() for w in ('camera', 'image', 'rgbd', 'stereo'))]
    result['camera_nodes'] = [dict(name=name, namespace=namespace)
                              for name, namespace in node.get_node_names_and_namespaces()
                              if any(w in name.lower() for w in ('camera', 'realsense', 'orbbec', 'zed'))]
    result['tf_static'] = list(static_edges.values())
    frames = {info['header']['frame_id'] for info in result['camera_info'].values()}
    frames.update(image['frame_id'] for image in result['images'].values())
    for frame in sorted(frames):
        for parent in ('astribot_head_link_2', 'astribot_arm_left_link_7',
                       'astribot_arm_right_link_7', 'astribot_torso_base'):
            try:
                transform = buffer.lookup_transform(parent, frame, Time())
                result['extrinsics'].append(message_to_ordereddict(transform))
            except Exception as error:
                result['errors'].append(parent + ' <- ' + frame + ': ' + str(error))
    result['duration_wall_s'] = time.monotonic() - start
    result['status'] = 'OBSERVED' if result['camera_info'] else 'NO_CAMERA_INFO_OBSERVED'
    result['limitations'] = ['Model must be confirmed from device/driver metadata.',
                            'A dynamic base-to-camera TF is a pose sample, not a fixed mounting calibration.',
                            'No camera activation or robot motion command was issued.']
    node.destroy_node(); rclpy.shutdown()
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
