"""RGB-D observation adapter for the controlled simulation transport scenario."""
import hashlib
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
from .rgbd import localize_orange_box


def ns(stamp):
    return stamp.sec*10**9 + stamp.nanosec


def decode(message):
    encoding = message.encoding.upper()
    if encoding in ('RGB8', 'BGR8'):
        image = np.ndarray((message.height, message.width, 3), np.uint8,
                           buffer=bytes(message.data), strides=(message.step, 3, 1)).copy()
        return image if encoding == 'RGB8' else image[:, :, ::-1]
    if encoding == '32FC1':
        dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
        return np.ndarray((message.height, message.width), dtype, buffer=bytes(message.data),
                          strides=(message.step, 4)).astype(float)
    if encoding == '16UC1':
        dtype = np.dtype('>u2' if message.is_bigendian else '<u2')
        return np.ndarray((message.height, message.width), dtype, buffer=bytes(message.data),
                          strides=(message.step, 2)).astype(float) * .001
    raise ValueError('UNSUPPORTED_IMAGE_ENCODING:' + message.encoding)


class CameraObserver(Node):
    def __init__(self):
        super().__init__('transport_rgbd_observer')
        self.declare_parameter('scenario', '')
        self.declare_parameter('camera_profile', '')
        self.c = json.loads(Path(self.get_parameter('scenario').value).read_text())
        self.profile_hash = hashlib.sha256(Path(self.get_parameter('camera_profile').value).read_bytes()).hexdigest()
        self.depths = {}
        self.colors = {}
        self.info = None
        self.last_stamp = -1
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.output = self.create_publisher(String, '/transport/object_observation', 10)
        self.health = self.create_publisher(String, '/transport/camera_health', 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.color, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', lambda m: setattr(self, 'info', m), qos_profile_sensor_data)
        # Images can arrive before the matching dynamic TF. Retry buffered pairs
        # without blocking the executor that must receive that TF.
        self.create_timer(.02, self.process)

    def depth(self, message):
        self.depths[ns(message.header.stamp)] = message
        self.depths = dict(sorted(self.depths.items())[-8:])
        self.process()

    def color(self, message):
        self.colors[ns(message.header.stamp)] = message
        self.colors = dict(sorted(self.colors.items())[-8:])
        self.process()

    def process(self):
        shared = [s for s in self.depths.keys() & self.colors.keys() if s > self.last_stamp]
        if not shared or self.info is None:
            return
        available = [s for s in shared if self.tf.can_transform(
            self.c['map_frame'], self.colors[s].header.frame_id, Time(nanoseconds=s))]
        if available:
            stamp = max(available)
        elif self.get_clock().now().nanoseconds - min(shared) > 500000000:
            stamp = min(shared)  # Surface a stale observation instead of waiting forever.
        else:
            return
        self.last_stamp = stamp
        rgb, depth, info = self.colors[stamp], self.depths[stamp], self.info
        try:
            now = self.get_clock().now().nanoseconds
            if not -10000000 <= now-stamp <= 500000000:
                raise ValueError('CAMERA_STALE')
            if (rgb.header.frame_id != depth.header.frame_id or rgb.header.frame_id != info.header.frame_id or
                    info.width != rgb.width or info.height != rgb.height):
                raise ValueError('CAMERA_FRAME_OR_RESOLUTION_MISMATCH')
            if abs(ns(info.header.stamp)-stamp) > 100000000:
                raise ValueError('CAMERA_INFO_STALE')
            if any(abs(v) > 1e-9 for v in info.d):
                raise ValueError('RECTIFIED_IMAGE_REQUIRED')
            tf = self.tf.lookup_transform(self.c['map_frame'], rgb.header.frame_id, Time(nanoseconds=stamp)).transform
            q, p = tf.rotation, tf.translation
            transform = np.eye(4)
            transform[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            transform[:3, 3] = [p.x, p.y, p.z]
            observation = localize_orange_box(decode(rgb), decode(depth), info.k, transform,
                                              self.c['size_xyz'], search_center=self.c['pick_xyz'])
            packet = dict(schema_version=1, object_id=self.c['object_id'], sensor_id='sim_head_rgbd',
                          stamp_ns=stamp, frame_id=self.c['map_frame'], calibration_epoch=self.profile_hash,
                          source='rgbd_color_template', **observation)
            self.output.publish(String(data=json.dumps(packet)))
            reason = 'OBJECT_OBSERVED'
        except Exception as error:
            reason = str(error)
        self.health.publish(String(data=json.dumps(dict(stamp_ns=stamp, reason=reason,
            calibration_status='designed_simulation_only', calibration_epoch=self.profile_hash))))


def main():
    rclpy.init(); node = CameraObserver()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
