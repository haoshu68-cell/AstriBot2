"""Apply robot camera calibration to simulated images and CameraInfo.

Gazebo remains the scene renderer. This node applies the calibrated Brown
model in pixel space and republishes the exact robot K/D contract.
"""
from pathlib import Path
import yaml
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraCalibrationPostprocess(Node):
    def __init__(self):
        super().__init__('camera_calibration_postprocess')
        profile = Path(self.declare_parameter('profile', '').value)
        cfg = yaml.safe_load(profile.read_text())
        intr = cfg['intrinsics']['color']
        self.width, self.height = int(cfg['width']), int(cfg['height'])
        self.k = np.asarray(intr, dtype=np.float64).reshape(3, 3)
        # Profiles use ROS/OpenCV order: k1, k2, p1, p2, k3.
        self.d = np.asarray(cfg.get('distortion', [0] * 5), dtype=np.float64)
        if np.allclose(self.d, 1.0):
            self.d[:] = 0.0  # firmware sentinel: calibration unavailable
        self.map_x, self.map_y = cv2.initUndistortRectifyMap(
            self.k, self.d, None, self.k, (self.width, self.height), cv2.CV_32FC1)
        self.bridge = CvBridge()
        self.info_pub = self.create_publisher(
            CameraInfo, self.declare_parameter('output_info', '').value, 10)
        self.image_pub = self.create_publisher(
            Image, self.declare_parameter('output_image', '').value, 10)
        self.depth_pub = None
        depth_out = self.declare_parameter('output_depth', '').value
        if depth_out:
            self.depth_pub = self.create_publisher(Image, depth_out, 10)
        self.create_subscription(CameraInfo, self.declare_parameter('input_info', '').value,
                                 self.info, qos_profile_sensor_data)
        self.create_subscription(Image, self.declare_parameter('input_image', '').value,
                                 self.image, qos_profile_sensor_data)
        depth_in = self.declare_parameter('input_depth', '').value
        if depth_in:
            self.create_subscription(Image, depth_in, self.depth, qos_profile_sensor_data)
        self.get_logger().info(f'loaded {profile} ({self.width}x{self.height})')

    def info(self, msg):
        out = msg
        out.width, out.height = self.width, self.height
        out.k = self.k.reshape(-1).tolist()
        out.d = self.d.tolist()
        out.distortion_model = 'plumb_bob'
        self.info_pub.publish(out)

    def _remap(self, msg, interpolation):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        result = cv2.remap(image, self.map_x, self.map_y, interpolation,
                           borderMode=cv2.BORDER_CONSTANT)
        return self.bridge.cv2_to_imgmsg(result, encoding=msg.encoding)

    def image(self, msg):
        out = self._remap(msg, cv2.INTER_LINEAR)
        out.header = msg.header
        self.image_pub.publish(out)

    def depth(self, msg):
        out = self._remap(msg, cv2.INTER_NEAREST)
        out.header = msg.header
        self.depth_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationPostprocess()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
