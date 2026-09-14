#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：双 Livox Mid-360（左/右）点云时间同步 + 坐标变换融合节点。

数据链路位置：livox_preprocess_node(左右各一) --> 本节点 --> pointcloud_to_laserscan --> SLAM Toolbox

功能（对应任务书"多激光时间同步""多激光点云融合"要求）：
  1. 时间同步：用 message_filters.ApproximateTimeSynchronizer 对齐左右两路点云
     （允许 slop 参数配置的时间差，Mid-360 仿真/真实驱动两路各自独立采集，时间戳不会
     完全一致，必须近似同步，不允许把两路时间戳差异很大的点云直接拼在一起送 SLAM）。
  2. 坐标变换：用 tf2 把两路点云分别从各自的 sensor frame(livox_mid360_left/right)
     变换到统一的 target_frame（默认 astribot_torso_base，与 SLAM Toolbox 的
     base_frame 保持一致），变换以各自点云的实际采集时刻(header.stamp)为准。
  3. 融合：变换后的两路点云直接拼接(concatenate)成一份 PointCloud2 发布。

去畸变说明（如实标注，不回避，见任务书"禁止忽略时间同步问题"要求）：
  - 仿真分支：gz-sim 的 gpu_lidar 传感器输出的点云没有逐点时间戳字段，每一帧点云
    只有一个整体 header.stamp，物理上没法做"逐点插值去畸变"，本节点对仿真数据只做
    "整帧到达时刻"的一次性 tf2 变换对齐，这是仓储机器人室内低速场景下的合理简化，
    不是完整的运动畸变补偿方案。
  - 硬件分支：若 livox_ros_driver2 切到 xfer_format=1(CustomMsg)，消息里每个点都带
    相对于帧起始时刻的时间偏移(point.offset_time)，可以结合 /odom 做逐点插值去畸变。
    当前未实现逐点去畸变；deskew_enable:=true 会明确拒绝启动，避免静默空操作。
"""

from copy import deepcopy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import message_filters
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class LivoxFusionNode(Node):

    def __init__(self):
        super().__init__('livox_fusion_node')

        self.declare_parameter('left_topic', 'left/cloud_filtered')
        self.declare_parameter('right_topic', 'right/cloud_filtered')
        self.declare_parameter('output_topic', 'fused_points')
        self.declare_parameter('target_frame', 'astribot_torso_base')
        self.declare_parameter('sync_slop_sec', 0.05)
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('sync_queue_size', 2)
        self.declare_parameter('max_output_age_sec', 0.25)
        self.declare_parameter('missing_frame_threshold', 3)
        self.declare_parameter('deskew_enable', False)
        self.declare_parameter('future_tolerance_sec', 0.05)
        if self.get_parameter('deskew_enable').value:
            raise ValueError('deskew_enable is unsupported: per-point timestamps and motion compensation are required')

        self.add_on_set_parameters_callback(self._validate_parameters)

        self.target_frame = self.get_parameter('target_frame').value
        self.tf_timeout = Duration(
            seconds=self.get_parameter('tf_timeout_sec').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        left_topic = self.get_parameter('left_topic').value
        right_topic = self.get_parameter('right_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.left_sub = message_filters.Subscriber(self, PointCloud2, left_topic)
        self.right_sub = message_filters.Subscriber(self, PointCloud2, right_topic)
        queue_size = int(self.get_parameter('sync_queue_size').value)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], queue_size=queue_size,
            slop=self.get_parameter('sync_slop_sec').value)
        self.sync.registerCallback(self.sync_callback)

        self.missing_frames = {}
        self.missing_frame_threshold = int(
            self.get_parameter('missing_frame_threshold').value)
        self.max_output_age = float(self.get_parameter('max_output_age_sec').value)
        self.stale_dropped = 0

        self.pub = self.create_publisher(PointCloud2, output_topic, 10)

        self.get_logger().info(
            f"livox_fusion_node 启动：融合 [{left_topic}] + [{right_topic}] -> "
            f"[{output_topic}]，目标坐标系 [{self.target_frame}]，"
            f"同步队列深度 {queue_size}，输出龄期上限 {self.max_output_age:.2f}s")

    @staticmethod
    def _validate_parameters(parameters):
        if any(p.name == 'deskew_enable' and p.value for p in parameters):
            return SetParametersResult(successful=False, reason='Per-point deskew is not implemented')
        return SetParametersResult(successful=True)

    def _lookup_timeout_for(self, frame_id: str) -> Duration:
        """对"已判定压根不在 TF 树里"的 frame 不再等超时。

        等待是给"TF 还在路上"用的；frame 名字本身不存在时，每帧白等 tf_timeout_sec
        会把吞吐直接砍掉。实测(2026-09-07)：左雷达的 alias 静态 TF 因 DDS 掉线没送达，
        本节点从 9.2Hz 掉到 4.8Hz(≈1/0.208s，恰好是一次 0.2s 超时)，同步队列随即积满，
        输出帧龄期恒为 1.1s，下游 0.3s 陈旧闸门把每一帧都丢掉 ⇒ /scan、/map 全空、
        机器人不动，而本节点当时只打了一条被 throttle 压住的 WARN。
        宁可这一路不融合，也不能把整条链拖死。
        """
        if self.missing_frames.get(frame_id, 0) >= self.missing_frame_threshold:
            return Duration(seconds=0.0)
        return self.tf_timeout

    def _transform_to_target(self, cloud_msg: PointCloud2):
        """把一路点云变换到 target_frame，返回 (N,3) numpy 数组；变换失败返回 None。"""
        src = cloud_msg.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, src,
                Time.from_msg(cloud_msg.header.stamp),
                timeout=self._lookup_timeout_for(src))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            n = self.missing_frames.get(src, 0) + 1
            self.missing_frames[src] = n
            if n == self.missing_frame_threshold:
                self.get_logger().error(
                    f"[{src}] -> [{self.target_frame}] 连续 {n} 次查不到，判定该 frame "
                    f"不在 TF 树里：这一路点云从此被整路丢弃(只发另一路)。"
                    f"仿真下最常见的原因是 warehouse_sim.launch.py 里那三个 "
                    f"static_transform_publisher 别名进程有一个没进 ROS 图(DDS 掉线)，"
                    f"用 `ros2 run tf2_ros tf2_echo {self.target_frame} {src}` 复核。"
                    f"原始异常：{ex}")
            else:
                self.get_logger().warn(
                    f"tf2 变换失败 [{src}] -> [{self.target_frame}]: {ex}",
                    throttle_duration_sec=5.0)
            return None
        if self.missing_frames.get(src):
            self.get_logger().info(f"[{src}] 的 TF 已恢复，这一路重新参与融合")
            self.missing_frames[src] = 0
        transformed = do_transform_cloud(cloud_msg, tf)
        structured = pc2.read_points(transformed, field_names=('x', 'y', 'z'), skip_nans=True)
        if structured.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float32)
        pts = np.column_stack(
            [structured['x'], structured['y'], structured['z']]).astype(np.float32)
        return pts

    def sync_callback(self, left_msg: PointCloud2, right_msg: PointCloud2):
        now = self.get_clock().now()
        future_tolerance = self.get_parameter('future_tolerance_sec').value
        if any((now - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9 < -future_tolerance
               for msg in (left_msg, right_msg)):
            self.get_logger().warn('点云时间戳超前，丢弃此同步帧', throttle_duration_sec=5.0)
            return
        left_pts = self._transform_to_target(left_msg)
        right_pts = self._transform_to_target(right_msg)

        clouds = [p for p in (left_pts, right_pts) if p is not None and p.shape[0] > 0]
        if not clouds:
            return
        fused = np.concatenate(clouds, axis=0)

        participating = [msg for pts, msg in ((left_pts, left_msg), (right_pts, right_msg))
                         if pts is not None and pts.shape[0] > 0]
        header = deepcopy(min(participating, key=lambda msg:
                              Time.from_msg(msg.header.stamp).nanoseconds).header)
        header.frame_id = self.target_frame
        out = pc2.create_cloud_xyz32(header, fused.astype(np.float32))

        age = (self.get_clock().now() - Time.from_msg(header.stamp)).nanoseconds / 1e9
        if age > self.max_output_age or age < -future_tolerance:
            self.stale_dropped += 1
            self.get_logger().error(
                f"融合帧龄期 {age:.3f}s 超过上限 {self.max_output_age:.2f}s，不发布"
                f"(累计 {self.stale_dropped} 帧)。本节点吞吐跟不上输入才会这样："
                f"先看上面有没有 TF 查不到的 ERROR，再看是不是 sync_queue_size 调大了。",
                throttle_duration_sec=5.0)
            return

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
