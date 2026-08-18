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
    本节点已预留 `_deskew_placeholder()` 挂载点和详细注释，当前默认关闭
    （deskew_enable:=false），后续要接入 CustomMsg 精细去畸变时在这里扩展，
    不假装当前版本已经实现了完整的逐点去畸变。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
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
        # 去畸变预留开关，见文件头部说明；默认关闭，本版本不实现逐点插值。
        self.declare_parameter('deskew_enable', False)

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
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], queue_size=10,
            slop=self.get_parameter('sync_slop_sec').value)
        self.sync.registerCallback(self.sync_callback)

        self.pub = self.create_publisher(PointCloud2, output_topic, 10)

        self.get_logger().info(
            f"livox_fusion_node 启动：融合 [{left_topic}] + [{right_topic}] -> "
            f"[{output_topic}]，目标坐标系 [{self.target_frame}]")

    def _transform_to_target(self, cloud_msg: PointCloud2):
        """把一路点云变换到 target_frame，返回 (N,3) numpy 数组；变换失败返回 None。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, cloud_msg.header.frame_id,
                Time.from_msg(cloud_msg.header.stamp), timeout=self.tf_timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(
                f"tf2 变换失败 [{cloud_msg.header.frame_id}] -> [{self.target_frame}]: {ex}",
                throttle_duration_sec=5.0)
            return None
        transformed = do_transform_cloud(cloud_msg, tf)
        # 用 read_points()（结构化数组）而不是 read_points_numpy()：后者要求消息里
        # "所有字段"数据类型统一，遇到带 intensity/ring 等混合类型字段的点云会直接
        # assert 崩溃（livox_preprocess_node.py 里踩过这个坑，这里预防性同样处理）。
        structured = pc2.read_points(transformed, field_names=('x', 'y', 'z'), skip_nans=True)
        if structured.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float32)
        pts = np.column_stack(
            [structured['x'], structured['y'], structured['z']]).astype(np.float32)
        return pts

    def _deskew_placeholder(self, points: np.ndarray, cloud_msg: PointCloud2):
        """
        逐点去畸变扩展点（当前不启用，见文件头部说明）。
        若后续接入 livox_ros_driver2 的 CustomMsg（含每点 offset_time）：
          1. 在这里按 offset_time 把每个点的采集时刻精确定位到 scan_start ~ scan_end 之间；
          2. 用 /odom 里 scan_start~scan_end 时间段内的位姿插值，算出每个点相对 scan_start
             时刻的位姿修正量；
          3. 把每个点变换到 scan_start 时刻的传感器系下，再统一做本节点已有的 tf2 变换。
        目前 gz-sim 仿真点云和 xfer_format=2/0 的标准 PointCloud2 都没有逐点时间戳，
        直接原样返回，不做任何处理。
        """
        return points

    def sync_callback(self, left_msg: PointCloud2, right_msg: PointCloud2):
        left_pts = self._transform_to_target(left_msg)
        right_pts = self._transform_to_target(right_msg)

        clouds = [p for p in (left_pts, right_pts) if p is not None and p.shape[0] > 0]
        if not clouds:
            return
        fused = np.concatenate(clouds, axis=0)

        if self.get_parameter('deskew_enable').value:
            fused = self._deskew_placeholder(fused, left_msg)

        header = left_msg.header
        header.frame_id = self.target_frame
        out = pc2.create_cloud_xyz32(header, fused.astype(np.float32))
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
