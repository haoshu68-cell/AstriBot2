#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：单路 Livox Mid-360 点云预处理节点。

功能（对应任务书"点云预处理"要求）：
  1. 直通滤波(passthrough)：按到传感器原点的欧氏距离 [range_min, range_max] 保留点，
     去掉盲区自遮挡点(如扫到机器人自己躯干/机械臂)和过远的无效远点(仓储世界外墙/屋顶)。
  2. 高度直通滤波：按传感器局部坐标系 z 值 [ground_z_min, height_max] 保留点——
     ground_z_min 用来剔除地面点（每台雷达安装高度/俯仰角固定已知，地面在雷达局部系里
     大致是一个常数 z 阈值，不需要额外做实时地面平面拟合，工程上足够用且开销很小）；
     height_max 用来剔除过高的无效点（仓储货架顶部、屋顶等）。
  3. 轻量"半径离群点"密度滤波：不依赖 PCL/scipy，用体素分箱统计每个点所在体素里的
     点数，点数低于阈值的体素判定为离群噪声整体剔除——近似 PCL RadiusOutlierRemoval
     的效果，但实现和依赖都更轻量，方便在没有额外重量级三方库的环境里跑。

节点是"哑对称"的：同一份代码通过 remap ~cloud_in/~cloud_out 和参数文件里不同的
namespace 分别用于左右两台雷达（见 launch/sim_perception.launch.py 里各起一个实例）。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class LivoxPreprocessNode(Node):

    def __init__(self):
        super().__init__('livox_preprocess_node')

        # ---- 所有可调参数，外部通过 yaml/launch 传入，禁止写死 ----
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 35.0)
        self.declare_parameter('ground_z_min', -0.05)
        self.declare_parameter('height_max', 3.0)
        self.declare_parameter('voxel_size', 0.15)
        self.declare_parameter('min_points_per_voxel', 2)
        # 注意：use_sim_time 是 rclpy Node 自动声明的标准参数，这里不需要（也不能）
        # 再手动 declare 一遍，否则会抛 ParameterAlreadyDeclaredException——
        # 通过 launch 的 parameters=[{'use_sim_time': ...}] 传入即可自动生效。

        # !!! 订阅端必须用 BEST_EFFORT（SensorDataQoS）!!!
        # 原来这里写 `create_subscription(..., 10)`，depth=10 走的是**默认 QoS**，
        # 而默认 QoS 是 RELIABLE。仿真里恰好能用（ros_gz_bridge 发 RELIABLE），
        # 但实机上游（livox_custom_to_pc2_node）按点云惯例发 BEST_EFFORT，于是：
        #   BEST_EFFORT 发布者 + RELIABLE 订阅者 = **不兼容，一帧都收不到**
        # rclpy 会打一条 WARNING（"offering incompatible QoS ... RELIABILITY"），
        # 但节点照常活着、`cloud_out` 的发布者也照常存在 —— 于是下游看到
        # "pub=1 但 0 Hz"，整条链静默断在这里。实测就是这么断的。
        #
        # 反向是兼容的：BEST_EFFORT 订阅者可以收 RELIABLE 发布者。
        # 所以改成 BEST_EFFORT 同时满足仿真与实机，不需要分支。
        #
        # 发布端**刻意保持默认 RELIABLE 不动**：下游 livox_fusion_node 用
        # message_filters.Subscriber（默认 RELIABLE），改发布端会把断点挪到那里。
        self.sub = self.create_subscription(
            PointCloud2, 'cloud_in', self.cloud_callback,
            qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, 'cloud_out', 10)

        self.get_logger().info(
            f"livox_preprocess_node 启动：range=[{self.get_parameter('range_min').value}, "
            f"{self.get_parameter('range_max').value}]m, "
            f"z=[{self.get_parameter('ground_z_min').value}, "
            f"{self.get_parameter('height_max').value}]m")

    def cloud_callback(self, msg: PointCloud2):
        range_min = self.get_parameter('range_min').value
        range_max = self.get_parameter('range_max').value
        ground_z_min = self.get_parameter('ground_z_min').value
        height_max = self.get_parameter('height_max').value
        voxel_size = self.get_parameter('voxel_size').value
        min_pts = self.get_parameter('min_points_per_voxel').value

        # !!! 实测踩坑记录 !!!：一开始用 read_points_numpy 一把梭直接读 xyz，
        # 结果在 gz-sim 仿真雷达的点云上崩了——gz-sim 的 gpu_lidar 点云除了
        # x/y/z(float32)，还带了 intensity(float32) 和 ring(uint16) 两个字段，
        # read_points_numpy 内部要求"消息里所有字段"统一数据类型才能一次性转成
        # 同构 numpy 数组，字段类型不一致就直接 assert 崩溃退出（不是只看我们
        # 请求的 x/y/z 三个字段，而是看消息自带的全部字段）。改用通用的
        # read_points()，它返回按字段名索引的结构化数组，不同字段类型混着也没事，
        # 用完后再手动拼成 (N,3) 的 float 数组。
        structured = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if structured.shape[0] == 0:
            self._publish(msg, np.zeros((0, 3), dtype=np.float32))
            return
        points = np.column_stack(
            [structured['x'], structured['y'], structured['z']]).astype(np.float32)

        # 1) 直通滤波：距离 + 高度
        dist = np.linalg.norm(points, axis=1)
        mask = (dist >= range_min) & (dist <= range_max)
        mask &= (points[:, 2] >= ground_z_min) & (points[:, 2] <= height_max)
        points = points[mask]

        # 2) 体素密度滤波（近似半径离群点剔除）
        if points.shape[0] > 0 and voxel_size > 0:
            voxel_idx = np.floor(points / voxel_size).astype(np.int64)
            # 用 (vx,vy,vz) 组合成唯一 key，统计每个体素内点数
            keys = (voxel_idx[:, 0].astype(np.int64) * 73856093) ^ \
                   (voxel_idx[:, 1].astype(np.int64) * 19349663) ^ \
                   (voxel_idx[:, 2].astype(np.int64) * 83492791)
            unique_keys, inverse, counts = np.unique(keys, return_inverse=True, return_counts=True)
            keep = counts[inverse] >= min_pts
            points = points[keep]

        self._publish(msg, points)

    def _publish(self, src_msg: PointCloud2, points: np.ndarray):
        out = pc2.create_cloud_xyz32(src_msg.header, points.astype(np.float32))
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxPreprocessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
