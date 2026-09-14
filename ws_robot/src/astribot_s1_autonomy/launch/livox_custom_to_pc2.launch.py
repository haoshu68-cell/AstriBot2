#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""文件用途：实机 CustomMsg → PointCloud2 转换节点的启动文件。

════════════════ 为什么需要这一环 ════════════════
实机厂商驱动以 `xfer_format=1` 发 `livox_ros_driver2/msg/CustomMsg`
（SLAM 的 `mid360.yaml` 里 `lidar_type=0` 需要它，改不了），
而我们的感知链订阅 `sensor_msgs/PointCloud2`：

    livox_fusion_node（订阅 PointCloud2×2）
      → pointcloud_slice_scan_node（自滤 + 多层切片）
      → /scan
      → nav2 两个 costmap 的 obstacle_layer（**唯一**数据源）

所以这一环断着就等于**没有动态避障**。早先记录写的"xfer_format 1-vs-2 互斥、
无解"是错的：不是无解，是中间缺一个转换环节。

════════════════ 两路的无效点位置不同，别配串 ════════════════
驱动通过 `SetLivoxLidarInstallAttitude` 把外参**写进雷达设备**，
所以无回波的零点也被外参变换过：

    front（IP .12，外参全零）      无效点在 (0, 0, 0)               实测占 33.4%
    back （IP .13，外参 y=-496mm）  无效点在 (0.001, -0.496, 0.084)   实测占 34.0%

back 那一团距原点 0.496m，**大于** `livox_preprocess_node` 的 `range_min=0.35`，
现有球面距离门限滤不掉它；而它落在机器人足迹内，自滤没接上就直接变成障碍物。

用法：
    ros2 launch astribot_s1_autonomy livox_custom_to_pc2.launch.py
    # 换话题名
    ros2 launch astribot_s1_autonomy livox_custom_to_pc2.launch.py \\
        front_input:=/livox/lidar_front back_input:=/livox/lidar_back
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('astribot_s1_autonomy')
    default_params = PathJoinSubstitution(
        [pkg, 'config', 'livox_custom_to_pc2_params.yaml'])

    args = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='参数文件；无效点位置等都在里面'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='实机必须 false。仿真里本节点用不到（仿真直接出 PointCloud2）'),
    ]

    node = Node(
        package='astribot_s1_autonomy',
        executable='livox_custom_to_pc2_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        emulate_tty=True,
    )

    return LaunchDescription(args + [node])
