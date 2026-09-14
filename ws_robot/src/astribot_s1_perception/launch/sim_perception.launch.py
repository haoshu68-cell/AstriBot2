#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：点云预处理 + 双雷达融合 + 2D投影 —— 仿真/硬件两个分支共用同一份节点图，
只是 use_sim_time 默认值不同（本文件默认 true，用于仿真；
launch/hardware_perception.launch.py 直接 include 本文件并把默认值覆盖成 false，
不重复写一遍节点定义）。

【仿真分支】假定 astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py 已经在跑
（Gazebo + 机器人 + /livox/lidar_left + /livox/lidar_right 已经有数据）。
【硬件分支】假定 launch/hardware_livox.launch.py 已经在跑
（robot_state_publisher + 两个 livox_ros_driver2 实例，同样发布到
/livox/lidar_left + /livox/lidar_right——两分支话题名统一，本文件代码零差异）。

数据链路：
  /livox/lidar_left  --> livox_preprocess_left  --> /livox/left/cloud_filtered  \\
                                                                                  --> livox_fusion_node --> /livox/fused_points --> pointcloud_to_laserscan --> /scan
  /livox/lidar_right --> livox_preprocess_right --> /livox/right/cloud_filtered /
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_perception = FindPackageShare('astribot_s1_perception')

    filter_params = PathJoinSubstitution(
        [pkg_perception, 'config', 'pointcloud_filter_params.yaml'])
    fusion_params = PathJoinSubstitution(
        [pkg_perception, 'config', 'livox_fusion_params.yaml'])
    p2l_params = PathJoinSubstitution(
        [pkg_perception, 'config', 'pointcloud_to_laserscan_params.yaml'])

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='仿真分支固定用仿真时钟'),
        DeclareLaunchArgument(
            'left_input_topic', default_value='/livox/lidar_left',
            description='左/前雷达的 PointCloud2 输入；实机传 /livox/lidar_front_pc2'),
        DeclareLaunchArgument(
            'right_input_topic', default_value='/livox/lidar_right',
            description='右/后雷达的 PointCloud2 输入；实机传 /livox/lidar_back_pc2'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')
    left_input = LaunchConfiguration('left_input_topic')
    right_input = LaunchConfiguration('right_input_topic')

    preprocess_left = Node(
        package='astribot_s1_perception',
        executable='livox_preprocess_node',
        name='livox_preprocess_left',
        output='screen',
        parameters=[filter_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cloud_in', left_input),
            ('cloud_out', '/livox/left/cloud_filtered'),
        ],
    )
    preprocess_right = Node(
        package='astribot_s1_perception',
        executable='livox_preprocess_node',
        name='livox_preprocess_right',
        output='screen',
        parameters=[filter_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cloud_in', right_input),
            ('cloud_out', '/livox/right/cloud_filtered'),
        ],
    )
    fusion = Node(
        package='astribot_s1_perception',
        executable='livox_fusion_node',
        name='livox_fusion_node',
        output='screen',
        parameters=[fusion_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('left/cloud_filtered', '/livox/left/cloud_filtered'),
            ('right/cloud_filtered', '/livox/right/cloud_filtered'),
            ('fused_points', '/livox/fused_points'),
        ],
    )
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[p2l_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cloud_in', '/livox/cloud_self_filtered'),
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription(declare_args + [
        preprocess_left,
        preprocess_right,
        fusion,
        pointcloud_to_laserscan,
    ])
