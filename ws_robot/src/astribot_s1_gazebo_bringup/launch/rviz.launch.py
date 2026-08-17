#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：离线核对模型/TF树的辅助 launch（不启动 Gazebo）。
只跑 robot_state_publisher + joint_state_publisher_gui + RViz2，
用于在没有仿真器、或怀疑 URDF/TF 有问题时快速核对：
  - 模型是否完整显示（贴图、mesh 路径是否解析成功）
  - 拖动 joint_state_publisher_gui 滑条，检查每个关节是否按预期转动、有没有断裂的 TF 分支
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument('robot_name', default_value='astribot_s1'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
    ]

    pkg_description = FindPackageShare('astribot_s1_description')
    pkg_bringup = FindPackageShare('astribot_s1_gazebo_bringup')

    xacro_file = PathJoinSubstitution([pkg_description, 'urdf', 'astribot_s1.xacro'])
    controllers_yaml = PathJoinSubstitution(
        [pkg_bringup, 'config', 'astribot_s1_controllers.yaml'])
    rviz_config = PathJoinSubstitution([pkg_description, 'rviz', 'astribot_s1_view.rviz'])

    robot_description_content = Command([
        'xacro', ' ', xacro_file, ' ',
        'robot_name:=', LaunchConfiguration('robot_name'), ' ',
        'use_lidar:=', LaunchConfiguration('use_lidar'), ' ',
        'use_camera:=', LaunchConfiguration('use_camera'), ' ',
        'controllers_config:=', controllers_yaml,
    ])

    return LaunchDescription(declare_args + [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
