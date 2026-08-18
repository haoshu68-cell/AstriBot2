#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：封装官方 slam_toolbox 的 online_async_launch.py，接入本机器人的
自定义参数文件（mapper_params_online_async.yaml），实现"在线建图模式"。

用法：
    ros2 launch astribot_s1_perception slam_mapping.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('astribot_s1_perception'),
                 'config', 'mapper_params_online_async.yaml']),
            description='SLAM Toolbox 建图模式参数文件路径'),
    ]

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription(declare_args + [slam_toolbox_launch])
