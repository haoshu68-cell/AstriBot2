#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compatibility-free simulation alias for :mod:`voxel_slam.launch.py`."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('astribot_s1_perception'), 'launch',
                'voxel_slam.launch.py'])),
            launch_arguments={
                'use_sim_time': 'true',
                'lidar_topic': '/livox/lidar_left',
                'lidar_topic_back': '/livox/lidar_right',
                'imu_topic': '/livox/imu',
                'point_notime': '1',
            }.items(),
        )
    ])
