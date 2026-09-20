#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simulation point-cloud projection for the unified Voxel-SLAM path.

Gazebo publishes the two standard PointCloud2 inputs directly to Voxel-SLAM.
This launch file only projects Voxel-SLAM's world-frame filtered cloud into
the LaserScan consumed by Nav2 and the independent safety bridge.  The old
preprocess/fusion chain is intentionally removed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('navigation_geometry_mode',default_value='legacy'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cloud_pose_frame', default_value=''),
        DeclareLaunchArgument(
            'slam_cloud_topic', default_value=PythonExpression([
                "'/map_scan' if '",LaunchConfiguration('navigation_geometry_mode'),
                "' == 'fixed_v2' else '/map_scan_filtered'"]),
            description='fixed_v2 uses the full-height SLAM cloud before navigation projection'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('astribot_s1_perception_components'),
                'launch', 'slice_scan.launch.py'])),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'navigation_geometry_mode':LaunchConfiguration('navigation_geometry_mode'),
                'input_cloud_topic': LaunchConfiguration('slam_cloud_topic'),
                'cloud_pose_frame': LaunchConfiguration('cloud_pose_frame'),
                'output_scan_topic': '/scan_from_cloud',
                'publish_markers': 'false',
            }.items(),
        ),
    ])
