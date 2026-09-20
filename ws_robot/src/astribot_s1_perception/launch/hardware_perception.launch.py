#!/usr/bin/env python3
"""One projection node for the hardware and simulation SLAM cloud contract."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam_cloud_topic', default_value='/map_scan_filtered'),
        DeclareLaunchArgument('scan_topic', default_value='/scan_from_cloud'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('astribot_s1_perception_components'),
                'launch', 'slice_scan.launch.py'])),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_cloud_topic': LaunchConfiguration('slam_cloud_topic'),
                'output_scan_topic': LaunchConfiguration('scan_topic'),
                'publish_markers': 'false',
            }.items()),
    ])
