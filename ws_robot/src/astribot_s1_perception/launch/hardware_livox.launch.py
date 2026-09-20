#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Release-owned dual MID360 drivers; PointCloud2 coordinates stay sensor-local."""

from launch import LaunchDescription
import json
from pathlib import Path

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from astribot_logging.launch import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _validate_configs(context):
    for side in ('left', 'right'):
        path = Path(LaunchConfiguration(f'{side}_config').perform(context))
        config = json.loads(path.read_text())
        if not config.get('lidar_configs'):
            raise RuntimeError(f'雷达配置为空: {path}')
        for lidar in config['lidar_configs']:
            if any(float(v) != 0 for v in lidar['extrinsic_parameter'].values()):
                raise RuntimeError(f'{path}: 驱动外参必须为零，刚体外参由 Voxel-SLAM 统一应用')
    return []


def generate_launch_description():
    pkg_description = FindPackageShare('astribot_s1_description')
    pkg_perception = FindPackageShare('astribot_s1_perception')

    declare_args = [
        DeclareLaunchArgument('robot_name', default_value='astribot_s1'),
        DeclareLaunchArgument('publish_robot_description', default_value='true'),
        *[DeclareLaunchArgument(f'{side}_config', default_value=PathJoinSubstitution([
            pkg_perception, 'config', f'MID360_config_{side}.json'])) for side in ('left', 'right')],
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('publish_freq', default_value='10.0',
                              description='Mid-360 点云发布频率(Hz)，见官方推荐值 '
                                          '5/10/20/50，最大100'),
    ]

    xacro_file = PathJoinSubstitution([pkg_description, 'urdf', 'astribot_s1.xacro'])
    robot_description_content = ParameterValue(
        Command([
            'xacro', ' ', xacro_file, ' ',
            'robot_name:=', LaunchConfiguration('robot_name'), ' ',
            'use_lidar:=', LaunchConfiguration('use_lidar'), ' ',
            'use_camera:=', LaunchConfiguration('use_camera'),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        condition=IfCondition(LaunchConfiguration('publish_robot_description')),
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': False},
        ],
    )

    def make_livox_driver(side, config_file):
        return Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name=f'livox_lidar_publisher_{side}',
            output='screen',
            parameters=[{
                'xfer_format': 0,
                'multi_topic': 0,
                'data_src': 0,             # 0=雷达，其它值是回放bag等场景，见官方文档
                'publish_freq': LaunchConfiguration('publish_freq'),
                'output_data_type': 0,
                'frame_id': f'livox_mid360_{side}',
                'user_config_path': config_file,
                'cmdline_input_bd_code': 'livox0000000001',
            }],
            remappings=[
                ('livox/lidar', f'/livox/lidar_{side}'),
                ('livox/imu', '/livox/imu' if side == 'left' else '/livox/imu_back'),
            ],
        )

    livox_left = make_livox_driver(
        'left', LaunchConfiguration('left_config'))
    livox_right = make_livox_driver(
        'right', LaunchConfiguration('right_config'))

    return LaunchDescription(declare_args + [
        OpaqueFunction(function=_validate_configs),
        robot_state_publisher,
        livox_left,
        livox_right,
    ])
