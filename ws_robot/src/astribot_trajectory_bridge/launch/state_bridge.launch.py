#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动状态桥和 robot_state_publisher，生成本体连杆 TF。

feedback_source=manufacturer 使用厂家 ROS 反馈，不创建 SDK 会话；sdk 保留
原读取方式。已有 /joint_states 或模型发布者时，可分别关闭对应启动项。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from astribot_logging.launch import Node


def _setup(context, *args, **kwargs):
    bridge_params = LaunchConfiguration('bridge_params').perform(context)
    use_rsp = LaunchConfiguration('use_robot_state_publisher').perform(context)
    use_bridge = LaunchConfiguration('use_state_bridge').perform(context)
    feedback_source = LaunchConfiguration('feedback_source').perform(context)

    description_share = get_package_share_directory('astribot_s1_description')
    xacro_path = os.path.join(description_share, 'urdf', 'astribot_s1.xacro')

    import subprocess
    result = subprocess.run(
        ['xacro', xacro_path, 'robot_name:=astribot_s1'],
        capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            'xacro 展开失败（%s）:\n%s' % (xacro_path, result.stderr))
    robot_description = result.stdout

    nodes = []
    if use_rsp.lower() in ('1', 'true', 'yes'):
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': False}],
        ))

    if use_bridge.lower() in ('1', 'true', 'yes'):
        nodes.append(Node(
            package='astribot_trajectory_bridge',
            executable='state_bridge_node',
            name='astribot_state_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[bridge_params, {'bridge.feedback_source': feedback_source}],
        ))
    return nodes


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('astribot_trajectory_bridge'),
        'config', 'bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'bridge_params', default_value=default_params,
            description='bridge.yaml 路径（部件->关节映射表在里面）'),
        DeclareLaunchArgument(
            'feedback_source', default_value='sdk',
            description='sdk 或 manufacturer；真机部署直接订阅厂家反馈，不创建 SDK 会话'),
        DeclareLaunchArgument(
            'use_state_bridge', default_value='true',
            description='已有 joint_states 发布者时复用该来源'),
        DeclareLaunchArgument(
            'use_robot_state_publisher', default_value='true',
            description='是否同时拉起 robot_state_publisher。'
                        '已经有一个在跑时置 false，避免两个 TF 源。'),
        OpaqueFunction(function=_setup),
    ])
