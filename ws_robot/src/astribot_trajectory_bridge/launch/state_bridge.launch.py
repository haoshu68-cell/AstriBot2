#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""状态桥接（Gate 2，只读）+ robot_state_publisher。

拉起的东西
========
1. robot_state_publisher —— 拿 URDF 出 TF，并按 URDF 的 mimic 关系补出
   夹爪那 5 个从动关节。桥接只发主动关节，从动关节是它算的。
2. state_bridge_node —— 用厂商 SDK 读状态，展开成逐关节 /joint_states。

!!! 用之前先确认没有别的 /joint_states 发布者 !!!
Gazebo 的 joint_state_broadcaster 也发这个话题。两个发布者同时在，
robot_state_publisher 会交替收到两份不同的姿态，TF 会抖，而**两边都没有报错**。
切到厂商栈时先把 Gazebo 那套整个关掉。

前置条件
=======
必须先有一个 SDK 后端在跑（厂商 MuJoCo 仿真或真机），否则
Astribot() 会抛 "No simulation or real robot is started." 并由本节点变成
启动期失败（这是有意的：宁可响亮失败，也不要静默不发状态）。
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

    nodes.append(Node(
        package='astribot_trajectory_bridge',
        executable='state_bridge_node',
        name='astribot_state_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[bridge_params],
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
            'use_robot_state_publisher', default_value='true',
            description='是否同时拉起 robot_state_publisher。'
                        '已经有一个在跑时置 false，避免两个 TF 源。'),
        OpaqueFunction(function=_setup),
    ])
