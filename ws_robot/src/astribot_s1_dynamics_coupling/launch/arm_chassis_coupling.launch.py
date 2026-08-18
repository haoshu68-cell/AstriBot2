#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：独立、可复用的臂-底盘动力学耦合调速节点启动文件。设计成通用的
"input_topic → output_topic 桥接器"，不跟任何具体上游节点(Nav2的
cmd_vel_body_to_world_node、或者以后可能接入的其它cmd_vel来源)绑定死，
接入哪条链路只需要在上层launch文件里：
  1. 把要接管的上游节点的输出话题参数覆盖成跟这里 input_topic 一致的中间话题名
     （不修改上游节点的代码，只是launch层的参数覆盖）；
  2. include 本文件，output_topic 填真正的 /cmd_vel。

respawn=True：节点进程崩溃时自动重启（见节点文件头部"静默失效"能力边界说明——
这是进程级兜底，重启期间会有短暂中断，不是零感知无缝失效转移）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('astribot_s1_dynamics_coupling')

    declare_args = [
        DeclareLaunchArgument(
            'input_topic', default_value='/cmd_vel_pre_arm_coupling',
            description='上游(Nav2/巡游, 经body→world转换后)真正想下发的world系Twist，'
                        '这个话题名要跟上游节点被覆盖后的输出话题参数一致'),
        DeclareLaunchArgument(
            'output_topic', default_value='/cmd_vel',
            description='缩放后最终发给 gz-sim VelocityControl 插件的话题'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [pkg, 'config', 'arm_chassis_coupling_params.yaml']),
            description='本节点的YAML参数文件，可整体替换成自定义调参版本'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    coupling_node = Node(
        package='astribot_s1_dynamics_coupling',
        executable='arm_chassis_speed_coupling_node',
        name='arm_chassis_speed_coupling_node',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    return LaunchDescription(declare_args + [coupling_node])
