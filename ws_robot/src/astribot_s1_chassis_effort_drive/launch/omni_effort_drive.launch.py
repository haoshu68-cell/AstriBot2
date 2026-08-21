#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：X 型全向轮力矩闭环驱动节点启动文件。

单独暴露 wheel_radius 这个launch参数（而不是只靠yaml），是专门为"改变轮子
大小做多轮测试"这个扫描场景准备的——扫描不同轮径时只需要在命令行加
`wheel_radius:=0.12` 之类的覆盖，不用改yaml文件、也不用改代码。
注意：改这个参数只影响力矩闭环节点自己的逆解计算，SDF里碰撞体半径、spawn_z、
轮关节origin的z偏移是几何模型的一部分，必须跟这里同步改(xacro参数，
见astribot_s1_torso_wheel.xacro的wheel_collision_radius)，否则轮子和地面
的实际接触几何跟节点这里算的运动学半径不一致，扫描结果没有意义。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('astribot_s1_chassis_effort_drive')

    declare_args = [
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument(
            'effort_command_topic', default_value='/wheel_effort_controller/commands'),
        DeclareLaunchArgument(
            'wheel_radius', default_value='0.08',
            description='轮径扫描测试用：命令行覆盖即可，务必和SDF碰撞体半径同步'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [pkg, 'config', 'omni_effort_drive_params.yaml']),
            description='本节点的YAML参数文件，可整体替换成自定义调参版本'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    drive_node = Node(
        package='astribot_s1_chassis_effort_drive',
        executable='omni_effort_drive_node',
        name='omni_effort_drive_node',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'effort_command_topic': LaunchConfiguration('effort_command_topic'),
                # ParameterValue(..., value_type=float)：命令行传进来的 wheel_radius
                # 本质是字符串，强制转成float，否则跟节点里declare_parameter的
                # 0.08(float)类型不一致，rclpy会拒绝覆盖并报类型错误。
                'wheel_radius': ParameterValue(
                    LaunchConfiguration('wheel_radius'), value_type=float),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    return LaunchDescription(declare_args + [drive_node])
