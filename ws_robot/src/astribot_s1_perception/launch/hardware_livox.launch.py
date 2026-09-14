#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：【硬件分支】实体机器人启动 —— robot_state_publisher（机器人状态发布，
提供 TF）+ 两个独立的 livox_ros_driver2 节点实例（左右各一台真实 Mid-360）。

!!! 重要：本文件在没有真实 Mid-360 硬件、且本环境刚源码编译完 livox_ros_driver2 的情况下，
只做到"语法完整、可直接运行"，节点起来后能不能连上真实雷达取决于：
  1) config/MID360_config_left.json / MID360_config_right.json 里的设备IP、
     host_net_info 是否和现场网络配置一致（出厂默认设备IP通常是 192.168.1.1xx 段，
     需要按实际雷达和网卡配置修改，不能直接照抄仓库里的占位值）；
  2) 两台 Mid-360 是否真的用独立网卡/独立IP别名接入（多雷达典型走线方式）。
见 README_PERCEPTION.md 的"硬件分支联调清单"一节。

frame_id / multi_topic / xfer_format / publish_freq 都是 livox_ros_driver2 的 ROS 参数
（不是 JSON 字段，见 astribot_s1_perception 顶层方案说明）：
  - xfer_format 固定用 2（标准 sensor_msgs/PointCloud2），不用默认的 CustomMsg，
    这样和仿真分支输出的消息类型完全一致，下游预处理/融合节点代码不用区分分支；
  - 每台雷达一个独立节点实例、独立 frame_id（livox_mid360_left/right），
    不允许两台雷达共用一个 frame_id（任务书强制要求）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_description = FindPackageShare('astribot_s1_description')
    pkg_perception = FindPackageShare('astribot_s1_perception')

    declare_args = [
        DeclareLaunchArgument('robot_name', default_value='astribot_s1'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('publish_freq', default_value='10.0',
                              description='Mid-360 点云发布频率(Hz)，见官方推荐值 '
                                          '5/10/20/50，最大100'),
    ]

    xacro_file = PathJoinSubstitution([pkg_description, 'urdf', 'astribot_s1.xacro'])
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('astribot_s1_gazebo_bringup'),
         'config', 'astribot_s1_controllers.yaml'])

    robot_description_content = ParameterValue(
        Command([
            'xacro', ' ', xacro_file, ' ',
            'robot_name:=', LaunchConfiguration('robot_name'), ' ',
            'use_lidar:=', LaunchConfiguration('use_lidar'), ' ',
            'use_camera:=', LaunchConfiguration('use_camera'), ' ',
            'controllers_config:=', controllers_yaml,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
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
                'xfer_format': 2,          # 0/2=标准PointCloud2，1(默认)=CustomMsg，见文件头说明
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
                ('livox/imu', f'/livox/imu_{side}'),
            ],
        )

    livox_left = make_livox_driver(
        'left', PathJoinSubstitution([pkg_perception, 'config', 'MID360_config_left.json']))
    livox_right = make_livox_driver(
        'right', PathJoinSubstitution([pkg_perception, 'config', 'MID360_config_right.json']))

    return LaunchDescription(declare_args + [
        robot_state_publisher,
        livox_left,
        livox_right,
    ])
