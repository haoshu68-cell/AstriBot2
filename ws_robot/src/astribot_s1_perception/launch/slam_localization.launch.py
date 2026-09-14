#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：SLAM Toolbox"纯定位模式"——加载之前建图并保存好的地图，只做重定位，
不再持续扩建地图。

没有直接 IncludeLaunchDescription 官方的 localization_launch.py，是因为那个 launch
文件的 slam_params_file 是一整个静态 yaml，没有单独暴露 map_file_name 这个launch参数
接口；这里改成直接起 slam_toolbox 的 localization_slam_toolbox_node 节点，
用 "yaml文件 + 一个小的覆盖字典" 的方式动态注入 map_file_name/map_start_pose
（ROS2 parameters 列表里后面的字典会覆盖前面 yaml 里的同名 key，是标准写法），
这样命令行传参更直接，也不用每次为了换地图去手改 yaml。

map_file_name 是 slam_toolbox 自己序列化位姿图的"基础文件名"(不带扩展名)，
建图完成后用它的 /slam_toolbox/serialize_map 服务保存得到 <name>.data/<name>.posegraph，
不是 nav2_map_server 的 .pgm+.yaml 栅格地图。

用法：
    ros2 launch astribot_s1_perception slam_localization.launch.py \
        use_sim_time:=true map_file_name:=/绝对路径/或/相对路径/my_map
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from astribot_logging.launch import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_perception = FindPackageShare('astribot_s1_perception')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution(
                [pkg_perception, 'config', 'mapper_params_localization.yaml']),
            description='SLAM Toolbox 定位模式参数文件路径'),
        DeclareLaunchArgument(
            'map_file_name', default_value='',
            description='要加载的地图基础文件名(不带扩展名)，建图阶段用 '
                        'ros2 service call /slam_toolbox/serialize_map 保存得到；'
                        '留空则定位模式启动时不预加载地图(相当于从空地图开始纯定位，'
                        '通常没有意义，正常使用务必显式传这个参数)。'
                        '常见路径配置错误：忘记去掉扩展名、用了相对路径但工作目录不对——'
                        '建议直接传绝对路径，参见 README_PERCEPTION.md 故障排查。'),
        DeclareLaunchArgument(
            'map_start_pose', default_value='[0.0, 0.0, 0.0]',
            description='（仅文档用途，不会真正生效，见下方注释）定位起始位姿 [x, y, yaw]，'
                        '要改请直接编辑 config/mapper_params_localization.yaml 里的同名字段'),
    ]

    localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_file_name': LaunchConfiguration('map_file_name'),
            },
        ],
    )

    return LaunchDescription(declare_args + [localization_node])
