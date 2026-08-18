#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：【顶层入口】一条命令切换 仿真/硬件 × 建图/定位 四种组合。

用法示例：
    # 仿真 + 建图，顺带拉起仓储世界仿真
    ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
        env:=sim mode:=mapping launch_gazebo:=true

    # 仿真已经在另一个终端跑着了，这里只挂感知+建图
    ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
        env:=sim mode:=mapping launch_gazebo:=false

    # 仿真 + 定位（用建图阶段 /slam_toolbox/serialize_map 保存好的地图）
    ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
        env:=sim mode:=localization map_file_name:=/path/to/my_map

    # 实体机器人 + 建图
    ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
        env:=hardware mode:=mapping
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_perception = FindPackageShare('astribot_s1_perception')
    pkg_bringup = FindPackageShare('astribot_s1_gazebo_bringup')

    declare_args = [
        DeclareLaunchArgument(
            'env', default_value='sim',
            description='运行环境：sim(仿真) 或 hardware(实体机器人)'),
        DeclareLaunchArgument(
            'mode', default_value='mapping',
            description='SLAM 模式：mapping(建图) 或 localization(纯定位)'),
        DeclareLaunchArgument(
            'launch_gazebo', default_value='true',
            description='env:=sim 时是否顺带拉起 warehouse_sim.launch.py；'
                        '如果仓储仿真已经在别的终端跑着了，设成 false 避免重复生成机器人'),
        DeclareLaunchArgument(
            'map_file_name', default_value='',
            description='mode:=localization 时要加载的地图基础文件名，见 '
                        'slam_localization.launch.py 里的详细说明'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='astribot_s1'),
    ]

    env = LaunchConfiguration('env')
    mode = LaunchConfiguration('mode')
    is_sim = PythonExpression(["'", env, "' == 'sim'"])
    is_hardware = PythonExpression(["'", env, "' == 'hardware'"])
    is_mapping = PythonExpression(["'", mode, "' == 'mapping'"])
    is_localization = PythonExpression(["'", mode, "' == 'localization'"])

    # ---- 仿真分支 ----
    warehouse_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'warehouse_sim.launch.py'])),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_rviz': 'false',  # 用本文件统一的感知+SLAM RViz配置，不重复开两个RViz
        }.items(),
        condition=IfCondition(PythonExpression(
            ["'", env, "' == 'sim' and '", LaunchConfiguration('launch_gazebo'), "' == 'true'"])),
    )
    sim_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_perception, 'launch', 'sim_perception.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(is_sim),
    )

    # ---- 硬件分支 ----
    hardware_livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_perception, 'launch', 'hardware_livox.launch.py'])),
        launch_arguments={'robot_name': LaunchConfiguration('robot_name')}.items(),
        condition=IfCondition(is_hardware),
    )
    hardware_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_perception, 'launch', 'hardware_perception.launch.py'])),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(is_hardware),
    )

    # ---- SLAM（建图/定位二选一，use_sim_time 跟随 env） ----
    use_sim_time_str = PythonExpression(["'true' if '", env, "' == 'sim' else 'false'"])
    slam_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_perception, 'launch', 'slam_mapping.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time_str}.items(),
        condition=IfCondition(is_mapping),
    )
    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_perception, 'launch', 'slam_localization.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time_str,
            'map_file_name': LaunchConfiguration('map_file_name'),
        }.items(),
        condition=IfCondition(is_localization),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution(
            [pkg_perception, 'rviz', 'perception_slam_view.rviz'])],
        parameters=[{'use_sim_time': use_sim_time_str}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(declare_args + [
        warehouse_sim,
        sim_perception,
        hardware_livox,
        hardware_perception,
        slam_mapping,
        slam_localization,
        rviz_node,
    ])
