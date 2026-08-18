#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：【硬件分支】点云预处理 + 双雷达融合 + 2D投影。

节点图和仿真分支完全一样，直接复用 sim_perception.launch.py（DRY，不重复写一遍），
唯一区别是 use_sim_time 默认改成 false（对应任务书"严格区分use_sim_time开关，
硬件环境关闭"）——实体机器人没有 /clock 仿真时钟，必须用系统真实时间。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='硬件分支固定关闭仿真时钟'),
    ]

    perception_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('astribot_s1_perception'),
                 'launch', 'sim_perception.launch.py'])),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
    )

    return LaunchDescription(declare_args + [perception_pipeline])
