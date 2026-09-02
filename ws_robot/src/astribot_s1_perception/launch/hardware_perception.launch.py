#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：【硬件分支】点云预处理 + 双雷达融合 + 2D投影。

节点图和仿真分支完全一样，直接复用 sim_perception.launch.py（DRY，不重复写一遍），
两处区别：
  1. use_sim_time 默认改成 false（对应任务书"严格区分use_sim_time开关，
     硬件环境关闭"）——实体机器人没有 /clock 仿真时钟，必须用系统真实时间。
  2. **输入话题名不同**。实机厂商驱动以 xfer_format=1 发 CustomMsg
     （SLAM 的 lidar_type=0 需要它，不能改），话题是
     /livox/lidar_{front,back}；而本链订阅 PointCloud2。中间必须先跑
     astribot_s1_autonomy 的 livox_custom_to_pc2_node 转一道，
     它输出 /livox/lidar_{front,back}_pc2。

     ⚠️ 本文件原先只覆盖 use_sim_time 就直接 include，于是实机上这条链
     订阅了 /livox/lidar_{left,right} —— 两个**根本不存在**的话题。
     后果是整条链静默无输出、/scan 一帧不出，而每个节点进程看着都正常。
     nav2 两个 costmap 的 obstacle_layer 唯一数据源就是 /scan，
     所以那等于**完全没有动态避障**。

前置依赖（本文件不启动它们，缺了就没有输出）：
  · livox_custom_to_pc2_node —— 提供 _pc2 两路输入
  · pointcloud_slice_scan_node（astribot_s1_autonomy）—— 提供
    /livox/cloud_self_filtered，本链末端的 pointcloud_to_laserscan 订阅它
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
        DeclareLaunchArgument(
            'left_input_topic', default_value='/livox/lidar_front_pc2',
            description='前雷达经 CustomMsg→PointCloud2 转换后的话题'),
        DeclareLaunchArgument(
            'right_input_topic', default_value='/livox/lidar_back_pc2',
            description='后雷达经 CustomMsg→PointCloud2 转换后的话题'),
    ]

    perception_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('astribot_s1_perception'),
                 'launch', 'sim_perception.launch.py'])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'left_input_topic': LaunchConfiguration('left_input_topic'),
            'right_input_topic': LaunchConfiguration('right_input_topic'),
        }.items(),
    )

    return LaunchDescription(declare_args + [perception_pipeline])
