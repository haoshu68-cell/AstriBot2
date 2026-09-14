#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：同时启动【感知模块 + 决策模块】，两个节点作为 rclcpp 组件
装进同一个 component_container，进程内直连、共享内存零拷贝。

为什么默认用组件容器：
    两个节点装在同一进程里，少一个进程、共享执行器线程池、便于统一管理。
    需要独立调试/单独 gdb 某个节点时，把 use_composition:=false，
    会退化成两个独立进程（复用 slice_scan.launch.py / frontier_explore.launch.py 的可执行文件）。

    注意：这里**没有**开 use_intra_process_comms。实测开了会直接抛
    "intraprocess communication allowed only with volatile durability"——
    因为两个节点都要用 TF，而 /tf_static 必须是 transient_local。详见下方代码注释。

用法示例：
    # 组件容器方式（默认，推荐）
    ros2 launch astribot_s1_autonomy autonomy_bringup.launch.py

    # 两个独立进程方式，便于单独 gdb
    ros2 launch astribot_s1_autonomy autonomy_bringup.launch.py use_composition:=false

    # 只要感知不要探索
    ros2 launch astribot_s1_autonomy autonomy_bringup.launch.py enable_explorer:=false

    # 让感知直接顶替既有 /scan，并同时开 RViz
    ros2 launch astribot_s1_autonomy autonomy_bringup.launch.py \\
        output_scan_topic:=/scan use_rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from astribot_logging import log_level as default_log_level
from astribot_logging.launch import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def _to_bool(text):
    lowered = text.strip().lower()
    if lowered in ('true', '1', 'yes'):
        return True
    if lowered in ('false', '0', 'no'):
        return False
    raise RuntimeError(f'布尔参数只接受 true/false，收到: {text!r}')


def _build(context, *args, **kwargs):
    pkg_share = FindPackageShare('astribot_s1_autonomy')

    perception_params = LaunchConfiguration('perception_params_file').perform(context)
    explorer_params = LaunchConfiguration('explorer_params_file').perform(context)
    use_sim_time = _to_bool(LaunchConfiguration('use_sim_time').perform(context))
    use_composition = _to_bool(LaunchConfiguration('use_composition').perform(context))
    enable_perception = _to_bool(LaunchConfiguration('enable_perception').perform(context))
    enable_explorer = _to_bool(LaunchConfiguration('enable_explorer').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)

    perception_overrides = {}
    scan_topic = LaunchConfiguration('output_scan_topic').perform(context)
    if scan_topic != '':
        perception_overrides['output_scan_topic'] = scan_topic
    cloud_topic = LaunchConfiguration('input_cloud_topic').perform(context)
    if cloud_topic != '':
        perception_overrides['input_cloud_topic'] = cloud_topic

    explorer_overrides = {}
    goal_topic = LaunchConfiguration('goal_topic').perform(context)
    if goal_topic != '':
        explorer_overrides['goal_topic'] = goal_topic

    perception_parameters = [perception_params, {'use_sim_time': use_sim_time}]
    if perception_overrides:
        perception_parameters.append(perception_overrides)
    explorer_parameters = [explorer_params, {'use_sim_time': use_sim_time}]
    if explorer_overrides:
        explorer_parameters.append(explorer_overrides)

    if not enable_perception and not enable_explorer:
        raise RuntimeError('enable_perception 和 enable_explorer 不能同时为 false，那样什么都不会启动')

    actions = []

    if use_composition:
        composable = []
        if enable_perception:
            composable.append(
                ComposableNode(
                    package='astribot_s1_perception_components',
                    plugin='astribot_s1_autonomy::PointcloudSliceScanNode',
                    name='pointcloud_slice_scan_node',
                    parameters=perception_parameters,
                ))
        if enable_explorer:
            composable.append(
                ComposableNode(
                    package='astribot_s1_exploration',
                    plugin='astribot_s1_autonomy::FrontierExplorerNode',
                    name='frontier_explorer_node',
                    parameters=explorer_parameters,
                ))
        actions.append(
            ComposableNodeContainer(
                name='astribot_autonomy_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=composable,
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                emulate_tty=True,
            ))
    else:
        if enable_perception:
            actions.append(
                Node(
                    package='astribot_s1_perception_components',
                    executable='pointcloud_slice_scan_node',
                    name='pointcloud_slice_scan_node',
                    output='screen',
                    parameters=perception_parameters,
                    arguments=['--ros-args', '--log-level', log_level],
                    emulate_tty=True,
                ))
        if enable_explorer:
            actions.append(
                Node(
                    package='astribot_s1_exploration',
                    executable='frontier_explorer_node',
                    name='frontier_explorer_node',
                    output='screen',
                    parameters=explorer_parameters,
                    arguments=['--ros-args', '--log-level', log_level],
                    emulate_tty=True,
                ))

    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='autonomy_debug_rviz',
            output='screen',
            arguments=[
                '-d', PathJoinSubstitution([pkg_share, 'rviz', 'autonomy_debug.rviz'])],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ))

    return actions


def generate_launch_description():
    pkg_share = FindPackageShare('astribot_s1_autonomy')

    declare_args = [
        DeclareLaunchArgument(
            'perception_params_file',
            default_value=PathJoinSubstitution(
                [pkg_share, 'config', 'pointcloud_slice_scan_params.yaml']),
            description='感知模块参数 yaml。'),
        DeclareLaunchArgument(
            'explorer_params_file',
            default_value=PathJoinSubstitution(
                [pkg_share, 'config', 'frontier_explorer_params.yaml']),
            description='探索模块参数 yaml。'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真必须为 true。'),
        DeclareLaunchArgument(
            'use_composition', default_value='true',
            description='true=两节点装进同一 component_container(进程内零拷贝); '
                        'false=拆成两个独立进程，便于单独 gdb。'),
        DeclareLaunchArgument(
            'enable_perception', default_value='true',
            description='是否启动感知模块。'),
        DeclareLaunchArgument(
            'enable_explorer', default_value='true',
            description='是否启动探索模块。'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='是否同时开一个预配好所有调试 Marker 的 RViz。'),
        DeclareLaunchArgument(
            'log_level', default_value=default_log_level(),
            description='日志级别 debug/info/warn/error。'),
        DeclareLaunchArgument(
            'output_scan_topic', default_value='',
            description='覆盖感知输出话题；设成 /scan 可直接顶替既有单层切片方案。'),
        DeclareLaunchArgument(
            'input_cloud_topic', default_value='',
            description='覆盖感知输入点云话题。'),
        DeclareLaunchArgument(
            'goal_topic', default_value='',
            description='覆盖探索目标输出话题。'),
    ]

    return LaunchDescription(declare_args + [OpaqueFunction(function=_build)])
