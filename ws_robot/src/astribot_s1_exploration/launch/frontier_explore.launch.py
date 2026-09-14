#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：只启动【决策模块】——边界遍历 + 前沿点自适应采样自主探索。

本节点只输出目标位姿 /explore/goal_pose，不发速度指令、不内置 Nav2 客户端。
要真正驱动机器人，需要外部把这个位姿送进 Nav2 的 NavigateToPose action
（本包刻意不提供该桥接：按需求约束，Nav2 的调用属于「外部」职责，
 且 Python 在本包内只用于调试可视化脚本）。README「如何驱动机器人」一节给了具体做法。

用法示例：
    ros2 launch astribot_s1_autonomy frontier_explore.launch.py
    ros2 launch astribot_s1_autonomy frontier_explore.launch.py planning_period_sec:=1.0
    ros2 launch astribot_s1_autonomy frontier_explore.launch.py params_file:=/path/to/my.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from astribot_logging import log_level as default_log_level
from astribot_logging.launch import Node
from launch_ros.substitutions import FindPackageShare

_OVERRIDABLE = {
    'map_topic': str,
    'goal_topic': str,
    'robot_base_frame': str,
    'planning_period_sec': float,
    'publish_markers': bool,
}


def _to_bool(text):
    lowered = text.strip().lower()
    if lowered in ('true', '1', 'yes'):
        return True
    if lowered in ('false', '0', 'no'):
        return False
    raise RuntimeError(f'布尔参数只接受 true/false，收到: {text!r}')


def _build_nodes(context, *args, **kwargs):
    """留空的命令行参数不下发，避免把 yaml 里的正确值冲成空字符串。"""
    overrides = {}
    for name, value_type in _OVERRIDABLE.items():
        raw = LaunchConfiguration(name).perform(context)
        if raw == '':
            continue
        if value_type is bool:
            overrides[name] = _to_bool(raw)
        elif value_type is float:
            overrides[name] = float(raw)
        else:
            overrides[name] = raw

    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = _to_bool(LaunchConfiguration('use_sim_time').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)

    parameters = [params_file, {'use_sim_time': use_sim_time}]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package='astribot_s1_exploration',
            executable='frontier_explorer_node',
            name='frontier_explorer_node',
            output='screen',
            parameters=parameters,
            arguments=['--ros-args', '--log-level', log_level],
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('astribot_s1_exploration'), 'config',
         'frontier_explorer_params.yaml'])

    declare_args = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='探索模块参数 yaml，默认用本包 config 下的版本。'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真必须为 true，否则地图时效判定和 TF 查询都会异常。'),
        DeclareLaunchArgument(
            'log_level', default_value=default_log_level(),
            description='日志级别。想看每个前沿块被拒的原因用 debug。'),
        DeclareLaunchArgument(
            'map_topic', default_value='',
            description='覆盖占据栅格话题；留空用 yaml 值。'),
        DeclareLaunchArgument(
            'goal_topic', default_value='',
            description='覆盖目标位姿输出话题；留空用 yaml 值。'),
        DeclareLaunchArgument(
            'robot_base_frame', default_value='',
            description='覆盖机器人本体坐标系；留空用 yaml 值'
                        '(本机器人为 astribot_torso_base)。'),
        DeclareLaunchArgument(
            'planning_period_sec', default_value='',
            description='覆盖探索规划周期(s)；留空用 yaml 值。'),
        DeclareLaunchArgument(
            'publish_markers', default_value='',
            description='覆盖是否发布调试 Marker (true/false)；留空用 yaml 值。'),
    ]

    return LaunchDescription(declare_args + [OpaqueFunction(function=_build_nodes)])
