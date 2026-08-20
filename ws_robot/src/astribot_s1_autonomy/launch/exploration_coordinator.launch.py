#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：启动【探索协调器】——严格时序探索调度 + 未知区域禁行强校验。

和 frontier_explore.launch.py 的区别（很容易混）：
    frontier_explore        —— 候选点建议流。每个规划周期都发一次 /explore/goal_pose，
                               不管上一个目标走到哪了。适合调参和看 Marker。
    exploration_coordinator —— 调度器。必须完全抵达并稳定驻留当前目标才生成下一个，
                               且目标点与全局路径都要通过未知区禁行校验。
                               它自己调 Nav2 的 action，是真正驱动机器人的那个。
两者不要同时启动：会有两个源在往 Nav2 塞目标，正是需求禁止的「重叠下发」。

前置条件：Nav2 必须已经在跑（需要 navigate_to_pose 和 compute_path_to_pose 两个 action），
SLAM 必须已经在发 /map。协调器起得比 Nav2 早没问题，它会等到 action server 就绪。

用法示例：
    ros2 launch astribot_s1_autonomy exploration_coordinator.launch.py
    ros2 launch astribot_s1_autonomy exploration_coordinator.launch.py log_level:=debug
    ros2 launch astribot_s1_autonomy exploration_coordinator.launch.py dwell_time_sec:=3.0
    ros2 launch astribot_s1_autonomy exploration_coordinator.launch.py params_file:=/path/my.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 只暴露最常在现场调的几个：其余一律改 yaml。
# 需求要求所有阈值外置可配置，命令行覆盖是补充手段，不是参数的正式来源。
_OVERRIDABLE = {
    'map_topic': str,
    'odom_topic': str,
    'robot_base_frame': str,
    'arrival_xy_tolerance': float,
    'dwell_time_sec': float,
    'nav_timeout_sec': float,
    'check_yaw': bool,
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
            package='astribot_s1_autonomy',
            executable='exploration_coordinator_node',
            name='exploration_coordinator_node',
            output='screen',
            parameters=parameters,
            arguments=['--ros-args', '--log-level', log_level],
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('astribot_s1_autonomy'), 'config',
         'exploration_coordinator_params.yaml'])

    declare_args = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='协调器参数 yaml，默认用本包 config 下的版本。'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真必须为 true，否则地图时效判定、驻留计时和 TF 查询都会异常。'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='日志级别。想看每个候选点被拒的具体原因用 debug。'),
        DeclareLaunchArgument(
            'map_topic', default_value='',
            description='覆盖占据栅格话题；留空用 yaml 值。'),
        DeclareLaunchArgument(
            'odom_topic', default_value='',
            description='覆盖里程计话题（用于驻留速度判定）；留空用 yaml 值。'),
        DeclareLaunchArgument(
            'robot_base_frame', default_value='',
            description='覆盖机器人本体坐标系；留空用 yaml 值'
                        '(本机器人为 astribot_torso_base，没有 base_link)。'),
        DeclareLaunchArgument(
            'arrival_xy_tolerance', default_value='',
            description='覆盖抵达位置容差(m)；必须 >= Nav2 的 xy_goal_tolerance。'),
        DeclareLaunchArgument(
            'dwell_time_sec', default_value='',
            description='覆盖稳定驻留时长(s)；调大可让每个目标停得更实。'),
        DeclareLaunchArgument(
            'nav_timeout_sec', default_value='',
            description='覆盖单个目标最长导航时间(s)；超时按导航失败处理。'),
        DeclareLaunchArgument(
            'check_yaw', default_value='',
            description='覆盖是否把朝向纳入抵达判定 (true/false)。'),
    ]

    return LaunchDescription(declare_args + [OpaqueFunction(function=_build_nodes)])
