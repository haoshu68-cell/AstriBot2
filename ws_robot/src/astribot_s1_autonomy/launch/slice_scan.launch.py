#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：只启动【感知模块】——3D 点云多层高度切片投影生成 2D LaserScan。

全部使用 ROS2 Humble 原生 Python Launch API，路径统一走 FindPackageShare，
不出现任何硬编码绝对路径。

用法示例：
    # 用默认参数（输入 /livox/fused_points，输出 /scan_from_cloud）
    ros2 launch astribot_s1_autonomy slice_scan.launch.py

    # 换输入点云、关掉 Marker
    ros2 launch astribot_s1_autonomy slice_scan.launch.py \\
        input_cloud_topic:=/livox/lidar_left publish_markers:=false

    # 让本节点直接顶替既有 /scan，Nav2 不改配置即可吃到多层切片结果
    ros2 launch astribot_s1_autonomy slice_scan.launch.py output_scan_topic:=/scan

    # 用自己的参数文件
    ros2 launch astribot_s1_autonomy slice_scan.launch.py params_file:=/path/to/my.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 支持从命令行覆盖的参数名 → 值类型。
# 留空（默认值 ''）表示「不覆盖」，从而保留 yaml 里的配置。
_OVERRIDABLE = {
    'input_cloud_topic': str,
    'output_scan_topic': str,
    'base_frame': str,
    'publish_markers': bool,
    'invalid_input_policy': str,
}


def _to_bool(text):
    """把 launch 参数的字符串转成 bool，非法值直接报错而不是静默当 False。"""
    lowered = text.strip().lower()
    if lowered in ('true', '1', 'yes'):
        return True
    if lowered in ('false', '0', 'no'):
        return False
    raise RuntimeError(f'布尔参数只接受 true/false，收到: {text!r}')


def _build_nodes(context, *args, **kwargs):
    """
    用 OpaqueFunction 是因为：只有在这里才能拿到参数的**实际字符串值**，
    才能判断「使用者到底有没有传这个参数」。
    直接把 LaunchConfiguration 塞进 parameters 的话，没传的参数会变成空字符串，
    反而把 yaml 里的正确值冲掉——那是个很隐蔽的坑。
    """
    overrides = {}
    for name, value_type in _OVERRIDABLE.items():
        raw = LaunchConfiguration(name).perform(context)
        if raw == '':
            continue    # 使用者没传 → 保留 yaml 的值
        overrides[name] = _to_bool(raw) if value_type is bool else raw

    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = _to_bool(LaunchConfiguration('use_sim_time').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)

    # parameters 列表按顺序生效，后面的覆盖前面的：
    # yaml → use_sim_time → 命令行覆盖项
    parameters = [params_file, {'use_sim_time': use_sim_time}]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package='astribot_s1_autonomy',
            executable='pointcloud_slice_scan_node',
            name='pointcloud_slice_scan_node',
            output='screen',
            parameters=parameters,
            arguments=['--ros-args', '--log-level', log_level],
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('astribot_s1_autonomy'), 'config',
         'pointcloud_slice_scan_params.yaml'])

    declare_args = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='感知模块参数 yaml，默认用本包 config 下的版本。'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真必须为 true。否则节点用系统时钟、'
                        '点云时间戳校验会把每一帧都判为超时丢弃。'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='日志级别 debug/info/warn/error。调切片参数时建议用 debug。'),
        # 以下留空即「不覆盖 yaml」
        DeclareLaunchArgument(
            'input_cloud_topic', default_value='',
            description='覆盖输入点云话题；留空用 yaml 值。'),
        DeclareLaunchArgument(
            'output_scan_topic', default_value='',
            description='覆盖输出 LaserScan 话题；留空用 yaml 值。'
                        '设成 /scan 可直接顶替既有单层切片方案。'),
        DeclareLaunchArgument(
            'base_frame', default_value='',
            description='覆盖投影本体坐标系；留空用 yaml 值'
                        '(本机器人为 astribot_torso_base，没有 base_link)。'),
        DeclareLaunchArgument(
            'publish_markers', default_value='',
            description='覆盖是否发布调试 Marker (true/false)；留空用 yaml 值。'),
        DeclareLaunchArgument(
            'invalid_input_policy', default_value='',
            description='覆盖输入异常策略 hold_last/stop_output；留空用 yaml 值。'),
    ]

    return LaunchDescription(declare_args + [OpaqueFunction(function=_build_nodes)])
