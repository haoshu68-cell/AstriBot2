#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图来源分发：按 config/map_source.yaml 决定 /map 与 map→odom 谁来提供。

============================ 两个正交的轴 ============================
    map_source   —— 谁提供静态 /map    : real_file | real_live
    localization —— 谁提供 map→odom    : slam | ground_truth

========================= 为什么这里可以跑 map_server =========================
静态地图模式只启动一个地图发布者，并由本模块负责必要的 map→odom：

    /map      由 map_server（real_file）或 map_domain_relay（real_live）提供
    map→odom  由静态 TF 提供（ground_truth）

各只有一个发布者，不存在抢发布权。全局代价地图的 static_layer 本来就是
`map_subscribe_transient_local: True`，对两种来源都适用，代价地图侧一行不用改。

用法：
    # 用配置文件里的值
    ros2 launch astribot_s1_perception map_provider.launch.py

    # 临时覆盖（留空的参数不下发，不会把 yaml 里的正确值冲掉）
    ros2 launch astribot_s1_perception map_provider.launch.py \
        map_source:=real_file map_yaml_path:=/abs/path/to/map.yaml
"""

from astribot_logging import get_logger

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from astribot_logging.launch import Node

from ament_index_python.packages import get_package_share_directory

from astribot_s1_perception.map_source_config import (
    MapSourceConfigError,
    check_live_transport_env,
    publishes_static_map_to_odom,
    require,
    resolve,
)


def _build(context, *args, **kwargs):
    overrides = {
        key: LaunchConfiguration(key).perform(context)
        for key in ('map_source', 'localization', 'map_yaml_path')
    }
    config_file = LaunchConfiguration('map_source_file').perform(context) or None
    config_path, params, map_source, localization = resolve(config_file, overrides)

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    actions = []

    get_logger('astribot.map_provider').info(f'[map_provider] 配置文件 = {config_path}')
    get_logger('astribot.map_provider').info(f'[map_provider] map_source = {map_source}  localization = {localization}')

    if map_source == 'real_file':
        map_yaml = require(params, 'map_yaml_path', str, default='')
        if not map_yaml:
            raise MapSourceConfigError(
                'map_source=real_file 但 map_yaml_path 为空。'
                '要给 **yaml** 的绝对路径（不是 pgm）—— map_server 读的是 yaml，'
                '其中的 image 字段相对该 yaml 所在目录解析。')
        if not os.path.isfile(map_yaml):
            raise MapSourceConfigError(f'map_yaml_path 指向的文件不存在: {map_yaml}')

        actions.append(Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': use_sim_time == 'true',
            }],
        ))
        actions.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time == 'true',
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ))
        get_logger('astribot.map_provider').info(f'[map_provider] /map 由 map_server 提供: {map_yaml}')

    else:  # real_live
        transport_reason = check_live_transport_env(
            os.environ.get('ROS_LOCALHOST_ONLY', '0'))
        if transport_reason is not None:
            raise MapSourceConfigError(transport_reason)

        remote_domain = require(params, 'remote_domain_id', int, 25)
        local_domain = require(params, 'local_domain_id', int, 25)
        if remote_domain == local_domain:
            get_logger('astribot.map_provider').info(f'[map_provider] /map 由真机直接提供（remote/local 同为 domain '
                  f'{local_domain}，跳过 map_domain_relay）')
            get_logger('astribot.map_provider').info('[map_provider] !!! 跨域隔离已关闭：本机的 /cmd_vel、'
                  '/wheel_effort_controller/commands 对真机可见。'
                  '要恢复网络层隔离就把 local_domain_id 改成与真机不同的值，'
                  '并把整条栈起在那个 domain 上 !!!')
        else:
            relay = Node(
                package='astribot_s1_perception_native',
                executable='map_domain_relay',
                name='map_domain_relay',
                output='screen',
                parameters=[{
                    'remote_domain_id': remote_domain,
                    'local_domain_id': local_domain,
                    'remote_map_topic': require(params, 'remote_map_topic', str, '/map'),
                    'local_map_topic': require(params, 'local_map_topic', str, '/map'),
                    'relay_timeout_sec': require(params, 'relay_timeout_sec', float, 30.0),
                }],
            )
            actions.append(relay)
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=relay, on_exit=[EmitEvent(event=Shutdown(
                    reason='map_domain_relay 退出：跨机地图中继失败'))])))
            get_logger('astribot.map_provider').info(f'[map_provider] /map 由 map_domain_relay 跨机中继提供: '
                  f'domain {remote_domain} -> {local_domain}')

    if publishes_static_map_to_odom(localization):
        pose = params.get('map_to_odom_xyz_yaw') or [0.0, 0.0, 0.0, 0.0]
        if len(pose) != 4:
            raise MapSourceConfigError(
                f'map_to_odom_xyz_yaw 必须是 4 个数 [x, y, z, yaw]，收到 {len(pose)} 个')
        x, y, z, yaw = (float(v) for v in pose)
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static',
            output='screen',
            arguments=[
                '--x', str(x), '--y', str(y), '--z', str(z),
                '--yaw', str(yaw), '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            parameters=[{'use_sim_time': use_sim_time == 'true'}],
        ))
        get_logger('astribot.map_provider').info(f'[map_provider] map→odom 由静态 TF 提供: '
              f'xyz=({x}, {y}, {z}) yaw={yaw}')

        if params.get('validate_start_cell', True):
            checker = Node(
                package='astribot_s1_perception',
                executable='map_start_cell_check',
                name='map_start_cell_check',
                output='screen',
                parameters=[{
                    'map_topic': require(params, 'local_map_topic', str, '/map'),
                    'map_to_odom_xyz_yaw': [x, y, z, yaw],
                    'start_cell_clearance_m': require(
                        params, 'start_cell_clearance_m', float, 0.25),
                    'use_sim_time': use_sim_time == 'true',
                }],
            )
            actions.append(checker)
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=checker,
                on_exit=lambda event, ctx: (
                    [EmitEvent(event=Shutdown(
                        reason='出生点栅格校验未通过（见上面的 ERROR）'))]
                    if event.returncode != 0 else []))))
    else:
        get_logger('astribot.map_provider').info('[map_provider] 外部定位负责 map→odom')

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_source_file', default_value='',
            description='地图来源配置文件路径。留空用包内 config/map_source.yaml'),
        DeclareLaunchArgument(
            'map_source', default_value='',
            description='覆盖配置里的 map_source（real_file|real_live）。'
                        '留空则用配置文件的值'),
        DeclareLaunchArgument(
            'localization', default_value='',
            description='覆盖配置里的 localization（slam|ground_truth|external）。留空同上'),
        DeclareLaunchArgument(
            'map_yaml_path', default_value='',
            description='覆盖配置里的 map_yaml_path。留空同上'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_build),
    ])
