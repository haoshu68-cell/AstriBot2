#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：Nav2 核心节点组（controller_server/smoother_server/planner_server/
behavior_server/bt_navigator/waypoint_follower/velocity_smoother +
lifecycle_manager_navigation），照抄官方
`/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py` 的节点结构和
`RewrittenYaml`/`ParameterFile` 参数模式（规规矩矩复用官方已验证过的模式，不是
重新发明），**故意不包含** `map_server`/`amcl`——SLAM Toolbox 在建图和定位两种模式下
都会自己发布 `/map` 和 `map->odom` TF，跑一套 map_server+amcl 会跟它抢着发布/消费
`map->odom`，详见 astribot_s1_navigation/README_NAVIGATION.md。

!!! 跟官方文件唯一的关键差异（不是疏漏，是故意改的）!!!：官方文件里
`velocity_smoother` 最后一步把内部话题名 `cmd_vel_smoothed` 重映射成 `cmd_vel`
——也就是说官方设计下 Nav2 整套的最终输出话题名就是 `/cmd_vel`，会直接怼到真机的
`/cmd_vel`。但本机器人底盘用 gz-sim `VelocityControl` 插件、按 world 系解释
`/cmd_vel`（Nav2标准控制器按车体系发布），直接对接会重现本 session 修过的"原地打转
不挪窝"bug。这里改成把 `cmd_vel_smoothed` 重映射到 `cmd_vel_nav_body`
（还是车体系，只是话题名不同），交给 `cmd_vel_body_to_world_node`
（本包自己的节点）订阅后换算成 world 系再发真正的 `/cmd_vel`。
没有用"IncludeLaunchDescription官方文件 + 外层SetRemap覆盖"的做法——官方文件内部的
remappings是在每个Node构造时写死传入的，外层SetRemap能否正确覆盖掉这种写死的
remapping在ROS2 launch里没有保证，与其猜不如直接复制节点定义、只改这一处，明确可控。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_navigation = FindPackageShare('astribot_s1_navigation')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    controller_plugin = LaunchConfiguration('controller_plugin')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # controller_plugin(rpp|mppi) 决定用哪一份参数文件——两份文件除了
    # controller_server.FollowPath 那一段之外完全一致，见 config/ 目录下两个文件的注释。
    params_file = PathJoinSubstitution([
        pkg_navigation, 'config',
        PythonExpression(["'nav2_params_' + '", controller_plugin, "' + '.yaml'"]),
    ])

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_args = [
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'controller_plugin', default_value='rpp',
            description='rpp(任务书默认，非全向退化行为) 或 mppi(推荐，全向)'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    load_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                # controller_server 自己内部话题名是 cmd_vel，重映射成 cmd_vel_nav_body
                # 交给 velocity_smoother 平滑，最终由 velocity_smoother 再重映射一次。
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav_body_raw')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                # !!! 本文件跟官方版本唯一的关键差异，见文件头部说明 !!!：
                # 官方是 ('cmd_vel_smoothed', 'cmd_vel')，这里改成 'cmd_vel_nav_body'，
                # 不直接碰真正的 /cmd_vel。
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav_body_raw'),
                         ('cmd_vel_smoothed', 'cmd_vel_nav_body')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
            # 本包自己的两个适配节点：body->world cmd_vel 转换(必须存在，见文件头部说明)
            # + 机械臂展开限速。
            Node(
                package='astribot_s1_navigation',
                executable='cmd_vel_body_to_world_node',
                name='cmd_vel_body_to_world_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='astribot_s1_navigation',
                executable='arm_speed_limiter_node',
                name='arm_speed_limiter_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    for action in declare_args:
        ld.add_action(action)
    ld.add_action(load_nodes)
    return ld
