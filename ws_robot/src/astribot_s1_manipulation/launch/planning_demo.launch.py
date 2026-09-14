#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：一键拉起「move_group + 规划 demo 节点」。

前置条件：
    机器人模型必须已经在跑（robot_state_publisher 提供 /robot_description 与 TF，
    /joint_states 提供当前关节值）。两种满足方式：
      1. 仿真：先起 astribot_s1_gazebo_bringup 或 nav2_full_bringup（含 Gazebo）
      2. 只看规划不看执行：单独起 robot_state_publisher + joint_state_publisher

    没有 /joint_states 时 demo 会在初始化阶段报
    "current state unavailable" 并优雅退出（不会段错误）。

用法：
    # 只规划、不执行（首次验证建议这样）
    ros2 launch astribot_s1_manipulation planning_demo.launch.py

    # 切规划器
    ros2 launch astribot_s1_manipulation planning_demo.launch.py planner_id:=BITstarConfig

    # 规划并真的下发到 Gazebo 执行
    ros2 launch astribot_s1_manipulation planning_demo.launch.py execute:=true

    # 不重复起 move_group（比如已经单独起过了）
    ros2 launch astribot_s1_manipulation planning_demo.launch.py launch_move_group:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _to_bool(text):
    lowered = text.strip().lower()
    if lowered in ('true', '1', 'yes'):
        return True
    if lowered in ('false', '0', 'no'):
        return False
    raise RuntimeError(f'布尔参数只接受 true/false，收到: {text!r}')


def _build_demo(context, *args, **kwargs):
    """留空的命令行参数不下发，避免把 yaml 里的正确值冲掉。"""
    overrides = {}

    planner_id = LaunchConfiguration('planner_id').perform(context)
    if planner_id:
        overrides['planner_id'] = planner_id

    dual_arm_group = LaunchConfiguration('dual_arm_group').perform(context)
    if dual_arm_group:
        overrides['dual_arm_group'] = dual_arm_group

    execute = LaunchConfiguration('execute').perform(context)
    if execute:
        overrides['demo.execute_trajectory'] = _to_bool(execute)

    planning_attempts = LaunchConfiguration('planning_attempts').perform(context)
    if planning_attempts:
        overrides['planning_attempts'] = int(planning_attempts)

    move_to_ready = LaunchConfiguration('move_to_ready').perform(context)
    if move_to_ready:
        overrides['demo.move_to_ready_first'] = _to_bool(move_to_ready)

    scenarios = LaunchConfiguration('scenarios').perform(context)
    if scenarios:
        overrides['demo.scenarios'] = [s for s in scenarios.split(',') if s]

    nav_goal = LaunchConfiguration('nav_goal').perform(context)
    if nav_goal:
        parts = [p for p in nav_goal.split(',') if p.strip()]
        if len(parts) not in (2, 3):
            raise RuntimeError(
                f'nav_goal 需要 "x,y" 或 "x,y,yaw"，收到: {nav_goal!r}')
        try:
            numbers = [float(p) for p in parts]
        except ValueError as exc:
            raise RuntimeError(f'nav_goal 里有非数字: {nav_goal!r}') from exc
        overrides['demo.mobile_transport_nav_goal_xy'] = numbers[:2]
        if len(numbers) == 3:
            overrides['demo.mobile_transport_nav_goal_yaw'] = numbers[2]

    use_sim_time = _to_bool(LaunchConfiguration('use_sim_time').perform(context))
    overrides['use_sim_time'] = use_sim_time

    params_file = LaunchConfiguration('params_file').perform(context)

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'urdf', 'astribot_s1.xacro']),
        ' robot_name:=astribot_s1',
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
    }

    moveit_config_share = get_package_share_directory('astribot_s1_moveit_config')
    with open(os.path.join(moveit_config_share, 'config', 'astribot_s1.srdf'),
              'r', encoding='utf-8') as handle:
        robot_description_semantic = {'robot_description_semantic': handle.read()}
    with open(os.path.join(moveit_config_share, 'config', 'kinematics.yaml'),
              'r', encoding='utf-8') as handle:
        robot_description_kinematics = {
            'robot_description_kinematics': yaml.safe_load(handle),
        }
    with open(os.path.join(moveit_config_share, 'config', 'joint_limits.yaml'),
              'r', encoding='utf-8') as handle:
        robot_description_planning = {
            'robot_description_planning': yaml.safe_load(handle),
        }

    demo_node = Node(
        package='astribot_s1_manipulation',
        executable='planning_demo_node',
        name='planning_demo_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            overrides,
        ],
        arguments=[
            '--ros-args', '--log-level',
            LaunchConfiguration('log_level').perform(context),
        ],
    )

    return [
        demo_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=demo_node,
                on_exit=[EmitEvent(event=Shutdown(reason='planning_demo_node finished'))],
            )
        ),
    ]


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('astribot_s1_manipulation'),
        'config', 'manipulation_params.yaml',
    ])

    declared_args = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='demo 参数 yaml，默认用本包 config 下的版本。'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真必须 true。'),
        DeclareLaunchArgument(
            'launch_move_group', default_value='true',
            description='是否同时拉起 move_group。已单独起过时设 false。'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='是否拉起带 MoveIt 插件的 RViz。'),
        DeclareLaunchArgument(
            'planner_id', default_value='',
            description='覆盖规划器：RRTstarConfig / BITstarConfig / '
                        'InformedRRTstarConfig / RRTConnectConfig。留空用 yaml 值。'),
        DeclareLaunchArgument(
            'dual_arm_group', default_value='',
            description='覆盖双臂组：dual_arm(14 DOF) 或 dual_arm_with_torso(18 DOF)。'
                        '留空用 yaml 值。'),
        DeclareLaunchArgument(
            'execute', default_value='',
            description='是否把规划出的轨迹下发执行(true/false)。留空用 yaml 值。'
                        '首次验证建议留空(yaml 里默认 false)，先看规划指标。'),
        DeclareLaunchArgument(
            'scenarios', default_value='',
            description='逗号分隔的场景列表，覆盖 yaml。可选：single_arm_named,'
                        'single_arm_joint,closed_chain,planner_comparison,'
                        'transport,transport_probe,mobile_transport'),
        DeclareLaunchArgument(
            'nav_goal', default_value='',
            description='mobile_transport 的导航目标，格式 "x,y" 或 "x,y,yaw"（map 系，'
                        'yaw 单位 rad）。留空用 yaml 值。\n'
                        '换目标点时务必先确认两件事，否则容易得到"机器人走不动"的假结论：\n'
                        '  1) 该点净空 > 1.62m（MPPI 足迹代价在更窄处会饱和成零梯度）\n'
                        '  2) ComputePathToPose 能出全局路径\n'
                        '反复跑对比实验时用它在起点/终点之间来回换，'
                        '否则第二轮起机器人已经在目标上、导航段等于没跑。'),
        DeclareLaunchArgument(
            'planning_attempts', default_value='',
            description='覆盖 OMPL 并行规划次数。留空用 yaml 值。\n'
                        '!!! 实测重要 !!!：>1 时 MoveIt 走 ParallelPlan，'
                        'InformedRRTstar 会因为 informed 采样器在懒惰目标采样线程'
                        '还没产出任何目标状态时就被创建而抛异常'
                        '(PathLengthDirectInfSampler: There must be at least 1 start '
                        'and 1 goal state)。用 InformedRRTstar 时设 1。'),
        DeclareLaunchArgument(
            'move_to_ready', default_value='',
            description='是否先把双臂摆到 ready 姿态(true/false)。留空用 yaml 值。'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='demo 节点日志级别。'),
    ]

    move_group = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('astribot_s1_moveit_config'),
                        'launch', 'move_group.launch.py',
                    ])),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'use_rviz': LaunchConfiguration('use_rviz'),
                }.items(),
            ),
        ],
        condition=IfCondition(LaunchConfiguration('launch_move_group')),
    )

    return LaunchDescription(
        declared_args + [move_group, OpaqueFunction(function=_build_demo)])
