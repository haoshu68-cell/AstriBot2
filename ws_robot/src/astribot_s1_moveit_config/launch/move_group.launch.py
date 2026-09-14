#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：拉起 move_group（Astribot S1 双臂 MoveIt2 规划服务）。

!!! 关键点：planning_plugin 指向本工程的自定义插件，不是官方那个 !!!
    MoveIt2 Humble 官方的 ompl_interface/OMPLPlanner 只注册了 25 个规划器，
    实测三个必需规划器里只有 RRTstar，BITstar / InformedRRTstar 都没注册。
    astribot_s1_manipulation/OmplPlannerExtension 用公开 API 把它们补注册进去。

    如果这里改回 ompl_interface/OMPLPlanner，ompl_planning.yaml 里的
    BITstarConfig / InformedRRTstarConfig 会被判为未知规划器而**静默回退**
    到组默认规划器，不报任何错 —— 于是"以为在跑 BIT*、实际跑的是 RRTConnect"
    完全看不出来。所以切换插件后一定要看 move_group 日志里打印的规划器注册结果。

用法：
    # 只起 move_group（需要 robot_state_publisher 已在跑，比如仿真已启动）
    ros2 launch astribot_s1_moveit_config move_group.launch.py

    # 带 RViz
    ros2 launch astribot_s1_moveit_config move_group.launch.py use_rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _load_yaml(package_name, relative_path):
    """读取 yaml 为 dict。找不到文件时返回 None 而不是抛异常，
    这样 launch 能给出明确的错误提示而不是一段 traceback。"""
    try:
        package_path = get_package_share_directory(package_name)
        absolute_path = os.path.join(package_path, relative_path)
        with open(absolute_path, 'r', encoding='utf-8') as handle:
            return yaml.safe_load(handle)
    except (EnvironmentError, yaml.YAMLError) as exc:
        print(f'[move_group.launch.py] 无法读取 {package_name}/{relative_path}: {exc}')
        return None


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真必须 true，否则轨迹时间戳与 /clock 不一致，执行会立刻超时。'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='是否同时拉起带 MoveIt 显示插件的 RViz。'),
        DeclareLaunchArgument(
            'planning_plugin',
            default_value='astribot_s1_manipulation/OmplPlannerExtension',
            description='规划器插件。默认用本工程的扩展插件（额外注册 BIT*/Informed RRT*）；'
                        '改成 ompl_interface/OMPLPlanner 会让那两个规划器静默失效。'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='move_group 日志级别。想看规划器注册细节与每次请求的规划器名用 info；'
                        '想看每个候选点被拒原因用 debug。'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'urdf', 'astribot_s1.xacro']),
        ' robot_name:=astribot_s1',
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
    }

    srdf_path = os.path.join(
        get_package_share_directory('astribot_s1_moveit_config'),
        'config', 'astribot_s1.srdf')
    with open(srdf_path, 'r', encoding='utf-8') as handle:
        robot_description_semantic = {'robot_description_semantic': handle.read()}

    kinematics = _load_yaml('astribot_s1_moveit_config', 'config/kinematics.yaml') or {}
    joint_limits = _load_yaml('astribot_s1_moveit_config', 'config/joint_limits.yaml') or {}
    ompl_planning = _load_yaml('astribot_s1_moveit_config', 'config/ompl_planning.yaml') or {}
    controllers = _load_yaml(
        'astribot_s1_moveit_config', 'config/moveit_controllers.yaml') or {}

    robot_description_kinematics = {'robot_description_kinematics': kinematics}
    robot_description_planning = {'robot_description_planning': joint_limits}

    planning_pipeline = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugin': LaunchConfiguration('planning_plugin'),
            'request_adapters': ' '.join([
                'default_planner_request_adapters/AddTimeOptimalParameterization',
                'default_planner_request_adapters/ResolveConstraintFrames',
                'default_planner_request_adapters/FixWorkspaceBounds',
                'default_planner_request_adapters/FixStartStateBounds',
                'default_planner_request_adapters/FixStartStateCollision',
                'default_planner_request_adapters/FixStartStatePathConstraints',
            ]),
            'start_state_max_bounds_error': 0.1,
        },
    }
    planning_pipeline['ompl'].update(ompl_planning)

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline,
            controllers,
            {'use_sim_time': use_sim_time},
            {'publish_robot_description_semantic': True},
            {'publish_planning_scene': True},
            {'publish_geometry_updates': True},
            {'publish_state_updates': True},
            {'publish_transforms_updates': True},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('astribot_s1_moveit_config'), 'rviz', 'moveit.rviz']),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(
        declared_args + [GroupAction(scoped=True, actions=[move_group_node, rviz_node])])
