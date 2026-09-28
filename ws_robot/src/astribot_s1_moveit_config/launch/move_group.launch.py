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

from astribot_logging import get_logger

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_package_prefix, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from astribot_logging import log_level as default_log_level
from astribot_logging.launch import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _fixed_camera_mount_semantic(urdf_xml, srdf_xml):
    """Allow only known mounting contacts connected entirely by fixed joints.

    Applied only with an explicit mount profile. The shared hardware SRDF is
    unchanged; no moving joint, finger, payload or world contact is exempted.
    """
    robot, semantic = ET.fromstring(urdf_xml), ET.fromstring(srdf_xml)
    fixed = {}
    for joint in robot.findall('joint'):
        if joint.get('type') == 'fixed':
            a, b = joint.find('parent').get('link'), joint.find('child').get('link')
            fixed.setdefault(a, set()).add(b)
            fixed.setdefault(b, set()).add(a)
    candidates = [(c+'_camera_link', 'astribot_head_link_2')
                  for c in ('head_rgbd', 'head_stereo_left', 'head_stereo_right')]
    candidates += [('torso_rgbd_camera_link', 'astribot_torso_link_4')]
    candidates += [(s+'_wrist_rgbd_camera_link', 'astribot_gripper_'+s+'_base')
                   for s in ('left', 'right')]
    existing = {frozenset((e.get('link1'), e.get('link2')))
                for e in semantic.findall('disable_collisions')}
    for a, b in candidates:
        pending, seen = [a], set()
        while pending:
            current = pending.pop()
            if current not in seen:
                seen.add(current)
                pending.extend(fixed.get(current, ()))
        if b in seen and frozenset((a, b)) not in existing:
            ET.SubElement(semantic, 'disable_collisions', link1=a, link2=b, reason='Adjacent')
    return ET.tostring(semantic, encoding='unicode')


def _prepare_robot_descriptions(context):
    # sensors_3d.yaml always enables the pointcloud updater. Fail before
    # move_group starts instead of silently planning with an empty OctoMap.
    try:
        sensor_prefix = get_package_prefix('astribot_s1_manipulation')
    except PackageNotFoundError as error:
        raise RuntimeError('Missing astribot_s1_manipulation: camera obstacle updater unavailable') from error
    if not os.path.isfile(os.path.join(sensor_prefix, 'lib', 'libastribot_observed_pointcloud_updater.so')):
        raise RuntimeError('Missing ObservedPointCloudUpdater library in ' + sensor_prefix)
    content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'urdf', 'astribot_s1.xacro']),
        ' robot_name:=astribot_s1',
        ' use_lidar:=', LaunchConfiguration('use_lidar'),
        ' use_camera:=', LaunchConfiguration('use_camera'),
        ' use_wrist_cameras:=', LaunchConfiguration('use_wrist_cameras'),
        ' use_stereo_cameras:=', LaunchConfiguration('use_stereo_cameras'),
        ' camera_profile:=', LaunchConfiguration('camera_profile'),
        ' torso_camera_profile:=', LaunchConfiguration('torso_camera_profile'),
        ' camera_calibration_dir:=', LaunchConfiguration('camera_calibration_dir'),
        ' camera_mounts_profile:="', LaunchConfiguration('camera_mounts_profile'), '"',
    ]).perform(context)
    srdf_path = os.path.join(get_package_share_directory('astribot_s1_moveit_config'),
                             'config', 'astribot_s1.srdf')
    with open(srdf_path, encoding='utf-8') as handle:
        semantic = handle.read()
    if LaunchConfiguration('camera_mounts_profile').perform(context):
        semantic = _fixed_camera_mount_semantic(content, semantic)
    return [SetLaunchConfiguration('resolved_robot_description', content),
            SetLaunchConfiguration('resolved_robot_semantic', semantic)]


def _load_yaml(package_name, relative_path):
    """读取 yaml 为 dict。找不到文件时返回 None 而不是抛异常，
    这样 launch 能给出明确的错误提示而不是一段 traceback。"""
    try:
        package_path = get_package_share_directory(package_name)
        absolute_path = os.path.join(package_path, relative_path)
        with open(absolute_path, 'r', encoding='utf-8') as handle:
            return yaml.safe_load(handle)
    except (EnvironmentError, yaml.YAMLError) as exc:
        get_logger('astribot.move_group').error(f'[move_group.launch.py] 无法读取 {package_name}/{relative_path}: {exc}')
        return None


def _controllers_for_execution(controllers, enabled):
    """Planning-only instances must not create clients of physical controllers."""
    import copy
    selected = copy.deepcopy(controllers)
    if not enabled:
        # Humble launch rejects an untyped empty list. Omitting the manager's
        # configuration makes its initialize() return before creating handles.
        # Its "No controller_names specified" log is expected in this mode.
        selected.pop('moveit_simple_controller_manager', None)
    return selected


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('extra_capabilities', default_value=''),
        # Match the physical attachments selected by the simulation launcher.
        # Defaults preserve the standalone full-robot model.
        *[DeclareLaunchArgument(name, default_value='true', choices=['true', 'false'])
          for name in ('use_lidar', 'use_camera', 'use_wrist_cameras', 'use_stereo_cameras')],
        DeclareLaunchArgument('disable_capabilities', default_value=''),
        DeclareLaunchArgument('allow_trajectory_execution', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真必须 true，否则轨迹时间戳与 /clock 不一致，执行会立刻超时。'),
        DeclareLaunchArgument('camera_profile', default_value=PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'config', 'camera_rgbd_transport.yaml']),
            description='默认使用最新机器人标定的头部 RGB-D profile；需要完整六路相机时由仿真启动传入 camera_calibration_dir。'),
        DeclareLaunchArgument('torso_camera_profile', default_value=PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'config', 'camera_torso_rgbd.yaml'])),
        DeclareLaunchArgument('camera_calibration_dir', default_value=PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'config'])),
        DeclareLaunchArgument('camera_mounts_profile', default_value=PythonExpression([
            "'", PathJoinSubstitution([FindPackageShare('astribot_s1_description'), 'config',
                                       'camera_mounts_reference_sim.yaml']),
            "' if '", LaunchConfiguration('use_sim_time'), "'.lower() == 'true' else ''"]),
            description='仿真与warehouse使用同一安装配置；真机默认空，不使用临时参考安装'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='是否同时拉起带 MoveIt 显示插件的 RViz。'),
        DeclareLaunchArgument(
            'planning_plugin',
            default_value='astribot_s1_manipulation/OmplPlannerExtension',
            description='规划器插件。默认用本工程的扩展插件（额外注册 BIT*/Informed RRT*）；'
                        '改成 ompl_interface/OMPLPlanner 会让那两个规划器静默失效。'),
        DeclareLaunchArgument(
            'log_level', default_value=default_log_level(),
            description='move_group 日志级别。想看规划器注册细节与每次请求的规划器名用 info；'
                        '想看每个候选点被拒原因用 debug。'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = {
        'robot_description': ParameterValue(LaunchConfiguration('resolved_robot_description'), value_type=str),
    }
    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        LaunchConfiguration('resolved_robot_semantic'), value_type=str)}

    kinematics = _load_yaml('astribot_s1_moveit_config', 'config/kinematics.yaml') or {}
    joint_limits = _load_yaml('astribot_s1_moveit_config', 'config/joint_limits.yaml') or {}
    ompl_planning = _load_yaml('astribot_s1_moveit_config', 'config/ompl_planning.yaml') or {}
    controllers = _load_yaml(
        'astribot_s1_moveit_config', 'config/moveit_controllers.yaml') or {}
    sensors_3d = _load_yaml(
        'astribot_s1_moveit_config', 'config/sensors_3d.yaml') or {}

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

    def _start_move_group(context):
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
                _controllers_for_execution(controllers,
                    LaunchConfiguration('allow_trajectory_execution').perform(context).lower() == 'true'),
                sensors_3d,
                {'use_sim_time': use_sim_time},
                {'allow_trajectory_execution': ParameterValue(LaunchConfiguration('allow_trajectory_execution'), value_type=bool)},
                {'capabilities': LaunchConfiguration('extra_capabilities'),
                 'disable_capabilities': LaunchConfiguration('disable_capabilities')},
                {'publish_robot_description_semantic': True},
                {'publish_planning_scene': True},
                {'publish_geometry_updates': True},
                {'publish_state_updates': True},
                {'publish_transforms_updates': True},
            ],
        )
        return [move_group_node]

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
        declared_args + [OpaqueFunction(function=_prepare_robot_descriptions),
                         GroupAction(scoped=True, actions=[OpaqueFunction(function=_start_move_group), rviz_node])])
