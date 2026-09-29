#!/usr/bin/env python3
"""Unified Voxel-SLAM backend for simulation and hardware."""

from pathlib import Path
import math
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from astribot_logging.launch import Node
from astribot_logging import log_level


def _build_voxel(context, *args, **kwargs):
    def vector(name, size):
        values = [float(v.strip()) for v in
                  LaunchConfiguration(name).perform(context).split(',')]
        if len(values) != size or not all(math.isfinite(v) for v in values):
            raise RuntimeError(f'{name} 需要 {size} 个数，收到 {values!r}')
        return values

    save_path = Path(LaunchConfiguration('save_path').perform(context)).expanduser().resolve()
    map_name = LaunchConfiguration('map_name').perform(context)
    previous = LaunchConfiguration('previous_map').perform(context)
    initial_pose = LaunchConfiguration('initial_chassis_pose').perform(context).strip()
    initial_override = {}
    if initial_pose:
        values = vector('initial_chassis_pose', 7)
        if previous:
            raise RuntimeError('initial_chassis_pose 不能叠加到 previous_map 载图定位')
        if abs(sum(v * v for v in values[3:]) - 1.0) > 1e-6:
            raise RuntimeError('initial_chassis_pose 四元数必须归一化')
        initial_override['General.initial_chassis_pose'] = values
    save_map = int(LaunchConfiguration('save_map').perform(context))
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('mapping', 'localization'):
        raise RuntimeError('mode 必须为 mapping 或 localization')
    if save_map not in (0, 1):
        raise RuntimeError('save_map 必须为 0 或 1')
    if map_name and not re.fullmatch(r'[A-Za-z0-9_-]+', map_name):
        raise RuntimeError('map_name 仅允许字母、数字、下划线和短横线')
    if save_map and (not map_name or (save_path / map_name).exists()):
        raise RuntimeError('存图必须指定尚不存在的 map_name；禁止覆盖既有会话')
    if mode == 'localization' and not previous:
        raise RuntimeError('localization 必须指定 previous_map，会话不能静默退回空图建图')
    if ',' in previous:
        raise RuntimeError('统一导航当前只接受一份已闭环的存图会话；禁止混合未对齐地图')
    for entry in filter(None, previous.split(',')):
        name, sep, threshold = entry.strip().partition(':')
        if not sep or not re.fullmatch(r'[A-Za-z0-9_-]+', name):
            raise RuntimeError('previous_map 格式为 session_name:threshold')
        if not math.isfinite(float(threshold)) or not 0 < float(threshold) < 1:
            raise RuntimeError('previous_map 匹配阈值须在 (0, 1) 范围内')
        session = save_path / name
        from astribot_s1_perception.slam_session import inspect_session
        try:
            inspect_session(session)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise RuntimeError(f'Voxel 会话不完整或校验失败: {session}: {exc}') from exc
    if save_map:
        save_path.mkdir(parents=True, exist_ok=True)

    voxel = Node(
        package='astribot_s1_slam',
        executable='voxelslam',
        name='voxelslam',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('astribot_s1_slam'), 'config', 'mid360.yaml']),
            PathJoinSubstitution([FindPackageShare('astribot_s1_perception_components'), 'config', 'self_filter.yaml']), {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'General.lid_topic': LaunchConfiguration('lidar_topic').perform(context),
            'General.lid_topic_back': LaunchConfiguration('lidar_topic_back').perform(context),
            'General.imu_topic': LaunchConfiguration('imu_topic').perform(context),
            # Simulation sensors are synchronized; do not replay an old DDS
            # backlog into navigation after a slow optimization iteration.
            'General.lidar_queue_depth': 1 if LaunchConfiguration('use_sim_time').perform(context).lower() == 'true' else 1000,
            'General.pending_scan_limit': 1 if LaunchConfiguration('use_sim_time').perform(context).lower() == 'true' else 0,
            'General.save_path': str(save_path) + '/',
            'General.previous_map': previous,
            'General.require_grid_subscriber': LaunchConfiguration('publish_grid').perform(context).lower() == 'true',
            'General.mapname': LaunchConfiguration('map_name').perform(context),
            'General.is_save_map': int(LaunchConfiguration('save_map').perform(context)),
            'General.extrinsic_tran': vector('imu_extrinsic_tran', 3),
            'General.back_extrinsic_tran': vector('back_extrinsic_tran', 3),
            'General.back_extrinsic_rota': vector('back_extrinsic_rota', 9),
            'Odometry.point_notime': int(LaunchConfiguration('point_notime').perform(context)),
            **initial_override,
        }],
    )
    return [voxel, RegisterEventHandler(OnProcessExit(target_action=voxel,
        on_exit=lambda event, ctx: [EmitEvent(event=Shutdown(reason='Voxel-SLAM failed'))]
        if event.returncode != 0 else []))]


def generate_launch_description():
    grid_config = PathJoinSubstitution([
        FindPackageShare('astribot_s1_mapping'), 'config', 'nav_prob_grid.yaml'])
    use_sim_time = LaunchConfiguration('use_sim_time')

    grid = Node(
        package='astribot_s1_mapping', executable='nav_prob_grid_node',
        name='nav_prob_grid_node', output='screen',
        condition=IfCondition(LaunchConfiguration('publish_grid')),
        parameters=[grid_config, {
            'use_sim_time': use_sim_time,
            'output_topic': LaunchConfiguration('map_topic'),
        }],
    )
    map_odom = Node(
        package='astribot_s1_perception_native', executable='map_odom_tf_node',
        name='map_odom_tf', output='screen',
        condition=IfCondition(LaunchConfiguration('publish_map_odom')),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value=log_level(),
                              description='Shared ROS log level for SLAM, grid and TF adapter'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('publish_grid', default_value='true'),
        DeclareLaunchArgument('publish_map_odom', default_value='true'),
        DeclareLaunchArgument('previous_map', default_value=''),
        DeclareLaunchArgument('initial_chassis_pose', default_value='',
                              description='静止启动时实测 map<-astribot_torso_base 的 x,y,z,qx,qy,qz,qw；留空新建局部地图'),
        DeclareLaunchArgument('imu_extrinsic_tran', default_value='0,0,0'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar_left'),
        DeclareLaunchArgument('lidar_topic_back', default_value='/livox/lidar_right'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('save_path', default_value='/tmp/astribot_slam_sessions/'),
        DeclareLaunchArgument('map_name', default_value=''),
        DeclareLaunchArgument('save_map', default_value='0'),
        DeclareLaunchArgument(
            'point_notime', default_value='1',
            description='Gazebo PointCloud2 无逐点时间时设为 1；真机按驱动字段设为 0'),
        DeclareLaunchArgument(
            'back_extrinsic_tran',
            default_value='0.00099903,-0.49599628,0.084'),
        DeclareLaunchArgument(
            'back_extrinsic_rota',
            default_value='-0.9994599973,0.0315070538,0.0093284204,'
                          '-0.0314947541,-0.9995028471,0.0014625345,'
                          '0.0093698629,0.0011679485,0.9999554198'),
        OpaqueFunction(function=_build_voxel),
        grid,
        map_odom,
    ])
