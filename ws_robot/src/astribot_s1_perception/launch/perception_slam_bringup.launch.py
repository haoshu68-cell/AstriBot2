#!/usr/bin/env python3
"""统一的仿真/硬件感知与 Voxel-SLAM 启动入口。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from astribot_logging.launch import Node


def _validate(context):
    env = LaunchConfiguration('env').perform(context)
    backend = LaunchConfiguration('slam_backend').perform(context)
    if env not in ('sim', 'hardware') or backend not in ('voxel', 'static_map'):
        raise RuntimeError('env 必须为 sim/hardware，slam_backend 必须为 voxel/static_map')
    if backend == 'static_map' and (env != 'sim' or LaunchConfiguration('mode').perform(context) != 'mapping'):
        raise RuntimeError('static_map 仅供仿真真值基线；载图定位请使用 voxel + previous_map')
    if (backend == 'static_map' and LaunchConfiguration('launch_slam').perform(context) == 'true'
            and not LaunchConfiguration('initial_chassis_pose').perform(context).strip()):
        raise RuntimeError('static_map 的 SLAM 必须提供实测 initial_chassis_pose 配准，不能把 spawn 当成实际启动位姿')
    if LaunchConfiguration('social_scenario').perform(context) and (env != 'sim' or backend != 'static_map'):
        raise RuntimeError('H1 social_scenario requires simulation with ground-truth static_map')
    return []


def generate_launch_description():
    pkg_perception = FindPackageShare('astribot_s1_perception')
    pkg_bringup = FindPackageShare('astribot_s1_gazebo_bringup')

    env = LaunchConfiguration('env')
    backend = LaunchConfiguration('slam_backend')
    sim_voxel = PythonExpression(["'", env, "' == 'sim'"])
    hardware_voxel = PythonExpression([
        "'", env, "' == 'hardware' and '", backend, "' == 'voxel'"])
    use_sim_time = PythonExpression(["'true' if '", env, "' == 'sim' else 'false'"])

    declare_args = [
        DeclareLaunchArgument('navigation_geometry_mode',default_value='legacy'),
        DeclareLaunchArgument('env', default_value='sim'),
        # Kept as a high-level task selector for callers; both mapping and
        # localization now use the same Voxel backend and topic contract.
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('slam_backend', default_value='voxel',
                              description='voxel 或仿真基线 static_map'),
        DeclareLaunchArgument('launch_gazebo', default_value='true'),
        DeclareLaunchArgument('launch_slam', default_value='true',
                              description='是否直接启动 Voxel-SLAM；受管建图入口会关闭此项，由 mapping_runtime 拥有会话'),
        DeclareLaunchArgument('ros_domain_id', default_value='25'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('social_scenario', default_value=''),
        DeclareLaunchArgument('previous_map', default_value=''),
        DeclareLaunchArgument('initial_chassis_pose', default_value=''),
        DeclareLaunchArgument('save_path', default_value='/tmp/astribot_slam_sessions/'),
        DeclareLaunchArgument('map_name', default_value=''),
        DeclareLaunchArgument('save_map', default_value='0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('control_loopback_udp', default_value='false'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_wrist_cameras', default_value='false'),
        DeclareLaunchArgument('use_stereo_cameras', default_value='false'),
        DeclareLaunchArgument('torso_camera_profile', default_value=PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'config', 'camera_torso_rgbd_nav_sim.yaml'])),
        DeclareLaunchArgument('camera_calibration_dir', default_value=PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'config'])),
        DeclareLaunchArgument('camera_mounts_profile', default_value=PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'config', 'camera_mounts_reference_sim.yaml'])),
        DeclareLaunchArgument('use_camera_postprocess', default_value='true'),
        DeclareLaunchArgument('use_camera_pointcloud', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='astribot_s1'),
        DeclareLaunchArgument('camera_profile', default_value=PathJoinSubstitution([FindPackageShare('astribot_s1_description'), 'config', 'camera_rgbd_transport.yaml'])),
        DeclareLaunchArgument('autonomous_patrol', default_value='false'),
        DeclareLaunchArgument('map_source_file', default_value=''),
        DeclareLaunchArgument('map_source', default_value=''),
        DeclareLaunchArgument('localization', default_value=''),
        DeclareLaunchArgument('map_yaml_path', default_value=''),
    ]

    save_use_rviz = SetLaunchConfiguration(
        'use_rviz_actual', LaunchConfiguration('use_rviz'))

    warehouse_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_bringup, 'launch', 'warehouse_sim.launch.py'])),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'navigation_geometry_mode': LaunchConfiguration('navigation_geometry_mode'),
            'ros_domain_id': LaunchConfiguration('ros_domain_id'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'use_rviz': 'false',
            'headless': LaunchConfiguration('headless'),
            'control_loopback_udp': LaunchConfiguration('control_loopback_udp'),
            'camera_profile': LaunchConfiguration('camera_profile'),
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_camera': LaunchConfiguration('use_camera'),
            'use_wrist_cameras': LaunchConfiguration('use_wrist_cameras'),
            'use_stereo_cameras': LaunchConfiguration('use_stereo_cameras'),
            'torso_camera_profile': LaunchConfiguration('torso_camera_profile'),
            'camera_calibration_dir': LaunchConfiguration('camera_calibration_dir'),
            'camera_mounts_profile': LaunchConfiguration('camera_mounts_profile'),
            'use_camera_postprocess': LaunchConfiguration('use_camera_postprocess'),
            'use_camera_pointcloud': LaunchConfiguration('use_camera_pointcloud'),
            'social_scenario': LaunchConfiguration('social_scenario'),
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", env, "' == 'sim' and '", LaunchConfiguration('launch_gazebo'),
            "' == 'true'"])),
    )

    voxel_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_perception, 'launch', 'voxel_slam.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'navigation_geometry_mode':LaunchConfiguration('navigation_geometry_mode'),
            'lidar_topic': '/livox/lidar_left',
            'lidar_topic_back': '/livox/lidar_right',
            'imu_topic': '/livox/imu',
            'point_notime': '1',
            'initial_chassis_pose': LaunchConfiguration('initial_chassis_pose'),
            'publish_grid': PythonExpression(["'", backend, "' == 'voxel'"]),
            'publish_map_odom': PythonExpression(["'", backend, "' == 'voxel'"]),
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", env, "' == 'sim' and '",
            LaunchConfiguration('launch_slam'), "' == 'true'"
        ])),
    )

    sim_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_perception, 'launch', 'sim_perception.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'navigation_geometry_mode':LaunchConfiguration('navigation_geometry_mode'),
            'cloud_pose_frame': PythonExpression([
                "'aft_mapped' if '", backend, "' == 'static_map' else ''"]),
        }.items(),
        condition=IfCondition(sim_voxel),
    )

    hardware_livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_perception, 'launch', 'hardware_livox.launch.py'])),
        launch_arguments={'robot_name': LaunchConfiguration('robot_name')}.items(),
        condition=IfCondition(hardware_voxel),
    )
    hardware_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_perception, 'launch', 'hardware_perception.launch.py'])),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(hardware_voxel),
    )
    voxel_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_perception, 'launch', 'voxel_slam.launch.py'])),
        launch_arguments={
            'use_sim_time': 'false',
            'lidar_topic': '/livox/lidar_left',
            'lidar_topic_back': '/livox/lidar_right',
            'imu_topic': '/livox/imu',
            'point_notime': '0',
            'initial_chassis_pose': LaunchConfiguration('initial_chassis_pose'),
            'imu_extrinsic_tran': '-0.011,-0.02329,0.04412',
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", env, "' == 'hardware' and '", backend, "' == 'voxel' and '",
            LaunchConfiguration('launch_slam'), "' == 'true'"
        ])),
    )

    map_provider = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_perception, 'launch', 'map_provider.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_source_file': LaunchConfiguration('map_source_file'),
            'map_source': 'real_file',
            'localization': 'ground_truth',
            'map_yaml_path': LaunchConfiguration('map_yaml_path'),
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", backend, "' == 'static_map'"])),
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([
            pkg_perception, 'rviz', 'perception_slam_view.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz_actual')),
    )
    patrol = Node(
        package='astribot_s1_perception', executable='autonomous_patrol_node',
        output='screen', parameters=[
            PathJoinSubstitution([pkg_perception, 'config',
                                  'autonomous_patrol_params.yaml']),
            {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('autonomous_patrol')),
    )

    return LaunchDescription(declare_args + [
        OpaqueFunction(function=_validate),
        save_use_rviz,
        warehouse_sim,
        voxel_sim,
        sim_perception,
        hardware_livox,
        hardware_perception,
        voxel_hardware,
        map_provider,
        rviz_node,
        patrol,
    ])
