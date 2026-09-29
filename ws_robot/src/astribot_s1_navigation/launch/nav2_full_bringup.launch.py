

"""Nav2 bringup for Astribot S1."""

from datetime import datetime
from uuid import uuid4

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from astribot_logging.launch import Node
from launch_ros.substitutions import FindPackageShare

def _static_map_context(context):
    path = ''
    if (LaunchConfiguration('env').perform(context) == 'sim' and
            LaunchConfiguration('slam_backend').perform(context) == 'static_map' and
            IfCondition(LaunchConfiguration('launch_navigation')).evaluate(context)):
        from astribot_s1_perception.map_source_config import resolve, require
        _, params, _, _ = resolve(
            LaunchConfiguration('map_source_file').perform(context) or None,
            {'map_source': 'real_file', 'localization': 'ground_truth',
             'map_yaml_path': LaunchConfiguration('map_yaml_path').perform(context)})
        path = require(params, 'map_yaml_path', str, default='')
    return [SetLaunchConfiguration('simulation_static_map_yaml', path)]

def generate_launch_description():
    pkg_navigation = FindPackageShare('astribot_s1_navigation')
    pkg_perception = FindPackageShare('astribot_s1_perception')

    declare_args = [
        DeclareLaunchArgument(
            'env', default_value='sim',
            description='运行环境：sim(仿真) 或 hardware(实体机器人)，透传给'
                        'perception_slam_bringup.launch.py'),
        DeclareLaunchArgument(
            'mode', default_value='mapping',
            description='SLAM 模式：mapping(建图+导航同时进行，对应任务书模式1) 或'
                        'localization(预加载地图+定位+导航，对应任务书模式2)'),
        DeclareLaunchArgument(
            'slam_backend', default_value='voxel',
            description='统一 SLAM 后端：voxel 或仿真基线 static_map'),
        DeclareLaunchArgument('launch_gazebo', default_value='true'),
        DeclareLaunchArgument(
            'launch_slam',
            default_value=PythonExpression([
                "'false' if ('", LaunchConfiguration('mode'), "' == 'mapping' and '",
                LaunchConfiguration('slam_backend'), "' == 'voxel') else 'true'"
            ]),
            description='voxel mapping 默认交给 mapping_runtime；static_map 基线仍启动障碍点云处理，但不发布 SLAM 地图/定位 TF'),
        DeclareLaunchArgument('ros_domain_id', default_value='25'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('initial_chassis_pose', default_value='',
                              description='显式 SLAM 初始 map 底盘实测位姿；不从 spawn 推测'),
        DeclareLaunchArgument('social_scenario', default_value=''),
        DeclareLaunchArgument('corridor_file', default_value=''),
        DeclareLaunchArgument('navigation_geometry_mode', default_value='legacy'),
        DeclareLaunchArgument('navigation_policy_stage', default_value='off',
                              description='透传已有策略阶段；不自动提高阶段放行状态'),
        DeclareLaunchArgument('launch_navigation', default_value='true',
                              description='是否启动 Nav2；false 可将仿真和导航分开启动'),
        DeclareLaunchArgument(
            'operator_runtime_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('astribot_operator_backend'), 'config',
                PythonExpression([
                    "'mapping_runtime_sim.yaml' if '", LaunchConfiguration('env'),
                    "' == 'sim' else 'mapping_runtime.yaml'"
                ])
            ]),
            description='operator/mapping_runtime 配置；仿真默认启用受管快速建图，硬件默认保持未配置'),
        DeclareLaunchArgument('previous_map', default_value=''),
        DeclareLaunchArgument('save_path', default_value='/tmp/astribot_slam_sessions/'),
        DeclareLaunchArgument(
            'map_source_file', default_value='',
            description='地图来源配置文件，留空用 astribot_s1_perception/config/map_source.yaml'),
        DeclareLaunchArgument(
            'map_source', default_value='',
            description='静态基线地图来源：real_file(地图落盘) | '
                        'real_live(跨机订阅真机 /map)。留空用配置文件的值'),
        DeclareLaunchArgument(
            'localization', default_value='',
            description='覆盖定位方式：slam(扫描匹配) | ground_truth(静态 TF，仿真真值)。'
                        '留空用配置文件的值'),
        DeclareLaunchArgument(
            'map_yaml_path', default_value='',
            description='map_source:=real_file 时的地图 **yaml** 绝对路径'
                        '（不是 pgm）。留空用配置文件的值'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([pkg_navigation, 'rviz', 'nav2_view.rviz']),
            description='RViz 配置文件；可传入 astribot_operator_station/config/operator.rviz，'
                        '将导航可视化与上位机控制面板合并为单一 RViz 窗口。'),
        DeclareLaunchArgument('robot_name', default_value='astribot_s1'),
        DeclareLaunchArgument('camera_profile', default_value=PathJoinSubstitution([FindPackageShare('astribot_s1_description'), 'config', 'camera_rgbd_transport.yaml'])),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_wrist_cameras', default_value='false'),
        DeclareLaunchArgument('use_stereo_cameras', default_value='false'),
        DeclareLaunchArgument('torso_camera_profile', default_value=PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'config', 'camera_torso_rgbd_nav_sim.yaml'])),
        DeclareLaunchArgument('camera_calibration_dir', default_value=PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'config'])),
        DeclareLaunchArgument('control_loopback_udp', default_value='false'),
        DeclareLaunchArgument('camera_mounts_profile', default_value=PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'config', 'camera_mounts_reference_sim.yaml'])),
        DeclareLaunchArgument('use_camera_postprocess', default_value='true'),
        DeclareLaunchArgument('use_camera_pointcloud', default_value='true'),
        DeclareLaunchArgument(
            'controller_plugin', default_value='rpp',
            description='rpp(任务书默认要求，非全向退化行为) 或 '
                        'mppi(推荐，能真正利用全向底盘能力，见README)'),
        DeclareLaunchArgument(
            'enable_arm_chassis_coupling', default_value='true',
            description='是否将臂展/关节活动限速接入 Nav2 上游约束，透传给 navigation.launch.py'),
        DeclareLaunchArgument(
            'enable_depth_obstacles', default_value='true',
            description='局部代价图是否要求 RGB-D 点云；关闭仅用于显式激光-only 仿真回归'),
        DeclareLaunchArgument(
            'obstacle_layer_plugin', default_value='nav2_costmap_2d::ObstacleLayer',
            description='代价图插件；VoxelLayer 仅用于三维覆盖回归，默认保持基线 ObstacleLayer'),
        DeclareLaunchArgument(
            'enable_posture_monitor',
            default_value=PythonExpression(

                ["'false' if '", LaunchConfiguration('env'), "' == 'hardware' else 'true'"]),
            description='cmd_vel_body_to_world_node 的姿态止损监控，透传给 navigation.launch.py'),
        DeclareLaunchArgument(
            'enable_body_to_world', default_value='false',
            description='是否将 Nav2 车体系速度旋转为 VelocityControl 所需的 world 系分量，透传给 navigation.launch.py'),
        DeclareLaunchArgument(
            'max_linear_speed', default_value='1.0',
            description='线速度上限(m/s)，透传给 navigation.launch.py，一键同时压住 MPPI 的 vx_max/vy_max/vx_min/vy_min 与 velocity_s'),
        DeclareLaunchArgument(
            'posture_normal_height', default_value='0.134',
            description='姿态监控的基准高度(m)，透传给 navigation.launch.py'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true 时 Gazebo 只起 server、不起 GUI（RViz 不受影响）'),
        DeclareLaunchArgument(
            'exploration', default_value='false',
            description='是否拉起探索协调器(严格时序调度 + 未知区禁行)。'
                        'true 时机器人会自主选点并调 Nav2 导航；'
                        '默认 false，避免和手动下发目标抢。'),
        DeclareLaunchArgument('map_name', default_value=
            'explore_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid4().hex[:8]),
        DeclareLaunchArgument('save_map', default_value=PythonExpression([
            "'1' if '", LaunchConfiguration('exploration'), "'.lower() == 'true' and '",
            LaunchConfiguration('mode'), "' == 'mapping' and '",
            LaunchConfiguration('slam_backend'), "' == 'voxel' else '0'"]),
            description='Voxel 自主探索建图默认保存会话；可显式设为 0 禁用。'),
        DeclareLaunchArgument(
            'scan_source', default_value='slice_scan',
            description='Nav2 costmap 的障碍物数据来源，一个开关同时切好两端：'),
    ]

    env = LaunchConfiguration('env')
    mode = LaunchConfiguration('mode')

    save_use_rviz = SetLaunchConfiguration('nav2_rviz_flag', LaunchConfiguration('use_rviz'))

    perception_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_perception, 'launch', 'perception_slam_bringup.launch.py'])),
        launch_arguments={
            'env': env,
            'navigation_geometry_mode':LaunchConfiguration('navigation_geometry_mode'),
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
            'mode': mode,
            'slam_backend': LaunchConfiguration('slam_backend'),
            'launch_slam': LaunchConfiguration('launch_slam'),
            'initial_chassis_pose': LaunchConfiguration('initial_chassis_pose'),
            'launch_gazebo': LaunchConfiguration('launch_gazebo'),
            'ros_domain_id': LaunchConfiguration('ros_domain_id'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'social_scenario': LaunchConfiguration('social_scenario'),
            'previous_map': LaunchConfiguration('previous_map'),
            'save_path': LaunchConfiguration('save_path'),
            'map_name': LaunchConfiguration('map_name'),
            'save_map': LaunchConfiguration('save_map'),
            'robot_name': LaunchConfiguration('robot_name'),

            'map_source_file': LaunchConfiguration('map_source_file'),
            'map_source': LaunchConfiguration('map_source'),
            'localization': LaunchConfiguration('localization'),
            'map_yaml_path': LaunchConfiguration('map_yaml_path'),
            'autonomous_patrol': 'false',
            'use_rviz': 'false',
        }.items(),
    )

    use_sim_time_expr = PythonExpression(["'true' if '", env, "' == 'sim' else 'false'"])

    scan_source = LaunchConfiguration('scan_source')
    scan_topic_expr = PythonExpression([
        "'/scan_from_cloud' if '", scan_source, "' == 'slice_scan' else '/scan'"])
    use_slice_scan = PythonExpression(["'", scan_source, "' == 'slice_scan'"])

    exploration = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('astribot_s1_exploration'), 'launch',
                        'exploration_coordinator.launch.py'])),
                launch_arguments={
                    'use_sim_time': use_sim_time_expr,
                    'require_fixed_envelope': PythonExpression([
                        "'true' if '", LaunchConfiguration('navigation_geometry_mode'),
                        "' == 'fixed_v2' else 'false'"]),
                    'finalize_on_completion': PythonExpression([
                        "'true' if '", LaunchConfiguration('save_map'), "' == '1' else 'false'"]),
                    'params_file': PathJoinSubstitution([
                        FindPackageShare('astribot_s1_exploration'), 'config',
                        'exploration_coordinator_params.yaml']),
                }.items(),
            ),
        ],
        condition=IfCondition(LaunchConfiguration('exploration')),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_navigation, 'launch', 'navigation.launch.py'])),
        condition=IfCondition(LaunchConfiguration('launch_navigation')),
        launch_arguments={
            'use_sim_time': use_sim_time_expr,
            'controller_plugin': LaunchConfiguration('controller_plugin'),
            'navigation_policy_stage': LaunchConfiguration('navigation_policy_stage'),
            'navigation_geometry_mode': LaunchConfiguration('navigation_geometry_mode'),
            'corridor_file': LaunchConfiguration('corridor_file'),
            'enable_arm_chassis_coupling': LaunchConfiguration('enable_arm_chassis_coupling'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'enable_posture_monitor': LaunchConfiguration('enable_posture_monitor'),
            'enable_body_to_world': LaunchConfiguration('enable_body_to_world'),
            'posture_normal_height': LaunchConfiguration('posture_normal_height'),
            'scan_topic': scan_topic_expr,
            'enable_depth_obstacles': LaunchConfiguration('enable_depth_obstacles'),
            'obstacle_layer_plugin': LaunchConfiguration('obstacle_layer_plugin'),
            'operator_runtime_params_file': LaunchConfiguration('operator_runtime_params_file'),
            'simulation_static_map_yaml': LaunchConfiguration('simulation_static_map_yaml'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        remappings=[('/scan', scan_topic_expr)],
        parameters=[{'use_sim_time': use_sim_time_expr}],
        condition=IfCondition(LaunchConfiguration('nav2_rviz_flag')),
    )

    return LaunchDescription(declare_args + [
        save_use_rviz,
        perception_slam,
        OpaqueFunction(function=_static_map_context),
        navigation,
        exploration,
        rviz_node,
    ])
