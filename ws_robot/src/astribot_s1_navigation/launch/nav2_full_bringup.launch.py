

"""Nav2 bringup for Astribot S1."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from astribot_logging.launch import Node
from launch_ros.substitutions import FindPackageShare

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
        DeclareLaunchArgument('launch_gazebo', default_value='true'),
        DeclareLaunchArgument('corridor_file', default_value=''),
        DeclareLaunchArgument('navigation_policy_stage', default_value='off',
                              description='透传已有策略阶段；不自动提高阶段放行状态'),
        DeclareLaunchArgument('launch_navigation', default_value='true',
                              description='是否启动 Nav2；false 可将仿真和导航分开启动'),
        DeclareLaunchArgument(
            'map_file_name', default_value='',
            description='mode:=localization 时要加载的 SLAM Toolbox 序列化地图基础文件名'),
        DeclareLaunchArgument(
            'map_source_file', default_value='',
            description='地图来源配置文件，留空用 astribot_s1_perception/config/map_source.yaml'),
        DeclareLaunchArgument(
            'map_source', default_value='',
            description='覆盖地图来源：sim_slam(仿真自建) | real_file(真机地图落盘) | '
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
        DeclareLaunchArgument('robot_name', default_value='astribot_s1'),
        DeclareLaunchArgument(
            'controller_plugin', default_value='rpp',
            description='rpp(任务书默认要求，非全向退化行为) 或 '
                        'mppi(推荐，能真正利用全向底盘能力，见README)'),
        DeclareLaunchArgument(
            'enable_arm_chassis_coupling', default_value='true',
            description='是否接入臂-底盘动力学耦合动态调速节点(astribot_s1_dynamics_'
                        'coupling)，透传给 navigation.launch.py'),
        DeclareLaunchArgument(
            'enable_posture_monitor',
            default_value=PythonExpression(

                ["'false' if '", LaunchConfiguration('env'), "' == 'hardware' else 'true'"]),
            description='cmd_vel_body_to_world_node 的姿态止损监控，透传给 navigation.launch.py'),
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
            'headless': LaunchConfiguration('headless'),
            'mode': mode,
            'launch_gazebo': LaunchConfiguration('launch_gazebo'),
            'map_file_name': LaunchConfiguration('map_file_name'),
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

    slice_scan = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('astribot_s1_perception_components'), 'launch',
                        'slice_scan.launch.py'])),
                launch_arguments={
                    'use_sim_time': use_sim_time_expr,
                    'params_file': PathJoinSubstitution([
                        FindPackageShare('astribot_s1_perception_components'), 'config',
                        'pointcloud_slice_scan_params.yaml']),
                }.items(),
            ),
        ],
        condition=IfCondition(use_slice_scan),
    )

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
            'corridor_file': LaunchConfiguration('corridor_file'),
            'enable_arm_chassis_coupling': LaunchConfiguration('enable_arm_chassis_coupling'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'enable_posture_monitor': LaunchConfiguration('enable_posture_monitor'),
            'posture_normal_height': LaunchConfiguration('posture_normal_height'),
            'scan_topic': scan_topic_expr,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_navigation, 'rviz', 'nav2_view.rviz'])],
        parameters=[{'use_sim_time': use_sim_time_expr}],
        condition=IfCondition(LaunchConfiguration('nav2_rviz_flag')),
    )

    return LaunchDescription(declare_args + [
        save_use_rviz,
        perception_slam,
        slice_scan,
        navigation,
        exploration,
        rviz_node,
    ])
