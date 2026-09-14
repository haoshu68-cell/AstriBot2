#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：主 launch 入口 —— 启动 Gazebo(Ignition/Gz Sim) 并加载
aws_robomaker_small_warehouse_world 的仓储场景，在其中生成 Astribot S1 轮式双臂机器人，
并拉起 robot_state_publisher + ros2_control 控制器 + ros_gz_bridge 话题桥接。

全部使用 ROS2 Humble 原生 Python Launch API（launch / launch_ros），未使用任何 XML launch。
资源路径统一使用 FindPackageShare + PathJoinSubstitution，不出现任何硬编码绝对路径。

用法示例：
    ros2 launch astribot_s1_gazebo_bringup warehouse_sim.launch.py
    ros2 launch astribot_s1_gazebo_bringup warehouse_sim.launch.py \
        world_name:=no_roof_small_warehouse robot_name:=astribot_s1_2 spawn_x:=1.0
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declare_args = [
        DeclareLaunchArgument(
            'world_name', default_value='small_warehouse',
            description='aws_robomaker_small_warehouse_world 里的世界名，'
                        '可选 small_warehouse 或 no_roof_small_warehouse'
                        '（无屋顶版本渲染更快、排查模型是否加载更方便）'),
        DeclareLaunchArgument(
            'robot_name', default_value='astribot_s1',
            description='生成的机器人实体名，多机器人/防止重名冲突时修改此参数'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true 时只起 Gazebo server(-s)，不起 GUI。\n'
                        '!!! 什么时候必须用它(实测踩过) !!!：本机 EGL 初始化失败时'
                        '（日志里 "libEGL warning: egl: failed to create dri2 screen"），'
                        'Gazebo GUI 会退化成软件渲染并把 CPU 吃满(实测 188%)，'
                        '把 server 挤到 2% 上不去、物理根本步不动 —— 表现是 /clock 不推进、'
                        '/joint_states 与所有传感器话题都有发布者但零消息，'
                        '上层 nav2/SLAM/探索全部卡在等一个永远不来的仿真时间。'
                        '这种情况下用 headless:=true + RViz 可视化即可正常跑，'
                        '不需要 GUI（RViz 用的是独立的 OpenGL 上下文，不受影响）。'),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='出生点 X (m)'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='出生点 Y (m)'),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.15',
            description='出生点 Z (m)。!!! 力控重构方案实测更新 !!!：改成0.15，'
                        '因为修掉"躯干碰撞圆柱体比轮子只高1.7mm、整机其实坐在肚皮上"'
                        '这个根因后(见astribot_s1_torso_wheel.xacro的记录)，'
                        '现在真正靠四个轮子接地，实测静止高度 z≈0.1292，'
                        '出生点比静止高度略高一点(留约2cm)自然落到轮子上，避免初始穿透。'
                        '!!! 跟 wheel_radius 联动扫描测试时 !!!：静止高度 ≈ wheel_radius+0.049，'
                        '出生点取 wheel_radius+0.07 左右；同时注意轮径变大后'
                        '"同样摩擦力产生的反抗力矩 τ=F·r"也按比例变大，'
                        'wheel_effort_limit 要同步放大，否则轮子会被憋死转不动'
                        '(实测 r=0.30 时需要 >20N·m)'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='出生朝向 yaw (rad)'),
        DeclareLaunchArgument('use_lidar', default_value='true', description='是否挂载双Livox Mid-360激光雷达'),
        DeclareLaunchArgument('use_camera', default_value='true', description='是否挂载头部RGB相机'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='是否使用仿真时钟'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='是否自动打开RViz2'),
        DeclareLaunchArgument(
            'wheel_radius', default_value='0.08',
            description='轮子碰撞球半径(m)，同时驱动逆解运动学(与enable_effort_drive节点的'
                        'wheel_radius参数保持一致，见下方)，扫描测试改这个'),
        DeclareLaunchArgument(
            'wheel_effort_limit', default_value='15.0',
            description='轮关节effort力矩限幅(N·m)，起始估算值，需要单轮测试标定'),
        DeclareLaunchArgument(
            'wheel_velocity_limit', default_value='40.0',
            description='轮关节速度限幅(rad/s)'),
        DeclareLaunchArgument(
            'wheel_joint_damping', default_value='1.0',
            description='轮关节粘性阻尼(N·m·s/rad)。力矩闭环稳定的必要条件——'
                        '没有阻尼时轮子是无阻尼自由旋转体，一旦地面反作用力不足就会'
                        '几毫秒内飙到速度限位、PID在扭矩限幅间bang-bang振荡'),
        DeclareLaunchArgument(
            'wheel_joint_friction', default_value='0.1',
            description='轮关节静摩擦(N·m)，模拟减速器/轴承的干摩擦'),
        DeclareLaunchArgument(
            'enable_effort_drive', default_value='true',
            description='是否拉起 astribot_s1_chassis_effort_drive 的力矩闭环驱动节点。'
                        'VelocityControl/MecanumDrive已整体移除，关掉这个底盘就完全没有'
                        '驱动力——仅用于调试阶段单独核对ros2_control/SDF是否加载正确的场景'),
        DeclareLaunchArgument(
            'ros_domain_id', default_value='25',
            description='本次仿真独占的 ROS_DOMAIN_ID。'
                        '取 25 是为了与厂商 env.sh:115 写死的值一致 —— 桥接要与 SDK '
                        '同域才能互相看见，而真机上 SDK 后端是既有进程、改不动。'
                        '同一台机器上如果还跑着别的 ROS2 图（哪怕是完全无关的项目），'
                        '只要都用默认 domain 0，/robot_description、'
                        '/controller_manager/... 这类未加命名空间的话题/服务就会被 DDS '
                        '发现机制"撞名"——本方案实测就遇到过：另一个无关工作空间残留的 '
                        'robot_state_publisher 通过 transient_local QoS 抢答了我们的 '
                        '/robot_description 订阅，导致生成的是别的模型、控制器加载互相冲突。 '
                        '固定一个独占 domain 可以彻底避免这类"看起来随机"的故障，'
                        '和别的机器人/别的仿真同时跑时改这个参数即可。'),
        DeclareLaunchArgument(
            'localhost_only', default_value='true',
            description='把本次仿真的所有话题限制在本机，局域网内其它机器发现不到也抓不到。'
                        '默认 true——仿真栈会拉起 25+ 个节点，其中 /cmd_vel 之类是可写的，'
                        '不该暴露在办公网。实测（/proc/net/igmp 逐网卡数多播加入次数）：'
                        'ROS_LOCALHOST_ONLY=1 能真正阻止 DDS 在物理网卡上加入 239.255.0.1；'
                        '而 Fast DDS 的 interfaceWhiteList XML 无效（只过滤单播 locator）。'
                        'Gazebo 的 ign-transport 是独立于 DDS 的第二条通道，'
                        '必须另设 IGN_IP/GZ_IP，否则它照样多播 239.255.0.7。'
                        '需要跨机联调（如另一台机器跑 RViz）时设 false，'
                        '并配合 ASTRIBOT_NET_MODE=lan source env.sh 走网段白名单。'),
    ]

    world_name = LaunchConfiguration('world_name')
    robot_name = LaunchConfiguration('robot_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    use_lidar = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_effort_limit = LaunchConfiguration('wheel_effort_limit')
    wheel_velocity_limit = LaunchConfiguration('wheel_velocity_limit')
    wheel_joint_damping = LaunchConfiguration('wheel_joint_damping')
    wheel_joint_friction = LaunchConfiguration('wheel_joint_friction')
    enable_effort_drive = LaunchConfiguration('enable_effort_drive')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    localhost_only = LaunchConfiguration('localhost_only')

    set_ros_domain_id = SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=ros_domain_id)

    set_localhost_env = [
        SetEnvironmentVariable(
            name='ROS_LOCALHOST_ONLY', value='1',
            condition=IfCondition(localhost_only)),
        SetEnvironmentVariable(
            name='IGN_IP', value='127.0.0.1',
            condition=IfCondition(localhost_only)),
        SetEnvironmentVariable(
            name='GZ_IP', value='127.0.0.1',
            condition=IfCondition(localhost_only)),
    ]

    pkg_warehouse = FindPackageShare('aws_robomaker_small_warehouse_world')
    pkg_description = FindPackageShare('astribot_s1_description')
    pkg_bringup = FindPackageShare('astribot_s1_gazebo_bringup')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    world_file = PathJoinSubstitution([
        pkg_warehouse, 'worlds', world_name,
        [world_name, TextSubstitution(text='.world')],
    ])

    warehouse_models_path = PathJoinSubstitution([pkg_warehouse, 'models'])
    warehouse_worlds_path = PathJoinSubstitution([pkg_warehouse, 'worlds'])

    xacro_file = PathJoinSubstitution([pkg_description, 'urdf', 'astribot_s1.xacro'])
    controllers_yaml = PathJoinSubstitution(
        [pkg_bringup, 'config', 'astribot_s1_controllers.yaml'])
    rviz_config = PathJoinSubstitution(
        [pkg_description, 'rviz', 'astribot_s1_view.rviz'])

    bringup_models_path = PathJoinSubstitution([pkg_bringup, 'models'])
    description_share_parent_path = PathJoinSubstitution([pkg_description, os.pardir])

    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    existing_ign_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[bringup_models_path, os.pathsep,
               warehouse_models_path, os.pathsep, warehouse_worlds_path, os.pathsep,
               description_share_parent_path, os.pathsep, existing_gz_path])
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[bringup_models_path, os.pathsep,
               warehouse_models_path, os.pathsep, warehouse_worlds_path, os.pathsep,
               description_share_parent_path, os.pathsep, existing_ign_path])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': [
                TextSubstitution(text='-r '),
                PythonExpression([
                    "'-s ' if '", LaunchConfiguration('headless'), "' == 'true' else ''"]),
                world_file,
            ],
        }.items(),
    )

    robot_description_content = ParameterValue(
        Command([
            'xacro', ' ',
            xacro_file, ' ',
            'robot_name:=', robot_name, ' ',
            'use_lidar:=', use_lidar, ' ',
            'use_camera:=', use_camera, ' ',
            'controllers_config:=', controllers_yaml, ' ',
            'wheel_radius:=', wheel_radius, ' ',
            'wheel_effort_limit:=', wheel_effort_limit, ' ',
            'wheel_velocity_limit:=', wheel_velocity_limit, ' ',
            'wheel_joint_damping:=', wheel_joint_damping, ' ',
            'wheel_joint_friction:=', wheel_joint_friction,
        ]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
            '-Y', spawn_yaw,
            '-allow_renaming', 'true',  # 若 robot_name 与场景内已有实体重名，自动改名而不是生成失败
        ],
    )

    controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            'torso_controller',
            'head_controller',
            'arm_left_controller',
            'arm_right_controller',
            'wheel_effort_controller',
            'gripper_left_controller',
            'gripper_right_controller',
            '--controller-manager-timeout', '60',
        ],
    )

    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[controllers_spawner],
        )
    )

    pkg_effort_drive = FindPackageShare('astribot_s1_chassis_effort_drive')
    effort_drive_node = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [pkg_effort_drive, 'launch', 'omni_effort_drive.launch.py'])),
                launch_arguments={
                    'wheel_radius': wheel_radius,
                    'use_sim_time': use_sim_time,
                    'params_file': PathJoinSubstitution(
                        [pkg_effort_drive, 'config', 'omni_effort_drive_params.yaml']),
                }.items(),
            ),
        ],
        condition=IfCondition(enable_effort_drive),
    )
    delay_effort_drive_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controllers_spawner,
            on_exit=[effort_drive_node],
        )
    )

    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        [TextSubstitution(text='/model/'), robot_name,
         TextSubstitution(text='/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry')],
        [TextSubstitution(text='/model/'), robot_name,
         TextSubstitution(text='/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V')],
        [TextSubstitution(text='/model/'), robot_name,
         TextSubstitution(text='/livox_mid360_left/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked')],
        [TextSubstitution(text='/model/'), robot_name,
         TextSubstitution(text='/livox_mid360_right/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked')],
        [TextSubstitution(text='/model/'), robot_name,
         TextSubstitution(text='/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image')],
    ]

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=bridge_args,
        remappings=[
            ([TextSubstitution(text='/model/'), robot_name,
              TextSubstitution(text='/odometry')], '/odom'),
            ([TextSubstitution(text='/model/'), robot_name,
              TextSubstitution(text='/pose')], '/tf'),
            ([TextSubstitution(text='/model/'), robot_name,
              TextSubstitution(text='/livox_mid360_left/points')], '/livox/lidar_left'),
            ([TextSubstitution(text='/model/'), robot_name,
              TextSubstitution(text='/livox_mid360_right/points')], '/livox/lidar_right'),
            ([TextSubstitution(text='/model/'), robot_name,
              TextSubstitution(text='/camera/image_raw')], '/image_raw'),
        ],
    )

    def make_sensor_frame_alias(real_link, gz_parent_link, gz_sensor_name):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', real_link,
                '--child-frame-id',
                [robot_name, TextSubstitution(text='/'), TextSubstitution(text=gz_parent_link),
                 TextSubstitution(text='/'), TextSubstitution(text=gz_sensor_name)],
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )

    livox_left_frame_alias = make_sensor_frame_alias(
        'livox_mid360_left', 'astribot_torso_base', 'livox_mid360_left_sensor')
    livox_right_frame_alias = make_sensor_frame_alias(
        'livox_mid360_right', 'astribot_torso_base', 'livox_mid360_right_sensor')
    camera_frame_alias = make_sensor_frame_alias(
        'camera_link', 'astribot_head_link_2', 'astribot_camera')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(declare_args + set_localhost_env + [
        set_ros_domain_id,
        set_gz_resource_path,
        set_ign_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        delay_controllers_after_spawn,
        delay_effort_drive_after_controllers,
        ros_gz_bridge,
        livox_left_frame_alias,
        livox_right_frame_alias,
        camera_frame_alias,
        rviz_node,
    ])
