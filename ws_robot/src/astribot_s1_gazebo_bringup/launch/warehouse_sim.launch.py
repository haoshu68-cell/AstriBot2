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
import math
import xml.etree.ElementTree as ET
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from astribot_logging.launch import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _validate_sim_initial_positions(description_xml, positions):
    robot = ET.fromstring(description_xml)
    controls = {joint.attrib['name']: joint for joint in
                robot.findall('ros2_control/joint')
                if joint.find("command_interface[@name='position']") is not None}
    if not isinstance(positions, dict) or set(positions) != set(controls):
        raise ValueError('Simulation initial positions must cover exactly the position-command joints')
    joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}
    for name, value in positions.items():
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('Non-finite or non-numeric simulation initial position: ' + name)
        limit = joints[name].find('limit')
        if limit is None or not float(limit.attrib['lower']) <= value <= float(limit.attrib['upper']):
            raise ValueError('Simulation initial position outside URDF hard limits: ' + name)
        initial = controls[name].find("state_interface[@name='position']/param[@name='initial_value']")
        if initial is None or float(initial.text) != value:
            raise ValueError('Simulation initial position missing from expanded URDF: ' + name)


def _prepare_sim_description(context, command):
    profile = LaunchConfiguration('sim_initial_joint_profile').perform(context)
    if profile not in ('', 'transport_ready'):
        raise ValueError('Unknown simulation initial joint profile: ' + profile)
    path = ''
    if profile:
        path = PathJoinSubstitution([FindPackageShare('astribot_s1_description'),
                'config', 'sim_initial_transport_ready.yaml']).perform(context)
    SetLaunchConfiguration('sim_initial_positions_file', path).execute(context)
    description = command.perform(context)
    if profile:
        with open(path, encoding='utf-8') as handle:
            positions = yaml.safe_load(handle)['initial_positions']
        _validate_sim_initial_positions(description, positions)
    return [SetLaunchConfiguration('sim_robot_description', description)]


def _camera_bridge_environment(context):
    profile = LaunchConfiguration('camera_bridge_dds_profile').perform(context)
    local = IfCondition(LaunchConfiguration('localhost_only')).evaluate(context)
    if not profile and not local:
        # LAN mode retains the caller's RMW/network profile. The bounded local
        # camera transport must not silently remove cross-machine discovery.
        return {}
    if not profile:
        profile = PathJoinSubstitution([
            FindPackageShare('astribot_s1_gazebo_bringup'), 'config',
            'camera_bridge_fastdds.xml']).perform(context)
    return {'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
            'FASTRTPS_DEFAULT_PROFILES_FILE': profile,
            'ROS_LOCALHOST_ONLY': '0'}


def _local_control_environment(context):
    if not IfCondition(LaunchConfiguration('localhost_only')).evaluate(context):
        return []
    values = {'ROS_LOCALHOST_ONLY': '1', 'IGN_IP': '127.0.0.1', 'GZ_IP': '127.0.0.1'}
    if IfCondition(LaunchConfiguration('control_loopback_udp')).evaluate(context):
        values.update(ROS_LOCALHOST_ONLY='0', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
            FASTRTPS_DEFAULT_PROFILES_FILE=PathJoinSubstitution([
                FindPackageShare('astribot_s1_gazebo_bringup'), 'config',
                'control_loopback_fastdds.xml']).perform(context))
    return [SetEnvironmentVariable(key, value) for key, value in values.items()]


def _camera_bridge_node(context, **kwargs):
    return [Node(package='ros_gz_bridge', executable='parameter_bridge',
                 output='screen', additional_env=_camera_bridge_environment(context),
                 **kwargs)]


def _camera_pipeline_environment(context):
    return [SetEnvironmentVariable(name, value)
            for name, value in _camera_bridge_environment(context).items()]


def _prepare_camera_mounts(context):
    """Resolve one mount source for URDF, native Gazebo frames and health epoch."""
    ids = ('head_rgbd', 'torso_rgbd', 'left_wrist_rgbd', 'right_wrist_rgbd',
           'head_stereo_left', 'head_stereo_right')
    path = LaunchConfiguration('camera_mounts_profile').perform(context)
    if path:
        with open(path, encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
        if (data.get('schema') != 'astribot.camera_mounts/1' or
                data.get('status') not in ('provisional_reference', 'calibrated') or
                type(data.get('revision')) is not int or data['revision'] <= 1):
            raise ValueError('Camera mount profile requires schema, status and a new revision > 1')
        mounts, revision = data['cameras'], data['revision']
    else:
        directory = LaunchConfiguration('camera_calibration_dir').perform(context)
        mounts = {}
        for camera in ids:
            profile = (LaunchConfiguration('camera_profile' if camera == 'head_rgbd' else
                                           'torso_camera_profile').perform(context)
                       if camera in ('head_rgbd', 'torso_rgbd') else
                       os.path.join(directory, 'camera_' + camera + '.yaml'))
            with open(profile, encoding='utf-8') as handle:
                mounts[camera] = yaml.safe_load(handle)
        revision = 1
    parents = {camera: mounts[camera]['parent_frame'] for camera in ids}
    actions = [SetLaunchConfiguration('camera_mount_revision', str(revision))]
    # Fixed camera links are lumped into their moving body link by Gazebo.
    # Resolve the stereo-right calibrated chain instead of inventing a raw frame.
    for camera in ids:
        parent, seen = parents[camera], {camera}
        while parent.endswith('_camera_optical_frame') or parent.endswith('_camera_link'):
            other = parent.rsplit('_camera_', 1)[0]
            if other not in parents or other in seen:
                raise ValueError('Invalid or cyclic camera parent chain: ' + camera)
            seen.add(other)
            parent = parents[other]
        actions.append(SetLaunchConfiguration('camera_native_parent_' + camera, parent))
    return actions


def _prepare_social(context, base_world):
    scenario = LaunchConfiguration('social_scenario').perform(context)
    if not scenario:
        return [SetLaunchConfiguration('active_warehouse_world', base_world),
                SetLaunchConfiguration('social_gui_args', '')]
    from pathlib import Path
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from astribot_s1_social_navigation.scenario import prepare_world, prepare_gui_config, load_scenario
    from astribot_logging import log_directory
    base = base_world.perform(context)
    output = Path(log_directory()) / 'social_world.sdf'
    config = load_scenario(scenario)
    with_hunav = bool(config['agents']) or config.get('enable_empty_observations', False)
    world = prepare_world(base, scenario, output,
                          get_package_prefix('hunav_gazebo_fortress_wrapper') if with_hunav else None,
                          get_package_prefix('astribot_s1_gazebo_bringup'))
    gui_config = prepare_gui_config(scenario, output.with_name('social_gui.config'),
                                   get_package_prefix('astribot_s1_gazebo_bringup'))
    return [SetLaunchConfiguration('active_warehouse_world', world),
            SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH',
                get_package_prefix('astribot_s1_gazebo_bringup') + '/lib' + os.pathsep +
                os.environ.get('IGN_GUI_PLUGIN_PATH', '')),
            SetLaunchConfiguration('social_gui_args', f'--gui-config "{gui_config}" '),
            *([IncludeLaunchDescription(PythonLaunchDescriptionSource(
                get_package_share_directory('astribot_s1_gazebo_bringup') + '/launch/hunav_agents.launch.py'),
                launch_arguments={'scenario': scenario}.items()),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                get_package_share_directory('astribot_s1_social_navigation') + '/launch/observe.launch.py'),
                launch_arguments={'source': 'hunav_truth', 'use_sim_time': 'true',
                                  'hunav_input_topic': '/simulation/hunav_actor_states'}.items())] if with_hunav else []),
            Node(package='tf2_ros', executable='static_transform_publisher', output='screen',
                 arguments=['--frame-id', 'odom', '--child-frame-id', 'social_sim_world'],
                 parameters=[{'use_sim_time': True}]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                 arguments=['/social_sim/state@std_msgs/msg/String[gz.msgs.StringMsg'],
                 parameters=[{'use_sim_time': True}])]


def generate_launch_description():

    declare_args = [
        DeclareLaunchArgument('sim_initial_joint_profile',
            default_value=EnvironmentVariable('ASTRIBOT_SIM_INITIAL_JOINT_PROFILE', default_value=''),
            choices=['', 'transport_ready'],
            description='Optional physical Gazebo initial posture; does not execute zero-to-READY motion'),
        DeclareLaunchArgument('navigation_geometry_mode', default_value='legacy'),
        DeclareLaunchArgument('social_scenario', default_value='', description='Optional HuNav scene in the same warehouse'),
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
        DeclareLaunchArgument('use_camera', default_value='true', description='是否挂载头部和躯干 RGB-D 相机'),
        DeclareLaunchArgument('camera_bridge_dds_profile', default_value='',
            description='空值：本机模式使用有界共享内存和 loopback；LAN 模式继承网络环境。'
                        '显式路径：覆盖桥接 Fast DDS profile（网络范围由该文件负责）'),
        DeclareLaunchArgument('camera_mounts_profile', default_value=PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'config', 'camera_mounts_reference_sim.yaml']),
            description='仿真参考安装位置；空值恢复原始外参映射。临时位置不用于真机标定'),
        DeclareLaunchArgument('use_wrist_cameras', default_value='false',
            description='是否额外挂载左右腕部 RGB-D；导航回归默认关闭以保证双源时效'),
        DeclareLaunchArgument('use_stereo_cameras', default_value='false',
            description='是否额外挂载头部双目；导航回归默认关闭以保证双源时效'),
        DeclareLaunchArgument('camera_profile', default_value=PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'config', 'camera_head_rgbd_nav_sim.yaml']),
            description='Head RGB-D profile; navigation simulation defaults to scaled robot intrinsics'),
        DeclareLaunchArgument('torso_camera_profile', default_value=PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'config', 'camera_torso_rgbd_nav_sim.yaml']),
            description='Torso RGB-D profile; navigation simulation defaults to scaled robot intrinsics'),
        DeclareLaunchArgument('camera_calibration_dir', default_value=PathJoinSubstitution([
            FindPackageShare('astribot_s1_description'), 'config']),
            description='Directory containing the six robot calibration camera profiles'),
        DeclareLaunchArgument('use_camera_postprocess', default_value='true',
            description='用真机 K/D 对仿真图像做 OpenCV 畸变重映射并重发布 CameraInfo'),
        DeclareLaunchArgument('use_camera_pointcloud', default_value='true',
            description='将头部和躯干 RGB-D 深度图投影为供 Nav2/MoveIt 使用的 PointCloud2'),
        DeclareLaunchArgument('enable_rgbd_pose_estimator', default_value='false',
            description='启用 C++ RGB-D 可见表面三维位置观测（朝向未知）'),
        DeclareLaunchArgument('enable_yolo_detector', default_value='false'),
        DeclareLaunchArgument('yolo_camera_id', default_value='head_rgbd', choices=['head_rgbd', 'torso_rgbd']),
        DeclareLaunchArgument('yolo_model_path', default_value=''),
        DeclareLaunchArgument('yolo_labels_path', default_value=''),
        DeclareLaunchArgument('yolo_model_revision', default_value=''),
        DeclareLaunchArgument('yolo_model_layout', default_value='yolov5', choices=['yolov5', 'yolov8']),
        DeclareLaunchArgument('use_native_camera_distortion', default_value='false',
            description='使用 Gazebo 原生畸变；与 camera postprocess 同时开启会重复畸变'),
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
                        '相机桥接使用禁用 builtin transport 的 Fast DDS loopback profile，'
                        '同时限制单播与多播接口，避免默认共享内存容量不足。'
                        'Gazebo 的 ign-transport 是独立于 DDS 的第二条通道，'
                        '必须另设 IGN_IP/GZ_IP，否则它照样多播 239.255.0.7。'
                        '需要跨机联调（如另一台机器跑 RViz）时设 false，'
                        '并配合 ASTRIBOT_NET_MODE=lan source env.sh 走网段白名单。'),
        DeclareLaunchArgument('control_loopback_udp', default_value='false',
                             description='Use explicit loopback UDP for local control nodes; camera transport is separate'),
    ]

    world_name = LaunchConfiguration('world_name')
    robot_name = LaunchConfiguration('robot_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    use_lidar = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')
    use_wrist_cameras = LaunchConfiguration('use_wrist_cameras')
    use_stereo_cameras = LaunchConfiguration('use_stereo_cameras')
    camera_profile = LaunchConfiguration('camera_profile')
    torso_camera_profile = LaunchConfiguration('torso_camera_profile')
    camera_calibration_dir = LaunchConfiguration('camera_calibration_dir')
    use_camera_postprocess = LaunchConfiguration('use_camera_postprocess')
    use_camera_pointcloud = LaunchConfiguration('use_camera_pointcloud')
    enable_rgbd_pose_estimator = LaunchConfiguration('enable_rgbd_pose_estimator')
    use_native_camera_distortion = LaunchConfiguration('use_native_camera_distortion')
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

    set_localhost_env = [OpaqueFunction(function=_local_control_environment)]

    pkg_warehouse = FindPackageShare('aws_robomaker_small_warehouse_world')
    pkg_description = FindPackageShare('astribot_s1_description')
    pkg_bringup = FindPackageShare('astribot_s1_gazebo_bringup')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    default_world_file = PathJoinSubstitution([
        pkg_warehouse, 'worlds', world_name,
        [world_name, TextSubstitution(text='.world')],
    ])
    world_file = LaunchConfiguration('active_warehouse_world')

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
                LaunchConfiguration('social_gui_args'),
                world_file,
            ],
        }.items(),
    )

    robot_description_command = Command([
            'xacro', ' ',
            xacro_file, ' ',
            'robot_name:=', robot_name, ' ',
            'use_lidar:=', use_lidar, ' ',
            'use_camera:=', use_camera, ' ',
            'use_wrist_cameras:=', use_wrist_cameras, ' ',
            'use_stereo_cameras:=', use_stereo_cameras, ' ',
            'camera_profile:=', camera_profile, ' ',
            'torso_camera_profile:=', torso_camera_profile, ' ',
            'camera_calibration_dir:=', camera_calibration_dir, ' ',
            'camera_mounts_profile:="', LaunchConfiguration('camera_mounts_profile'), '" ',
            'use_native_camera_distortion:=', use_native_camera_distortion, ' ',
            'controllers_config:=', controllers_yaml, ' ',
            'sim_initial_positions_file:="', LaunchConfiguration('sim_initial_positions_file'), '" ',
            'wheel_radius:=', wheel_radius, ' ',
            'wheel_effort_limit:=', wheel_effort_limit, ' ',
            'wheel_velocity_limit:=', wheel_velocity_limit, ' ',
            'wheel_joint_damping:=', wheel_joint_damping, ' ',
            'wheel_joint_friction:=', wheel_joint_friction,
        ])
    robot_description_content = ParameterValue(
        LaunchConfiguration('sim_robot_description'), value_type=str)
    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time,
            # Match the 100 Hz measured joints for the fixed-body simulation
            # contract. Keep source stamps; do not mask delay with receipt time.
            'publish_frequency': ParameterValue(PythonExpression([
                "100.0 if '", LaunchConfiguration('navigation_geometry_mode'),
                "' == 'fixed_v2' else 20.0"]), value_type=float)}],
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

    # Every firmware-calibrated camera has an isolated Gazebo namespace.  The
    # head RGB-D names remain the historical topics consumed by the transport
    # node; the other five cameras get explicit names to avoid collisions.
    camera_topics = {
        'head_rgbd': ('rgbd', '/camera/color/image_raw', '/camera/depth/image_raw', '/camera/color/camera_info'),
        'torso_rgbd': ('rgbd', '/camera/torso_rgbd/color/image_raw', '/camera/torso_rgbd/depth/image_raw', '/camera/torso_rgbd/color/camera_info'),
        'left_wrist_rgbd': ('rgbd', '/camera/left_wrist_rgbd/color/image_raw', '/camera/left_wrist_rgbd/depth/image_raw', '/camera/left_wrist_rgbd/color/camera_info'),
        'right_wrist_rgbd': ('rgbd', '/camera/right_wrist_rgbd/color/image_raw', '/camera/right_wrist_rgbd/depth/image_raw', '/camera/right_wrist_rgbd/color/camera_info'),
        'head_stereo_left': ('mono', '/camera/head_stereo_left/image_raw', None, '/camera/head_stereo_left/camera_info'),
        'head_stereo_right': ('mono', '/camera/head_stereo_right/image_raw', None, '/camera/head_stereo_right/camera_info'),
    }
    camera_bridge_args = []
    camera_remappings = []
    for camera_id, (kind, image_topic, depth_topic, info_topic) in camera_topics.items():
        gz_prefix = [TextSubstitution(text='/model/'), robot_name,
                     TextSubstitution(text=f'/{camera_id}')]
        if kind == 'rgbd':
            for suffix, ros_type, ros_topic in (
                ('/image', 'sensor_msgs/msg/Image', image_topic),
                ('/depth_image', 'sensor_msgs/msg/Image', depth_topic),
                ('/camera_info', 'sensor_msgs/msg/CameraInfo', info_topic),
            ):
                camera_bridge_args.append(gz_prefix + [TextSubstitution(text=f'{suffix}@{ros_type}[gz.msgs.{"CameraInfo" if ros_type.endswith("CameraInfo") else "Image"}')])
                camera_remappings.append((gz_prefix + [TextSubstitution(text=suffix)],
                                          f'/camera/raw/{camera_id}{suffix}'))
        else:
            for suffix, ros_type, ros_topic, gz_type in (
                ('/image_raw', 'sensor_msgs/msg/Image', image_topic, 'Image'),
                ('/camera_info', 'sensor_msgs/msg/CameraInfo', info_topic, 'CameraInfo'),
            ):
                camera_bridge_args.append(gz_prefix + [TextSubstitution(text=f'{suffix}@{ros_type}[gz.msgs.{gz_type}')])
                camera_remappings.append((gz_prefix + [TextSubstitution(text=suffix)],
                                          f'/camera/raw/{camera_id}{suffix}'))

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
         TextSubstitution(text='/livox_mid360_imu@sensor_msgs/msg/Imu[gz.msgs.IMU')],
    ]
    bridge_args.extend(camera_bridge_args)

    ros_gz_bridge = OpaqueFunction(function=_camera_bridge_node, kwargs=dict(
        arguments=bridge_args,
        # A raw RGB-D frame exceeds Fast DDS 2.6's default 512 KiB SHM.
        # Local XML provides bounded SHM and loopback discovery; LAN mode
        # retains the caller's environment unless an explicit profile is set.
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
              TextSubstitution(text='/livox_mid360_imu')], '/livox/imu'),
            *camera_remappings,
        ],
    ))

    camera_profiles = {
        'head_rgbd': 'camera_head_rgbd.yaml',
        'torso_rgbd': 'camera_torso_rgbd.yaml',
        'left_wrist_rgbd': 'camera_left_wrist_rgbd.yaml',
        'right_wrist_rgbd': 'camera_right_wrist_rgbd.yaml',
        'head_stereo_left': 'camera_head_stereo_left.yaml',
        'head_stereo_right': 'camera_head_stereo_right.yaml',
    }
    camera_postprocess_nodes = []
    camera_postprocess_enabled = PythonExpression([
        "'", use_camera, "' == 'true' and '", use_camera_postprocess, "' == 'true'"])
    # Navigation owns the two RGB-D streams only.  Wrist/stereo bridges stay
    # available when explicitly enabled for manipulation, but their image
    # remapping must not consume executor threads in the navigation launch.
    navigation_camera_ids = {'head_rgbd', 'torso_rgbd'}
    for camera_id, (kind, image_topic, depth_topic, info_topic) in camera_topics.items():
        if camera_id not in navigation_camera_ids:
            continue
        params = {
            'use_sim_time': use_sim_time,
            'profile': camera_profile if camera_id == 'head_rgbd' else torso_camera_profile,
            'input_image': f'/camera/raw/{camera_id}/image' if kind == 'rgbd' else f'/camera/raw/{camera_id}/image_raw',
            'output_image': image_topic,
            'input_info': f'/camera/raw/{camera_id}/camera_info',
            'output_info': info_topic,
            # The zero-transform Gazebo sensor alias below shares these axes.
            'output_frame': f'{camera_id}_camera_optical_frame',
        }
        if kind == 'rgbd':
            params.update({
                'input_depth': f'/camera/raw/{camera_id}/depth_image',
                'output_depth': depth_topic,
            })
        camera_postprocess_nodes.append(Node(
            # Runtime image/depth remapping is C++ so the two 1280x720 RGB-D
            # streams do not queue behind Python/OpenCV callbacks.  The
            # transport package keeps its Python entry point only as a
            # compatibility fallback for standalone tooling.
            package='astribot_s1_perception_components',
            executable='camera_calibration_postprocess',
            output='screen', parameters=[params],
            condition=IfCondition(camera_postprocess_enabled)))

    # depth_image_proc is intentionally not assumed to be installed on a clean
    # machine.  This small C++ node is part of the workspace and keeps the
    # camera projection dependency reproducible.  Wrist clouds remain private
    # to manipulation; only head/torso clouds feed navigation and MoveIt.
    camera_pointcloud_nodes = []
    camera_filter_nodes = []
    camera_health_nodes = []
    camera_detection_gate_nodes = []
    camera_yolo_nodes = []
    camera_pose_nodes = []
    camera_pointcloud_enabled = PythonExpression([
        "'", use_camera, "' == 'true' and '", use_camera_pointcloud, "' == 'true'"])
    camera_health_enabled = PythonExpression([
        "'", use_camera, "' == 'true'"])
    camera_detection_enabled = PythonExpression([
        "'", use_camera, "' == 'true' and ('", enable_rgbd_pose_estimator,
        "' == 'true' or '", LaunchConfiguration('enable_yolo_detector'), "' == 'true')"])
    camera_pose_enabled = PythonExpression([
        "'", use_camera, "' == 'true' and '", enable_rgbd_pose_estimator, "' == 'true'"])
    for camera_id, depth_topic, info_topic in (
        ('head_rgbd', '/camera/depth/image_raw', '/camera/color/camera_info'),
        ('torso_rgbd', '/camera/torso_rgbd/depth/image_raw',
         '/camera/torso_rgbd/color/camera_info'),
    ):
        camera_pointcloud_nodes.append(Node(
            package='astribot_s1_perception_components', executable='rgbd_pointcloud_node',
            name=f'{camera_id}_pointcloud', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'depth_topic': PythonExpression([
                    "'", depth_topic, "' if '", use_camera_postprocess,
                    "' == 'true' else '/camera/raw/", camera_id, "/depth_image'"]),
                'camera_info_topic': PythonExpression([
                    "'", info_topic, "' if '", use_camera_postprocess,
                    "' == 'true' else '/camera/raw/", camera_id, "/camera_info'"]),
                'output_topic': f'/camera/{camera_id}/points_raw',
                'camera_id': camera_id,
                'processing_health_topic': f'/perception/projection_health/{camera_id}',
                'min_depth': 0.20,
                'max_depth': 5.0,
                # Navigation only needs obstacle occupancy, not a dense VLA
                # image cloud.  Four-pixel decimation keeps the calibrated
                # projection while reducing each 1280x720 cloud from 230k to
                # at most 57.6k points before self-filtering.
                'decimation': 4,
            }],
            condition=IfCondition(camera_pointcloud_enabled)))
        camera_filter_nodes.append(Node(
            package='astribot_s1_perception_components', executable='pointcloud_slice_scan_node',
            name=f'{camera_id}_self_filter', output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('astribot_s1_perception_components'), 'config',
                    'pointcloud_slice_scan_params.yaml']),
                PathJoinSubstitution([
                    FindPackageShare('astribot_s1_perception_components'), 'config',
                    'self_filter.yaml']),
                {
                    'use_sim_time': use_sim_time,
                    'input_cloud_topic': f'/camera/{camera_id}/points_raw',
                    'filtered_cloud_topic': f'/camera/{camera_id}/points',
                    'output_scan_topic': f'/camera/{camera_id}/scan_unused',
                    'base_frame': 'astribot_torso_base',
                    'cloud_pose_frame': 'astribot_torso_base',
                    'enable_voxel_filter': False,
                    'enable_outlier_filter': False,
                    'min_valid_points': 1,
                    'publish_markers': False,
                    'invalid_input_policy': 'stop_output',
                    'input_timeout_sec': 0.5,
                    'max_cloud_age_sec': 0.5,
                    'tf_timeout_sec': 0.10,
                    'tf_total_budget_sec': 0.20,
                },
            ],
            condition=IfCondition(camera_pointcloud_enabled)))

    for camera_id, color_topic, depth_topic, info_topic, frame_id in (
        ('head_rgbd', '/camera/color/image_raw', '/camera/depth/image_raw',
         '/camera/color/camera_info', 'head_rgbd_camera_optical_frame'),
        ('torso_rgbd', '/camera/torso_rgbd/color/image_raw',
         '/camera/torso_rgbd/depth/image_raw',
         '/camera/torso_rgbd/color/camera_info', 'torso_rgbd_camera_optical_frame'),
    ):
        # Raw mode must use the same bridge topics and native frame throughout.
        color_topic = PythonExpression(["'", color_topic, "' if '", use_camera_postprocess,
                                        "' == 'true' else '/camera/raw/", camera_id, "/image'"])
        depth_topic = PythonExpression(["'", depth_topic, "' if '", use_camera_postprocess,
                                        "' == 'true' else '/camera/raw/", camera_id, "/depth_image'"])
        info_topic = PythonExpression(["'", info_topic, "' if '", use_camera_postprocess,
                                       "' == 'true' else '/camera/raw/", camera_id, "/camera_info'"])
        parent = LaunchConfiguration('camera_native_parent_' + camera_id)
        frame_id = PythonExpression(["'", frame_id, "' if '", use_camera_postprocess,
                                     "' == 'true' else '", robot_name, '/', parent, '/', camera_id, "_sensor'"])
        camera_yolo_nodes.append(Node(
            package='astribot_s1_perception_components', executable='yolo_detector_node',
            name=f'{camera_id}_yolo_detector', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time, 'camera_id': camera_id,
                'color_topic': color_topic,
                'health_topic': f'/perception/camera_health/{camera_id}',
                'output_topic': f'/perception/detections/{camera_id}',
                **{key: ParameterValue(LaunchConfiguration('yolo_' + key), value_type=str)
                   for key in ('model_path', 'labels_path', 'model_revision', 'model_layout')},
            }],
            condition=IfCondition(PythonExpression([
                "'", use_camera, "' == 'true' and '", LaunchConfiguration('enable_yolo_detector'),
                "' == 'true' and '", LaunchConfiguration('yolo_camera_id'), "' == '", camera_id, "'"]))))
        camera_health_nodes.append(Node(
            package='astribot_s1_perception_components',
            executable='camera_health_node',
            name=f'{camera_id}_camera_health',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_id': camera_id,
                'source_epoch': 'gazebo_camera',
                'frame_id': frame_id,
                'color_topic': color_topic,
                'depth_topic': depth_topic,
                'info_topic': info_topic,
                'health_topic': f'/perception/camera_health/{camera_id}',
                'expected_rate_hz': 10.0,
                'max_age_sec': 0.25,
                'max_sync_skew_sec': 0.03,
                'calibration_revision': ParameterValue(LaunchConfiguration('camera_mount_revision'), value_type=int),
            }],
            condition=IfCondition(camera_health_enabled)))

        camera_detection_gate_nodes.append(Node(
            package='astribot_s1_perception_components',
            executable='detection_gate_node',
            name=f'{camera_id}_detection_gate',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_id': camera_id,
                'input_topic': f'/perception/detections/{camera_id}',
                'output_topic': f'/perception/valid_detections/{camera_id}',
                'health_topic': f'/perception/camera_health/{camera_id}',
                'require_camera_health': True,
                'min_confidence': 0.25,
                'max_detection_age_sec': 0.30,
                'max_health_age_sec': 0.50,
            }],
            condition=IfCondition(camera_detection_enabled)))

        camera_pose_nodes.append(Node(
            package='astribot_s1_perception_components',
            executable='rgbd_object_pose_node',
            name=f'{camera_id}_rgbd_object_pose',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'detection_topic': f'/perception/valid_detections/{camera_id}',
                'depth_topic': depth_topic,
                'camera_info_topic': info_topic,
                'health_topic': f'/perception/camera_health/{camera_id}',
                'output_topic': f'/perception/object_pose/{camera_id}',
                'require_camera_health': True,
                'max_depth_age_sec': 0.20,
                'max_health_age_sec': 0.50,
                'min_confidence': 0.25,
                'max_depth_m': 5.0,
            }],
            condition=IfCondition(camera_pose_enabled)))

    def make_sensor_frame_alias(real_link, gz_parent_link, gz_sensor_name, enabled):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', real_link,
                '--child-frame-id',
                [robot_name, TextSubstitution(text='/'),
                 TextSubstitution(text=gz_parent_link) if isinstance(gz_parent_link, str) else gz_parent_link,
                 TextSubstitution(text='/'), TextSubstitution(text=gz_sensor_name)],
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(enabled),
        )

    livox_left_frame_alias = make_sensor_frame_alias(
        'livox_mid360_left', 'astribot_torso_base', 'livox_mid360_left_sensor', use_lidar)
    livox_right_frame_alias = make_sensor_frame_alias(
        'livox_mid360_right', 'astribot_torso_base', 'livox_mid360_right_sensor', use_lidar)
    camera_frame_aliases = [make_sensor_frame_alias(
        camera + '_camera_optical_frame', LaunchConfiguration('camera_native_parent_' + camera),
        camera + '_sensor', PythonExpression([
            "'", use_camera, "' == 'true' and '",
            use_wrist_cameras if 'wrist' in camera else use_stereo_cameras if 'stereo' in camera else use_camera,
            "' == 'true'"])) for camera in camera_profiles]

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
        OpaqueFunction(function=_prepare_camera_mounts),
        OpaqueFunction(function=_prepare_sim_description, kwargs={'command': robot_description_command}),
        OpaqueFunction(function=_prepare_social, args=[default_world_file]),
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        delay_controllers_after_spawn,
        delay_effort_drive_after_controllers,
        ros_gz_bridge,
        GroupAction([
            OpaqueFunction(function=_camera_pipeline_environment),
            *camera_postprocess_nodes,
            *camera_pointcloud_nodes,
            *camera_filter_nodes,
            *camera_health_nodes,
            *camera_yolo_nodes,
            *camera_detection_gate_nodes,
            *camera_pose_nodes,
        ]),
        livox_left_frame_alias,
        livox_right_frame_alias,
        *camera_frame_aliases,
        rviz_node,
    ])
