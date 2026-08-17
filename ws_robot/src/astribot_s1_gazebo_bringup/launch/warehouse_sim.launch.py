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
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---------------------------------------------------------------------
    # 1. 声明所有可调 launch 参数（不写死任何数值/路径）
    # ---------------------------------------------------------------------
    declare_args = [
        DeclareLaunchArgument(
            'world_name', default_value='small_warehouse',
            description='aws_robomaker_small_warehouse_world 里的世界名，'
                        '可选 small_warehouse 或 no_roof_small_warehouse'
                        '（无屋顶版本渲染更快、排查模型是否加载更方便）'),
        DeclareLaunchArgument(
            'robot_name', default_value='astribot_s1',
            description='生成的机器人实体名，多机器人/防止重名冲突时修改此参数'),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='出生点 X (m)'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='出生点 Y (m)'),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.10',
            description='出生点 Z (m)。0.10 是按轮子碰撞体半径(0.08)+轮关节z偏移(-0.015)算出的'
                        '安全值（见 astribot_s1_description/config/collision_overrides.yaml），'
                        '若更换轮子/底盘尺寸需要同步调整，否则机器人会陷进地板或悬空掉落'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='出生朝向 yaw (rad)'),
        DeclareLaunchArgument('use_lidar', default_value='true', description='是否挂载2D激光雷达'),
        DeclareLaunchArgument('use_camera', default_value='true', description='是否挂载头部RGB相机'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='是否使用仿真时钟'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='是否自动打开RViz2'),
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

    # ---------------------------------------------------------------------
    # 2. 依赖包的 share 目录（全部用 FindPackageShare 动态查找，不允许绝对路径）
    # ---------------------------------------------------------------------
    pkg_warehouse = FindPackageShare('aws_robomaker_small_warehouse_world')
    pkg_description = FindPackageShare('astribot_s1_description')
    pkg_bringup = FindPackageShare('astribot_s1_gazebo_bringup')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # PathJoinSubstitution 拼不出 "<world_name>.world" 这种"变量+固定后缀"的写法，
    # 用列表把 world_name 和 TextSubstitution(".world") 拼在同一段里。
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

    # ---------------------------------------------------------------------
    # 3. 修复"货架/箱子模型不显示、纹理丢失"：
    #    aws_robomaker_small_warehouse_world 的 README 只给了经典 Gazebo 的
    #    GAZEBO_MODEL_PATH，没有配新版 Gazebo(Ignition/Gz Sim) 的资源搜索路径，
    #    这里用 SetEnvironmentVariable 动态拼出 GZ_SIM_RESOURCE_PATH（新命名）
    #    和 IGN_GAZEBO_RESOURCE_PATH（旧命名，向后兼容）双写，覆盖 Garden/Fortress 两种环境。
    # ---------------------------------------------------------------------
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    existing_ign_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[warehouse_models_path, os.pathsep, warehouse_worlds_path,
               os.pathsep, existing_gz_path])
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[warehouse_models_path, os.pathsep, warehouse_worlds_path,
               os.pathsep, existing_ign_path])

    # ---------------------------------------------------------------------
    # 4. 启动 Gazebo(Ignition/Gz Sim)，加载仓储世界
    #    "-r" 表示启动后立即运行仿真（不暂停），来自 ros_gz_sim 官方 gz_sim.launch.py 示例写法
    # ---------------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-r '), world_file],
        }.items(),
    )

    # ---------------------------------------------------------------------
    # 5. robot_description（xacro 实时展开，参数用 mappings 透传，杜绝硬编码路径/数值）
    # ---------------------------------------------------------------------
    robot_description_content = Command([
        'xacro', ' ',
        xacro_file, ' ',
        'robot_name:=', robot_name, ' ',
        'use_lidar:=', use_lidar, ' ',
        'use_camera:=', use_camera, ' ',
        'controllers_config:=', controllers_yaml,
    ])
    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ---------------------------------------------------------------------
    # 6. 在 Gazebo 里生成机器人：用 ros_gz_sim 的 create 可执行文件，
    #    订阅 robot_state_publisher 发布的 /robot_description 话题来生成实体
    #    （不使用 ROS1 时代的 gazebo_ros/spawn_model 或 spawn_entity.py）。
    #    出生点：检查惯性参数/碰撞体/初始高度对应任务书"机器人生成后下坠/穿透地面"异常规则，
    #    spawn_z 默认值已按轮子碰撞体半径计算，见上面 DeclareLaunchArgument 里的说明注释。
    # ---------------------------------------------------------------------
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

    # ---------------------------------------------------------------------
    # 7. ros2_control 控制器：joint_state_broadcaster + 4 个 JointTrajectoryController
    #    用 OnProcessExit 事件串行等待，确保 gz_ros2_control 插件里的 controller_manager
    #    已经在 Gazebo 进程内起来、spawn_robot 已完成，再去 spawner，避免服务还没起来就报连接失败。
    # ---------------------------------------------------------------------
    def make_spawner(controller_name):
        return Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[controller_name, '--controller-manager-timeout', '60'],
        )

    joint_state_broadcaster_spawner = make_spawner('joint_state_broadcaster')
    torso_controller_spawner = make_spawner('torso_controller')
    head_controller_spawner = make_spawner('head_controller')
    arm_left_controller_spawner = make_spawner('arm_left_controller')
    arm_right_controller_spawner = make_spawner('arm_right_controller')

    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                torso_controller_spawner,
                head_controller_spawner,
                arm_left_controller_spawner,
                arm_right_controller_spawner,
            ],
        )
    )

    # ---------------------------------------------------------------------
    # 8. ros_gz_bridge：把 gz 话题桥接成标准 ROS2 话题
    #    （/clock 必须桥，否则 use_sim_time 的节点全部收不到仿真时间会卡住）
    #    cmd_vel/odometry 的 gz 话题名由 MecanumDrive/OdometryPublisher 插件按
    #    "/model/<robot_name>/..." 自动生成（对应 astribot_s1.gazebo.xacro 里的相对话题名配置）。
    # ---------------------------------------------------------------------
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        [TextSubstitution(text='/model/'), robot_name,
         TextSubstitution(text='/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist')],
        [TextSubstitution(text='/model/'), robot_name,
         TextSubstitution(text='/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry')],
        [TextSubstitution(text='/model/'), robot_name,
         TextSubstitution(text='/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V')],
        [TextSubstitution(text='/model/'), robot_name,
         TextSubstitution(text='/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan')],
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
              TextSubstitution(text='/cmd_vel')], '/cmd_vel'),
            ([TextSubstitution(text='/model/'), robot_name,
              TextSubstitution(text='/odometry')], '/odom'),
            ([TextSubstitution(text='/model/'), robot_name,
              TextSubstitution(text='/scan')], '/scan'),
            ([TextSubstitution(text='/model/'), robot_name,
              TextSubstitution(text='/camera/image_raw')], '/image_raw'),
        ],
    )

    # ---------------------------------------------------------------------
    # 9. RViz2（可选），用于离线核对模型外观/TF树是否断裂
    # ---------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(declare_args + [
        set_gz_resource_path,
        set_ign_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        delay_controllers_after_spawn,
        ros_gz_bridge,
        rviz_node,
    ])
