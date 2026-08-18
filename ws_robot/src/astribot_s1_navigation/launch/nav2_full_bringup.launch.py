#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：【顶层入口】接入 Nav2 自主导航。在已有的
`astribot_s1_perception/perception_slam_bringup.launch.py`（负责拉起 Gazebo仿真/
实体硬件分支 + 双Livox感知 + SLAM Toolbox 建图或定位）基础上叠加 Nav2 导航栈，
不重复实现已经验证过的那一套。

用法示例：
    # 仿真 + 建图 + 导航（推荐用 mppi 发挥全向底盘能力）
    ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
        env:=sim mode:=mapping launch_gazebo:=true controller_plugin:=mppi

    # 仿真 + 预建图定位 + 导航
    ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
        env:=sim mode:=localization launch_gazebo:=true \
        map_file_name:=/path/to/my_map controller_plugin:=mppi

    # 按任务书要求用 rpp+Smac 组合(默认值，见 config/nav2_params_rpp.yaml 里的取舍说明)
    ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py env:=sim mode:=mapping

发目标点：RViz 里用 "Nav2 Goal" 工具点一个点，或者
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}}}}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
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
        DeclareLaunchArgument(
            'map_file_name', default_value='',
            description='mode:=localization 时要加载的 SLAM Toolbox 序列化地图基础文件名'),
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
    ]

    env = LaunchConfiguration('env')
    mode = LaunchConfiguration('mode')

    # !!! 实测踩坑记录（跟 perception_slam_bringup.launch.py 里记录过的是同一类坑，
    # 这里当时漏加了，导致demo时rviz2根本没启动）!!!：下面 include
    # perception_slam_bringup.launch.py 时传了 launch_arguments={'use_rviz':'false',...}
    # （不想重复开两个RViz），但 ROS2 launch 的 LaunchConfiguration 不是按 include
    # 层级隔离的——IncludeLaunchDescription 的 launch_arguments 本质上是在"当前"这个
    # 共享的 launch 上下文里设置同名变量，会把共享上下文里的 'use_rviz' 覆盖成
    # 'false'，导致本文件自己下面的 rviz_node 读到的 LaunchConfiguration('use_rviz')
    # 也变成了'false'，不管用户传的是什么，rviz2 都不会被启动（而且没有任何报错日志，
    # 非常隐蔽——进程列表里就是干脆没有rviz2）。修复：在 include
    # perception_slam_bringup 之前，先把用户真正传入的 use_rviz 值另存一份到
    # 'nav2_rviz_flag' 这个不会被覆盖的独立变量名，本文件自己的 rviz_node 用这个
    # 副本判断。save_use_rviz 必须排在 include 前面，抢在共享变量被覆盖之前先存一份。
    save_use_rviz = SetLaunchConfiguration('nav2_rviz_flag', LaunchConfiguration('use_rviz'))

    # 复用已验证过的感知+SLAM栈，不重新实现。始终显式传 autonomous_patrol:='false'——
    # 反应式自主巡游节点和 Nav2 都会发 /cmd_vel，两者同时开会打架，这个入口存在的
    # 目的就是跑 Nav2，不暴露"跑这个入口但关掉Nav2"的场景。
    perception_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_perception, 'launch', 'perception_slam_bringup.launch.py'])),
        launch_arguments={
            'env': env,
            'mode': mode,
            'launch_gazebo': LaunchConfiguration('launch_gazebo'),
            'map_file_name': LaunchConfiguration('map_file_name'),
            'robot_name': LaunchConfiguration('robot_name'),
            'autonomous_patrol': 'false',
            'use_rviz': 'false',  # 用本文件自己整合了Nav2显示项的rviz，不重复开两个
        }.items(),
    )

    # use_sim_time 跟随 env，跟 perception_slam_bringup.launch.py 里的写法保持一致。
    use_sim_time_expr = PythonExpression(["'true' if '", env, "' == 'sim' else 'false'"])

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_navigation, 'launch', 'navigation.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time_expr,
            'controller_plugin': LaunchConfiguration('controller_plugin'),
            'enable_arm_chassis_coupling': LaunchConfiguration('enable_arm_chassis_coupling'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_navigation, 'rviz', 'nav2_view.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('nav2_rviz_flag')),
    )

    return LaunchDescription(declare_args + [
        save_use_rviz,   # 必须排在 perception_slam 前面，抢在共享变量被覆盖之前先存一份快照
        perception_slam,
        navigation,
        rviz_node,
    ])
