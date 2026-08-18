#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：【顶层入口】一条命令切换 仿真/硬件 × 建图/定位 四种组合。

用法示例：
    # 仿真 + 建图，顺带拉起仓储世界仿真
    ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
        env:=sim mode:=mapping launch_gazebo:=true

    # 仿真已经在另一个终端跑着了，这里只挂感知+建图
    ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
        env:=sim mode:=mapping launch_gazebo:=false

    # 仿真 + 定位（用建图阶段 /slam_toolbox/serialize_map 保存好的地图）
    ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
        env:=sim mode:=localization map_file_name:=/path/to/my_map

    # 实体机器人 + 建图
    ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
        env:=hardware mode:=mapping
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_perception = FindPackageShare('astribot_s1_perception')
    pkg_bringup = FindPackageShare('astribot_s1_gazebo_bringup')

    declare_args = [
        DeclareLaunchArgument(
            'env', default_value='sim',
            description='运行环境：sim(仿真) 或 hardware(实体机器人)'),
        DeclareLaunchArgument(
            'mode', default_value='mapping',
            description='SLAM 模式：mapping(建图) 或 localization(纯定位)'),
        DeclareLaunchArgument(
            'launch_gazebo', default_value='true',
            description='env:=sim 时是否顺带拉起 warehouse_sim.launch.py；'
                        '如果仓储仿真已经在别的终端跑着了，设成 false 避免重复生成机器人'),
        DeclareLaunchArgument(
            'map_file_name', default_value='',
            description='mode:=localization 时要加载的地图基础文件名，见 '
                        'slam_localization.launch.py 里的详细说明'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='astribot_s1'),
        DeclareLaunchArgument(
            'autonomous_patrol', default_value='true',
            description='是否启动反应式自主巡游节点(autonomous_patrol_node)，'
                        '让机器人边走边建图，而不是原地不动只靠雷达自转覆盖；'
                        '只想手动操控(遥操作/自己发cmd_vel)时设成 false 关掉它，'
                        '避免和手动指令打架。'),
    ]

    env = LaunchConfiguration('env')
    mode = LaunchConfiguration('mode')
    is_sim = PythonExpression(["'", env, "' == 'sim'"])
    is_hardware = PythonExpression(["'", env, "' == 'hardware'"])
    is_mapping = PythonExpression(["'", mode, "' == 'mapping'"])
    is_localization = PythonExpression(["'", mode, "' == 'localization'"])

    # !!! 实测踩坑记录 !!!：下面 include warehouse_sim.launch.py 时传了
    # launch_arguments={'use_rviz': 'false', ...}（不想重复开两个RViz），
    # 但 ROS2 launch 的 LaunchConfiguration 并不是按 include 层级严格隔离的——
    # IncludeLaunchDescription 的 launch_arguments 本质上是在"当前"这个共享的
    # launch 上下文里设置同名变量，等 warehouse_sim.launch.py 执行完
    # DeclareLaunchArgument('use_rviz', ...) 时，会把共享上下文里的 'use_rviz'
    # 覆盖成 'false'——导致本文件自己最下面的 rviz_node 读到的
    # LaunchConfiguration('use_rviz') 也变成了 'false'，不管用户在命令行传了
    # use_rviz:=true 还是默认值 true，RViz 都不会被打开（而且日志里连尝试启动的
    # 记录都没有，非常隐蔽）。修复：在 include warehouse_sim 之前，先把用户真正
    # 传入的 use_rviz 值另存一份到 'use_rviz_actual' 这个不会被覆盖的独立变量名，
    # 本文件自己的 rviz_node 用这个副本判断，不用原名。
    save_use_rviz = SetLaunchConfiguration('use_rviz_actual', LaunchConfiguration('use_rviz'))

    # ---- 仿真分支 ----
    warehouse_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'warehouse_sim.launch.py'])),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_rviz': 'false',  # 用本文件统一的感知+SLAM RViz配置，不重复开两个RViz
        }.items(),
        condition=IfCondition(PythonExpression(
            ["'", env, "' == 'sim' and '", LaunchConfiguration('launch_gazebo'), "' == 'true'"])),
    )
    sim_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_perception, 'launch', 'sim_perception.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(is_sim),
    )

    # ---- 硬件分支 ----
    hardware_livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_perception, 'launch', 'hardware_livox.launch.py'])),
        launch_arguments={'robot_name': LaunchConfiguration('robot_name')}.items(),
        condition=IfCondition(is_hardware),
    )
    hardware_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_perception, 'launch', 'hardware_perception.launch.py'])),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(is_hardware),
    )

    # ---- SLAM（建图/定位二选一，use_sim_time 跟随 env） ----
    use_sim_time_str = PythonExpression(["'true' if '", env, "' == 'sim' else 'false'"])
    slam_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_perception, 'launch', 'slam_mapping.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time_str}.items(),
        condition=IfCondition(is_mapping),
    )
    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_perception, 'launch', 'slam_localization.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time_str,
            'map_file_name': LaunchConfiguration('map_file_name'),
        }.items(),
        condition=IfCondition(is_localization),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution(
            [pkg_perception, 'rviz', 'perception_slam_view.rviz'])],
        parameters=[{'use_sim_time': use_sim_time_str}],
        condition=IfCondition(LaunchConfiguration('use_rviz_actual')),
    )

    autonomous_patrol_node = Node(
        package='astribot_s1_perception',
        executable='autonomous_patrol_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_perception, 'config', 'autonomous_patrol_params.yaml']),
            {'use_sim_time': use_sim_time_str},
        ],
        condition=IfCondition(LaunchConfiguration('autonomous_patrol')),
    )

    return LaunchDescription(declare_args + [
        save_use_rviz,   # 必须排在 warehouse_sim 前面，抢在共享变量被覆盖之前先存一份快照
        warehouse_sim,
        sim_perception,
        hardware_livox,
        hardware_perception,
        slam_mapping,
        slam_localization,
        rviz_node,
        autonomous_patrol_node,
    ])
