#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：【硬件分支】把 SLAM 已融合的世界系点云接入自滤 + 2D 投影，产出 /scan。

═══════════════════════════════════════════════════════════════════════════
为什么实机不再走 preprocess + fusion（2026-09-03 实测改造）
═══════════════════════════════════════════════════════════════════════════
实机 SLAM（Voxel-SLAM）本来就发布一路**已融合、已裁剪**的世界系点云
`/map_scan_filtered`，实测（不是推算）：

    type      sensor_msgs/PointCloud2   point_step=48 (pcl::PointXYZINormal)
    frame_id  camera_init               height=1  width≈2901  is_dense=True
    拍率      10.01 Hz
    z 范围    [-0.0454, 1.5258]         对应 mid360.yaml 的 nav_scan_z_min=-0.046

它已经是**两台雷达融合后**的结果，所以：
  · 不能再喂给 preprocess ×2 + fusion —— 那会把同一份数据算两遍，
    而且 livox_preprocess_node 的 ground_z_min 是作用在**雷达自身局部系**
    的阈值（节点不查 TF），拿它去裁一份**世界系**点云，语义完全错位。
  · 正确接入点在更下游：直接作为 pointcloud_slice_scan_node 的
    input_cloud_topic，跳过 preprocess 与 fusion 两级。

链路对比：
    仿真   gz 两路 PointCloud2 -> preprocess x2 -> fusion -> /livox/fused_points
                                                              |
    实机   SLAM /map_scan_filtered ------------------------------+
                                                              v
           pointcloud_slice_scan_node（astribot_s1_autonomy，本文件不启动它）
                     -> /livox/cloud_self_filtered
                     -> pointcloud_to_laserscan -> /scan

═══════════════════════════════════════════════════════════════════════════
!!! 前置依赖：缺任何一条，/scan 会**静默**一帧不出 !!!
═══════════════════════════════════════════════════════════════════════════
1. SLAM 在跑，且发 `camera_init -> aft_mapped`（实测 10.0 Hz，时间戳回跳 0 次）。
2. **`aft_mapped -> astribot_torso_base` 这条边必须有人发。**
   实测 2026-09-03：实机 TF 树上**只有** camera_init->aft_mapped 一条边，
   `astribot_torso_base` 这个 frame 完全不存在（RSP 也没在跑），于是
   pointcloud_slice_scan_node 的 lookupTransform(base_frame, cloud_frame)
   必然抛 LookupException、每帧都被丢掉。
   这条边是**零偏移**的，依据是 SLAM 侧的底盘坐标系改造（三条证据）：
     · voxelslam.cpp 广播前先调 imu_pose_to_chassis()，发的是 chassis 位姿
       而不是 IMU 位姿（xc.p 未直接使用）；
     · 该改造已编译进去（imu_pose_to_chassis 在 cpp/hpp 各出现 6/2 次）；
     · chassis_frame_changes.txt 明确「把创世点设成底盘位姿为 (I,0) 时对应的
       IMU 位姿，使底盘轨迹严格从 (I,0) 开始」，与实测静止时
       xyz≈(0.002,0.001,-0.003) 吻合。
   ⚠️ nav_prob_grid.yaml 里「每个关键帧自己的原始 IMU 位姿，不是底盘中心」
      那句注释描述的是**关键帧接口**，不适用于这条 TF 广播路径，已过时。
3. RSP（robot_state_publisher）在跑，提供 astribot_torso_base 以下的整棵
   本体树 —— 自滤的连杆链（torso/head/arm/gripper）全靠它。
4. pointcloud_slice_scan_node 在跑，且它的 input_cloud_topic 被设成
   本文件的 slam_cloud_topic（默认 /map_scan_filtered）。

═══════════════════════════════════════════════════════════════════════════
livox_custom_to_pc2_node 去哪了
═══════════════════════════════════════════════════════════════════════════
**代码与 27 个测试全部保留，只是不在这条链上了。** 它把厂商的
livox_ros_driver2/msg/CustomMsg 转成 PointCloud2，输出
/livox/lidar_{front,back}_pc2。改走 /map_scan_filtered 之后探索链不再依赖它。

保留的理由（它不是"多余"，是"暂时不在关键路径"）：
    项目          /map_scan_filtered      CustomMsg 转换后
    点数          约 2901                 每路约 2 万
    z 范围        已被 SLAM 裁成 ±        完整
    双雷达        不可分（已融合）        可分
    自滤输入      SLAM 已滤过一轮         原始，自滤完全可控
需要原始双雷达点云、或要让本仓库的自滤/ground_z_min 拆分在实机生效时，
把 slam_cloud_topic 换回 /livox/lidar_front_pc2 那条链即可（见 git 历史）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from astribot_logging.launch import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_perception = FindPackageShare('astribot_s1_perception')
    p2l_params = PathJoinSubstitution(
        [pkg_perception, 'config', 'pointcloud_to_laserscan_params.yaml'])

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='硬件分支固定关闭仿真时钟'),
        DeclareLaunchArgument(
            'slam_cloud_topic', default_value='/map_scan_filtered',
            description='SLAM 输出的已融合世界系点云（camera_init 系，10Hz）'),
        DeclareLaunchArgument(
            'self_filtered_topic', default_value='/livox/cloud_self_filtered',
            description='pointcloud_slice_scan_node 的自滤输出，p2l 订阅它'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='最终 LaserScan，nav2 两个 costmap 的唯一数据源'),
        DeclareLaunchArgument(
            'publish_chassis_alias', default_value='true',
            description='发 aft_mapped -> astribot_torso_base 零偏移静态边'),
        DeclareLaunchArgument(
            'launch_slice_node', default_value='true',
            description='是否一并启动 pointcloud_slice_scan_node'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    slice_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('astribot_s1_perception_components'),
                 'launch', 'slice_scan.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'input_cloud_topic': LaunchConfiguration('slam_cloud_topic'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_slice_node')),
    )

    chassis_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aft_mapped_to_torso_base',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'aft_mapped',
                   '--child-frame-id', 'astribot_torso_base'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('publish_chassis_alias')),
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[p2l_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cloud_in', LaunchConfiguration('self_filtered_topic')),
            ('scan', LaunchConfiguration('scan_topic')),
        ],
    )

    return LaunchDescription(
        declare_args + [chassis_alias, slice_node, pointcloud_to_laserscan])
