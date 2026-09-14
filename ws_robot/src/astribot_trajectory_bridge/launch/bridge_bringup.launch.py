#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""桥接容器启动文件（Gate 3 / Gate 5）。

!!! 环境变量的设置顺序是硬要求，不是风格问题 !!!
============================================
``SetEnvironmentVariable`` 作用于**它之后**的所有 action，所以三条环境变量必须
排在 Node 之前。原因见 astribot_sdk/core/astribot_api/astribot_interface.py:34-41：

* ``ASTRIBOT_LOG`` —— SDK 一被 import 就用 ``os.dup2`` 把进程的 fd 1/2 重定向到
  /dev/null（连 C 扩展输出一起吞）。**import 之后再设置完全无效**，且会连带把
  桥接自己的 ERROR 日志吞掉 —— 包括本该"响亮失败"的那些。
  合法值只有 ``1`` / ``true`` / ``on``（大小写不敏感）：``yes`` / ``2`` 都会被
  判为静音。
* ``ROBOT_TYPE`` —— astribot_base.py:34-38 用它决定 ``chassis_dof``：
  不设成 S1 时底盘是 **2** 自由度而不是 3；astribot_client.py:40 还会对
  非 S0/S1 直接 ``raise ValueError``。
* ``ROS_DOMAIN_ID`` —— 全栈统一 25，与厂商 env.sh:115 一致。真机上 SDK 后端是
  既有进程、domain 改不动，所以是本栈迁过去而不是反过来。

关于 domain：当前是 D-1（统一 domain）
====================================
开发阶段选 D-1，把本栈与 SDK 放到同一个 domain。**代价是原来靠 domain 隔离提供的
那条保护消失了** —— 即"仿真 /cmd_vel 绝不能有到真机的路径"。

D-1 下唯一的替代保护是 WriteGate（见 write_gate.py），但它是**流程性准入检查，
不是网络层隔离**，强度低于 D-2。而且当前 ``_discover_backends()`` 还不能真正发现
多后端（examples 里没有可用于区分 sim/real 的标识，列为 Gate 0-g）。

所以 D-1 期间必须靠操作纪律：**绝不同时拉起 MuJoCo 与真机后端**。
建议 Gate 4 通过后、进入真机联调前切回 D-2（双 Context 跨 domain 桥接，
本工程的 map_domain_relay 已有可用先例）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('astribot_trajectory_bridge')

    declared = [
        DeclareLaunchArgument(
            'target', default_value='sim',
            description='sim | real。**只被本 launch 文件读到，业务代码一行都不读它** —— '
                        'SDK 面向 ROS 图工作，连的是"当前图上有谁"，'
                        '所以不需要在代码里写 sim/real 分支。\n'
                        'target:=sim 时顺带把 MuJoCo 后端进程拉起来；real 时不拉。'),
        DeclareLaunchArgument(
            'domain_id', default_value='25',
            description='ROS_DOMAIN_ID。当前是 D-1 统一 domain：本栈与 SDK 都用 25 '
                        '（厂商 env.sh:115 写死的就是 25，真机上后端是既有进程、改不动，'
                        '所以是本栈迁过去）。现场实测到别的值时才需要覆盖，见 '
                        'docs/real_robot_deployment.md §1.3。'),
        DeclareLaunchArgument(
            'robot_type', default_value='S1',
            description='决定 chassis_dof（S1=3，否则 2）。非 S0/S1 会让 SDK 直接抛 ValueError。'),
        DeclareLaunchArgument(
            'allow_write_to_real', default_value='false',
            description='!!! 打真机的二次授权 !!! 刻意不写进任何 yaml —— '
                        '必须命令行显式给，避免被存进配置后忘记。'),
        DeclareLaunchArgument(
            'enable_slam_correction', default_value='true',
            description='底盘位姿反馈闭环。false=纯开环+leash，一键回归。'),
        DeclareLaunchArgument(
            'pose_source', default_value='slam',
            description='slam | ground_truth。必须显式声明，无默认推断。\n'
                        '!!! ground_truth 下闭环退化 !!! map->odom 是恒等静态 TF，'
                        '外环误差恒≈0、校正量恒≈0 —— 代码路径在跑但闭环没有作用，'
                        '此时会上报 CORRECTION_DEGENERATE。'
                        '它只能验证代码路径，不能作为闭环有效性证据。'),
        DeclareLaunchArgument(
            'enable_waypoints_service', default_value='false',
            description='方案 A（move_joints_waypoints，阻塞、不可取消）。'
                        '默认关闭，调用时显式返回 DISABLED_BY_CONFIG。'),
        DeclareLaunchArgument(
            'chassis_params_file',
            default_value=PathJoinSubstitution([pkg, 'config', 'chassis_bridge.yaml'])),
        DeclareLaunchArgument(
            'arm_params_file',
            default_value=PathJoinSubstitution([pkg, 'config', 'arm_bridge.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'sim_root', default_value=EnvironmentVariable('ASTRIBOT_SIM_ROOT',
                                                         default_value=''),
            description='MuJoCo 仿真仓库（Astribot-Dev/astribot_simulation）根目录。\n'
                        '!!! 它是**独立仓库**，不在本 SDK 仓里 !!! 所以必须走变量，'
                        '默认读环境变量 ASTRIBOT_SIM_ROOT。\n'
                        'target:=sim 且此项为空时，MuJoCo 不会被拉起 —— '
                        '此时 SDK 会以 "No simulation or real robot is started." 失败，'
                        '那是**后端缺失**，不是桥接的缺陷。'),
    ]

    env = [
        SetEnvironmentVariable('ASTRIBOT_LOG', '1'),
        SetEnvironmentVariable('ROBOT_TYPE', LaunchConfiguration('robot_type')),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
    ]

    mujoco = ExecuteProcess(
        cmd=['python3', 'astribot_simulation.py'],
        cwd=LaunchConfiguration('sim_root'),
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('target'), "' == 'sim' and '",
             LaunchConfiguration('sim_root'), "' != ''"])),
    )

    container = Node(
        package='astribot_trajectory_bridge',
        executable='bridge_container',
        name='astribot_bridge_container',
        output='screen',
        respawn=False,
        parameters=[
            LaunchConfiguration('chassis_params_file'),
            LaunchConfiguration('arm_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'declared_target': LaunchConfiguration('target'),
                'allow_write_to_real': LaunchConfiguration('allow_write_to_real'),
                'enable_slam_correction':
                    LaunchConfiguration('enable_slam_correction'),
                'pose_source': LaunchConfiguration('pose_source'),
                'enable_waypoints_service':
                    LaunchConfiguration('enable_waypoints_service'),
            },
        ],
    )

    return LaunchDescription(declared + env + [mujoco, container])
