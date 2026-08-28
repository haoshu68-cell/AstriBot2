#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图来源分发：按 config/map_source.yaml 决定 /map 与 map→odom 谁来提供。

============================ 两个正交的轴 ============================
    map_source   —— 谁提供 /map        : sim_slam | real_file | real_live
    localization —— 谁提供 map→odom    : slam | ground_truth

与已有 `mode:={mapping,localization}` 的关系（**刻意不改那个参数的语义**）：

    map_source=sim_slam   ≡  原来的 mode:=mapping     （slam_toolbox 在线建图）
    map_source=real_file  ─┐
    map_source=real_live  ─┴─ 新分支，不走 slam_toolbox

原来的 `mode:=localization`（slam_toolbox 加载 posegraph 做扫描匹配定位）
仍然保留、语义不变，它对应的是"用我们自己建的图 + 真扫描匹配"，
即 map_source=real_file 配 localization=slam 那条路线（路线 A）的前身。
路线 A 现在**暂时拒绝**：扫描匹配要求仿真几何与地图一致，而 Gazebo 里跑的是
AWS 仓库、地图来自真实场地，不匹配时定位必然发散。等"从占据栅格挤出 Gazebo
世界"的工具做出来再开。

========================= 为什么这里可以跑 map_server =========================
README_NAVIGATION.md §4.3 写着"不跑 map_server / amcl"，理由是 slam_toolbox
自己发 /map 和 map→odom，再跑一套会抢发布权。**那个理由成立，但适用条件是
"slam_toolbox 在跑"。** real_* 模式下 slam_toolbox 不启动，于是：

    /map      由 map_server（real_file）或 map_domain_relay（real_live）提供
    map→odom  由静态 TF 提供（ground_truth）

各只有一个发布者，不存在抢发布权。全局代价地图的 static_layer 本来就是
`map_subscribe_transient_local: True`，对两种来源都适用，代价地图侧一行不用改。

用法：
    # 用配置文件里的值
    ros2 launch astribot_s1_perception map_provider.launch.py

    # 临时覆盖（留空的参数不下发，不会把 yaml 里的正确值冲掉）
    ros2 launch astribot_s1_perception map_provider.launch.py \
        map_source:=real_file map_yaml_path:=/abs/path/to/map.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from astribot_s1_perception.map_source_config import (
    MapSourceConfigError,
    check_live_transport_env,
    needs_slam_adapter,
    publishes_static_map_to_odom,
    require,
    resolve,
)


def _build(context, *args, **kwargs):
    overrides = {
        key: LaunchConfiguration(key).perform(context)
        for key in ('map_source', 'localization', 'map_yaml_path')
    }
    config_file = LaunchConfiguration('map_source_file').perform(context) or None
    config_path, params, map_source, localization = resolve(config_file, overrides)

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    actions = []

    print(f'[map_provider] 配置文件 = {config_path}')
    print(f'[map_provider] map_source = {map_source}  localization = {localization}')

    # ---------------- /map 的提供者 ----------------
    if map_source == 'sim_slam':
        # 这条分支不在本文件里起 slam_toolbox —— 它由 perception_slam_bringup
        # 的 slam_mapping 分支负责，避免同一个节点有两处启动入口。
        print('[map_provider] /map 由 slam_toolbox 在线建图提供（本文件不额外起节点）')

    elif map_source == 'real_file':
        map_yaml = require(params, 'map_yaml_path', str, default='')
        if not map_yaml:
            raise MapSourceConfigError(
                'map_source=real_file 但 map_yaml_path 为空。'
                '要给 **yaml** 的绝对路径（不是 pgm）—— map_server 读的是 yaml，'
                '其中的 image 字段相对该 yaml 所在目录解析。')
        if not os.path.isfile(map_yaml):
            raise MapSourceConfigError(f'map_yaml_path 指向的文件不存在: {map_yaml}')

        actions.append(Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': use_sim_time == 'true',
                # topic_name/frame_id 用默认 /map 与 map
            }],
        ))
        # map_server 是 lifecycle 节点：不挂 manager 的话它停在 UNCONFIGURED、
        # 永远不发 /map，而且**不报错**。表现是"nav2 一直等地图"。
        actions.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time == 'true',
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ))
        print(f'[map_provider] /map 由 map_server 提供: {map_yaml}')

    else:  # real_live
        # ROS_LOCALHOST_ONLY 必须是 0，否则中继发出去没人听（实测：
        # localhost_only=1 与 =0 的参与者互相发现不了，同机同域也不行）。
        # 隔离靠 domain 分离而不是靠这个开关 —— 详见
        # map_source_config.check_live_transport_env() 的说明。
        transport_reason = check_live_transport_env(
            os.environ.get('ROS_LOCALHOST_ONLY', '0'))
        if transport_reason is not None:
            raise MapSourceConfigError(transport_reason)

        remote_domain = require(params, 'remote_domain_id', int, 25)
        local_domain = require(params, 'local_domain_id', int, 25)
        if remote_domain == local_domain:
            # 同域：真机的 /map 在本机栈里本来就直接可见，不需要中继
            # （起了反而是把同一张图回环发布给自己）。所以这里跳过节点，
            # 而不是像以前那样拒绝启动。
            #
            # 但要把代价讲清楚：跨域隔离是原来那条"仿真 /cmd_vel 绝无到真机
            # 的通路"的**唯一网络层保证**，同域后它不再成立。
            print(f'[map_provider] /map 由真机直接提供（remote/local 同为 domain '
                  f'{local_domain}，跳过 map_domain_relay）')
            print('[map_provider] !!! 跨域隔离已关闭：本机的 /cmd_vel、'
                  '/wheel_effort_controller/commands 对真机可见。'
                  '要恢复网络层隔离就把 local_domain_id 改成与真机不同的值，'
                  '并把整条栈起在那个 domain 上 !!!')
        else:
            relay = Node(
                package='astribot_s1_perception',
                executable='map_domain_relay',
                name='map_domain_relay',
                output='screen',
                parameters=[{
                    'remote_domain_id': remote_domain,
                    'local_domain_id': local_domain,
                    'remote_map_topic': require(params, 'remote_map_topic', str, '/map'),
                    'local_map_topic': require(params, 'local_map_topic', str, '/map'),
                    'relay_timeout_sec': require(params, 'relay_timeout_sec', float, 30.0),
                }],
            )
            actions.append(relay)
            # 中继起不来（超时、domain 配错、网络不通）时整条链没有意义，
            # 让它带着整个 launch 一起停，而不是留一个"等地图"的假活状态。
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=relay, on_exit=[EmitEvent(event=Shutdown(
                    reason='map_domain_relay 退出：跨机地图中继失败'))])))
            print(f'[map_provider] /map 由 map_domain_relay 跨机中继提供: '
                  f'domain {remote_domain} -> {local_domain}')

    # ---------------- map→odom 的提供者 ----------------
    if publishes_static_map_to_odom(localization):
        pose = params.get('map_to_odom_xyz_yaw') or [0.0, 0.0, 0.0, 0.0]
        if len(pose) != 4:
            raise MapSourceConfigError(
                f'map_to_odom_xyz_yaw 必须是 4 个数 [x, y, z, yaw]，收到 {len(pose)} 个')
        x, y, z, yaw = (float(v) for v in pose)
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static',
            output='screen',
            arguments=[
                '--x', str(x), '--y', str(y), '--z', str(z),
                '--yaw', str(yaw), '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            parameters=[{'use_sim_time': use_sim_time == 'true'}],
        ))
        print(f'[map_provider] map→odom 由静态 TF 提供: '
              f'xyz=({x}, {y}, {z}) yaw={yaw}')

        if params.get('validate_start_cell', True):
            checker = Node(
                package='astribot_s1_perception',
                executable='map_start_cell_check',
                name='map_start_cell_check',
                output='screen',
                parameters=[{
                    'map_topic': require(params, 'local_map_topic', str, '/map'),
                    'map_to_odom_xyz_yaw': [x, y, z, yaw],
                    'start_cell_clearance_m': require(
                        params, 'start_cell_clearance_m', float, 0.25),
                    'use_sim_time': use_sim_time == 'true',
                }],
            )
            actions.append(checker)
            # 校验不通过 → 非零退出 → 停掉整个 launch。
            # 不这样做的话，机器人会"出生在墙里"然后由规划器报
            # "Starting point in lethal space!"，那个症状离根因三层远。
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=checker,
                on_exit=lambda event, ctx: (
                    [EmitEvent(event=Shutdown(
                        reason='出生点栅格校验未通过（见上面的 ERROR）'))]
                    if event.returncode != 0 else []))))
    elif needs_slam_adapter(localization):
        # 外部 SLAM（Voxel-SLAM 等）提供 /map 与 map→odom，经适配层归一化。
        # 这里**不发**静态 TF：那会让 odom 有两个父源，位姿反复跳，
        # 症状看起来像"定位漂移"（与 sim_slam + ground_truth 同一个坑）。
        adapter_params = os.path.join(
            get_package_share_directory('astribot_s1_perception'),
            'config', 'slam_adapter_params.yaml')
        adapter = Node(
            package='astribot_s1_perception',
            executable='slam_adapter_node',
            # 刻意不设 name=：节点名 remap 是进程级的，remap 后 yaml 键匹配不上、
            # 整份参数文件静默失效并回落到代码默认值，且没有任何报错。
            output='screen',
            parameters=[
                adapter_params,
                # 只有这两项由配置轴决定，其余全部来自 yaml
                {'use_sim_time': use_sim_time == 'true',
                 'map_topic': require(params, 'local_map_topic', str, '/map')},
            ],
        )
        actions.append(adapter)
        # 适配层退出（源超时、契约违规）意味着 /map 和 map→odom 都没了，
        # 下游只会表现成"nav2 卡在启动"。让它带着整个 launch 一起停。
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=adapter, on_exit=[EmitEvent(event=Shutdown(
                reason='slam_adapter_node 退出：外部 SLAM 接入失败（见上面的 ERROR）'))])))
        print(f'[map_provider] /map 与 map→odom 由外部 SLAM 提供，'
              f'经 slam_adapter_node 归一化（参数 {adapter_params}）')
    else:
        print('[map_provider] map→odom 由 slam_toolbox 提供（扫描匹配）')

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_source_file', default_value='',
            description='地图来源配置文件路径。留空用包内 config/map_source.yaml'),
        DeclareLaunchArgument(
            'map_source', default_value='',
            description='覆盖配置里的 map_source（sim_slam|real_file|real_live）。'
                        '留空则用配置文件的值'),
        DeclareLaunchArgument(
            'localization', default_value='',
            description='覆盖配置里的 localization（slam|ground_truth|external）。留空同上'),
        DeclareLaunchArgument(
            'map_yaml_path', default_value='',
            description='覆盖配置里的 map_yaml_path。留空同上'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_build),
    ])
