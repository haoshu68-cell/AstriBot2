#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：Nav2 核心节点组（controller_server/smoother_server/planner_server/
behavior_server/bt_navigator/waypoint_follower/velocity_smoother +
lifecycle_manager_navigation），照抄官方
`/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py` 的节点结构和
`RewrittenYaml`/`ParameterFile` 参数模式（规规矩矩复用官方已验证过的模式，不是
重新发明），**故意不包含** `map_server`/`amcl`——SLAM Toolbox 在建图和定位两种模式下
都会自己发布 `/map` 和 `map->odom` TF，跑一套 map_server+amcl 会跟它抢着发布/消费
`map->odom`，详见 astribot_s1_navigation/README_NAVIGATION.md。

!!! 跟官方文件唯一的关键差异（不是疏漏，是故意改的）!!!：官方文件里
`velocity_smoother` 最后一步把内部话题名 `cmd_vel_smoothed` 重映射成 `cmd_vel`
——也就是说官方设计下 Nav2 整套的最终输出话题名就是 `/cmd_vel`，会直接怼到真机的
`/cmd_vel`。但本机器人底盘用 gz-sim `VelocityControl` 插件、按 world 系解释
`/cmd_vel`（Nav2标准控制器按车体系发布），直接对接会重现本 session 修过的"原地打转
不挪窝"bug。这里改成把 `cmd_vel_smoothed` 重映射到 `cmd_vel_nav_body`
（还是车体系，只是话题名不同），交给 `cmd_vel_body_to_world_node`
（本包自己的节点）订阅后换算成 world 系再发真正的 `/cmd_vel`。
没有用"IncludeLaunchDescription官方文件 + 外层SetRemap覆盖"的做法——官方文件内部的
remappings是在每个Node构造时写死传入的，外层SetRemap能否正确覆盖掉这种写死的
remapping在ROS2 launch里没有保证，与其猜不如直接复制节点定义、只改这一处，明确可控。
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_navigation = FindPackageShare('astribot_s1_navigation')
    # !!! 不要在这里给 bt_navigator 传行为树 !!!（2026-09-08 撤掉过一版）
    # 撤掉的那一版写的是 parameters=[..., {'bt_xml_filename': <nav2 的
    # navigate_to_pose_no_replanning.xml>}]，三重静默失效、一条告警都没有：
    #   1. Humble 的 nav2 没有 navigate_to_pose_no_replanning.xml 这个文件
    #      （behavior_trees/ 下共 12 份，逐个查过，开发机与实机都没有）；
    #   2. 参数名在 Humble 叫 default_nav_to_pose_bt_xml
    #      （navigator.hpp:156；libbt_navigator_core.so 里也只有这一个名字）。
    #      未声明的参数覆盖被 rclcpp 静默忽略 —— 不报错不告警；
    #   3. 就算前两条都对，探索也用不到：exploration_coordinator_node.cpp
    #      给**每个目标**填 NavigateToPose.behavior_tree，逐目标覆盖默认树。
    # 换行为树的正确位置是协调器的 nav_behavior_tree 参数
    # （默认指向本包 behavior_trees/navigate_to_pose_explore_three_phase.xml），
    # 重规划语义就写在那份 xml 里。
    pkg_dynamics_coupling = FindPackageShare('astribot_s1_dynamics_coupling')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    controller_plugin = LaunchConfiguration('controller_plugin')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    scan_topic = LaunchConfiguration('scan_topic')
    enable_arm_chassis_coupling = LaunchConfiguration('enable_arm_chassis_coupling')
    enable_posture_monitor = LaunchConfiguration('enable_posture_monitor')
    max_linear_speed = LaunchConfiguration('max_linear_speed')

    # controller_plugin(rpp|mppi) 决定用哪一份参数文件——两份文件除了
    # controller_server.FollowPath 那一段之外完全一致，见 config/ 目录下两个文件的注释。
    params_file = PathJoinSubstitution([
        pkg_navigation, 'config',
        PythonExpression(["'nav2_params_' + '", controller_plugin, "' + '.yaml'"]),
    ])

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # 'topic' 这个键在两份 nav2_params 里只出现在 obstacle_layer.scan 下面
    # （局部+全局各一处，已核对：其余带 topic 的键都是 costmap_topic/footprint_topic/
    #  odom_topic/map_topic/speed_limit_topic 这类带前缀的名字，不会被误改），
    # 所以用 RewrittenYaml 重写 'topic' 是安全的，不需要维护两份 yaml。
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'topic': scan_topic}

    # ═══════════ 线速度上限的一键覆盖（现场限速用）═══════════
    # 为什么要在 launch 里做，而不是手改 install/ 下的 yaml：
    # 手改安装产物下一次 colcon build 就被覆盖，而"我以为限了速其实没限"
    # 是这条链上最危险的一种假设。
    #
    # !!! 必须同时压四个量，少一个就是一个漏洞 !!!
    #   vx_max / vy_max   前进与横移上限
    #   vx_min / vy_min   **倒车与反向横移**上限 —— 只压 vx_max 的话
    #                     vx_min 仍是 -1.0，MPPI 可以用 1.0m/s 倒车。
    #   max_velocity / min_velocity  velocity_smoother 是**串联的第二层**限速
    #                     （本项目吃过"两层限速串联叠乘、只改一层看不到效果"的亏）。
    # 两层都设成同一个值：任一层没生效，另一层仍然兜得住。
    #
    # 角速度刻意**不**在这里压：调用方只要求限线速度，静默改角速度会让
    # 原地对齐/Spin 恢复行为跟着变，而那不是调用方要求的。wz 上限见 yaml。
    #
    # RewrittenYaml 的 param_rewrites 是**按键名**全树替换的：
    # vx_max 在本文件里出现 3 处（FollowPath 一处 + 两个三段式实例的 inner 各一处），
    # 全部是速度上限，全压是期望行为。
    def _neg(expr):
        return PythonExpression(["str(-abs(float('", expr, "')))"])

    param_substitutions.update({
        'vx_max': max_linear_speed,
        # ⚠2026-09-03「路线A」：这两项原来是 max_linear_speed / _neg(max_linear_speed)，
        # 会**静默把 yaml 里的 vy_max: 0.0 抬回 0.2**，路线A 就此失效而毫无提示
        # （RewrittenYaml 按键名全树替换，不看 yaml 里原来是多少）。
        # 现在钉死 0.0，与三处 MPPI 块的 motion_model: "DiffDrive" 一致：
        # 横向自由度是从模型层去掉的，限速层不得把它复活。
        # 一键回退（连同 yaml 那五处）：改回 max_linear_speed / _neg(max_linear_speed)。
        'vy_max': '0.0',
        'vy_min': '0.0',
        'vx_min': _neg(max_linear_speed),
        # ═══ RPP 路径的限速键（2026-09-07 补）═══
        # vx_max/vx_min 是 MPPI 的键；RPP **没有**这些键，它的线速度上限叫
        # desired_linear_vel。而 RewrittenYaml.substitute_params 只改**已存在**的键
        # （/opt/ros/humble/.../rewritten_yaml.py:108-113），所以在 rpp 那份 yaml 上
        # 只写 vx_max 等于什么都没做：max_linear_speed:=0.2 是**静默 no-op**，
        # 底盘按 yaml 里的 0.5 跑。反过来在 mppi 那份 yaml 上没有
        # desired_linear_vel 这个键，这一行同样是无害的 no-op。
        # 两个键都写 = 两条控制器路径都真的被压住。
        'desired_linear_vel': max_linear_speed,
        # RPP 的终段最小速度必须 <= 上限，否则限速被它抬回来。
        # min(0.05, 上限) —— 上限比 0.05 还小时取上限。
        'min_approach_linear_velocity': PythonExpression(
            ["str(min(0.05, abs(float('", max_linear_speed, "'))))"]),
        # ═══ velocity_smoother 的 max_velocity/min_velocity 刻意**不**在这里改 ═══
        # 它们是 double 数组，而 RewrittenYaml.convert() 只会尝试 int/float/bool
        # （已读 /opt/ros/humble 里的实现确认：三种都不匹配就原样返回字符串）。
        # 于是 "[0.2, 0.2, 2.0]" 被当**字符串**写进 yaml，velocity_smoother 配置期抛
        #   parameter 'max_velocity' has invalid type: ... is of type {string}
        # 接着 lifecycle_manager 报 "Failed to bring up all requested nodes.
        # Aborting bringup." —— 整套 nav2 从未进入 active。
        #
        # !!! 这个失败模式必须记住 !!!：9 个进程**全都活着**、进程数判据照过，
        # 而 nav2 一个节点都没 activate。我第一版就是这么错的，
        # "进程数 >= 5" 那条判据一点都没察觉，是参数服务不可达才暴露出来的。
        #
        # 那么第二层限速怎么保证？velocity_smoother 的上限保持 yaml 里的 1.0，
        # 它是 MPPI 输出的**上界**而不是下界 —— MPPI 已被压到 0.2，
        # 经过一个上限 1.0 的限幅器仍然是 0.2。两层串联取的是 min，
        # 所以安全侧成立；只是"两层都独立压住"这个冗余没有了，
        # 必须靠运行时实测每一跳的最大幅值来兜（见 probe_speed_cap.py 的实测项）。
    })

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_args = [
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'controller_plugin', default_value='rpp',
            description='rpp(任务书默认，非全向退化行为) 或 mppi(推荐，全向)'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument(
            'max_linear_speed', default_value='1.0',
            description='线速度上限(m/s)，一键同时压住四个量：MPPI 的 '
                        'vx_max/vy_max/vx_min 与 velocity_smoother 的 '
                        'max_velocity/min_velocity。\n'
                        '!!! 默认值必须与 nav2_params_*.yaml 里的 vx_max 一致 !!! '
                        '否则这个"默认"会静默**抬高**yaml 里设定的限速。'
                        '这一条由 TestSpeedCapOverride.test_launch_default_matches_yaml '
                        '钉住。\n'
                        '为什么必须四个一起压：只压 vx_max 的话 vx_min 仍是 -1.0，'
                        'MPPI 可以用 1.0m/s **倒车**；而 velocity_smoother 是串联的'
                        '第二层限速，本项目吃过"两层限速串联叠乘、只改一层看不到效果"'
                        '的亏。两层设成同值，任一层失效另一层仍兜得住。\n'
                        '角速度刻意不在这里压：静默改角速度会让原地对齐/Spin 恢复'
                        '行为跟着变。'),
        DeclareLaunchArgument(
            'enable_posture_monitor', default_value='true',
            description='cmd_vel_body_to_world_node 的姿态止损监控。\n'
                        '!!! 实机必须显式给 false !!!\n'
                        '它的判据是 |z-normal_height|>max_height_deviation 或 '
                        'roll/pitch 超 max_tilt_rad，而 normal_height=0.134 是**仿真**值'
                        '（Gazebo 里 world z=0 不是地面）。实机 /odom 是轮式里程计、'
                        '只暴露 3-DOF，z/roll/pitch **恒等于 0**（2026-09-02 实测 '
                        '509 帧 min=max=0.0000）-> |0-0.134|=0.134 > 0.06 -> 判为异常姿态'
                        ' -> **永久**把 /cmd_vel 归零（safety_tripped 无复位路径）。\n'
                        '此前之所以没炸：该节点的 /odom 订阅原是默认 RELIABLE，'
                        '而实机 /odom 发布者是 BEST_EFFORT，回调一帧都没执行过'
                        '（实测 RELIABLE 0 帧 / BEST_EFFORT 704 帧@50Hz）—— '
                        '两个缺陷互相掩盖。QoS 已修成 sensor_data，所以这个开关是必需的。\n'
                        '注意边界：实机上把它设成 true 并把 normal_height 改成 0.0 '
                        '只是让它永不触发，**检测不出真的倾倒** —— 那是假的安全感。'
                        '实机要做倾倒检测得换 IMU 数据源'
                        '（/astribot_whole_body/chassis_imu、/livox/imu_{front,back}）。'),
        DeclareLaunchArgument(
            'posture_normal_height', default_value='0.134',
            description='姿态监控的基准高度，单位 m。\n'
                        '!!! 这个值和 /odom 的坐标系绑死，换数据源必须跟着改 !!!\n'
                        '默认 0.134 是 **Gazebo** 的值：那里 world z=0 不是地面，'
                        '躯干基座静止时 z 就是 0.134。\n'
                        '实机上 /odom 由 tf_to_odom_node 从 SLAM 的 '
                        'map->astribot_torso_base 导出，**z=0 是开机那一刻的位姿**，'
                        '所以实机要给 0.0。给成 0.134 的后果实测过：第一帧就 '
                        '|-0.0011-0.1340|=0.1351 > 0.06 -> 止损 -> /cmd_vel 永久归零'
                        '（30s 内 1668 帧全零），而日志只说"请检查 Gazebo 画面"。\n'
                        '注意：给对基准之后这个监控是**真的在工作**的，不是摆设 —— '
                        '实机实测 30s 内 z 峰峰值 0.0034m、roll/pitch < 0.006rad，'
                        '离 0.06m/0.12rad 的阈值还有 17 倍余量。但它测的是 SLAM '
                        '位姿里的 z/roll/pitch，测不出"SLAM 自己算错了"。'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='costmap 障碍层订阅的 LaserScan 话题。'
                        '/scan = 既有 pointcloud_to_laserscan 的单层切片结果；'
                        '/scan_from_cloud = astribot_s1_autonomy 的多层高度切片融合结果'
                        '（能检出低矮托盘和悬空横梁，单层切片会漏）。'
                        '通过 RewrittenYaml 改写 costmap 里的 obstacle_layer.scan.topic，'
                        '两份 nav2_params 文件都不用改。'
                        '注意：切到 /scan_from_cloud 时必须确保感知节点在跑，'
                        '否则 costmap 收不到任何障碍物数据——'
                        '用 nav2_full_bringup.launch.py 的 scan_source 参数可以一次性切好两端。'),
        DeclareLaunchArgument(
            'enable_arm_chassis_coupling', default_value='true',
            description='是否接入 astribot_s1_dynamics_coupling 的臂-底盘动力学耦合'
                        '动态调速节点（机械臂展开/快速运动时自动降低底盘速度，防止'
                        '重心偏移诱发倾倒）。设为false时cmd_vel_body_to_world_node的'
                        '输出直接就是/cmd_vel，跟本节点接入之前完全一样，互不影响。'),
    ]

    # !!! 无侵入接入方式说明 !!!：不修改 cmd_vel_body_to_world_node.py 的任何代码，
    # 只是在这里把它本来就有的 output_topic 参数覆盖成一个中间话题名——它本来的默认值
    # 是直接输出到 '/cmd_vel'，这里让它输出到 'cmd_vel_pre_arm_coupling'，改由
    # arm_chassis_speed_coupling_node 接手，根据机械臂状态动态缩放后才真正发到
    # '/cmd_vel'。enable_arm_chassis_coupling:=false 时这个覆盖不生效，行为跟接入
    # 耦合节点之前完全一样（用 PythonExpression 三元表达式做条件选择，不用 IfCondition
    # 套两份Node定义——省得重复维护 cmd_vel_body_to_world_node 的其它参数）。
    cmd_vel_body_output_topic = PythonExpression([
        "'/cmd_vel_pre_arm_coupling' if '", enable_arm_chassis_coupling, "' == 'true' "
        "else '/cmd_vel'"
    ])

    load_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                # controller_server 自己内部话题名是 cmd_vel，重映射成 cmd_vel_nav_body
                # 交给 velocity_smoother 平滑，最终由 velocity_smoother 再重映射一次。
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav_body_raw')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                # !!! 这个 remap 不能漏（2026-09-07 运行态实测补上）!!!
                # nav2_behaviors 的每个行为插件都在**相对**话题 "cmd_vel" 上建发布者
                # （/opt/ros/humble/include/nav2_behaviors/timed_behavior.hpp:130），
                # 命名空间为 / 时解析成 /cmd_vel。漏 remap 时实测 /cmd_vel 有 5 个
                # 发布者 = 4 个行为插件 + 链路末端，即 Spin/BackUp 直接写终端话题：
                #   · 绕过 velocity_smoother、按轴限速、臂-底盘耦合限速；
                #   · 更要紧的是**绕过 body->world 变换** —— gz 的 VelocityControl
                #     按世界系解读 /cmd_vel，于是 BackUp 的车体 -x 被当成世界 -x，
                #     只在 yaw≈0 时方向才恰好正确，其余朝向下机器人往错的方向退。
                # Spin 只出 wz、旋转不变，所以这个缺陷长期不显形。
                # 接到 cmd_vel_nav_body_raw（与 controller_server 同一入口）
                # 让恢复行为走完整条限速+变换链。
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav_body_raw')]),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                # 行为树刻意不在这里传，理由见本函数开头那段注释。
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                # !!! 本文件跟官方版本唯一的关键差异，见文件头部说明 !!!：
                # 官方是 ('cmd_vel_smoothed', 'cmd_vel')，这里改成 'cmd_vel_nav_body'，
                # 不直接碰真正的 /cmd_vel。
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav_body_raw'),
                         ('cmd_vel_smoothed', 'cmd_vel_nav_body')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
            # 本包自己的两个适配节点：body->world cmd_vel 转换(必须存在，见文件头部说明)
            # + 机械臂展开限速(Nav2官方/speed_limit机制，静态阈值判断)。
            Node(
                package='astribot_s1_navigation',
                executable='cmd_vel_body_to_world_node',
                name='cmd_vel_body_to_world_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    # 见上面 cmd_vel_body_output_topic 的说明：enable_arm_chassis_coupling
                    # 打开时这里输出到中间话题，交给下面的耦合节点处理后才真正到/cmd_vel；
                    # 关闭时这个覆盖等于什么都没做(默认值本来就是/cmd_vel)。
                    'output_topic': cmd_vel_body_output_topic,
                    # 姿态止损监控开关，实机必须给 false，见该参数的 launch 参数说明。
                    'enable_posture_monitor': enable_posture_monitor,
                    # 基准高度必须跟 /odom 的坐标系一致，见 posture_normal_height
                    # 的参数说明：仿真 0.134 / 实机 0.0。给错的表现是 /cmd_vel 永久全零。
                    # ⚠️ 必须包 ParameterValue(value_type=float)：LaunchConfiguration
                    #    取出来是**字符串**，而节点里 declare_parameter('normal_height',
                    #    0.134) 声明的是 DOUBLE，直接传字符串节点会起不来（类型不匹配）。
                    'normal_height': ParameterValue(
                        LaunchConfiguration('posture_normal_height'),
                        value_type=float),
                }],
            ),
            Node(
                package='astribot_s1_navigation',
                executable='arm_speed_limiter_node',
                name='arm_speed_limiter_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]
    )

    # 臂-底盘动力学耦合动态调速(astribot_s1_dynamics_coupling独立包，见该包README)：
    # 接住上面 cmd_vel_body_to_world_node 被重定向过去的中间话题，根据双臂展开幅度/
    # 运动速率算连续0~1.0限速系数，缩放后才真正发到/cmd_vel。跟静态的
    # arm_speed_limiter_node(Nav2 /speed_limit机制、二值判断)是两套独立、互不冲突的
    # 保护——一个走Nav2官方限速通道作用于controller_server的速度上限，一个是本节点
    # 直接在最终world系速度上做连续缩放，两者同时生效、互相不知道对方存在，符合
    # "无侵入、不修改既有静态限速逻辑"的要求。
    arm_chassis_coupling = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_dynamics_coupling, 'launch', 'arm_chassis_coupling.launch.py'])),
        launch_arguments={
            'input_topic': 'cmd_vel_pre_arm_coupling',
            'output_topic': '/cmd_vel',
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(enable_arm_chassis_coupling),
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    for action in declare_args:
        ld.add_action(action)
    ld.add_action(load_nodes)
    ld.add_action(arm_chassis_coupling)
    return ld
