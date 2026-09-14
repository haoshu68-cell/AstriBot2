

"""Nav2 bringup for Astribot S1."""

import math
import yaml

from launch import LaunchDescription
from launch.utilities import perform_substitutions
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from astribot_logging import log_level as default_log_level
from astribot_logging.launch import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_navigation = FindPackageShare('astribot_s1_navigation')

    default_nav_to_pose_bt = PathJoinSubstitution([
        pkg_navigation, 'behavior_trees', 'navigate_to_pose_precise_goal.xml'])
    default_nav_through_poses_bt = PathJoinSubstitution([
        pkg_navigation, 'behavior_trees', 'navigate_through_poses_precise_goal.xml'])
    pkg_dynamics_coupling = FindPackageShare('astribot_s1_dynamics_coupling')

    policy_stage = LaunchConfiguration('navigation_policy_stage')
    policy_enabled = PythonExpression(["'", policy_stage, "' != 'off'"])
    policy_params = {'navigation_policy_enabled': ParameterValue(policy_enabled, value_type=bool),
                     'navigation_policy_stage': ParameterValue(policy_stage, value_type=str)}
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

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,

        'default_nav_to_pose_bt_xml': default_nav_to_pose_bt,
        'default_nav_through_poses_bt_xml': default_nav_through_poses_bt,
        'topic': PythonExpression(["'/navigation_policy/costmap_scan' if '", policy_stage,
                                   "' != 'off' else '", scan_topic, "'"]),
        'map_topic': LaunchConfiguration('map_topic'),
        'map_subscribe_transient_local': LaunchConfiguration('map_transient_local')}

    def costmap_scan_adapter(context):
        if policy_stage.perform(context)=='off':return []
        with open(perform_substitutions(context,[params_file]),encoding='utf-8') as stream:
            config=yaml.safe_load(stream)
        marking=max(float(config[name][name]['ros__parameters']['obstacle_layer']['scan']['obstacle_max_range'])
                    for name in ('local_costmap','global_costmap'))
        return [Node(package='astribot_s1_navigation_policy',executable='costmap_scan_adapter',output='screen',
                     parameters=[{'use_sim_time':use_sim_time,'scan_topic':scan_topic,'max_marking_range_m':marking}])]

    # Preserve configured angular and zero-axis limits while capping both XY axes.
    def smoother_limits(context):
        with open(perform_substitutions(context, [params_file]), encoding='utf-8') as stream:
            config = yaml.safe_load(stream)['velocity_smoother']['ros__parameters']
        if policy_stage.perform(context) not in ('off', 'p2', 'p3', 'p4', 'p5'):
            raise ValueError('unsupported navigation policy stage')
        cap = float(max_linear_speed.perform(context))
        if policy_stage.perform(context) != 'off' and cap > 0.35:
            raise ValueError('simulation policy profile requires max_linear_speed <= 0.35')
        if not math.isfinite(cap) or cap <= 0.0:
            raise ValueError('max_linear_speed must be finite and positive')
        limits = {}
        for key in ('max_velocity', 'min_velocity'):
            values = list(config[key])
            values[:2] = [max(-cap, min(cap, float(v))) for v in values[:2]]
            limits[key] = values
        return [Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', output='screen', respawn=use_respawn,
            respawn_delay=2.0, parameters=[configured_params, limits],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav_body_raw'),
                                    ('cmd_vel_smoothed', 'cmd_vel_nav_body')])]


    def _neg(expr):
        return PythonExpression(["str(-abs(float('", expr, "')))"])

    param_substitutions.update({
        'vx_max': max_linear_speed,

        'vy_max': max_linear_speed,
        'vy_min': _neg(max_linear_speed),
        'vx_min': _neg(max_linear_speed),

        'desired_linear_vel': max_linear_speed,

        'min_approach_linear_velocity': PythonExpression(
            ["str(min(0.05, abs(float('", max_linear_speed, "'))))"]),

    })

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '0')

    declare_args = [
        DeclareLaunchArgument('navigation_policy_stage', default_value='off'),
        DeclareLaunchArgument('corridor_file', default_value=''),
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('map_transient_local', default_value='true'),
        DeclareLaunchArgument(
            'controller_plugin', default_value='rpp',
            description='rpp(任务书默认，非全向退化行为) 或 mppi(推荐，全向)'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument(
            'max_linear_speed', default_value='1.0',
            description='线速度上限(m/s)，一键同时压住四个量：MPPI 的 vx_max/vy_max/vx_min 与 velocity_smoother 的 max_velocity/min_ve'),
        DeclareLaunchArgument(
            'enable_posture_monitor', default_value='true',
            description='cmd_vel_body_to_world_node 的姿态止损监控'),
        DeclareLaunchArgument(
            'posture_normal_height', default_value='0.134',
            description='姿态监控的基准高度，单位 m'),
        DeclareLaunchArgument('log_level', default_value=default_log_level()),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='costmap 障碍层订阅的 LaserScan 话题'),
        DeclareLaunchArgument(
            'enable_arm_chassis_coupling', default_value='true',
            description='是否接入 astribot_s1_dynamics_coupling 的臂-底盘动力学耦合动态调速节点（机械臂展开/快速运动时自动降低底盘速度，防止重心偏移诱发倾倒）'),
    ]

    cmd_vel_body_output_topic = PythonExpression([
        "'/cmd_vel_pre_arm_coupling' if '", enable_arm_chassis_coupling, "' == 'true' "
        "else ('/cmd_vel_policy_input' if '", policy_stage, "' != 'off' else '/cmd_vel')"
    ])

    def controller_node(context):
        limits = {}
        if controller_plugin.perform(context) == 'mppi':
            with open(perform_substitutions(context, [params_file]), encoding='utf-8') as stream:
                config = yaml.safe_load(stream)['controller_server']['ros__parameters']
            minimum = float(config['FollowPath']['inner']['CurvatureSpeedLimitCritic']['v_min_turn'])
            cap = float(max_linear_speed.perform(context))
            if not math.isfinite(minimum) or minimum < 0 or not math.isfinite(cap) or cap <= 0:
                raise ValueError('invalid configured turn minimum or maximum linear speed')
            limits['FollowPath.inner.CurvatureSpeedLimitCritic.v_min_turn'] = min(minimum, cap)
        return [Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, policy_params, limits, {
                    'progress_checker.plugin': ParameterValue(PythonExpression([
                        "'astribot_s1_path_tracking::PolicyProgressChecker' if '", policy_stage,
                        "' != 'off' else 'nav2_controller::PoseProgressChecker'"]), value_type=str)}],
                arguments=['--ros-args', '--log-level', log_level],

                remappings=remappings + [('cmd_vel', 'cmd_vel_nav_body_raw')])]

    load_nodes = GroupAction(
        actions=[
            OpaqueFunction(function=controller_node),
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

                remappings=remappings + [('cmd_vel', 'cmd_vel_nav_body_raw')]),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,

                # BT client nodes keep their root namespace and need their own clock parameters.
                parameters=[configured_params, ParameterFile(RewrittenYaml(source_file=params_file,
                    root_key='navigation_executor', param_rewrites=param_substitutions,
                    convert_types=True), allow_substs=True), policy_params],
                arguments=['--ros-args', '--log-level', log_level,
                    '-r', 'bt_navigator:__ns:=/navigation_executor'],
                remappings=[('/tf','/tf'),('/tf_static','/tf_static'),
                    ('/navigation_executor/bond','/bond')] + [
                    ('/navigation_executor/bt_navigator/'+service,'/bt_navigator/'+service)
                    for service in ('change_state','get_state','get_available_states',
                        'get_available_transitions','get_transition_graph')]),
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
            OpaqueFunction(function=smoother_limits),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),

            Node(
                package='astribot_s1_navigation',
                executable='cmd_vel_body_to_world_node',
                name='cmd_vel_body_to_world_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,

                    'output_topic': cmd_vel_body_output_topic,

                    'enable_posture_monitor': enable_posture_monitor,

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

    arm_chassis_coupling = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_dynamics_coupling, 'launch', 'arm_chassis_coupling.launch.py'])),
        launch_arguments={
            'input_topic': 'cmd_vel_pre_arm_coupling',
            'output_topic': PythonExpression(["'/cmd_vel_policy_input' if '", policy_stage, "' != 'off' else '/cmd_vel'"]),
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(enable_arm_chassis_coupling),
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    for action in declare_args:
        ld.add_action(action)
    ld.add_action(OpaqueFunction(function=costmap_scan_adapter))
    ld.add_action(Node(package='astribot_s1_navigation_policy', executable='task_arbiter',
        output='screen', parameters=[{'use_sim_time': use_sim_time}]))
    ld.add_action(load_nodes)
    ld.add_action(arm_chassis_coupling)
    for executable in ('envelope_coordinator', 'policy_controller', 'final_protection'):
        ld.add_action(Node(package='astribot_s1_navigation_policy', executable=executable,
            output='screen', parameters=[{'use_sim_time': use_sim_time, 'scan_topic': scan_topic,
                                         'navigation_policy_stage': ParameterValue(policy_stage, value_type=str),
                                         'corridor_file': LaunchConfiguration('corridor_file')}],
            condition=IfCondition(policy_enabled)))
    return ld
