

"""Nav2 bringup for Astribot S1."""

import math
import yaml

from launch import LaunchDescription
from launch.utilities import perform_substitutions
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction,
    RegisterEventHandler, EmitEvent,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from astribot_logging import log_level as default_log_level
from astribot_logging.launch import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def validate_voxel_layers(config, plugin):
    """Reject unsupported/truncated voxel profiles before starting Nav2.

    Humble stores only 16 vertical cells. A larger configured size is silently
    clamped by the native grid, so configuration presence is not height proof.
    """
    if plugin != 'nav2_costmap_2d::VoxelLayer':
        return
    while isinstance(config, dict) and len(config) == 1 and 'local_costmap' not in config:
        config = next(iter(config.values()))
    for name in ('local_costmap', 'global_costmap'):
        layer = config[name][name]['ros__parameters']['obstacle_layer']
        n, dz, origin = layer['z_voxels'], layer['z_resolution'], layer['origin_z']
        if type(n) is not int or not 1 <= n <= 16:
            raise ValueError(f'{name}: VoxelLayer requires 1..16 vertical cells')
        if not all(type(x) in (int, float) and math.isfinite(x) for x in (dz, origin)) or dz <= 0:
            raise ValueError(f'{name}: invalid voxel resolution/origin')
        top = origin + n * dz
        limits = [layer.get('max_obstacle_height', 2.0)]
        for source in layer.get('observation_sources', '').split():
            limits.append(layer[source].get('max_obstacle_height', 2.0))
        if not all(type(h) in (int, float) and math.isfinite(h) and origin < h < top for h in limits):
            raise ValueError(f'{name}: effective voxel height must cover every configured observation maximum')
        mark = layer.get('mark_threshold', 0)
        unknown = layer.get('unknown_threshold', 15)
        if type(mark) is not int or not 0 <= mark < n or type(unknown) is not int or not 0 <= unknown <= 16:
            raise ValueError(f'{name}: invalid voxel mark/unknown thresholds')


def generate_launch_description():
    pkg_navigation = FindPackageShare('astribot_s1_navigation')

    default_nav_to_pose_bt = PathJoinSubstitution([
        pkg_navigation, 'behavior_trees', 'navigate_to_pose_precise_goal.xml'])
    default_nav_through_poses_bt = PathJoinSubstitution([
        pkg_navigation, 'behavior_trees', 'navigate_through_poses_precise_goal.xml'])
    pkg_dynamics_coupling = FindPackageShare('astribot_s1_dynamics_coupling')

    geometry_mode = LaunchConfiguration('navigation_geometry_mode')
    policy_stage = LaunchConfiguration('navigation_policy_stage')
    policy_enabled = PythonExpression(["'", policy_stage, "' != 'off'"])
    social_stage = LaunchConfiguration('social_navigation_stage')
    social_enabled = PythonExpression(["'", social_stage, "' != 'off'"])
    def validate_social_stage(context):
        mode=geometry_mode.perform(context)
        if mode not in ('legacy','fixed_v2'):raise ValueError('invalid navigation_geometry_mode')
        if mode=='fixed_v2' and policy_stage.perform(context) not in ('p4','p5'):
            raise ValueError('fixed_v2 requires corridor policy p4 or p5')
        if policy_stage.perform(context) == 'off' and enable_arm_chassis_coupling.perform(context) == 'true':
            raise ValueError('arm speed limits require navigation policy; disable coupling explicitly for policy-off diagnostics')
        social = social_stage.perform(context)
        if social not in ('off', 'h2'):
            raise ValueError('social_navigation_stage must be off or h2')
        if social == 'h2' and policy_stage.perform(context) != 'p2':
            raise ValueError('H2 social arbitration requires navigation_policy_stage:=p2')
        return []
    policy_params = {'navigation_geometry_mode': ParameterValue(geometry_mode,value_type=str),
                     'navigation_policy_enabled': ParameterValue(policy_enabled, value_type=bool),
                     'social_navigation_enabled': ParameterValue(social_enabled, value_type=bool),
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
    enable_body_to_world = LaunchConfiguration('enable_body_to_world')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    enable_depth_obstacles = LaunchConfiguration('enable_depth_obstacles')

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
        # RewrittenYaml matches scalar keys recursively.  Rewriting the
        # generic key ``topic`` therefore changed the PointCloud2 sources
        # (head_depth/torso_depth) into the LaserScan adapter topic as well.
        # Keep the policy substitution scoped to the two obstacle-layer scan
        # entries so each sensor retains its declared message type/topic.
        'map_topic': LaunchConfiguration('map_topic'),
        'map_subscribe_transient_local': LaunchConfiguration('map_transient_local')}
    policy_scan_topic = PythonExpression(["'/navigation_policy/costmap_scan' if '", policy_stage,
                                          "' != 'off' else '", scan_topic, "'"])
    depth_observation_sources = PythonExpression([
        "'scan head_depth torso_depth' if '", enable_depth_obstacles,
        "' == 'true' else 'scan'"])
    for costmap in ('local_costmap', 'global_costmap'):
        param_substitutions[
            f'{costmap}.{costmap}.ros__parameters.obstacle_layer.scan.topic'] = policy_scan_topic
    # The global layer is deliberately scan-only: its source file has no
    # PointCloud2 entries and its static layer owns the long-lived map.  RGB-D
    # freshness is enforced by the local layer where the 3-D obstacle budget
    # is consumed.  Keep the switch explicit for both profiles so disabling
    # depth never leaves a hidden global subscription behind.
    param_substitutions[
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.observation_sources'] = \
        depth_observation_sources
    param_substitutions[
        'global_costmap.global_costmap.ros__parameters.obstacle_layer.observation_sources'] = 'scan'
    for costmap in ('local_costmap', 'global_costmap'):
        param_substitutions[f'{costmap}.{costmap}.ros__parameters.obstacle_layer.plugin'] = \
            LaunchConfiguration('obstacle_layer_plugin')

    def arrival_precision(context):
        profile = LaunchConfiguration('arrival_precision_profile').perform(context)
        if profile == 'standard':
            return {}
        if profile not in ('simulation_precision', 'hardware'):
            raise ValueError('unknown arrival precision profile: '+profile)
        if profile == 'simulation_precision' and use_sim_time.perform(context).lower() != 'true':
            raise ValueError('simulation_precision requires use_sim_time=true')
        filename = 'arrival_precision_hardware.yaml' if profile == 'hardware' else 'arrival_precision_sim.yaml'
        path = PathJoinSubstitution([pkg_navigation, 'config', filename])
        common = PathJoinSubstitution([pkg_navigation, 'config', 'arrival_motion.yaml'])
        with open(perform_substitutions(context, [common]), encoding='utf-8') as stream:
            values = yaml.safe_load(stream)
        with open(perform_substitutions(context, [path]), encoding='utf-8') as stream:
            values.update(yaml.safe_load(stream))
        return values

    def costmap_scan_adapter(context):
        if policy_stage.perform(context)=='off':return []
        with open(perform_substitutions(context,[params_file]),encoding='utf-8') as stream:
            config=yaml.safe_load(stream)
        marking=max(float(config[name][name]['ros__parameters']['obstacle_layer']['scan']['obstacle_max_range'])
                    for name in ('local_costmap','global_costmap'))
        return [Node(
            package='astribot_s1_navigation_policy_native',
            executable='costmap_scan_cpp', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'scan_topic': scan_topic,
                         'max_marking_range_m': marking}])]

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
        precision=arrival_precision(context)
        if precision:
            limits.update({
                'smoothing_frequency':50.0,'velocity_timeout':0.3,
                'normal_acceleration':[
                    precision['FollowPath.arrival.normal_acceleration'],
                    precision['FollowPath.arrival.normal_acceleration'],
                    precision['FollowPath.arrival.normal_angular_acceleration']],
                'normal_jerk':[
                    precision['FollowPath.arrival.normal_jerk'],
                    precision['FollowPath.arrival.normal_jerk'],
                    precision['FollowPath.arrival.normal_angular_jerk']]})
        return [Node(
            package='astribot_s1_path_tracking' if precision else 'nav2_velocity_smoother',
            executable='jerk_velocity_smoother' if precision else 'velocity_smoother',
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

    for costmap in ('local_costmap','global_costmap'):
        prefix=costmap+'.'+costmap+'.ros__parameters.'
        param_substitutions[prefix+'footprint_padding']=PythonExpression(["0.0 if '",geometry_mode,"' == 'fixed_v2' else 0.01"])
        param_substitutions[prefix+'obstacle_layer.footprint_clearing_enabled']=PythonExpression(["False if '",geometry_mode,"' == 'fixed_v2' else True"])

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    def validate_voxel_profile(context):
        plugin = LaunchConfiguration('obstacle_layer_plugin').perform(context)
        if plugin == 'nav2_costmap_2d::VoxelLayer':
            with open(configured_params.evaluate(context), encoding='utf-8') as stream:
                validate_voxel_layers(yaml.safe_load(stream), plugin)
        return []

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '0')

    declare_args = [
        DeclareLaunchArgument('navigation_geometry_mode', default_value='legacy'),
        DeclareLaunchArgument('navigation_policy_stage', default_value='off'),
        DeclareLaunchArgument('social_navigation_stage', default_value='off'),
        DeclareLaunchArgument('social_allow_simulation_truth', default_value='false'),
        DeclareLaunchArgument('arrival_precision_profile', default_value='standard',
                              description='standard / hardware / simulation_precision (ground-truth calibration)'),
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
            'enable_body_to_world', default_value='false',
            description='是否将 Nav2 车体系速度旋转为 VelocityControl 所需的 world 系分量'),
        DeclareLaunchArgument(
            'posture_normal_height', default_value='0.134',
            description='姿态监控的基准高度，单位 m'),
        DeclareLaunchArgument('log_level', default_value=default_log_level()),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='costmap 障碍层订阅的 LaserScan 话题'),
        DeclareLaunchArgument(
            'enable_arm_chassis_coupling', default_value='true',
            description='是否将臂展/关节活动限速接入 Nav2 上游约束（不写底盘速度，不代表动力学稳定性保证）'),
        DeclareLaunchArgument(
            'enable_depth_obstacles', default_value='true',
            description='是否将头部/躯干 RGB-D 点云接入局部代价图；关闭仅用于明确标注的激光-only 仿真回归，默认保持三维障碍门槛'),
    ]

    cmd_vel_body_output_topic = '/cmd_vel'

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
                parameters=[configured_params, policy_params, limits, arrival_precision(context), {
                    'progress_checker.plugin': 'astribot_s1_path_tracking::PolicyProgressChecker'}],
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
                parameters=[configured_params, {
                    'navigation_geometry_mode': ParameterValue(geometry_mode, value_type=str),
                    'navigation_policy_stage': ParameterValue(policy_stage, value_type=str)}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                # Shipped trees use the constrained FollowPath controller for
                # motion. Unadapted Spin/BackUp/DriveOnHeading would bypass it.
                parameters=[configured_params, {'behavior_plugins': ['wait']}],
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
                package='astribot_s1_navigation_policy_native',
                executable='cmd_vel_body_to_world_cpp',
                name='cmd_vel_body_to_world_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'output_topic': cmd_vel_body_output_topic,
                    'enable_posture_monitor': enable_posture_monitor,
                    'enable_body_to_world': ParameterValue(
                        enable_body_to_world, value_type=bool),
                    'normal_height': ParameterValue(
                        LaunchConfiguration('posture_normal_height'),
                        value_type=float),
                }],
            ),
            Node(
                package='astribot_trajectory_bridge_native',
                executable='arm_speed_limiter_cpp',
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
            'output_topic': '/navigation_policy/arm_speed_limit',
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(enable_arm_chassis_coupling),
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('obstacle_layer_plugin',
        default_value='nav2_costmap_2d::ObstacleLayer'))
    ld.add_action(stdout_linebuf_envvar)
    for action in declare_args:
        ld.add_action(action)
    ld.add_action(OpaqueFunction(function=validate_social_stage))
    ld.add_action(OpaqueFunction(function=validate_voxel_profile))
    ld.add_action(OpaqueFunction(function=costmap_scan_adapter))
    ld.add_action(Node(package='astribot_s1_task_arbiter_native', executable='task_arbiter_cpp',
        output='screen', parameters=[{'use_sim_time': use_sim_time, 'require_navigation_zones': True,
                                     'navigation_geometry_mode': ParameterValue(geometry_mode, value_type=str),
                                     'robot_base_frame': 'astribot_torso_base'}]))
    ld.add_action(Node(package='astribot_route_executor', executable='loop_route_executor',
        name='loop_route_executor', output='screen', parameters=[{'use_sim_time': use_sim_time, 'require_navigation_zones': True}]))
    operator_backend = Node(package='astribot_operator_backend', executable='operator_backend',
        name='operator_backend', output='screen', parameters=[{'require_navigation_zones': True, 'require_map_manager': True, 'use_sim_time': use_sim_time}])
    ld.add_action(DeclareLaunchArgument('map_manager_params_file',
        default_value=PathJoinSubstitution([FindPackageShare('astribot_map_manager'),
            'config', 'map_manager.yaml'])))
    ld.add_action(DeclareLaunchArgument('simulation_static_map_yaml', default_value=''))
    ld.add_action(Node(package='astribot_map_manager', executable='map_manager',
        name='map_manager', output='screen', parameters=[LaunchConfiguration('map_manager_params_file'),
            {'use_sim_time': use_sim_time,
             'simulation_static_map_yaml': LaunchConfiguration('simulation_static_map_yaml')}]))
    ld.add_action(DeclareLaunchArgument('enable_voxel_adapter', default_value='false'))
    ld.add_action(DeclareLaunchArgument('voxel_adapter_params_file',
        default_value=PathJoinSubstitution([FindPackageShare('astribot_map_manager'),
            'config', 'voxel_session_adapter.yaml'])))
    ld.add_action(Node(package='astribot_map_manager', executable='voxel_session_adapter',
        name='map_session_adapter', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_voxel_adapter')),
        parameters=[LaunchConfiguration('voxel_adapter_params_file'),
            {'use_sim_time': use_sim_time}]))
    def zone_server(context):
        consumers = ['global_costmap', 'local_costmap']
        if policy_stage.perform(context) != 'off':
            consumers.append('navigation_constraint')
        return [Node(package='astribot_navigation_zones', executable='zone_server',
                     output='screen', parameters=[{'use_sim_time': use_sim_time,
                                                  'required_consumers': consumers}])]
    ld.add_action(OpaqueFunction(function=zone_server))
    ld.add_action(operator_backend)
    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=operator_backend,
        on_exit=[EmitEvent(event=Shutdown(reason='Operator authority exited; stop managed navigation'))])))
    ld.add_action(DeclareLaunchArgument('operator_runtime_params_file',
        default_value=PathJoinSubstitution([FindPackageShare('astribot_operator_backend'),
            'config', 'mapping_runtime.yaml'])))
    ld.add_action(Node(package='astribot_operator_backend', executable='mapping_runtime',
        name='mapping_runtime', output='screen',
        parameters=[LaunchConfiguration('operator_runtime_params_file')]))
    # Explicit binding to one execution-environment inventory. Missing identity
    # keeps geometry unavailable; enabling this does not create an empty source.
    ld.add_action(DeclareLaunchArgument('enable_payload_ledger', default_value='false'))
    ld.add_action(DeclareLaunchArgument('payload_environment', default_value='simulation'))
    ld.add_action(DeclareLaunchArgument('payload_session_id', default_value=''))
    ld.add_action(DeclareLaunchArgument('payload_source_id', default_value=''))
    ld.add_action(DeclareLaunchArgument('payload_journal_path', default_value=''))
    ld.add_action(Node(package='astribot_s1_payload_state', executable='payload_state', output='screen',
        parameters=[{'use_sim_time':use_sim_time,
                     'environment':ParameterValue(LaunchConfiguration('payload_environment'), value_type=str),
                     'session_id':ParameterValue(LaunchConfiguration('payload_session_id'), value_type=str),
                     'source_id':ParameterValue(LaunchConfiguration('payload_source_id'), value_type=str),
                     'journal_path':ParameterValue(LaunchConfiguration('payload_journal_path'), value_type=str)}],
        condition=IfCondition(PythonExpression(["'",geometry_mode,"' == 'fixed_v2' and '",
            LaunchConfiguration('enable_payload_ledger'),"'.lower() == 'true'"]))))
    ld.add_action(Node(package='astribot_s1_robot_geometry',executable='geometry_state',output='screen',
        parameters=[{'use_sim_time':use_sim_time,
                     'payload_environment':ParameterValue(LaunchConfiguration('payload_environment'), value_type=str),
                     'payload_session_id':ParameterValue(LaunchConfiguration('payload_session_id'), value_type=str),
                     'payload_source_id':ParameterValue(LaunchConfiguration('payload_source_id'), value_type=str)}],
        condition=IfCondition(PythonExpression(["'",geometry_mode,"' == 'fixed_v2'"]))))
    ld.add_action(load_nodes)
    ld.add_action(arm_chassis_coupling)
    profile_path = PathJoinSubstitution([FindPackageShare('astribot_s1_navigation_policy'), 'config',
                                         PythonExpression(["'h2_simulation.json' if '", social_stage,
                                                           "' == 'h2' else 'simulation.json'"])])
    ld.add_action(Node(
        package='astribot_s1_navigation_policy_native', executable='envelope_coordinator_cpp',
        output='screen', parameters=[{'use_sim_time': use_sim_time, 'profile': profile_path,
                                     'navigation_geometry_mode': ParameterValue(geometry_mode, value_type=str)}],
        condition=IfCondition(PythonExpression(["'", geometry_mode, "' == 'legacy' and '", policy_stage, "' != 'off'"]))))
    ld.add_action(Node(
        package='astribot_s1_navigation_policy_native', executable='fixed_envelope_cpp',
        output='screen', parameters=[{'use_sim_time': use_sim_time, 'profile': profile_path,
                                     'payload_environment': ParameterValue(LaunchConfiguration('payload_environment'), value_type=str),
                                     'payload_session_id': ParameterValue(LaunchConfiguration('payload_session_id'), value_type=str),
                                     'payload_source_id': ParameterValue(LaunchConfiguration('payload_source_id'), value_type=str),
                                     'navigation_geometry_mode': ParameterValue(geometry_mode, value_type=str)}],
        condition=IfCondition(PythonExpression(["'", geometry_mode, "' == 'fixed_v2' and '", policy_stage, "' != 'off'"]))))
    ld.add_action(Node(package='astribot_s1_navigation_policy', executable='policy_controller',
            output='screen', parameters=[{'use_sim_time': use_sim_time, 'scan_topic': scan_topic,
                                         'profile': profile_path,
                                         'navigation_policy_stage': ParameterValue(policy_stage, value_type=str),
                                         'navigation_geometry_mode': ParameterValue(geometry_mode, value_type=str),
                                         'social_navigation_stage': ParameterValue(social_stage, value_type=str),
                                         'social_allow_simulation_truth': ParameterValue(LaunchConfiguration('social_allow_simulation_truth'), value_type=bool),
                                         'corridor_file': LaunchConfiguration('corridor_file')}],
            condition=IfCondition(policy_enabled)))
    # Navigation admission and motion constraints are upstream decisions. This
    # node observes commands; it never owns or republishes /cmd_vel.
    ld.add_action(Node(package='astribot_s1_navigation_policy_native', executable='navigation_constraint_cpp',
        output='screen', parameters=[{'use_sim_time': use_sim_time, 'scan_topic': scan_topic,
                                     'command_topic': '/cmd_vel_nav_body_raw',
                                     'require_arm_speed_limit': ParameterValue(enable_arm_chassis_coupling, value_type=bool),
                                     'profile': profile_path, 'require_navigation_zones': True,
                                     'required_projection_cameras': ParameterValue(PythonExpression([
                                         "'[\"head_rgbd\",\"torso_rgbd\"]' if '", enable_depth_obstacles,
                                         "' == 'true' else '[]'"]), value_type=str),
                                     'navigation_geometry_mode': ParameterValue(geometry_mode, value_type=str)}],
        condition=IfCondition(policy_enabled)))
    return ld
