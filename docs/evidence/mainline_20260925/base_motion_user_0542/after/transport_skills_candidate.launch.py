"""MoveIt plus validated skill planner; robot simulation is launched separately."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from astribot_logging.launch import Node


def generate_launch_description():
    config = get_package_share_directory('astribot_s1_moveit_config')
    description = get_package_share_directory('astribot_s1_description')
    # Share the navigation baseline rather than maintaining a manipulation-only
    # camera default. Explicit session arguments still override each field.
    preset_path = os.path.join(description, 'config/simulation_navigation_full/launch_preset.yaml')
    with open(preset_path, encoding='utf-8') as stream:
        preset = yaml.safe_load(stream)
    if preset.get('schema_version') != 1:
        raise ValueError('Unsupported shared navigation camera preset')
    camera_defaults = preset['parameters']
    camera_paths = ('camera_profile', 'torso_camera_profile', 'camera_calibration_dir', 'camera_mounts_profile')
    camera_switches = ('use_camera', 'use_wrist_cameras', 'use_stereo_cameras')
    if any(type(camera_defaults[name]) is not bool for name in camera_switches):
        raise ValueError('Shared camera sensor switches must be boolean')
    with open(os.path.join(config, 'config/kinematics.yaml')) as stream:
        kinematics = yaml.safe_load(stream)
    with open(os.path.join(config, 'config/joint_limits.yaml')) as stream:
        limits = yaml.safe_load(stream)
    with open(os.path.join(config, 'config/ompl_planning.yaml')) as stream:
        ompl = yaml.safe_load(stream)
    ompl.update(planning_plugin='astribot_s1_manipulation/OmplPlannerExtension',
                request_adapters='default_planner_request_adapters/ResolveConstraintFrames',
                start_state_max_bounds_error=0.0)
    # Small workstations need collision sampling at least as fine as the final
    # validator. These overrides apply only to the MTC node's planning pipeline.
    for group in ('arm_left', 'arm_right'):
        ompl[group]['longest_valid_segment_fraction'] = 0.0004
    ompl['planner_configs']['RRTConnectConfig']['range'] = 0.2
    model = {'use_sim_time': True,
             # The included move_group launch resolves these first. All three
             # consumers use identical URDF/SRDF bytes, including mount contacts.
             'robot_description': ParameterValue(LaunchConfiguration('resolved_robot_description'), value_type=str),
             'robot_description_semantic': ParameterValue(LaunchConfiguration('resolved_robot_semantic'), value_type=str),
             'robot_description_kinematics': kinematics,
             'robot_description_planning': limits}
    return LaunchDescription([
        DeclareLaunchArgument('simulation_relaxed_base_motion', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_lidar', default_value='true', choices=['true', 'false']),
        *[DeclareLaunchArgument(name, default_value=str(camera_defaults[name]).lower(), choices=['true', 'false'])
          for name in camera_switches],
        *[DeclareLaunchArgument(name, default_value=os.path.normpath(os.path.join(os.path.dirname(preset_path), camera_defaults[name])))
          for name in camera_paths],
        DeclareLaunchArgument('allow_trajectory_execution', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('mtc_velocity_scaling', default_value='0.1'),
        DeclareLaunchArgument('mtc_acceleration_scaling', default_value='0.1'),
        DeclareLaunchArgument('mtc_joint_limit_margin_rad', default_value='0.1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(config, 'launch/move_group.launch.py')),
                                 launch_arguments={'use_sim_time': 'true',
                                     **{name: LaunchConfiguration(name) for name in ('use_lidar', *camera_switches, *camera_paths)},
                                     'allow_trajectory_execution': LaunchConfiguration('allow_trajectory_execution'),
                                     'extra_capabilities': PythonExpression(["'astribot_s1_manipulation/CancellableExecution' if '", LaunchConfiguration('allow_trajectory_execution'), "' == 'true' else ''"]),
                                     'disable_capabilities': 'move_group/MoveGroupExecuteTrajectoryAction'}.items()),
        Node(package='astribot_s1_manipulation', executable='transport_skill_planner', output='screen',
             parameters=[model]),
        Node(package='astribot_s1_transport_mtc', executable='mtc_planner', output='screen',
             parameters=[model, {'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl', 'ompl': ompl,
                 'trajectory_time_scaling': 2.5,
                 'max_velocity_scaling': ParameterValue(LaunchConfiguration('mtc_velocity_scaling'),value_type=float),
                 'max_acceleration_scaling': ParameterValue(LaunchConfiguration('mtc_acceleration_scaling'),value_type=float),
                 'joint_limit_margin_rad': ParameterValue(LaunchConfiguration('mtc_joint_limit_margin_rad'),value_type=float)}]),
        Node(package='astribot_s1_transport_mtc', executable='execution_guard', output='screen',
             parameters=[{'use_sim_time': True, 'simulation_relaxed_base_motion': ParameterValue(LaunchConfiguration('simulation_relaxed_base_motion'), value_type=bool)}])])
