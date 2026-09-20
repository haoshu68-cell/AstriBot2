"""MoveIt plus validated skill planner; robot simulation is launched separately."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from astribot_logging.launch import Node


def generate_launch_description():
    config = get_package_share_directory('astribot_s1_moveit_config')
    description = get_package_share_directory('astribot_s1_description')
    camera_profile = os.path.join(description, 'config/camera_rgbd_transport.yaml')
    with open(os.path.join(config, 'config/astribot_s1.srdf')) as stream:
        semantic = stream.read()
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
             'robot_description': ParameterValue(Command(['xacro ', os.path.join(description, 'urdf/astribot_s1.xacro'),
                 ' robot_name:=astribot_s1 camera_profile:=', camera_profile]), value_type=str),
             'robot_description_semantic': semantic, 'robot_description_kinematics': kinematics,
             'robot_description_planning': limits}
    return LaunchDescription([
        DeclareLaunchArgument('mtc_velocity_scaling', default_value='0.1'),
        DeclareLaunchArgument('mtc_acceleration_scaling', default_value='0.1'),
        DeclareLaunchArgument('mtc_joint_limit_margin_rad', default_value='0.1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(config, 'launch/move_group.launch.py')),
                                 launch_arguments={'use_sim_time': 'true', 'camera_profile': camera_profile,
                                     'extra_capabilities': 'astribot_s1_manipulation/CancellableExecution',
                                     'disable_capabilities': 'move_group/MoveGroupExecuteTrajectoryAction'}.items()),
        Node(package='astribot_s1_manipulation', executable='transport_skill_planner', output='screen',
             parameters=[model]),
        Node(package='astribot_s1_transport_mtc', executable='mtc_planner', output='screen',
             parameters=[model, {'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl', 'ompl': ompl,
                 'max_velocity_scaling': ParameterValue(LaunchConfiguration('mtc_velocity_scaling'),value_type=float),
                 'max_acceleration_scaling': ParameterValue(LaunchConfiguration('mtc_acceleration_scaling'),value_type=float),
                 'joint_limit_margin_rad': ParameterValue(LaunchConfiguration('mtc_joint_limit_margin_rad'),value_type=float)}]),
        Node(package='astribot_s1_transport_mtc', executable='execution_guard', output='screen',
             parameters=[{'use_sim_time': True}])])
