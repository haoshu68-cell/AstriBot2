#!/usr/bin/env python3
"""Publish joint-derived navigation limits; this node never owns chassis commands."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from astribot_logging.launch import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('astribot_s1_dynamics_coupling')
    declarations = [
        DeclareLaunchArgument('output_topic', default_value='/navigation_policy/arm_speed_limit',
                              description='Dedicated upstream percentage constraint; zero means HOLD'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [pkg, 'config', 'arm_chassis_coupling_params.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]
    node = Node(
        package='astribot_s1_dynamics_coupling', executable='arm_chassis_speed_coupling_node',
        name='arm_chassis_speed_coupling_node', output='screen', respawn=True, respawn_delay=1.0,
        parameters=[LaunchConfiguration('params_file'), {
            'output_topic': LaunchConfiguration('output_topic'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }])
    return LaunchDescription(declarations + [node])
