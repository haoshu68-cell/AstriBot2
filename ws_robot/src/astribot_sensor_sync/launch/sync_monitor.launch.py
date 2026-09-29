"""Opt-in sidecar monitor; never reconfigures a physical sensor or starts pulses."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('allow_simulated', default_value='false'),
        DeclareLaunchArgument('reference_clock_epoch', default_value='UNCONFIGURED'),
        Node(package='astribot_sensor_sync', executable='sync_monitor', output='screen',
             parameters=[get_package_share_directory('astribot_sensor_sync') + '/config/sync_monitor.yaml',
                         {'reference_clock_epoch': LaunchConfiguration('reference_clock_epoch'),
                          'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                          'allow_simulated': ParameterValue(LaunchConfiguration('allow_simulated'), value_type=bool)}]),
    ])
