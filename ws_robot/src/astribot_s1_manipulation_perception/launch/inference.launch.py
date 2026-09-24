"""Runtime is C++; launch only supplies an explicitly configured deployment file."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config'),
        Node(package='astribot_s1_manipulation_perception',
             executable='manipulation_perception_server',
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
