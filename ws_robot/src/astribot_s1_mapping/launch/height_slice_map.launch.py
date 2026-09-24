from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    required = ["session_directory", "output_directory", "frame_id", "ground_z", "ground_reference"]
    config = PathJoinSubstitution([FindPackageShare("astribot_s1_mapping"), "config", "height_slices.yaml"])
    return LaunchDescription([
        *[DeclareLaunchArgument(name) for name in required],
        Node(package="astribot_s1_mapping", executable="height_slice_map_node", output="screen",
             parameters=[config, {name: ParameterValue(LaunchConfiguration(name),
                                      value_type=float if name == "ground_z" else str)
                                  for name in required}]),
    ])
