from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from astribot_logging.launch import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("astribot_s1_mapping"), "config", "nav_prob_grid.yaml"]
    )

    node = Node(
        package="astribot_s1_mapping",
        executable="nav_prob_grid_node",
        name="nav_prob_grid_node",
        output="screen",
        parameters=[config_path],
    )

    return LaunchDescription([node])
