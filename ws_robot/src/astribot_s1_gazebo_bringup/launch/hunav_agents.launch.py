"""Optional HuNav services for the existing warehouse world; does not start Gazebo."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from astribot_logging.launch import Node


def start(context):
    from pathlib import Path
    from astribot_logging import log_directory
    from astribot_s1_social_navigation.scenario import load_scenario, prepare_behavior_trees
    scenario = LaunchConfiguration('scenario').perform(context)
    config = load_scenario(scenario)
    scene_dir = PathJoinSubstitution([FindPackageShare('astribot_s1_gazebo_bringup'), 'config', 'social']).perform(context)
    trees = prepare_behavior_trees(scenario,
                                  Path(scene_dir) / 'h1_agents__agent_1_bt.xml',
                                  Path(log_directory()) / 'human_behavior_trees')
    # Humble's YAML parameter parser cannot infer the type of an empty array.
    # The loader already declares a typed empty string array as its default.
    loader_parameters = scenario if config['agents'] else {
        key: config[key] for key in ('yaml_base_name', 'simulator', 'map', 'publish_people')}
    return [
        Node(package='astribot_s1_social_navigation', executable='social_scenario_provider', output='screen',
             parameters=[{'use_sim_time': True, 'scenario': LaunchConfiguration('scenario')}]),
        Node(package='hunav_agent_manager', executable='hunav_loader', output='screen',
             parameters=[loader_parameters, {'use_sim_time': True}]),
        Node(package='hunav_agent_manager', executable='hunav_agent_manager', output='screen',
             parameters=[{'use_sim_time': True, 'publish_tf': False, 'publish_sfm_forces': False,
                          'enable_groot': False, 'behavior_tree_directory': trees}])]


def generate_launch_description():
    scene_dir = PathJoinSubstitution([FindPackageShare('astribot_s1_gazebo_bringup'), 'config', 'social'])
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value=PathJoinSubstitution([scene_dir, 'h1_crossing.yaml'])),
        OpaqueFunction(function=start)])
