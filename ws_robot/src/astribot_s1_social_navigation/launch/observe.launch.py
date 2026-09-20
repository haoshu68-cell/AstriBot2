from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from astribot_logging.launch import Node


def start(context):
    source = LaunchConfiguration('source').perform(context)
    simulation = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    if source not in ('perception', 'hunav_truth'):
        raise ValueError('source must be perception or hunav_truth')
    truth = source == 'hunav_truth'
    if truth and not simulation:
        raise ValueError('hunav_truth requires use_sim_time:=true')
    nodes = []
    if truth:
        nodes.append(Node(package='astribot_s1_social_navigation', executable='hunav_truth_adapter',
                          parameters=[{'use_sim_time': True,
                                       'input_topic': LaunchConfiguration('hunav_input_topic').perform(context)}], output='screen'))
    nodes.append(Node(package='astribot_s1_social_navigation', executable='social_observer', output='screen',
                      parameters=[{'use_sim_time': simulation, 'allow_simulation_truth': truth,
                                   'input_topic': '/simulation/social_agents_truth' if truth else '/perception/social_agents',
                                   'target_frame': LaunchConfiguration('target_frame').perform(context)}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='perception'),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('hunav_input_topic', default_value='/human_states'),
        OpaqueFunction(function=start)])
