"""Opt-in contact recording for the isolated domain-213 warehouse validation."""
import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from astribot_logging.launch import Node


def generate_launch_description():
    if (os.environ.get('ROS_DOMAIN_ID') != '213' or
            os.environ.get('IGN_PARTITION') != 'astribot_operator_validation_213'):
        raise RuntimeError('Contact evidence requires the isolated simulation environment')
    plugin = get_package_prefix('astribot_s1_gazebo_bringup') + '/lib/libastribot_contact_evidence.so'
    from xml.sax.saxutils import quoteattr
    sdf = ('<sdf version="1.7"><model name="astribot_contact_evidence"><static>true</static>'
           '<plugin filename=' + quoteattr(plugin) + ' name="astribot::ContactEvidence"/>'
           '</model></sdf>')
    return LaunchDescription([
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'default', '-string', sdf, '-allow_renaming', 'false']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['/simulation/left_arm_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],
             parameters=[{'use_sim_time': True}]),
    ])
