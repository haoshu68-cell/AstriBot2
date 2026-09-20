"""Task sensors and Gazebo services on the running navigation warehouse stack."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from astribot_logging.launch import Node


def generate_launch_description():
    share = get_package_share_directory('astribot_s1_transport')
    description = get_package_share_directory('astribot_s1_description')
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value=os.path.join(share, "config/warehouse_transfer.json")),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                '/model/transport_box_01/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',
                '/model/transport_box_01/kinematic_attachment/state@std_msgs/msg/String[ignition.msgs.StringMsg'],
             remappings=[('/model/transport_box_01/pose', '/simulation/transport_payload_pose')],
             parameters=[{'use_sim_time': True}]),
        Node(package='astribot_s1_transport', executable='camera_observer',
             parameters=[{'use_sim_time': True,
                'scenario': LaunchConfiguration('scenario'),
                'camera_profile': os.path.join(description, 'config/camera_rgbd_transport.yaml')}]),
    ])
