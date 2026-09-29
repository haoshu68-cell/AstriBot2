"""Task-requested C++ wrist subscriptions, health and private point clouds."""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction,
                            GroupAction,SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution,PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from astribot_logging.launch import Node

def session_node(context):
    from pathlib import Path
    profile=LaunchConfiguration('dds_profile').perform(context)
    environment={}
    if profile:
        if not Path(profile).is_file():
            raise ValueError('Explicit wrist DDS profile does not exist')
        environment={'RMW_IMPLEMENTATION':'rmw_fastrtps_cpp',
            'FASTRTPS_DEFAULT_PROFILES_FILE':str(Path(profile).resolve()),'ROS_LOCALHOST_ONLY':'0'}
    camera=LaunchConfiguration('camera_id')
    return [*[SetEnvironmentVariable(key,value) for key,value in environment.items()],
        Node(package='astribot_s1_perception_components',executable='wrist_camera_session_node',
        name=[camera,'_session'],parameters=[{
            'camera_id':camera,'use_sim_time':LaunchConfiguration('use_sim_time'),
            'queue_depth':ParameterValue(LaunchConfiguration('queue_depth'),value_type=int)}])]

def generate_launch_description():
    camera=LaunchConfiguration('camera_id');active=['/manipulation/camera/',camera,'/active']
    return LaunchDescription([
        DeclareLaunchArgument('camera_id',default_value='left_wrist_rgbd'),
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('calibration_revision',default_value='0'),
        DeclareLaunchArgument('expected_rate_hz',default_value='10.0'),
        DeclareLaunchArgument('queue_depth',default_value='4'),
        DeclareLaunchArgument('projection_backend',default_value='cpu',choices=['cpu','cuda']),
        DeclareLaunchArgument('dds_profile',default_value='',
            description='Explicit large-image Fast DDS transport; empty retains caller network settings'),
        DeclareLaunchArgument('frame_id',default_value=PythonExpression([
            "'astribot_s1/astribot_arm_left_link_7/left_wrist_rgbd_sensor' if '",camera,
            "' == 'left_wrist_rgbd' else 'astribot_s1/astribot_arm_right_link_7/right_wrist_rgbd_sensor'"])),
        # Apply the explicit transport to every wrist consumer, within this group.
        GroupAction(scoped=True,actions=[OpaqueFunction(function=session_node),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('astribot_s1_perception_components'),'launch','rgbd_vision_pipeline.launch.py'])),
            launch_arguments={
                'camera_id':camera,'use_sim_time':LaunchConfiguration('use_sim_time'),
                'calibration_revision':LaunchConfiguration('calibration_revision'),
                'frame_id':LaunchConfiguration('frame_id'),'expected_rate_hz':LaunchConfiguration('expected_rate_hz'),
                'activation_topic':['/perception/camera_session/',camera],
                'projection_backend':LaunchConfiguration('projection_backend'),
                'color_topic':[*active,'/image'],'depth_topic':[*active,'/depth_image'],
                'camera_info_topic':[*active,'/camera_info'],
                'enable_cloud':'true','enable_pose':'false','enable_detection_gate':'false',
            }.items())]),
    ])
