"""Two existing single-job servers admit the same fresh snapshot concurrently."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    config = yaml.safe_load(Path(LaunchConfiguration('config').perform(context)).read_text())
    common = dict(config['manipulation_perception_server']['ros__parameters'])
    for key in ('calibration_revision', 'planning_scene_revision', 'envelope_epoch'):
        value = int(LaunchConfiguration(key).perform(context))
        if value <= 0:
            raise ValueError(f'{key} must be an authoritative, nonzero task version')
        common[key] = value
    if not common.get('pose_worker') or not common.get('grasp_worker'):
        raise ValueError('Both real worker paths must be configured')
    pose = dict(common, grasp_worker='')
    grasp = dict(common, pose_worker='')
    return [
        Node(package='astribot_s1_manipulation_perception', executable='manipulation_perception_server',
             name='object_pose_server', parameters=[pose], output='screen',
             remappings=[('/perception/compute_grasps', '/object_pose_server/disabled_compute_grasps')]),
        Node(package='astribot_s1_manipulation_perception', executable='manipulation_perception_server',
             name='grasp_proposal_server', parameters=[grasp], output='screen',
             remappings=[('/perception/estimate_object_pose', '/grasp_proposal_server/disabled_estimate_object_pose')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config'),
        DeclareLaunchArgument('calibration_revision'),
        DeclareLaunchArgument('planning_scene_revision'),
        DeclareLaunchArgument('envelope_epoch'),
        OpaqueFunction(function=start),
    ])
