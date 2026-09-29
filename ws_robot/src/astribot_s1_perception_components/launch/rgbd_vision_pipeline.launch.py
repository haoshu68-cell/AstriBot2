#!/usr/bin/env python3
"""One-camera C++ health -> optional YOLO -> detection gate -> position observation.

Use enable_health:=false to reuse health owned by the warehouse launch.
Model files are explicit local artifacts; this launch does not download weights.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from astribot_logging.launch import Node


def generate_launch_description():
    camera_id = LaunchConfiguration('camera_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_pose = LaunchConfiguration('enable_pose')
    enable_detection_gate = LaunchConfiguration('enable_detection_gate')
    return LaunchDescription([
        DeclareLaunchArgument('camera_id', default_value='head_rgbd'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_pose', default_value='true'),
        DeclareLaunchArgument('enable_health', default_value='true'),
        DeclareLaunchArgument('activation_topic', default_value=''),
        DeclareLaunchArgument('expected_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('enable_cloud', default_value='false'),
        DeclareLaunchArgument('projection_backend', default_value='cpu', choices=['cpu', 'cuda']),
        DeclareLaunchArgument('allow_cpu_fallback', default_value='false'),
        DeclareLaunchArgument('cloud_topic', default_value=['/manipulation/camera/', camera_id, '/points']),
        DeclareLaunchArgument('enable_yolo', default_value='false'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('labels_path', default_value=''),
        DeclareLaunchArgument('model_revision', default_value=''),
        DeclareLaunchArgument('model_layout', default_value='yolov5', choices=['yolov5', 'yolov8']),
        DeclareLaunchArgument('calibration_revision', default_value='1'),
        DeclareLaunchArgument('enable_detection_gate', default_value='true'),
        DeclareLaunchArgument('color_topic', default_value=PythonExpression(["'/camera/color/image_raw' if '", camera_id, "' == 'head_rgbd' else '/camera/", camera_id, "/color/image_raw'"])),
        DeclareLaunchArgument('depth_topic', default_value=PythonExpression(["'/camera/depth/image_raw' if '", camera_id, "' == 'head_rgbd' else '/camera/", camera_id, "/depth/image_raw'"])),
        DeclareLaunchArgument('camera_info_topic', default_value=PythonExpression(["'/camera/color/camera_info' if '", camera_id, "' == 'head_rgbd' else '/camera/", camera_id, "/color/camera_info'"])),
        DeclareLaunchArgument('detection_topic', default_value=['/perception/detections/', camera_id]),
        DeclareLaunchArgument(
            'valid_detection_topic', default_value=['/perception/valid_detections/', camera_id]),
        DeclareLaunchArgument('health_topic', default_value=['/perception/camera_health/', camera_id]),
        DeclareLaunchArgument('processing_health_topic', default_value=['/perception/projection_health/', camera_id]),
        DeclareLaunchArgument('output_topic', default_value=['/perception/object_pose/', camera_id]),
        DeclareLaunchArgument('frame_id', default_value=[camera_id, '_camera_optical_frame']),
        Node(
            package='astribot_s1_perception_components', executable='camera_health_node',
            name=[camera_id, '_camera_health'], output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_id': camera_id,
                'source_epoch': 'rgbd_pipeline',
                'activation_topic': LaunchConfiguration('activation_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'color_topic': LaunchConfiguration('color_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'info_topic': LaunchConfiguration('camera_info_topic'),
                'health_topic': LaunchConfiguration('health_topic'),
                'expected_rate_hz': ParameterValue(LaunchConfiguration('expected_rate_hz'), value_type=float),
                'max_age_sec': 0.25,
                'max_sync_skew_sec': 0.03,
                'calibration_revision': ParameterValue(LaunchConfiguration('calibration_revision'), value_type=int),
            }],
            condition=IfCondition(LaunchConfiguration('enable_health')),
        ),
        Node(
            package='astribot_s1_perception_components', executable='rgbd_pointcloud_node',
            name=[camera_id, '_manipulation_pointcloud'], output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'output_topic': LaunchConfiguration('cloud_topic'),
                'min_depth': 0.20, 'max_depth': 5.0, 'decimation': 4,
                'max_pair_age_sec': 0.25,
                'projection_backend': LaunchConfiguration('projection_backend'),
                'allow_cpu_fallback': ParameterValue(LaunchConfiguration('allow_cpu_fallback'), value_type=bool),
                'camera_id': camera_id,
                'processing_health_topic': LaunchConfiguration('processing_health_topic'),
                'activation_topic': LaunchConfiguration('activation_topic'),
            }],
            condition=IfCondition(LaunchConfiguration('enable_cloud')),
        ),
        Node(
            package='astribot_s1_perception_components', executable='yolo_detector_node',
            name=[camera_id, '_yolo_detector'], output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_id': camera_id,
                'color_topic': LaunchConfiguration('color_topic'),
                'health_topic': LaunchConfiguration('health_topic'),
                'output_topic': LaunchConfiguration('detection_topic'),
                **{key: ParameterValue(LaunchConfiguration(key), value_type=str)
                   for key in ('model_path', 'labels_path', 'model_revision', 'model_layout')},
            }],
            condition=IfCondition(LaunchConfiguration('enable_yolo')),
        ),
        Node(
            package='astribot_s1_perception_components', executable='detection_gate_node',
            name=[camera_id, '_detection_gate'], output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_id': camera_id,
                'input_topic': LaunchConfiguration('detection_topic'),
                'output_topic': LaunchConfiguration('valid_detection_topic'),
                'health_topic': LaunchConfiguration('health_topic'),
                'require_camera_health': True,
                'min_confidence': 0.25,
                'max_detection_age_sec': 0.30,
                'max_health_age_sec': 0.50,
            }],
            condition=IfCondition(enable_detection_gate),
        ),
        Node(
            package='astribot_s1_perception_components', executable='rgbd_object_pose_node',
            name=[camera_id, '_rgbd_object_pose'], output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'detection_topic': LaunchConfiguration('valid_detection_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'health_topic': LaunchConfiguration('health_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'require_camera_health': True,
                'max_detection_age_sec': 0.30,
                'max_depth_age_sec': 0.20,
                'max_health_age_sec': 0.50,
            }],
            condition=IfCondition(enable_pose),
        ),
    ])
