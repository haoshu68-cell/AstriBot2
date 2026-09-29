#!/usr/bin/env python3
"""Offline ROS Humble launch contract checks; does not start any ROS node."""
import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[2]


def nodes(relative, executables=None, **overrides):
    spec = importlib.util.spec_from_file_location('vision_launch', ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    if hasattr(module, '_prepare_camera_mounts'):
        for action in module._prepare_camera_mounts(context):
            action.execute(context)
    result = []
    for action in description.entities:
        if isinstance(action, Node) and action.node_executable in (executables or {
            'camera_health_node', 'yolo_detector_node', 'detection_gate_node', 'rgbd_object_pose_node'
        }):
            # Humble normalized parameter dictionaries; never execute the Node action.
            parameters = {}
            for entry in evaluate_parameters(context, action._Node__parameters):
                if isinstance(entry, dict):
                    parameters.update(entry)
            result.append((action.node_executable, parameters, action.condition is None or action.condition.evaluate(context)))
    return result


def main():
    standalone = 'ws_robot/src/astribot_s1_perception_components/launch/rgbd_vision_pipeline.launch.py'
    warehouse = 'ws_robot/src/astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py'
    for camera in ('left_wrist_rgbd', 'right_wrist_rgbd'):
        selected = nodes(standalone, executables={'camera_health_node', 'rgbd_pointcloud_node'},
                         camera_id=camera, expected_rate_hz='5.0', enable_cloud='true')
        health = next(p for exe, p, active in selected if exe == 'camera_health_node' and active)
        cloud = next(p for exe, p, active in selected if exe == 'rgbd_pointcloud_node' and active)
        assert health['expected_rate_hz'] == 5.0
        assert cloud['output_topic'] == '/manipulation/camera/' + camera + '/points'
        assert cloud['max_pair_age_sec'] == .25
    print('PASS wrist role rate and private optional pointcloud outputs')
    for lidar, camera, wrist, stereo, expected in (
        ('false', 'false', 'false', 'false', 0),
        ('true', 'false', 'true', 'true', 2),
        ('false', 'true', 'false', 'false', 2),
        ('false', 'true', 'true', 'false', 4),
        ('true', 'true', 'true', 'true', 8),
    ):
        aliases = nodes(warehouse, executables={'static_transform_publisher'},
                        use_lidar=lidar, use_camera=camera, use_wrist_cameras=wrist, use_stereo_cameras=stereo)
        assert sum(active for _, _, active in aliases) == expected
    print('PASS sensor aliases follow five attachment selections; no disconnected disabled sensors')
    for camera in ('head_rgbd', 'torso_rgbd'):
        evaluated = nodes(standalone, camera_id=camera)
        health = next(p for exe, p, _ in evaluated if exe == 'camera_health_node')
        assert health['health_topic'] == '/perception/camera_health/' + camera
        assert health['frame_id'] == camera + '_camera_optical_frame'
        if camera == 'torso_rgbd':
            assert health['color_topic'] == '/camera/torso_rgbd/color/image_raw'
        assert not next(active for exe, _, active in evaluated if exe == 'yolo_detector_node')
        print('PASS standalone camera topics and default detector disabled:', camera)
    for postprocess in ('true', 'false'):
        evaluated = nodes(warehouse, use_camera='true', use_camera_postprocess=postprocess,
                          enable_yolo_detector='true', yolo_camera_id='torso_rgbd', enable_rgbd_pose_estimator='true')
        detectors = [p for exe, p, active in evaluated if exe == 'yolo_detector_node' and active]
        assert len(detectors) == 1 and detectors[0]['camera_id'] == 'torso_rgbd'
        for camera in ('head_rgbd', 'torso_rgbd'):
            health = next(p for exe, p, _ in evaluated if exe == 'camera_health_node' and p['camera_id'] == camera)
            pose = next(p for exe, p, _ in evaluated if exe == 'rgbd_object_pose_node' and p['health_topic'].endswith('/'+camera))
            assert pose['depth_topic'] == health['depth_topic']
            assert pose['camera_info_topic'] == health['info_topic']
            if postprocess == 'false':
                assert health['color_topic'] == '/camera/raw/' + camera + '/image'
                assert health['depth_topic'] == '/camera/raw/' + camera + '/depth_image'
                assert health['frame_id'].endswith('/' + camera + '_sensor')
            else:
                assert health['frame_id'] == camera + '_camera_optical_frame'
            assert health['calibration_revision'] == 2026092101
            if postprocess == 'false' and camera == 'torso_rgbd':
                assert health['frame_id'] == 'astribot_s1/astribot_torso_link_4/torso_rgbd_sensor'
        torso = next(p for exe, p, _ in evaluated if exe == 'camera_health_node' and p['camera_id'] == 'torso_rgbd')
        assert detectors[0]['color_topic'] == torso['color_topic']
        print('PASS warehouse selected detector and matched depth/info/frame, postprocess:', postprocess)
    assert not any(active for _, _, active in nodes(warehouse, use_camera='false', enable_yolo_detector='true', enable_rgbd_pose_estimator='true'))
    print('PASS camera-disabled warehouse starts no vision consumers')
    fallback = nodes(warehouse, use_camera='true', use_camera_postprocess='false', camera_mounts_profile='')
    for exe, health, _ in fallback:
        if exe == 'camera_health_node':
            assert health['calibration_revision'] == 1
            if health['camera_id'] == 'torso_rgbd':
                assert health['frame_id'] == 'astribot_s1/astribot_torso_base/torso_rgbd_sensor'
    print('PASS empty mount profile restores historical native frame and revision')
    spec = importlib.util.spec_from_file_location('bridge_launch', ROOT / warehouse)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for local, explicit in [('true', ''), ('false', ''), ('false', '/tmp/user-lan.xml')]:
        context = LaunchContext()
        context.launch_configurations.update(localhost_only=local, camera_bridge_dds_profile=explicit)
        env = module._camera_bridge_environment(context)
        if explicit:
            assert env['FASTRTPS_DEFAULT_PROFILES_FILE'] == explicit
        elif local == 'false':
            assert env == {}, 'LAN must inherit the caller network/RMW environment'
        else:
            import xml.etree.ElementTree as ET
            document = ET.parse(env['FASTRTPS_DEFAULT_PROFILES_FILE'])
            ns = {'d': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
            assert document.find('.//d:segment_size', ns).text == '16777216'
            assert document.find('.//d:useBuiltinTransports', ns).text == 'false'
            assert document.find('.//d:interfaceWhiteList/d:address', ns).text == '127.0.0.1'
            assert env['ROS_LOCALHOST_ONLY'] == '0'
        assert len(module._camera_bridge_node(context, arguments=[], remappings=[])) == 1
        print('PASS bridge transport selection:', local, explicit or 'automatic')


if __name__ == '__main__':
    main()
