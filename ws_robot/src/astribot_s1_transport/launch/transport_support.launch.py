"""Task sensors and Gazebo services on the running navigation warehouse stack."""
import os
import hashlib
import json
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from astribot_logging.launch import Node


def get_log_directory():
    return launch_config.log_dir


def calibration_manifest(profile, mounts, postprocess=False):
    """Bind the legacy observer's opaque epoch to actual intrinsics and mounts.

    The observer only hashes this file. Keep complete source text in the launch
    evidence so an epoch remains reviewable after source files are edited.
    """
    def source(path):
        path = Path(path).resolve(strict=True)
        raw = path.read_bytes()
        document = yaml.safe_load(raw)
        if not isinstance(document, dict):
            raise ValueError('Camera calibration source must be a mapping: ' + str(path))
        return {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest(),
                'source_yaml': raw.decode('utf-8')}, document

    profile_source, profile_data = source(profile)
    mounts_source, mounts_data = source(mounts)
    if not isinstance(mounts_data.get('cameras', {}).get('head_rgbd'), dict):
        raise ValueError('Camera mounts source must contain head_rgbd')
    profile_source['calibration_status'] = profile_data.get('calibration_status', 'unspecified')
    mounts_source.update(revision=mounts_data.get('revision'), status=mounts_data.get('status', 'unspecified'),
                         scope=mounts_data.get('scope', 'unspecified'))
    manifest = {'schema': 'astribot.transport_camera_manifest/1', 'camera_id': 'head_rgbd',
                'evidence_level': 'simulation_configuration_not_physical_calibration',
                'use_camera_postprocess': postprocess,
                'profile': profile_source, 'mounts': mounts_source}
    return (json.dumps(manifest, sort_keys=True, indent=2) + '\n').encode('utf-8')


def launch_support(context):
    mode = LaunchConfiguration('use_camera_postprocess').perform(context)
    if mode not in ('true', 'false'):
        raise ValueError('use_camera_postprocess must be true or false')
    postprocess = mode == 'true'
    manifest = calibration_manifest(LaunchConfiguration('camera_profile').perform(context),
                                    LaunchConfiguration('camera_mounts_profile').perform(context), postprocess)
    digest = hashlib.sha256(manifest).hexdigest()
    directory = Path(get_log_directory())
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / ('transport_camera_calibration_' + digest + '.json')
    if path.exists():
        if path.read_bytes() != manifest:
            raise ValueError('Camera calibration evidence conflict: ' + str(path))
    else:
        with path.open('xb') as stream:
            stream.write(manifest)
    return [
        LogInfo(msg='Transport camera calibration epoch=' + digest + ' manifest=' + str(path)),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                '/model/transport_box_01/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',
                '/model/transport_box_01/kinematic_attachment/state@std_msgs/msg/String[ignition.msgs.StringMsg'],
             remappings=[('/model/transport_box_01/pose', '/simulation/transport_payload_pose')],
             parameters=[{'use_sim_time': True}]),
        Node(package='astribot_s1_transport', executable='camera_observer',
             remappings=[] if postprocess else [
                 ('/camera/color/image_raw', '/camera/raw/head_rgbd/image'),
                 ('/camera/depth/image_raw', '/camera/raw/head_rgbd/depth_image'),
                 ('/camera/color/camera_info', '/camera/raw/head_rgbd/camera_info')],
             parameters=[{'use_sim_time': True,
                'scenario': LaunchConfiguration('scenario').perform(context),
                'camera_profile': str(path)}]),
    ]


def generate_launch_description():
    share = get_package_share_directory('astribot_s1_transport')
    description = get_package_share_directory('astribot_s1_description')
    preset_path = Path(description) / 'config/simulation_navigation_full/launch_preset.yaml'
    preset = yaml.safe_load(preset_path.read_text())
    if preset.get('schema_version') != 1:
        raise ValueError('Unsupported shared navigation camera preset')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value=os.path.join(share, 'config/warehouse_transfer.json')),
        *[DeclareLaunchArgument(name, default_value=str((preset_path.parent/preset['parameters'][name]).resolve()))
          for name in ('camera_profile', 'camera_mounts_profile')],
        DeclareLaunchArgument('use_camera_postprocess',
                              default_value=str(preset['parameters']['use_camera_postprocess']).lower(),
                              choices=['true', 'false']),
        OpaqueFunction(function=launch_support),
    ])
