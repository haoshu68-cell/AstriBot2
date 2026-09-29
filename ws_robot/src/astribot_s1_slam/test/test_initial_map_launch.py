"""Launch plumbing only; no ROS nodes, simulation, or motion are started."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchContext


LAUNCH = Path(__file__).resolve().parents[2] / 'astribot_s1_perception' / 'launch'


def load(name):
    spec = importlib.util.spec_from_file_location(name, LAUNCH / f'{name}.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(**overrides):
    ctx = LaunchContext()
    ctx.launch_configurations.update({
        'env': 'sim', 'slam_backend': 'voxel', 'mode': 'mapping',
        'launch_slam': 'true', 'social_scenario': '', 'initial_chassis_pose': '',
        'save_path': '/tmp/unused_slam_launch_test', 'map_name': '',
        'previous_map': '', 'save_map': '0', 'use_sim_time': 'true',
        'lidar_topic': '/livox/lidar_left', 'lidar_topic_back': '/livox/lidar_right',
        'imu_topic': '/livox/imu', 'publish_grid': 'false', 'point_notime': '1',
        'imu_extrinsic_tran': '0,0,0', 'back_extrinsic_tran': '0,0,0',
        'back_extrinsic_rota': '1,0,0,0,1,0,0,0,1', **overrides,
    })
    return ctx


def overrides_from_build(ctx):
    module = load('voxel_slam')
    with patch.object(module, 'Node') as node, patch.object(module, 'RegisterEventHandler'), \
            patch.object(module, 'OnProcessExit'):
        module._build_voxel(ctx)
        return node.call_args.kwargs['parameters'][-1]


def test_static_map_requires_measured_registration_and_local_mapping_is_unchanged():
    module = load('perception_slam_bringup')
    assert module._validate(context()) == []
    with pytest.raises(RuntimeError, match='实测 initial_chassis_pose'):
        module._validate(context(slam_backend='static_map'))
    assert module._validate(context(slam_backend='static_map',launch_slam='false')) == []
    assert module._validate(context(slam_backend='static_map',
                                  initial_chassis_pose='.25,.15,.1292,0,0,0,1')) == []
    assert 'General.initial_chassis_pose' not in overrides_from_build(context())


def test_measured_pose_reaches_estimator_without_spawn_substitution():
    values = overrides_from_build(context(initial_chassis_pose='.25,.15,.1292,0,0,0,1',
                                          spawn_x='8', spawn_y='9', spawn_z='.15'))
    assert values['General.initial_chassis_pose'] == [.25,.15,.1292,0,0,0,1]


@pytest.mark.parametrize('pose,previous', [
    ('0,0,0,0,0,0,1', 'saved_map:0.5'), ('0,0,0,0', ''),
    ('nan,0,0,0,0,0,1', ''), ('0,0,0,0,0,0,2', ''),
])
def test_invalid_registration_fails_at_launch_boundary(pose, previous):
    with pytest.raises(RuntimeError):
        overrides_from_build(context(initial_chassis_pose=pose,previous_map=previous))
