"""Exercise the public dry-run entry; no ROS or Gazebo process is started."""
import json
from pathlib import Path
import subprocess
import sys

import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT/'tools/sim_stack_supervisor.py'
PRESET = ROOT/'ws_robot/src/astribot_s1_description/config/simulation_navigation_full/launch_preset.yaml'


def run(*args):
    return subprocess.run([sys.executable, str(SCRIPT), '--dry-run', *args],
                          capture_output=True, text=True)


def test_default_entry_uses_joint_six_camera_baseline():
    result = run()
    assert result.returncode == 0, result.stderr
    command = json.loads(result.stdout)['simulation']
    for name in ('use_camera', 'use_wrist_cameras', 'use_stereo_cameras', 'use_camera_pointcloud'):
        assert name+':=true' in command
    assert 'use_camera_postprocess:=false' in command
    assert 'camera_calibration_dir:='+str(PRESET.parent) in command
    assert 'camera_mounts_profile:='+str(PRESET.parent.parent/'camera_mounts_reference_sim.yaml') in command


def test_explicit_fixture_override_does_not_change_motion_defaults():
    result = run('--use-wrist-cameras', 'false')
    assert result.returncode == 0, result.stderr
    data = json.loads(result.stdout)
    assert 'use_wrist_cameras:=false' in data['simulation']
    assert 'use_stereo_cameras:=true' in data['simulation']
    assert 'max_linear_speed:=0.35' in data['navigation']


def test_missing_preset_is_reported_before_launch(tmp_path):
    result = run('--camera-preset', str(tmp_path/'absent.yaml'))
    assert result.returncode != 0
    assert 'camera preset' in result.stderr.lower()
    assert 'Traceback' not in result.stderr


def test_non_boolean_sensor_switch_is_rejected(tmp_path):
    data = yaml.safe_load(PRESET.read_text())
    data['parameters']['use_camera'] = 'false'
    path = tmp_path/'invalid.yaml'
    path.write_text(yaml.safe_dump(data))
    result = run('--camera-preset', str(path))
    assert result.returncode != 0
    assert 'boolean' in result.stderr.lower()


def test_public_launch_forwards_lidar_only_override():
    launch = ROOT/'ws_robot/src/astribot_s1_navigation/launch/sim_stack.launch.py'
    result = subprocess.run(['ros2', 'launch', str(launch), 'repo_dir:='+str(ROOT),
        'dry_run:=true', 'use_camera:=false', 'enable_depth_obstacles:=false'],
        capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'use_camera:=false' in result.stdout
    assert 'enable_depth_obstacles:=false' in result.stdout
