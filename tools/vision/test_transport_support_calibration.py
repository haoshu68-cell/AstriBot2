"""Launch-only camera provenance checks; no ROS nodes or simulator are started."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT/'ws_robot/src/astribot_s1_transport/launch/transport_support.launch.py'
PRESET = ROOT/'ws_robot/src/astribot_s1_description/config/simulation_navigation_full/launch_preset.yaml'


class TransportSupportCalibrationTest(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('support_calibration_test', SOURCE)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)

    def test_default_profiles_match_shared_navigation_preset(self):
        with patch.object(self.module, 'get_package_share_directory', side_effect=lambda p: str(ROOT/'ws_robot/src'/p)):
            launch = self.module.generate_launch_description()
        context = LaunchContext()
        for item in launch.entities:
            if isinstance(item, DeclareLaunchArgument):
                item.execute(context)
        expected = yaml.safe_load(PRESET.read_text())['parameters']
        for name in ('camera_profile', 'camera_mounts_profile'):
            self.assertEqual(Path(context.launch_configurations[name]).resolve(),
                             (PRESET.parent/expected[name]).resolve())
        self.assertEqual(context.launch_configurations['use_camera_postprocess'],
                         str(expected['use_camera_postprocess']).lower())

    def test_hash_tracks_intrinsics_and_mount_revision(self):
        with tempfile.TemporaryDirectory() as directory:
            profile, mounts = Path(directory)/'head.yaml', Path(directory)/'mounts.yaml'
            profile.write_text('model: fixture\nwidth: 640\n')
            mounts.write_text('revision: 1\ncameras: {head_rgbd: {mount_xyz: "0 0 0"}}\n')
            first = self.module.calibration_manifest(profile, mounts)
            self.assertEqual(first, self.module.calibration_manifest(profile, mounts))
            mounts.write_text('revision: 2\ncameras: {head_rgbd: {mount_xyz: "0.01 0 0"}}\n')
            second = self.module.calibration_manifest(profile, mounts)
            self.assertNotEqual(first, second)
            profile.write_text('model: fixture\nwidth: 320\n')
            third = self.module.calibration_manifest(profile, mounts)
            self.assertNotEqual(second, third)
            data = json.loads(third)
            self.assertEqual(data['camera_id'], 'head_rgbd')
            self.assertEqual(data['mounts']['revision'], 2)
            self.assertEqual(data['profile']['sha256'], hashlib.sha256(profile.read_bytes()).hexdigest())
            self.assertEqual(data['mounts']['sha256'], hashlib.sha256(mounts.read_bytes()).hexdigest())
            self.assertEqual(data['mounts']['source_yaml'], mounts.read_text())

    def test_missing_source_rejected_before_node_start(self):
        with self.assertRaises(FileNotFoundError):
            self.module.calibration_manifest(Path('/missing/camera.yaml'), Path('/missing/mounts.yaml'))

    def check_explicit_sources(self, postprocess):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            profile, mounts = folder/'head.yaml', folder/'mounts.yaml'
            profile.write_text('model: explicit_fixture\n')
            mounts.write_text('revision: 9\ncameras: {head_rgbd: {parent_frame: fixture_parent}}\n')
            context = LaunchContext()
            context.launch_configurations.update(camera_profile=str(profile), camera_mounts_profile=str(mounts),
                                                  scenario='/fixture/scenario.json',
                                                  use_camera_postprocess=str(postprocess).lower())
            observed = []
            with patch.object(self.module, 'get_log_directory', return_value=directory), \
                    patch.object(self.module, 'Node', side_effect=lambda **kw: observed.append(kw) or kw):
                self.module.launch_support(context)
            node = next(item for item in observed if item['executable'] == 'camera_observer')
            manifest = Path(node['parameters'][0]['camera_profile'])
            self.assertEqual(manifest.read_bytes(), self.module.calibration_manifest(profile, mounts, postprocess))
            self.assertEqual(json.loads(manifest.read_bytes())['mounts']['revision'], 9)
            self.assertEqual(node['parameters'][0]['scenario'], '/fixture/scenario.json')
            self.assertEqual(node['remappings'], [] if postprocess else [
                ('/camera/color/image_raw', '/camera/raw/head_rgbd/image'),
                ('/camera/depth/image_raw', '/camera/raw/head_rgbd/depth_image'),
                ('/camera/color/camera_info', '/camera/raw/head_rgbd/camera_info')])

    def test_raw_camera_sources_and_manifest(self):
        self.check_explicit_sources(False)

    def test_processed_camera_sources_and_manifest(self):
        self.check_explicit_sources(True)


if __name__ == '__main__':
    unittest.main()
