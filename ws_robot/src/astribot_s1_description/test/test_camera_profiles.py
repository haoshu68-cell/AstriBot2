from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


REQUIRED_PROFILES = (
    "camera_head_rgbd.yaml",
    "camera_head_rgbd_nav_sim.yaml",
    "camera_torso_rgbd.yaml",
    "camera_torso_rgbd_nav_sim.yaml",
)


def test_camera_profiles_are_installed_and_complete():
    """The xacro camera inputs must survive a clean package installation."""
    config_dir = Path(get_package_share_directory("astribot_s1_description")) / "config"
    for filename in REQUIRED_PROFILES:
        profile = config_dir / filename
        assert profile.is_file(), f"missing installed camera profile: {profile}"
        data = yaml.safe_load(profile.read_text())
        assert isinstance(data, dict), f"camera profile is not a mapping: {profile}"
        for key in ("sensor_type", "parent_frame", "child_frame", "mount_xyz", "mount_rpy"):
            assert data.get(key), f"camera profile {profile} has no {key}"
