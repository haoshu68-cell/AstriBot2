"""Expand the actual robot to protect calibrated and provisional camera mounts.

These tests are offline: they do not start ROS nodes, Gazebo or renderers.
"""

import hashlib
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config"
REFERENCE_PROFILE = CONFIG / "camera_mounts_reference_sim.yaml"
CALIBRATION_SHA256 = "40ffd44fa5b103b2b03f2877fa9f4c4bd4511de2bd72d17263456a0b7acc1cd3"
CAMERA_IDS = (
    "head_rgbd", "torso_rgbd", "left_wrist_rgbd", "right_wrist_rgbd",
    "head_stereo_left", "head_stereo_right",
)
REFERENCE_PARENTS = {
    "head_rgbd": "astribot_head_link_2",
    "torso_rgbd": "astribot_torso_link_4",
    "left_wrist_rgbd": "astribot_arm_left_link_7",
    "right_wrist_rgbd": "astribot_arm_right_link_7",
    "head_stereo_left": "astribot_head_link_2",
    "head_stereo_right": "astribot_head_link_2",
}
# Recorded before introducing the reference profile. Reading only the current
# YAML would let an accidental calibration rewrite silently redefine the test.
CALIBRATED_MOUNTS = {
    "head_rgbd": ("astribot_head_link_2", "0.06034375 -0.21385876 -0.02767104",
                  "1.55787852 0.02188740 0.20119478"),
    "torso_rgbd": ("astribot_torso_base", "0.06775267 0.00133190 -0.08793599",
                   "-0.03077572 0.08157172 -0.01985774"),
    "left_wrist_rgbd": ("astribot_arm_left_link_7", "-0.01938667 -0.00412844 0.04767655",
                        "-0.24958987 -0.09270533 -1.60741310"),
    "right_wrist_rgbd": ("astribot_arm_right_link_7", "0.00798956 0.01107185 0.05206840",
                         "0.15186302 -0.06475795 -1.59568150"),
    "head_stereo_left": ("astribot_head_link_2", "0.05308852 -0.14422492 0.04584377",
                         "1.56209723 0.03048011 0.00483466"),
    "head_stereo_right": ("head_stereo_left_camera_optical_frame",
                          "0.04606753 0.00092537 0.06224035",
                          "-2.16352842 -1.53746069 -2.54764696"),
}


@pytest.fixture(scope="module", autouse=True)
def source_description_index(tmp_path_factory):
    """Resolve all xacro includes from this source, not an old installed copy."""
    prefix = tmp_path_factory.mktemp("camera_mount_source_index")
    resource = prefix / "share/ament_index/resource_index/packages"
    resource.mkdir(parents=True)
    (resource / "astribot_s1_description").touch()
    (prefix / "share/astribot_s1_description").symlink_to(PACKAGE, target_is_directory=True)
    previous = os.environ.get("AMENT_PREFIX_PATH")
    os.environ["AMENT_PREFIX_PATH"] = str(prefix) + (os.pathsep + previous if previous else "")
    try:
        yield
    finally:
        if previous is None:
            os.environ.pop("AMENT_PREFIX_PATH", None)
        else:
            os.environ["AMENT_PREFIX_PATH"] = previous


def _expand(profile=None, overrides=None):
    mappings = {
        "robot_name": "astribot_s1", "use_camera": "true",
        "use_wrist_cameras": "true", "use_stereo_cameras": "true",
        "use_lidar": "false",
    }
    if profile is not None:
        mappings["camera_mounts_profile"] = str(profile)
    mappings.update(overrides or {})
    return ET.fromstring(xacro.process_file(
        str(PACKAGE / "urdf/astribot_s1.xacro"), mappings=mappings).toxml())


@pytest.mark.parametrize("flags, enabled", [
    ({"use_camera": "false"}, set()),
    ({"use_wrist_cameras": "false", "use_stereo_cameras": "false"}, {"head_rgbd", "torso_rgbd"}),
    ({"use_wrist_cameras": "false"}, {"head_rgbd", "torso_rgbd", "head_stereo_left", "head_stereo_right"}),
])
def test_sensor_disable_preserves_physical_geometry_and_fixed_frames(models, flags, enabled):
    actual = _expand(REFERENCE_PROFILE, flags)
    reference = models["reference"]
    for camera in CAMERA_IDS:
        for tag, suffix in (("link", "camera_link"), ("joint", "camera_joint"),
                            ("link", "camera_optical_frame"), ("joint", "camera_optical_joint")):
            path = f"./{tag}[@name='{camera}_{suffix}']"
            got = actual.find(path)
            assert got is not None, "disabling capture removed physical geometry or TF"
            assert ET.tostring(got) == ET.tostring(reference.find(path))
    sensors = {s.get("name").removesuffix("_sensor") for s in actual.findall("./gazebo/sensor")}
    assert sensors == enabled


@pytest.fixture(scope="module")
def models():
    return {"default": _expand(), "calibrated": _expand(""),
            "reference": _expand(REFERENCE_PROFILE)}


def _vector(value, length=3):
    assert isinstance(value, str), f"expected a whitespace-separated string, got {value!r}"
    numbers = tuple(float(part) for part in value.split())
    assert len(numbers) == length
    assert all(math.isfinite(number) for number in numbers)
    return numbers


def _reference():
    assert REFERENCE_PROFILE.is_file(), "reference installation profile has not been created"
    return yaml.safe_load(REFERENCE_PROFILE.read_text())


def _joint(model, camera_id, optical=False):
    suffix = "camera_optical_joint" if optical else "camera_joint"
    joint = model.find(f"./joint[@name='{camera_id}_{suffix}']")
    assert joint is not None, f"missing {camera_id}_{suffix}"
    return joint


def _assert_mount(model, camera_id, expected):
    joint = _joint(model, camera_id)
    assert joint.get("type") == "fixed"
    assert joint.find("parent").get("link") == expected["parent_frame"]
    assert joint.find("child").get("link") == f"{camera_id}_camera_link"
    assert _vector(joint.find("origin").get("xyz")) == pytest.approx(
        _vector(expected["mount_xyz"]), abs=1e-10)
    assert _vector(joint.find("origin").get("rpy")) == pytest.approx(
        _vector(expected["mount_rpy"]), abs=1e-10)


def test_reference_profile_is_explicitly_provisional_and_complete():
    profile = _reference()
    assert profile["schema"] == "astribot.camera_mounts/1"
    assert profile["status"] == "provisional_reference"
    assert type(profile["revision"]) is int and profile["revision"] > 0
    assert set(profile["cameras"]) == set(CAMERA_IDS)
    for camera_id, mount in profile["cameras"].items():
        assert mount["parent_frame"] == REFERENCE_PARENTS[camera_id]
        for field in ("mount_xyz", "mount_rpy", "housing_size", "housing_center_xyz"):
            _vector(mount[field])
        assert all(size > 0 for size in _vector(mount["housing_size"]))


@pytest.mark.parametrize("camera_id", CAMERA_IDS)
def test_omitted_and_empty_overlay_preserve_calibrated_mounts(models, camera_id):
    calibrated = yaml.safe_load((CONFIG / f"camera_{camera_id}.yaml").read_text())
    parent, xyz, rpy = CALIBRATED_MOUNTS[camera_id]
    assert calibrated["parent_frame"] == parent
    assert _vector(calibrated["mount_xyz"]) == pytest.approx(_vector(xyz), abs=1e-10)
    assert _vector(calibrated["mount_rpy"]) == pytest.approx(_vector(rpy), abs=1e-10)
    for mode in ("default", "calibrated"):
        _assert_mount(models[mode], camera_id, calibrated)


@pytest.mark.parametrize("camera_id", CAMERA_IDS)
def test_reference_parent_and_origin_reach_expanded_urdf(models, camera_id):
    expected = _reference()["cameras"][camera_id]
    assert expected["parent_frame"] == REFERENCE_PARENTS[camera_id]
    _assert_mount(models["reference"], camera_id, expected)


def test_explicit_overlay_path_controls_mounts_and_housing(tmp_path):
    """Detect an ignored argument or a hardcoded reference profile."""
    mounts = {}
    for camera_id in CAMERA_IDS:
        mounts[camera_id] = {
            "parent_frame": "astribot_torso_link_4",
            "mount_xyz": "0.123 -0.047 0.089", "mount_rpy": "0.17 -0.23 0.31",
            "housing_size": "0.04 0.07 0.02", "housing_center_xyz": "-0.025 0 0",
        }
    profile = tmp_path / "custom_mounts.yaml"
    profile.write_text(yaml.safe_dump({"schema": "astribot.camera_mounts/1",
                                     "status": "provisional_reference", "revision": 1,
                                     "cameras": mounts}))
    model = _expand(profile)
    for camera_id in CAMERA_IDS:
        _assert_mount(model, camera_id, mounts[camera_id])
        link = model.find(f"./link[@name='{camera_id}_camera_link']")
        for geometry_kind in ("visual", "collision"):
            geometry = link.find(geometry_kind)
            assert _vector(geometry.find("geometry/box").get("size")) == pytest.approx(
                (0.04, 0.07, 0.02))
            assert _vector(geometry.find("origin").get("xyz")) == pytest.approx(
                (-0.025, 0.0, 0.0))


@pytest.mark.parametrize("mode", ("default", "calibrated", "reference"))
def test_optical_frames_keep_standard_camera_axis_rotation(models, mode):
    for camera_id in CAMERA_IDS:
        joint = _joint(models[mode], camera_id, optical=True)
        assert joint.find("parent").get("link") == f"{camera_id}_camera_link"
        assert joint.find("child").get("link") == f"{camera_id}_camera_optical_frame"
        assert _vector(joint.find("origin").get("xyz")) == (0.0, 0.0, 0.0)
        assert _vector(joint.find("origin").get("rpy")) == pytest.approx(
            (-math.pi / 2, 0.0, -math.pi / 2), abs=1e-12)
        sensor = models[mode].find(f"./gazebo/sensor[@name='{camera_id}_sensor']")
        assert sensor is not None
        assert _vector(sensor.findtext("pose"), length=6) == (0.0,) * 6


@pytest.mark.parametrize("mode", ("default", "calibrated", "reference"))
def test_camera_frames_joints_and_sensor_names_remain_unique(models, mode):
    model = models[mode]
    for xpath in ("./link", "./joint", "./gazebo/sensor"):
        names = [element.get("name") for element in model.findall(xpath)]
        assert len(names) == len(set(names)), f"duplicate name in {xpath}"
    for camera_id in CAMERA_IDS:
        assert model.find(f"./link[@name='{camera_id}_camera_link']") is not None
        assert model.find(f"./link[@name='{camera_id}_camera_optical_frame']") is not None
    sensor_names = {element.get("name") for element in model.findall("./gazebo/sensor")}
    assert {f"{camera_id}_sensor" for camera_id in CAMERA_IDS} <= sensor_names


@pytest.mark.parametrize("mode", ("default", "calibrated", "reference"))
def test_opaque_housing_and_collision_stay_behind_lens(models, mode):
    profile = _reference()["cameras"] if mode == "reference" else None
    for camera_id in CAMERA_IDS:
        link = models[mode].find(f"./link[@name='{camera_id}_camera_link']")
        for geometry_kind in ("visual", "collision"):
            geometry = link.find(geometry_kind)
            assert geometry is not None
            origin = geometry.find("origin")
            center = _vector(origin.get("xyz")) if origin is not None else (0.0,) * 3
            rotation = _vector(origin.get("rpy", "0 0 0")) if origin is not None else (0.0,) * 3
            size = _vector(geometry.find("geometry/box").get("size"))
            assert rotation == (0.0,) * 3
            assert all(length > 0 for length in size)
            assert center[0] < 0, f"{mode}/{camera_id}/{geometry_kind}: lens inside housing"
            assert center[0] + size[0] / 2 <= -0.001 + 1e-12
            if profile is not None:
                mount = profile[camera_id]
                prefix = "collision_" if geometry_kind == "collision" else ""
                assert center == pytest.approx(_vector(mount.get(prefix+"housing_center_xyz", mount["housing_center_xyz"])))
                assert size == pytest.approx(_vector(mount.get(prefix+"housing_size", mount["housing_size"])))
        rgba = link.find("visual/material/color").get("rgba")
        assert _vector(rgba, length=4)[3] == 1.0


def test_reference_stereo_has_symmetric_nonzero_baseline(models):
    model = models["reference"]
    left = _joint(model, "head_stereo_left")
    right = _joint(model, "head_stereo_right")
    assert left.find("parent").get("link") == "astribot_head_link_2"
    assert right.find("parent").get("link") == "astribot_head_link_2"
    a = _vector(left.find("origin").get("xyz"))
    b = _vector(right.find("origin").get("xyz"))
    # The existing head_link_2 mount frame has +z to the robot's left and
    # -y upwards; the stereo baseline therefore lies along local z.
    assert a[0] == pytest.approx(b[0], abs=1e-10)
    assert a[1] == pytest.approx(b[1], abs=1e-10)
    assert a[2] == pytest.approx(-b[2], abs=1e-10)
    assert a[2] > 0.0 and b[2] < 0.0
    assert a[2] - b[2] > 0.001, "stereo optical centers must not coincide"
    assert _vector(left.find("origin").get("rpy")) == pytest.approx(
        _vector(right.find("origin").get("rpy")), abs=1e-10)


def test_reference_expansion_preserves_original_robot_calibration(models):
    # Pin the existing calibration artifact, not a hash recomputed after migration.
    calibration = CONFIG / "camera_calibration_robot.json"
    assert hashlib.sha256(calibration.read_bytes()).hexdigest() == CALIBRATION_SHA256


def test_torso_collision_dedup_preserves_complete_robot_volume(models):
    """No collision volume is lost when avoiding duplicate parent/camera shapes.

    Prove box interval coverage from the expanded URDF (not YAML comments).
    Everything removed remains covered by the rigid parent in every pose.
    """
    model = models['reference']
    joint = _joint(model, 'torso_rgbd')
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').get('link') == 'astribot_torso_link_4'
    assert _vector(joint.find('origin').get('rpy')) == (0., 0., 0.)
    translation = _vector(joint.find('origin').get('xyz'))
    parent = model.find("./link[@name='astribot_torso_link_4']/collision")
    camera = model.find("./link[@name='torso_rgbd_camera_link']")
    def bounds(element, offset=(0., 0., 0.)):
        origin = element.find('origin')
        assert _vector(origin.get('rpy', '0 0 0')) == (0., 0., 0.)
        center = _vector(origin.get('xyz'))
        size = _vector(element.find('geometry/box').get('size'))
        return [(center[i]+offset[i]-size[i]/2, center[i]+offset[i]+size[i]/2)
                for i in range(3)]
    body = bounds(parent)
    full = bounds(camera.find('visual'), translation)
    kept = bounds(camera.find('collision'), translation)
    bracket = bounds(camera.find("visual[@name='torso_rgbd_mount_bracket']"), translation)
    assert camera.find("collision[@name='torso_rgbd_mount_bracket_collision']") is None
    for i in range(3):
        assert body[i][0] <= bracket[i][0] <= bracket[i][1] <= body[i][1]
    assert full[0][0] >= body[0][0]
    assert kept[0][0] == pytest.approx(body[0][1], abs=1e-12)
    assert kept[0][1] == pytest.approx(full[0][1], abs=1e-12)
    assert kept[0][0] > full[0][0]
    for i in (1, 2):
        assert kept[i] == pytest.approx(full[i], abs=1e-12)
        assert body[i][0] <= full[i][0] <= full[i][1] <= body[i][1]
