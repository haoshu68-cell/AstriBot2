"""Process evidence must distinguish camera filters from the main slice node."""
import pytest

from run_regression import is_primary_slice_process


@pytest.mark.parametrize('args, expected', [
    ([], True),
    (['--ros-args', '-r', '__node:=pointcloud_slice_scan_node'], True),
    (['--ros-args', '-r', '__node:=head_rgbd_self_filter'], False),
    (['--ros-args', '--remap', '__node:=torso_rgbd_self_filter'], False),
    (['--ros-args', '--remap=__node:=head_rgbd_self_filter'], False),
    (['--ros-args', '--remap=__node:=pointcloud_slice_scan_node'], False),
    (['--ros-args', '-r', '__ns:=/camera'], False),
    (['--ros-args', '-r', '__ns:=/'], True),
    (['--ros-args', '-p', 'label:=__node:=head_rgbd_self_filter'], True),
    (['--ros-args', '--', '-r', '__node:=head_rgbd_self_filter'], False),
    (['__node:=head_rgbd_self_filter'], False),
    (['--ros-args', '-r', 'other:__node:=camera'], True),
    (['--ros-args', '-r', 'pointcloud_slice_scan_node:__node:=pointcloud_slice_scan_node'], True),
    (['--ros-args', '-r', 'pointcloud_slice_scan_node:__node:=camera'], False),
    (['--ros-args', '-r', 'other:__ns:=/camera'], True),
    (['--ros-args', '-r', 'pointcloud_slice_scan_node:__ns:=/camera'], False),
    (['--ros-args', '-r', '__node:=pointcloud_slice_scan_node', '-r', '__node:=camera'], True),
    (['--ros-args', '-r', '__ns:=/', '-r', '__ns:=/camera'], True),
    (['--ros-args', '-r'], False),
    (['--ros-args', '-r', '__name:=head_rgbd_self_filter'], False),
    (['--ros-args', '-r', '__node:=camera', '--ros-args', '-r',
      '__node:=pointcloud_slice_scan_node'], False),
])
def test_primary_slice_identity(args, expected):
    command = ['/isolated/install/lib/pointcloud_slice_scan_node'] + args
    assert is_primary_slice_process([item.encode() for item in command]) is expected


def test_other_program_cannot_supply_slice_identity():
    assert not is_primary_slice_process([b'/usr/bin/python3', b'pointcloud_slice_scan_node'])


def test_owned_camera_filters_do_not_hide_duplicate_main_processes():
    main = [b'/install/pointcloud_slice_scan_node']
    camera = main + [b'--ros-args', b'-r', b'__node:=head_rgbd_self_filter']
    assert sum(is_primary_slice_process(c) for c in [main, camera]) == 1
    assert sum(is_primary_slice_process(c) for c in [main, camera, main]) == 2
