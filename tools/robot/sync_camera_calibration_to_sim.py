#!/usr/bin/env python3
"""Convert robot sensor_calib.json into Gazebo camera profiles.

This is a one-way tool: robot calibration file -> simulation profile. It never
writes to a robot. ``--camera all`` emits the six calibrated camera profiles.
"""
from __future__ import annotations
import argparse, hashlib, json, math
from pathlib import Path

# The URDF inserts this fixed camera_link -> camera_optical_frame rotation.
_OPTICAL_R = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))


def mmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transpose(a):
    return [list(row) for row in zip(*a)]


def rpy_from_matrix(m):
    # URDF roll-pitch-yaw convention, with a stable gimbal-lock branch.
    pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
    if abs(abs(pitch) - math.pi / 2) < 1e-8:
        roll = math.atan2(-m[0][1], m[1][1])
        yaw = 0.0
    else:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    return roll, pitch, yaw


CAMERAS = {
    'head_rgbd': ('camera_head_rgbd.yaml', 'astribot_head_link_2', 'rgbd_camera', 'head_rgbd'),
    'torso_rgbd': ('camera_torso_rgbd.yaml', 'astribot_torso_base', 'rgbd_camera', 'torso_rgbd'),
    'left_wrist_rgbd': ('camera_left_wrist_rgbd.yaml', 'astribot_arm_left_link_7', 'rgbd_camera', 'left_wrist_rgbd'),
    'right_wrist_rgbd': ('camera_right_wrist_rgbd.yaml', 'astribot_arm_right_link_7', 'rgbd_camera', 'right_wrist_rgbd'),
    'head_stereo_left': ('camera_head_stereo_left.yaml', 'astribot_head_link_2', 'camera', 'head_stereo_left'),
    'head_stereo_right': ('camera_head_stereo_right.yaml', 'head_stereo_left_camera_optical_frame', 'camera', 'head_stereo_right'),
}


def profile(source: Path, camera_name='head_rgbd', parent_frame=None):
    raw = source.read_bytes()
    data = json.loads(raw)
    camera = data['camera'][camera_name]
    resolution = camera.get('resolution', camera['intrinsics'].get('resolution'))
    width, height = (int(x) for x in resolution.split('x'))
    km = camera['intrinsics']['color']['matrix']
    fx, fy, cx, cy = float(km[0]), float(km[4]), float(km[2]), float(km[5])
    matrix = camera['extrinsics']['matrix']
    if len(matrix) != 16:
        raise ValueError('head_rgbd extrinsics must contain 16 values')
    parent_r = [matrix[0:3], matrix[4:7], matrix[8:11]]
    # source is parent -> optical; profile is parent -> camera_link
    mount_r = mmul(parent_r, transpose(_OPTICAL_R))
    roll, pitch, yaw = rpy_from_matrix(mount_r)
    xyz = [float(matrix[3]), float(matrix[7]), float(matrix[11])]
    distortion = camera.get('distortions', {}).get('matrix', [])
    if len(distortion) != 5:
        raise ValueError('head_rgbd distortion must contain 5 values')
    # The wrist firmware uses [1,1,1,1,1] as an unavailable-calibration
    # sentinel, not as physical Brown coefficients.
    distortion_unavailable = all(abs(float(v) - 1.0) < 1e-12 for v in distortion)
    if distortion_unavailable:
        distortion = [0.0] * 5
    fov = 2.0 * math.atan(width / (2.0 * fx))
    sha = hashlib.sha256(raw).hexdigest()
    depth = camera['intrinsics'].get('depth', {}).get('matrix', km)
    filename, default_parent, sensor_type, _ = CAMERAS[camera_name]
    parent_frame = parent_frame or default_parent
    return f'''# Robot-calibrated {camera_name} profile mirrored into Gazebo.
# Source: robot {source}
# SHA256: {sha}
# Gazebo consumes mount, image, FOV and clipping fields. Distortion is retained
# as provenance; the current Gazebo camera path does not render lens distortion.
model: robot_{camera_name}
calibration_status: robot_firmware_calibrated
calibration_source: /etc/config/sensor_calib.json
calibration_sha256: {sha}
sensor_type: {sensor_type}
parent_frame: {parent_frame}
child_frame: {camera_name}_camera_optical_frame
mount_xyz: '{xyz[0]:.8f} {xyz[1]:.8f} {xyz[2]:.8f}'
mount_rpy: '{roll:.8f} {pitch:.8f} {yaw:.8f}'
width: {width}
height: {height}
rate_hz: 15
horizontal_fov: {fov:.9f}
near_m: 0.08
far_m: 5.0
intrinsics:
  color: [{', '.join(f'{float(v):.9g}' for v in km)}]
  depth: [{', '.join(f'{float(v):.9g}' for v in depth)}]
distortion_model: plumb_bob
distortion: [{', '.join(f'{float(v):.9g}' for v in distortion)}]
distortion_status: {'unavailable_sentinel_zeroed' if distortion_unavailable else 'robot_firmware_calibrated'}
extrinsic_parent_frame: {camera['extrinsics'].get('parent_frame', camera_name)}
extrinsic_matrix_row_major: [{', '.join(f'{float(v):.9g}' for v in matrix)}]
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--camera', choices=[*CAMERAS, 'all'], default='head_rgbd')
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if args.camera == 'all':
        if not args.output_dir:
            parser.error('--output-dir is required with --camera all')
        args.output_dir.mkdir(parents=True, exist_ok=True)
        for name, (filename, parent, _, _) in CAMERAS.items():
            path = args.output_dir / filename
            path.write_text(profile(args.input, name, parent))
            print(f'wrote {path}')
    else:
        if not args.output:
            parser.error('--output is required unless --camera all')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(profile(args.input, args.camera, CAMERAS[args.camera][1]))
        print(f'wrote {args.output}')


if __name__ == '__main__':
    main()
