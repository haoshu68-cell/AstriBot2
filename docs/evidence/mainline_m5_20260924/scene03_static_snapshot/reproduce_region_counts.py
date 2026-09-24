#!/usr/bin/env python3
"""One-off offline count of scene03 measured points; no target truth input."""
import hashlib
import itertools
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

source = Path('/home/yjh/WorkSpace/astribot_validation/M1_scene_prepare_20260924_1024/first_scene03')
output = Path(__file__).resolve().parent / 'region_counts.json'
metadata = json.loads((source / 'head_snapshot/camera_info.json').read_text())
region = json.loads((source / 'pick_station_region.json').read_text())
scene_path = Path(region['evidence']['scene'])
assert hashlib.sha256(scene_path.read_bytes()).hexdigest() == region['evidence']['scene_sha256']
scene = json.loads(scene_path.read_text())
station = region['evidence']['station_binding']['station_object']
assert [item for item in scene['world']['collision_objects'] if item['id'] == station['id']] == [station]
assert not region['evidence']['target_object_pose_used']
assert not region['evidence']['target_object_dimensions_used']
assert not region['evidence']['pick_xyz_used']
tf = metadata['base_from_camera']
stamp = metadata['capture_stamp_ns']
assert metadata['exact_sync_stamps'] == dict(rgb=stamp, depth=stamp, info=stamp)
assert tf['stamp_ns'] == stamp and tf['child'] == metadata['frame']
assert tf['parent'] == region['station_frame'] == station['header']['frame_id']
assert np.all(np.asarray(metadata['D']) == 0)
rotation = Rotation.from_quat(tf['quaternion_xyzw']).as_matrix()
translation = np.asarray(tf['translation'])
station_matrix = np.asarray(region['evidence']['station_to_frame_matrix'])
station_pose = station['pose']
assert np.allclose(station_matrix[:3, :3], Rotation.from_quat(
    [station_pose['orientation'][k] for k in 'xyzw']).as_matrix(), atol=1e-12, rtol=0)
assert np.allclose(station_matrix[:3, 3], [station_pose['position'][k] for k in 'xyz'], atol=1e-12, rtol=0)
dimensions = np.asarray(station['primitives'][0]['dimensions'])
assert station['primitives'][0]['type'] == 1
assert region['evidence']['station_binding']['horizontal_expansion_m'] == 0
assert station['primitive_poses'] == [{'position': dict(x=0.0, y=0.0, z=0.0),
                                     'orientation': dict(x=0.0, y=0.0, z=0.0, w=1.0)}]
height = region['evidence']['station_binding']['height_above_table_m']
table_corners = np.asarray(list(itertools.product(
    [-dimensions[0]/2, dimensions[0]/2], [-dimensions[1]/2, dimensions[1]/2],
    [dimensions[2]/2, dimensions[2]/2+height])))
base_corners = table_corners @ station_matrix[:3, :3].T + station_matrix[:3, 3]
minimum, maximum = np.asarray(region['min_xyz']), np.asarray(region['max_xyz'])
assert np.allclose(base_corners.min(axis=0), minimum, atol=1e-12, rtol=0)
assert np.allclose(base_corners.max(axis=0), maximum, atol=1e-12, rtol=0)
aabb_corners = np.asarray(list(itertools.product(*zip(minimum, maximum))))
camera_corners = (aabb_corners-translation) @ rotation
depth = np.load(source / 'head_snapshot/depth.npy', allow_pickle=False)
bgr = cv2.imread(str(source / 'head_snapshot/rgb.png'))
assert depth.shape == (metadata['height'], metadata['width']) == bgr.shape[:2]
hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
orange = cv2.inRange(hsv, np.array([5, 100, 45]), np.array([30, 255, 255])) != 0
k_inverse = np.linalg.inv(np.asarray(metadata['K']).reshape(3, 3))
counts = {}
for stride in (1, 2):
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z > .08) & (z < 5.)
    rays = np.stack((u[valid], v[valid], np.ones(int(valid.sum()))), axis=-1) @ k_inverse.T
    base_points = (rays*z[valid, None]) @ rotation.T + translation
    inside = ((base_points >= minimum) & (base_points <= maximum)).all(axis=1)
    orange_valid = orange[::stride, ::stride][valid]
    counts[str(stride)] = dict(grid_pixels=int(z.size), valid_depth_points=int(valid.sum()),
        station_region_valid_points=int(inside.sum()),
        full_image_orange_valid_points=int(orange_valid.sum()),
        station_region_orange_valid_points=int((orange_valid & inside).sum()),
        region_has_at_least_2048_measured_points=bool(inside.sum() >= 2048))
valid_depth = depth[np.isfinite(depth) & (depth > .08) & (depth < 5.)]
paths = [source / 'head_snapshot' / name for name in ('rgb.png', 'depth.npy', 'camera_info.json')]
paths += [source / 'pick_station_region.json', scene_path, Path(__file__).resolve()]
report = dict(schema='astribot.m5.scene03_region_counts/1', capture_stamp_ns=stamp,
    source_directory=str(source), station_frame=region['station_frame'], region_revision=region['region_revision'],
    calibration_revision=metadata['calibration_revision'], source_epoch=metadata['source_epoch'],
    method='Measured optical-Z * inverse(K) * [u,v,1], capture-time base_from_camera, inclusive region AABB; finite and 0.08 < Z < 5 metres.',
    grid_origin_uv=[0, 0], counts_by_grid_stride=counts,
    region_optical_z_range_m=[float(camera_corners[:, 2].min()), float(camera_corners[:, 2].max())],
    full_image_valid_depth_range_m=[float(valid_depth.min()), float(valid_depth.max())],
    optical_forward_in_base=rotation[:, 2].tolist(), region_provenance_checks='PASS',
    region_scene_stamp=station['header']['stamp'],
    region_time_scope='Captured static scene region; zero scene stamp is not capture-time or moving-scene certification.',
    stride_semantics='Offline stride 1 and stride 2 comparisons; not proof of live projector configuration.',
    orange_semantics='Raw project HSV range only, no component cleaning or target identity; background pixels cannot count as target points.',
    target_pose_used=False, target_dimensions_used=False, truth_used_for_point_selection=False,
    duplicated_or_padded_points=0, inference_run=False, motion_acceptance='NOT_EVALUATED',
    source_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths})
output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
print(json.dumps({'counts': counts, 'region_optical_z_range_m': report['region_optical_z_range_m']}))
