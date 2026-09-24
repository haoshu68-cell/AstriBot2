#!/usr/bin/env python3
"""One-off offline count of ready_scene01 measured points; no target truth input."""
import hashlib
import itertools
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

source = Path('/home/yjh/WorkSpace/astribot_validation/READY_profile_20260924_1133/ready_scene01')
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
report = dict(schema='astribot.m5.ready_scene01_region_counts/1', capture_stamp_ns=stamp,
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

# Mirror only the current C++ geometry/segmentation gates, not live admission.
v, u = np.indices(depth.shape)
k = np.asarray(metadata['K']).reshape(3, 3)
z = depth.astype(np.float32)
z64 = z.astype(np.float64)
cloud = np.full((*depth.shape, 3), np.nan, dtype=np.float32)
projected = np.isfinite(z) & (z64 >= .08) & (z64 <= 5.)
cloud[..., 0][projected] = ((u[projected]-k[0, 2])*z64[projected]/k[0, 0]).astype(np.float32)
cloud[..., 1][projected] = ((v[projected]-k[1, 2])*z64[projected]/k[1, 1]).astype(np.float32)
cloud[..., 2][projected] = z[projected]
valid = np.isfinite(cloud).all(axis=-1) & (cloud[..., 2] > np.float32(.08)) & (cloud[..., 2] < np.float32(5.))
valid_orange = orange & valid
measured = cloud[valid_orange].astype(np.float64) @ rotation.T + translation
inside = ((measured >= minimum) & (measured <= maximum)).all(axis=1)
mask = np.zeros(depth.shape, dtype=np.uint8)
mask[valid_orange] = inside
p = cloud[valid_orange].astype(np.float64)
pixel_error = np.maximum(np.abs(k[0, 0]*p[:, 0]/p[:, 2]+k[0, 2]-u[valid_orange]),
                         np.abs(k[1, 1]*p[:, 1]/p[:, 2]+k[1, 2]-v[valid_orange]))
assert np.all(pixel_error < .25)
invalid_orange = orange & ~valid
rays = np.stack(((u[invalid_orange]-k[0, 2])/k[0, 0],
                 (v[invalid_orange]-k[1, 2])/k[1, 1], np.ones(int(invalid_orange.sum()))), axis=-1) @ rotation.T
entry, leave = np.full(len(rays), .08), np.full(len(rays), 5.)
intersects = np.ones(len(rays), dtype=bool)
for axis in range(3):
    parallel = np.abs(rays[:, axis]) < 1e-12
    intersects[parallel] &= minimum[axis] <= translation[axis] <= maximum[axis]
    moving = ~parallel
    first = (minimum[axis]-translation[axis])/rays[moving, axis]
    second = (maximum[axis]-translation[axis])/rays[moving, axis]
    entry[moving] = np.maximum(entry[moving], np.minimum(first, second))
    leave[moving] = np.minimum(leave[moving], np.maximum(first, second))
mask[invalid_orange] = intersects & (entry <= leave)
component_count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
candidates = [i for i in range(1, component_count) if stats[i, cv2.CC_STAT_AREA] >= 30]
assert len(candidates) == 1, f'SINGLE_COLOR_COMPONENT_REQUIRED: {candidates}'
selected = labels == candidates[0]
selected_valid = selected & valid
points = cloud[selected_valid]
assert np.all(np.abs(points[:, :2]) <= 10)
area, count = int(selected.sum()), int(selected_valid.sum())
grid2 = (u % 2 == 0) & (v % 2 == 0)
core_paths = [Path('ws_robot/src/astribot_s1_manipulation_perception/src/single_box_request.cpp'),
              Path('ws_robot/src/astribot_s1_manipulation_perception/config/single_box_projection.yaml'),
              Path('ws_robot/src/astribot_s1_perception_components/src/depth_projection.cpp')]
gate = dict(schema='astribot.m5.ready_scene01_request_point_gate/1', capture_stamp_ns=stamp,
    station_frame=region['station_frame'], region_revision=region['region_revision'],
    camera_info_path=str(source/'head_snapshot/camera_info.json'), calibration_revision=metadata['calibration_revision'],
    source_epoch=metadata['source_epoch'], actual_K=metadata['K'], capture_station_from_camera=tf,
    cloud_semantics='Offline reconstruction of CPU decimation1 FLOAT32 XYZ from actual raw depth and CameraInfo; no live PointCloud2 or ProjectionHealth was captured by this tool.',
    candidate_components_total=component_count-1, candidate_components_area_at_least_30=len(candidates),
    region_orange_mask_pixels=int(mask.sum()), invalid_depth_ray_pixels_retained=int((mask.astype(bool)&~valid).sum()),
    selected_component_bbox_xywh=[int(x) for x in stats[candidates[0], :4]],
    selected_component_area_pixels=area, selected_component_measured_points=count,
    selected_component_invalid_depth_pixels=area-count, selected_component_valid_fraction=count/area,
    selected_component_points_after_12000_cap=min(count, 12000),
    selected_component_grid2_subset_area=int((selected & grid2).sum()),
    selected_component_grid2_subset_points=int((selected_valid & grid2).sum()),
    grid2_scope='Counterfactual subset of full-resolution selected mask, not an accepted M3 projector layout.',
    pixel_registration_max_error_px=float(pixel_error.max()),
    selected_points_min_xyz_m=points.min(axis=0).tolist(), selected_points_max_xyz_m=points.max(axis=0).tolist(),
    minimum_2048_points_pass=count >= 2048, minimum_valid_fraction_0_8_pass=count/area >= .8,
    point_cardinality_and_validity_result='PASS_OFFLINE_SINGLE_FRAME' if count >= 2048 and count/area >= .8 else 'FAIL',
    request_live_admission='NOT_EVALUATED', recorded_capture_age_sec=metadata['capture_age_at_end_sec'],
    freshness_scope='Age is the historical capture receipt; frozen input is not fresh and must not be admitted as a current request.',
    missing_live_evidence=['Actual dedicated PointCloud2 layout', 'ProjectionHealth processing epoch and current validity',
                           'Current single-instance fixture identity/window and region context', 'Receipt steady deadlines'],
    target_entity_pose_used=False, target_entity_dimensions_used=False, duplicated_or_padded_points=0,
    inference_run=False, zero_start_motion_validated=False, motion_acceptance='NOT_EVALUATED',
    source_sha256=report['source_sha256'] | {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest() for path in core_paths})
(output.parent/'request_point_gate.json').write_text(json.dumps(gate, indent=2, allow_nan=False)+'\n')
np.savez_compressed(output.parent/'selected_measured_pixels.npz', selected_mask=selected,
                    valid_mask=selected_valid, pixels_uv=np.stack((u[selected_valid], v[selected_valid]), axis=-1),
                    points_camera_m=points)
print(json.dumps({key: gate[key] for key in ('candidate_components_area_at_least_30',
    'selected_component_area_pixels', 'selected_component_measured_points',
    'selected_component_valid_fraction', 'point_cardinality_and_validity_result')}))
