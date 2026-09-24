#!/usr/bin/env python3
"""Offline, evaluation-only box ROI scoring; never publishes or supplies inference."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def rigid_matrix(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-6, rtol=0)):
        raise ValueError('Expected a finite rigid transform in metres')
    return matrix


def project_box(camera, camera_from_box, size):
    """Pixel rays use integer u/v and z=1, so slab parameter is optical Z."""
    v, u = np.indices((camera['height'], camera['width']))
    pixels = np.stack((u, v, np.ones_like(u)), axis=-1)
    k = np.asarray(camera['K'], dtype=float).reshape(3, 3)
    rays = pixels @ np.linalg.inv(k).T
    rotation = camera_from_box[:3, :3]
    origin = -rotation.T @ camera_from_box[:3, 3]
    directions = rays @ rotation
    half = size / 2
    if np.all(np.abs(origin) <= half):
        raise ValueError('Camera inside/on box is outside external-surface evaluation scope')
    # Parallel rays have an unbounded slab interval if inside that axis slab.
    parallel = np.abs(directions) < 1e-12
    low = np.full_like(directions, -np.inf)
    high = np.full_like(directions, np.inf)
    np.divide(-half-origin, directions, out=low, where=~parallel)
    np.divide(half-origin, directions, out=high, where=~parallel)
    near, far = np.minimum(low, high), np.maximum(low, high)
    outside_parallel = np.any(parallel & (np.abs(origin) > half), axis=-1)
    enter, leave = near.max(axis=-1), far.min(axis=-1)
    hit = (~outside_parallel) & (enter > 0) & (leave >= enter)
    axis = near.argmax(axis=-1)
    sign = np.take_along_axis(directions, axis[..., None], axis=-1)[..., 0] < 0
    faces = np.where(hit, axis*2 + sign, -1).astype(np.int8)
    return hit, np.where(hit, enter, np.nan), faces


def roi_metrics(mask, valid):
    pixels = int(mask.sum())
    count = int((mask & valid).sum())
    return {'pixels': pixels, 'valid_depth_pixels': count,
            'invalid_depth_pixels': pixels-count,
            'valid_depth_fraction': count/pixels if pixels else None}


def residual_statistics(values):
    result = {'samples': int(values.size), 'percentile_method': 'linear'}
    if values.size:
        result.update(min=float(values.min()), max=float(values.max()))
        for label, percentile in [('p05', 5), ('p50', 50), ('p95', 95)]:
            result[label] = float(np.percentile(values, percentile))
        result['absolute_p95'] = float(np.percentile(np.abs(values), 95))
    return result

def score_sample(sample, depth, visible_mask=None):
    if sample['schema'] != 'astribot.m5.box_roi/1' or sample['evaluation_only'] is not True:
        raise ValueError('Only the evaluation-only box ROI schema is accepted')
    identity = ('session_id', 'task_id', 'phase', 'object_id', 'camera_id', 'source_epoch', 'clock_epoch')
    if any(not isinstance(sample[key], str) or not sample[key] for key in identity):
        raise ValueError('Explicit sample identity and phase are required')
    if not isinstance(sample['calibration_revision'], int) or sample['calibration_revision'] <= 0:
        raise ValueError('Camera calibration revision must be recorded, not inferred')
    camera = sample['camera']
    stamps = camera['stamps_ns']
    stamp = stamps['depth']
    if stamp <= 0 or stamps['rgb'] != stamp or stamps['info'] != stamp:
        raise ValueError('Exact RGB/depth/info capture stamps are required')
    depth = np.asarray(depth)
    if (depth.shape != (camera['height'], camera['width']) or depth.ndim != 2
            or not np.issubdtype(depth.dtype, np.number) or not np.isrealobj(depth)):
        raise ValueError('Depth array does not match camera dimensions or numeric type')
    k = np.asarray(camera['K'], dtype=float).reshape(3, 3)
    distortion = np.asarray(camera['D'], dtype=float)
    if (camera['depth_convention'] != 'optical_z_m' or not camera['frame_id']
            or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1], atol=1e-12, rtol=0)
            or not np.isfinite(distortion).all() or np.any(np.abs(distortion) > 1e-9)):
        raise ValueError('Requires a zero-distortion pinhole camera and optical-Z metres')
    policy = sample['depth_policy']
    minimum, maximum = policy['min_m'], policy['max_m']
    if not (np.isfinite([minimum, maximum]).all() and 0 <= minimum < maximum and policy['source']):
        raise ValueError('Depth validity range requires explicit values and provenance')
    tolerance = sample.get('surface_tolerance_m')
    if tolerance is not None and (not np.isfinite(tolerance) or tolerance < 0
                                   or not sample.get('surface_tolerance_source')):
        raise ValueError('Surface tolerance requires a nonnegative value and explicit provenance')
    valid = np.isfinite(depth) & (depth > minimum) & (depth < maximum)
    report = {key: sample[key] for key in identity}
    report.update(schema='astribot.m5.box_roi.score/1', evaluation_only=True,
                  capture_stamp_ns=stamp, calibration_revision=sample['calibration_revision'],
                  depth_policy=policy, coverage_acceptance='UNKNOWN_THRESHOLD',
                  temporal_coverage='SINGLE_SNAPSHOT_ONLY',
                  full_box_surface_coverage='NOT_MEASURED',
                  geometry_status='NO_BOX_TRUTH',
                  surface_comparison={'status': 'NO_BOX_TRUTH'})
    maps = {'valid_depth': valid}
    truth = sample.get('box_truth')
    if truth is not None:
        transform = sample['world_from_camera']
        if (truth['object_id'] != sample['object_id'] or truth['capture_stamp_ns'] != stamp
                or truth['clock_epoch'] != sample['clock_epoch']
                or transform['clock_epoch'] != sample['clock_epoch']
                or truth['frame_id'] != transform['frame_id']
                or transform['capture_stamp_ns'] != stamp
                or transform['child_frame_id'] != camera['frame_id']
                or transform['calibration_revision'] != sample['calibration_revision']):
            raise ValueError('Truth/TF identity, frame, calibration or capture time mismatch')
        if truth['source'] not in ('gazebo_model_pose', 'manual_3d_annotation', 'synthetic_independent_fixture'):
            raise ValueError('Box truth must have an independent evaluated pose source')
        size = np.asarray(truth['size_m'], dtype=float)
        if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError('Box size must contain three positive metre dimensions')
        camera_from_box = np.linalg.inv(rigid_matrix(transform['matrix'])) @ rigid_matrix(truth['matrix'])
        roi, expected, faces = project_box(camera, camera_from_box, size)
        supported = roi & valid
        residual = np.where(supported, depth-expected, np.nan)
        report['geometry_status'] = 'PROJECTED' if roi.any() else 'NO_PIXEL_RAY_HIT'
        report['projected_roi'] = roi_metrics(roi, valid)
        report['projected_roi']['expected_in_depth_range_pixels'] = int(
            (roi & (expected > minimum) & (expected < maximum)).sum())
        comparison = {'status': 'UNKNOWN_TOLERANCE',
                      'residual_m': residual_statistics(residual[supported]),
                      'sign_convention': 'observed optical Z minus expected first box surface Z',
                      'interpretation': 'Closer depth is occlusion evidence conditional on correct pose/time/calibration; matching depth alone does not prove object identity'}
        face_names = ('x-', 'x+', 'y-', 'y+', 'z-', 'z+')
        report['faces'] = {name: roi_metrics(faces == index, valid)
                           for index, name in enumerate(face_names)}
        if tolerance is not None:
            classifications = {'consistent': supported & (np.abs(residual) <= tolerance),
                               'closer': supported & (residual < -tolerance),
                               'farther': supported & (residual > tolerance),
                               'unknown': roi & ~valid}
            comparison.update(status='MEASURED_WITH_EXPLICIT_TOLERANCE',
                              tolerance_m=tolerance, tolerance_source=sample['surface_tolerance_source'])
            total = int(roi.sum())
            for name, mask in classifications.items():
                count = int(mask.sum())
                comparison[name+'_pixels'] = count
                comparison[name+'_fraction_of_roi'] = count/total if total else None
                maps[name] = mask
                for index, face in enumerate(face_names):
                    report['faces'][face][name+'_pixels'] = int((mask & (faces == index)).sum())
        report['surface_comparison'] = comparison
        report['face_scope'] = 'First-hit box faces sampled by in-image pixel rays, not full 3D surface area coverage'
        maps.update(projected_roi=roi, expected_depth_m=expected, face_id=faces, residual_m=residual)
    annotation = sample.get('visible_annotation')
    if annotation is not None:
        if (visible_mask is None or annotation['object_id'] != sample['object_id']
                or annotation['capture_stamp_ns'] != stamp
                or annotation['clock_epoch'] != sample['clock_epoch']
                or annotation['frame_id'] != camera['frame_id'] or not annotation['annotation_id']
                or annotation['source'] not in ('manual', 'gazebo_instance_id', 'synthetic_fixture')):
            raise ValueError('Independent visible annotation is absent or incorrectly bound')
        mask = np.asarray(visible_mask)
        if mask.shape != depth.shape or not np.isin(mask, [0, 1, 255]).all():
            raise ValueError('Visible target mask must match depth and contain binary values')
        mask = mask.astype(bool)
        report['annotated_visible_roi'] = roi_metrics(mask, valid)
        report['annotated_visible_roi']['annotation'] = annotation
        maps['annotated_visible_roi'] = mask
        if truth is not None:
            report['annotated_visible_roi']['outside_projected_roi_pixels'] = int((mask & ~roi).sum())
            report['annotated_visible_roi']['projected_roi_unlabelled_pixels'] = int((roi & ~mask).sum())
            report['annotated_visible_roi']['scope'] = 'Unlabelled projection alone is not a measured occlusion cause'
    elif visible_mask is not None:
        raise ValueError('Mask has no independent annotation provenance')
    if truth is None and annotation is None:
        raise ValueError('An independent box pose or visible target annotation is required')
    return report, maps


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sample', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New directory; existing paths are refused')
    args = parser.parse_args()
    sample = json.loads(args.sample.read_text())
    files = {name: args.sample.parent/path for name, path in sample['files'].items()}
    # RGB is retained and hashed for independent review; numerical scoring uses depth.
    hashes = {name: hashlib.sha256(files[name].read_bytes()).hexdigest() for name in ('rgb', 'depth')}
    depth = np.load(files['depth'], allow_pickle=False)
    mask = None
    if 'visible_mask' in files:
        mask = np.load(files['visible_mask'], allow_pickle=False)
        hashes['visible_mask'] = hashlib.sha256(files['visible_mask'].read_bytes()).hexdigest()
    report, maps = score_sample(sample, depth, mask)
    report['input_sha256'] = hashes
    report['sample_sha256'] = hashlib.sha256(args.sample.read_bytes()).hexdigest()
    report['scorer_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    np.savez_compressed(args.output/'pixel_maps.npz', **maps)
    (args.output/'sample.json').write_text(json.dumps(sample, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'geometry_status': report['geometry_status'],
                      'coverage_acceptance': report['coverage_acceptance']}))


if __name__ == '__main__':
    main()
