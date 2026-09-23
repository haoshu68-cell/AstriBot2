"""Simulation template detector: orange box, aligned RGB-D, calibrated pinhole.

Not a general object recognizer. Returns observed surface bounds; it never reads
Gazebo object poses. Known dimensions and configured search region are priors.
"""
import cv2
import numpy as np
from .core import TaskFailure


def localize_orange_box(rgb, depth, intrinsics, optical_to_map, expected_size,
                        search_center=None, search_radius=.08):
    if rgb.shape[:2] != depth.shape or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise TaskFailure('RGB_DEPTH_NOT_ALIGNED')
    k = np.asarray(intrinsics, dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise TaskFailure('INVALID_CAMERA_INTRINSICS')
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.array([5, 100, 45]), np.array([30, 255, 255]))
    if search_center is not None:
        center = np.asarray(search_center, dtype=float)
        if center.shape != (3,) or not np.isfinite(center).all() or not 0 < search_radius <= .2:
            raise TaskFailure('INVALID_SEARCH_REGION')
        # Separate same-color background by measured position before image
        # connectivity: a floor stripe can touch the box in the RGB image.
        # The region covers surfaces of objects whose centers pass the original
        # center gate below; it does not replace or relax that gate.
        radius = search_radius + np.linalg.norm(np.asarray(expected_size)) / 2
        ys, xs = np.nonzero(mask)
        z = depth[ys, xs]
        valid = np.isfinite(z) & (z > .08) & (z < 5.)
        rays = np.column_stack(((xs-k[0, 2])/k[0, 0],
                                (ys-k[1, 2])/k[1, 1], np.ones(len(xs))))
        world_rays = (optical_to_map[:3, :3] @ rays.T).T
        origin = optical_to_map[:3, 3]
        keep = np.zeros(len(xs), dtype=bool)
        measured = origin + world_rays[valid] * z[valid, None]
        keep[valid] = np.linalg.norm(measured-center, axis=1) <= radius
        # Missing depth cannot be discarded to inflate quality. Keep invalid
        # color pixels whose forward camera rays intersect the search sphere;
        # project_component still counts them in the 80% validity denominator.
        delta = center-origin
        dot = world_rays @ delta
        ray_norm2 = np.sum(world_rays*world_rays, axis=1)
        intersects = (dot > 0) & (np.dot(delta, delta)-dot*dot/ray_norm2 <= radius*radius)
        keep[~valid] = intersects[~valid]
        mask[:] = 0
        mask[ys[keep], xs[keep]] = 255
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    candidates = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= 30]
    if search_center is not None:
        found = []
        for candidate in candidates:
            try:
                observed = project_component(labels == candidate, depth, k, optical_to_map, expected_size)
            except TaskFailure:
                continue
            if np.linalg.norm(np.asarray(observed['center_m']) - center) <= search_radius:
                found.append(observed)
        if len(found) != 1:
            raise TaskFailure('OBJECT_AMBIGUOUS_OR_NOT_VISIBLE_IN_REGION')
        return found[0]
    if len(candidates) != 1:
        raise TaskFailure('OBJECT_AMBIGUOUS_OR_NOT_VISIBLE')
    return project_component(labels == candidates[0], depth, k, optical_to_map, expected_size)


def project_component(component, depth, k, optical_to_map, expected_size):
    ys, xs = np.nonzero(component)
    z = depth[ys, xs]
    valid = np.isfinite(z) & (z > .08) & (z < 5.)
    if valid.sum() < 30 or valid.mean() < .8:
        raise TaskFailure('DEPTH_INVALID')
    x, y, z = xs[valid], ys[valid], z[valid]
    points = np.column_stack(((x-k[0, 2])*z/k[0, 0], (y-k[1, 2])*z/k[1, 1], z, np.ones(len(z))))
    world = (optical_to_map @ points.T).T[:, :3]
    low, high = np.percentile(world, [1, 99], axis=0)
    size = high-low
    expected = np.asarray(expected_size)
    if np.any(size > expected + .025) or np.any(size < expected * .25):
        raise TaskFailure('OBSERVED_OBJECT_GEOMETRY_MISMATCH')
    return dict(center_m=((low+high)/2).tolist(), observed_size_m=size.tolist(),
                points=int(valid.sum()), box_xyxy_px=[int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())],
                position_variance_m2=.0001, geometry_quality=float(valid.mean()))
