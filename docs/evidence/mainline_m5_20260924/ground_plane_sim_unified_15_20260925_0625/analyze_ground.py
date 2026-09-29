#!/usr/bin/env python3
"""Offline archive reconstruction and ground-plane candidates, never ground truth."""
import os
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("archive", type=Path)
parser.add_argument("output", type=Path)
args = parser.parse_args()
started = time.monotonic()
archive = args.archive.resolve()
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=True)
manifest = json.loads((archive / "manifest.json").read_text())
poses = np.loadtxt(archive / "alidarState.txt", ndmin=2)
if poses.shape[1] != 26 or not np.isfinite(poses).all():
    raise ValueError("Expected finite 26-column archived poses")
files = sorted((archive / "kf").glob("*.pcd"), key=lambda p: int(p.stem))
if len(files) != manifest["keyframes"] or len(poses) != manifest["scans"]:
    raise ValueError("Archive counts do not match manifest")
source_hashes = {}
for path in [archive / "alidarState.txt", *files]:
    relative = str(path.relative_to(archive))
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if manifest["sha256"][relative] != actual:
        raise ValueError(f"Manifest hash mismatch: {relative}")
    source_hashes[relative] = actual

clouds = []
owners = []
frames = []
expected_fields = "x y z intensity normal_x normal_y normal_z curvature"
for path in files:
    frame_id = int(path.stem)
    if frame_id < 0 or frame_id >= len(poses):
        raise ValueError(f"Invalid pose index: {frame_id}")
    with path.open("rb") as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"Incomplete PCD header: {path}")
            text = line.decode("ascii").strip()
            if not text or text.startswith("#"):
                continue
            key, value = text.split(maxsplit=1)
            header[key] = value
            if key == "DATA":
                break
        if (header["DATA"] != "binary" or header["FIELDS"] != expected_fields
                or header["SIZE"].split() != ["4"] * 8
                or header["TYPE"].split() != ["F"] * 8
                or header["COUNT"].split() != ["1"] * 8):
            raise ValueError(f"Unexpected PCD layout: {path}")
        points = np.frombuffer(stream.read(), dtype="<f4").reshape(-1, 8)
        if len(points) != int(header["POINTS"]):
            raise ValueError(f"PCD payload count mismatch: {path}")
    xyz = points[:, :3].astype(np.float64)
    if not np.isfinite(xyz).all():
        raise ValueError(f"Nonfinite point in {path}")
    pose = poses[frame_id]
    quat = pose[4:8]
    if abs(np.linalg.norm(quat) - 1.0) > 1e-5:
        raise ValueError(f"Non-unit archive quaternion: {frame_id}")
    rotation = Rotation.from_quat(quat).as_matrix()
    world = xyz @ rotation.T + pose[1:4]
    clouds.append(world)
    owners.append(np.full(len(world), frame_id, dtype=np.int32))
    frames.append({"keyframe_id": frame_id, "point_count": len(world),
                   "pose_stamp": float(pose[0]), "translation_m": pose[1:4].tolist()})
xyz = np.concatenate(clouds)
owner = np.concatenate(owners)
pcd_path = out / "merged_map_xyz.pcd"
pcd_header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
              "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
              f"WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {len(xyz)}\nDATA binary\n")
with pcd_path.open("wb") as stream:
    stream.write(pcd_header.encode("ascii"))
    stream.write(xyz.astype("<f4").tobytes())

# Search only the lower 40% of the observed height range by point quantile.
# This is a candidate-selection heuristic, not a certified floor classifier.
seed = 20260925
rng = np.random.default_rng(seed)
low_z_limit = float(np.quantile(xyz[:, 2], 0.40))
low = xyz[xyz[:, 2] <= low_z_limit]
sample = low[rng.choice(len(low), min(20000, len(low)), replace=False)]
threshold = 0.015  # perpendicular plane inlier distance, metres
max_tilt = 10.0
models = []
for iteration in range(1200):
    triple = sample[rng.choice(len(sample), 3, replace=False)]
    design = np.column_stack([triple[:, :2], np.ones(3)])
    if abs(np.linalg.det(design)) < 1e-9:
        continue
    abc = np.linalg.solve(design, triple[:, 2])
    tilt = np.degrees(np.arctan(np.linalg.norm(abc[:2])))
    if tilt > max_tilt:
        continue
    distance = np.abs(sample[:, 2] - sample[:, :2] @ abc[:2] - abc[2]) / np.sqrt(1 + abc[:2] @ abc[:2])
    count = int(np.count_nonzero(distance <= threshold))
    if count >= 100:
        models.append((count, abc))
models.sort(key=lambda item: item[0], reverse=True)
candidates = []
center = np.median(xyz[:, :2], axis=0)
all_xy_cells = np.unique(np.floor(xyz[:, :2] / 0.25).astype(np.int64), axis=0)
for sample_count, initial in models:
    height_at_center = float(center @ initial[:2] + initial[2])
    if any(abs(height_at_center - item["height_at_cloud_xy_median_m"]) < 0.04 for item in candidates):
        continue
    abc = initial
    converged = False
    for refinement in range(64):
        distance = np.abs(xyz[:, 2] - xyz[:, :2] @ abc[:2] - abc[2]) / np.sqrt(1 + abc[:2] @ abc[:2])
        mask = distance <= threshold
        support = xyz[mask]
        fit = np.column_stack([support[:, :2], np.ones(len(support))])
        updated = np.linalg.lstsq(fit, support[:, 2], rcond=None)[0]
        if np.max(np.abs(updated - abc)) < 1e-8:
            abc = updated
            converged = True
            break
        abc = updated
    distance = np.abs(xyz[:, 2] - xyz[:, :2] @ abc[:2] - abc[2]) / np.sqrt(1 + abc[:2] @ abc[:2])
    mask = distance <= threshold
    support = xyz[mask]
    residual = distance[mask]
    support_ids, counts = np.unique(owner[mask], return_counts=True)
    xy_min = support[:, :2].min(axis=0)
    xy_max = support[:, :2].max(axis=0)
    cells = np.unique(np.floor(support[:, :2] / 0.25).astype(np.int64), axis=0)
    hull_area = float(ConvexHull(support[:, :2]).volume)
    tilt = float(np.degrees(np.arctan(np.linalg.norm(abc[:2]))))
    broad = len(support) >= 500 and len(cells) >= 50 and np.min(xy_max - xy_min) >= 2.0 and hull_area >= 8.0 and tilt <= max_tilt
    candidates.append({
        "plane_equation": "z = a*x + b*y + c (map frame, metres)",
        "abc": abc.tolist(), "ground_z_at_map_origin_candidate_m": float(abc[2]),
        "height_at_cloud_xy_median_m": float(center @ abc[:2] + abc[2]),
        "tilt_deg": tilt, "normal_unit_up": (np.array([-abc[0], -abc[1], 1.0]) / np.sqrt(1 + abc[:2] @ abc[:2])).tolist(),
        "inlier_count": len(support), "inlier_fraction_all_points": float(len(support) / len(xyz)),
        "perpendicular_residual_m": {"rmse": float(np.sqrt(np.mean(residual ** 2))),
            "median": float(np.median(residual)), "p95": float(np.quantile(residual, 0.95)), "max": float(residual.max())},
        "xy_min_m": xy_min.tolist(), "xy_max_m": xy_max.tolist(), "xy_span_m": (xy_max - xy_min).tolist(),
        "xy_convex_hull_area_m2": hull_area,
        "xy_occupied_025m_cells": len(cells), "xy_cell_footprint_m2": float(len(cells) * 0.25 ** 2),
        "xy_cell_fraction_of_observed_cloud": float(len(cells) / len(all_xy_cells)),
        "support_keyframe_count": len(support_ids),
        "refinement_iterations": refinement + 1, "refinement_converged": converged,
        "support_per_keyframe": {str(int(i)): int(n) for i, n in zip(support_ids, counts)},
        "broad_horizontal_candidate": bool(broad), "initial_ransac_sample_inliers": sample_count,
    })
    if len(candidates) >= 6:
        break
candidates.sort(key=lambda item: item["height_at_cloud_xy_median_m"])
broad_candidates = [item for item in candidates if item["broad_horizontal_candidate"]]
selected = broad_candidates[0] if broad_candidates else None
if selected is not None:
    sensitivity = []
    for tolerance in [0.010, 0.015, 0.025, 0.040]:
        abc = np.array(selected["abc"])
        converged = False
        for refinement in range(64):
            residual = (xyz[:, 2] - xyz[:, :2] @ abc[:2] - abc[2]) / np.sqrt(1 + abc[:2] @ abc[:2])
            support = xyz[np.abs(residual) <= tolerance]
            updated = np.linalg.lstsq(np.column_stack([support[:, :2], np.ones(len(support))]), support[:, 2], rcond=None)[0]
            if np.max(np.abs(updated - abc)) < 1e-8:
                abc = updated
                converged = True
                break
            abc = updated
        residual = (xyz[:, 2] - xyz[:, :2] @ abc[:2] - abc[2]) / np.sqrt(1 + abc[:2] @ abc[:2])
        inlier_residual = residual[np.abs(residual) <= tolerance]
        sensitivity.append({"inlier_threshold_m": tolerance, "abc": abc.tolist(),
            "height_at_cloud_xy_median_m": float(center @ abc[:2] + abc[2]),
            "tilt_deg": float(np.degrees(np.arctan(np.linalg.norm(abc[:2])))),
            "inlier_count": len(inlier_residual),
            "refinement_iterations": refinement + 1, "refinement_converged": converged,
            "perpendicular_rmse_m": float(np.sqrt(np.mean(inlier_residual ** 2)))})
    selected["threshold_sensitivity"] = sensitivity
    abc = np.array(selected["abc"])
    signed = (xyz[:, 2] - xyz[:, :2] @ abc[:2] - abc[2]) / np.sqrt(1 + abc[:2] @ abc[:2])
    neighborhood = np.abs(signed) <= 0.05
    neighborhood_points = xyz[neighborhood]
    neighborhood_ids = owner[neighborhood]
    selected["within_5cm_neighborhood_count"] = len(neighborhood_points)
    selected["within_5cm_neighborhood_residual_p95_m"] = float(np.quantile(np.abs(signed[neighborhood]), .95))
    selected["within_5cm_per_frame_z_medians_m"] = {
        str(int(i)): float(np.median(neighborhood_points[neighborhood_ids == i, 2]))
        for i in np.unique(neighborhood_ids)}
report = {
    "evidence_level": "offline_pointcloud_fit_only", "ground_truth_available": False,
    "source_archive": str(archive), "source_manifest_sha256": hashlib.sha256((archive / "manifest.json").read_bytes()).hexdigest(),
    "source_hashes_verified": source_hashes, "frame_id": manifest["world_frame"],
    "transform": "kf/<id>.pcd point -> pose at zero-based alidarState row id; p_map = R(qx,qy,qz,qw) * p_local + t",
    "keyframe_count": len(files), "pose_count": len(poses), "merged_point_count": len(xyz),
    "xyz_min_m": xyz.min(axis=0).tolist(), "xyz_max_m": xyz.max(axis=0).tolist(),
    "z_quantiles_m": {str(q): float(np.quantile(xyz[:, 2], q)) for q in [0, .01, .05, .1, .25, .4, .5, .75, .9, .99, 1]},
    "cloud_xy_median_m": center.tolist(), "frames": frames,
    "method": {"random_seed": seed, "ransac_iterations": 1200, "sample_count": len(sample),
        "candidate_pool_upper_z_quantile": .4, "candidate_pool_upper_z_m": low_z_limit,
        "inlier_threshold_m": threshold, "max_tilt_deg": max_tilt, "ls_refinement_iterations_max": 64,
        "broadness_thresholds": {"min_points": 500, "min_025m_xy_cells": 50, "min_each_xy_span_m": 2.0, "min_xy_hull_area_m2": 8.0},
        "selection": "lowest broad horizontal candidate at cloud XY median"},
    "status": "candidate_found_not_ground_truth" if selected else "no_clear_ground_candidate",
    "selected_candidate": selected, "candidate_planes": candidates,
    "limitations": ["Candidate geometry only; no independently measured ground height or plane identity.",
        "Original acquisition, voxel averaging and SLAM errors remain; merged counts include repeated observations.",
        "No true per-ray origin is available; this analysis does not mark free space.",
        "XY hull spans gaps; occupied-cell footprint is sampled support, not verified traversable area.",
        "c is height at map x=y=0; nonzero tilt means one constant ground_z cannot describe the whole plane.",
        "Reported primary residuals are conditional on the 15 mm RANSAC inlier gate; threshold sensitivity and 50 mm neighborhood describe a wider band.",
        "Plane search is limited to lower 40% of observed points and <=10 degree tilt, not an exhaustive surface inventory."],
    "merged_cloud_path": str(pcd_path), "merged_cloud_sha256": hashlib.sha256(pcd_path.read_bytes()).hexdigest(),
    "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    "elapsed_seconds": time.monotonic() - started,
}
(out / "ground_analysis.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
print(json.dumps({"status": report["status"], "points": len(xyz), "selected": selected,
                  "candidate_count": len(candidates), "elapsed_seconds": report["elapsed_seconds"],
                  "report": str(out / "ground_analysis.json"), "cloud": str(pcd_path)}, indent=2))
