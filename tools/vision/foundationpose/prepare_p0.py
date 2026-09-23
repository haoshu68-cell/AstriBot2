#!/usr/bin/env python3
"""Prepare immutable, CPU-only FoundationPose P0 assets; never run inference."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import fcntl
import hashlib
import io
import itertools
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import time
import urllib.parse

import numpy as np
from PIL import Image


REPO = Path(__file__).resolve().parents[3]
COMMITS = {
    "isaac_ros_common": "fcf4d9e17f8f0a7f47f1d22d6a18421ce3768c01",
    "isaac_ros_pose_estimation": "9caca619bcc9d637b3107e17c1a77132c9d7863b",
}
NGC_VERSION_URL = "https://api.ngc.nvidia.com/v2/models/nvidia/isaac/foundationpose/versions/1.0.0_onnx"
NGC_LICENSE_URL = "https://developer.download.nvidia.com/licenses/tao_toolkit_21-08_models_eula.pdf"
# Observed from NGC's exact-version files API on 2026-09-23, not invented hashes.
MODELS = {
    "refine_model.onnx": (68170161, "06ad19f2c3598cb76733feec084d3f6802e7ff143882ec42ba368df7e38ae094"),
    "score_model.onnx": (63910294, "49a4f5f094358913670733ec31e856b96271c869f9949aa3a0361cf7cf8f0be8"),
}
SCENARIOS = ("clear", "close", "far", "near", "occluded", "tilted", "yawed")


def json_bytes(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def file_record(path):
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"not a regular non-symlink file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return {"path": str(path.resolve()), "bytes": path.stat().st_size, "sha256": digest.hexdigest()}


def write_immutable(path, content):
    """Publish atomically without overwriting; identical content is a no-op."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_symlink():
        raise ValueError(f"immutable destination is a symlink: {path}")
    if path.exists():
        if path.read_bytes() != content:
            raise ValueError(f"existing immutable file differs; choose a new output: {path}")
        return
    fd, temporary = tempfile.mkstemp(prefix="." + path.name + ".", suffix=".part", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, path)
        except FileExistsError:
            if path.is_symlink() or path.read_bytes() != content:
                raise ValueError(f"concurrent immutable output differs: {path}")
    finally:
        Path(temporary).unlink(missing_ok=True)


def download_atomic(url, destination, *, expected_sha256=None, expected_bytes=None,
                    timeout_s=120, allow_network=True):
    """Bound curl wall time, verify before publish, and preserve first provenance."""
    destination = Path(destination)
    parsed = urllib.parse.urlparse(url)
    if parsed.scheme != "https" and not (parsed.scheme == "http" and parsed.hostname in ("127.0.0.1", "localhost")):
        raise ValueError("downloads require HTTPS (loopback HTTP is for offline tests)")
    receipt_path = destination.with_name(destination.name + ".receipt.json")
    receipt = json.loads(receipt_path.read_text()) if receipt_path.exists() else None
    if receipt:
        if receipt["source_url"] != url:
            raise ValueError(f"cached source URL changed: {destination}")
        if expected_sha256 and receipt["sha256"] != expected_sha256:
            raise ValueError(f"cached expected SHA256 changed: {destination}")
        expected_sha256 = expected_sha256 or receipt["sha256"]
        expected_bytes = expected_bytes if expected_bytes is not None else receipt["bytes"]

    def verify(path):
        record = file_record(path)
        if expected_bytes is not None and record["bytes"] != expected_bytes:
            raise ValueError(f"download size mismatch for {destination.name}: {record['bytes']} != {expected_bytes}")
        if expected_sha256 and record["sha256"] != expected_sha256:
            raise ValueError(f"download SHA256 mismatch for {destination.name}")
        if not record["bytes"]:
            raise ValueError("empty download")
        return record

    if destination.exists() or destination.is_symlink():
        record = verify(destination)
        if not receipt:
            raise ValueError(f"existing cache has no provenance receipt: {destination}")
        return dict(receipt, path=str(destination.resolve()))
    if not allow_network:
        raise RuntimeError(f"offline cache miss: {destination.name}")
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise RuntimeError("total network time budget exhausted")
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix="." + destination.name + ".", suffix=".part", dir=destination.parent)
    os.close(fd)
    try:
        command = ["curl", "--fail", "--location", "--silent", "--show-error", "--retry", "0",
                   "--connect-timeout", str(min(15, timeout_s)), "--max-time", str(timeout_s),
                   "--output", temporary, url]
        try:
            completed = subprocess.run(command, capture_output=True, text=True, timeout=timeout_s + 5)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"download process exceeded total timeout: {destination.name}") from exc
        if completed.returncode:
            # curl's diagnostic can include signed redirect URLs: retain only the exit code.
            raise RuntimeError(f"download failed: curl exit {completed.returncode}, artifact {destination.name}")
        record = verify(temporary)
        new_receipt = receipt or {
            "source_url": url, "sha256": record["sha256"], "bytes": record["bytes"],
            "retrieved_at_utc": datetime.now(timezone.utc).isoformat(),
            "publisher_hash_verified": expected_sha256 is not None,
            "hash_scope": "downloaded bytes; publisher signature was not cryptographically verified",
        }
        # Receipt first: an interrupted publish can retry against the pinned receipt hash.
        write_immutable(receipt_path, json_bytes(new_receipt))
        with open(temporary, "rb") as stream:
            os.fsync(stream.fileno())
        try:
            os.link(temporary, destination)
        except FileExistsError:
            verify(destination)
        return dict(new_receipt, path=str(destination.resolve()))
    finally:
        Path(temporary).unlink(missing_ok=True)


def make_box_union(geometry):
    """Exact axis-aligned arrangement; internal cells never produce surface faces."""
    if geometry.get("units") != "m":
        raise ValueError("CAD units must be m; no implicit rescaling")
    if geometry.get("schema") != "astribot.box_union/1" or not geometry.get("object_frame"):
        raise ValueError("unsupported CAD schema or missing business object frame")
    boxes = []
    for box in geometry.get("boxes", []):
        center = np.asarray(box["center_m"], dtype=float)
        size = np.asarray(box["size_m"], dtype=float)
        if center.shape != (3,) or size.shape != (3,) or not np.isfinite(center).all() or not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError("invalid CAD box dimensions")
        # Decimal metre inputs are normalized at 1 pm to avoid spurious sliver cells.
        boxes.append((np.round(center - size / 2, 12), np.round(center + size / 2, 12)))
    if not boxes or len(boxes) > 32:
        raise ValueError("CAD requires between 1 and 32 boxes")
    axes = [sorted({float(corner[a]) for bounds in boxes for corner in bounds}) for a in range(3)]
    occupied = set()
    volume = 0.0
    for cell in itertools.product(*(range(len(axis) - 1) for axis in axes)):
        center = np.array([(axes[a][cell[a]] + axes[a][cell[a] + 1]) / 2 for a in range(3)])
        if any(np.all(center > low) and np.all(center < high) for low, high in boxes):
            occupied.add(cell)
            volume += math.prod(axes[a][cell[a] + 1] - axes[a][cell[a]] for a in range(3))
    if not occupied:
        raise ValueError("empty CAD union")
    pending = [next(iter(occupied))]
    reached = set(pending)
    while pending:
        cell = pending.pop()
        for axis in range(3):
            for direction in (-1, 1):
                other = list(cell); other[axis] += direction; other = tuple(other)
                if other in occupied and other not in reached:
                    reached.add(other); pending.append(other)
    if reached != occupied:
        raise ValueError("CAD solid is not face-connected; possible nonmanifold contact")
    vertices, faces, indices = [], [], {}
    for cell in sorted(occupied):
        for axis in range(3):
            u, v = (axis + 1) % 3, (axis + 2) % 3
            for direction in (-1, 1):
                neighbor = list(cell); neighbor[axis] += direction
                if tuple(neighbor) in occupied:
                    continue
                corners = []
                for du, dv in [(0, 0), (1, 0), (1, 1), (0, 1)]:
                    point = [axes[a][cell[a]] for a in range(3)]
                    point[axis] = axes[axis][cell[axis] + (direction == 1)]
                    point[u], point[v] = axes[u][cell[u] + du], axes[v][cell[v] + dv]
                    key = tuple(point)
                    if key not in indices:
                        indices[key] = len(vertices); vertices.append(key)
                    corners.append(indices[key])
                if direction == -1:
                    corners = [corners[0], corners[3], corners[2], corners[1]]
                faces.extend([(corners[0], corners[1], corners[2]), (corners[0], corners[2], corners[3])])
    edge_count, oriented, links = Counter(), Counter(), defaultdict(list)
    signed_volume = 0.0
    for a, b, c in faces:
        av, bv, cv = (np.array(vertices[i]) for i in (a, b, c))
        if np.linalg.norm(np.cross(bv - av, cv - av)) < 1e-15:
            raise ValueError("degenerate CAD triangle")
        signed_volume += float(np.dot(av, np.cross(bv, cv))) / 6
        for x, y in [(a, b), (b, c), (c, a)]:
            edge_count[tuple(sorted((x, y)))] += 1; oriented[(x, y)] += 1
        for vertex, edge in [(a, (b, c)), (b, (c, a)), (c, (a, b))]:
            links[vertex].append(edge)
    boundary = sum(count == 1 for count in edge_count.values())
    nonmanifold = sum(count != 2 for count in edge_count.values())
    if nonmanifold or any(oriented[(b, a)] != n for (a, b), n in oriented.items()):
        raise ValueError("CAD boundary is not a closed consistently oriented manifold")
    for edges in links.values():
        adjacency = defaultdict(set)
        for a, b in edges:
            adjacency[a].add(b); adjacency[b].add(a)
        visited, queue = set(), [next(iter(adjacency))]
        while queue:
            a = queue.pop()
            if a not in visited:
                visited.add(a); queue.extend(adjacency[a] - visited)
        if len(visited) != len(adjacency) or any(len(n) != 2 for n in adjacency.values()):
            raise ValueError("CAD has a nonmanifold vertex link")
    if signed_volume <= 0 or not math.isclose(signed_volume, volume, abs_tol=1e-12, rel_tol=1e-9):
        raise ValueError("CAD signed volume does not equal exact union volume")
    bounds = [np.min(vertices, axis=0).tolist(), np.max(vertices, axis=0).tolist()]
    center = ((np.array(bounds[0]) + bounds[1]) / 2).tolist()
    return vertices, faces, {
        "status": "PASS", "method": "axis-aligned arrangement exposed faces; no approximate Boolean mesh",
        "units": "m", "object_frame": geometry["object_frame"], "vertices": len(vertices), "triangles": len(faces),
        "boundary_edges": boundary, "nonmanifold_edges": nonmanifold,
        "vertex_links": "one closed cycle per vertex", "solid_components": 1,
        "euler_characteristic": len(vertices) - len(edge_count) + len(faces),
        "signed_volume_m3": signed_volume, "exact_union_volume_m3": volume,
        "bounds_m": bounds, "dimensions_m": (np.array(bounds[1]) - bounds[0]).tolist(),
        "mesh_center_in_object_m": center, "coordinate_rounding_m": 1e-12,
    }


def prepare_cad(source, output):
    geometry = json.loads(source.read_text())
    vertices, faces, report = make_box_union(geometry)
    folder = output / "cad"; center = np.array(report["mesh_center_in_object_m"])
    write_immutable(folder / "source_geometry.json", source.read_bytes())
    for name, shift in [("asymmetric_union.obj", np.zeros(3)), ("centered.obj", center)]:
        lines = ["# metres; business frame transform in registry.json", "mtllib material.mtl", "usemtl magenta_fixture"]
        lines += ["v " + " ".join(f"{n:.12g}" for n in (np.array(vertex) - shift)) for vertex in vertices]
        lines += ["vt 0 0", "vt 1 0", "vt 0 1"]
        lines += ["f " + " ".join(f"{vertex + 1}/{i + 1}" for i, vertex in enumerate(face)) for face in faces]
        write_immutable(folder / name, ("\n".join(lines) + "\n").encode())
    write_immutable(folder / "material.mtl", b"newmtl magenta_fixture\nKa 1 0 1\nKd 1 0 1\nKs 0 0 0\nd 1\nillum 1\nmap_Kd albedo.png\n")
    buffer = io.BytesIO(); Image.new("RGB", (2, 2), (255, 0, 255)).save(buffer, format="PNG")
    write_immutable(folder / "albedo.png", buffer.getvalue())
    mesh_from_object = np.eye(4); mesh_from_object[:3, 3] = -center
    object_from_mesh = np.eye(4); object_from_mesh[:3, 3] = center
    registry = {
        "schema": "astribot.foundationpose.cad/1", "model_id": "asymmetric_union_v1", "units": "m",
        "object_frame": geometry["object_frame"], "mesh": "centered.obj", "business_mesh": "asymmetric_union.obj",
        "collision_mesh": "asymmetric_union.obj", "texture": "albedo.png", "symmetry": "none",
        "T_mesh_object": mesh_from_object.tolist(), "T_object_mesh": object_from_mesh.tolist(),
        "transform_convention": "T_A_B maps B-frame points into A; T_camera_object=T_camera_mesh*T_mesh_object",
        "material_scope": "uniform synthetic magenta fixture; not a measured real-object texture",
        "source": file_record(source), "frozen_source": file_record(folder / "source_geometry.json"), "geometry_validation": report,
        "assets": {name: file_record(folder / name) for name in ["asymmetric_union.obj", "centered.obj", "material.mtl", "albedo.png"]},
        "backend_output_frame_validation": "NOT_RUN",
    }
    write_immutable(folder / "registry.json", json_bytes(registry))
    return registry


def validate_snapshot(source):
    source = Path(source)
    for name in ["rgb.png", "depth.npz", "mask.png", "camera_info.json"]:
        file_record(source / name)
    metadata = json.loads((source / "camera_info.json").read_text())
    with Image.open(source / "rgb.png") as image:
        rgb = np.asarray(image)
    with Image.open(source / "mask.png") as image:
        mask = np.asarray(image)
    with np.load(source / "depth.npz", allow_pickle=False) as archive:
        if archive.files != ["depth"]:
            raise ValueError("unexpected depth archive members")
        depth = archive["depth"]
    shape = (metadata["height"], metadata["width"])
    if rgb.shape != shape + (3,) or mask.shape != shape or depth.shape != shape:
        raise ValueError(f"RGB/depth/mask dimensions or shape mismatch: {source.name}")
    if rgb.dtype != np.uint8 or mask.dtype != np.uint8 or depth.dtype != np.float32:
        raise ValueError("RGB/depth/mask dtype mismatch")
    if not set(np.unique(mask)).issubset({0, 255}) or not np.any(mask):
        raise ValueError("mask must be nonempty mono8 with 0/255 labels")
    valid = np.isfinite(depth) & (depth > 0)
    if np.any((mask > 0) & ~valid):
        raise ValueError("mask includes invalid depth")
    if metadata["encoding"] != {"rgb": "rgb8", "depth": "32FC1"}:
        raise ValueError("historical encoding contract mismatch")
    K = np.array(metadata["K"], dtype=float).reshape(3, 3)
    P = np.array(metadata["P"], dtype=float).reshape(3, 4)
    if not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0 or not np.allclose(K[2], [0, 0, 1]) or not np.allclose(K, P[:, :3]):
        raise ValueError("invalid or inconsistent pinhole K/P")
    if np.any(np.asarray(metadata["D"]) != 0) or not np.allclose(np.asarray(metadata["R"]).reshape(3, 3), np.eye(3)):
        raise ValueError("historical fixture must have zero distortion and identity rectification")
    stamp = metadata["capture_stamp_ns"]
    if not isinstance(stamp, int) or stamp <= 0 or any(metadata["exact_sync_stamps"].get(k) != stamp for k in ["rgb", "depth", "info"]):
        raise ValueError("RGB/depth/CameraInfo collection stamps differ")
    for key in ["base_from_camera", "odom_from_camera"]:
        tf = metadata[key]
        if tf["stamp_ns"] != stamp or tf["child"] != metadata["frame"]:
            raise ValueError("capture-time TF stamp/frame differs")
        vector = np.asarray(tf["translation"] + tf["quaternion_xyzw"])
        if vector.shape != (7,) or not np.isfinite(vector).all() or not math.isclose(float(np.linalg.norm(vector[3:])), 1., abs_tol=1e-6):
            raise ValueError("invalid capture-time TF")
    if not metadata.get("source_epoch") or not metadata.get("calibration_revision"):
        raise ValueError("missing calibration/source provenance")
    if metadata.get("truth_used_for_segmentation_or_projection") is not False or "HSV magenta" not in metadata.get("segmentation", ""):
        raise ValueError("unknown historical mask provenance")
    if metadata["target_mask_pixels"] != int(np.count_nonzero(mask)):
        raise ValueError("mask pixel count does not match source metadata")
    return metadata, {
        "status": "PASS", "scenario": source.name, "shape": list(shape), "rgb_encoding": "rgb8",
        "depth_encoding": "32FC1", "depth_units": "m", "mask_source": "hsv_color_fixture",
        "mask_pixels": int(np.count_nonzero(mask)), "valid_depth_pixels": int(valid.sum()),
        "mask_invalid_depth_pixels": 0, "frame": metadata["frame"], "capture_stamp_ns": stamp,
        "calibration_revision": metadata["calibration_revision"], "source_epoch": metadata["source_epoch"],
        "tf_at_capture_stamp": True, "mask_is_general_instance_segmentation": False,
        "current_camera_geometry_validated": False, "sequence_frames": 1,
        "source_files": {name: file_record(source / name) for name in ["rgb.png", "depth.npz", "mask.png", "camera_info.json"]},
    }


def export_snapshot(source, output):
    metadata, report = validate_snapshot(source)
    algorithm = output / "algorithm_inputs" / source.name
    for name in ["rgb.png", "depth.npz", "mask.png"]:
        write_immutable(algorithm / name, (source / name).read_bytes())
    camera = {key: metadata[key] for key in ["K", "D", "R", "P", "width", "height", "frame", "encoding", "capture_stamp_ns", "exact_sync_stamps", "source_epoch", "calibration_revision"]}
    camera.update(schema="astribot.offline_rgbd_fixture/1", mask_source="hsv_color_fixture", depth_units="m", execution_authorized=False)
    write_immutable(algorithm / "camera.json", json_bytes(camera))
    report["algorithm_files"] = {name: file_record(algorithm / name) for name in ["rgb.png", "depth.npz", "mask.png", "camera.json"]}
    evaluator = output / "evaluation_only" / source.name
    report["evaluation_files"] = {}
    for name in ["camera_info.json", "scene.xyz", "truth.json", "truth_setup.json"]:
        path = source / name
        write_immutable(evaluator / name, path.read_bytes())
        report["evaluation_files"][name] = file_record(evaluator / name)
    truth = json.loads((source / "truth.json").read_text())
    if truth["capture_stamp_ns"] != metadata["capture_stamp_ns"] or truth.get("target_static") is not True or truth.get("truth_used_for_estimator_input") is not False:
        raise ValueError("historical scoring timestamp/static-target contract mismatch")
    report["truth_sample_delta_s"] = truth["truth_minus_capture_sec"]
    report["evaluation_scope"] = "static target; later independent truth sample with camera TF at image stamp"
    return report


def source_licenses(archive_path, repository, commit, output):
    prefix = f"{repository}-{commit}/"
    records = []
    with tarfile.open(archive_path, "r:gz") as archive:
        members = archive.getmembers()
        if not members or any(member.name != prefix.rstrip("/") and not member.name.startswith(prefix) for member in members):
            raise ValueError("source archive does not match pinned repository/commit")
        for member in members:
            if member.isfile() and Path(member.name).name.lower() in {"license", "license.txt", "license.md", "notice", "notice.txt"}:
                if member.size > 5_000_000:
                    raise ValueError("unexpected license file size")
                relative = Path(member.name[len(prefix):])
                if relative.is_absolute() or ".." in relative.parts:
                    raise ValueError("unsafe archive license path")
                path = output / "licenses" / repository / relative
                write_immutable(path, archive.extractfile(member).read())
                records.append(dict(file_record(path), archive_member=member.name))
    if not records:
        raise ValueError("pinned source archive contains no license")
    return records


def prepare_remote_assets(cache, output, offline, timeout_s, budget_s):
    deadline = time.monotonic() + budget_s
    results, failures = {}, []

    def fetch(key, url, filename, **expected):
        try:
            record = download_atomic(url, cache / filename, timeout_s=min(timeout_s, deadline - time.monotonic()), allow_network=not offline, **expected)
            results[key] = dict(record, status="PASS")
            return Path(record["path"])
        except (OSError, ValueError, RuntimeError) as exc:
            results[key] = {"status": "BLOCKED", "source_url": url, "reason": str(exc)}
            failures.append(key)
            return None

    listing = fetch("ngc_files_api", NGC_VERSION_URL + "/files", "ngc_1.0.0_onnx_files.json")
    metadata_path = fetch("ngc_version_api", NGC_VERSION_URL, "ngc_1.0.0_onnx_metadata.json")
    listing_valid = False
    if listing:
        try:
            import base64
            metadata = json.loads(listing.read_text())
            if metadata["modelVersion"]["versionId"] != "1.0.0_onnx":
                raise ValueError("NGC model version changed")
            published = {row["path"]: (row["sizeInBytes"], base64.b64decode(row["sha256_base64"], validate=True).hex()) for row in metadata["modelFiles"]}
            if any(published.get(name) != expected for name, expected in MODELS.items()):
                raise ValueError("NGC published asset size/hash differs from pinned 2026-09-23 values")
            listing_valid = True
            write_immutable(output / "provenance" / "ngc_files_api.json", listing.read_bytes())
        except (KeyError, ValueError) as exc:
            results["ngc_files_api"].update(status="BLOCKED", reason=str(exc)); failures.append("ngc_files_api_validation")
    if metadata_path:
        try:
            metadata = json.loads(metadata_path.read_text())
            if NGC_LICENSE_URL not in metadata["model"]["description"]:
                raise ValueError("exact-version NGC metadata does not name expected model EULA")
            write_immutable(output / "provenance" / "ngc_version_api.json", metadata_path.read_bytes())
            license_path = fetch("ngc_model_license", NGC_LICENSE_URL, "tao_toolkit_21-08_models_eula.pdf")
            if license_path:
                if not license_path.read_bytes().startswith(b"%PDF-"):
                    raise ValueError("NGC EULA download is not PDF")
                write_immutable(output / "licenses" / "ngc_model_eula.pdf", license_path.read_bytes())
            results["license_scope"] = {"model_source": NGC_VERSION_URL, "model_license_url": NGC_LICENSE_URL,
                "note": "License link observed on exact-version API; not inferred from ROS Apache license or NVlabs research license."}
        except (KeyError, ValueError) as exc:
            results["ngc_version_api"].update(status="BLOCKED", reason=str(exc)); failures.append("ngc_license_validation")
    for name, (size, digest) in MODELS.items():
        if listing_valid:
            fetch(name, NGC_VERSION_URL + "/files/" + name, "models/" + name, expected_sha256=digest, expected_bytes=size)
        else:
            results[name] = {"status": "BLOCKED", "reason": "NGC exact-version source/hash verification unavailable"}
            failures.append(name)
    for repository, commit in COMMITS.items():
        url = f"https://codeload.github.com/NVIDIA-ISAAC-ROS/{repository}/tar.gz/{commit}"
        archive = fetch(repository, url, f"sources/{repository}-{commit}.tar.gz")
        results[repository].update(commit=commit, release="release-3.2", archive_contains_git_history=False)
        if archive:
            try:
                results[repository]["licenses"] = source_licenses(archive, repository, commit, output)
            except (tarfile.TarError, ValueError, OSError) as exc:
                results[repository].update(status="BLOCKED", reason=str(exc)); failures.append(repository + "_validation")
    return {"status": "PASS" if not failures else "BLOCKED", "artifacts": results, "blocked_items": failures,
            "onnx_semantic_check": {"status": "NOT_RUN", "reason": "No ONNX checker is invoked; exact published bytes/hash validation is separate from model execution."}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="immutable run output directory")
    parser.add_argument("--cache", type=Path, required=True, help="isolated downloads cache, separate from GraspNet")
    parser.add_argument("--dataset", type=Path, required=True, help="explicit historical seven-snapshot root")
    parser.add_argument("--geometry", type=Path, default=REPO / "ws_robot/src/astribot_object_pose_core/models/asymmetric_union.visibility.json")
    parser.add_argument("--offline", action="store_true", help="only validate existing cached downloads")
    parser.add_argument("--download-timeout-seconds", type=float, default=120)
    parser.add_argument("--network-budget-seconds", type=float, default=300)
    args = parser.parse_args(argv)
    for value in [args.download_timeout_seconds, args.network_budget_seconds]:
        if not math.isfinite(value) or value <= 0:
            parser.error("network timeout/budget must be positive and finite")
    args.output = args.output.resolve(); args.cache = args.cache.resolve(); args.dataset = args.dataset.resolve()
    if args.output == args.dataset or args.output in args.dataset.parents or args.dataset in args.output.parents:
        parser.error("output and source dataset must be disjoint")
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / ".prepare.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("another preparation owns this output directory")
        script = file_record(Path(__file__).resolve())
        prior_path = args.output / "manifest.json"
        prior = json.loads(prior_path.read_text()) if prior_path.exists() else None
        if prior and prior["preparation_script"]["sha256"] != script["sha256"]:
            raise ValueError("existing output used a different preparation script; choose a new output")
        write_immutable(args.output / "provenance" / "prepare_p0.py", Path(__file__).read_bytes())
        # Validate all inputs before publishing any dataset copy.
        for scenario in SCENARIOS:
            validate_snapshot(args.dataset / scenario)
        cad = prepare_cad(args.geometry.resolve(), args.output)
        snapshots = [export_snapshot(args.dataset / scenario, args.output) for scenario in SCENARIOS]
        if len({(s["frame"], s["calibration_revision"], s["source_epoch"]) for s in snapshots}) != 1:
            raise ValueError("historical dataset mixes camera frame/calibration/source epoch")
        remote = prepare_remote_assets(args.cache, args.output, args.offline, args.download_timeout_seconds, args.network_budget_seconds)
        manifest = {
            "schema": "astribot.foundationpose.p0_preparation/1", "prepared_at_utc": prior["prepared_at_utc"] if prior else datetime.now(timezone.utc).isoformat(),
            "preparation_script": script, "output": str(args.output), "cache": str(args.cache), "dataset_source": str(args.dataset),
            "cpu_preparation": "PASS" if remote["status"] == "PASS" else "BLOCKED", "official_assets": remote,
            "cad": cad, "dataset": {"status": "PASS", "scenarios": snapshots, "independent_snapshots": 7, "continuous_sequence": False},
            "consumer_boundary": {"algorithm_root": str(args.output / "algorithm_inputs"), "evaluation_only_root": str(args.output / "evaluation_only"),
                "instruction": "Give estimator only algorithm_inputs and cad. Never mount evaluation_only or this manifest into estimator. Directory separation alone is not OS access control."},
            "dependency_environment": {"status": "BLOCKED", "reason": "Docker/NVIDIA Container Toolkit/TensorRT/GXF not provisioned or tested; remaining transitive dependencies/container digest unpinned."},
            "engine_build": {"status": "NOT_RUN", "precision": "FP32", "reason": "Requires isolated supported environment and explicit release of shared GPU ownership."},
            "official_example_inference": {"status": "NOT_RUN"}, "project_inference": {"status": "NOT_RUN"},
            "tracking_benchmark": {"status": "NOT_RUN", "reason": "Seven static snapshots are not a tracking sequence."},
            "p0_stage": "BLOCKED", "robot_execution_authorized": False,
        }
        write_immutable(args.output / "dataset_validation.json", json_bytes(manifest["dataset"]))
        write_immutable(args.output / "manifest.json", json_bytes(manifest))
        print(json.dumps({"manifest": str(prior_path), "cpu_preparation": manifest["cpu_preparation"], "p0_stage": "BLOCKED", "blocked_downloads": remote["blocked_items"]}))
        return 0 if manifest["cpu_preparation"] == "PASS" else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, OSError, KeyError) as error:
        print(f"P0 preparation rejected: {error}", file=sys.stderr)
        sys.exit(1)
