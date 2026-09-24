#!/usr/bin/env python3
"""Isolated ROS/static-archive acceptance; does not launch Gazebo or command motion."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from astribot_slam_msgs.msg import HeightSliceMaps
from nav_msgs.msg import OccupancyGrid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    root = Path(args.output).resolve()
    root.mkdir(parents=True, exist_ok=False)
    assert os.environ.get("ROS_DOMAIN_ID") == "219", "isolated protocol domain must be 219"
    session = root / "archive"
    (session / "kf").mkdir(parents=True)
    # Pose 1 rotates LOCAL x into map z and local z into -map x.
    q = math.sqrt(0.5)
    (session / "alidarState.txt").write_text(
        f"1 0 0 0 0 0 0 1\n2 0 0 0 0 {-q} 0 {q}\n")
    expected = [(1.0, 0.0, .1), (5.0, 0.0, .1), (2.0, 0.0, .5),
                (3.0, 0.0, .9), (4.0, 0.0, 1.4), (6.0, 0.0, 2.0),
                (-.1, -.1, .1), (7.0, 0.0, 2.4)]
    local = [(z, y, -x) for x, y, z in expected]
    header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
              f"COUNT 1 1 1\nWIDTH {len(local)}\nHEIGHT 1\nPOINTS {len(local)}\nDATA binary\n")
    (session / "kf/1.pcd").write_bytes(header.encode() + b"".join(struct.pack("fff", *p) for p in local))
    manifest = {"world_frame": "map", "keyframes": 1, "scans": 2,
                "sha256": {str(p.relative_to(session)): hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in (session / "alidarState.txt", session / "kf/1.pcd")}}
    (session / "manifest.json").write_text(json.dumps(manifest))
    command = [args.executable, "--ros-args", "--params-file", args.config,
               "-p", f"session_directory:={session}", "-p", f"output_directory:={root / 'maps'}",
               "-p", "frame_id:=map", "-p", "ground_z:=0.0", "-p", "ground_reference:=known_fixture_plane"]
    rclpy.init()
    node = rclpy.create_node("height_map_protocol_verifier")
    assert not node.count_publishers("/height_maps/snapshot"), "domain already occupied"
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    received, layers, subscriptions = [], {}, []
    subscriptions.append(node.create_subscription(HeightSliceMaps, "/height_maps/snapshot", received.append, qos))
    names = ["low_obstacle", "main_nav", "torso", "head", "upper_extension"]
    for name in names:
        subscriptions.append(node.create_subscription(OccupancyGrid, f"/height_maps/{name}",
                                                       lambda msg, key=name: layers.__setitem__(key, msg), qos))
    result = {"passed": False, "evidence": "isolated ROS protocol and synthetic 3D archive, not Gazebo",
              "domain": 219, "command": command,
              "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip()}
    with (root / "node.log").open("w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        result["pid"] = process.pid
        try:
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline and (not received or len(layers) != 5):
                assert process.poll() is None, "node exited; see node.log"
                rclpy.spin_once(node, timeout_sec=.05)
            assert received and len(layers) == 5, "missing atomic/individual maps"
            bundle = received[-1]
            assert list(bundle.layer_names) == names
            assert list(bundle.point_counts) == [3, 1, 1, 1, 1]
            assert bundle.above_band_points == 1
            assert len(bundle.map_revision) == len(bundle.profile_revision) == 64
            for i, name in enumerate(names):
                grid = layers[name]
                assert grid == bundle.grids[i], "torn layer output"
                assert grid.header.frame_id == "map"
                assert set(grid.data) <= {-1, 100} and -1 in grid.data
                assert sum(v == 100 for v in grid.data) == [3, 1, 1, 1, 1][i]
                assert grid.info == bundle.grids[0].info, "layer bounds differ"
                for x, y, z in expected:
                    if bundle.height_edges[i] <= z < bundle.height_edges[i + 1]:
                        # FLOAT32 coordinates can straddle a cell edge: use the actual
                        # fixture's transformed float coordinate, exactly as the C++ input.
                        zf, yf, minus_xf = struct.unpack("fff", struct.pack("fff", z, y, -x))
                        xf = -minus_xf
                        ix = round((xf - grid.info.origin.position.x) / .05)
                        iy = math.floor((yf - grid.info.origin.position.y + 1e-10) / .05)
                        # Check cell membership from world coordinates with a tiny edge
                        # neighborhood, not just total occupancy counts.
                        cells = [grid.data[cy * grid.info.width + cx]
                                 for cx in (ix - 1, ix) for cy in (iy - 1, iy)
                                 if 0 <= cx < grid.info.width and 0 <= cy < grid.info.height]
                        assert 100 in cells, (i, x, y)
            result.update(point_counts=list(bundle.point_counts),
                          above_band_points=bundle.above_band_points,
                          map_revision=bundle.map_revision, profile_revision=bundle.profile_revision,
                          free_cells=0, separate_maps_match_atomic=True, full_pose_rotation_verified=True)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                result["forced_cleanup"] = True
            result["exit_code"] = process.returncode
            node.destroy_node()
            rclpy.shutdown()
            (root / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    # A corrupt saved source must fail before publishing/exporting maps.
    with (session / "kf/1.pcd").open("ab") as f:
        f.write(b"corruption")
    bad_command = [s.replace(str(root / "maps"), str(root / "invalid_maps")) for s in command]
    rejected = subprocess.run(bad_command, capture_output=True, text=True, timeout=10)
    (root / "corrupt_source.log").write_text(rejected.stdout + rejected.stderr)
    assert rejected.returncode != 0 and "archive hash mismatch" in rejected.stderr
    assert not (root / "invalid_maps").exists()
    result["corrupt_source_rejected_before_export"] = True
    result["passed"] = True
    (root / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
