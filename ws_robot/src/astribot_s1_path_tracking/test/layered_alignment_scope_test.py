#!/usr/bin/env python3
"""Offline byte-scope check against the saved pre-alignment working tree.

This checks edit scope, not collision correctness or runtime behavior.
Run with --baseline-dir pointing to the directory containing
three_phase_controller.before.{cpp,hpp} and before.sha256.
"""

import argparse
import hashlib
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-dir", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[4])
    args = parser.parse_args()
    package = args.repo_root / "ws_robot/src/astribot_s1_path_tracking"
    sources = {
        "cpp": package / "src/three_phase_controller.cpp",
        "hpp": package / "include/astribot_s1_path_tracking/three_phase_controller.hpp",
    }
    additions = {
        "cpp": [
            b"  alignment_collision_.configure(node,costmap_ros);\n",
            b"  alignment_collision_.cleanup();\n",
            b"      if (std::abs(start_err)>align_tol_rad_ || std::abs(wz_now)>align_settled_wz_) {\n"
            b"        alignment_collision_.requireRotationClear(pose,yawOf(pose.pose)+start_err);\n"
            b"      }\n",
            b"      if (std::abs(goal_err)>align_tol_rad_ || std::abs(wz_now)>align_settled_wz_) {\n"
            b"        alignment_collision_.requireRotationClear(pose,yawOf(pose.pose)+goal_err);\n"
            b"      }\n",
        ],
        "hpp": [
            b'#include "astribot_s1_path_tracking/layered_collision_reader.hpp"\n',
            b"  LayeredCollisionReader alignment_collision_;\n",
        ],
    }
    for extension, source in sources.items():
        before = (args.baseline_dir / f"three_phase_controller.before.{extension}").read_bytes()
        current = source.read_bytes()
        for addition in additions[extension]:
            if current.count(addition) != 1 or addition in before:
                raise AssertionError(f"missing, duplicate, or pre-existing alignment insertion: {source}")
            current = current.replace(addition, b"", 1)
        if current != before:
            raise AssertionError(f"changes outside the two alignment guards and lifecycle wiring: {source}")

    # Check insertion placement as well as the unchanged remaining bytes.
    cpp = sources["cpp"].read_bytes()
    for phase, error, addition in (
        (b"kAlignStart", b"start_err", additions["cpp"][2]),
        (b"kAlignGoal", b"goal_err", additions["cpp"][3]),
    ):
        expected = (
            b"    case Phase::" + phase + b":\n" + addition
            + b"      return rotateOnly(" + error + b", wz_now, pose.header);"
        )
        if cpp.count(expected) != 1:
            raise AssertionError(f"alignment guard moved outside its original case: {phase.decode()}")

    # The manifest also covers Arrival, the planner, critics, and EnvelopeGuard.
    checked = 0
    for line in (args.baseline_dir / "before.sha256").read_text().splitlines():
        digest, relative = line.split(maxsplit=1)
        source = args.repo_root / relative
        extension = next((ext for ext, path in sources.items() if path == source), None)
        content = (
            (args.baseline_dir / f"three_phase_controller.before.{extension}").read_bytes()
            if extension else source.read_bytes()
        )
        if hashlib.sha256(content).hexdigest() != digest:
            raise AssertionError(f"baseline or protected file differs from saved hash: {source}")
        checked += 1
    print(f"PASS: only START_ALIGN/GOAL_ALIGN guards and lifecycle wiring changed; {checked} saved hashes matched")


if __name__ == "__main__":
    main()
