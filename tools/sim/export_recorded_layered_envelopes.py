#!/usr/bin/env python3
"""Obtain offline envelopes from recorded URDF, joint feedback and payload ledger.

Uses the C++ geometry probe for all geometry. Never publishes runtime authority.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--recording", type=Path, required=True)
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--description-package", type=Path, required=True)
    parser.add_argument("--ground-in-base", type=float, required=True)
    parser.add_argument("--sample-times", type=float, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    profile = yaml.safe_load(args.profile.read_text())["/**"]["ros__parameters"]
    edges = [h + args.ground_in_base for h in profile["height_edges"]]
    urdf_path = args.recording / "fixtures/robot_state_publisher.urdf"
    events_path = args.recording / "m5_observer/events.jsonl"
    ledger_path = args.recording / "stack/payload_ledger.jsonl"
    selected = {t: None for t in args.sample_times}
    for line in events_path.open():
        event = json.loads(line)
        if event.get("topic") != "/joint_states":
            continue
        stamp = event["source_ns"]
        for t in selected:
            if stamp <= round(t * 1e9):
                previous = selected[t]
                if previous is None or previous["source_ns"] < stamp:
                    selected[t] = event
    ledger = [json.loads(line) for line in ledger_path.open()]
    inputs, source_records = [], []
    for t, sample in selected.items():
        assert sample is not None, f"no joint feedback before {t}"
        stamp = sample["source_ns"]
        assert round(t * 1e9) - stamp <= 300000000, "joint feedback stale at selected time"
        observations = [e for e in ledger if e["event"] == "observation" and e["observed_at_ns"] <= stamp]
        observation = max(observations, key=lambda e: e["observed_at_ns"])
        # This log persists ledger transitions, not every lease heartbeat.
        # Retain its age; it can supply historical shapes, never live admission.
        confirmations = [e for e in ledger if e["event"] == "scene_confirmed"
                         and e["ledger_epoch"] == observation["ledger_epoch"]
                         and e["ledger_revision"] == observation["ledger_revision"]
                         and e["confirmed_at_ns"] <= stamp]
        assert confirmations, "no independent recorded scene confirmation"
        confirmation = max(confirmations, key=lambda e: e["confirmed_at_ns"])
        attachments = []
        for obj in observation["objects"]:
            object_pose = np.eye(4)
            object_pose[:3, :3] = Rotation.from_quat(obj["pose"]["quaternion"]).as_matrix()
            object_pose[:3, 3] = obj["pose"]["position"]
            for shape in obj["shapes"]:
                local = np.eye(4)
                local[:3, :3] = Rotation.from_quat(shape["pose"]["quaternion"]).as_matrix()
                local[:3, 3] = shape["pose"]["position"]
                kind = {1: "box", 2: "sphere", 3: "cylinder"}[shape["type"]]
                dimensions = shape["dimensions"]
                if kind == "cylinder":
                    dimensions = [dimensions[1], dimensions[0]]
                attachments.append(dict(link=obj["link"], kind=kind, dimensions=dimensions,
                                        pose=(object_pose @ local).tolist()))
        joint = sample["data"]
        assert len(joint["name"]) == len(joint["position"])
        q = dict(zip(joint["name"], joint["position"]))
        inputs.append(dict(urdf=urdf_path.read_text(), frame="astribot_torso_base",
                           packages={"astribot_s1_description": str(args.description_package.resolve())},
                           q=q, errors={name: .003 for name in q}, padding=.01,
                           attachments=attachments, layer_edges=edges))
        source_records.append(dict(requested_time_s=t, joint_event=sample, observation=observation,
                                   scene_confirmation=confirmation))
    encoded = "".join(json.dumps(item) + "\n" for item in inputs)
    (args.output / "input.jsonl").write_text(encoded)
    result = subprocess.run([str(args.probe.resolve())], input=encoded, text=True,
                            capture_output=True, check=True, timeout=120)
    outputs = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(outputs) == len(inputs)
    summary = []
    for data, sources in zip(outputs, source_records):
        assert "error" not in data, data
        assert len(data["slices"]) == len(edges) - 1
        assert set(data["required"]) <= set(sources["joint_event"]["data"]["name"])
        summary.append(dict(time_s=sources["joint_event"]["source_ns"] / 1e9,
                            attachment_revision=sources["scene_confirmation"]["attachment_revision"],
                            attachment_ids=[o["id"] for o in sources["observation"]["objects"]],
                            model_revision=data["revision"], top_above_ground_m=data["height"] - args.ground_in_base,
                            height_covered=data["height"] <= edges[-1],
                            ledger_fresh_at_joint_sample=sources["joint_event"]["source_ns"] < sources["observation"]["valid_until_ns"],
                            ledger_observation_age_s=(sources["joint_event"]["source_ns"] - sources["observation"]["observed_at_ns"]) / 1e9,
                            runtime_admissible=False,
                            layer_vertex_counts=[len(s["footprint"]) for s in data["slices"]]))
    (args.output / "envelopes.json").write_text(json.dumps(outputs, indent=2) + "\n")
    (args.output / "recorded_sources.json").write_text(json.dumps(source_records, indent=2) + "\n")
    report = dict(passed=True, evidence="offline recomputation from recorded simulation feedback; not current runtime authority",
                  profile=str(args.profile.resolve()), height_edges_ground_m=profile["height_edges"],
                  layer_edges_base_m=edges, ground_in_base_m=args.ground_in_base,
                  ground_reference="scene39 flat near-level simulation model audit; not hardware calibration",
                  assumed_joint_error_bound=.003, model_padding_m=.01,
                  artifacts_sha256={str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in (urdf_path, ledger_path, args.profile, args.probe)}, samples=summary)
    (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
