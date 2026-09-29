#!/usr/bin/env python3
"""Verify camera collision slices using the production C++ RobotModel probe.

Offline model coverage only: does not grant planning or motion acceptance.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull


def contains(outer, inner):
    eq = ConvexHull(np.asarray(outer)).equations
    return bool(np.all(np.asarray(inner) @ eq[:, :2].T + eq[:, 2] <= 1e-8))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--urdf', type=Path, required=True)
    p.add_argument('--probe', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    root = Path(__file__).resolve().parents[2]
    text = a.urdf.read_text()
    robot = ET.fromstring(text)
    camera_links = [link.get('name') for link in robot.findall('link')
                    if link.findall('collision') and '_camera_link' in link.get('name')]
    assert len(camera_links) == 6, camera_links
    q = {j.get('name'): 0. for j in robot.findall('joint') if j.get('type') != 'fixed'}
    srdf = ET.parse(root / 'ws_robot/src/astribot_s1_moveit_config/config/astribot_s1.srdf')
    poses = {}
    for name in ('home', 'ready_left', 'ready_right', 'ready_both'):
        state = dict(q)
        for group in srdf.findall('group_state'):
            g = group.get('group')
            selected = 'ready' if (g == 'arm_left' and name in ('ready_left', 'ready_both') or
                                   g == 'arm_right' and name in ('ready_right', 'ready_both')) else 'home'
            if group.get('name') == selected:
                state.update({j.get('name'): float(j.get('value')) for j in group.findall('joint')})
        poses[name] = state
    variants = {'full': text}
    for camera in camera_links:
        subset = copy.deepcopy(robot)
        for link in subset.findall('link'):
            if link.get('name') != camera:
                for c in link.findall('collision'):
                    link.remove(c)
        variants[camera] = ET.tostring(subset, encoding='unicode')
    requests = []
    keys = []
    for name, pose in poses.items():
        for variant, urdf in variants.items():
            keys.append((name, variant))
            requests.append(dict(urdf=urdf, frame='astribot_torso_base', q=pose,
                                 errors={k: .01 for k in pose}, padding=.01,
                                 packages={'astribot_s1_description': str(root / 'ws_robot/install/astribot_s1_description/share/astribot_s1_description')}))
    result = subprocess.run([str(a.probe.resolve())], input=''.join(json.dumps(r)+'\n' for r in requests),
                            capture_output=True, text=True, check=True, timeout=90)
    rows = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(rows) == len(keys)
    assert all('error' not in row for row in rows), [r for r in rows if 'error' in r]
    outputs = dict(zip(keys, rows))
    cases = []
    for name in poses:
        full = outputs[name, 'full']
        layers = {s['z_min']: s for s in full['slices']}
        for camera in camera_links:
            subset = outputs[name, camera]
            layer_ok = all(s['z_min'] in layers and contains(layers[s['z_min']]['footprint'], s['footprint']) for s in subset['slices'])
            cases.append(dict(pose=name, camera=camera, slices=len(subset['slices']),
                              physical_contained=contains(full['physical'], subset['physical']),
                              reserved_contained=contains(full['reserved'], subset['reserved']),
                              slices_contained=layer_ok))
    report = dict(scope='production C++ geometry; offline coverage, not navigation/ACK acceptance',
                  urdf_sha256=hashlib.sha256(text.encode()).hexdigest(),
                  probe_sha256=hashlib.sha256(a.probe.read_bytes()).hexdigest(),
                  camera_collision_count=sum(len(l.findall('collision')) for l in robot.findall('link') if l.get('name') in camera_links),
                  cases=cases, passed=all(c['physical_contained'] and c['reserved_contained'] and c['slices_contained'] for c in cases))
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'passed': report['passed'], 'cases': len(cases)}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
