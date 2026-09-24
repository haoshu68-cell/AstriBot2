#!/usr/bin/env python3
"""Independent geometry/file-binding checks, not a camera or inference test."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

out = Path(__file__).resolve().parent
root = out.parents[3]
scenario = json.loads((root / 'ws_robot/src/astribot_s1_transport/config/warehouse_transfer.json').read_text())
provenance = json.loads((out / 'provenance.json').read_text())
geometry = json.loads((out / 'geometry.json').read_text())
registry = json.loads((out / 'registry.json').read_text())
size = np.array(scenario['size_xyz'])
model = np.loadtxt(registry[geometry['model_id']]['path'])
assert model.shape == (4000, 6) and np.isfinite(model).all()
assert np.allclose(model[:, :3].min(axis=0), -size/2, atol=1e-9, rtol=0)
assert np.allclose(model[:, :3].max(axis=0), size/2, atol=1e-9, rtol=0)
assert np.allclose(np.linalg.norm(model[:, 3:], axis=1), 1)
assert np.allclose(np.sum(model[:, :3] * model[:, 3:], axis=1),
                   np.sum(np.abs(model[:, 3:]) * size/2, axis=1))
samples = set(map(tuple, np.round(model, 9)))
assert len(samples) == len(model)
# Independently enumerate signed permutations that preserve the unequal Z axis.
rotations = []
for swap in (False, True):
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                rotation = np.zeros((3, 3), dtype=int)
                rotation[0, int(swap)] = sx
                rotation[1, int(not swap)] = sy
                rotation[2, 2] = sz
                if round(np.linalg.det(rotation)) != 1:
                    continue
                rotations.append(rotation)
                transformed = np.hstack((model[:, :3] @ rotation.T, model[:, 3:] @ rotation.T))
                assert set(map(tuple, np.round(transformed, 9))) == samples
assert len(rotations) == 8
vertices, faces = [], []
for line in (out / 'transport_box_60x60x120.obj').read_text().splitlines():
    if line.startswith('v '): vertices.append(list(map(float, line.split()[1:])))
    if line.startswith('f '): faces.append([int(v)-1 for v in line.split()[1:]])
vertices = np.array(vertices)
edges = Counter(tuple(sorted((face[i], face[(i+1)%3]))) for face in faces for i in range(3))
assert all(count == 2 for count in edges.values())
volume = sum(np.dot(vertices[a], np.cross(vertices[b], vertices[c]))/6 for a,b,c in faces)
assert abs(volume - float(np.prod(size))) < 1e-12
assert np.allclose(np.ptp(vertices, axis=0), size)
for filename, digest in provenance['generated_sha256'].items():
    assert hashlib.sha256((out / filename).read_bytes()).hexdigest() == digest, filename
params = yaml.safe_load((out / 'inference.template.yaml').read_text())['manipulation_perception_server']['ros__parameters']
assert all(params[name] == 0 for name in ('calibration_revision','planning_scene_revision','envelope_epoch'))
assert params['max_input_age_sec'] == .5 and params['max_result_age_sec'] == 5.
assert geometry['object_instance'] == scenario['object_id']
assert registry[geometry['model_id']]['symmetry'] == 'square_prism_z'
result = {'status': 'PASS', 'scope': 'offline CAD, closed mesh, symmetry and file bindings',
          'cad_points': len(model), 'proper_rotations_including_identity': len(rotations),
          'closed_mesh_signed_volume_m3': volume,
          'geometry_size_m': list(size), 'synthetic_sensor_data_used': False,
          'actual_camera_or_inference_test': False}
(out / 'assets_validation.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result))
