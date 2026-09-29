#!/usr/bin/env python3
"""Exercise the public executable boundary, including real global registration."""
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

executable, model_file, visibility_file = sys.argv[1:]
with tempfile.TemporaryDirectory() as temporary:
    directory = Path(temporary)
    scene = directory/'scene.xyz'
    rows = [list(map(float, line.split())) for line in Path(model_file).read_text().splitlines()
            if line and not line.startswith('#')]
    # Independently specified 98.4-degree orientation, viewing two complete faces.
    # This expectation does not use any production transform helper.
    boxes = json.loads(Path(visibility_file).read_text())['boxes']
    k = math.sqrt(.5)
    eye = [.9*k, -.9*k, 0.]  # -R^T t for the fixed transform.
    visible = []
    for p in rows:
        ray = [p[k]-eye[k] for k in range(3)]
        if sum(ray[k]*p[k+3] for k in range(3)) >= 0:
            continue
        blocked = False
        for box in boxes:
            for axis in range(3):
                if abs(ray[axis]) < 1e-12:
                    continue
                for sign in (-1, 1):
                    hit = (box['center_m'][axis]+sign*box['size_m'][axis]/2-eye[axis])/ray[axis]
                    if 0 <= hit < 1-1e-5:
                        contact = [eye[k]+hit*ray[k] for k in range(3)]
                        blocked |= all(abs(contact[k]-box['center_m'][k]) <= box['size_m'][k]/2+1e-7 for k in range(3))
        if not blocked:
            visible.append(p)
    scene.write_text('\n'.join(' '.join(map(str, [k*(p[0]+p[1]), -p[2], k*(-p[0]+p[1])+.9,
                                                       k*(p[3]+p[4]), -p[5], k*(-p[3]+p[4])])) for p in visible)+'\n')
    result = subprocess.run([executable, '--model', model_file, '--scene', str(scene), '--visibility-model', visibility_file],
                            capture_output=True, text=True, timeout=120)
    assert result.returncode != 64, ('required visibility-model CLI option is unavailable', result.stderr)
    payload = json.loads(result.stdout)
    assert result.returncode == 0 and payload['success'], (result.returncode, payload, result.stderr)
    matrix = payload['camera_from_object']
    assert len(matrix) == 4 and all(len(row) == 4 for row in matrix), matrix
    expected = [[k,k,0,0],[0,0,-1,0],[-k,k,0,.9],[0,0,0,1]]
    assert sum((matrix[i][3]-expected[i][3])**2 for i in range(3))**.5 < .02, matrix
    rotation_trace = sum(matrix[i][j]*expected[i][j] for i in range(3) for j in range(3))
    assert math.acos(max(-1,min(1,(rotation_trace-1)/2))) < math.radians(10), matrix
    assert payload['visible_model_coverage'] >= .75 and payload['observable_normal_directions'] >= 2, payload
    assert payload['coverage'] > 0 and payload['scene_coverage'] >= .65, payload
    output = directory/'result.json'
    scene.write_text('')
    invalid = subprocess.run([executable, '--model', model_file, '--scene', str(scene), '--output', str(output)],
                             capture_output=True, text=True, timeout=10)
    assert invalid.stdout == '', invalid.stdout
    failure = json.loads(output.read_text())
    assert invalid.returncode == 2 and not failure['success'] and failure['camera_from_object'] is None, failure
    scene.write_text('1 2 3 4\n')
    malformed = subprocess.run([executable, '--model', model_file, '--scene', str(scene)],
                               capture_output=True, text=True, timeout=10)
    failure = json.loads(malformed.stdout)
    assert malformed.returncode == 2 and failure['reason'] == 'invalid_scene_file', failure
    bad_geometry = directory/'bad_geometry.json'
    bad_geometry.write_text('{"schema":"wrong", "boxes":[]}')
    invalid_geometry = subprocess.run([executable, '--model', model_file, '--scene', str(scene),
                                       '--visibility-model', str(bad_geometry)],
                                      capture_output=True, text=True, timeout=10)
    failure = json.loads(invalid_geometry.stdout)
    assert invalid_geometry.returncode == 2 and failure['reason'] == 'invalid_visibility_model_file', failure
print('CLI transform, visibility registry, rejection, JSON and exit status contract passed')
