#!/usr/bin/env python3
"""Replay captured optical-frame XYZ through the real C++ worker.

Reads scene.xyz and camera_info.json only; ground-truth pose is not model input.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--captures', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--device', choices=['cpu', 'cuda'], required=True)
    parser.add_argument('--cases', nargs='+', default=['close', 'tilted', 'yawed'])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for name in args.cases:
        capture = args.captures / name
        info = json.loads((capture / 'camera_info.json').read_text())
        xyz = np.loadtxt(capture / 'scene.xyz')[:, :3].astype('<f4')
        input_path, output_path = args.output / f'{name}.cloud.bin', args.output / f'{name}.grasps.bin'
        xyz.tofile(input_path)
        command = [str(args.worker.resolve()), '--model', str(args.model.resolve()),
                   '--input', str(input_path.resolve()), '--output', str(output_path.resolve()),
                   '--device', args.device]
        started = time.perf_counter()
        completed = subprocess.run(command, capture_output=True, text=True, timeout=120)
        elapsed = (time.perf_counter() - started) * 1000
        (args.output / f'{name}.stdout.log').write_text(completed.stdout)
        (args.output / f'{name}.stderr.log').write_text(completed.stderr)
        if completed.returncode != 0:
            raise RuntimeError(f'{name}: worker failed: {completed.stderr}')
        result = json.loads(completed.stdout)
        values = np.fromfile(output_path, dtype='<f4').reshape(-1, 17)
        if not np.isfinite(values).all():
            raise RuntimeError('nonfinite output')
        result.update({
            'case': name, 'process_wall_ms': elapsed,
            'input_sha256': sha256(input_path), 'output_sha256': sha256(output_path),
            'capture_stamp_ns': info['capture_stamp_ns'], 'frame': info['frame'],
            'source_epoch': info['source_epoch'], 'calibration_revision': info['calibration_revision'],
            'camera_health_at_capture': info['camera_health_at_capture'],
            'segmentation': info['segmentation'], 'truth_used_for_inference': False,
            'score_min': float(values[:, 0].min()) if len(values) else None,
            'score_max': float(values[:, 0].max()) if len(values) else None,
            'width_min_m': float(values[:, 1].min()) if len(values) else None,
            'width_max_m': float(values[:, 1].max()) if len(values) else None,
            'physical_grasp_success_tested': False,
        })
        if len(values):
            rotations = values[:, 4:13].reshape(-1, 3, 3)
            error = float(np.max(np.abs(rotations.transpose(0,2,1) @ rotations - np.eye(3))))
            if error > 1e-4 or np.max(np.abs(np.linalg.det(rotations)-1)) > 1e-4:
                raise RuntimeError('output rotation is not a proper rotation')
            result['rotation_orthogonality_max_error'] = error
        results.append(result)
        print(json.dumps({k: result[k] for k in ('case', 'input_points', 'candidates', 'score_max', 'process_wall_ms')}), flush=True)
    # Malformed data must fail before loading/inferencing the model and must not
    # create the final output artifact. Each probe has its own fresh output path.
    rejected = []
    probes = {'empty': np.empty((0, 3), dtype='<f4'),
              'sparse': np.ones((32, 3), dtype='<f4'),
              'nonfinite': np.ones((2048, 3), dtype='<f4'),
              'negative_depth': np.ones((2048, 3), dtype='<f4')}
    probes['nonfinite'][10, 0] = np.nan
    probes['negative_depth'][10, 2] = -1
    for name, points in probes.items():
        input_path, output_path = args.output / f'reject_{name}.cloud.bin', args.output / f'reject_{name}.grasps.bin'
        points.tofile(input_path)
        if output_path.exists():
            output_path.unlink()
        completed = subprocess.run([str(args.worker.resolve()), '--model', str(args.model.resolve()),
                     '--input', str(input_path.resolve()), '--output', str(output_path.resolve()), '--device', args.device],
                     capture_output=True, text=True, timeout=30)
        if completed.returncode == 0 or output_path.exists():
            raise RuntimeError(f'malformed probe accepted: {name}')
        rejected.append({'input': name, 'exit_code': completed.returncode, 'error': completed.stderr.strip()})
    summary = {'model_sha256': sha256(args.model), 'device': args.device,
               'mode': 'offline replay of actual Gazebo RGB-D captures',
               'fresh_action_service_acceptance': False,
               'results': results, 'rejected_inputs': rejected}
    (args.output / 'report.json').write_text(json.dumps(summary, indent=2) + '\n')


if __name__ == '__main__':
    main()
