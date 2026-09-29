#!/usr/bin/env python3
"""Verify a pinned GraspNet checkpoint, run it, and export an isolated artifact.

This is preparation/validation tooling. Runtime inference is the C++ worker.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

os.environ.setdefault('CUBLAS_WORKSPACE_CONFIG', ':4096:8')
import numpy as np
import torch

SOURCE_COMMIT = '280c215129f759ed8649cb4e89fc5dfee55f4f80'
CHECKPOINT_HASH = '60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868'


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sample_cloud(path):
    values = np.fromfile(path, dtype='<f4')
    if values.size % 3 or values.size < 2048 * 3 or values.size > 2000000 * 3:
        raise ValueError('input must be 2048..2000000 XYZ float32 points')
    points = values.reshape(-1, 3)
    if not np.isfinite(points).all() or (points[:, 2] <= 0).any():
        raise ValueError('points must be finite optical-frame coordinates with positive Z')
    count = len(points)
    generator = torch.Generator(device='cpu').manual_seed(0)
    if count >= 20000:
        indices = torch.randperm(count, generator=generator)[:20000]
    else:
        indices = torch.cat((torch.arange(count), torch.randint(count, (20000-count,), generator=generator)))
    return points[indices.numpy()].copy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--checkpoint', required=True, type=Path)
    parser.add_argument('--cloud', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--device', choices=['cuda', 'cpu'], default='cuda')
    parser.add_argument('--skip-export', action='store_true')
    args = parser.parse_args()
    source = args.source.resolve()
    commit = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    if commit != SOURCE_COMMIT:
        raise RuntimeError(f'unsupported source commit: {commit}')
    subprocess.run(['git', '-C', str(source), 'diff', '--exit-code', 'HEAD', '--',
                    'models', 'pointnet2', 'utils'], check=True, stdout=subprocess.DEVNULL)
    if sha256(args.checkpoint) != CHECKPOINT_HASH:
        raise RuntimeError('checkpoint hash mismatch: refusing unverified or random weights')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for relative in ['models', 'pointnet2', 'utils']:
        sys.path.insert(0, str(source / relative))
    sys.path.insert(0, str(source))
    from graspnet_torch_ops import install_into_upstream
    install_into_upstream()
    from graspnet import GraspNet, pred_decode

    torch.set_num_threads(1)
    torch.manual_seed(0)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.use_deterministic_algorithms(True)
    device = torch.device(args.device)
    begin = time.perf_counter()
    network = GraspNet(input_feature_dim=0, num_view=300, num_angle=12, num_depth=4,
                       cylinder_radius=.05, hmin=-.02, hmax_list=[.01, .02, .03, .04],
                       is_training=False)
    checkpoint = torch.load(args.checkpoint, map_location='cpu', weights_only=True)
    network.load_state_dict(checkpoint['model_state_dict'], strict=True)
    network.eval().to(device)

    class DecodedNetwork(torch.nn.Module):
        def __init__(self, net):
            super().__init__()
            self.net = net

        def forward(self, points):
            return pred_decode(self.net({'point_clouds': points}))[0]

    wrapped = DecodedNetwork(network).eval()
    cloud = sample_cloud(args.cloud)
    tensor = torch.from_numpy(cloud[None]).to(device)
    if device.type == 'cuda':
        torch.cuda.synchronize()
    load_ms = (time.perf_counter() - begin) * 1000
    provenance = {
        'source_url': 'https://github.com/graspnet/graspnet-baseline',
        'source_commit': commit,
        'checkpoint_sha256': CHECKPOINT_HASH,
        'checkpoint_epoch': int(checkpoint['epoch']),
        'checkpoint_source': 'pinned HF mirror; direct official Drive quota exceeded',
        'publisher_signature_verified': False,
        'checkpoint_keys_loaded_strict': len(checkpoint['model_state_dict']),
        'torch_version': torch.__version__,
        'export_device': args.device,
        'numpy_version': np.__version__,
        'source_license_sha256': sha256(source / 'LICENSE'),
        'operators': 'scripted_torch_compatibility_v2',
        'fps_ties': 'pinned CUDA left-subtree reduction; bit-reversed lane rank then lane-local index',
        'operators_sha256': sha256(Path(__file__).with_name('graspnet_torch_ops.py')),
        'original_cuda_extension_bitwise_parity_verified': False,
        'sample_points': 20000,
        'sampling': 'torch CPU generator seed 0: randperm or randint existing points',
        'input': '1x20000x3 float32, camera optical XYZ metres',
        'output': 'Nx17 upstream pred_decode; no collision or NMS filter',
    }
    durations = []
    with torch.inference_mode():
        for _ in range(2):
            start = time.perf_counter()
            predicted = wrapped(tensor)
            values = predicted.cpu().numpy()
            durations.append((time.perf_counter() - start) * 1000)
        if values.ndim != 2 or values.shape[1] != 17 or not np.isfinite(values).all():
            raise RuntimeError('model output has invalid shape or nonfinite values')
        if not len(values):
            raise RuntimeError('verification fixture returned no grasps; not accepted as inference evidence')
        values.astype('<f4').tofile(str(args.output) + '.eager.bin')
        report = {'provenance': provenance, 'device': args.device,
                  'input_sha256': sha256(args.cloud), 'load_ms': load_ms,
                  'inference_ms': durations, 'candidates': len(values),
                  'score_min': float(values[:, 0].min()), 'score_max': float(values[:, 0].max()),
                  'rot_det_max_error': float(np.max(np.abs(np.linalg.det(values[:, 4:13].reshape(-1,3,3)) - 1))),
                  'rot_orthogonality_max_error': float(np.max(np.abs(np.matmul(values[:, 4:13].reshape(-1,3,3).transpose(0,2,1), values[:, 4:13].reshape(-1,3,3)) - np.eye(3)))),
                  'collision_checked': False, 'physical_grasp_success_tested': False}
        print(json.dumps(report, indent=2), flush=True)
        if not args.skip_export:
            # Scripted helper loops remain loops in the traced network graph.
            traced = torch.jit.trace(wrapped, (tensor,), check_trace=False, strict=True)
            extra = {'provenance.json': json.dumps(provenance, separators=(',', ':'))}
            torch.jit.save(traced, str(args.output), _extra_files=extra)
            reloaded = torch.jit.load(str(args.output), map_location=device).eval()
            actual = reloaded(tensor).cpu().numpy()
            np.testing.assert_allclose(actual, values, rtol=2e-5, atol=2e-6)
            report['torchscript_max_abs_error'] = float(np.max(np.abs(actual - values)))
            # A changed point cloud catches a trace that accidentally captured
            # data-dependent geometry or output masks as constants.
            changed = tensor.clone()
            changed[:, :, 0] += .011
            expected_changed = wrapped(changed).cpu().numpy()
            actual_changed = reloaded(changed).cpu().numpy()
            np.testing.assert_allclose(actual_changed, expected_changed, rtol=2e-5, atol=2e-6)
            if expected_changed.shape == values.shape and np.allclose(expected_changed, values):
                raise RuntimeError('model output did not respond to changed point cloud')
            # Persist a changed full input and its eager output so the C++ worker
            # can verify the same changed-input contract independently.
            changed_cloud = np.fromfile(args.cloud, dtype='<f4').reshape(-1, 3)
            changed_cloud[:, 0] += np.float32(.011)
            changed_cloud.tofile(str(args.output) + '.changed.cloud.bin')
            expected_changed.astype('<f4').tofile(str(args.output) + '.changed.eager.bin')
            report['changed_input_sha256'] = sha256(str(args.output) + '.changed.cloud.bin')
            report['changed_input_candidates'] = len(actual_changed)
            report['changed_input_torchscript_max_abs_error'] = float(np.max(np.abs(actual_changed - expected_changed)))
            report['export_sha256'] = sha256(args.output)
            report['export_bytes'] = args.output.stat().st_size
    shutil.copy2(source / 'LICENSE', args.output.parent / 'GRASPNET_UPSTREAM_LICENSE')
    Path(str(args.output) + '.report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
