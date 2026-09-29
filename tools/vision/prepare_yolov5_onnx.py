#!/usr/bin/env python3
"""Offline fixed-shape conversion with numerical parity; not a runtime dependency.

Run in a dedicated environment containing onnx, onnxsim and onnxruntime.
Does not download models or modify the original artifact.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import onnx
import onnxruntime as ort
import onnxsim


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--size', type=int, default=640)
    args = parser.parse_args()
    if args.input.resolve() == args.output.resolve() or not 32 <= args.size <= 2048:
        parser.error('output must differ from input; size must be 32..2048')
    original = onnx.load(args.input)
    if len(original.graph.input) != 1 or 'output' not in [o.name for o in original.graph.output]:
        raise ValueError('expected YOLOv5 single input and decoded output named output')
    name = original.graph.input[0].name
    shape = [1, 3, args.size, args.size]
    fixed, checked = onnxsim.simplify(original, overwrite_input_shapes={name: shape})
    if not checked:
        raise RuntimeError('ONNX simplification failed')
    fixed = onnx.utils.Extractor(fixed).extract_model([name], ['output'])
    onnx.checker.check_model(fixed)
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = 2
    opts.inter_op_num_threads = 1
    before = ort.InferenceSession(original.SerializeToString(), opts, providers=['CPUExecutionProvider'])
    after = ort.InferenceSession(fixed.SerializeToString(), opts, providers=['CPUExecutionProvider'])
    errors = []
    for seed in (0, 1):
        image = np.random.default_rng(seed).random(shape, dtype=np.float32)
        expected = before.run(['output'], {name: image})[0]
        actual = after.run(['output'], {name: image})[0]
        np.testing.assert_allclose(actual, expected, rtol=1e-4, atol=1e-4)
        errors.append(float(np.max(np.abs(actual-expected))))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(fixed, args.output)
    manifest = {
        'source_url': 'https://github.com/ultralytics/yolov5/releases/tag/v6.0',
        'input': str(args.input.resolve()), 'output': str(args.output.resolve()),
        'input_sha256': sha(args.input), 'output_sha256': sha(args.output),
        'input_shape': shape, 'output_shape': list(actual.shape),
        'onnx': onnx.__version__, 'onnxsim': onnxsim.__version__, 'onnxruntime': ort.__version__,
        'parity': {'seeds': [0, 1], 'max_absolute_errors': errors, 'rtol': 1e-4, 'atol': 1e-4},
        'scope': 'graph numerical equivalence; not detector accuracy or robot acceptance',
    }
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
