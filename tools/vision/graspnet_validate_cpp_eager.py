#!/usr/bin/env python3
"""Compare the C++ worker with saved original and changed-input eager inference."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--device', choices=['cpu', 'cuda'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for name, cloud, expected_path in [
        ('original', args.input, Path(str(args.model) + '.eager.bin')),
        ('changed', Path(str(args.model) + '.changed.cloud.bin'),
         Path(str(args.model) + '.changed.eager.bin')),
    ]:
        output = args.output / (name + '.grasps.bin')
        output.unlink(missing_ok=True)
        command = [str(args.worker.resolve()), '--model', str(args.model.resolve()),
                   '--input', str(cloud.resolve()), '--output', str(output.resolve()),
                   '--device', args.device]
        completed = subprocess.run(command, capture_output=True, text=True, timeout=120)
        (args.output / (name + '.stdout.log')).write_text(completed.stdout)
        (args.output / (name + '.stderr.log')).write_text(completed.stderr)
        if completed.returncode != 0:
            raise RuntimeError(f'{name}: worker exited {completed.returncode}: {completed.stderr}')
        actual = np.fromfile(output, dtype='<f4').reshape(-1, 17)
        expected = np.fromfile(expected_path, dtype='<f4').reshape(-1, 17)
        assert len(expected) and np.isfinite(actual).all(), 'empty or invalid comparison'
        np.testing.assert_allclose(actual, expected, rtol=2e-5, atol=2e-6)
        results.append(dict(case=name,device=args.device,input_sha256=sha(cloud),
                            cpp_output_sha256=sha(output),eager_output_sha256=sha(expected_path),
                            candidates=len(actual),cpp_eager_max_abs_error=float(np.max(np.abs(actual-expected))),
                            worker_report=json.loads(completed.stdout)))
    report = dict(model_sha256=sha(args.model),worker_sha256=sha(args.worker),results=results,
                  tolerances=dict(rtol=2e-5,atol=2e-6),passed=True)
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps([{k:v for k,v in row.items() if k != 'worker_report'} for row in results],indent=2))


if __name__ == '__main__':
    main()
