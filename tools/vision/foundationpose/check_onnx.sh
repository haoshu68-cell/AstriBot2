#!/usr/bin/env bash
# CPU-only inspection in a disposable container; no GPU flag is needed.
set -euo pipefail
test ! -e /output/base_and_models.json
python3 -m pip install --disable-pip-version-check --no-cache-dir \
  --only-binary=:all: --no-deps --target /tmp/p0-onnx \
  --report /output/checker_packages.json onnx==1.16.0 protobuf==4.25.3 \
  > /output/checker_install.log 2>&1
PYTHONPATH=/tmp/p0-onnx python3 -B /tools/inspect_base.py > /output/base_and_models.json
python3 - <<'PY'
import json
from pathlib import Path
r = json.loads(Path('/output/base_and_models.json').read_text())
expected = {'refine', 'score'}
assert set(r['models']) == expected, 'Both models must actually be checked'
for name, model in r['models'].items():
    assert model['check'] == 'PASS', (name, model)
    assert [x['name'] for x in model['inputs']] == ['input1', 'input2']
    assert all(x['element_type'] == 1 and x['shape'][1:] == [160, 160, 6]
               for x in model['inputs']), model['inputs']
print('Two official ONNX models: semantic and input-contract checks PASS; inference NOT_RUN')
PY
