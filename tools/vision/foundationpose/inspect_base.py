#!/usr/bin/env python3
"""Read-only container environment/ONNX inspection. Does not initialize CUDA."""
import hashlib
import importlib
import json
from pathlib import Path
import subprocess

record = {'schema': 'astribot.foundationpose.base_environment/1',
          'gpu_algorithm_run': False, 'files': {}, 'modules': {}, 'models': {}}
for name in ['/etc/os-release', '/usr/local/cuda/version.json',
             '/usr/include/x86_64-linux-gnu/NvInferVersion.h',
             '/usr/include/NvInferVersion.h', '/opt/tensorrt/include/NvInferVersion.h']:
    path = Path(name)
    if path.is_file():
        record['files'][name] = path.read_text()
for name in ['tensorrt', 'onnx', 'numpy']:
    try:
        module = importlib.import_module(name)
        record['modules'][name] = {'version': module.__version__, 'path': module.__file__}
    except ImportError as error:
        record['modules'][name] = {'error': str(error)}
packages = subprocess.run(['dpkg-query', '-W', '-f=${Package}\t${Version}\n'],
                          capture_output=True, text=True, check=True).stdout
record['selected_packages'] = [line for line in packages.splitlines()
                               if any(s in line for s in ['nvinfer', 'tensorrt', 'cuda-', 'cublas'])]
record['apt_sources'] = {}
for path in sorted(Path('/etc/apt/sources.list.d').glob('*')):
    if path.is_file() and path.suffix in ['.list', '.sources']:
        record['apt_sources'][str(path)] = path.read_text()
if 'error' not in record['modules']['onnx']:
    import onnx
    for name in ['refine', 'score']:
        path = Path('/models') / f'{name}_model.onnx'
        if not path.is_file():
            continue
        item = {'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
        try:
            model = onnx.load(str(path))
            onnx.checker.check_model(model, full_check=True)
            def tensors(values):
                return [{'name': value.name, 'element_type': value.type.tensor_type.elem_type,
                         'shape': [d.dim_param if d.dim_param else d.dim_value
                                   for d in value.type.tensor_type.shape.dim]} for value in values]
            item.update({'check': 'PASS', 'ir_version': model.ir_version,
                         'opsets': [{'domain': o.domain, 'version': o.version} for o in model.opset_import],
                         'inputs': tensors(model.graph.input), 'outputs': tensors(model.graph.output)})
        except Exception as error:
            item.update({'check': 'FAIL', 'error': str(error)})
        record['models'][name] = item
print(json.dumps(record, indent=2))
