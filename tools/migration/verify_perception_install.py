#!/usr/bin/env python3
"""Fresh package install audit; never touch the shared ROS install or start nodes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--native-prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix='codex_perception_packaging_'))
    source = work / 'source'
    shutil.copytree(args.root / 'ws_robot/src/astribot_s1_perception', source,
                    ignore=shutil.ignore_patterns('__pycache__', 'build', 'dist', '*.egg-info'))
    prefix = work / 'install'
    command = [sys.executable, 'setup.py', 'install', '--prefix', str(prefix),
               '--single-version-externally-managed', '--record', str(work / 'install.txt')]
    with (args.output / 'python_install.log').open('w') as log:
        subprocess.run(command, cwd=source, stdout=log, stderr=subprocess.STDOUT, check=True)
    site = next(prefix.glob('lib/python*/site-packages'))
    executable_dir = prefix / 'lib/astribot_s1_perception'
    executables = sorted(p.name for p in executable_dir.iterdir())
    assert executables == ['autonomous_patrol_node', 'map_start_cell_check', 'slam_session'], executables
    code = '''import importlib.util,json,os
import astribot_s1_perception as package
assert os.path.realpath(package.__file__).startswith(os.environ['EXPECTED_PREFIX']+'/')
retired=['map_domain_relay','map_odom_tf_node','map_odom_decompose']
for name in retired:
    assert importlib.util.find_spec('astribot_s1_perception.'+name) is None,name
for name in ['slam_session','autonomous_patrol_node','map_source_config','map_start_cell_check']:
    assert importlib.util.find_spec('astribot_s1_perception.'+name) is not None,name
print(json.dumps({'package_path':package.__file__,'retired_absent':retired}))
'''
    env = dict(os.environ, PYTHONPATH=str(site), EXPECTED_PREFIX=str(prefix))
    imports = json.loads(subprocess.check_output([sys.executable, '-c', code], cwd=work, env=env, text=True))
    native_dir = args.native_prefix / 'lib/astribot_s1_perception_native'
    native_names = sorted(p.name for p in native_dir.iterdir())
    assert native_names == ['map_domain_relay', 'map_odom_tf_node'], native_names
    native = []
    for name in native_names:
        path = native_dir / name
        raw = path.read_bytes()
        assert raw[:4] == b'\x7fELF', path
        linked = subprocess.check_output(['ldd', str(path)], text=True)
        assert 'not found' not in linked, linked
        assert not any(token in linked.lower() for token in ('libpython', 'pybind', '_native.cpython')), linked
        native.append({'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest(), 'ldd': linked})
    assert not list(args.native_prefix.rglob('*.py')), 'native package must not install test oracles'
    result = {'fresh_work': str(work), 'python_executables': executables,
              'python_import_check': imports, 'native_executables': native,
              'scope': 'Clean independent install/import/ELF audit, not deployment or whole-stack acceptance'}
    (args.output / 'install_audit.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
