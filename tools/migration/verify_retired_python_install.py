#!/usr/bin/env python3
"""Install three Python packages into a fresh owned prefix and audit old imports.

The geometry prefix is supplied by a separate compatibility-enabled CMake
build. This validator never writes the repository's shared build/install trees.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
RETIRED = {
    'astribot_s1_chassis_effort_drive': ['omni_effort_drive_node'],
    'astribot_s1_navigation': ['arm_reach_metric', 'arm_speed_limiter_node',
                              'cmd_vel_body_to_world_node', 'posture_monitor_policy'],
    'astribot_s1_navigation_policy': ['cmd_vel_math', 'costmap_scan_node', 'envelope_node',
        'fixed_envelope', 'fixed_envelope_node', 'protection_node', 'control_time', 'scan_occupancy'],
    'astribot_s1_robot_geometry': ['model', 'node', 'projection_contract', 'state'],
}
PROBE = '''
import importlib, importlib.util, json, pathlib, sys
configuration = json.loads(sys.argv[1])
report = []
for package, modules in configuration['retired'].items():
    loaded = importlib.import_module(package)
    location = pathlib.Path(loaded.__file__).resolve()
    assert location.is_relative_to(configuration['expected'][package]), str(location)
    assert all('python_reference' not in p for p in loaded.__path__)
    missing = [m for m in modules if importlib.util.find_spec(package + '.' + m) is None]
    assert missing == modules, (package, missing, modules)
    report.append(dict(package=package, loaded_file=str(location), missing_modules=missing))
for package, module in [('astribot_s1_robot_geometry', 'polygon'),
                        ('astribot_s1_navigation_policy', 'protection')]:
    assert importlib.util.find_spec(package + '.' + module) is not None
print(json.dumps(report))
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work', type=Path, required=True)
    parser.add_argument('--geometry-prefix', type=Path, required=True)
    args = parser.parse_args()
    work = args.work.resolve()
    if work.exists():
        parser.error('--work must be a fresh directory')
    work.mkdir(parents=True)
    prefix = work / 'install'
    records = []
    for package in list(RETIRED)[:3]:
        temp = work / package
        (temp / 'egg').mkdir(parents=True)
        command = [sys.executable, 'setup.py', 'egg_info', '--egg-base', str(temp / 'egg'),
                   'build', '--build-base', str(temp / 'build'), 'install', '--prefix', str(prefix),
                   '--single-version-externally-managed', '--record', str(temp / 'installed_files.txt')]
        with (temp / 'install.log').open('w') as log:
            subprocess.run(command, cwd=ROOT / 'ws_robot/src' / package,
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        records.append(dict(package=package, command=command))
    geometry_prefix = args.geometry_prefix.resolve()
    assert (geometry_prefix / 'share/astribot_s1_robot_geometry/package.xml').is_file()
    installed_paths = sorted({str(p.parent.parent) for base in (prefix, geometry_prefix)
                              for p in base.rglob('__init__.py') if p.parent.name in RETIRED})
    assert len(installed_paths) >= 2, installed_paths
    env = dict(os.environ)
    inherited = [p for p in env.get('PYTHONPATH', '').split(os.pathsep)
                 if p and 'python_reference' not in p]
    configurations = {
        'fresh_install': (installed_paths, {p: str(geometry_prefix if p.endswith('robot_geometry')
                                                 else prefix) for p in RETIRED}),
        'production_source': ([str(ROOT / 'ws_robot/src' / p) for p in RETIRED],
                              {p: str(ROOT / 'ws_robot/src' / p) for p in RETIRED}),
    }
    observations = {}
    for mode, (paths, expected) in configurations.items():
        env['PYTHONPATH'] = os.pathsep.join(paths + inherited)
        output = subprocess.check_output([sys.executable, '-c', PROBE,
            json.dumps(dict(retired=RETIRED, expected=expected))], env=env, cwd=work, text=True)
        observations[mode] = json.loads(output)
    report = dict(scope='Fresh normal install and ordinary source import discovery, not shared deployment',
                  work=str(work), geometry_prefix=str(geometry_prefix),
                  install_records=records, observations=observations)
    (work / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
