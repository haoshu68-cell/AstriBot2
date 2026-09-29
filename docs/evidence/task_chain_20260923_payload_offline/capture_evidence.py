#!/usr/bin/env python3
"""Collect existing offline evidence only; never start nodes or simulations."""
from pathlib import Path
import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET
from datetime import datetime
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[3]
RUN = ROOT / 'runs/task_chain_20260923_payload_offline'
OUT = Path(__file__).resolve().parent


def write(name, value):
    (OUT / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def keep(path):
    return path.is_file() and not any(part in {'__pycache__', '.pytest_cache', 'log'} for part in path.parts)


def main():
    now = datetime.now(ZoneInfo('Asia/Shanghai')).isoformat()
    suites = []
    for component in ('payload', 'geometry', 'hold', 'navigation'):
        for path in sorted((RUN / (component + '_build') / 'test_results').rglob('*.gtest.xml')):
            root = ET.parse(path).getroot()
            counts = {key: int(root.attrib.get(key, 0)) for key in ('tests', 'failures', 'errors', 'disabled')}
            cases = [{'name': c.attrib['name'], 'suite': c.attrib.get('classname'),
                      'skipped': c.find('skipped') is not None} for c in root.iter('testcase')]
            assert not any(counts[k] for k in ('failures', 'errors', 'disabled'))
            assert not any(c['skipped'] for c in cases)
            suites.append(dict(component=component, file=str(path), sha256=sha(path), **counts, cases=cases))
    assert sum(s['tests'] for s in suites) == 91
    logs = ['final_payload_tests.log', 'final_geometry_tests.log', 'final_hold_tests.log', 'chain_tests.log']
    for name in logs:
        assert '100% tests passed, 0 tests failed' in (RUN / name).read_text()
    assert '26 passed' in (RUN / 'final_geometry_tests.log').read_text()
    assert '5 passed' in (RUN / 'final_launch_tests.log').read_text()
    write('test_results.json', dict(
        recorded_at=now, evidence_class='offline_native_and_static_launch_only',
        gtest_cases=91, failures=0, errors=0, skipped=0, suites=suites,
        native_executables=['lease_time', 'filled_collision', 'fusion_snapshot', 'geometry_state_core', 'fixed_envelope_authority'],
        geometry_differential_pytest_cases=26, launch_parameter_pytest_cases=5,
        final_logs=[str(RUN / name) for name in logs + ['final_launch_tests.log']],
        not_run=['ROS protocol tests', 'Gazebo', 'ROS attachment source and scene service',
                 'real resource/executor/hold publisher', 'motion and camera workload matrix', 'VLA', 'hardware'],
        count_note='CTest wrapper suites are not added to GTest/native/pytest case counts.'))

    paths = set()
    for package in ('astribot_payload_msgs', 'astribot_s1_payload_state', 'astribot_s1_transport_native'):
        paths.update(p.relative_to(ROOT) for p in (ROOT / 'ws_robot/src' / package).rglob('*') if keep(p))
    baseline = RUN / 'baseline'
    changed = {}
    for old in baseline.rglob('*'):
        if not keep(old):
            continue
        relative = old.relative_to(baseline)
        current = ROOT / relative
        if current.is_file() and sha(old) != sha(current):
            paths.add(relative)
            changed[str(relative)] = dict(baseline=str(old), baseline_sha256=sha(old), baseline_kind='pre-edit_snapshot')
    for relative in (
        'ws_robot/src/astribot_s1_robot_geometry/include/astribot_s1_robot_geometry/attachment_geometry.hpp',
        'ws_robot/src/astribot_s1_robot_geometry/test/attachment_geometry_test.cpp',
        'ws_robot/src/astribot_s1_navigation_policy_native/test/fixed_hold_flow_test.cpp',
        'ws_robot/src/astribot_s1_navigation_policy_native/src/fixed_envelope_node.cpp',
    ):
        paths.add(Path(relative))
    node = Path('ws_robot/src/astribot_s1_navigation_policy_native/src/fixed_envelope_node.cpp')
    changed[str(node)] = dict(baseline_kind='observed_callback_bodies_before_edit',
                             change='added tick() after hold and ACK callbacks; no pre-edit file snapshot')
    source = {str(path): dict(sha256=sha(ROOT / path), **changed.get(str(path), {'baseline_kind': 'new_file'}))
              for path in sorted(paths)}
    patch = []
    for path in sorted(paths):
        old = baseline / path
        if not old.exists() and path == node:
            continue  # Do not mislabel an existing untracked file as newly implemented.
        result = subprocess.run(['git', 'diff', '--no-index', '--', str(old) if old.exists() else '/dev/null', str(ROOT / path)],
                                text=True, capture_output=True, check=False)
        assert result.returncode in (0, 1), result.stderr
        patch.append(result.stdout)
    (RUN / 'scoped_candidate.patch').write_text(''.join(patch))
    check = subprocess.run(['git', 'diff', '--check', '--'] + [str(p) for p in sorted(paths)],
                           cwd=ROOT, text=True, capture_output=True, check=False)
    assert check.returncode == 0, check.stdout + check.stderr
    whitespace = []
    for path in sorted(paths):
        for line, content in enumerate((ROOT / path).read_text().splitlines(), 1):
            if content.rstrip() != content:
                whitespace.append(f'{path}:{line}')
    assert not whitespace, whitespace
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    write('source_manifest.json', dict(recorded_at=now, base_commit=commit,
        shared_worktree_dirty=True, shared_install_overwritten=False, files=source,
        patch=str(RUN / 'scoped_candidate.patch'), diff_check='passed', whitespace_check='passed'))

    builds = {}
    for component in ('payload', 'geometry', 'hold', 'navigation'):
        directory = RUN / (component + '_build')
        config = {}
        for line in (directory / 'CMakeCache.txt').read_text().splitlines():
            if line.startswith(('CMAKE_BUILD_TYPE:', 'CMAKE_CXX_COMPILER:', 'CMAKE_CXX_FLAGS', 'CMAKE_INSTALL_PREFIX:',
                                'astribot_', 'rclcpp_DIR:', 'moveit_msgs_DIR:', 'controller_manager_msgs_DIR:')) and '=' in line:
                key, value = line.split('=', 1)
                config[key] = value
        builds[component] = dict(directory=str(directory), cache_sha256=sha(directory / 'CMakeCache.txt'), selected_cache=config)
    write('build_context.json', dict(recorded_at=now, builds=builds,
        compiled_not_started=['payload_state', 'geometry_state', 'fixed_envelope_cpp'],
        native_library='arm_hold_core', replay_script=str(OUT / 'verify_offline.sh')))
    print(json.dumps(dict(gtest_cases=91, native_executables=5, pytest_cases=31,
                          source_files=len(source), diff_check='passed'), ensure_ascii=False))


if __name__ == '__main__':
    main()
