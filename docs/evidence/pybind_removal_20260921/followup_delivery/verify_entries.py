"""Read-only executable/index audit in the explicitly selected isolated prefixes."""
import hashlib
import json
from pathlib import Path
import subprocess
from ament_index_python.packages import get_package_prefix
from ros2pkg.api import get_executable_paths

EXPECTED = {
    'astribot_s1_navigation_policy_native': ['cmd_vel_body_to_world_cpp', 'costmap_scan_cpp',
        'envelope_coordinator_cpp', 'fixed_envelope_cpp', 'final_protection_cpp'],
    'astribot_trajectory_bridge_native': ['arm_speed_limiter_cpp'],
    'astribot_s1_chassis_effort_drive_native': ['omni_effort_drive_cpp'],
    'astribot_s1_robot_geometry': ['geometry_state'],
    'astribot_s1_dynamics_coupling': ['arm_chassis_speed_coupling_node'],
    'astribot_s1_task_arbiter_native': ['task_arbiter_cpp'],
    'astribot_s1_navigation': ['explore_metrics_recorder_node', 'path_tracking_diagnostics_node'],
    'astribot_s1_navigation_policy': ['policy_controller', 'policy_observer'],
    'astribot_s1_chassis_effort_drive': [],
}
results = []
for package, wanted in EXPECTED.items():
    prefix = Path(get_package_prefix(package)).resolve()
    assert str(prefix).startswith('/tmp/'), (package, prefix)
    executables = [Path(p).resolve() for p in get_executable_paths(package_name=package)]
    assert sorted(p.name for p in executables) == sorted(wanted), (package, executables)
    entries = []
    for path in executables:
        native = package not in ('astribot_s1_navigation', 'astribot_s1_navigation_policy')
        record = dict(path=str(path), native=native, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        if native:
            assert path.read_bytes()[:4] == b'\x7fELF', path
            ldd = subprocess.check_output(['ldd', str(path)], text=True)
            assert 'not found' not in ldd
            assert not any(token in ldd.lower() for token in ('libpython', 'pybind', '_geometry_native', '_chassis_math_native', '_navigation_math_native'))
            record['ldd'] = ldd
        entries.append(record)
    results.append(dict(package=package, prefix=str(prefix), executables=entries))
print(json.dumps(dict(scope='Ten migrated native entries plus remaining Python policy/diagnostic entries; isolated prefixes only, no shared deployment or whole-project dependency claim', packages=results), indent=2))
