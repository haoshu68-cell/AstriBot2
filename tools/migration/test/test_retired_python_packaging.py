"""Retired implementations are explicit validation inputs, never runtime modules."""
import ast
from importlib.machinery import PathFinder
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
REFERENCE_ROOT = ROOT / 'tools/migration/python_reference'
RETIRED = {
    'astribot_s1_chassis_effort_drive': ('omni_effort_drive_node',),
    'astribot_s1_navigation': ('arm_reach_metric', 'arm_speed_limiter_node',
                              'cmd_vel_body_to_world_node', 'posture_monitor_policy'),
    'astribot_s1_navigation_policy': ('cmd_vel_math', 'costmap_scan_node', 'envelope_node',
        'fixed_envelope', 'fixed_envelope_node', 'protection_node', 'control_time', 'scan_occupancy'),
    'astribot_s1_robot_geometry': ('model', 'node', 'projection_contract', 'state'),
}


@pytest.mark.parametrize('package,module', [
    (package, module) for package, modules in RETIRED.items() for module in modules])
def test_retired_module_is_only_an_explicit_test_reference(package, module):
    production = ROOT / 'ws_robot/src' / package / package
    assert not (production / (module + '.py')).exists()
    assert PathFinder.find_spec(package + '.' + module, [str(production)]) is None
    reference = REFERENCE_ROOT / package / (module + '.py')
    tree = ast.parse(reference.read_text())
    assert not any(isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and
                   node.name == 'main' for node in tree.body)
    assert not any(isinstance(node, ast.If) and '__name__' in ast.unparse(node.test)
                   for node in tree.body)


def test_live_geometry_and_protection_dependencies_are_not_retired():
    for package, module in [('astribot_s1_robot_geometry', 'polygon'),
                            ('astribot_s1_navigation_policy', 'protection')]:
        production = ROOT / 'ws_robot/src' / package / package / (module + '.py')
        assert production.is_file()
        assert not (REFERENCE_ROOT / package / (module + '.py')).exists()
