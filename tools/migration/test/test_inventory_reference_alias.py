"""Frozen oracles can live in a new native package under their original namespace."""
import importlib.util
from pathlib import Path

TOOL = Path(__file__).resolve().parents[1] / 'inventory_python_runtime.py'
SPEC = importlib.util.spec_from_file_location('inventory_reference_alias', TOOL)
INVENTORY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(INVENTORY)


def test_native_package_oracle_retains_original_import_identity():
    path = Path('ws_robot/src/astribot_s1_perception_native/test/reference/astribot_s1_perception/map_odom_decompose.py')
    assert INVENTORY.module_name(path) == 'astribot_s1_perception.map_odom_decompose'


def test_flat_and_same_package_reference_identities_remain_supported():
    paths = {
        'ws_robot/src/astribot_s1_navigation_policy/test/reference/task_arbiter_node.py':
            'astribot_s1_navigation_policy.task_arbiter_node',
        'ws_robot/src/astribot_s1_dynamics_coupling/test/reference/astribot_s1_dynamics_coupling/arm_reach_metric.py':
            'astribot_s1_dynamics_coupling.arm_reach_metric',
        'tools/migration/python_reference/astribot_s1_robot_geometry/model.py':
            'astribot_s1_robot_geometry.model',
    }
    for path, expected in paths.items():
        assert INVENTORY.module_name(Path(path)) == expected


def test_oracle_cannot_hide_accidentally_restored_production_module():
    rows = {
        'ws_robot/src/old_pkg/old_pkg/node.py': {'module': 'old_pkg.node', 'category': 'candidate'},
        'ws_robot/src/native_pkg/test/reference/old_pkg/node.py':
            {'module': 'old_pkg.node', 'category': 'cpp_validation_reference'},
    }
    indexed = INVENTORY.index_modules(rows)
    assert indexed['old_pkg.node'] == 'ws_robot/src/old_pkg/old_pkg/node.py'
    assert INVENTORY.index_modules(dict(reversed(list(rows.items())))) == indexed
