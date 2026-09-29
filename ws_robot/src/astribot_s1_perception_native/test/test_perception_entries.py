"""Migration boundary: each retired role has one selected native owner."""
import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[2]
PERCEPTION = ROOT / 'astribot_s1_perception'
NATIVE = ROOT / 'astribot_s1_perception_native'


@pytest.mark.parametrize('executable,launch,modules', [
    ('map_domain_relay', 'map_provider.launch.py', ['map_domain_relay']),
    ('map_odom_tf_node', 'voxel_slam.launch.py', ['map_odom_tf_node', 'map_odom_decompose']),
])
def test_only_native_entry_selected(executable, launch, modules):
    # These are explicit retirement/selection requirements, not a claim that
    # source text proves ROS behavior. Independent protocol replays cover that.
    tree = ast.parse((PERCEPTION / 'launch' / launch).read_text())
    selected = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        args = {kw.arg: kw.value.value for kw in node.keywords if isinstance(kw.value, ast.Constant)}
        if args.get('executable') == executable:
            selected.append(args.get('package'))
    assert selected == ['astribot_s1_perception_native']
    setup = ast.parse((PERCEPTION / 'setup.py').read_text())
    console_strings = [node.value for node in ast.walk(setup) if isinstance(node, ast.Constant)
                       and isinstance(node.value, str) and ' = ' in node.value]
    assert not any(value.split(' = ')[0] == executable for value in console_strings)
    for module in modules:
        assert not (PERCEPTION / 'astribot_s1_perception' / (module + '.py')).exists()
        assert (NATIVE / 'test/reference/astribot_s1_perception' / (module + '.py')).is_file()
    manifest = ET.parse(PERCEPTION / 'package.xml').getroot()
    assert 'astribot_s1_perception_native' in [el.text for el in manifest.findall('exec_depend')]
