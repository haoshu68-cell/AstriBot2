"""The oracle is never registered as a production runtime fallback."""
from pathlib import Path
import hashlib

ROOT = Path(__file__).resolve().parents[2]


def test_no_production_python_entry():
    policy = ROOT / 'astribot_s1_navigation_policy'
    assert not (policy / 'astribot_s1_navigation_policy/task_arbiter_node.py').exists()
    assert 'task_arbiter_node' not in (policy / 'setup.py').read_text()
    assert 'task_arbiter =' not in (policy / 'setup.py').read_text()
    # setup find_packages() must not install the frozen executable oracle.
    assert not (policy / 'test/__init__.py').exists()
    assert not (policy / 'test/reference/__init__.py').exists()


def test_oracle_is_frozen():
    oracle = ROOT / 'astribot_s1_navigation_policy/test/reference/task_arbiter_node.py'
    assert hashlib.sha256(oracle.read_bytes()).hexdigest() == 'c898f9d0cc459d3cf6b6b28d9199fbcb5c2c367a669e72fb0b11e98c302090a3'


def test_native_dependency_and_install():
    native = ROOT / 'astribot_s1_task_arbiter_native'
    cmake = (native / 'CMakeLists.txt').read_text()
    assert 'install(TARGETS task_arbiter_cpp DESTINATION lib/${PROJECT_NAME})' in cmake
    for dependency in ['rclcpp', 'rclcpp_action', 'nav2_msgs', 'astribot_navigation_msgs']:
        assert f'<depend>{dependency}</depend>' in (native / 'package.xml').read_text()
    assert 'pybind' not in cmake and 'Python' not in cmake
