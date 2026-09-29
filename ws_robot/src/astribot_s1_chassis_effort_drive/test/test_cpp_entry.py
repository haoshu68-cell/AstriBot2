"""Parse real launch actions without starting ROS or Gazebo processes."""
import importlib.util
from pathlib import Path
import runpy

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.events.process import ProcessExited
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

PACKAGE = Path(__file__).resolve().parents[1]
WAREHOUSE = PACKAGE.parent / 'astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py'


def load(path):
    spec = importlib.util.spec_from_file_location('wheel_launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def context_for(description, overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


@pytest.mark.parametrize('radius,sim_time', [('0.08', 'true'), ('0.12', 'false')])
def test_only_cpp_entry_preserves_parameters_and_respawn(radius, sim_time, tmp_path):
    description = load(PACKAGE / 'launch/omni_effort_drive.launch.py')
    arguments = [a.name for a in description.entities if isinstance(a, DeclareLaunchArgument)]
    assert 'node_impl' not in arguments
    context = context_for(description, {
        'params_file': str(tmp_path / 'custom.yaml'), 'wheel_radius': radius,
        'use_sim_time': sim_time, 'cmd_vel_topic': '/test/velocity',
        'joint_states_topic': '/test/feedback', 'effort_command_topic': '/test/efforts',
    })
    nodes = [a for a in description.entities if isinstance(a, Node)]
    assert len(nodes) == 1
    node = nodes[0]
    assert node.node_package == 'astribot_s1_chassis_effort_drive_native'
    assert node.node_executable == 'omni_effort_drive_cpp'
    assert node.condition is None
    # Humble exposes normalized parameter/respawn state through these fields.
    parameters = evaluate_parameters(context, node._Node__parameters)
    assert parameters[0] == tmp_path / 'custom.yaml'
    assert parameters[1] == {
        'cmd_vel_topic': '/test/velocity', 'joint_states_topic': '/test/feedback',
        'effort_command_topic': '/test/efforts', 'wheel_radius': float(radius),
        'use_sim_time': sim_time == 'true',
    }
    assert node._ExecuteLocal__respawn is True
    assert node._ExecuteLocal__respawn_delay == 1.0


def event(action):
    return ProcessExited(action=action, name='test', cmd=[], cwd=None, env=None,
                         pid=1, returncode=0)


def test_warehouse_forwards_parameters_after_controllers_and_honors_enable():
    description = load(WAREHOUSE)
    assert 'effort_drive_node_impl' not in [
        a.name for a in description.entities if isinstance(a, DeclareLaunchArgument)]
    context = context_for(description, {'wheel_radius': '0.12', 'use_sim_time': 'false'})
    handlers = [a.event_handler for a in description.entities if isinstance(a, RegisterEventHandler)]
    # Read event results only: never execute the returned process actions.
    actions = [(h, h.handle(event(None), context)) for h in handlers]
    spawner = next(a for _, children in actions for a in children
                   if isinstance(a, Node) and a.node_executable == 'spawner')
    candidates = [(h, group, include) for h, children in actions for group in children
                  if isinstance(group, GroupAction) for include in group.get_sub_entities()
                  if isinstance(include, IncludeLaunchDescription)
                  and 'omni_effort_drive.launch.py' in perform_substitutions(
                      context, include.launch_description_source._LaunchDescriptionSource__location)]
    assert len(candidates) == 1
    handler, group, include = candidates[0]
    assert handler.matches(event(spawner))
    assert not handler.matches(event(None))
    assert group not in description.entities
    for enabled in ('false', 'true'):
        context.launch_configurations['enable_effort_drive'] = enabled
        assert group.condition.evaluate(context) == (enabled == 'true')
    arguments = {name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
                 for name, value in include.launch_arguments}
    assert set(arguments) == {'wheel_radius', 'use_sim_time', 'params_file'}
    assert arguments['wheel_radius'] == '0.12'
    assert arguments['use_sim_time'] == 'false'
    assert arguments['params_file'].endswith('/config/omni_effort_drive_params.yaml')
    # Resolve the nested launch from the source checkout, rather than a stale install.
    child = load(PACKAGE / 'launch/omni_effort_drive.launch.py')
    child_context = context_for(child, arguments)
    nodes = [a for a in child.entities if isinstance(a, Node)]
    assert len(nodes) == 1 and nodes[0].node_executable == 'omni_effort_drive_cpp'
    parameters = evaluate_parameters(child_context, nodes[0]._Node__parameters)
    assert parameters[1]['wheel_radius'] == .12
    assert parameters[1]['use_sim_time'] is False


def test_distribution_has_no_python_runtime_console_entry(monkeypatch):
    captured = {}
    monkeypatch.setattr('setuptools.setup', lambda **kwargs: captured.update(kwargs))
    runpy.run_path(str(PACKAGE / 'setup.py'))
    assert not captured.get('entry_points', {}).get('console_scripts', [])


def test_python_reference_cannot_start_a_second_runtime(monkeypatch):
    from astribot_s1_chassis_effort_drive import omni_effort_drive_node as reference

    assert not hasattr(reference, 'main')
    monkeypatch.setattr('rclpy.init', lambda *args, **kwargs: pytest.fail(
        'The Python reference must not initialize a ROS runtime'))
    namespace = runpy.run_path(reference.__file__,
        run_name='__main__')
    assert namespace['OmniEffortDriveNode'].__name__ == 'OmniEffortDriveNode'
