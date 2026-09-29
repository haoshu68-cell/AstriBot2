"""Validate one owner per endpoint across both geometry modes without starting nodes."""
import importlib.util
from pathlib import Path
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions


def load():
    path=Path(__file__).resolve().parents[1]/'launch/navigation.launch.py'
    spec=importlib.util.spec_from_file_location('native_selection_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    nodes=[];validators=[];original=module.Node;original_opaque=module.OpaqueFunction
    def capture(*args,**kwargs):
        nodes.append(kwargs);return original(*args,**kwargs)
    module.Node=capture
    def capture_opaque(*args,**kwargs):
        function=kwargs['function']
        if function.__name__=='validate_social_stage':validators.append(function)
        return original_opaque(*args,**kwargs)
    module.OpaqueFunction=capture_opaque
    return module,nodes,validators


@pytest.mark.parametrize('mode',('legacy','fixed_v2'))
def test_exactly_one_cpp_runtime_without_compatibility_arguments(mode):
    module,nodes,validators=load();description=module.generate_launch_description();context=LaunchContext()
    for action in description.entities:
        if isinstance(action,DeclareLaunchArgument):
            context.launch_configurations[action.name]=perform_substitutions(context,action.default_value)
    assert not any(name.endswith('node_impl') for name in context.launch_configurations)
    context.launch_configurations.update(navigation_geometry_mode=mode,navigation_policy_stage='p5')
    assert len(validators)==1
    assert validators[0](context)==[]
    def chosen(executables):
        return [n['executable'] for n in nodes if n['executable'] in executables and
                (n.get('condition') is None or n['condition'].evaluate(context))]
    assert chosen(('envelope_coordinator','envelope_coordinator_cpp','fixed_envelope_cpp'))==[
        'fixed_envelope_cpp' if mode=='fixed_v2' else 'envelope_coordinator_cpp']
    assert chosen(('final_protection','final_protection_cpp'))==[]
    assert chosen(('navigation_constraint_cpp',))==['navigation_constraint_cpp']
    assert chosen(('cmd_vel_body_to_world_node','cmd_vel_body_to_world_cpp'))==['cmd_vel_body_to_world_cpp']
    assert chosen(('arm_speed_limiter_node','arm_speed_limiter_cpp'))==['arm_speed_limiter_cpp']
    assert chosen(('geometry_state',))==([] if mode=='legacy' else ['geometry_state'])
    arbiter=next(n for n in nodes if n['executable']=='task_arbiter_cpp')
    assert arbiter['parameters'][0]['navigation_geometry_mode'].evaluate(context)==mode


@pytest.mark.parametrize('coupled', ('true','false'))
def test_velocity_output_has_no_policy_gate(coupled):
    from launch.actions import IncludeLaunchDescription
    module,nodes,_=load();description=module.generate_launch_description();context=LaunchContext()
    for action in description.entities:
        if isinstance(action,DeclareLaunchArgument):
            context.launch_configurations[action.name]=perform_substitutions(context,action.default_value)
    context.launch_configurations.update(navigation_policy_stage='p5',enable_arm_chassis_coupling=coupled)
    conversion=next(n for n in nodes if n['executable']=='cmd_vel_body_to_world_cpp')
    output=conversion['parameters'][0]['output_topic']
    assert output=='/cmd_vel'
    coupling=next(a for a in description.entities if isinstance(a,IncludeLaunchDescription) and
                  dict(a.launch_arguments).get('output_topic')=='/navigation_policy/arm_speed_limit')
    assert 'input_topic' not in dict(coupling.launch_arguments)
    constraint=next(n for n in nodes if n['executable']=='navigation_constraint_cpp')
    assert constraint['parameters'][0]['command_topic']=='/cmd_vel_nav_body_raw'
    assert constraint['parameters'][0]['require_arm_speed_limit'].evaluate(context)==(coupled=='true')


@pytest.mark.parametrize('coupled', ('true','false'))
def test_policy_off_cannot_silently_drop_arm_speed_limits(coupled):
    module,_,validators=load();description=module.generate_launch_description();context=LaunchContext()
    for action in description.entities:
        if isinstance(action,DeclareLaunchArgument):
            context.launch_configurations[action.name]=perform_substitutions(context,action.default_value)
    context.launch_configurations.update(navigation_policy_stage='off',enable_arm_chassis_coupling=coupled)
    if coupled=='true':
        with pytest.raises(ValueError,match='arm speed limits require navigation policy'):
            validators[0](context)
    else:
        assert validators[0](context)==[]


def test_migrated_python_console_entry_points_are_not_installed():
    import ast
    root=Path(__file__).resolve().parents[2]
    for package,removed in {
        'astribot_s1_navigation':{'cmd_vel_body_to_world_node','arm_speed_limiter_node'},
        'astribot_s1_navigation_policy':{'envelope_coordinator','costmap_scan_adapter','final_protection','task_arbiter'},
    }.items():
        tree=ast.parse((root/package/'setup.py').read_text())
        values=[n.value for n in ast.walk(tree) if isinstance(n,ast.Constant) and isinstance(n.value,str)]
        entries={value.split('=',1)[0].strip() for value in values if '=' in value}
        assert not entries & removed


def test_unadapted_motion_behaviors_cannot_bypass_navigation_constraints():
    import xml.etree.ElementTree as ET
    module,nodes,_=load();module.generate_launch_description()
    behavior=next(n for n in nodes if n['executable']=='behavior_server')
    assert behavior['parameters'][-1]['behavior_plugins']==['wait']
    trees=Path(__file__).resolve().parents[1]/'behavior_trees'
    for path in trees.glob('*.xml'):
        tags={element.tag for element in ET.parse(path).iter()}
        assert not tags & {'Spin','BackUp','DriveOnHeading','AssistedTeleop'}


def test_task_arbiter_has_one_native_owner_entry():
    module, nodes, _ = load()
    module.generate_launch_description()
    owners = [node for node in nodes if node['executable'] in ('task_arbiter', 'task_arbiter_cpp')]
    assert len(owners) == 1
    assert owners[0]['package'] == 'astribot_s1_task_arbiter_native'
    assert owners[0]['executable'] == 'task_arbiter_cpp'
    assert owners[0].get('condition') is None
    import xml.etree.ElementTree as ET
    manifest = ET.parse(Path(__file__).resolve().parents[1] / 'package.xml')
    assert 'astribot_s1_task_arbiter_native' in {
        dependency.text for dependency in manifest.getroot().findall('exec_depend')}


@pytest.mark.parametrize('relative', (
    'astribot_s1_navigation/astribot_s1_navigation/cmd_vel_body_to_world_node.py',
    'astribot_s1_navigation/astribot_s1_navigation/arm_speed_limiter_node.py',
    'astribot_s1_navigation_policy/astribot_s1_navigation_policy/envelope_node.py',
    'astribot_s1_navigation_policy/astribot_s1_navigation_policy/protection_node.py',
    'astribot_s1_navigation_policy/astribot_s1_navigation_policy/costmap_scan_node.py',
))
def test_retired_python_reference_has_no_runtime_entry(relative):
    import ast
    from reference_bootstrap import source_or_reference
    tree = ast.parse(source_or_reference(
        Path(__file__).resolve().parents[2] / relative).read_text())
    assert not any(isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and
                   node.name == 'main' for node in tree.body)
    assert not any(isinstance(node, ast.If) and '__name__' in ast.unparse(node.test)
                   for node in tree.body)


@pytest.mark.parametrize('stage', ('off', 'p5'))
def test_costmap_adapter_is_cpp_with_original_range(stage):
    import yaml
    from launch.substitutions import TextSubstitution
    package = Path(__file__).resolve().parents[1]
    module, nodes, _ = load()
    original_share = module.FindPackageShare
    module.FindPackageShare = lambda name: (
        TextSubstitution(text=str(package)) if name == 'astribot_s1_navigation'
        else original_share(name))
    opaque = []
    original = module.OpaqueFunction
    def capture(*args, **kwargs):
        if kwargs['function'].__name__ == 'costmap_scan_adapter':
            opaque.append(kwargs['function'])
        return original(*args, **kwargs)
    module.OpaqueFunction = capture
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    context.launch_configurations.update(navigation_policy_stage=stage, controller='mppi')
    assert len(opaque) == 1
    actions = opaque[0](context)
    adapters = [n for n in nodes if n['executable'] in ('costmap_scan_cpp', 'costmap_scan_adapter')]
    if stage == 'off':
        assert actions == [] and adapters == []
    else:
        assert len(actions) == len(adapters) == 1
        assert adapters[0]['package'] == 'astribot_s1_navigation_policy_native'
        assert adapters[0]['executable'] == 'costmap_scan_cpp'
        params = yaml.safe_load((package / 'config/nav2_params_mppi.yaml').read_text())
        expected = max(params[n][n]['ros__parameters']['obstacle_layer']['scan']['obstacle_max_range']
                       for n in ('local_costmap', 'global_costmap'))
        assert adapters[0]['parameters'][0]['max_marking_range_m'] == expected


def test_full_bringup_exposes_no_implementation_switches():
    import ast
    path = Path(__file__).resolve().parents[1] / 'launch/nav2_full_bringup.launch.py'
    tree = ast.parse(path.read_text())
    assert not any(isinstance(n, ast.Constant) and isinstance(n.value, str) and
                   n.value.endswith('node_impl') for n in ast.walk(tree))
