"""Optional HuNav scene preparation; retain the existing warehouse and physics."""
import copy
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import yaml


def load_scenario(path):
    config = yaml.safe_load(Path(path).read_text())['hunav_loader']['ros__parameters']
    if not isinstance(config.get('enable_empty_observations', False), bool):
        raise ValueError('enable_empty_observations must be boolean')
    names = config['agents']
    if len(names) != len(set(names)):
        raise ValueError('Duplicate scenario names')
    ids = set()
    for name in names:
        if not re.fullmatch(r'social_person_[0-9]+', name):
            raise ValueError('Scenario actors must use social_person_<number> names')
        spec = config[name]
        if spec['id'] in ids:
            raise ValueError('Duplicate person id')
        ids.add(spec['id'])
        values = [spec['radius'], spec['max_vel'], spec['goal_radius'], *spec['init_pose'].values()]
        if not all(math.isfinite(v) for v in values) or spec['radius'] <= 0 or spec['max_vel'] < 0:
            raise ValueError('Invalid scenario geometry or motion')
        if spec['behavior']['configuration'] != 1 or spec['behavior']['type'] != 'Regular':
            raise ValueError('This scene adapter supports explicit Regular behavior only')
        for goal in spec['goals']:
            point = config['global_goals'][goal]
            if not all(math.isfinite(point[k]) for k in ('x', 'y')):
                raise ValueError('Invalid goal')
    return config


def prepare_behavior_trees(scenario_path, template, output):
    config = load_scenario(scenario_path)
    base = config['yaml_base_name']
    if not re.fullmatch(r'[A-Za-z0-9_-]+', base):
        raise ValueError('Invalid behavior tree base name')
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    tree = Path(template).read_text()
    ET.fromstring(tree)
    for name in config['agents']:
        (output / f"{base}__agent_{config[name]['id']}_bt.xml").write_text(tree)
    return str(output)


def agents_from_config(config):
    from geometry_msgs.msg import Pose
    from hunav_msgs.msg import Agents, Agent
    result = Agents(); result.header.frame_id = 'social_sim_world'
    for name in config['agents']:
        spec = config[name]; agent = Agent()
        agent.name = name; agent.id = spec['id']; agent.type = Agent.PERSON
        agent.group_id = spec['group_id']; agent.skin = 5
        agent.radius = float(spec['radius']); agent.desired_velocity = float(spec['max_vel'])
        agent.goal_radius = float(spec['goal_radius']); agent.cyclic_goals = spec['cyclic_goals']
        p = spec['init_pose']; agent.position.position.x = float(p['x']); agent.position.position.y = float(p['y'])
        agent.position.position.z = float(p['z']); agent.yaw = float(p['h'])
        agent.position.orientation.z = math.sin(p['h']/2); agent.position.orientation.w = math.cos(p['h']/2)
        agent.behavior.type = 1; agent.behavior.configuration = 1
        for key in ('goal_force_factor', 'social_force_factor', 'obstacle_force_factor', 'other_force_factor'):
            setattr(agent.behavior, key, float(spec['behavior'][key]))
        for index in spec['goals']:
            p = config['global_goals'][index]; goal = Pose()
            goal.position.x = float(p['x']); goal.position.y = float(p['y']); goal.orientation.w = 1.
            agent.goals.append(goal)
        result.agents.append(agent)
    return result


def prepare_world(base_world, scenario_path, output, wrapper_prefix, bringup_prefix):
    config = load_scenario(scenario_path)
    root = ET.parse(base_world); world = root.getroot().find('world')
    if world is None or world.find('actor') is not None:
        raise ValueError('Expected one warehouse world without preexisting actors')
    if world.find('plugin') is None:
        # Explicit world plugins disable Gazebo's default server.config systems.
        for library, system in (
                ('physics', 'Physics'), ('user-commands', 'UserCommands'),
                ('scene-broadcaster', 'SceneBroadcaster')):
            ET.SubElement(world, 'plugin', filename=f'ignition-gazebo-{library}-system',
                          name=f'ignition::gazebo::systems::{system}')
    monitor_library = Path(bringup_prefix)/'lib/libastribot_social_scene.so'
    required = [monitor_library]
    with_hunav = bool(config['agents']) or config.get('enable_empty_observations', False)
    if with_hunav:
        if wrapper_prefix is None:
            raise FileNotFoundError('HuNav wrapper is required for observed social scenarios')
        library = Path(wrapper_prefix)/'lib/libHuNavSystemPluginIGN.so'
        required.append(library)
        if config['agents']:
            mesh = Path(wrapper_prefix)/'share/hunav_gazebo_fortress_wrapper/worlds/models/walk.dae'
            required.append(mesh)
    for path in required:
        if not path.is_file():
            raise FileNotFoundError(path)
    def element(parent, tag, text):
        node = ET.SubElement(parent, tag); node.text = str(text); return node
    if with_hunav:
        plugin = ET.SubElement(world, 'plugin', filename=str(library), name='HuNavSystemPluginIGN')
        for name, value in {'robot_name': 'astribot_s1', 'global_frame_to_publish': 'social_sim_world',
                        'use_navgoal_to_start': str(config.get('wait_for_episode_start', False)).lower(), 'update_rate': 20,
                        'use_gazebo_obs': 'false'}.items():
            element(plugin, name, value)
        ignored = ET.SubElement(plugin, 'ignore_models')
    monitor = ET.SubElement(world, 'plugin', filename=str(monitor_library), name='astribot::SocialScene')
    element(monitor, 'robot_name', 'astribot_s1')
    for name in config['agents']:
        spec = config[name]; p = spec['init_pose']; radius = spec['radius']; height = 1.7
        actor = ET.SubElement(world, 'actor', name=name)
        # The wrapper supplies an absolute world trajectory; do not apply it twice.
        element(actor, 'pose', '0 0 0 0 0 0')
        skin = ET.SubElement(actor, 'skin'); element(skin, 'filename', mesh); element(skin, 'scale', 1)
        animation = ET.SubElement(actor, 'animation', name='walk')
        element(animation, 'filename', mesh); element(animation, 'scale', 1); element(animation, 'interpolate_x', 'true')
        proxy = ET.SubElement(world, 'model', name=name+'_collision')
        element(proxy, 'static', 'true'); element(proxy, 'pose', f"{p['x']} {p['y']} 0 0 0 0")
        link = ET.SubElement(proxy, 'link', name='body')
        element(link, 'pose', f'0 0 {height/2} 0 0 0')
        for kind in ('collision', 'visual'):
            body = ET.SubElement(link, kind, name='body_'+kind)
            cylinder = ET.SubElement(ET.SubElement(body, 'geometry'), 'cylinder')
            element(cylinder, 'radius', radius); element(cylinder, 'length', height)
            if kind == 'visual':
                material = ET.SubElement(body, 'material')
                element(material, 'ambient', '0.2 0.6 0.8 1'); element(material, 'diffuse', '0.2 0.6 0.8 1')
        pair = ET.SubElement(monitor, 'person')
        element(pair, 'name', name); element(pair, 'radius', radius); element(pair, 'height', height)
        element(ignored, 'model', name+'_collision')
    output = Path(output); output.parent.mkdir(parents=True, exist_ok=True)
    root.write(output, encoding='utf-8', xml_declaration=True)
    return str(output)


def prepare_gui_config(scenario_path, output, bringup_prefix):
    """Copy the Fortress GUI layout and hide sensor proxies in that client only."""
    config = load_scenario(scenario_path)
    source = Path.home() / '.ignition/gazebo/6/gui.config'
    if not source.is_file():
        source = Path('/usr/share/ignition/ignition-gazebo6/gui/gui.config')
    library = Path(bringup_prefix) / 'lib/libSocialProxyDisplay.so'
    if not library.is_file():
        raise FileNotFoundError(library)
    plugin = ET.Element('plugin', filename='SocialProxyDisplay', name='Social proxy display')
    gui = ET.SubElement(plugin, 'ignition-gui')
    for key, kind, value in [('state', 'string', 'floating'), ('showTitleBar', 'bool', 'false'),
                             ('width', 'double', '1'), ('height', 'double', '1')]:
        ET.SubElement(gui, 'property', key=key, type=kind).text = value
    for name in config['agents']:
        ET.SubElement(plugin, 'proxy_model').text = name + '_collision'
    # Gazebo's config accepts multiple top-level XML elements.
    output = Path(output); output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(source.read_text() + '\n' + ET.tostring(plugin, encoding='unicode') + '\n')
    return str(output)


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from hunav_msgs.srv import GetAgents
    rclpy.init(args=args)
    node = Node('social_scenario_provider')
    if not node.get_parameter('use_sim_time').value:
        raise ValueError('Scenario provider is simulation-only')
    node.declare_parameter('scenario', '')
    agents = agents_from_config(load_scenario(node.get_parameter('scenario').value))
    def get_agents(request, response):
        response.agents = copy.deepcopy(agents)
        response.agents.header.stamp = node.get_clock().now().to_msg()
        return response
    node.create_service(GetAgents, '/get_agents', get_agents)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
