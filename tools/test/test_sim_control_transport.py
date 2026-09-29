import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
from sim_stack_supervisor import control_environment


def test_pre_navigation_transport_is_udp_and_loopback(tmp_path):
    original = {'ROS_DOMAIN_ID': '89', 'ROS_LOCALHOST_ONLY': '1'}
    profile = tmp_path/'control.xml'
    result = control_environment(original, 'udp', profile)
    assert original['ROS_LOCALHOST_ONLY'] == '1'
    assert result['ROS_LOCALHOST_ONLY'] == '0'
    assert result['ROS_DOMAIN_ID'] == '89'
    assert result['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
    tree = ET.parse(profile)
    ns = {'f': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    assert [n.text for n in tree.findall('.//f:interfaceWhiteList/f:address', ns)] == ['127.0.0.1']
    assert [n.text for n in tree.findall('.//f:useBuiltinTransports', ns)] == ['false']


def test_default_transport_preserves_caller(tmp_path):
    env = {'ROS_LOCALHOST_ONLY': '1', 'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'}
    path = tmp_path/'unused.xml'
    assert control_environment(env, 'default', path) == env
    assert not path.exists()


def test_warehouse_control_override_preserves_camera_and_lan_transport():
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location('warehouse',
        ROOT/'ws_robot/src/astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(localhost_only='true', control_loopback_udp='true',
                                          camera_bridge_dds_profile='')
    for action in module._local_control_environment(context):
        action.execute(context)
    assert context.environment['ROS_LOCALHOST_ONLY'] == '0'
    assert context.environment['FASTRTPS_DEFAULT_PROFILES_FILE'].endswith('control_loopback_fastdds.xml')
    camera = module._camera_bridge_environment(context)
    assert camera['FASTRTPS_DEFAULT_PROFILES_FILE'].endswith('camera_bridge_fastdds.xml')
    context.launch_configurations['localhost_only'] = 'false'
    assert module._local_control_environment(context) == []
    assert module._camera_bridge_environment(context) == {}
