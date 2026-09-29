"""Source interface boundary: the upstream producer cannot own Twist output."""
from pathlib import Path
import ast
import xml.etree.ElementTree as ET
import yaml

PACKAGE = Path(__file__).resolve().parents[1]


def test_no_chassis_command_interface_and_preserved_capture():
    source = (PACKAGE / 'src/arm_chassis_speed_coupling_node.cpp').read_text()
    assert 'geometry_msgs::msg::Twist' not in source
    assert 'create_publisher<nav2_msgs::msg::SpeedLimit>' in source
    assert 'message.header.stamp=stamp_' in source
    assert 'stamp_=msg.header.stamp' in source
    assert 'message.speed_limit=valid_?100.*scale_:0.' in source
    assert 'std::chrono::milliseconds(50)' in source
    assert 'degraded_scale' not in source


def test_launch_and_config_only_dedicated_constraint():
    source = (PACKAGE / 'launch/arm_chassis_coupling.launch.py').read_text()
    ast.parse(source)
    assert '/cmd_vel' not in source and 'input_topic' not in source
    config = yaml.safe_load((PACKAGE / 'config/arm_chassis_coupling_params.yaml').read_text())
    params = config['arm_chassis_speed_coupling_node']['ros__parameters']
    assert params['output_topic'] == '/navigation_policy/arm_speed_limit'
    assert 'joint_state_timeout_sec' not in params
    assert 'reach_tf_timeout_sec' not in params
    assert 'input_topic' not in params and 'degraded_scale' not in params
    dependencies = [item.text for item in ET.parse(PACKAGE / 'package.xml').findall('depend')]
    assert 'nav2_msgs' in dependencies
