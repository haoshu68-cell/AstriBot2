"""Offline plugin configuration checks; runtime BT checks live in the C++ test."""
from pathlib import Path

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[2] / 'astribot_s1_navigation'


@pytest.mark.parametrize('name', ['nav2_params_mppi.yaml', 'nav2_params_rpp.yaml'])
def test_departure_plugin_selection(name):
    config = yaml.safe_load((PACKAGE / 'config' / name).read_text())
    planner = config['planner_server']['ros__parameters']
    controller = config['controller_server']['ros__parameters']
    assert planner['planner_plugins'] == ['GridBased', 'Departure']
    assert planner['Departure']['plugin'] == 'astribot_s1_navigation_recovery::DeparturePlanner'
    assert controller['controller_plugins'] == ['FollowPath', 'Departure']
    assert controller['Departure']['plugin'] == 'astribot_s1_navigation_recovery::DepartureController'
    assert controller['goal_checker_plugins'] == ['precise_goal_checker']
    assert controller['precise_goal_checker']['plugin'] == 'astribot_s1_path_tracking::ArrivalGoalChecker'
    assert controller['progress_checker_plugin'] == 'progress_checker'
    libraries = config['bt_navigator']['ros__parameters']['plugin_lib_names']
    assert libraries.count('astribot_navigation_recovery_bt_node') == 1
