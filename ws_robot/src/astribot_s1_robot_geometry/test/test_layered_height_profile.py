"""Offline shared-profile check; -0.095 m is the current flat-simulation datum."""
from pathlib import Path

import pytest
import yaml

from test_cpp_robot_model import URDF, probe


def test_shared_ground_profile_becomes_ordered_base_layers_without_legacy_change(probe):
    profile_path = Path(__file__).parents[2] / 'astribot_s1_mapping/config/height_slices.yaml'
    profile = yaml.safe_load(profile_path.read_text())['/**']['ros__parameters']
    assert profile['height_edges'] == [0.05, 0.25, 0.68, 1.18, 1.63, 2.30]
    assert len(profile['layer_names']) == 5
    ground_in_base = -0.095
    edges = [height + ground_in_base for height in profile['height_edges']]
    assert edges == pytest.approx([-.045, .155, .585, 1.085, 1.535, 2.205], abs=1e-14)
    request = dict(urdf=URDF, q={'arm': .4}, errors={'arm': .003})
    original = probe(**request, layer_edges=[-1., 4.])
    result = probe(**request, layer_edges=edges)
    assert len(original.pop('slices')) == 1
    layers = result.pop('slices')
    assert result == original
    assert 'layered_slices' not in result
    assert len(layers) == len(profile['layer_names'])
    assert [layer['z_min'] for layer in layers] == edges[:-1]
    assert [layer['z_max'] for layer in layers] == edges[1:]
    assert layers[-1]['footprint'] == []
