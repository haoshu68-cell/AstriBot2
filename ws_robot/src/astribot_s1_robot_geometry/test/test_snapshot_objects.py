"""Compare the static-cell C++ path with the unchanged fusion calculation."""
import copy
import math

import pytest

from astribot_s1_navigation_policy.fusion import ConservativeFusion, _Track
from astribot_s1_navigation_policy.contracts import Stamp
from astribot_s1_robot_geometry._geometry_native import snapshot_objects
from test_fusion_snapshot import (
    _assert_snapshots_equal, _box, _obs, _profile, _track, _vec,
)


def reference(fusion, now, region=None):
    method = getattr(ConservativeFusion, '_snapshot_reference', ConservativeFusion.snapshot)
    return method(fusion, now, region)


def test_mixed_tracks_keep_order_geometry_and_capture_time():
    fusion = ConservativeFusion(_profile())
    fusion.epoch = ('sim', 0)
    for i in range(24):
        name = 'cell-' + str(i)
        geometry = _box((i * .3 - 2., .25, .5), (.1, .1, 1.))
        observation = _obs('scan', name, 1_800_000_000, geometry,
                           velocity_observable=False, spatial_occupancy=True, source_track_id=name)
        fusion.tracks[name] = _Track(name, observation, _vec(0., 0., 0.))
        if i % 6 == 0:
            name = 'moving-' + str(i)
            fusion.tracks[name] = _track(name, 1_800_000_000,
                _box((.3, i * .1, .5), (.3, .4, 1.)), _vec(.3, -.1, 0.))
    before = copy.deepcopy(fusion.tracks)
    original = fusion.tracks
    for stamp in (1_790_000_000, 2_000_000_000, 8_000_000_000):
        for region in (None, (0., 0., 0.), (1., -1., 2.)):
            now = Stamp(stamp, 'sim', 0)
            expected = reference(fusion, now, region)
            actual = snapshot_objects(fusion, now, region, reference)
            _assert_snapshots_equal(expected, actual)
            assert fusion.tracks is original and fusion.tracks == before
            for track in actual.tracks:
                if track.fused_track_id.startswith('cell-'):
                    assert track.geometry is fusion.tracks[track.fused_track_id].observation.geometry


def test_region_boundary_keeps_all_cells_and_prediction_membership():
    fusion = ConservativeFusion(_profile())
    p = fusion.profile
    radius = (p.half_length_m + p.half_width_m + p.clearance_margin_m +
              p.payload_extra_margin_m + math.hypot(.1, .1) / 2)
    for i, offset in enumerate((-1e-10, 0., 1e-10)):
        name = str(i)
        observation = _obs('scan', name, 1_000_000_000,
            _box((radius + offset, 0., .5), (.1, .1, 1.)),
            velocity_observable=False, spatial_occupancy=True, source_track_id=name)
        fusion.tracks[name] = _Track(name, observation, _vec(0., 0., 0.))
    now = Stamp(2_000_000_000, 'sim', 0)
    expected = reference(fusion, now, (0., 0., 0.))
    actual = snapshot_objects(fusion, now, (0., 0., 0.), reference)
    _assert_snapshots_equal(expected, actual)
    assert len(actual.tracks) == 3
    assert [t.prediction_model is not None for t in actual.tracks] == [True, True, False]


def test_epoch_reset_still_clears_source_state():
    fusion = ConservativeFusion(_profile())
    fusion.epoch = ('sim', 0)
    name = 'old'
    observation = _obs('scan', name, 1_000_000_000,
        _box((1., 0., .5), (.1, .1, 1.)), velocity_observable=False, spatial_occupancy=True, source_track_id=name)
    fusion.tracks[name] = _Track(name, observation, _vec(0., 0., 0.))
    other = ConservativeFusion(fusion.profile)
    other.epoch = fusion.epoch
    other.tracks = fusion.tracks.copy()
    now = Stamp(2_000_000_000, 'sim', 1)
    _assert_snapshots_equal(reference(fusion, now),
                           snapshot_objects(other, now, None, reference))
    assert fusion.tracks == other.tracks == {}
    assert other.epoch == ('sim', 1)


def test_original_time_domain_error_is_not_swallowed():
    fusion = ConservativeFusion(_profile())
    name = 'wrong-clock'
    observation = _obs('scan', name, 1_000_000_000,
        _box((1., 0., .5), (.1, .1, 1.)), velocity_observable=False, spatial_occupancy=True, source_track_id=name)
    fusion.tracks[name] = _Track(name, observation, _vec(0., 0., 0.))
    now = Stamp(2_000_000_000, 'different-clock', 0)
    with pytest.raises(ValueError) as expected:
        reference(fusion, now)
    with pytest.raises(type(expected.value)):
        snapshot_objects(fusion, now, None, reference)
