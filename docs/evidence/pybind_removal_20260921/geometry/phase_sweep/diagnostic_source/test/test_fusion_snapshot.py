"""Differential tests: ConservativeFusion.snapshot_native vs the pure-Python
snapshot() reference, plus the boundary cases the kernel must reject.
"""
import math
import random

import numpy as np
import pytest

from astribot_s1_navigation_policy.fusion import ConservativeFusion, _Track
from astribot_s1_navigation_policy.profile import Profile
from astribot_s1_navigation_policy.contracts import (
    Covariance3, MetricBox, Observation, Stamp, Vec3,
)
from astribot_s1_robot_geometry._geometry_native import snapshot_tracks as _kernel


def _profile():
    from pathlib import Path
    return Profile.load(str(Path(__file__).resolve().parents[2]
                              / 'astribot_s1_navigation_policy/config/simulation.json'))


def _vec(x, y, z):
    return Vec3(x, y, z)


def _box(center, size, cov=(0.,) * 9, velocity=None, velocity_cov=None):
    return MetricBox(_vec(*center), _vec(*size), Covariance3(tuple(cov)),
                     _vec(*velocity) if velocity else None,
                     Covariance3(tuple(velocity_cov)) if velocity_cov else None)


def _obs(sensor, identifier, capture_ns, geometry, *, velocity_observable=True,
         spatial_occupancy=False, source_track_id=None, provenance=None):
    clock = 'sim'
    return Observation(
        sensor, identifier, source_track_id,
        Stamp(capture_ns, clock, 0), Stamp(capture_ns, 'steady', 0),
        Stamp(capture_ns + 300_000_000, clock, 0),
        'odom', 0, geometry, 1., (),
        (provenance if provenance is not None else (identifier,)),
        velocity_observable=velocity_observable, spatial_occupancy=spatial_occupancy)


def _track(identifier, capture_ns, geometry, velocity, samples=()):
    return _Track(identifier, _obs('scan', identifier, capture_ns, geometry,
                                   provenance=(identifier,)), velocity, samples)


def _isclose(a, b):
    if isinstance(a, float) or isinstance(b, float):
        return math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-12)
    return a == b


def _assert_geometry_equal(a, b):
    for k in ('x', 'y', 'z'):
        assert _isclose(getattr(a.center_m, k), getattr(b.center_m, k)), 'center'
        assert _isclose(getattr(a.size_m, k), getattr(b.size_m, k)), 'size'
    for i in range(9):
        assert _isclose(a.position_covariance_m2.values[i],
                        b.position_covariance_m2.values[i]), 'covariance'
    assert (a.velocity_m_s is None) == (b.velocity_m_s is None)
    if a.velocity_m_s is not None:
        for k in ('x', 'y', 'z'):
            assert _isclose(getattr(a.velocity_m_s, k), getattr(b.velocity_m_s, k))
    assert (a.velocity_covariance_m2_s2 is None) == (b.velocity_covariance_m2_s2 is None)
    if a.velocity_covariance_m2_s2 is not None:
        for i in range(9):
            assert _isclose(a.velocity_covariance_m2_s2.values[i],
                            b.velocity_covariance_m2_s2.values[i])


def _assert_snapshots_equal(a, b):
    assert a.version == b.version
    assert a.stamp == b.stamp
    assert a.frame_id == b.frame_id
    assert a.observation_seq == b.observation_seq
    assert len(a.tracks) == len(b.tracks)
    assert a.unassociated == b.unassociated
    assert a.sensors == b.sensors
    for ta, tb in zip(a.tracks, b.tracks):
        assert ta.fused_track_id == tb.fused_track_id
        assert ta.frame_id == tb.frame_id
        assert ta.stamp == tb.stamp
        assert ta.provenance == tb.provenance
        assert ta.predictions == tb.predictions == ()
        _assert_geometry_equal(ta.geometry, tb.geometry)
        assert (ta.prediction_model is None) == (tb.prediction_model is None)
        if ta.prediction_model is not None:
            ma, mb = ta.prediction_model, tb.prediction_model
            for k in ('x', 'y', 'z'):
                assert _isclose(getattr(ma.velocity, k), getattr(mb.velocity, k))
            assert _isclose(ma.variance_m2_s2, mb.variance_m2_s2)
            assert ma.steps == mb.steps


def _assert_matches(fusion, now, region=None):
    _assert_snapshots_equal(fusion.snapshot(now, region), fusion.snapshot_native(now, region))


def test_snapshot_native_matches_reference():
    profile = _profile()
    fusion = ConservativeFusion(profile, frame_id='odom')
    now = Stamp(2_000_000_000, 'sim', 0)
    # Stationary: enough samples, sub-threshold speed.
    fusion.tracks['stationary'] = _track(
        'stationary', 1_500_000_000, _box((0.4, 0.2, 0.5), (0.4, 0.4, 1.0)),
        _vec(0.02, 0.01, 0.0),
        ((0.0, 0.0, 0.0), (0.1, 0.01, 0.01), (0.2, -0.01, 0.01),
         (0.3, 0.0, 0.0), (0.4, 0.01, -0.01), (0.5, 0.0, 0.0)))
    # Moving: super-threshold speed.
    fusion.tracks['moving'] = _track(
        'moving', 1_500_000_000, _box((0.0, 0.0, 0.5), (0.3, 0.3, 1.0)),
        _vec(1.0, 0.5, 0.0),
        ((0.0, 0.0, 0.0), (0.1, 0.5, 0.25), (0.2, 1.0, 0.5),
         (0.3, 1.5, 0.75), (0.4, 2.0, 1.0), (0.5, 2.5, 1.25)))
    # Direct measured velocity + covariance.
    fusion.tracks['direct'] = _track(
        'direct', 1_600_000_000, _box((1.0, 1.0, 0.5), (0.2, 0.2, 1.0),
                                      (0.01, 0., 0., 0., 0.02, 0., 0., 0., 0.03),
                                      velocity=(0.3, 0.0, 0.0),
                                      velocity_cov=(0.09, 0., 0., 0., 0.09, 0., 0., 0., 0.09)),
        _vec(0.3, 0.0, 0.0))
    # Occupied cell: never moved, zero variance.
    fusion.tracks['occupied'] = _Track(
        'occupied',
        _obs('scan', 'occupied', 1_500_000_000, _box((2.0, 2.0, 0.5), (0.1, 0.1, 1.0)),
             velocity_observable=False, spatial_occupancy=True,
             source_track_id='occupied'),
        _vec(0.0, 0.0, 0.0))
    # Non-observable, no velocity -> occupancy_only.
    fusion.tracks['occ_only'] = _track(
        'occ_only', 1_500_000_000, _box((-1.0, -1.0, 0.5), (0.4, 0.4, 1.0)),
        _vec(0.0, 0.0, 0.0))
    fusion.tracks['occ_only'].observation = _obs(
        'scan', 'occ_only', 1_500_000_000, _box((-1.0, -1.0, 0.5), (0.4, 0.4, 1.0)),
        velocity_observable=False, provenance=('occ_only',))
    # Old track: age > track_memory.
    fusion.tracks['old'] = _track(
        'old', 500_000_000, _box((3.0, 3.0, 0.5), (0.3, 0.3, 1.0)),
        _vec(0.5, 0.5, 0.0))
    # Future capture (clamped to age 0).
    fusion.tracks['future'] = _track(
        'future', 2_010_000_000, _box((4.0, 0.0, 0.5), (0.2, 0.2, 1.0)),
        _vec(0.2, 0.0, 0.0))
    _assert_matches(fusion, now)
    _assert_matches(fusion, now, region=(0.0, 0.0, 2.0))


def test_snapshot_native_matches_reference_random():
    profile = _profile()
    rng = random.Random(20260919)
    fusion = ConservativeFusion(profile, frame_id='odom')
    now = Stamp(2_000_000_000, 'sim', 0)
    for i in range(200):
        capture = 2_000_000_000 - rng.randint(0, 2_000_000_000)
        center = (rng.uniform(-10, 10), rng.uniform(-10, 10), rng.uniform(0.1, 2.0))
        size = (rng.uniform(0.05, 0.8), rng.uniform(0.05, 0.8), rng.uniform(0.2, 1.5))
        diag = tuple(rng.uniform(0., 0.1) if k in (0, 4, 8) else 0. for k in range(9))
        velocity = (rng.uniform(-2, 2), rng.uniform(-2, 2), 0.0)
        has_direct = rng.random() < 0.3
        obs = _obs(
            'scan', f'id{i}', capture,
            _box(center, size, diag,
                 velocity=velocity if has_direct else None,
                 velocity_cov=(rng.uniform(0, 0.1) if k in (0, 4, 8) else 0. for k in range(9))
                 if has_direct else None),
            velocity_observable=rng.random() < 0.9)
        samples = ()
        if not has_direct and rng.random() < 0.7:
            t0 = rng.uniform(0., 1.0)
            samples = tuple((t0 + 0.1 * j, rng.uniform(-1, 1), rng.uniform(-1, 1))
                            for j in range(rng.randint(1, 8)))
        fusion.tracks[f'id{i}'] = _Track(f'id{i}', obs, _vec(*velocity), samples)
    _assert_matches(fusion, now)
    _assert_matches(fusion, now, region=(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(0, 3)))


def test_empty_snapshot_native_matches_reference():
    profile = _profile()
    fusion = ConservativeFusion(profile, frame_id='odom')
    now = Stamp(1_000_000_000, 'sim', 0)
    _assert_matches(fusion, now)


def test_epoch_change_clears_tracks_in_both_paths():
    profile = _profile()
    fusion = ConservativeFusion(profile, frame_id='odom')
    fusion.tracks['t'] = _track('t', 1_000_000_000, _box((0., 0., 0.5), (0.3, 0.3, 1.)),
                                _vec(0., 0., 0.))
    fusion.epoch = ('sim', 0)
    # Clock/epoch mismatch triggers update((), now) -> clears tracks.
    a = fusion.snapshot(Stamp(2_000_000_000, 'sim', 1))
    b = fusion.snapshot_native(Stamp(2_000_000_000, 'sim', 1))
    assert a.tracks == b.tracks == ()


def test_kernel_rejects_nan_and_bad_shape():
    rows = np.zeros((1, 32))
    rows[0, 0] = 1_000_000_000
    rows[0, 1] = np.nan
    with pytest.raises(ValueError):
        _kernel(rows, 2_000_000_000, {}, np.asarray([0, 0], dtype=np.int64),
                np.empty(0), np.empty(0), np.empty(0))
    with pytest.raises(ValueError):
        _kernel(np.zeros((1, 5)), 2_000_000_000, {}, np.asarray([0, 0], dtype=np.int64),
                np.empty(0), np.empty(0), np.empty(0))


def test_kernel_rejects_inf_velocity_covariance():
    rows = np.zeros((1, 32))
    rows[0, 0] = 1_000_000_000
    rows[0, 4:7] = (0.3, 0.3, 1.0)
    rows[0, 19] = 1.0  # has_velocity_covariance
    rows[0, 23] = np.inf
    with pytest.raises(ValueError):
        _kernel(rows, 2_000_000_000, {}, np.asarray([0, 0], dtype=np.int64),
                np.empty(0), np.empty(0), np.empty(0))


def test_runtime_capture_stamps_keep_single_nanosecond_precision():
    rows = np.zeros((1, 32))
    rows[0, 1:4] = (1., 2., .5)
    rows[0, 4:7] = (.3, .3, 1.)
    # The floating compatibility column is intentionally unusable here. The
    # runtime list is the authoritative integer nanosecond transport.
    capture = [9_007_199_254_740_993]
    result = _kernel(rows, capture[0] + 100, {}, np.asarray([0, 0], dtype=np.int64),
                     np.empty(0), np.empty(0), np.empty(0), capture)
    assert result['centers'][0, 0] == pytest.approx(1.0)


def test_float_capture_stamp_rejects_rounding_when_no_integer_transport():
    rows = np.zeros((1, 32))
    rows[0, 0] = 9_007_199_254_740_993.0
    rows[0, 4:7] = (.3, .3, 1.)
    with pytest.raises(ValueError):
        _kernel(rows, 9_007_199_254_740_993, {}, np.asarray([0, 0], dtype=np.int64),
                np.empty(0), np.empty(0), np.empty(0))
