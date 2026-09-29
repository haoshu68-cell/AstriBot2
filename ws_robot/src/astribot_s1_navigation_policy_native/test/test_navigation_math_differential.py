import math
import random
from types import SimpleNamespace

import numpy as np
import pytest

from astribot_s1_navigation_policy import motion_geometry
from astribot_s1_navigation_policy import swept_geometry
from astribot_s1_navigation_policy import cmd_vel_math
from astribot_s1_navigation_policy import continuous_sweep
from astribot_s1_navigation_policy import risk
from astribot_s1_navigation_policy import scan_occupancy
from astribot_s1_navigation_policy import control_time
from astribot_s1_navigation_policy import candidate_variants
from astribot_s1_navigation_policy import path_evidence
from astribot_s1_navigation_policy import behavior
from astribot_s1_navigation_policy import execution_context
from astribot_s1_navigation_policy import sensor_health
from astribot_s1_navigation_policy import planning_session
from astribot_s1_navigation_policy.contracts import (
    Version, BearingCone, Vec3, Planning, Trigger, Stamp)
from astribot_s1_navigation_policy_native import _navigation_math_native as native


PROFILE = SimpleNamespace(
    reaction_time_s=0.18,
    brake_deceleration_m_s2=0.72,
    angular_brake_deceleration_rad_s2=1.3,
    linear_stop_delay_s=0.04,
    half_length_m=0.36,
    half_width_m=0.27,
    clearance_margin_m=0.08,
    payload_extra_margin_m=0.01,
)


def _close(actual, expected, tol=3e-12):
    if isinstance(actual, (list, tuple)):
        assert len(actual) == len(expected)
        for lhs, rhs in zip(actual, expected):
            _close(lhs, rhs, tol)
        return
    assert math.isclose(actual, expected, rel_tol=tol, abs_tol=tol), (actual, expected)


def _reference(module, call, *args, **kwargs):
    old = module._native
    module._native = None
    try:
        return call(*args, **kwargs)
    finally:
        module._native = old


def test_motion_math_randomized_and_thresholds():
    rng = random.Random(20260920)
    motion_geometry._native = native
    swept_geometry._native = native
    for _ in range(5000):
        command = [rng.uniform(-2.0, 2.0) for _ in range(3)]
        if rng.random() < 0.2:
            command[2] = rng.choice((-1e-6, 1e-6, 0.0))
        t = rng.uniform(-1.0, 4.0)
        step = rng.uniform(0.0, 0.2)
        _close(motion_geometry.stopping_horizon(command, PROFILE),
               _reference(motion_geometry, motion_geometry.stopping_horizon,
                          command, PROFILE))
        _close(motion_geometry.body_pose(command, t),
               _reference(motion_geometry, motion_geometry.body_pose, command, t))
        _close(motion_geometry.sampling_margin(command, PROFILE, step),
               _reference(motion_geometry, motion_geometry.sampling_margin,
                          command, PROFILE, step))
        yaw = rng.uniform(-math.pi, math.pi)
        _close(swept_geometry.footprint_axes(PROFILE.half_length_m,
                                             PROFILE.half_width_m, yaw),
               _reference(swept_geometry, swept_geometry.footprint_axes,
                          PROFILE.half_length_m, PROFILE.half_width_m, yaw))
        cmd_vel_math._native = native
        _close(cmd_vel_math.body_to_world_xy(command[0], command[1], yaw),
               _reference(cmd_vel_math, cmd_vel_math.body_to_world_xy,
                          command[0], command[1], yaw))


def test_body_pose_custom_trig_stays_on_python_path():
    class Trig:
        sin = staticmethod(math.sin)
        cos = staticmethod(math.cos)

    motion_geometry._native = native
    expected = _reference(motion_geometry, motion_geometry.body_pose,
                          [0.4, -0.2, 0.3], 0.7, Trig)
    actual = motion_geometry.body_pose([0.4, -0.2, 0.3], 0.7, Trig)
    _close(actual, expected)


def test_rectangle_clearance_batch_matches_python_reference():
    rng = random.Random(20260920)
    lower = []
    upper = []
    x = []
    y = []
    yaw = []
    for _ in range(2000):
        cx, cy = (rng.uniform(-2.0, 2.0) for _ in range(2))
        hx, hy = (rng.uniform(0.05, 0.5) for _ in range(2))
        lower.append((cx - hx, cy - hy))
        upper.append((cx + hx, cy + hy))
        x.append(rng.uniform(-2.0, 2.0))
        y.append(rng.uniform(-2.0, 2.0))
        yaw.append(rng.uniform(-math.pi, math.pi))
    old = swept_geometry._native
    try:
        swept_geometry._native = None
        expected = swept_geometry.clearance_many(
            x, y, yaw, np.asarray(lower), np.asarray(upper), PROFILE)
        swept_geometry._native = native
        actual = swept_geometry.clearance_many(
            x, y, yaw, np.asarray(lower), np.asarray(upper), PROFILE)
    finally:
        swept_geometry._native = old
    assert actual.shape == expected.shape
    for lhs, rhs in zip(actual.ravel(), expected.ravel()):
        _close(float(lhs), float(rhs), tol=2e-10)


def test_motion_clearance_randomized_intervals_match_python_reference():
    rng = random.Random(20260922)
    old = continuous_sweep._native
    try:
        for _ in range(250):
            count = rng.randint(1, 8)
            begin = np.asarray([rng.uniform(0.0, 1.5) for _ in range(count)])
            end = begin + np.asarray([rng.uniform(0.0, 0.6) for _ in range(count)])
            centers = np.asarray([[rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5)]
                                  for _ in range(count)])
            half = np.asarray([[rng.uniform(0.05, 0.45), rng.uniform(0.05, 0.45)]
                               for _ in range(count)])
            lower = centers - half
            upper = centers + half
            command = [rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0),
                       rng.uniform(-1.2, 1.2)]
            origin = (rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5),
                      rng.uniform(-math.pi, math.pi))
            continuous_sweep._native = None
            expected = continuous_sweep.motion_clearance(
                command, begin, end, lower, upper, PROFILE, origin)
            continuous_sweep._native = native
            actual = continuous_sweep.motion_clearance(
                command, begin, end, lower, upper, PROFILE, origin)
            assert actual.shape == expected.shape
            for lhs, rhs in zip(actual.ravel(), expected.ravel()):
                _close(float(lhs), float(rhs), tol=3e-10)
    finally:
        continuous_sweep._native = old


def test_motion_clearance_invalid_intervals_keep_python_error_boundary():
    old = continuous_sweep._native
    try:
        continuous_sweep._native = native
        with pytest.raises(ValueError):
            continuous_sweep.motion_clearance(
                [0.1, 0.0, 0.0], [0.2], [0.1],
                [[-0.2, -0.2]], [[0.2, 0.2]], PROFILE)
    finally:
        continuous_sweep._native = old


def test_path_position_batch_preserves_route_endpoint_tangents():
    rng = random.Random(20260922)
    old = risk._native
    fallback = risk.RobotState(0.4, -0.3, 0.7)
    try:
        for _ in range(200):
            path = [(rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0))
                    for _ in range(rng.randint(0, 8))]
            if path and rng.random() < 0.5:
                path.insert(rng.randrange(len(path)), path[0])
            distances = [rng.uniform(-0.2, 3.0) for _ in range(8)]
            risk._native = None
            expected = risk._path_position_batch(path, distances, fallback)
            risk._native = native
            actual = risk._path_position_batch(path, distances, fallback)
            assert actual.shape == expected.shape
            for lhs, rhs in zip(actual.ravel(), expected.ravel()):
                _close(float(lhs), float(rhs), tol=3e-12)
    finally:
        risk._native = old


def test_remaining_path_projection_matches_python_reference():
    rng = random.Random(20260922)
    old = risk._native
    robot = risk.RobotState(0.4, -0.3, 0.7)
    try:
        for _ in range(200):
            path = [(rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0))
                    for _ in range(rng.randint(0, 8))]
            risk._native = None
            expected = risk.remaining_path(path, robot)
            risk._native = native
            actual = risk.remaining_path(path, robot)
            assert actual == expected
    finally:
        risk._native = old


def test_path_samples_replay_matches_python_reference():
    rng = random.Random(20260922)
    old = swept_geometry._native
    try:
        for _ in range(100):
            route = [(rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0))
                     for _ in range(rng.randint(1, 8))]
            if len(route) > 2 and rng.random() < 0.5:
                route.insert(1, route[0])
            reach = rng.uniform(-0.1, 3.0)
            swept_geometry._native = None
            expected = list(swept_geometry.path_samples(route, reach, PROFILE))
            swept_geometry._native = native
            actual = list(swept_geometry.path_samples(route, reach, PROFILE))
            assert len(actual) == len(expected)
            for lhs, rhs in zip(actual, expected):
                for lhs_value, rhs_value in zip(lhs, rhs):
                    _close(float(lhs_value), float(rhs_value), tol=3e-12)
    finally:
        swept_geometry._native = old


def test_scan_occupancy_replay_matches_python_reference():
    rng = random.Random(20260922)
    old = scan_occupancy._native
    try:
        for _ in range(300):
            points = [(rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0))
                      for _ in range(rng.randint(0, 40))]
            resolution = rng.uniform(0.02, 0.5)
            scan_occupancy._native = None
            expected_cells = scan_occupancy.occupied_cells(points, resolution)
            scan_occupancy._native = native
            actual_cells = scan_occupancy.occupied_cells(points, resolution)
            assert actual_cells == expected_cells

            corners = [(rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5))
                       for _ in range(4)]
            ranges = [rng.uniform(0.2, 4.0) for _ in range(rng.randint(1, 80))]
            args = (corners, ranges, 0.1, 4.5, -math.pi, 2.0 * math.pi / len(ranges),
                    resolution)
            scan_occupancy._native = None
            expected_free = scan_occupancy.angular_box_free(*args)
            scan_occupancy._native = native
            actual_free = scan_occupancy.angular_box_free(*args)
            assert actual_free == expected_free
    finally:
        scan_occupancy._native = old


def test_scan_occupancy_invalid_inputs_keep_python_boundary():
    old = scan_occupancy._native
    try:
        scan_occupancy._native = native
        with pytest.raises(ValueError):
            scan_occupancy.occupied_cells([(0.0, 0.0)], 0.0)
        with pytest.raises(ValueError):
            scan_occupancy.occupied_cells([(math.nan, 0.0)], 0.1)
        with pytest.raises(ZeroDivisionError):
            scan_occupancy.angular_box_free([], [], 0.1, 4.0, -1.0, 0.1, 0.2)
    finally:
        scan_occupancy._native = old


def test_control_time_replay_matches_python_watchdog():
    sequences = [
        (True, [(10.0, 100.0), (10.01, 100.01), (10.01, 100.60),
                (10.02, 100.61), (9.0, 100.62), (9.01, 100.63)]),
        (False, [(10.0, 100.0), (10.01, 100.01), (9.0, 100.60),
                 (9.01, 100.61)]),
    ]
    old = control_time._native_time
    try:
        for simulated, ticks in sequences:
            control_time._native_time = None
            expected = control_time.ControlTime(simulated, 0.5)
            control_time._native_time = native
            actual = control_time.ControlTime(simulated, 0.5)
            for ros, wall in ticks:
                control_time._native_time = None
                lhs = expected.advance(ros, wall)
                control_time._native_time = native
                rhs = actual.advance(ros, wall)
                assert rhs == lhs
                for capture, received_wall, ttl in (
                        (ros, wall, 0.3), (ros - 0.2, wall - 0.2, 0.3),
                        (ros + 0.1, wall, 0.3)):
                    control_time._native_time = None
                    expected_values = (
                        expected.accepts(capture),
                        expected.fresh(capture, received_wall, ttl, ros, wall),
                        expected.command_fresh(capture, received_wall, ttl, ros, wall))
                    control_time._native_time = native
                    actual_values = (
                        actual.accepts(capture),
                        actual.fresh(capture, received_wall, ttl, ros, wall),
                        actual.command_fresh(capture, received_wall, ttl, ros, wall))
                    assert actual_values == expected_values
    finally:
        control_time._native_time = old


def test_control_time_invalid_clock_boundary_stays_python():
    old = control_time._native_time
    try:
        for use_native in (False, True):
            control_time._native_time = native if use_native else None
            with pytest.raises(ValueError):
                control_time.ControlTime(True, 0.0)
            clock = control_time.ControlTime(True, 0.5)
            with pytest.raises(ValueError):
                clock.advance(float('nan'), 1.0)
    finally:
        control_time._native_time = old


def test_lateral_variants_replay_matches_python_reference():
    rng = random.Random(20260923)
    old = candidate_variants._native
    try:
        for _ in range(300):
            route = [(float(i) * 0.25 + rng.uniform(-0.03, 0.03),
                      rng.uniform(-0.4, 0.4)) for i in range(rng.randint(3, 10))]
            maximum = rng.uniform(0.01, 0.2)
            candidate_variants._native = None
            expected = candidate_variants.lateral_variants(route, maximum)
            candidate_variants._native = native
            actual = candidate_variants.lateral_variants(route, maximum)
            assert len(actual) == len(expected)
            for lhs_variant, rhs_variant in zip(actual, expected):
                for lhs, rhs in zip(lhs_variant, rhs_variant):
                    _close(lhs, rhs, tol=3e-12)

        for route, maximum in (
                ([(0.0, 0.0), (0.1, 0.0), (0.2, 0.0)], 0.1),
                ([(0.0, 0.0), (1.0, 0.0)], 0.1),
                ([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], 0.0)):
            candidate_variants._native = None
            expected = candidate_variants.lateral_variants(route, maximum)
            candidate_variants._native = native
            actual = candidate_variants.lateral_variants(route, maximum)
            assert actual == expected
    finally:
        candidate_variants._native = old


def test_path_evidence_assessment_matches_python_reference():
    rng = random.Random(20260923)
    profile = SimpleNamespace(path_risk_timeout_s=0.5,
                              max_speed_m_s=0.7,
                              clearance_margin_m=0.08,
                              payload_extra_margin_m=0.02)
    old = path_evidence._native
    try:
        for _ in range(500):
            evidence = path_evidence.PathEvidence(
                path_key=('path', rng.randint(0, 3)),
                stamp_s=10.0 + rng.uniform(-0.4, 0.8),
                received_wall_s=100.0 + rng.uniform(-0.4, 0.8),
                epoch=rng.randint(1, 2), known=bool(rng.randrange(2)),
                blocked=bool(rng.randrange(2)),
                distance_m=rng.uniform(-0.2, 2.0))
            args = (evidence, ('path', rng.randint(0, 3)),
                    10.0 + rng.uniform(-0.2, 0.8),
                    100.0 + rng.uniform(-0.2, 0.8), rng.randint(1, 2),
                    bool(rng.randrange(2)), profile)
            path_evidence._native = None
            expected = path_evidence.assess_path(*args)
            path_evidence._native = native
            actual = path_evidence.assess_path(*args)
            assert actual == expected
    finally:
        path_evidence._native = old


def test_path_evidence_invalid_and_legacy_boundaries_stay_python():
    profile = SimpleNamespace(path_risk_timeout_s=0.5,
                              max_speed_m_s=0.7,
                              clearance_margin_m=0.08,
                              payload_extra_margin_m=0.02)
    evidence = path_evidence.PathEvidence(
        path_key='p', stamp_s=float('nan'), received_wall_s=100.0,
        epoch=1, known=True, blocked=True, distance_m=1.0)
    old = path_evidence._native
    try:
        path_evidence._native = None
        expected = path_evidence.assess_path(evidence, 'p', 10.0, 100.0, 1, False, profile)
        path_evidence._native = native
        actual = path_evidence.assess_path(evidence, 'p', 10.0, 100.0, 1, False, profile)
        assert actual == expected
        assert path_evidence.assess_path(None, 'p', 10.0, 100.0, 1, True, profile).status == 'LEGACY'
    finally:
        path_evidence._native = old


def test_yield_policy_replay_matches_python_state_machine():
    profile = SimpleNamespace(max_speed_m_s=0.7, narrow_speed_m_s=0.2,
                              reaction_time_s=0.18,
                              brake_deceleration_m_s2=0.72,
                              angular_brake_deceleration_rad_s2=1.3,
                              linear_stop_delay_s=0.04,
                              wait_budget_s=2.0, clear_hold_s=0.3)
    rng = random.Random(20260923)
    sequence = []
    now = 10.0
    for _ in range(1000):
        now += rng.choice((0.02, 0.05, 0.2, -0.1))
        sequence.append((rng.choice((None, SimpleNamespace(
            immediate=bool(rng.randrange(3) == 0),
            blocked=bool(rng.randrange(2)),
            uncertain=bool(rng.randrange(5) == 0),
            conflict_time_s=rng.uniform(0.0, 4.0)))),
                         bool(rng.randrange(7)), now))
    old = behavior._native
    try:
        behavior._native = None
        expected_policy = behavior.YieldPolicy(profile)
        expected = [expected_policy.select(risk, valid, stamp)
                    for risk, valid, stamp in sequence]
        behavior._native = native
        actual_policy = behavior.YieldPolicy(profile)
        actual = [actual_policy.select(risk, valid, stamp)
                  for risk, valid, stamp in sequence]
        assert actual == expected
        assert actual_policy.held == expected_policy.held
        assert actual_policy.episode == expected_policy.episode
        assert actual_policy.in_episode == expected_policy.in_episode

        risk = SimpleNamespace(immediate=False, blocked=True, uncertain=False,
                               conflict_time_s=0.1)
        behavior._native = None
        expected_stop = behavior.requires_stop(risk, profile)
        behavior._native = native
        actual_stop = behavior.requires_stop(risk, profile)
        assert actual_stop == expected_stop
    finally:
        behavior._native = old


def test_execution_context_replay_matches_python_version_state():
    operations = [
        ('task', ('goal-a', 'EXECUTING', 1)),
        ('task', ('stale', 'EXECUTING', 0)),
        ('map', (('map', 1, 2), b'abc')),
        ('map', (('map', 1, 2), b'abc')),
        ('localization', ((0.0, 0.0, 0.0), 0.2, 0.15)),
        ('localization', ((0.1, 0.0, 0.05), 0.2, 0.15)),
        ('localization', ((0.5, 0.0, 0.05), 0.2, 0.15)),
        ('path', ()),
        ('task', ('goal-a', 'SUCCEEDED', 2)),
    ]
    old = execution_context._native
    try:
        execution_context._native = None
        expected = execution_context.ExecutionContext()
        execution_context._native = native
        actual = execution_context.ExecutionContext()
        for name, args in operations:
            lhs = getattr(expected, name)(*args)
            rhs = getattr(actual, name)(*args)
            assert rhs == lhs
            assert actual.version == expected.version
            assert actual.sequence == expected.sequence
            assert actual.map_key == expected.map_key
        replacement = Version('external', 7, 8, 9, 10, 11)
        expected.version = replacement
        actual.version = replacement
        actual.path()
        expected.path()
        assert actual.version == expected.version
    finally:
        execution_context._native = old


def _cone_signature(cones):
    return tuple((cone.direction.x, cone.direction.y, cone.direction.z,
                  cone.half_angle_rad) for cone in cones)


def test_sensor_health_directional_kernels_match_python_reference():
    rng = random.Random(20260923)
    old = sensor_health._native
    try:
        for _ in range(1000):
            count = rng.randint(0, 96)
            ranges = []
            for _ in range(count):
                choice = rng.randrange(12)
                if choice == 0:
                    ranges.append(float('inf'))
                elif choice == 1:
                    ranges.append(float('nan'))
                else:
                    ranges.append(rng.uniform(-0.2, 6.0))
            args = (ranges, rng.uniform(0.0, 0.5), rng.uniform(1.0, 8.0),
                    rng.uniform(-math.pi, math.pi),
                    rng.uniform(-0.2, 0.2), rng.uniform(-math.pi, math.pi))
            sensor_health._native = None
            expected = sensor_health.scan_coverage(*args)
            sensor_health._native = native
            actual = sensor_health.scan_coverage(*args)
            actual_signature = _cone_signature(actual)
            expected_signature = _cone_signature(expected)
            assert len(actual_signature) == len(expected_signature)
            for actual_cone, expected_cone in zip(actual_signature, expected_signature):
                assert actual_cone == pytest.approx(expected_cone, rel=0.0, abs=3e-14)

            motion = (rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0),
                      rng.uniform(-1.0, 1.0))
            sensor_health._native = None
            expected_directions = sensor_health.movement_directions(*motion)
            sensor_health._native = native
            actual_directions = sensor_health.movement_directions(*motion)
            assert actual_directions == pytest.approx(expected_directions,
                                                      rel=0.0, abs=3e-14)

            cones = tuple(BearingCone(
                Vec3(math.cos(angle), math.sin(angle), 0.0),
                rng.uniform(0.0, math.pi))
                for angle in (rng.uniform(-math.pi, math.pi)
                              for _ in range(rng.randint(0, 12))))
            sensor_health._native = None
            expected_allows = sensor_health.coverage_allows_motion(
                cones, *motion)
            sensor_health._native = native
            actual_allows = sensor_health.coverage_allows_motion(cones, *motion)
            assert actual_allows is expected_allows
    finally:
        sensor_health._native = old


def test_sensor_health_native_dispatch_preserves_python_boundaries():
    old = sensor_health._native
    try:
        sensor_health._native = native
        with pytest.raises(TypeError):
            sensor_health.scan_coverage(iter((1.0, 2.0)), 0.1, 3.0, 0.0, 0.1)
        with pytest.raises(TypeError):
            sensor_health.movement_directions('0.1', 0.0, 0.0)
        with pytest.raises(AttributeError):
            sensor_health.coverage_allows_motion((object(),), 0.1, 0.0, 0.0)
    finally:
        sensor_health._native = old


def test_planning_session_replay_matches_native_state_machine():
    budget = planning_session.PlanningBudget(0.1, 0.5, 2)
    version = Version('goal-a', 1, 2, 3, 4, 5)
    updated = Version('goal-a', 2, 2, 3, 4, 5)
    other = Version('goal-b', 1, 2, 3, 4, 5)
    def stamp(ns, epoch=5):
        return Stamp(ns, 'steady', epoch)

    old = planning_session._native
    try:
        planning_session._native = None
        expected = planning_session.PlanningSession('replay', budget)
        planning_session._native = native
        actual = planning_session.PlanningSession('replay', budget)
        expected.activate(version, stamp(1_000_000_000))
        actual.activate(version, stamp(1_000_000_000))

        kinds = frozenset({Planning.LOCAL})
        expected_request = expected.request(
            version, Trigger.PATH_RISK, kinds, 10, stamp(1_000_000_000))
        actual_request = actual.request(
            version, Trigger.PATH_RISK, kinds, 10, stamp(1_000_000_000))
        assert actual_request == expected_request
        assert actual.request(version, Trigger.PATH_RISK, kinds, 11,
                             stamp(1_050_000_000)) == expected.request(
                                 version, Trigger.PATH_RISK, kinds, 11,
                                 stamp(1_050_000_000))
        assert actual.response_current(actual_request, version,
                                      stamp(1_060_000_000)) == expected.response_current(
                                          expected_request, version,
                                          stamp(1_060_000_000))
        actual.retire(actual_request)
        expected.retire(expected_request)
        assert actual.failure_reason(stamp(1_200_000_000)) == expected.failure_reason(
            stamp(1_200_000_000))

        expected.activate(updated, stamp(1_250_000_000))
        actual.activate(updated, stamp(1_250_000_000))
        expected_second = expected.request(
            updated, Trigger.PATH_RISK, kinds, 12, stamp(1_300_000_000))
        actual_second = actual.request(
            updated, Trigger.PATH_RISK, kinds, 12, stamp(1_300_000_000))
        assert actual_second == expected_second
        with pytest.raises(planning_session.PlanningBudgetExhausted) as lhs:
            expected.request(updated, Trigger.PATH_RISK, kinds, 13,
                             stamp(1_500_000_001))
        with pytest.raises(planning_session.PlanningBudgetExhausted) as rhs:
            actual.request(updated, Trigger.PATH_RISK, kinds, 13,
                           stamp(1_500_000_001))
        assert str(rhs.value) == str(lhs.value)

        expected.activate(other, stamp(1_600_000_000))
        actual.activate(other, stamp(1_600_000_000))
        assert actual.request(other, Trigger.NEW_GOAL, kinds, 14,
                             stamp(1_600_000_000)) == expected.request(
                                 other, Trigger.NEW_GOAL, kinds, 14,
                                 stamp(1_600_000_000))
        with pytest.raises(Exception) as lhs:
            expected.request(other, Trigger.NEW_GOAL, kinds, 15,
                             stamp(1_500_000_000))
        with pytest.raises(Exception) as rhs:
            actual.request(other, Trigger.NEW_GOAL, kinds, 15,
                           stamp(1_500_000_000))
        assert type(rhs.value) is type(lhs.value)
        assert str(rhs.value) == str(lhs.value)
    finally:
        planning_session._native = old
