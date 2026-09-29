import math
import random

from astribot_s1_navigation_policy import continuous_sweep
from astribot_s1_navigation_policy import protection
from astribot_s1_navigation_policy_native import _navigation_math_native as native


class Profile:
    max_acceleration_m_s2 = 0.8
    max_angular_acceleration_rad_s2 = 1.7


class SweepProfile:
    reaction_time_s = 0.18
    brake_deceleration_m_s2 = 0.72
    angular_brake_deceleration_rad_s2 = 1.3
    linear_stop_delay_s = 0.04
    half_length_m = 0.36
    half_width_m = 0.27
    clearance_margin_m = 0.08
    payload_extra_margin_m = 0.01


def _run(use_native, cases, recovering=True):
    old = protection._native
    protection._native = native if use_native else None
    try:
        limiter = protection.CommandRestriction(Profile())
        limiter.recovering = recovering
        trace = []
        for command, cap, angular_cap, stop, dt, allow_zero_dt in cases:
            output = limiter.apply(command, cap, angular_cap, stop, dt,
                                   allow_zero_dt=allow_zero_dt)
            trace.append((tuple(output), limiter.recovering))
        return trace, tuple(limiter.output), limiter.recovering
    finally:
        protection._native = old


def _close(lhs, rhs, tol=3e-12):
    if math.isnan(lhs) and math.isnan(rhs):
        return
    assert math.isclose(lhs, rhs, rel_tol=tol, abs_tol=tol), (lhs, rhs)


def test_command_restriction_randomized_replay_matches_python():
    rng = random.Random(20260920)
    cases = []
    for _ in range(10000):
        cases.append((
            tuple(rng.uniform(-2.0, 2.0) for _ in range(3)),
            rng.uniform(0.0, 2.0), rng.uniform(0.0, 2.0),
            bool(rng.randrange(7) == 0), rng.choice((0.0, 1e-12, 0.004, 0.1)),
            bool(rng.randrange(5) == 0)))
    python_trace, python_output, python_recovering = _run(False, cases)
    native_trace, native_output, native_recovering = _run(True, cases)
    assert len(python_trace) == len(native_trace)
    for (lhs, lhs_recovering), (rhs, rhs_recovering) in zip(python_trace, native_trace):
        assert lhs_recovering == rhs_recovering
        for a, b in zip(lhs, rhs):
            _close(a, b)
    for a, b in zip(python_output, native_output):
        _close(a, b)
    assert python_recovering == native_recovering


def test_command_restriction_invalid_and_nonfinite_inputs_retain_stop_boundary():
    cases = [
        ((1.0, 0.0, 0.0), -1.0, 1.0, False, 0.1, False),
        ((1.0, 0.0, 0.0), 1.0, 1.0, False, -0.1, False),
        ((math.nan, 0.0, 0.0), 1.0, 1.0, False, 0.1, False),
        ((1.0, 0.0, 0.0), 1.0, 1.0, True, 0.1, False),
    ]
    python_trace, _, _ = _run(False, cases)
    native_trace, _, _ = _run(True, cases)
    for (lhs, lhs_recovering), (rhs, rhs_recovering) in zip(python_trace, native_trace):
        assert lhs_recovering == rhs_recovering
        for a, b in zip(lhs, rhs):
            _close(a, b)


def test_nonfinite_boundary_does_not_rejoin_stale_native_state():
    finite = ((1.0, 0.0, 0.0), 1.0, 1.0, False, 0.1, False)
    nonfinite = ((math.nan, 0.0, 0.0), 1.0, 1.0, False, 0.1, False)
    follow = ((0.2, 0.0, 0.0), 1.0, 1.0, False, 0.1, False)
    python_trace, python_output, _ = _run(False, [finite, nonfinite, follow])
    native_trace, native_output, _ = _run(True, [finite, nonfinite, follow])
    for (lhs, lhs_recovering), (rhs, rhs_recovering) in zip(python_trace, native_trace):
        assert lhs_recovering == rhs_recovering
        for a, b in zip(lhs, rhs):
            _close(a, b)
    for a, b in zip(python_output, native_output):
        _close(a, b)


def test_python_recovering_override_is_preserved_by_native_facade():
    cases = [((0.2, 0.0, 0.0), 1.0, 1.0, False, 0.1, False)]
    python_trace, _, _ = _run(False, cases, recovering=False)
    native_trace, _, _ = _run(True, cases, recovering=False)
    assert python_trace == native_trace


def test_scan_and_costmap_helpers_match_python_reference():
    ranges = [0.1, 0.2, math.inf, 0.7, math.nan]
    old = protection._native
    try:
        protection._native = None
        expected_scan = protection.scan_usable(ranges, 0.05, 1.0, -1.0, 0.1, 0.6)
        expected_clear = protection.costmap_clearing_ranges(
            ranges, 3.0, 1.0)
        protection._native = native
        actual_scan = protection.scan_usable(ranges, 0.05, 1.0, -1.0, 0.1, 0.6)
        actual_clear = protection.costmap_clearing_ranges(ranges, 3.0, 1.0)
    finally:
        protection._native = old
    assert actual_scan == expected_scan
    assert len(actual_clear) == len(expected_clear)
    for lhs, rhs in zip(actual_clear, expected_clear):
        _close(lhs, rhs)
    for args in (([], 0.05, 1.0, -1.0, 0.1, 0.5),
                 ([0.1], 1.0, 1.0, -1.0, 0.1, 0.5)):
        protection._native = None
        expected = protection.scan_usable(*args)
        protection._native = native
        actual = protection.scan_usable(*args)
        assert actual == expected
    protection._native = old


def test_swept_point_collision_replay_uses_the_same_motion_kernel():
    rng = random.Random(20260922)
    cases = []
    for _ in range(250):
        points = [(rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5))
                  for _ in range(rng.randint(0, 5))]
        command = tuple(rng.uniform(-0.8, 0.8) for _ in range(3))
        cases.append((points, command))
    old = continuous_sweep._native
    try:
        continuous_sweep._native = None
        expected = [protection.swept_point_collision(points, command, SweepProfile())
                    for points, command in cases]
        continuous_sweep._native = native
        actual = [protection.swept_point_collision(points, command, SweepProfile())
                  for points, command in cases]
    finally:
        continuous_sweep._native = old
    assert actual == expected
