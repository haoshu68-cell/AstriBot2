"""Same-input differential test for the first bridge migration step.

Run after building this package and sourcing its isolated install overlay:

  PYTHONPATH=<repo>/ws_robot/src/astribot_trajectory_bridge \
    python3 -m pytest -q <repo>/ws_robot/src/astribot_trajectory_bridge_native/test/test_chassis_math_differential.py

The Python implementation is deliberately retained as the oracle here.  A
future phase may remove it only after the whole bridge state-machine replay
suite is passing.
"""

import math
import random

import pytest

from astribot_trajectory_bridge_native import _chassis_math_native as native
from astribot_trajectory_bridge import chassis_integrator as bridge
from astribot_trajectory_bridge import chassis_feedback as feedback
from astribot_trajectory_bridge import chassis_odom_source as odom

# Production keeps Python as the default during the compatibility phase; this
# test explicitly exercises the candidate native implementation.
bridge._native = native
feedback._native = native


def _close(actual, expected, tol=3e-12):
    if isinstance(actual, (list, tuple)):
        assert len(actual) == len(expected)
        for lhs, rhs in zip(actual, expected):
            _close(lhs, rhs, tol)
        return
    if math.isnan(actual) and math.isnan(expected):
        return
    assert math.isclose(actual, expected, rel_tol=tol, abs_tol=tol)


def _reference(call, *args):
    old = bridge._native
    bridge._native = None
    try:
        return call(*args)
    finally:
        bridge._native = old


def _feedback_reference(call, *args):
    old = feedback._native
    feedback._native = None
    try:
        return call(*args)
    finally:
        feedback._native = old


@pytest.mark.parametrize("theta", [0.0, math.pi, -math.pi, 3.0 * math.pi, -17.0 * math.pi])
def test_wrap_angle_boundary(theta):
    _close(native.wrap_angle(theta), _reference(bridge.wrap_angle, theta))


def test_randomized_same_input_kernels():
    rng = random.Random(20260920)
    for _ in range(10000):
        previous = [rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0), rng.uniform(-40.0, 40.0)]
        current = [rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0), rng.uniform(-40.0, 40.0)]
        velocity = [rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0), rng.uniform(-3.0, 3.0)]
        dt = rng.uniform(1e-8, 0.2)
        frequency = rng.uniform(1.0, 500.0)
        theta = rng.uniform(-40.0, 40.0)
        cases = [
            (native.wrap_angle, bridge.wrap_angle, (previous[2],)),
            (native.local_pose_displacement, bridge.local_pose_displacement, (previous, current)),
            (native.integrate_step_dt, bridge.integrate_step_dt, (previous, velocity, dt)),
            (native.integrate_step, bridge.integrate_step, (previous, velocity, frequency)),
            (native.pose_error, bridge.pose_error, (previous, current)),
            (native.error_magnitude, bridge.error_magnitude, (velocity,)),
            (native.to_local_velocity, bridge.to_local_velocity, (velocity, "body", theta)),
            (native.to_local_velocity, bridge.to_local_velocity, (velocity, "world", theta)),
        ]
        for native_call, python_call, args in cases:
            _close(native_call(*args), _reference(python_call, *args))


def test_feedback_safety_math_matches_python_reference():
    rng = random.Random(20260920)
    for _ in range(10000):
        command = [rng.uniform(-4.0, 4.0) for _ in range(3)]
        actual = [rng.uniform(-4.0, 4.0) for _ in range(3)]
        previous = [rng.uniform(-4.0, 4.0) for _ in range(3)]
        now = [rng.uniform(-4.0, 4.0) for _ in range(3)]
        sdk_disp = [rng.uniform(-1.0, 1.0) for _ in range(2)]
        slam_disp = [rng.uniform(-1.0, 1.0) for _ in range(2)]
        leash_expected = _feedback_reference(
            feedback.check_leash, command, actual, 0.25, 0.35)
        _close(native.leash_error(tuple(command), tuple(actual)),
               (leash_expected.err_xy, leash_expected.err_theta))
        _close(native.pose_jump_distance(tuple(now), tuple(previous)),
               _feedback_reference(feedback.detect_pose_jump, now, previous, 0.3)[1])
        _close(native.odom_drift(tuple(sdk_disp), tuple(slam_disp)),
               _feedback_reference(feedback.odom_drift, sdk_disp, slam_disp))


def test_measure_tick_dt_matches_python_reference_and_reason_text():
    rng = random.Random(20260920)
    cases = [(1.0, None, 0.004, 0.04),
             (1.0, 1.0, 0.004, 0.04),
             (0.99, 1.0, 0.004, 0.04),
             (1.2, 1.0, 0.004, 0.04),
             (float('nan'), 1.0, 0.004, 0.04),
             (1.0, float('inf'), 0.004, 0.04)]
    cases.extend((rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0),
                  rng.uniform(1e-5, 0.2), rng.uniform(1e-5, 0.3))
                 for _ in range(10000))
    for args in cases:
        try:
            expected = _reference(bridge.measure_tick_dt, *args)
        except Exception as expected_error:  # noqa: BLE001
            expected = type(expected_error)
        try:
            actual = bridge.measure_tick_dt(*args)
        except Exception as actual_error:  # noqa: BLE001
            actual = type(actual_error)
        if isinstance(expected, type):
            assert actual is expected
        else:
            assert actual.clamped == expected.clamped
            assert actual.reason == expected.reason
            if expected.raw is None:
                assert actual.raw is None
            else:
                _close(actual.raw, expected.raw)
            _close(actual.dt, expected.dt)


def test_invalid_inputs_keep_python_error_contract():
    for call in (
        lambda: bridge.integrate_step_dt([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0),
        lambda: bridge.integrate_step([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0),
        lambda: bridge.to_local_velocity([0.0, 0.0, 0.0], "invalid", 0.0),
    ):
        with pytest.raises(bridge.ChassisConfigError):
            call()


def test_nonfinite_angle_boundaries_match_python_reference():
    for theta in (float("nan"), float("inf"), float("-inf")):
        for bridge_call, args in (
            (bridge.wrap_angle, (theta,)),
            (bridge.pose_error, ([0.0, 0.0, theta], [0.0, 0.0, 0.0])),
            (bridge.local_pose_displacement, ([0.0, 0.0, theta], [0.0, 0.0, 0.0])),
        ):
            try:
                actual = bridge_call(*args)
            except Exception as actual_error:  # noqa: BLE001
                actual = type(actual_error)
            try:
                expected = _reference(bridge_call, *args)
            except Exception as expected_error:  # noqa: BLE001
                expected = type(expected_error)
            if isinstance(expected, type):
                assert actual is expected
            else:
                _close(actual, expected)


def test_chassis_odom_source_replay_matches_python_state_and_frames():
    rng = random.Random(20260922)
    old = odom._native_odom
    try:
        for velocity_frame in ('body', 'world'):
            samples = []
            pose = [0.0, 0.0, 0.0]
            for index in range(500):
                if index in (120, 360):
                    pose[0] += 0.8
                else:
                    pose[0] += rng.uniform(-0.04, 0.04)
                    pose[1] += rng.uniform(-0.04, 0.04)
                pose[2] += rng.uniform(-0.8, 0.8)
                samples.append((list(pose), [rng.uniform(-1.0, 1.0) for _ in range(3)]))

            odom._native_odom = None
            expected = odom.ChassisOdomSource(0.3, velocity_frame)
            expected_rows = [expected.sample(pos, vel) for pos, vel in samples]
            odom._native_odom = native
            actual = odom.ChassisOdomSource(0.3, velocity_frame)
            actual_rows = [actual.sample(pos, vel) for pos, vel in samples]
            for lhs, rhs in zip(actual_rows, expected_rows):
                _close((lhs.x, lhs.y, lhs.theta, lhs.vx_body, lhs.vy_body,
                        lhs.wz, lhs.jump_m, lhs.jumped),
                       (rhs.x, rhs.y, rhs.theta, rhs.vx_body, rhs.vy_body,
                        rhs.wz, rhs.jump_m, rhs.jumped), tol=3e-12)
            assert actual.stats.samples == expected.stats.samples
            assert actual.stats.jumps == expected.stats.jumps
            _close(actual.stats.max_jump_m, expected.stats.max_jump_m)
            _close(actual.stats.travelled_m, expected.stats.travelled_m)
            assert actual.stats.jump_history == expected.stats.jump_history
            _close(actual.jump_ratio, expected.jump_ratio)
    finally:
        odom._native_odom = old


def test_chassis_odom_source_invalid_inputs_keep_python_boundary():
    old = odom._native_odom
    try:
        for use_native in (False, True):
            odom._native_odom = native if use_native else None
            with pytest.raises(odom.OdomSourceError):
                odom.ChassisOdomSource(0.0, 'body')
            with pytest.raises(odom.OdomSourceError):
                odom.ChassisOdomSource(0.3, 'map')
            source = odom.ChassisOdomSource(0.3, 'body')
            with pytest.raises(odom.OdomSourceError):
                source.sample(None, [0.0, 0.0, 0.0])
            with pytest.raises(odom.OdomSourceError):
                source.sample([0.0, 0.0], [0.0, 0.0, 0.0])
    finally:
        odom._native_odom = old


def test_stateful_integrator_and_history_replay():
    poses = [
        ([0.0, 0.0, 0.0], 0.0),
        ([0.2, 0.1, 0.05], 0.1),
        ([0.4, 0.1, 0.10], 0.2),
        ([0.1, -0.1, 6.35], 0.3),
    ]
    bridge._native = None
    reference = bridge._PythonPoseFrameIntegrator(poses[0][0], poses[0][1], [10.0, 20.0, 30.0])
    history_reference = bridge._PythonSdkPoseHistory(0.25)
    bridge._native = native
    candidate = native.PoseFrameIntegrator(tuple(poses[0][0]), poses[0][1], (10.0, 20.0, 30.0))
    history_candidate = native.SdkPoseHistory(0.25)

    for pose, stamp in poses:
        bridge._native = None
        reference.observe(pose, stamp)
        history_reference.append(stamp, [stamp, 2.0 * stamp, 3.0 * stamp])
        bridge._native = native
        candidate.observe(tuple(pose), stamp)
        history_candidate.append(stamp, (stamp, 2.0 * stamp, 3.0 * stamp))
        _close(candidate.anchor(), reference.anchor)
        _close(candidate.integral(), reference.integral)
        _close(candidate.velocity(), reference.velocity)

        velocity = [0.1, -0.2, 0.3]
        times = [0.5, 0.5, 0.25]
        bridge._native = None
        _close(candidate.target(tuple(velocity), 0.01), reference.target(velocity, 0.01))
        _close(candidate.preview_target(tuple(velocity), 0.01, tuple(times), 0.2, 0.34),
               reference.preview_target(velocity, 0.01, times, 0.2, 0.34))
        reference.commit_preview(velocity, 0.01)
        bridge._native = native
        candidate.commit_preview(tuple(velocity), 0.01)
        _close(candidate.integral(), reference.integral)

    for stamp in (-0.1, 0.05, 0.18, 0.31, 0.5):
        bridge._native = None
        expected = history_reference.at(stamp)
        bridge._native = native
        _close(history_candidate.at(stamp), expected)
