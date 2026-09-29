import math
import random

from astribot_s1_chassis_effort_drive import omni_effort_drive_node as drive


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _reference_update(state, target, measured, kp, ki, kd, tau_c, tau_v,
                      deadband, tau_max, dt):
    integral, previous = state
    error = target - measured
    tau_ff = 0.0
    if abs(target) > deadband:
        tau_ff = tau_c * (1.0 if target > 0.0 else -1.0) + tau_v * measured
    derivative = (error - previous) / dt if dt > 1e-9 else 0.0
    unclamped = kp * error + ki * integral + kd * derivative + tau_ff
    tau = _clamp(unclamped, -tau_max, tau_max)
    if tau == unclamped:
        integral += error * dt
    return (integral, error), (tau, error)


def test_wheel_pid_randomized_same_input_replay():
    rng = random.Random(20260920)
    cases = [(
        rng.uniform(-50.0, 50.0), rng.uniform(-30.0, 30.0),
        rng.uniform(0.0, 2.0), rng.uniform(0.0, 0.5), rng.uniform(0.0, 0.2),
        rng.uniform(0.0, 0.5), rng.uniform(0.0, 2.0), rng.uniform(0.0, 0.2),
        rng.uniform(0.01, 20.0), rng.choice((0.0, 1e-12, 0.001, 0.01)))
        for _ in range(10000)]
    loop = drive._WheelLoop()
    state = (0.0, 0.0)
    for args in cases:
        actual = loop.update(*args)
        state, expected = _reference_update(state, *args)
        assert math.isclose(actual[0], expected[0], rel_tol=3e-12, abs_tol=3e-12)
        assert math.isclose(actual[1], expected[1], rel_tol=3e-12, abs_tol=3e-12)
    assert math.isclose(loop.integral, state[0], rel_tol=3e-12, abs_tol=3e-12)
    assert math.isclose(loop.prev_error, state[1], rel_tol=3e-12, abs_tol=3e-12)


def test_wheel_pid_reset_and_saturation_do_not_integrate():
    loop = drive._WheelLoop()
    cases = [(100.0, 0.0, 0.4, 0.1, 0.0, 0.1, 1.0, 0.05, 0.01, 0.01)] * 4
    for args in cases:
        tau, error = loop.update(*args)
        assert abs(tau) <= 0.01 + 1e-12
        assert error == 100.0
    assert loop.integral == 0.0
    loop.reset()
    assert loop.integral == 0.0
    assert loop.prev_error == 0.0


def test_nonfinite_inputs_keep_python_boundary():
    loop = drive._WheelLoop()
    tau, error = loop.update(math.nan, 0.0, 0.4, 0.1, 0.0,
                             0.1, 1.0, 0.05, 15.0, 0.01)
    assert tau == 15.0
    assert math.isnan(error)
    assert math.isnan(loop.prev_error)


def test_inverse_kinematics_randomized_and_zero_radius():
    rng = random.Random(20260920)
    values = {
        'wheel_radius': 0.5,
        'wheel_coeff_vx': [1.0, -1.0, 1.0, -1.0],
        'wheel_coeff_vy': [0.0] * 4,
        'wheel_coeff_wz': [0.0] * 4,
        'kinematics_global_sign': 1.0,
    }
    class Params:
        def _safe_param(self, name, default):
            return values.get(name, default)
        class Logger:
            def error(self, *_args):
                pass
        def get_logger(self):
            return self.Logger()
    params = Params()
    for _ in range(10000):
        vx, vy, wz = (rng.uniform(-2.0, 2.0) for _ in range(3))
        expected = [values['kinematics_global_sign'] / values['wheel_radius'] *
                    (values['wheel_coeff_vx'][i] * vx +
                     values['wheel_coeff_vy'][i] * vy +
                     values['wheel_coeff_wz'][i] * wz) for i in range(4)]
        actual = drive.OmniEffortDriveNode._inverse_kinematics(params, vx, vy, wz)
        assert actual == dict(zip(drive.JOINT_NAMES, expected))
    values['wheel_radius'] = 0.0
    assert drive.OmniEffortDriveNode._inverse_kinematics(params, 1.0, 2.0, 3.0) == {
        name: 0.0 for name in drive.JOINT_NAMES}
    assert drive._clamp(0.0, -0.0, 0.0) == 0.0
