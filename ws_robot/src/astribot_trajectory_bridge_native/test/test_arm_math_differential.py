import math
import random
import sys
from pathlib import Path

_src = Path(__file__).resolve().parents[2]
for _package in ("astribot_s1_navigation",):
    sys.path.insert(0, str(_src / _package))
sys.path.insert(0, str(_src / "astribot_s1_dynamics_coupling" / "test" / "reference"))

from astribot_trajectory_bridge import arm_traj_math as arm
from astribot_trajectory_bridge import gripper_math as gripper
from astribot_s1_dynamics_coupling import arm_reach_metric as dynamics_reach
from astribot_s1_navigation import arm_reach_metric as navigation_reach
from astribot_trajectory_bridge_native import _chassis_math_native as native


def _close(actual, expected, tol=3e-12):
    if isinstance(actual, (list, tuple)):
        assert len(actual) == len(expected)
        for lhs, rhs in zip(actual, expected):
            _close(lhs, rhs, tol)
        return
    if math.isnan(actual) and math.isnan(expected):
        return
    assert math.isclose(actual, expected, rel_tol=tol, abs_tol=tol), (actual, expected)


def _reference(module, call, *args, **kwargs):
    old = module._native
    module._native = None
    try:
        return call(*args, **kwargs)
    finally:
        module._native = old


def test_trajectory_interpolation_randomized():
    rng = random.Random(20260920)
    arm._native = native
    for _ in range(5000):
        dof = rng.randint(1, 12)
        count = rng.randint(2, 8)
        times = []
        value = 0.0
        for _ in range(count):
            value += rng.uniform(0.001, 0.5)
            times.append(value)
        positions = [[rng.uniform(-3.0, 3.0) for _ in range(dof)] for _ in range(count)]
        velocities = [[rng.uniform(-2.0, 2.0) for _ in range(dof)] for _ in range(count)]
        query = rng.uniform(-0.5, times[-1] + 0.5)
        mode = rng.choice((arm.INTERP_LINEAR, arm.INTERP_CUBIC))
        actual = arm.interpolate_trajectory(times, positions, velocities, query, mode)
        expected = _reference(arm, arm.interpolate_trajectory, times, positions, velocities, query, mode)
        _close(actual, expected)


def test_trajectory_boundaries_and_fallbacks():
    arm._native = native
    times = [0.0, 1.0]
    positions = [[0.0, 1.0], [1.0, 3.0]]
    assert arm.interpolate_trajectory(times, positions, None, -1.0) == positions[0]
    assert arm.interpolate_trajectory(times, positions, None, 2.0) == positions[-1]
    valid_velocities = [[0.0, 0.0], [0.0, 0.0]]
    _close(arm.interpolate_trajectory(times, positions, valid_velocities, 0.5, arm.INTERP_LINEAR),
           _reference(arm, arm.interpolate_trajectory, times, positions, valid_velocities,
                      0.5, arm.INTERP_LINEAR))
    for call in (
        lambda: arm.interpolate_trajectory([0.0, 0.0], positions, None, 0.5),
        lambda: arm.interpolate_trajectory(times, positions, None, 0.5, "bad"),
    ):
        try:
            call()
        except arm.ArmConfigError:
            pass
        else:
            raise AssertionError("expected ArmConfigError")


def test_gripper_conversions_and_boundaries():
    gripper._native = native
    for value in (-20.0, 0.0, 0.5, 50.0, 100.0, 150.0):
        _close(gripper.clamp_cmd(value), _reference(gripper, gripper.clamp_cmd, value))
        _close(gripper.cmd_to_rad(value), _reference(gripper, gripper.cmd_to_rad, value))
        _close(gripper.rad_to_cmd(value * 0.0093),
               _reference(gripper, gripper.rad_to_cmd, value * 0.0093))
        _close(gripper.cmd_to_opening_fraction(value),
               _reference(gripper, gripper.cmd_to_opening_fraction, value))
    for fraction in (-1.0, 0.0, 0.2, 1.0, 2.0):
        _close(gripper.opening_fraction_to_cmd(fraction),
               _reference(gripper, gripper.opening_fraction_to_cmd, fraction))
    for value in (0.0, 50.0, 100.0):
        assert gripper.is_cmd_in_range(value) == _reference(gripper, gripper.is_cmd_in_range, value)


def test_reach_metrics_match_python_reference():
    dynamics_reach._native = native
    navigation_reach._native = native
    for x, y in ((0.0, 0.0), (0.42, -0.17), (-0.8, 0.3)):
        _close(dynamics_reach.horizontal_reach(x, y),
               _reference(dynamics_reach, dynamics_reach.horizontal_reach, x, y))
        _close(navigation_reach.horizontal_reach(x, y),
               _reference(navigation_reach, navigation_reach.horizontal_reach, x, y))
    for value in (-0.2, 0.0, 0.3, 0.9, 2.0):
        _close(dynamics_reach.reach_activity(value, 0.42, 0.8865),
               _reference(dynamics_reach, dynamics_reach.reach_activity, value, 0.42, 0.8865))
        _close(dynamics_reach.scale_from_activity(value, 0.35),
               _reference(dynamics_reach, dynamics_reach.scale_from_activity, value, 0.35))
    for was_extended in (False, True):
        assert navigation_reach.is_extended_by_reach(0.5, 0.5, 0.02, was_extended) == \
            _reference(navigation_reach, navigation_reach.is_extended_by_reach,
                       0.5, 0.5, 0.02, was_extended)
