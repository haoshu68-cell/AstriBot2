import math
import random

import pytest

from astribot_s1_navigation import posture_monitor_policy as policy
from astribot_s1_navigation_policy_native import _navigation_math_native as native


def _reference(call, *args, **kwargs):
    old = policy._native
    policy._native = None
    try:
        return call(*args, **kwargs)
    finally:
        policy._native = old


def test_posture_threshold_reasons_match_python_reference():
    rng = random.Random(20260922)
    old = policy._native
    try:
        for _ in range(1000):
            args = (
                rng.uniform(0.2, 1.2), rng.uniform(-0.8, 0.8),
                rng.uniform(-0.8, 0.8), rng.uniform(0.4, 0.8),
                rng.uniform(0.01, 0.4), rng.uniform(0.05, 0.8))
            expected = _reference(policy.posture_out_of_bounds, *args)
            policy._native = native
            actual = policy.posture_out_of_bounds(*args)
            assert actual == expected
        for value in (math.nan, math.inf, -math.inf):
            policy._native = None
            expected = policy.posture_out_of_bounds(value, 0.0, 0.0, 0.6, 0.1, 0.2)
            policy._native = native
            actual = policy.posture_out_of_bounds(value, 0.0, 0.0, 0.6, 0.1, 0.2)
            assert actual == expected
    finally:
        policy._native = old


def test_posture_window_and_monitor_text_match_python_reference():
    rng = random.Random(20260922)
    old = policy._native
    try:
        for _ in range(300):
            samples = [(rng.uniform(0.4, 0.8), rng.uniform(-0.2, 0.2),
                        rng.uniform(-0.2, 0.2))
                       for _ in range(rng.randint(0, 30))]
            eps = rng.choice((1e-9, 1e-4, 0.1))
            minimum = rng.randint(1, 25)
            expected = _reference(policy.is_degenerate_attitude_source,
                                  samples, eps, minimum)
            policy._native = native
            actual = policy.is_degenerate_attitude_source(samples, eps, minimum)
            assert actual == expected

            args = (rng.choice((False, True)), samples,
                    rng.uniform(0.4, 0.8), rng.uniform(-0.3, 0.3),
                    rng.uniform(-0.3, 0.3), 0.6, 0.08, 0.2, minimum)
            expected = _reference(policy.evaluate_posture, *args)
            policy._native = native
            actual = policy.evaluate_posture(*args)
            assert actual == expected

        for enabled in (False, True):
            for degenerate in (False, True):
                for tripped in (False, True):
                    expected = _reference(policy.describe_monitor_state,
                                          enabled, degenerate, tripped)
                    policy._native = native
                    actual = policy.describe_monitor_state(enabled, degenerate,
                                                           tripped)
                    assert actual == expected
    finally:
        policy._native = old


def test_posture_malformed_sample_keeps_python_boundary():
    old = policy._native
    try:
        policy._native = native
        assert policy.evaluate_posture(
            True, [(0.6, 0.0)], 0.6, 0.0, 0.0, 0.6, 0.1, 0.2,
            min_samples=20) == (policy.ACT_COLLECTING, None)
        with pytest.raises(IndexError):
            policy.is_degenerate_attitude_source([(0.6, 0.0)] * 20)
    finally:
        policy._native = old
