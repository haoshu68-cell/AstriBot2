#!/usr/bin/env python3
"""Offline checks against one explicitly named native publication-lease binary."""
import argparse
from fractions import Fraction
import hashlib
import importlib.util
import json
from pathlib import Path
import time
import unittest


class PublicationLeaseBinding(unittest.TestCase):
    def evaluate(self, now=299999999, decision=250000000, epochs=(1, 1), cap=0.2,
                 intervals=((0, 300000000),), valid=True):
        return NATIVE.evaluate_publication_lease(now, decision, *epochs, cap, intervals, valid)

    def test_expired_exact_deadline_and_publication_overrun(self):
        for now in (300000000, 305000000, 322000000):
            result = self.evaluate(now=now)
            self.assertEqual(result, dict(allowed=False, lease_s=0.0, deadline_ns=300000000,
                                         reason="SOURCE_EXPIRED"))

    def test_one_ns_and_rounding_with_exact_rational_reference(self):
        for remaining in (1, 2, 3, 7, 999, 99999999, 299999999, 499999999, 500000000):
            result = self.evaluate(now=0, decision=0, cap=0.5, intervals=((0, remaining),))
            self.assertTrue(result["allowed"])
            self.assertGreater(result["lease_s"], 0)
            self.assertLessEqual(Fraction.from_float(result["lease_s"]), Fraction(remaining, 10**9))
            self.assertEqual(result["deadline_ns"], remaining)

    def test_epoch_clock_missing_future_invalid_and_integer_extremes(self):
        invalid = [dict(epochs=(0, 1)), dict(now=1), dict(intervals=()), dict(valid=False),
                   dict(intervals=((300000000, 400000000),)), dict(cap=float("nan")),
                   dict(cap=float("inf")), dict(cap=0), dict(cap=0.5001),
                   dict(now=-(2**63)), dict(decision=-(2**63)),
                   dict(intervals=((-(2**63), 2**63-1),))]
        for values in invalid:
            with self.subTest(values=values):
                result = self.evaluate(**values)
                self.assertFalse(result["allowed"])
                self.assertEqual(result["lease_s"], 0)
        result = self.evaluate(now=2**63-2, decision=2**63-3, epochs=(2**64-1, 2**64-1),
                               intervals=((2**63-4, 2**63-1),))
        self.assertTrue(result["allowed"])
        self.assertLessEqual(Fraction.from_float(result["lease_s"]), Fraction(1, 10**9))
        with self.assertRaises(TypeError):
            self.evaluate(now=2**63)

    def test_minimum_source_and_no_deadline_renewal(self):
        intervals = ((0, 300000000), (100000000, 200000000))
        early = self.evaluate(now=150000000, decision=140000000, intervals=intervals)
        late = self.evaluate(now=199999999, decision=140000000, intervals=intervals)
        self.assertEqual(early["deadline_ns"], 200000000)
        self.assertEqual(late["deadline_ns"], early["deadline_ns"])
        self.assertLess(late["lease_s"], early["lease_s"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--binding", required=True, type=Path)
    args, extra = parser.parse_known_args()
    print(json.dumps({"binding": str(args.binding.resolve()),
                      "sha256": hashlib.sha256(args.binding.read_bytes()).hexdigest()}), flush=True)
    spec = importlib.util.spec_from_file_location("_navigation_math_native", args.binding)
    NATIVE = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(NATIVE)
    unittest.main(argv=[__file__] + extra, verbosity=2)
