#!/usr/bin/env python3
"""Isolated ROS integration contract for the opt-in native TF receiver.

Only publishes synthetic TF and clock, never commands/ACKs. Run in an explicitly
owned empty domain. --binding is required so an installed module cannot mask a
missing or stale candidate.
"""
import argparse
import ctypes
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time
import threading
import unittest


def load_binding(path):
    spec = importlib.util.spec_from_file_location("_native_tf_buffer", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class NativeTfContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from rclpy.time import Time
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        import tf2_ros
        cls.rclpy, cls.Time, cls.Transform = rclpy, Time, TransformStamped
        cls.Clock, cls.TFMessage, cls.tf2 = Clock, TFMessage, tf2_ros
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node("native_tf_contract_publisher", context=cls.context)
        cls.dynamic = cls.node.create_publisher(TFMessage, "/tf", QoSProfile(depth=100))
        cls.static = cls.node.create_publisher(TFMessage, "/tf_static", QoSProfile(
            depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.clock = cls.node.create_publisher(Clock, "/clock", QoSProfile(depth=10))
        cls.static.publish(TFMessage(transforms=[cls.transform("static_root", "static_child", 1, 7)]))
        cls.bridge = BINDING.NativeTfBuffer(True, "native_tf_contract_receiver")
        cls.wait(lambda: cls.dynamic.get_subscription_count() == 1)
        cls.wait(lambda: cls.bridge.can_transform("static_root", "static_child", Time(seconds=1)))
        print("private_context=" + json.dumps(cls.bridge.diagnostics()), flush=True)

    @classmethod
    def tearDownClass(cls):
        cls.bridge.close()
        cls.node.destroy_node()
        cls.context.shutdown()

    @classmethod
    def wait(cls, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        raise AssertionError("condition did not become true within steady deadline")

    @classmethod
    def transform(cls, parent, child, seconds, x):
        msg = cls.Transform()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.header.stamp = cls.Time(seconds=seconds).to_msg()
        msg.transform.translation.x = float(x)
        msg.transform.rotation.w = 1.0
        return msg

    def setUp(self):
        self.bridge.clear()
        self.clock.publish(self.Clock(clock=self.Time(seconds=100).to_msg()))
        self.wait(lambda: self.bridge.diagnostics()["clock_ns"] == 100_000_000_000)

    def send(self, transforms):
        self.dynamic.publish(self.TFMessage(transforms=transforms))
        last = transforms[-1]
        self.wait(lambda: self.bridge.can_transform(
            last.header.frame_id, last.child_frame_id,
            self.Time.from_msg(last.header.stamp)))

    def test_01_exact_interpolation_and_standard_message(self):
        self.send([self.transform("odom", "base", 100, 0), self.transform("odom", "base", 101, 2)])
        stamp = self.Time(seconds=100, nanoseconds=500_000_000)
        result = self.bridge.lookup_transform("odom", "base", stamp)
        self.assertIsInstance(result, self.Transform)
        self.assertEqual(result.header.stamp, stamp.to_msg())
        self.assertAlmostEqual(result.transform.translation.x, 1.0)
        self.assertEqual(self.bridge.can_transform("odom", "base", stamp, True), (True, ""))

    def test_02_missing_disconnected_future_past_and_invalid_exceptions(self):
        self.send([self.transform("a", "b", 100, 0), self.transform("a", "b", 101, 1),
                   self.transform("c", "d", 100, 0)])
        cases = [("a", "missing", 100, self.tf2.LookupException),
                 ("a", "d", 100, self.tf2.ConnectivityException),
                 ("a", "b", 99, self.tf2.ExtrapolationException),
                 ("a", "b", 102, self.tf2.ExtrapolationException),
                 ("", "b", 100, self.tf2.InvalidArgumentException)]
        for target, source, stamp, exception in cases:
            with self.subTest(target=target, source=source, stamp=stamp):
                ready, error = self.bridge.can_transform(target, source, self.Time(seconds=stamp), True)
                self.assertFalse(ready)
                self.assertTrue(error)
                with self.assertRaises(exception):
                    self.bridge.lookup_transform(target, source, self.Time(seconds=stamp))

    def test_03_cache_is_ten_seconds_and_zero_is_only_explicit_latest(self):
        self.send([self.transform("cache_root", "cache_child", 100, 0),
                   self.transform("cache_root", "cache_child", 105, 5),
                   self.transform("cache_root", "cache_child", 111, 11),
                   self.transform("cache_root", "cache_child", 112, 12)])
        self.assertFalse(self.bridge.can_transform("cache_root", "cache_child", self.Time(seconds=100)))
        self.assertTrue(self.bridge.can_transform("cache_root", "cache_child", self.Time(seconds=105)))
        self.assertEqual(self.bridge.lookup_transform("cache_root", "cache_child", self.Time()).transform.translation.x, 12)
        with self.assertRaises(self.tf2.ExtrapolationException):
            self.bridge.lookup_transform("cache_root", "cache_child", self.Time(seconds=113))

    def test_04_static_late_join_survives_clear_and_arbitrary_time(self):
        self.bridge.clear()
        self.assertEqual(self.bridge.lookup_transform("static_root", "static_child", self.Time(seconds=999)).transform.translation.x, 7)

    def test_05_frozen_clock_never_waits_for_future(self):
        self.send([self.transform("freeze_root", "freeze_child", 100, 0)])
        start = time.monotonic()
        for _ in range(100):
            self.assertFalse(self.bridge.can_transform("freeze_root", "freeze_child", self.Time(seconds=101)))
        self.assertLess(time.monotonic() - start, 0.5)
        self.assertEqual(self.bridge.diagnostics()["clock_ns"], 100_000_000_000)

    def test_06_clock_rollback_keeps_latest_source_tf_available(self):
        self.send([self.transform("rollback_root", "rollback_child", 100, 1)])
        epoch = self.bridge.diagnostics()["clock_epoch"]
        self.clock.publish(self.Clock(clock=self.Time(seconds=1).to_msg()))
        self.wait(lambda: self.bridge.diagnostics()["clock_epoch"] > epoch)
        self.assertTrue(self.bridge.can_transform("rollback_root", "rollback_child", self.Time()))
        self.assertEqual(self.bridge.lookup_transform("rollback_root", "rollback_child", self.Time()).transform.translation.x, 1)
        self.send([self.transform("rollback_root", "rollback_child", 101, 3)])
        self.dynamic.publish(self.TFMessage(transforms=[self.transform("rollback_root", "rollback_child", 100, 9)]))
        time.sleep(0.05)
        self.assertEqual(self.bridge.lookup_transform("rollback_root", "rollback_child", self.Time()).transform.translation.x, 3)

    def test_07_receiver_progress_while_python_holds_gil(self):
        self.dynamic.publish(self.TFMessage(transforms=[self.transform("gil_root", "gil_child", 100, 8)]))
        # PyDLL deliberately holds the GIL across this native sleep; an ordinary
        # Python busy loop periodically yields it and would be a weaker fixture.
        ctypes.PyDLL("libc.so.6").usleep(150_000)
        self.assertTrue(self.bridge.can_transform("gil_root", "gil_child", self.Time(seconds=100)))

    def test_08_private_context_close_is_idempotent_and_joined(self):
        baseline = len(list(Path("/proc/self/task").iterdir()))
        for _ in range(3):
            candidate = BINDING.NativeTfBuffer(False)
            self.assertNotEqual(candidate.diagnostics()["node_name"], self.bridge.diagnostics()["node_name"])
            candidate.close()
            candidate.close()
            self.assertTrue(candidate.diagnostics()["closed"])
            self.assertFalse(candidate.diagnostics()["executor_running"])
            with self.assertRaises(RuntimeError):
                candidate.can_transform("a", "b", self.Time(seconds=100))
        self.wait(lambda: len(list(Path("/proc/self/task").iterdir())) <= baseline)

    def test_09_queries_and_close_are_safe_concurrently(self):
        candidate = BINDING.NativeTfBuffer(False)
        ready = threading.Event()
        errors = []

        def query():
            ready.set()
            for _ in range(1000):
                try:
                    candidate.can_transform("missing_a", "missing_b", self.Time(seconds=1))
                except RuntimeError as error:
                    if "closed" not in str(error):
                        errors.append(str(error))
                    return
                time.sleep(0.0001)

        workers = [threading.Thread(target=query) for _ in range(3)]
        for worker in workers:
            worker.start()
        self.assertTrue(ready.wait(1))
        candidate.close()
        for worker in workers:
            worker.join(1)
            self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])

    def test_10_failed_construction_does_not_leak_context_threads(self):
        baseline = len(list(Path("/proc/self/task").iterdir()))
        with self.assertRaises(Exception):
            BINDING.NativeTfBuffer(False, "invalid node name!")
        self.wait(lambda: len(list(Path("/proc/self/task").iterdir())) <= baseline)

    def test_11_receiver_has_no_publishers_and_expected_qos(self):
        name = self.bridge.diagnostics()["node_name"]
        self.assertEqual(self.node.get_publisher_names_and_types_by_node(name, "/"), [])
        for topic, durability in (("/tf", 2), ("/tf_static", 1)):
            endpoints = [entry for entry in self.node.get_subscriptions_info_by_topic(topic)
                         if entry.node_name == name]
            self.assertEqual(len(endpoints), 1)
            qos = endpoints[0].qos_profile
            # Fast DDS can report depth=0 (unknown) through endpoint discovery.
            # The receiver construction explicitly uses depth=100 for both.
            self.assertIn(qos.depth, (0, 100))
            self.assertEqual(int(qos.reliability), 1)
            self.assertEqual(int(qos.durability), durability)

    def test_12_positive_to_zero_clock_retains_static_transform(self):
        candidate = BINDING.NativeTfBuffer(True)
        try:
            def positive_clock_received():
                self.clock.publish(self.Clock(clock=self.Time(seconds=100).to_msg()))
                return candidate.diagnostics()["clock_ns"] == 100_000_000_000

            self.wait(positive_clock_received)
            self.wait(lambda: candidate.can_transform("static_root", "static_child", self.Time()))
            epoch = candidate.diagnostics()["clock_epoch"]
            self.clock.publish(self.Clock(clock=self.Time().to_msg()))
            self.wait(lambda: candidate.diagnostics()["clock_epoch"] > epoch)
            self.assertTrue(candidate.can_transform("static_root", "static_child", self.Time()))
            self.assertEqual(candidate.lookup_transform("static_root", "static_child", self.Time()).transform.translation.x, 7)
        finally:
            candidate.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--binding", required=True, type=Path)
    parser.add_argument("--domain", required=True, type=int)
    args, remainder = parser.parse_known_args()
    if os.environ.get("ROS_DOMAIN_ID") != str(args.domain):
        parser.error("ROS_DOMAIN_ID must match the explicitly owned --domain")
    print(json.dumps({"binding": str(args.binding.resolve()),
                      "sha256": hashlib.sha256(args.binding.read_bytes()).hexdigest(),
                      "pid": os.getpid(), "domain": os.environ["ROS_DOMAIN_ID"]}), flush=True)
    BINDING = load_binding(args.binding)
    unittest.main(argv=[__file__] + remainder, verbosity=2)
