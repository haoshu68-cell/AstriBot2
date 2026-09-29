"""Message/parameter differential for the direct scan adapter on an owned domain."""
from pathlib import Path as _ReferencePath
import sys as _reference_sys
_reference_sys.path.insert(0, str(_ReferencePath(__file__).resolve().parents[4] / "tools/migration/python_reference"))
from reference_bootstrap import REFERENCE_ROOT, enable as _enable_references
_enable_references()

import copy
import math
import os
from pathlib import Path
import signal
import subprocess
import struct
import sys
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import LaserScan

POLICY = Path(__file__).resolve().parents[2] / 'astribot_s1_navigation_policy'
sys.path.insert(0, str(POLICY))
from astribot_s1_navigation_policy import protection

MAIN = '''from reference_bootstrap import enable
enable()

import rclpy
from rclpy.signals import SignalHandlerOptions
from astribot_s1_navigation_policy.costmap_scan_node import CostmapScanAdapter
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = CostmapScanAdapter()
try: rclpy.spin(node)
except KeyboardInterrupt: pass
finally:
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
'''


def scan_values(message):
    # CDR alignment padding is not a message field and can contain uninitialized
    # bytes. Compare every field at its wire precision, including NaN bit values.
    scalars = ('angle_min', 'angle_max', 'angle_increment', 'time_increment',
               'scan_time', 'range_min', 'range_max')
    return (message.header.stamp.sec, message.header.stamp.nanosec, message.header.frame_id,
            tuple(struct.pack('=f', getattr(message, field)) for field in scalars),
            message.ranges.tobytes(), message.intensities.tobytes())


class ScanReplay:
    def __init__(self, impl, binary, log):
        self.context = self.node = self.executor = self.proc = None
        self.log = log.open('w')
        self.events = []
        try:
            env = dict(os.environ, ROS_DOMAIN_ID='110', ROS_LOCALHOST_ONLY='1',
                       ASTRIBOT_NAV_NATIVE_KERNELS='0')
            env['PYTHONPATH'] = str(REFERENCE_ROOT) + os.pathsep + str(POLICY) + os.pathsep + env.get('PYTHONPATH', '')
            command = [binary] if impl == 'cpp' else [sys.executable, '-c', MAIN]
            command += ['--ros-args', '-p', 'scan_topic:=/migration/input_scan']
            self.proc = subprocess.Popen(command, env=env, stdout=self.log, stderr=subprocess.STDOUT)
            os.environ['ROS_LOCALHOST_ONLY'] = '1'
            self.context = Context()
            rclpy.init(context=self.context, domain_id=110)
            self.node = Node('scan_replay', context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.output = self.node.create_subscription(LaserScan, '/navigation_policy/costmap_scan',
                                                       self.events.append, qos_profile_sensor_data)
            self.input = self.node.create_publisher(LaserScan, '/migration/input_scan', qos_profile_sensor_data)
            self.parameters = self.node.create_client(SetParameters,
                '/navigation_costmap_scan_adapter/set_parameters')
            self.until(lambda: self.parameters.service_is_ready() and self.input.get_subscription_count() == 1
                       and self.node.count_publishers('/navigation_policy/costmap_scan') == 1)
            # DDS graph discovery can precede the best-effort data path. Retry
            # only the setup probe; inputs under test are always sent once.
            last_probe = [0.]
            def connected():
                if time.monotonic() - last_probe[0] >= .05:
                    self.scan(0, [1.])
                    last_probe[0] = time.monotonic()
                return any(message.header.stamp.sec == 0 for message in self.events)
            self.until(connected)
            settle = time.monotonic() + .1
            self.until(lambda: time.monotonic() >= settle)
            self.events.clear()
        except BaseException:
            self.close()
            raise

    def until(self, condition, timeout=5):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            assert self.proc.poll() is None, 'adapter exited'
            self.executor.spin_once(timeout_sec=.01)
            if condition(): return
        raise AssertionError('scan protocol timeout')

    def set_range(self, value):
        request = SetParameters.Request(parameters=[Parameter(name='max_marking_range_m',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))])
        future = self.parameters.call_async(request)
        self.until(future.done)
        assert all(result.successful for result in future.result().results)

    def scan(self, index, ranges, maximum=8.):
        message = LaserScan()
        message.header.stamp.sec = index
        message.header.frame_id = 'migration_scan_frame'
        message.angle_min, message.angle_max, message.angle_increment = -1., 1., .01
        message.time_increment, message.scan_time = .001, .1
        message.range_min, message.range_max = .05, maximum
        message.ranges = ranges
        message.intensities = [float(i) for i in range(len(ranges))]
        self.input.publish(message)
        return message

    def expect(self, message, marking_range):
        self.until(lambda: any(x.header.stamp == message.header.stamp for x in self.events))
        received = next(x for x in self.events if x.header.stamp == message.header.stamp)
        expected = copy.deepcopy(message)
        previous = protection._native
        try:
            protection._native = None
            expected.ranges = protection.costmap_clearing_ranges(message.ranges, message.range_max, marking_range)
        finally:
            protection._native = previous
        assert scan_values(received) == scan_values(expected)

    def close(self):
        if self.executor is not None: self.executor.shutdown()
        if self.node is not None: self.node.destroy_node()
        if self.context is not None and self.context.ok(): self.context.shutdown()
        if self.proc is not None:
            if self.proc.poll() is None: self.proc.send_signal(signal.SIGINT)
            try: self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=3)
        self.log.close()


@pytest.fixture(params=['python', 'cpp'])
def scan(request, tmp_path):
    binary = os.environ.get('COSTMAP_SCAN_CPP_BINARY')
    if not binary: pytest.skip('COSTMAP_SCAN_CPP_BINARY required')
    replay = ScanReplay(request.param, binary, tmp_path / 'scan.log')
    try: yield replay
    finally: replay.close()


def test_metadata_nonreturns_and_float32_edges(scan):
    for i, ranges in enumerate([[], [math.inf, math.nan, -math.inf, 0., .05, 1., 8., 8.1],
                                [7.9999995, 8., 8.000001],
                                [.5, 1., math.inf, 8., math.nan] * 216], start=1):
        scan.expect(scan.scan(i, ranges), 5.5)


def test_runtime_marking_parameter_takes_effect(scan):
    scan.set_range(1.)
    scan.expect(scan.scan(1, [math.inf, 2., .5], maximum=2.), 1.)
    scan.set_range(0.)
    scan.expect(scan.scan(2, [math.inf, .1], maximum=.5), 0.)


def test_rejected_scan_does_not_publish_and_recovers(scan):
    # A valid later sample acts as a stream barrier after invalid input.
    scan.scan(1, [math.inf], maximum=5.5)
    scan.expect(scan.scan(2, [math.inf]), 5.5)
    assert not any(msg.header.stamp.sec == 1 for msg in scan.events)
    scan.set_range(-1.)
    scan.scan(3, [math.inf])
    # Keep the invalid parameter active until the next valid-range change; use
    # a bounded delivery window so the callback is not racing the update.
    end = time.monotonic() + .15
    scan.until(lambda: time.monotonic() >= end)
    assert not any(msg.header.stamp.sec == 3 for msg in scan.events)
    scan.set_range(1.)
    scan.expect(scan.scan(4, [math.inf]), 1.)


@pytest.mark.parametrize('impl', ['python', 'cpp'])
def test_invalid_parameter_type_reports_startup_failure(impl):
    binary = os.environ.get('COSTMAP_SCAN_CPP_BINARY')
    if not binary: pytest.skip('COSTMAP_SCAN_CPP_BINARY required')
    command = [binary] if impl == 'cpp' else [sys.executable, '-c', MAIN]
    command += ['--ros-args', '-p', 'scan_topic:=42']
    env = dict(os.environ, ROS_DOMAIN_ID='110', ROS_LOCALHOST_ONLY='1')
    env['PYTHONPATH'] = str(REFERENCE_ROOT) + os.pathsep + str(POLICY) + os.pathsep + env.get('PYTHONPATH', '')
    result = subprocess.run(command, env=env, capture_output=True, text=True, timeout=5)
    assert result.returncode != 0, result.stdout + result.stderr


@pytest.mark.parametrize('field,value', [('maximum', math.nan), ('maximum', math.inf),
    ('maximum', 0.), ('maximum', -1.), ('marking', math.nan), ('marking', math.inf)])
def test_nonfinite_or_unusable_limits_reject_then_recover(scan, field, value):
    if field == 'marking': scan.set_range(value)
    scan.scan(1, [math.inf], maximum=value if field == 'maximum' else 8.)
    end = time.monotonic() + .15
    scan.until(lambda: time.monotonic() >= end)
    assert not any(msg.header.stamp.sec == 1 for msg in scan.events)
    scan.set_range(5.5)
    scan.expect(scan.scan(2, [math.inf]), 5.5)
