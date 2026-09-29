"""Real ROS two-context replay; each child has an owned PID and isolated domains.

Detects missing/corrupted forwarding, volatile QoS, reverse/control routing,
changed startup timeout/parameter handling and unreaped shutdown. Oracle is a
frozen file explicitly run by path, never an installed compatibility wrapper.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import uuid

import pytest

HERE = Path(__file__).resolve().parent
BINARY = Path(os.environ.get('MAP_RELAY_CPP', '/missing/map_domain_relay'))
EVIDENCE = Path(os.environ.get('MAP_RELAY_EVIDENCE', '/tmp/codex_map_relay_20260921/ros'))


def qos(volatile=False):
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE if volatile else DurabilityPolicy.TRANSIENT_LOCAL)


def signature(msg):
    """Compare every field and float bit pattern, excluding unspecified CDR padding."""
    origin = msg.info.origin
    return (msg.header.frame_id, msg.header.stamp.sec, msg.header.stamp.nanosec,
            msg.info.map_load_time.sec, msg.info.map_load_time.nanosec,
            msg.info.width, msg.info.height, struct.pack('!f', msg.info.resolution),
            struct.pack('!7d', origin.position.x, origin.position.y, origin.position.z,
                        origin.orientation.x, origin.orientation.y, origin.orientation.z,
                        origin.orientation.w), bytes(msg.data))


class Relay:
    def __init__(self, impl, label, *, timeout=8., params=None, volatile=False,
                 remap=False, start=True):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from nav_msgs.msg import OccupancyGrid
        from geometry_msgs.msg import Twist
        self.rclpy, self.Grid, self.Twist = rclpy, OccupancyGrid, Twist
        self.impl = impl
        self.prefix = 'relay_' + uuid.uuid4().hex[:12]
        self.path = EVIDENCE / label / (impl + '_' + self.prefix)
        self.path.mkdir(parents=True, exist_ok=True)
        self.remote_domain, self.local_domain = 162, 163
        self.contexts, self.nodes, self.executors = [], [], []
        self.p = None
        self.handle = None
        self.extra_subs = []
        self.messages, self.reverse, self.controls = [], [], []
        self.source = '/' + self.prefix + '/source'
        self.destination = '/' + self.prefix + '/destination'
        for suffix, domain in [('remote', self.remote_domain), ('local', self.local_domain)]:
            context = Context()
            rclpy.init(args=[], context=context, domain_id=domain)
            self.contexts.append(context)
            node = rclpy.create_node(self.prefix + '_' + suffix, context=context)
            self.nodes.append(node)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self.executors.append(executor)
        self.publisher = self.nodes[0].create_publisher(OccupancyGrid, self.source, qos(volatile))
        self.subscription = self.nodes[1].create_subscription(OccupancyGrid, self.destination,
            lambda msg: self.messages.append((time.perf_counter_ns(), msg)), qos())
        self.reverse_subscription = self.nodes[0].create_subscription(OccupancyGrid, self.destination,
            lambda msg: self.reverse.append(msg), qos())
        self.control_subscription = self.nodes[0].create_subscription(Twist, '/' + self.prefix + '/cmd_vel',
            lambda msg: self.controls.append(msg), 10)
        self.local_publisher = self.nodes[1].create_publisher(OccupancyGrid, self.destination, qos())
        self.control_publisher = self.nodes[1].create_publisher(Twist, '/' + self.prefix + '/cmd_vel', 10)
        values = {'remote_domain_id': self.remote_domain, 'local_domain_id': self.local_domain,
                  'remote_map_topic': self.source, 'local_map_topic': self.destination,
                  'relay_timeout_sec': timeout}
        values.update(params or {})
        extra = []
        if remap:
            values['remote_map_topic'] = '/' + self.prefix + '/configured_input'
            values['local_map_topic'] = '/' + self.prefix + '/configured_output'
            extra = ['-r', values['remote_map_topic'] + ':=' + self.source,
                     '-r', values['local_map_topic'] + ':=' + self.destination,
                     '-r', '__ns:=/relay_test']
        import yaml
        self.config = self.path / 'params.yaml'
        # Node-scoped parameters require global node-name remapping on the temporary reader.
        key = '/relay_test/' + self.prefix if remap else '/' + self.prefix
        self.node_path = key
        self.config.write_text(yaml.safe_dump({key: {'ros__parameters': values}}))
        command = ([str(BINARY)] if impl == 'cpp' else
                   [sys.executable, str(HERE / 'reference/astribot_s1_perception/map_domain_relay.py')])
        self.command = command + ['--ros-args', '-r', '__node:=' + self.prefix,
                                  '--params-file', str(self.config)] + extra
        if start:
            self.start()

    def start(self):
        assert self.impl != 'cpp' or BINARY.is_file(), 'Native map relay is not implemented/built'
        env = dict(os.environ, ROS_DOMAIN_ID='164', ROS_LOCALHOST_ONLY='1',
                   RMW_IMPLEMENTATION='rmw_fastrtps_cpp', RCUTILS_LOGGING_BUFFERED_STREAM='0')
        self.handle = (self.path / 'node.log').open('w')
        self.p = subprocess.Popen(self.command, stdout=self.handle, stderr=subprocess.STDOUT, env=env)
        proc_env = []
        deadline = time.monotonic() + 2.
        while self.p.poll() is None and time.monotonic() < deadline:
            proc_env = Path(f'/proc/{self.p.pid}/environ').read_bytes().split(b'\0')
            if b'ROS_DOMAIN_ID=164' in proc_env and b'ROS_LOCALHOST_ONLY=1' in proc_env:
                break
            time.sleep(.005)
        assert b'ROS_DOMAIN_ID=164' in proc_env and b'ROS_LOCALHOST_ONLY=1' in proc_env
        self.details = {'pid': self.p.pid, 'command': self.command,
                        'bootstrap_domain': 164, 'remote_domain': 162, 'local_domain': 163}

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            for index, ex in enumerate(self.executors):
                # Publication needs no remote executor wait. Only the local
                # receiving side blocks, so no fixed 4ms polling floor masks C++.
                ex.spin_once(timeout_sec=.004 if index == 1 else 0.)

    def until(self, predicate, timeout=6.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            self.pump(.02)
        assert predicate(), (self.impl, self.command, (self.path / 'node.log').read_text())

    def ready(self):
        self.until(lambda: self.publisher.get_subscription_count() >= 1 and
                   self.nodes[1].count_publishers(self.destination) >= 2)
        # Discovery precedes the original process's executor setup. A service
        # response proves the local spin thread has started before sending SIGINT.
        from rcl_interfaces.srv import GetParameters
        client = self.nodes[1].create_client(GetParameters, self.node_path + '/get_parameters')
        self.until(client.service_is_ready)
        future = client.call_async(GetParameters.Request(names=['use_sim_time']))
        self.until(future.done)
        assert future.result() is not None
        self.nodes[1].destroy_client(client)

    def message(self, sequence, width=5, height=2, data=None):
        msg = self.Grid()
        msg.header.frame_id = 'non_default_map'
        msg.header.stamp.sec = sequence
        msg.header.stamp.nanosec = 123456789
        msg.info.map_load_time.sec = -2
        msg.info.map_load_time.nanosec = 333333333
        msg.info.resolution = .03125
        msg.info.width, msg.info.height = width, height
        msg.info.origin.position.x, msg.info.origin.position.y = -1.25, 3.5
        msg.info.origin.position.z = 7.5
        msg.info.origin.orientation.z, msg.info.origin.orientation.w = .6, .8
        msg.data = [-128, -1, 0, 1, 64, 65, 100, 127, 0, 37] if data is None else data
        return msg

    def send(self, msg):
        old = len(self.messages)
        started = time.perf_counter_ns()
        self.publisher.publish(msg)
        self.until(lambda: len(self.messages) > old)
        stamp, out = self.messages[-1]
        assert signature(out) == signature(msg)
        return (stamp - started) / 1e6

    def close(self, expected=0):
        code = None
        if self.p:
            if self.p.poll() is None:
                self.p.send_signal(signal.SIGINT)
            try:
                code = self.p.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                self.p.kill()
                self.p.wait()
                raise AssertionError('Owned relay required SIGKILL')
            finally:
                self.handle.close()
                self.details['returncode'] = self.p.returncode
                self.details['samples'] = len(self.messages)
                (self.path / 'result.json').write_text(json.dumps(self.details, indent=2) + '\n')
        for executor, node, context in zip(self.executors, self.nodes, self.contexts):
            executor.shutdown()
            node.destroy_node()
            context.try_shutdown()
        if expected is not None:
            assert code == expected, (code, (self.path / 'node.log').read_text())


@pytest.mark.parametrize('impl', ['python', 'cpp'])
@pytest.mark.parametrize('remap', [False, True])
def test_latched_exact_forwarding_late_subscriber_and_no_reverse(impl, remap):
    relay = Relay(impl, 'latched_remap_' + str(remap), remap=remap, start=False)
    try:
        first = relay.message(1)
        relay.publisher.publish(first)
        relay.start()
        relay.until(lambda: len(relay.messages) > 0)
        assert signature(relay.messages[-1][1]) == signature(first)
        relay.ready()
        for msg in [relay.message(2), relay.message(3, width=0, height=99, data=[]),
                    relay.message(4, width=999, height=0, data=[-1, 65]),
                    relay.message(5, width=1024, height=1024, data=[-1, 0, 64, 100] * (256*1024))]:
            relay.send(msg)
        latest = relay.message(6)
        latest.info.origin.position.x = math.nan
        latest.info.origin.position.y = math.inf
        relay.send(latest)
        late = []
        relay.extra_subs.append(relay.nodes[1].create_subscription(relay.Grid, relay.destination, late.append, qos()))
        relay.until(lambda: late)
        assert signature(late[-1]) == signature(latest)
        # Locally originating maps and commands must never cross back to the remote domain.
        for _ in range(5):
            relay.local_publisher.publish(relay.message(999))
            relay.control_publisher.publish(relay.Twist())
            relay.pump(.05)
        assert relay.reverse == [] and relay.controls == []
        assert len(relay.messages) == 11  # 6 forwarded + 5 local publications; no replay loop.
        from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
        endpoints = relay.nodes[1].get_publishers_info_by_topic(relay.destination)
        own = [e for e in endpoints if e.node_name == relay.prefix]
        assert own and own[0].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert own[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
    finally:
        relay.close()


@pytest.mark.parametrize('impl', ['python', 'cpp'])
def test_initial_timeout_only_and_duplicate_content_still_forwarded(impl):
    relay = Relay(impl, 'initial_only', timeout=1.5, start=False)
    try:
        relay.publisher.publish(relay.message(1))
        relay.start()
        relay.until(lambda: relay.messages)
        relay.pump(1.7)
        assert relay.p.poll() is None
        for i in range(2, 8):
            relay.send(relay.message(i))
        log = (relay.path / 'node.log').read_text()
        assert log.count('自由=') == 1, log
    finally:
        relay.close()


@pytest.mark.parametrize('impl', ['python', 'cpp'])
@pytest.mark.parametrize('volatile', [False, True])
def test_no_first_compatible_map_times_out(impl, volatile):
    relay = Relay(impl, 'timeout_' + str(volatile), timeout=.6, volatile=volatile)
    try:
        relay.publisher.publish(relay.message(1)) if volatile else None
        relay.until(lambda: relay.p.poll() is not None, timeout=5.)
        assert not relay.messages
        assert relay.p.returncode == 1
    finally:
        relay.close(expected=1)


@pytest.mark.parametrize('impl', ['python', 'cpp'])
@pytest.mark.parametrize('values', [
    {'remote_domain_id': 163}, {'remote_domain_id': -1},
    {'remote_domain_id': 162.0}, {'relay_timeout_sec': 1},
    {'remote_domain_id': True}, {'relay_timeout_sec': '1.0'},
    {'remote_map_topic': 3}, {'local_map_topic': 'invalid topic'},
])
def test_rejected_startup(impl, values):
    relay = Relay(impl, 'reject_' + next(iter(values)) + '_' + str(next(iter(values.values()))), params=values)
    try:
        relay.until(lambda: relay.p.poll() is not None, timeout=5.)
        assert relay.p.returncode != 0
    finally:
        relay.close(expected=None)


@pytest.mark.parametrize('impl', ['python', 'cpp'])
@pytest.mark.parametrize('timeout', [0., -1., -math.inf, math.nan, math.inf])
def test_timeout_numeric_boundaries(impl, timeout):
    relay = Relay(impl, 'numeric_' + str(timeout), timeout=timeout)
    try:
        if timeout <= 0:
            relay.until(lambda: relay.p.poll() is not None)
            assert relay.p.returncode == 1
        else:
            relay.ready()
            relay.pump(.3)
            relay.send(relay.message(1))
    finally:
        relay.close(expected=1 if timeout <= 0 else 0)


@pytest.mark.parametrize('impl', ['python', 'cpp'])
def test_signal_shutdown_while_waiting_for_first_map(impl):
    relay = Relay(impl, 'shutdown_no_map')
    try:
        relay.ready()
    finally:
        relay.close()


@pytest.mark.parametrize('impl', ['python', 'cpp'])
def test_sigint_during_wait_after_deadline_does_not_become_timeout(impl):
    relay = Relay(impl, 'sigint_after_deadline', timeout=.12)
    try:
        relay.until(lambda: '地图中继启动' in (relay.path / 'node.log').read_text())
        # The remote executor is waiting for up to .2s. Interrupt after the
        # .12s deadline but before it returns: Python skips the timeout branch.
        time.sleep(.16)
        assert relay.p.poll() is None, 'Fixture missed the active executor-wait interval'
    finally:
        relay.close()


@pytest.mark.parametrize('impl', ['python', 'cpp'])
def test_first_map_timeout_ignores_paused_ros_clock(impl):
    relay = Relay(impl, 'timeout_paused_clock', timeout=.5, params={'use_sim_time': True})
    try:
        relay.until(lambda: relay.p.poll() is not None)
        assert relay.p.returncode == 1 and relay.messages == []
    finally:
        relay.close(expected=1)


@pytest.mark.parametrize('impl', ['python', 'cpp'])
def test_sigterm_preserves_os_termination_status(impl):
    relay = Relay(impl, 'sigterm')
    try:
        relay.ready()
        relay.p.send_signal(signal.SIGTERM)
        relay.until(lambda: relay.p.poll() is not None)
    finally:
        relay.close(expected=-signal.SIGTERM)


@pytest.mark.parametrize('impl', ['python', 'cpp'])
def test_burst_converges_to_latest_without_reordering(impl):
    relay = Relay(impl, 'burst')
    try:
        relay.ready()
        relay.send(relay.message(1))
        start = len(relay.messages)
        # Depth 1 is intentionally lossy under overload. Require intact,
        # ordered outputs and eventual latest-map delivery, not invented 100% delivery.
        for sequence in range(100, 200):
            msg = relay.message(sequence, width=256, height=256,
                                data=[sequence % 100] * (256*256))
            relay.publisher.publish(msg)
        relay.until(lambda: relay.messages[-1][1].header.stamp.sec == 199)
        received = [m for _, m in relay.messages[start:]]
        ids = [m.header.stamp.sec for m in received]
        assert ids == sorted(set(ids))
        for msg in received:
            assert len(msg.data) == 256*256
            assert set(msg.data) == {msg.header.stamp.sec % 100}
    finally:
        relay.close()
