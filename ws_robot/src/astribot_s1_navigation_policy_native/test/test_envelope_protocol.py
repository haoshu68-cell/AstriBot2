"""Direct Python/C++ message tests with controlled ROS time and no automatic acks.

Each case runs one owned process; no Nav2, Gazebo or hardware is started.
ENVELOPE_CPP_BINARY selects the candidate; Python is the unchanged oracle.
"""
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
import sys
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from astribot_navigation_msgs.msg import RobotEnvelope
from astribot_navigation_msgs.srv import SetRobotEnvelope

POLICY = Path(__file__).resolve().parents[2] / 'astribot_s1_navigation_policy'
FRAME = 'astribot_torso_base'
PYTHON_MAIN = '''from reference_bootstrap import enable
enable()

import rclpy
from rclpy.signals import SignalHandlerOptions
from astribot_s1_navigation_policy.envelope_node import EnvelopeNode
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = EnvelopeNode()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
'''


def stamp(ns):
    return Time(sec=ns // 10**9, nanosec=ns % 10**9)


class Protocol:
    def __init__(self, impl, binary, log_path):
        self.proc = self.log = self.node = self.context = self.executor = None
        try:
            self.start(impl, binary, log_path)
        except BaseException:
            self.close()
            raise

    def start(self, impl, binary, log_path):
        self.ns = 1_000_000_000
        self.events = []
        self.impl = impl
        self.env = dict(os.environ, ROS_DOMAIN_ID='109', ROS_LOCALHOST_ONLY='1')
        self.env['PYTHONPATH'] = str(REFERENCE_ROOT) + os.pathsep + str(POLICY) + os.pathsep + self.env.get('PYTHONPATH', '')
        command = [binary] if impl == 'cpp' else [sys.executable, '-c', PYTHON_MAIN]
        command += ['--ros-args', '-p', 'use_sim_time:=true', '-p',
                    f'profile:={POLICY / "config/simulation.json"}', '-r',
                    'robot_envelope_coordinator:/tf_static:=/migration/tf_static']
        self.log = log_path.open('w')
        self.proc = subprocess.Popen(command, env=self.env, stdout=self.log, stderr=subprocess.STDOUT)
        self.context = Context()
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init(context=self.context, domain_id=109)
        self.node = Node('envelope_protocol_probe', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.node.create_subscription(RobotEnvelope, '/navigation/robot_envelope', self.events.append, 100)
        self.clock = self.node.create_publisher(Clock, '/clock', 10)
        self.odom = self.node.create_publisher(Odometry, '/odom', 10)
        self.footprints = {name: self.node.create_publisher(PolygonStamped,
            f'/{name}_costmap/published_footprint', 10) for name in ('global', 'local')}
        self.tf = self.node.create_publisher(TFMessage, '/migration/tf_static',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = self.node.create_client(SetRobotEnvelope, '/navigation/set_robot_envelope')
        self.until(self.client.service_is_ready, 8)
        # Both query and node use the same isolated domain; verify the owned child.
        child_env = Path(f'/proc/{self.proc.pid}/environ').read_bytes().split(b'\0')
        assert b'ROS_DOMAIN_ID=109' in child_env
        assert b'ROS_LOCALHOST_ONLY=1' in child_env
        self.clock.publish(Clock(clock=stamp(self.ns)))
        self.drain(.1)
        self.advance()

    def until(self, predicate, timeout=3):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            assert self.proc.poll() is None, 'candidate process exited during protocol'
            self.executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        raise AssertionError(f'{self.impl}: protocol condition timed out')

    def drain(self, duration=.07):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            assert self.proc.poll() is None, 'candidate process exited during protocol'
            self.executor.spin_once(timeout_sec=.005)

    def advance(self, delta=200_000_000):
        self.ns += delta
        self.clock.publish(Clock(clock=stamp(self.ns)))
        self.until(lambda: bool(self.events) and
                   self.events[-1].stamp.sec * 10**9 + self.events[-1].stamp.nanosec == self.ns)
        return self.events[-1]

    def stopped(self, source=None, vx=0.0):
        msg = Odometry()
        msg.header.stamp = stamp(self.ns if source is None else source)
        msg.twist.twist.linear.x = vx
        self.odom.publish(msg)
        self.drain()

    def ack(self, name, half_x=.31, half_y=.31, source=None, frame=FRAME, points=None):
        msg = PolygonStamped()
        msg.header.stamp = stamp(self.ns if source is None else source)
        msg.header.frame_id = frame
        msg.polygon.points = points if points is not None else [
            Point32(x=float(x), y=float(y), z=0.0)
            for x,y in ((half_x, half_y), (-half_x, half_y), (-half_x, -half_y), (half_x, -half_y))]
        self.footprints[name].publish(msg)
        self.drain()

    def request(self, **updates):
        candidate = copy.deepcopy(self.events[-1])
        for name, value in updates.items():
            setattr(candidate, name, value)
        future = self.client.call_async(SetRobotEnvelope.Request(envelope=candidate))
        self.until(future.done)
        result = future.result()
        if result.accepted:
            self.until(lambda: self.events[-1].epoch == result.epoch)
        return result

    def ready(self):
        self.stopped()
        self.ack('global')
        self.ack('local')
        assert self.advance().transport_ready

    def close(self):
        if self.executor is not None:
            self.executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        if self.context is not None and self.context.ok():
            self.context.shutdown()
        if self.proc is None:
            if self.log is not None:
                self.log.close()
            return
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait(timeout=3)
        self.log.close()


@pytest.fixture(params=['python', 'cpp'])
def protocol(request, tmp_path):
    binary = os.environ.get('ENVELOPE_CPP_BINARY')
    if not binary:
        pytest.skip('ENVELOPE_CPP_BINARY required for message replay')
    fixture = Protocol(request.param, binary, tmp_path / 'node.log')
    try:
        yield fixture
    finally:
        fixture.close()


def test_paused_ros_clock_does_not_refresh_heartbeat(protocol):
    protocol.drain(.15)
    count = len(protocol.events)
    protocol.drain(.35)
    assert len(protocol.events) == count, 'wall timer refreshed envelope while ROS time was paused'


def test_nonfinite_shape_revokes_previously_acknowledged_costmap(protocol):
    protocol.ready()
    protocol.ack('global', points=[Point32(x=math.nan, y=math.nan, z=0.0) for _ in range(4)])
    result = protocol.advance()
    assert not result.transport_ready, 'NaN footprint retained a previous authorization'
    assert result.reason == 'POSTURE_OR_FOOTPRINT_PENDING'


def test_both_costmaps_and_new_proposal_epoch_required(protocol):
    protocol.stopped()
    protocol.ack('global')
    assert not protocol.advance().transport_ready
    protocol.ack('local')
    initial = protocol.advance()
    assert initial.transport_ready
    protocol.stopped()
    result = protocol.request(epoch=0, half_length_m=.35, transport_ready=True)
    assert result.accepted and result.epoch == initial.epoch + 1
    protocol.ack('global', half_x=.35, source=protocol.ns - 1)
    protocol.ack('local', half_x=.35, source=protocol.ns - 1)
    assert not protocol.advance().transport_ready, 'pre-proposal footprint released new envelope'
    protocol.ack('global', half_x=.35)
    assert not protocol.advance().transport_ready
    protocol.ack('local', half_x=.35)
    ready = protocol.advance()
    assert ready.transport_ready and ready.half_length_m == .35


def test_node_scoped_tf_remap_is_used_for_footprint_ack(protocol):
    protocol.stopped()
    tf = TransformStamped()
    tf.header.frame_id = FRAME
    tf.child_frame_id = 'migration_map'
    tf.transform.rotation.w = 1.0
    tf.transform.translation.x = 2.0
    protocol.tf.publish(TFMessage(transforms=[tf]))
    protocol.drain(.15)
    points = [Point32(x=x - 2.0, y=y, z=0.0) for x, y in
              ((.31,.31), (-.31,.31), (-.31,-.31), (.31,-.31))]
    protocol.ack('global', frame='migration_map', points=points)
    protocol.ack('local', frame='migration_map', points=points)
    assert protocol.advance().transport_ready


def test_malformed_source_stamp_does_not_exit_node_or_ack(protocol):
    protocol.stopped()
    msg = PolygonStamped()
    msg.header.frame_id = FRAME
    # Composite arithmetic looks fresh (1.2s), but sec < 0 is invalid for TF time.
    msg.header.stamp = Time(sec=-1, nanosec=2_200_000_000)
    msg.polygon.points = [Point32(x=x, y=y, z=0.0) for x,y in
                          ((.31,.31), (-.31,.31), (-.31,-.31), (.31,-.31))]
    protocol.footprints['global'].publish(msg)
    protocol.drain()
    protocol.ack('local')
    assert not protocol.advance().transport_ready


def test_service_freshness_and_thresholds(protocol):
    protocol.stopped(source=protocol.ns - 300_000_001)
    assert not protocol.request(transport_ready=True).accepted
    protocol.stopped(source=protocol.ns + 1)
    assert not protocol.request(transport_ready=True).accepted
    protocol.stopped(vx=.0200001)
    assert not protocol.request(transport_ready=True).accepted
    protocol.stopped(source=protocol.ns - 300_000_000, vx=.02)
    result = protocol.request(transport_ready=True)
    assert result.accepted and result.reason == 'WAITING_FOR_BOTH_COSTMAPS'


@pytest.mark.parametrize('updates', [
    {'lease_s': 0.0}, {'lease_s': .5000001}, {'lease_s': math.nan},
    {'posture_id': ''}, {'frame_id': 'other_frame'},
    {'half_length_m': .30}, {'half_width_m': .30}, {'height_m': .01},
    {'max_speed_m_s': 10.}, {'max_angular_speed_rad_s': 10.},
    {'max_acceleration_m_s2': 10.}, {'brake_deceleration_m_s2': 10.},
    {'payload_mass_kg': -1.}, {'payload_mass_kg': math.inf},
])
def test_invalid_service_preserves_epoch_and_ready(protocol, updates):
    protocol.ready()
    before = copy.deepcopy(protocol.events[-1])
    protocol.stopped()
    assert not protocol.request(**updates).accepted
    after = protocol.advance()
    before.stamp = after.stamp
    assert before == after


def test_all_fields_and_desired_hold_survive_proposal(protocol):
    protocol.stopped()
    fields = dict(lease_s=.5, frame_id=FRAME, posture_id='payload_transport',
                  half_length_m=.35, half_width_m=.34, height_m=2., payload_mass_kg=3.,
                  max_speed_m_s=.2, max_angular_speed_rad_s=.3,
                  max_acceleration_m_s2=.2, brake_deceleration_m_s2=.2,
                  transport_ready=False)
    response = protocol.request(**fields)
    assert response.accepted
    assert not protocol.events[-1].transport_ready
    protocol.ack('global', half_x=.35, half_y=.34)
    protocol.ack('local', half_x=.35, half_y=.34)
    message = protocol.advance()
    assert message.epoch == response.epoch
    for key, value in fields.items():
        assert getattr(message, key) == value, key
    assert message.reason == 'POSTURE_OR_FOOTPRINT_PENDING'


@pytest.mark.parametrize('age,ready', [(500_000_000, True), (500_000_001, False), (-1, False)])
def test_footprint_timestamp_boundaries(protocol, age, ready):
    protocol.stopped()
    protocol.ack('global', source=protocol.ns - age)
    protocol.ack('local', source=protocol.ns - age)
    assert protocol.advance().transport_ready is ready


def test_missing_tf_holds_until_corrected(protocol):
    protocol.stopped()
    protocol.ack('global', frame='missing')
    protocol.ack('local')
    assert not protocol.advance().transport_ready
    protocol.ack('global')
    assert protocol.advance().transport_ready


def test_ros_time_rewind_rejects_future_odom(protocol):
    protocol.stopped()
    protocol.ns -= 500_000_000
    protocol.clock.publish(Clock(clock=stamp(protocol.ns)))
    protocol.drain()
    assert not protocol.request(transport_ready=True).accepted
    protocol.advance()
    protocol.stopped()
    assert protocol.request(transport_ready=True).accepted


def test_nonfinite_odom_documents_reference_gap(protocol):
    # Legacy Python compares `speed > limit`, which lets NaN through. The C++
    # candidate intentionally keeps a finite stopping gate; never copy that hole.
    protocol.stopped(vx=math.nan)
    accepted = protocol.request(transport_ready=True).accepted
    assert accepted is (protocol.impl == 'python')


@pytest.mark.parametrize('half_x,ready', [(.31, True), (.311, True), (.3095, True),
                                       (.3089, False), (.3499, True), (.3501, False)])
def test_footprint_padding_and_shrink_boundaries(protocol, half_x, ready):
    protocol.stopped()
    protocol.ack('global', half_x=half_x)
    protocol.ack('local', half_x=half_x)
    assert protocol.advance().transport_ready is ready


@pytest.mark.parametrize('quaternion_scale', [1.0, 1.001])
def test_rotated_frame_uses_normalized_tf(protocol, quaternion_scale):
    protocol.stopped()
    angle = math.pi / 3
    tf = TransformStamped()
    tf.header.frame_id = FRAME
    tf.child_frame_id = 'migration_rotated'
    tf.transform.rotation.z = math.sin(angle / 2) * quaternion_scale
    tf.transform.rotation.w = math.cos(angle / 2) * quaternion_scale
    tf.transform.translation.x = 2.
    tf.transform.translation.y = -1.
    protocol.tf.publish(TFMessage(transforms=[tf]))
    protocol.drain(.15)
    points = []
    for x, y in ((.31,.31), (-.31,.31), (-.31,-.31), (.31,-.31)):
        dx, dy = x - 2., y + 1.
        points.append(Point32(x=math.cos(angle)*dx + math.sin(angle)*dy,
                              y=-math.sin(angle)*dx + math.cos(angle)*dy, z=0.))
    protocol.ack('global', frame='migration_rotated', points=points)
    protocol.ack('local', frame='migration_rotated', points=points)
    assert protocol.advance().transport_ready
