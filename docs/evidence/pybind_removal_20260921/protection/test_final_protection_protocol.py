"""Owned-domain wire replay of Python oracle and direct C++ final protection.

No simulator, robot or shared node is started/stopped. Only the child PID owned
by this fixture is terminated. ROS_DOMAIN_ID is verified from that child's env.
"""
import json
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
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from astribot_navigation_msgs.msg import RobotEnvelope, MotionConstraint, NavigationEnvelopeV2, EnvelopeApplyStatus

ROOT = Path(__file__).resolve().parents[2]
POLICY = ROOT / 'astribot_s1_navigation_policy'
sys.path.insert(0, str(POLICY))
from astribot_s1_navigation_policy.profile import Profile

MAIN = '''from astribot_s1_navigation_policy.protection_node import main
main()
'''


class Replay:
    def __init__(self, impl, mode, tmp):
        self.proc = self.context = self.node = self.executor = None
        self.log = (tmp / 'session.log').open('w')
        self.results, self.outputs, self.diagnostics, self.acks = [], [], [], []
        self.seconds = 10.
        self.sequence = 0
        self.mode = mode
        self.p = Profile.load(str(POLICY / 'config/simulation.json'))
        try:
            binary = os.environ.get('FINAL_PROTECTION_CPP_BINARY', '/tmp/astribot_protection_build/final_protection_cpp')
            assert impl != 'cpp' or Path(binary).is_file(), 'direct C++ final protection executable is missing'
            cmd = [binary] if impl == 'cpp' else [sys.executable, '-c', MAIN]
            cmd += ['--ros-args', '-p', 'use_sim_time:=true', '-p',
                    'profile:=' + str(POLICY / 'config/simulation.json'), '-p',
                    'navigation_geometry_mode:=' + mode]
            env = dict(os.environ, ROS_DOMAIN_ID='114', ROS_LOCALHOST_ONLY='1', ASTRIBOT_NAV_NATIVE_KERNELS='0')
            env['PYTHONPATH'] = str(POLICY) + os.pathsep + env.get('PYTHONPATH', '')
            self.proc = subprocess.Popen(cmd, env=env, stdout=self.log, stderr=subprocess.STDOUT)
            self.context = Context()
            rclpy.init(context=self.context, domain_id=114)
            self.node = Node('final_protection_replay', context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.clock = self.node.create_publisher(Clock, '/clock', 10)
            self.cmd = self.node.create_publisher(Twist, '/cmd_vel_policy_input', 10)
            self.odom = self.node.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
            self.scan = self.node.create_publisher(LaserScan, '/scan_from_cloud', qos_profile_sensor_data)
            self.proposal = self.node.create_publisher(MotionConstraint, '/navigation_policy/proposed_constraint', 10)
            self.envelope = self.node.create_publisher(RobotEnvelope if mode == 'legacy' else NavigationEnvelopeV2,
                '/navigation/robot_envelope' if mode == 'legacy' else '/navigation/envelope_v2', 10)
            self.node.create_subscription(MotionConstraint, '/navigation_policy/constraint', self.results.append, 100)
            self.node.create_subscription(Twist, '/cmd_vel', self.outputs.append, 100)
            self.node.create_subscription(String, '/navigation_policy/protection_state',
                lambda m: self.diagnostics.append(json.loads(m.data)), 100)
            self.node.create_subscription(EnvelopeApplyStatus, '/navigation/envelope_applied', self.acks.append, 100)
            # Read an actual constraint frame; graph discovery is not readiness.
            self.wait(lambda: bool(self.results), timeout=8)
            assert b'ROS_DOMAIN_ID=114' in Path(f'/proc/{self.proc.pid}/environ').read_bytes().split(b'\0')
        except BaseException:
            self.close()
            raise

    def wait(self, predicate, timeout=3):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            assert self.proc.poll() is None, 'protection child exited; inspect session.log'
            self.executor.spin_once(timeout_sec=.005)
            if predicate(): return
        raise AssertionError('protection protocol timeout')

    def spin(self, seconds=.025):
        end = time.monotonic() + seconds
        self.wait(lambda: time.monotonic() >= end, timeout=seconds + 1)

    def stamp(self, age=0.):
        from builtin_interfaces.msg import Time
        ns = round((self.seconds - age) * 1e9)
        return Time(sec=ns // 10**9, nanosec=ns % 10**9)

    def make_envelope(self):
        e = RobotEnvelope(stamp=self.stamp(), epoch=1, lease_s=.3, frame_id=self.p.base_frame,
                          posture_id='transport', transport_ready=True)
        for name in ('half_length_m', 'half_width_m', 'height_m', 'payload_mass_kg', 'max_speed_m_s',
                     'max_angular_speed_rad_s', 'max_acceleration_m_s2', 'brake_deceleration_m_s2'):
            setattr(e, name, getattr(self.p, name))
        if self.mode == 'legacy': return e
        from astribot_s1_robot_geometry.polygon import inflate, geometry_hash
        import numpy as np
        v = NavigationEnvelopeV2()
        v.header.stamp = self.stamp(); v.header.frame_id = self.p.base_frame
        v.valid_until = self.stamp(-.3); v.coordinator_session_id = 'protection-test'
        v.hold_id = 'hold'; v.epoch = 1; v.mode = v.FIXED_POSTURE; v.limits = e
        v.clearance_m = self.p.clearance_margin_m; v.navigation_allowed = True
        # Diamond deliberately differs from the enclosing rectangle.
        points = np.array([[-.31, 0.], [0., -.31], [.31, 0.], [0., .31]])
        v.reserved_footprint.points = [Point32(x=float(x), y=float(y)) for x, y in points]
        points = np.array([(x.x, x.y) for x in v.reserved_footprint.points])
        inflated = inflate(points, v.clearance_m + .000002)
        v.installed_footprint.points = [Point32(x=float(x), y=float(y)) for x, y in inflated]
        installed = [(x.x, x.y) for x in v.installed_footprint.points]
        v.installed_geometry_hash = geometry_hash(installed, self.p.base_frame, v.clearance_m)
        return v

    def step(self, command=(.1, 0., 0.), measured=(0., 0., 0.), obstacle=None,
             proposal=True, envelope=True, sensors=True, advance=True, mutate=None, mutate_proposal=None,
             mutate_scan=None):
        if advance: self.seconds += .025
        self.clock.publish(Clock(clock=self.stamp()))
        self.spin(.005)
        if sensors:
            od = Odometry(); od.header.stamp = self.stamp()
            od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.angular.z = measured
            self.odom.publish(od)
            scan = LaserScan(); scan.header.stamp = self.stamp(); scan.header.frame_id = self.p.base_frame
            scan.range_min = .01; scan.range_max = 10.
            scan.angle_min = -math.pi; scan.angle_increment = math.pi / 180.; scan.angle_max = math.pi
            scan.ranges = [math.inf] * 361
            if obstacle is not None:
                x, y = obstacle
                index = round((math.atan2(y, x) + math.pi) / scan.angle_increment)
                scan.ranges[index] = math.hypot(x, y)
            if mutate_scan: mutate_scan(scan)
            self.scan.publish(scan)
        if envelope:
            e = self.make_envelope()
            if mutate: mutate(e)
            self.envelope.publish(e)
        if proposal:
            self.sequence += 1
            p = MotionConstraint(stamp=self.stamp(), epoch=1, sequence=self.sequence, lease_s=.3,
                                 max_linear_speed=.2, max_angular_speed=.4, reason='CLEAR')
            if mutate_proposal: mutate_proposal(p)
            self.proposal.publish(p)
        if command is not None:
            c = Twist(); c.linear.x, c.linear.y, c.angular.z = command; self.cmd.publish(c)
        self.spin()

    def warm(self, **kw):
        for _ in range(40): self.step(**kw)
        assert not self.results[-1].hold, self.results[-1].reason

    def close(self):
        if self.executor: self.executor.shutdown()
        if self.node: self.node.destroy_node()
        if self.context and self.context.ok(): self.context.shutdown()
        if self.proc:
            if self.proc.poll() is None: self.proc.send_signal(signal.SIGINT)
            try: self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill(); self.proc.wait(timeout=3)
        self.log.close()


@pytest.fixture(params=['python', 'cpp'])
def replay(request, tmp_path):
    replay = Replay(request.param, 'legacy', tmp_path)
    try: yield replay
    finally: replay.close()


def test_release_then_policy_lease_expiry(replay):
    replay.warm()
    assert replay.outputs[-1].linear.x > .09
    for _ in range(16): replay.step(proposal=False)
    assert replay.results[-1].hold and replay.results[-1].reason == 'POLICY_UNAVAILABLE'
    assert replay.outputs[-1].linear.x == 0.


def test_clock_stall_and_rewind_revoke_motion(replay):
    replay.warm()
    replay.spin(.65)
    assert replay.results[-1].hold and replay.results[-1].reason == 'SIM_CLOCK_STALLED'
    replay.seconds -= 5.
    replay.step(command=None, proposal=False, envelope=False, sensors=False)
    assert replay.results[-1].hold and replay.outputs[-1].linear.x == 0.
    replay.warm()


def test_command_expiry_does_not_invent_motion(replay):
    replay.warm()
    for _ in range(16): replay.step(command=None)
    assert not replay.results[-1].hold
    assert replay.outputs[-1].linear.x == 0.


def test_measured_velocity_sweep_stops_zero_requested_velocity(replay):
    replay.warm(command=(0., 0., 0.))
    replay.step(command=(0., 0., 0.), measured=(.2, 0., 0.), obstacle=(.6, 0.))
    assert replay.results[-1].hold and replay.results[-1].reason == 'INDEPENDENT_SWEEP_RISK'


def test_expired_envelope_cannot_be_refreshed_by_receipt_time(replay):
    replay.warm()
    for _ in range(16):
        replay.step(mutate=lambda e: setattr(e, 'stamp', replay.stamp(age=1.)))
    assert replay.results[-1].hold and replay.results[-1].reason == 'ROBOT_ENVELOPE_UNAVAILABLE'
    replay.warm()


def test_replayed_proposal_cannot_extend_lease(replay):
    replay.warm()
    sequence = replay.sequence
    for _ in range(16):
        replay.step(mutate_proposal=lambda p: setattr(p, 'sequence', sequence))
    assert replay.results[-1].hold and replay.results[-1].reason == 'POLICY_UNAVAILABLE'


def test_alignment_and_centering_never_create_forbidden_components(replay):
    replay.warm(command=(.1, .02, .1))
    for _ in range(4):
        replay.step(command=(.1, .02, .1), mutate_proposal=lambda p: setattr(p, 'alignment_required', True))
    assert replay.outputs[-1].linear.x == replay.outputs[-1].linear.y == 0.
    assert replay.results[-1].alignment_required
    for _ in range(4):
        replay.step(command=(.1, .02, .1), mutate_proposal=lambda p: setattr(p, 'centering_required', True))
    assert replay.outputs[-1].angular.z == 0. and replay.results[-1].centering_required


def test_nonfinite_command_stops_then_requires_clear_confirmation(replay):
    replay.warm()
    replay.step(command=(math.nan, 0., 0.))
    assert replay.results[-1].hold and replay.results[-1].reason == 'INDEPENDENT_SWEEP_RISK'
    assert replay.outputs[-1].linear.x == 0.
    replay.step()
    assert replay.results[-1].hold and replay.results[-1].reason == 'PROTECTION_CLEAR_CONFIRMATION'
    replay.warm()


def test_invalid_scan_does_not_refresh_sensor_lease(replay):
    replay.warm()
    for _ in range(16):
        replay.step(mutate_scan=lambda s: setattr(s, 'ranges', [math.nan] * 361))
    assert replay.results[-1].hold and replay.results[-1].reason == 'INPUT_UNAVAILABLE'
    assert replay.diagnostics[-1]['scan_invalid'] > 0
    replay.warm()


def test_failed_tf_queue_is_bounded_and_does_not_refresh_source_ttl(replay):
    replay.warm()
    for _ in range(16):
        replay.step(mutate_scan=lambda s: setattr(s.header, 'frame_id', 'missing_owned_test_frame'))
    assert replay.results[-1].hold and replay.results[-1].reason == 'INPUT_UNAVAILABLE'
    assert 0 < replay.diagnostics[-1]['pending_scans'] <= 5
    assert replay.diagnostics[-1]['scan_tf_waits'] > 0
    replay.warm()


@pytest.mark.parametrize('impl', ['python', 'cpp'])
def test_fixed_polygon_preserves_corner_clearance_and_revokes_bad_hash(impl, tmp_path):
    replay = Replay(impl, 'fixed_v2', tmp_path)
    try:
        # This point lies inside a rectangular footprint but outside the diamond
        # plus clearance. A rectangular substitute would fail this test.
        replay.warm(command=(0., 0., 0.), obstacle=(.28, .28))
        assert replay.acks and replay.acks[-1].applied
        replay.step(command=(0., 0., 0.), obstacle=(.28, .28),
                    mutate=lambda e: setattr(e, 'installed_geometry_hash', 'bad'))
        assert replay.results[-1].hold and replay.results[-1].reason == 'ROBOT_ENVELOPE_UNAVAILABLE'
    finally: replay.close()
