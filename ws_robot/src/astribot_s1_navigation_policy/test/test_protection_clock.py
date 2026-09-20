"""Exercise the production final-writer tick with deterministic clock/input ports."""
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from rclpy.time import Time
from astribot_navigation_msgs.msg import MotionConstraint
from astribot_s1_navigation_policy.contracts import BearingCone, Stamp, Vec3
from astribot_s1_navigation_policy.control_time import ControlTime
from astribot_s1_navigation_policy.profile import Profile
from astribot_s1_navigation_policy.protection import CommandRestriction
from astribot_s1_navigation_policy.protection_node import FinalProtection


class Output:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class ReadyProfile:
    def __init__(self, profile):
        self.profile = profile

    def __getattr__(self, name):
        return getattr(self.profile, name)

    def ready(self, stamp):
        return True


class ClockFixture:
    def __init__(self, simulated=True):
        self.ros, self.wall = 10., 100.
        profile = Profile.load(Path(__file__).parents[1] / 'config/simulation.json')
        self.profile = ReadyProfile(profile)
        self.control_time = ControlTime(simulated, profile.command_timeout_s)
        self.control_time.advance(self.ros, self.wall)
        self.command = (.1, 0., 0.)
        self.command_at = self.wall
        self.command_ros_at = self.ros
        self.measured = (0., 0., 0.)
        self.scan_at = self.odom_at = self.wall
        self.scan_stamp = self.odom_stamp = self.ros
        self.points = ()
        self.coverage = (BearingCone(Vec3(1., 0., 0.), math.pi),)
        self.pending_scans = []
        self.proposal = None
        self.proposal_at = -math.inf
        self.epoch, self.sequence = 1, 0
        self.clear_at = self.ros - 1. if simulated else self.wall - 1.
        self.restriction = CommandRestriction(profile)
        self.restriction.recovering = False
        self.output, self.constraint, self.diagnostics = Output(), Output(), Output()
        self.scan_received = self.scan_transformed = self.scan_invalid = 0
        self.scan_tf_waits = self.scan_expired = 0
        self.last_scan_error = ''
        self.issue_proposal()

    def get_clock(self):
        return SimpleNamespace(now=lambda: Time(nanoseconds=round(self.ros * 1e9)))

    def envelope_stamp(self):
        return Stamp(round(self.ros * 1e9), 'ros', self.epoch)

    def process_scans(self, seconds):
        pass  # The fixture supplies already-transformed observations.

    def issue_proposal(self):
        message = MotionConstraint()
        message.stamp = self.get_clock().now().to_msg()
        message.epoch, message.sequence = self.epoch, self.sequence + 1
        message.lease_s = .3
        message.max_linear_speed, message.max_angular_speed = .2, .3
        message.corridor_tracking_required = True
        message.reason = 'CORRIDOR_TRANSIT'
        with patch('astribot_s1_navigation_policy.protection_node.time.monotonic', return_value=self.wall):
            FinalProtection.propose(self, message)

    def tick(self, ros_step=0., wall_step=0.):
        self.ros += ros_step
        self.wall += wall_step
        with patch('astribot_s1_navigation_policy.protection_node.time.monotonic', return_value=self.wall):
            FinalProtection.tick(self)
        return self.output.messages[-1], self.constraint.messages[-1]


def test_slow_physics_keeps_fresh_corridor_motion():
    node = ClockFixture()
    output, constraint = node.tick(.044, .346424)
    assert output.linear.x == .1 and not constraint.hold


def test_pause_revokes_authorization_and_resume_does_not_reuse_commands():
    node = ClockFixture()
    assert node.tick(.01, .01)[0].linear.x == .1
    output, constraint = node.tick(0., .51)
    assert output.linear.x == 0. and constraint.hold and constraint.reason == 'SIM_CLOCK_STALLED'
    assert node.proposal is None
    assert node.tick(.01, .01)[0].linear.x == 0.
    node.issue_proposal()
    for _ in range(15):
        node.scan_stamp = node.odom_stamp = node.ros
        node.scan_at = node.odom_at = node.wall
        node.issue_proposal()
        output, _ = node.tick(.05, .05)
        assert output.linear.x == 0.  # Fresh policy alone must not replay the old command.
    node.command = (.1, 0., 0.)
    node.command_ros_at, node.command_at = node.ros, node.wall
    node.issue_proposal()
    assert node.tick(.01, .01)[0].linear.x > 0.


def test_source_expiry_and_clock_rewind_stop_motion():
    node = ClockFixture()
    output, constraint = node.tick(.301, .05)
    assert output.linear.x == 0. and constraint.hold
    node = ClockFixture()
    output, constraint = node.tick(-1., .01)
    assert output.linear.x == 0. and constraint.hold and node.proposal is None
    assert node.scan_stamp == -math.inf and node.odom_stamp == -math.inf
    assert node.tick(1.01, .05)[0].linear.x == 0.


def test_hardware_retains_wall_freshness():
    node = ClockFixture(simulated=False)
    output, constraint = node.tick(.044, .346424)
    assert output.linear.x == 0. and constraint.hold
