"""Validate and transform observations; never publish motion or planning requests."""
import copy
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
from astribot_navigation_msgs.msg import SocialAgentArray, SocialObservationStatus
from .contracts import SampleOrder, stamp_ns, validate_sample


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def transform_sample(message, transform, target_frame):
    output = copy.deepcopy(message)
    r = rotation(transform.transform.rotation)
    basis = np.zeros((6, 6)); basis[:3, :3] = r; basis[3:, 3:] = r
    for agent in output.agents:
        agent.pose.pose = do_transform_pose(agent.pose.pose, transform)
        agent.pose.covariance = (basis @ np.asarray(agent.pose.covariance).reshape(6, 6) @ basis.T).ravel().tolist()
        if agent.velocity_valid:
            for vector in (agent.velocity.twist.linear, agent.velocity.twist.angular):
                vector.x, vector.y, vector.z = (r @ np.array([vector.x, vector.y, vector.z])).tolist()
            agent.velocity.covariance = (basis @ np.asarray(agent.velocity.covariance).reshape(6, 6) @ basis.T).ravel().tolist()
    output.header.frame_id = target_frame
    return output


class SocialObserver(Node):
    def __init__(self, **kwargs):
        super().__init__('social_observer', **kwargs)
        self.declare_parameter('input_topic', '/perception/social_agents')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('sample_timeout_s', 0.3)
        self.declare_parameter('allow_simulation_truth', False)
        self.timeout = float(self.get_parameter('sample_timeout_s').value)
        self.allow_truth = bool(self.get_parameter('allow_simulation_truth').value)
        self.simulation = bool(self.get_parameter('use_sim_time').value)
        self.pending = None
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('sample_timeout_s must be finite and positive')
        if self.allow_truth and not self.get_parameter('use_sim_time').value:
            raise ValueError('Simulation truth requires use_sim_time')
        self.frame = self.get_parameter('target_frame').value
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.order = SampleOrder()
        self.last = None; self.previous_time = None; self.epoch = 0
        self.accepted = 0; self.rejected = 0; self.reason = 'NO_OBSERVATION'
        self.output = self.create_publisher(SocialAgentArray, '/social_navigation/observed_agents', 10)
        self.status = self.create_publisher(SocialObservationStatus, '/social_navigation/observation_status', 10)
        self.create_subscription(SocialAgentArray, self.get_parameter('input_topic').value,
                                 self.receive, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)

    def now_checked(self):
        now = self.get_clock().now().nanoseconds
        if self.previous_time is not None and now < self.previous_time:
            self.epoch += 1; self.last = None; self.order = SampleOrder()
            self.pending = None
            self.reason = 'CLOCK_RESET'
            self.publish_invalid()
        self.previous_time = now
        return now

    def publish_invalid(self):
        msg = copy.deepcopy(self.last) if self.last is not None else SocialAgentArray()
        msg.valid = False; msg.reason = self.reason; msg.agents = []
        msg.header.frame_id = self.frame
        self.output.publish(msg)

    def receive(self, message):
        now = self.now_checked()
        self.drain_pending(now)
        # Gazebo publishes /clock and sensor data on different DDS streams.
        # Wait for a slightly newer simulation stamp; never relabel it as current.
        ahead = stamp_ns(message.header.stamp) - now
        if self.simulation and 0 < ahead <= 50_000_000:
            if self.pending is None or stamp_ns(message.header.stamp) > stamp_ns(self.pending.header.stamp):
                self.pending = copy.deepcopy(message)
            return
        self.process(message, now)

    def drain_pending(self, now):
        if self.pending is not None and stamp_ns(self.pending.header.stamp) <= now:
            message = self.pending; self.pending = None
            self.process(message, now)

    def process(self, message, now):
        error = validate_sample(message, now, self.timeout, self.allow_truth)
        if not error:
            try:
                converted = message if message.header.frame_id == self.frame else transform_sample(
                    message, self.tf.lookup_transform(self.frame, message.header.frame_id,
                                                     Time.from_msg(message.header.stamp)), self.frame)
                error = self.order.accept(message)
            except TransformException:
                error = 'CAPTURE_TIME_TF_UNAVAILABLE'
        if error:
            self.rejected += 1
            # Repeated frames never refresh the lease; malformed/new invalid data invalidates it.
            if error != 'DUPLICATE_OR_OUT_OF_ORDER':
                self.reason = error; self.last = None; self.publish_invalid()
            return
        self.last = copy.deepcopy(converted); self.reason = ''
        self.accepted += 1
        self.output.publish(converted)

    def tick(self):
        now = self.now_checked()
        self.drain_pending(now)
        age = -1.0 if self.last is None else (now - stamp_ns(self.last.header.stamp)) * 1e-9
        if self.last is not None:
            error = validate_sample(self.last, now, self.timeout, self.allow_truth)
            if error and not self.reason:
                self.reason = error; self.publish_invalid()
        status = SocialObservationStatus()
        status.header.stamp = self.get_clock().now().to_msg(); status.header.frame_id = self.frame
        status.observer_epoch = self.epoch; status.accepted_frames = self.accepted
        status.rejected_frames = self.rejected; status.input_valid = not bool(self.reason)
        status.sample_age_s = age; status.reason = self.reason
        if self.last is not None:
            status.simulation_truth = self.last.provenance == SocialAgentArray.SIMULATION_TRUTH
            status.observed_count = sum(a.tracking_state == 0 for a in self.last.agents)
            status.occluded_count = sum(a.tracking_state == 1 for a in self.last.agents)
            status.lost_count = sum(a.tracking_state == 2 for a in self.last.agents)
        self.status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SocialObserver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
