"""Simulation truth adapter. Deliberately strips behavior, desired speed and goals."""
import copy
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from astribot_navigation_msgs.msg import SocialAgent, SocialAgentArray
from .contracts import stamp_ns


class HuNavTruthAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__('hunav_truth_adapter', **kwargs)
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('HuNav truth is simulation-only')
        from hunav_msgs.msg import Agents
        self.declare_parameter('input_topic', '/human_states')
        self.declare_parameter('source_id', 'hunav')
        self.declare_parameter('observation_mode', 'normal')
        if self.get_parameter('observation_mode').value not in ('normal', 'drop', 'repeat'):
            raise ValueError('Invalid simulation observation mode')
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.last_result = None
        self.epoch = time.time_ns(); self.sequence = 0
        self.last_stamp = None; self.last_clock = None
        self.output = self.create_publisher(SocialAgentArray, '/simulation/social_agents_truth', 10)
        self.create_subscription(Agents, self.get_parameter('input_topic').value,
                                 self.receive, qos_profile_sensor_data)
        self.create_timer(0.05, self.check_clock)

    def validate_parameters(self, parameters):
        valid = all(p.name != 'observation_mode' or p.value in ('normal', 'drop', 'repeat') for p in parameters)
        return SetParametersResult(successful=valid, reason='' if valid else 'normal, drop or repeat required')

    def check_clock(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.epoch += 1; self.last_stamp = None; self.sequence = 0
            self.last_result = None
        self.last_clock = now
        if self.get_parameter('observation_mode').value == 'repeat' and self.last_result is not None:
            self.output.publish(self.last_result)

    def receive(self, message):
        self.check_clock()
        if self.get_parameter('observation_mode').value != 'normal':
            return
        stamp = stamp_ns(message.header.stamp)
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp; self.sequence += 1
        result = SocialAgentArray()
        result.header = message.header; result.source_id = self.get_parameter('source_id').value
        result.source_epoch = self.epoch; result.sequence = self.sequence
        result.provenance = SocialAgentArray.SIMULATION_TRUTH; result.valid = True
        for person in message.agents:
            if person.type != person.PERSON:
                continue
            agent = SocialAgent()
            agent.track_id = str(person.id); agent.last_observed = message.header.stamp
            agent.pose.pose = copy.deepcopy(person.position)
            agent.pose.pose.orientation.x = 0.0; agent.pose.pose.orientation.y = 0.0
            agent.pose.pose.orientation.z = math.sin(person.yaw / 2)
            agent.pose.pose.orientation.w = math.cos(person.yaw / 2)
            agent.velocity.twist = person.velocity; agent.velocity_valid = True
            agent.radius = person.radius; agent.person_confidence = 1.0
            agent.heading_confidence = 1.0
            agent.group_id = str(person.group_id) if person.group_id >= 0 else ''
            agent.group_confidence = 1.0 if agent.group_id else 0.0
            result.agents.append(agent)
        self.last_result = copy.deepcopy(result)
        self.output.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = HuNavTruthAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
