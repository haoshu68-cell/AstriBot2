"""Stage P2 adapter: shared observations, one leased constraint output."""
import json
import time
from dataclasses import asdict, replace
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
from astribot_navigation_msgs.msg import MotionConstraint
from .observer_node import PolicyObserver
from .behavior import YieldPolicy

class PolicyNode(PolicyObserver):
    observation_only = False
    default_plan_topic = '/path_tracking/active_path'
    def __init__(self):
        super().__init__()
        self.boot=time.monotonic_ns()
        self.selector=YieldPolicy(self.profile);self.sequence=0
        self.path_blocked=False
        self.create_subscription(Bool,'/navigation_policy/path_blocked',self.path_risk,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.constraint=self.create_publisher(MotionConstraint,'/navigation_policy/proposed_constraint',10)
        self.state=self.create_publisher(String,'/navigation_policy/state',10)

    def path_risk(self,msg):
        self.path_blocked=msg.data

    def tick(self):
        super().tick()
        now=self.stamp();seconds=now.ns*1e-9
        valid=bool(self.path) and self.last_inputs_valid and self.last_evaluation_epoch==now.epoch
        risk=self.last_risk
        if self.path_blocked and risk is not None:
            risk=replace(risk,blocked=True,conflict_time_s=0.)
        selection=self.selector.select(risk,valid,seconds)
        msg=MotionConstraint();msg.stamp=self.get_clock().now().to_msg()
        self.sequence+=1;msg.epoch=self.boot+now.epoch;msg.sequence=self.sequence;msg.lease_s=self.profile.constraint_lease_s
        msg.hold=selection.motion=='HOLD';msg.max_linear_speed=selection.speed
        msg.max_angular_speed=0. if msg.hold else self.profile.max_angular_speed_rad_s
        msg.planning=selection.planning;msg.reason=selection.reason
        self.constraint.publish(msg)
        self.state.publish(String(data=json.dumps(dict(asdict(selection),stamp_ns=now.ns))))

def main():
    rclpy.init();node=PolicyNode()
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()
