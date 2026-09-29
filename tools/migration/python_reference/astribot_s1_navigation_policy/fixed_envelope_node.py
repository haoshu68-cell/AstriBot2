"""V2 adapter. Legacy requests can revoke, but cannot grant V2 motion."""
import copy
import math
from astribot_navigation_msgs.msg import RobotGeometryState, ArmHoldStatus, EnvelopeApplyStatus, NavigationEnvelopeV2, RobotEnvelope
from astribot_navigation_msgs.srv import SetFixedEnvelope, SetRobotEnvelope, ReserveArmMotion
from astribot_s1_robot_geometry.node import ns
from geometry_msgs.msg import Polygon
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from .fixed_envelope import FixedEnvelope
from .robot_envelope import FIELDS

class FixedEnvelopeAdapter:
    def __init__(self,node,profile):
        self.node=node;self.core=FixedEnvelope(profile);self.odom=None
        self.pub=node.create_publisher(NavigationEnvelopeV2,'/navigation/envelope_v2',10)
        self.legacy=node.create_publisher(RobotEnvelope,'/navigation/robot_envelope',10)
        self.footprints=[node.create_publisher(Polygon,'/'+n+'/footprint',1) for n in ('global_costmap','local_costmap')]
        node.create_subscription(RobotGeometryState,'/navigation/geometry_state',self.geometry,10)
        node.create_subscription(ArmHoldStatus,'/navigation/arm_hold',self.hold,10)
        node.create_subscription(EnvelopeApplyStatus,'/navigation/envelope_applied',lambda m:self.core.acknowledge(m,self.now()),20)
        node.create_subscription(Odometry,'/odom',self.odom_callback,qos_profile_sensor_data)
        node.create_service(SetFixedEnvelope,'/navigation/set_fixed_envelope',self.propose)
        node.create_service(SetRobotEnvelope,'/navigation/set_robot_envelope',self.legacy_request)
        node.create_service(ReserveArmMotion,'/navigation/reserve_arm_motion',self.reserve)
        node.create_timer(.1,self.tick)
    def now(self):return self.node.get_clock().now().nanoseconds
    def geometry(self,msg):
        self.core.state(msg,self.now())
        # Relay new source evidence immediately. Waiting for the next 10 Hz
        # heartbeat can consume the remaining acquisition lease and create
        # brief stop gaps even though the next fresh state has already arrived.
        self.tick()
    def hold(self,msg):self.core.hold=msg
    def odom_callback(self,msg):self.odom=msg
    def propose(self,req,res):
        try:
            now=self.now();o=self.odom;v=o.twist.twist if o else None
            stopped=o is not None and 0<=now-ns(o.header.stamp)<=300_000_000 and math.hypot(v.linear.x,v.linear.y)<=.02 and abs(v.angular.z)<=.03
            e=self.core.propose(req,now,stopped);res.accepted=True;res.epoch=e.epoch;res.reason=e.reason
        except ValueError as error:res.reason=str(error)
        return res
    def legacy_request(self,req,res):
        if req.envelope.transport_ready:res.reason='FIXED_V2_REQUIRES_GEOMETRY_AND_HOLD'
        else:self.core.revoke('TASK_HOLD');res.accepted=True;res.epoch=self.core.epoch;res.reason='TASK_HOLD'
        self.tick();return res
    def reserve(self,req,res):res.accepted=False;res.reason='RESERVED_ARM_MOTION_NOT_ENABLED';return res
    def tick(self):
        e=self.core.tick(self.now())
        if e is not None:
            self.pub.publish(e)
            if not e.navigation_allowed:
                for pub in self.footprints:pub.publish(e.installed_footprint)
            old=copy.deepcopy(e.limits)
        else:
            old=RobotEnvelope();old.epoch=self.core.epoch;old.frame_id=self.core.baseline.base_frame;old.posture_id='fixed_v2_hold';old.lease_s=.3
            for name in FIELDS:setattr(old,name,getattr(self.core.baseline,name))
        old.transport_ready=False;old.reason='FIXED_V2_NO_LEGACY_MOTION';old.stamp=self.node.get_clock().now().to_msg()
        self.legacy.publish(old)
