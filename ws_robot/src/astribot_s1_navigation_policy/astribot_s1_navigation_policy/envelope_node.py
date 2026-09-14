"""Transport posture handshake: stop, install both costmap footprints, acknowledge, release."""
import math
import time
import copy
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Polygon,PolygonStamped,Point32
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener
from astribot_navigation_msgs.msg import RobotEnvelope
from astribot_navigation_msgs.srv import SetRobotEnvelope
from ament_index_python.packages import get_package_share_directory
from .profile import Profile
from .robot_envelope import FIELDS,validate_envelope
from .observation_adapters import VisionAdapter


class EnvelopeNode(Node):
    def __init__(self):
        super().__init__('robot_envelope_coordinator')
        self.declare_parameter('profile',get_package_share_directory('astribot_s1_navigation_policy')+'/config/simulation.json')
        self.profile=Profile.load(self.get_parameter('profile').value)
        self.profile.require_environment(self.get_parameter('use_sim_time').value)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.envelope=RobotEnvelope();self.envelope.epoch=time.monotonic_ns()
        self.envelope.frame_id=self.profile.base_frame;self.envelope.posture_id='simulation_transport' if self.profile.environment=='simulation' else 'validated_transport'
        self.envelope.lease_s=.3
        for name in FIELDS:setattr(self.envelope,name,getattr(self.profile,name))
        self.desired_ready=True;self.acks=set();self.changed_ns=0;self.speed=None;self.odom_ns=None
        self.publisher=self.create_publisher(RobotEnvelope,'/navigation/robot_envelope',10)
        self.footprints={name:self.create_publisher(Polygon,'/'+name+'/footprint',1) for name in ('global_costmap','local_costmap')}
        for name in self.footprints:
            self.create_subscription(PolygonStamped,'/'+name+'/published_footprint',lambda m,n=name:self.ack(n,m),10)
        self.create_subscription(Odometry,'/odom',self.odom,qos_profile_sensor_data)
        self.create_service(SetRobotEnvelope,'/navigation/set_robot_envelope',self.propose)
        self.create_timer(.1,self.tick)

    def odom(self,msg):
        v=msg.twist.twist
        self.speed=(math.hypot(v.linear.x,v.linear.y),abs(v.angular.z))
        self.odom_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec

    def polygon(self):
        x=self.envelope.half_length_m;y=self.envelope.half_width_m
        return Polygon(points=[Point32(x=float(a),y=float(b),z=0.) for a,b in ((x,y),(-x,y),(-x,-y),(x,-y))])

    def ack(self,name,msg):
        ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        now=self.get_clock().now().nanoseconds
        if ns<self.changed_ns or not 0<=now-ns<=500000000 or len(msg.polygon.points)!=4:return
        points=msg.polygon.points
        edges=[math.hypot(points[(i+1)%4].x-points[i].x,points[(i+1)%4].y-points[i].y) for i in range(4)]
        expected=sorted([2*self.envelope.half_length_m]*2+[2*self.envelope.half_width_m]*2)
        shape_ok=all(want-.002<=actual<=want+.08 for actual,want in zip(sorted(edges),expected))
        if not shape_ok:
            self.acks.discard(name);return
        # Published footprint and TF may have different pose sampling instants
        # during motion. Once acknowledged at rest, compare shape, not that pose drift.
        if name in self.acks:return
        if self.speed is None or self.speed[0]>.02 or self.speed[1]>.03:return
        try:
            tf=self.tf.lookup_transform(self.envelope.frame_id,msg.header.frame_id,Time.from_msg(msg.header.stamp))
            points=[VisionAdapter.point((p.x,p.y,p.z),tf) for p in msg.polygon.points]
            # Nav2 adds configured footprint_padding; allow that outward expansion only.
            target=self.polygon().points
            if (all(any(abs(x-p.x)<.04 and abs(y-p.y)<.04 for x,y,z in points) for p in target) and
                min(p[0] for p in points)<=-self.envelope.half_length_m+.001 and
                max(p[0] for p in points)>=self.envelope.half_length_m-.001 and
                min(p[1] for p in points)<=-self.envelope.half_width_m+.001 and
                max(p[1] for p in points)>=self.envelope.half_width_m-.001):self.acks.add(name)
            else:self.acks.discard(name)
        except Exception:pass

    def propose(self,req,res):
        try:
            validate_envelope(req.envelope,self.profile)
            now=self.get_clock().now().nanoseconds
            if self.speed is None or self.odom_ns is None or not 0<=now-self.odom_ns<=300000000 or self.speed[0]>.02 or self.speed[1]>.03:
                raise ValueError('ROBOT_MUST_BE_STOPPED_WITH_FRESH_ODOMETRY')
            next_envelope=copy.deepcopy(req.envelope)
            next_envelope.epoch=self.envelope.epoch+1
            self.desired_ready=next_envelope.transport_ready
            next_envelope.transport_ready=False
            self.envelope=next_envelope;self.acks.clear();self.changed_ns=now
            res.accepted=True;res.epoch=self.envelope.epoch;res.reason='WAITING_FOR_BOTH_COSTMAPS'
            self.tick()
        except ValueError as error:res.accepted=False;res.reason=str(error)
        return res

    def tick(self):
        self.envelope.stamp=self.get_clock().now().to_msg()
        ready=self.desired_ready and len(self.acks)==2
        self.envelope.transport_ready=ready
        self.envelope.reason='TRANSPORT_READY' if ready else 'POSTURE_OR_FOOTPRINT_PENDING'
        self.publisher.publish(self.envelope)
        if len(self.acks)<2:
            for publisher in self.footprints.values():publisher.publish(self.polygon())


def main():
    rclpy.init();node=EnvelopeNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
