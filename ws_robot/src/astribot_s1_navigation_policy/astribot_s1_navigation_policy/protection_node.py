"""Single final command writer with independent scan and wall-clock watchdogs."""
import math
import time
import json
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import Buffer,TransformListener
from ament_index_python.packages import get_package_share_directory
from astribot_navigation_msgs.msg import MotionConstraint, RobotEnvelope
from .profile import Profile
from .robot_envelope import EnvelopeProfile
from .observer_node import rotate, yaw
from .sensor_health import scan_coverage, movement_directions, coverage_allows_motion
from .contracts import Vec3, Stamp
from .protection import CommandRestriction,swept_point_collision,scan_usable

class FinalProtection(Node):
    def __init__(self):
        super().__init__('navigation_final_protection')
        self.declare_parameter('profile',get_package_share_directory('astribot_s1_navigation_policy')+'/config/simulation.json')
        self.declare_parameter('scan_topic','/scan_from_cloud')
        self.profile=EnvelopeProfile(Profile.load(self.get_parameter('profile').value))
        self.profile.require_environment(self.get_parameter('use_sim_time').value)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.command=(0.,0.,0.);self.command_at=-math.inf
        self.measured=(0.,0.,0.);self.odom_at=-math.inf;self.odom_stamp=-math.inf
        self.coverage=()
        self.points=();self.scan_at=-math.inf;self.scan_stamp=-math.inf
        self.pending_scans=[];self.scan_received=0;self.scan_transformed=0
        self.scan_invalid=0;self.scan_tf_waits=0;self.scan_expired=0;self.last_scan_error=""
        self.proposal=None;self.proposal_at=-math.inf;self.sequence=0;self.epoch=time.monotonic_ns()
        self.last_ros=None;self.last_wall=time.monotonic();self.clear_at=None
        self.restriction=CommandRestriction(self.profile)
        self.output=self.create_publisher(Twist,'/cmd_vel',10)
        self.constraint=self.create_publisher(MotionConstraint,'/navigation_policy/constraint',10)
        self.diagnostics=self.create_publisher(String,'/navigation_policy/protection_state',10)
        self.create_subscription(RobotEnvelope,'/navigation/robot_envelope',self.envelope,10)
        self.create_subscription(Twist,'/cmd_vel_policy_input',self.velocity,10)
        self.create_subscription(Odometry,'/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(LaserScan,self.get_parameter('scan_topic').value,self.scan,qos_profile_sensor_data)
        self.create_subscription(MotionConstraint,'/navigation_policy/proposed_constraint',self.propose,10)
        self.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def envelope_stamp(self):
        return Stamp(self.get_clock().now().nanoseconds,'ros',self.epoch)

    def envelope(self,msg):
        try:self.profile.accept(msg,self.envelope_stamp())
        except ValueError as error:self.get_logger().warning(str(error))

    def velocity(self,msg):
        self.command=(msg.linear.x,msg.linear.y,msg.angular.z);self.command_at=time.monotonic()
        # Validate each new command immediately; the wall timer still enforces
        # expiry when commands stop. Avoid adding one timer period to tracking.
        self.tick()

    def odom(self,msg):
        self.measured=(msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z)
        self.odom_at=time.monotonic();self.odom_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def propose(self,msg):
        if self.proposal and msg.epoch==self.proposal.epoch and msg.sequence<=self.proposal.sequence:return
        if not all(math.isfinite(v) for v in (msg.lease_s,msg.max_linear_speed,msg.max_angular_speed)):return
        if not 0<msg.lease_s<=.5 or min(msg.max_linear_speed,msg.max_angular_speed)<0:return
        self.proposal=msg;self.proposal_at=time.monotonic()

    def scan(self,msg):
        self.scan_received+=1
        if not scan_usable(msg.ranges,msg.range_min,msg.range_max,msg.angle_min,msg.angle_increment,
                           self.profile.scan_min_valid_fraction):
            self.scan_invalid+=1
            return
        self.pending_scans.append(msg)
        self.pending_scans=self.pending_scans[-5:]
        self.process_scans(self.get_clock().now().nanoseconds*1e-9)

    def process_scans(self,seconds):
        pending=[]
        for msg in self.pending_scans:
            capture=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if capture<=self.scan_stamp:continue
            if seconds-capture>self.profile.sensor_timeout_s:
                self.scan_expired+=1
                continue
            if capture>seconds:
                pending.append(msg)
                continue
            try:
                tf=self.tf.lookup_transform(self.profile.base_frame,msg.header.frame_id,Time.from_msg(msg.header.stamp))
                points=[]
                for i,r in enumerate(msg.ranges):
                    if math.isfinite(r) and msg.range_min<=r<msg.range_max:
                        a=msg.angle_min+i*msg.angle_increment
                        v=rotate(Vec3(r*math.cos(a),r*math.sin(a),0.),tf.transform.rotation)
                        points.append((v.x+tf.transform.translation.x,v.y+tf.transform.translation.y))
                self.coverage=scan_coverage(msg.ranges,msg.range_min,msg.range_max,
                    msg.angle_min,msg.angle_increment,yaw(tf.transform.rotation))
                self.points=tuple(points);self.scan_at=time.monotonic();self.scan_stamp=capture
                self.scan_transformed+=1;self.last_scan_error=""
            except Exception as exc:
                self.scan_tf_waits+=1;self.last_scan_error=str(exc)
                pending.append(msg)
        self.pending_scans=[m for m in pending if m.header.stamp.sec+m.header.stamp.nanosec*1e-9>self.scan_stamp]

    def tick(self):
        wall=time.monotonic();ros=self.get_clock().now();seconds=ros.nanoseconds*1e-9
        dt=wall-self.last_wall;self.last_wall=wall
        if self.last_ros is not None and seconds<self.last_ros:
            self.epoch+=1;self.proposal=None;self.scan_at=-math.inf;self.odom_at=-math.inf
            self.scan_stamp=-math.inf;self.pending_scans=[]
        self.last_ros=seconds
        self.process_scans(seconds)
        p=self.profile;m=self.proposal
        fresh=(wall-self.scan_at<=p.sensor_timeout_s and 0<=seconds-self.scan_stamp<=p.sensor_timeout_s and
               wall-self.odom_at<=p.sensor_timeout_s and 0<=seconds-self.odom_stamp<=p.sensor_timeout_s)
        lease=bool(m and wall-self.proposal_at<=m.lease_s and
                   0<=seconds-(m.stamp.sec+m.stamp.nanosec*1e-9)<=m.lease_s)
        reason='INPUT_UNAVAILABLE' if not fresh else ('POLICY_UNAVAILABLE' if not lease else m.reason)
        independent_stop=not fresh or not lease
        if not self.profile.ready(self.envelope_stamp()):
            independent_stop=True;reason='ROBOT_ENVELOPE_UNAVAILABLE'
        if fresh and (not coverage_allows_motion(self.coverage,*self.command) or
                      not coverage_allows_motion(self.coverage,*self.measured)):
            independent_stop=True;reason='INDEPENDENT_COVERAGE_UNAVAILABLE'
        if fresh and (swept_point_collision(self.points,self.command,p) or
                      swept_point_collision(self.points,self.measured,p)):
            independent_stop=True;reason='INDEPENDENT_SWEEP_RISK'
        # Confirm local safety while the policy is still yielding. Policy HOLD
        # remains authoritative, but does not restart an already-clear guard.
        if independent_stop:self.clear_at=None
        else:
            if self.clear_at is None:self.clear_at=wall
        stop=independent_stop or m.hold
        if not stop and wall-self.clear_at<p.clear_hold_s:
            stop=True;reason='PROTECTION_CLEAR_CONFIRMATION'
        command=self.command if wall-self.command_at<=p.input_command_timeout_s else (0.,0.,0.)
        if lease and m.alignment_required:command=(0.,0.,command[2])
        if lease and m.centering_required:command=(command[0],command[1],0.)
        output=self.restriction.apply(command,min(p.max_speed_m_s,m.max_linear_speed) if lease else 0.,
                                      m.max_angular_speed if lease else 0.,stop,dt)
        msg=Twist();msg.linear.x,msg.linear.y,msg.angular.z=output;self.output.publish(msg)
        self.sequence+=1;c=MotionConstraint();c.stamp=ros.to_msg();c.epoch=self.epoch
        c.sequence=self.sequence;c.lease_s=p.constraint_lease_s;c.hold=stop;c.reason=reason
        c.max_linear_speed=0. if stop else min(p.max_speed_m_s,m.max_linear_speed)
        c.max_angular_speed=0. if stop else m.max_angular_speed;c.planning=m.planning if lease else 0
        c.alignment_required=bool(lease and m.alignment_required)
        c.centering_required=bool(lease and m.centering_required)
        c.corridor_tracking_required=bool(lease and m.corridor_tracking_required)
        self.constraint.publish(c)
        if self.sequence%5==0:
            def age(v):return v if math.isfinite(v) else None
            self.diagnostics.publish(String(data=json.dumps({
                'reason':reason,'hold':stop,'scan_age_s':age(seconds-self.scan_stamp),
                'scan_wall_age_s':age(wall-self.scan_at),'odom_age_s':age(seconds-self.odom_stamp),
                'odom_wall_age_s':age(wall-self.odom_at),'policy_wall_age_s':age(wall-self.proposal_at),
                'loop_wall_dt_s':dt,'points':len(self.points),
                'scan_received':self.scan_received,'scan_transformed':self.scan_transformed,
                'scan_invalid':self.scan_invalid,'scan_tf_waits':self.scan_tf_waits,
                'scan_expired':self.scan_expired,'pending_scans':len(self.pending_scans),
                'last_scan_error':self.last_scan_error,'limiter_recovering':self.restriction.recovering,
                'input_speed_m_s':age(math.hypot(*self.command[:2])),
                'output_speed_m_s':math.hypot(*output[:2])},allow_nan=False)))

def main():
    rclpy.init();node=FinalProtection()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.output.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
