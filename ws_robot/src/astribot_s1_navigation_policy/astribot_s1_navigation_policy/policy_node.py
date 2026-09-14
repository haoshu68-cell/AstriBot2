"""Stage P2 adapter: shared observations, one leased constraint output."""
import json
import time
import threading
from dataclasses import asdict, replace
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
from astribot_navigation_msgs.msg import MotionConstraint, PathRisk, NavigationPolicyStatus, CorridorAlignment
from .observer_node import PolicyObserver
from .execution_context import to_wire
from .sensor_health import movement_directions
from .behavior import YieldPolicy, requires_stop
from .risk import evaluate_risk
from .path_evidence import PathEvidence, assess_path, path_identity

class PolicyNode(PolicyObserver):
    observation_only = False
    default_plan_topic = '/path_tracking/active_path'
    def __init__(self):
        super().__init__()
        self.boot=time.monotonic_ns()
        self.declare_parameter('navigation_policy_stage','p2')
        stage=self.get_parameter('navigation_policy_stage').value
        if stage not in ('p2','p3','p4','p5'):raise ValueError('unsupported navigation policy stage')
        self.selector=YieldPolicy(self.profile);self.sequence=0
        self.path_blocked=False
        self.active_path_message=None
        self.alignment_anchor=None
        self.path_evidence=None;self.active_path_key=None;self.report_order=None
        self.pending_path_report=None;self.path_report_lock=threading.Lock()
        self.create_subscription(Bool,'/navigation_policy/path_blocked',self.path_risk,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PathRisk,'/navigation_policy/path_risk',self.path_report,1)
        self.constraint=self.create_publisher(MotionConstraint,'/navigation_policy/proposed_constraint',10)
        self.state=self.create_publisher(String,'/navigation_policy/state',10)
        self.typed_state=self.create_publisher(NavigationPolicyStatus,'/navigation/policy_status',10)
        self.coordinator=None
        if stage in ('p3','p4','p5'):
            from .route_coordinator import RouteCoordinator
            self.coordinator=RouteCoordinator(self)
        self.corridor=None
        if stage in ('p4','p5'):
            from .corridor_adapter import CorridorAdapter
            self.corridor=CorridorAdapter(self)
            self.alignment_pub=self.create_publisher(CorridorAlignment,'/navigation_policy/corridor_alignment',1)

    def path_risk(self,msg):
        self.path_blocked=msg.data

    def process_plan(self):
        pending=self.pending_plan
        super().process_plan()
        if pending is not None and self.pending_plan is None and self.path:
            try:
                self.active_path_key=path_identity(pending);self.active_path_message=pending
            except ValueError:self.path=();self.active_path_key=None

    def path_report(self,msg):
        with self.path_report_lock:self.pending_path_report=(msg,time.monotonic())

    def process_path_report(self):
        with self.path_report_lock:
            pending=self.pending_path_report;self.pending_path_report=None
        if pending is None:return
        msg,received=pending
        now=self.stamp()
        stamp_ns=msg.stamp.sec*10**9+msg.stamp.nanosec
        if not 0<=now.ns-stamp_ns<=int(self.profile.path_risk_timeout_s*1e9):
            return
        try:key=path_identity(msg.checked_path)
        except ValueError:return
        if self.report_order is not None and self.report_order[0]==now.epoch and stamp_ns<self.report_order[1]:
            return
        self.report_order=(now.epoch,stamp_ns)
        pose_stamp=msg.evaluated_start.header.stamp
        effective_stamp_ns=min(stamp_ns,pose_stamp.sec*10**9+pose_stamp.nanosec) if msg.known else stamp_ns
        self.path_evidence=PathEvidence(key,effective_stamp_ns*1e-9,received,now.epoch,
                                        msg.known,msg.blocked,msg.distance_m)

    def publish_alignment(self, heading, centering_target=None, tracking=False):
        import math
        from rclpy.time import Time
        from geometry_msgs.msg import PoseStamped
        from .observer_node import yaw
        if self.active_path_message is None:return
        try:
            frame=self.active_path_message.header.frame_id
            transform=self.tf.lookup_transform(frame,self.profile.tracking_frame,Time())
            centering=centering_target is not None
            mode='tracking' if tracking else ('centering' if centering else 'alignment')
            if self.alignment_anchor is None or getattr(self,'alignment_mode','')!=mode:
                self.alignment_mode=mode
                self.alignment_anchor=PoseStamped();self.alignment_anchor.header.frame_id=frame
                x,y,_=self.point((self.last_robot.x,self.last_robot.y,0.),transform)
                self.alignment_anchor.pose.position.x=x;self.alignment_anchor.pose.position.y=y
                target=(self.last_robot.yaw if centering else heading)+yaw(transform.transform.rotation)
                self.alignment_anchor.pose.orientation.z=math.sin(target/2)
                self.alignment_anchor.pose.orientation.w=math.cos(target/2)
            msg=CorridorAlignment();msg.stamp=self.get_clock().now().to_msg()
            msg.lease_s=min(.3,self.profile.constraint_lease_s)
            msg.reference_path=self.active_path_message;msg.anchor=self.alignment_anchor
            msg.centering_required=centering
            msg.tracking_required=tracking
            if centering:
                msg.target=PoseStamped();msg.target.header.frame_id=frame
                x,y,_=self.point((*centering_target,0.),transform)
                msg.target.pose.position.x=x;msg.target.pose.position.y=y
                msg.target.pose.orientation=msg.anchor.pose.orientation
            self.alignment_pub.publish(msg)
        except Exception as error:self.get_logger().warning('corridor alignment unavailable: '+str(error))

    def tick(self):
        processing_start=time.monotonic()
        super().tick()
        self.process_path_report()
        if self.coordinator is not None:self.coordinator.process_route()
        now=self.stamp();seconds=now.ns*1e-9
        valid=bool(self.path) and self.last_inputs_valid and self.last_evaluation_epoch==now.epoch
        risk=self.last_risk
        path_risk=assess_path(self.path_evidence,self.active_path_key,seconds,time.monotonic(),
                              now.epoch,self.path_blocked,self.profile)
        if path_risk.blocked and risk is not None:
            risk=replace(risk,blocked=True,conflict_time_s=min(risk.conflict_time_s,path_risk.conflict_time_s))
        robot=self.last_robot
        coverage_ok=self.health_registry.allows_motion(now,robot.vx,robot.vy,robot.wz) if robot else False
        valid=valid and coverage_ok
        slow_original=False
        if (valid and path_risk.status=='CLEAR' and risk is not None and
            risk.blocked and not risk.immediate and not risk.uncertain):
            slow_risk=evaluate_risk(self.last_world,robot,self.path,self.profile,
                                    speed_limit=self.profile.narrow_speed_m_s)
            if not requires_stop(slow_risk,self.profile):
                risk=slow_risk;slow_original=True
        selection=self.selector.select(risk,valid,seconds)
        if slow_original and selection.motion!='HOLD':
            selection=replace(selection,motion='SLOW',speed=self.profile.narrow_speed_m_s,
                              reason='ORIGINAL_PATH_SLOW_SAFE')
        if not coverage_ok:selection=replace(selection,motion='HOLD',speed=0.,reason='REQUIRED_COVERAGE_UNAVAILABLE')
        passage=self.corridor.advance(selection,valid,time.monotonic()) if self.corridor else None
        if passage is not None and passage.state!='NORMAL':
            selection=passage.selection
            # A constrained passage owns waiting; no candidate may turn inside it.
            self.coordinator.cancel()
            if passage.failure:self.coordinator.failure=passage.failure
        elif self.coordinator is not None:
            selection=self.coordinator.advance(selection,risk,valid)
        if passage is not None and passage.tracking_heading is not None:
            self.publish_alignment(passage.tracking_heading,tracking=True)
        elif passage is not None and (passage.alignment_heading is not None or passage.centering_target is not None):
            self.publish_alignment(passage.alignment_heading,passage.centering_target)
        else:self.alignment_anchor=None
        msg=MotionConstraint();msg.stamp=self.get_clock().now().to_msg()
        self.sequence+=1;msg.epoch=self.boot+now.epoch;msg.sequence=self.sequence;msg.lease_s=self.profile.constraint_lease_s
        msg.hold=selection.motion=='HOLD';msg.max_linear_speed=selection.speed
        msg.max_angular_speed=0. if msg.hold else min(self.profile.max_angular_speed_rad_s, passage.angular_cap if passage else float('inf'))
        msg.alignment_required=bool(passage and passage.alignment_heading is not None)
        msg.centering_required=bool(passage and passage.centering_target is not None)
        msg.corridor_tracking_required=bool(passage and passage.tracking_heading is not None)
        msg.planning=selection.planning;msg.reason=selection.reason
        self.constraint.publish(msg)
        status=NavigationPolicyStatus();status.stamp=msg.stamp
        to_wire(self.last_world.version,status.version)
        status.motion=selection.motion;status.reason=selection.reason;status.inputs_valid=valid
        status.lease_s=msg.lease_s;self.typed_state.publish(status)
        self.state.publish(String(data=json.dumps(dict(asdict(selection),stamp_ns=now.ns,
            processing_wall_s=time.monotonic()-processing_start,
            corridor_state=passage.state if passage else 'DISABLED',
            corridor_id=passage.corridor_id if passage else '',
            corridor_permit=list(passage.permit) if passage else [],
            corridor_evidence=self.corridor.evidence if self.corridor else {},
            corridor_error=self.corridor.last_error if self.corridor else '',
            path_risk_status=path_risk.status,path_distance_m=path_risk.distance_m,
            original_path_admission='SLOW_EXECUTABLE' if slow_original else 'STANDARD',
            candidate_audit=self.coordinator.audit[-12:] if self.coordinator else [],
            candidate_safety_evidence=self.coordinator.safety_evidence[-12:] if self.coordinator else []))))

def main():
    rclpy.init();node=PolicyNode()
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()
