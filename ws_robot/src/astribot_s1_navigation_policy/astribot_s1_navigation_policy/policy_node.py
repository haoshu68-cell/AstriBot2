"""Stage P2 adapter: shared observations, one leased constraint output."""
import json
import math
import time
import threading
from dataclasses import asdict, replace
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
from astribot_navigation_msgs.msg import MotionConstraint, PathRisk, NavigationPolicyStatus, CorridorAlignment
from astribot_navigation_msgs.srv import ResolveRoute, GetRecoveryObstacles
from geometry_msgs.msg import Polygon, Point32
from .observer_node import PolicyObserver
from .execution_context import to_wire
from .sensor_health import movement_directions
from .behavior import Selection, YieldPolicy, requires_stop
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
        self.last_world=None
        self.recovery_obstacles_service=self.create_service(GetRecoveryObstacles,
            '/navigation_policy/recovery_obstacles',self.recovery_obstacles,
            callback_group=self.processing_group)
        self.coordinator=None
        if stage in ('p3','p4','p5'):
            from .route_coordinator import RouteCoordinator
            self.coordinator=RouteCoordinator(self)
        self.corridor=None
        if stage in ('p4','p5'):
            from .corridor_adapter import CorridorAdapter
            self.corridor=CorridorAdapter(self)
            self.alignment_pub=self.create_publisher(CorridorAlignment,'/navigation_policy/corridor_alignment',1)
        self.declare_parameter('social_navigation_stage','off')
        social_stage=self.get_parameter('social_navigation_stage').value
        if social_stage not in ('off','h2'):raise ValueError('unsupported social navigation stage')
        self.social=None
        if social_stage=='h2':
            if stage!='p2':raise ValueError('H2 uses P2 waiting without route replacement')
            from .route_coordinator import RouteCoordinator
            from .social_adapter import SocialAdapter
            self.coordinator=RouteCoordinator(self,candidate_enabled=False)
            self.social=SocialAdapter(self)

    def recovery_obstacles(self,request,response):
        # This callback shares the observer processing group. The C++ recovery
        # planner receives the same immutable world that produced policy risk.
        from .world_geometry import prediction_rows
        import numpy as np
        world=self.last_world
        if (world is None or not self.last_inputs_valid or
            self.last_evaluation_epoch!=self.epoch):
            response.reason='RECOVERY_POLICY_INPUT_UNAVAILABLE';return response
        if world.unassociated:
            response.reason='RECOVERY_NONMETRIC_OBSTACLES';return response
        rows=prediction_rows(world,include_current=True,swept=True)
        lower=np.full((len(world.tracks),2),np.inf)
        upper=np.full((len(world.tracks),2),-np.inf)
        np.minimum.at(lower,rows.owners,rows.lower)
        np.maximum.at(upper,rows.owners,rows.upper)
        response.header.frame_id=world.frame_id
        response.header.stamp=rclpy.time.Time(nanoseconds=world.stamp.ns).to_msg()
        for lo,hi in zip(lower,upper):
            # Polygon uses float32: round outwards when serializing bounds.
            lo=np.nextafter(lo.astype(np.float32),np.float32(-np.inf))
            hi=np.nextafter(hi.astype(np.float32),np.float32(np.inf))
            response.obstacles.append(Polygon(points=[Point32(x=float(x),y=float(y))
                for x,y in ((lo[0],lo[1]),(hi[0],lo[1]),(hi[0],hi[1]),(lo[0],hi[1]))]))
        response.valid=True;response.reason='POLICY_WORLD_SNAPSHOT'
        return response

    def path_risk(self,msg):
        if self.coordinator is not None and self.coordinator.workstation_alignment:return
        self.path_blocked=msg.data

    def retire_path_risk(self):
        self.path_blocked=False;self.path_evidence=None;self.report_order=None
        with self.path_report_lock:self.pending_path_report=None

    def process_plan(self):
        pending=self.pending_plan
        super().process_plan()
        if pending is not None and self.pending_plan is None and self.path:
            try:
                self.active_path_key=path_identity(pending);self.active_path_message=pending
            except ValueError:self.path=();self.active_path_key=None

    def path_report(self,msg):
        with self.path_report_lock:
            if self.pending_path_report is not None:
                old=self.pending_path_report[0]
                if (msg.stamp.sec,msg.stamp.nanosec)<(old.stamp.sec,old.stamp.nanosec):return
            self.pending_path_report=(msg,time.monotonic())

    def process_path_report(self):
        with self.path_report_lock:
            pending=self.pending_path_report;self.pending_path_report=None
        if pending is None:return
        if self.coordinator is not None and self.coordinator.workstation_alignment:return
        msg,received=pending
        now=self.stamp()
        stamp_ns=msg.stamp.sec*10**9+msg.stamp.nanosec
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
        processing_cpu_start=time.thread_time()
        super().tick()
        observer_done=time.monotonic()
        self.process_path_report()
        if self.coordinator is not None:self.coordinator.process_route()
        route_done=time.monotonic()
        now=self.stamp();seconds=now.ns*1e-9
        valid=bool(self.path) and self.last_inputs_valid and self.last_evaluation_epoch==now.epoch
        risk=self.last_risk
        path_risk=assess_path(self.path_evidence,self.active_path_key,seconds,time.monotonic(),
                              now.epoch,self.path_blocked,self.profile)
        if path_risk.blocked and risk is not None:
            risk=replace(risk,blocked=True,conflict_time_s=min(risk.conflict_time_s,path_risk.conflict_time_s))
        robot=self.last_robot
        # Observer health is published before risk evaluation. Preserve the
        # decision-time snapshot too, so an expired lease is visible in replay.
        coverage_health=self.health_registry.health(now)
        coverage_ok=self.health_registry.allows_motion(now,robot.vx,robot.vy,robot.wz) if robot else False
        valid=valid and coverage_ok
        workstation_alignment=bool(self.coordinator and self.coordinator.workstation_alignment)
        if workstation_alignment:
            # The committed controller owns layered collision prediction. Keep
            # the observation capability gate; retire only the old path risk.
            valid=self.last_inputs_valid and self.last_evaluation_epoch==now.epoch and coverage_ok
            selection=Selection('CONTINUE',self.profile.max_speed_m_s,'WORKSTATION_ALIGNMENT') if valid else Selection('HOLD',0.,'WORKSTATION_INPUT_UNAVAILABLE')
            slow_original=False;passage=None;corridor_start=corridor_done=time.monotonic()
        else:
            slow_original=False
            if (valid and path_risk.status=='CLEAR' and risk is not None and
                risk.blocked and not risk.immediate and not risk.uncertain):
                slow_risk=evaluate_risk(self.last_world,robot,self.path,self.profile,
                                        speed_limit=self.profile.narrow_speed_m_s)
                if not requires_stop(slow_risk,self.profile):
                    risk=slow_risk;slow_original=True
            selection=self.selector.select(risk,valid,time.monotonic())
            if slow_original and selection.motion!='HOLD':
                selection=replace(selection,motion='SLOW',speed=self.profile.narrow_speed_m_s,
                                  reason='ORIGINAL_PATH_SLOW_SAFE')
            if not coverage_ok:selection=replace(selection,motion='HOLD',speed=0.,reason='REQUIRED_COVERAGE_UNAVAILABLE')
            if self.social is not None:
                context=self.coordinator.social_context()
                social=self.social.apply(selection,valid,now,context)
                selection=social.selection
                self.coordinator.record_social_result(context,social.failure)
            corridor_start=time.monotonic()
            passage=self.corridor.advance(selection,valid,time.monotonic()) if self.corridor else None
            corridor_done=time.monotonic()
            if passage is not None and passage.state!='NORMAL':
                selection=passage.selection
                # A constrained passage owns waiting; no candidate may turn inside it.
                self.coordinator.cancel()
                if passage.failure:
                    self.coordinator.failure=passage.failure
                    self.coordinator.failure_code=ResolveRoute.Response.NONE
            elif self.coordinator is not None:
                selection=self.coordinator.advance(selection,risk,valid)
        if passage is not None and passage.tracking_heading is not None:
            self.publish_alignment(passage.tracking_heading,tracking=True)
        elif passage is not None and (passage.alignment_heading is not None or passage.centering_target is not None):
            self.publish_alignment(passage.alignment_heading,passage.centering_target)
        else:self.alignment_anchor=None
        # This is the policy heartbeat. The upstream navigation-constraint node
        # owns current sensor freshness and collision admission; do not recycle
        # a scan deadline here. It never writes chassis velocity commands.
        publication=self.stamp()
        msg=MotionConstraint();msg.stamp=rclpy.time.Time(nanoseconds=publication.ns).to_msg()
        self.sequence+=1;msg.epoch=self.boot+publication.epoch;msg.sequence=self.sequence;msg.lease_s=self.profile.constraint_lease_s
        msg.hold=selection.motion=='HOLD';msg.max_linear_speed=selection.speed
        msg.max_angular_speed=0. if msg.hold else min(self.profile.max_angular_speed_rad_s, passage.angular_cap if passage else float('inf'))
        msg.alignment_required=bool(passage and passage.alignment_heading is not None)
        msg.centering_required=bool(passage and passage.centering_target is not None)
        msg.corridor_tracking_required=bool(passage and passage.tracking_heading is not None)
        msg.workstation_alignment=workstation_alignment
        msg.planning=selection.planning;msg.reason=selection.reason
        timing={}
        if self.scan_timing is not None:
            timing['scan_timing_constraint']={'publish_started':{
                'ros_ns':self.get_clock().now().nanoseconds,'steady_ns':time.monotonic_ns()},
                'world_stamp_ns':self.last_world.stamp.ns,'world_version':asdict(self.last_world.version)}
        self.constraint.publish(msg)
        if self.scan_timing is not None:
            timing['scan_timing_constraint']['publish_finished']={
                'ros_ns':self.get_clock().now().nanoseconds,'steady_ns':time.monotonic_ns()}
            timing['scan_timing_constraint']['successful']=self.scan_timing.snapshot()['successful']
        status=NavigationPolicyStatus();status.stamp=msg.stamp
        to_wire(self.last_world.version,status.version)
        status.motion=selection.motion;status.reason=selection.reason;status.inputs_valid=valid
        status.lease_s=msg.lease_s;self.typed_state.publish(status)
        self.state.publish(String(data=json.dumps(dict(asdict(selection),stamp_ns=now.ns,
            coverage_ok=coverage_ok,
            coverage_motion={'vx':robot.vx,'vy':robot.vy,'wz':robot.wz} if robot else None,
            coverage_health=[{'sensor_id':h.sensor_id,
                              'required':h.sensor_id in self.health_registry.required,
                              'state':h.health.value,'reason':h.reason,
                              'capture_ns':h.stamp.ns,'valid_until_ns':h.valid_until.ns,
                              'depth_available':h.depth_available,
                              'coverage_yaw':[math.atan2(c.direction.y,c.direction.x) for c in h.coverage],
                              'coverage_half_angle':[c.half_angle_rad for c in h.coverage]}
                             for h in coverage_health],
            processing_wall_s=time.monotonic()-processing_start,
            processing_cpu_s=time.thread_time()-processing_cpu_start,
            processing_stages_s={'observation':observer_done-processing_start,
                                 'route_mailbox':route_done-observer_done,
                                 'corridor':corridor_done-corridor_start,
                                 'arbitration':time.monotonic()-route_done},
            corridor_state=passage.state if passage else 'DISABLED',
            corridor_id=passage.corridor_id if passage else '',
            corridor_permit=list(passage.permit) if passage else [],
            corridor_evidence=self.corridor.evidence if self.corridor else {},
            corridor_error=self.corridor.last_error if self.corridor else '',
            workstation_alignment=workstation_alignment,
            path_risk_status='RETIRED_FOR_WORKSTATION' if workstation_alignment else path_risk.status,path_distance_m=path_risk.distance_m,
            original_path_admission='SLOW_EXECUTABLE' if slow_original else 'STANDARD',
            candidate_audit=self.coordinator.audit[-12:] if self.coordinator else [],
            candidate_safety_evidence=self.coordinator.safety_evidence[-12:] if self.coordinator else [],**timing))))

def main():
    rclpy.init();node=PolicyNode()
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()
