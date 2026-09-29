"""ROS observation adapter; this node has no command or planning publishers."""
import json
import math
import os
import time
import threading
from dataclasses import asdict, replace

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan, CameraInfo
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory

from .contracts import (BearingCone, Covariance3, ImageBox, MetricBox, Observation,
                        Stamp, Vec3, Version, CameraCalibration)
from .fusion import ConservativeFusion
from .sensor_health import SensorHealthRegistry, CameraCalibrationRegistry, scan_coverage, movement_directions
from .observation_adapters import VisionAdapter, make_adapter, adapter_message_type
from .execution_context import ExecutionContext
from astribot_navigation_msgs.msg import NavigationExecutionStatus, SensorHealth as HealthMessage, SensorHealthArray, RobotEnvelope
from .profile import Profile
from .robot_envelope import configure_envelope_input
from .protection import scan_usable
from .risk import RobotState, evaluate_risk
from .world_geometry import has_predictions
from astribot_s1_robot_geometry._geometry_native import scan_boxes_free, scan_occupied_cells


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def rotate(v,q):
    # Quaternion rotation without numpy or a ROS dependency in the domain core.
    tx=2*(q.y*v.z-q.z*v.y);ty=2*(q.z*v.x-q.x*v.z);tz=2*(q.x*v.y-q.y*v.x)
    return Vec3(v.x+q.w*tx+q.y*tz-q.z*ty,v.y+q.w*ty+q.z*tx-q.x*tz,v.z+q.w*tz+q.x*ty-q.y*tx)


class PolicyObserver(Node):
    observation_only = True
    default_plan_topic = '/plan'
    def __init__(self):
        super().__init__('navigation_policy_observer')
        default=get_package_share_directory('astribot_s1_navigation_policy')+'/config/simulation.json'
        self.declare_parameter('profile',default)
        self.declare_parameter('scan_topic','/scan_from_cloud')
        self.declare_parameter('vision_topic','/navigation_policy/vision_observations')
        self.declare_parameter('plan_topic',self.default_plan_topic)
        self.profile,self.envelope_ack=configure_envelope_input(self,Profile.load(self.get_parameter('profile').value),self.envelope,'policy',1)
        self.profile.require_environment(self.get_parameter('use_sim_time').value)
        self.clock_id='sim' if self.get_parameter('use_sim_time').value else 'ros'
        self.configure_tf()
        self.fusion=ConservativeFusion(self.profile)
        self.execution=ExecutionContext()
        self.health_registry=SensorHealthRegistry(self.profile.sensor_timeout_s)
        self.calibrations=CameraCalibrationRegistry()
        self.declare_parameter('robot_base_frame','astribot_torso_base')
        self.base_frame=self.get_parameter('robot_base_frame').value
        self.declare_parameter('localization_jump_m',0.20)
        self.declare_parameter('localization_jump_rad',0.15)
        self.robot=None;self.odom_at=None;self.scan_at=None;self.map=None;self.path=()
        self.epoch=0;self.last_time=None;self.errors=0;self.scan_count=0;self.vision_count=0
        self.path_revision=0;self.last_risk=None
        self.last_inputs_valid=False;self.last_evaluation_epoch=0
        self.last_error=''
        self.pending_scans=[]
        self.scan_timing=None
        if os.environ.get('ASTRIBOT_SCAN_TIMING_DIAGNOSTICS','').lower() in ('1','true','yes'):
            from astribot_s1_navigation_policy_native import _navigation_math_native
            self.scan_timing=_navigation_math_native.ScanTiming()
        self.pending_plan=None
        self.scan_lock=threading.Lock()
        self.odom_lock=threading.Lock()
        self.envelope_lock=threading.Lock()
        self.pending_envelope=None
        self.processing_group=MutuallyExclusiveCallbackGroup()
        self.publisher=self.create_publisher(String,'/navigation_policy/observation',10)
        self.create_subscription(LaserScan,self.get_parameter('scan_topic').value,self.scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map',self.mapping,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL),callback_group=self.processing_group)
        plan_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if not self.observation_only else 10
        self.create_subscription(Path,self.get_parameter('plan_topic').value,self.plan,plan_qos)
        self.vision_adapter=VisionAdapter(self.profile,self.tf,self.stamp)
        self.declare_parameter('observation_sources','[]')
        self.source_adapters=[]
        self.health_pub=self.create_publisher(SensorHealthArray,'/navigation/sensor_health',10)
        for spec in json.loads(self.get_parameter('observation_sources').value):
            adapter=make_adapter(spec['adapter'],self.profile,self.tf,self.stamp,spec)
            self.source_adapters.append(adapter)
            if spec.get('required',False):self.health_registry.required.add(spec['sensor_id'])
            if spec.get('camera_info_topic'):
                self.create_subscription(CameraInfo,spec['camera_info_topic'],
                    lambda msg,a=adapter:self.camera_info(a,msg),qos_profile_sensor_data,
                    callback_group=self.processing_group)
            self.create_subscription(adapter_message_type(spec['adapter']),spec['topic'],
                lambda msg,a=adapter:self.accept_observations(a,msg),qos_profile_sensor_data,
                callback_group=self.processing_group)
        self.create_subscription(String,self.get_parameter('vision_topic').value,self.vision,10,
                                 callback_group=self.processing_group)
        self.create_subscription(NavigationExecutionStatus,'/navigation/execution_status',
            lambda m:self.execution.task(m.task_id,m.state,m.sequence),
            QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL),callback_group=self.processing_group)
        self.create_timer(.1,self.tick,callback_group=self.processing_group)

    def configure_tf(self):
        # Temporary compatibility seam: reception, cache, queries and thread
        # ownership are C++; the existing policy uses the same TF interface.
        self.native_tf=os.environ.get('ASTRIBOT_POLICY_NATIVE_TF','').lower() in ('1','true','yes')
        if self.native_tf:
            from astribot_s1_navigation_policy_native._native_tf_buffer import NativeTfBuffer
            self.tf=NativeTfBuffer(bool(self.get_parameter('use_sim_time').value))
            self.listener=None
        else:
            self.tf=Buffer();self.listener=TransformListener(self.tf,self)

    def destroy_node(self):
        try:
            if getattr(self,'native_tf',False) and getattr(self,'tf',None) is not None:self.tf.close()
        finally:
            result=super().destroy_node()
        return result

    def envelope(self,msg):
        # Receive heartbeats independently of risk computation; apply at its boundary.
        with self.envelope_lock:
            old=self.pending_envelope
            if old is not None and self.envelope_ack is not None and (old.coordinator_session_id,old.epoch,old.clock_epoch)==(msg.coordinator_session_id,msg.epoch,msg.clock_epoch):
                if (msg.header.stamp.sec,msg.header.stamp.nanosec)<(old.header.stamp.sec,old.header.stamp.nanosec):return
            self.pending_envelope=msg
        # A matching configuration was already applied. Confirm its live source
        # without mutating the profile being used by an in-flight decision.
        # Source timestamps order observations; execution budgets use steady time.
        if self.envelope_ack:
            now=Stamp(self.get_clock().now().nanoseconds,self.clock_id,self.epoch)
            if self.profile.confirms_applied(msg,now):self.envelope_ack(msg)

    def process_envelope(self):
        with self.envelope_lock:
            msg=self.pending_envelope;self.pending_envelope=None
        if msg is None:return
        try:
            if self.profile.accept(msg,self.stamp()):
                self.execution.version=replace(self.execution.version,envelope_epoch=msg.epoch)
                if self.envelope_ack:self.envelope_ack(msg)
        except ValueError as error:self.get_logger().warning(str(error))

    def stamp(self):
        ns=self.get_clock().now().nanoseconds
        self.last_time=ns
        return Stamp(ns,self.clock_id,self.epoch)

    def odom(self,msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation;v=msg.twist.twist
        values=(p.x,p.y,yaw(q),v.linear.x,v.linear.y,v.angular.z)
        if all(math.isfinite(x) for x in values) and msg.header.frame_id==self.profile.tracking_frame:
            with self.odom_lock:
                source=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                if self.odom_at is not None and source<=self.odom_at:return
                self.robot=RobotState(*values)
                self.odom_at=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def mapping(self,msg):
        info=msg.info
        if msg.header.frame_id!='map' or not math.isfinite(info.resolution) or info.resolution<=0 or len(msg.data)!=info.width*info.height or not msg.data:return
        origin=info.origin
        metadata=(info.width,info.height,info.resolution,origin.position.x,origin.position.y,
                  origin.orientation.x,origin.orientation.y,origin.orientation.z,origin.orientation.w)
        if not all(math.isfinite(v) for v in metadata):return
        self.execution.map(metadata,msg.data.tobytes())
        self.map=msg

    def plan(self,msg):
        self.pending_plan=msg

    def process_plan(self):
        msg=self.pending_plan
        if msg is None:return
        try:
            transform=self.tf.lookup_transform(self.profile.tracking_frame,msg.header.frame_id,Time())
            self.path=tuple(self.point((p.pose.position.x,p.pose.position.y,p.pose.position.z),transform)[:2] for p in msg.poses)
            self.path_revision+=1
            self.execution.path()
            self.fusion.version=self.execution.version
            if self.pending_plan is msg:self.pending_plan=None
        except Exception:
            self.path=();self.errors+=1

    @staticmethod
    def point(xyz,transform):
        x,y,z=xyz;q=transform.transform.rotation;t=transform.transform.translation
        if not all(math.isfinite(v) for v in xyz):raise ValueError('finite transform input required')
        tx=2*(q.y*z-q.z*y);ty=2*(q.z*x-q.x*z);tz=2*(q.x*y-q.y*x)
        rotated=(x+q.w*tx+q.y*tz-q.z*ty,y+q.w*ty+q.z*tx-q.x*tz,z+q.w*tz+q.x*ty-q.y*tx)
        if not all(math.isfinite(v) for v in rotated):raise ValueError('finite transform rotation required')
        return rotated[0]+t.x,rotated[1]+t.y,rotated[2]+t.z

    def static_at(self,x,y):
        if self.map is None:return False
        _,c,s,ox,oy,resolution,mask=self.static_mask()
        dx,dy=x-ox,y-oy
        ix=math.floor((c*dx+s*dy)/resolution)+2;iy=math.floor((-s*dx+c*dy)/resolution)+2
        return bool(mask[iy,ix]) if 0<=iy<mask.shape[0] and 0<=ix<mask.shape[1] else False

    def static_mask(self):
        cached=getattr(self,'static_lookup',None)
        if cached is None or cached[0] is not self.map:
            import numpy as np
            info=self.map.info;origin=info.origin.position;theta=yaw(info.origin.orientation)
            occupied=np.asarray(self.map.data).reshape((info.height,info.width))>=65
            # The same five-by-five neighborhood as the scalar lookup, including
            # queries up to two cells outside the map boundary.
            padded=np.pad(occupied,4);mask=np.zeros((info.height+4,info.width+4),dtype=bool)
            for dy in range(5):
                for dx in range(5):mask|=padded[dy:dy+info.height+4,dx:dx+info.width+4]
            cached=(self.map,math.cos(theta),math.sin(theta),origin.x,origin.y,info.resolution,mask)
            self.static_lookup=cached
        return cached

    def scan(self,msg):
        trace=self.scan_timing
        sequence=None
        if trace is not None:
            # Callback entry, not DDS wire receipt; do not call stamp() here.
            sequence=trace.receive(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                self.get_clock().now().nanoseconds,time.monotonic_ns(),msg.header.frame_id,self.epoch)
        if not scan_usable(msg.ranges,msg.range_min,msg.range_max,msg.angle_min,msg.angle_increment,
                           self.profile.scan_min_valid_fraction):
            if trace is not None:trace.drop(sequence,'invalid',self.get_clock().now().nanoseconds,time.monotonic_ns())
            return
        with self.scan_lock:
            self.pending_scans.append((msg,sequence) if trace is not None else msg)
            self.pending_scans.sort(key=lambda entry: ((entry[0] if trace is not None else entry).header.stamp.sec,
                                                      (entry[0] if trace is not None else entry).header.stamp.nanosec))
            discarded=self.pending_scans[:-5] if trace is not None else ()
            self.pending_scans=self.pending_scans[-5:]
        for _,sequence in discarded:
            trace.drop(sequence,'queue_capacity',self.get_clock().now().nanoseconds,time.monotonic_ns())

    def process_scans(self, now):
        trace=self.scan_timing
        if self.map is None:
            if trace is not None:
                with self.scan_lock:waiting=list(self.pending_scans)
                for _,sequence in waiting:
                    trace.defer(sequence,'no_map',self.get_clock().now().nanoseconds,time.monotonic_ns())
            return
        ready=[];pending=[]
        with self.scan_lock:
            batch=self.pending_scans;self.pending_scans=[]
        for entry in batch:
            msg,sequence=entry if trace is not None else (entry,None)
            source=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if self.scan_at is not None and source<=self.scan_at:
                if trace is not None:trace.drop(sequence,'superseded',self.get_clock().now().nanoseconds,time.monotonic_ns())
                continue
            tracking=self.tf.can_transform(self.profile.tracking_frame,msg.header.frame_id,Time())
            mapped=self.tf.can_transform('map',msg.header.frame_id,Time()) if tracking else False
            available=tracking and mapped
            if trace is not None:
                trace.tf_check(sequence,int(bool(tracking)),int(bool(mapped)) if tracking else -1,
                    self.get_clock().now().nanoseconds,time.monotonic_ns())
            if available:ready.append(entry)
            else:pending.append(entry)
        with self.scan_lock:
            combined=pending+self.pending_scans
            combined.sort(key=lambda entry: ((entry[0] if trace is not None else entry).header.stamp.sec,
                                            (entry[0] if trace is not None else entry).header.stamp.nanosec))
            discarded=combined[:-5] if trace is not None else ()
            self.pending_scans=combined[-5:]
        for _,sequence in discarded:
            trace.drop(sequence,'queue_capacity',self.get_clock().now().nanoseconds,time.monotonic_ns())
        if ready:
            ready.sort(key=lambda entry: ((entry[0] if trace is not None else entry).header.stamp.sec,
                                         (entry[0] if trace is not None else entry).header.stamp.nanosec))
            if trace is None:self.process_scan(ready[-1])
            else:
                for _,sequence in ready[:-1]:
                    trace.drop(sequence,'superseded',self.get_clock().now().nanoseconds,time.monotonic_ns())
                msg,sequence=ready[-1]
                trace.select(sequence,self.get_clock().now().nanoseconds,time.monotonic_ns())
                before=self.scan_count
                self.process_scan(msg)
                success=self.scan_count>before
                trace.finish(sequence,success,self.get_clock().now().nanoseconds,time.monotonic_ns(),
                    '' if success else self.last_error or 'PROCESSING_NOT_COMMITTED')

    def process_scan(self,msg):
        try:
            now=self.stamp();capture=Stamp(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,self.clock_id,self.epoch)
            if self.scan_at is not None and capture.ns*1e-9<=self.scan_at:return
            transform=self.tf.lookup_transform(self.profile.tracking_frame,msg.header.frame_id,Time())
            to_map=self.tf.lookup_transform('map',msg.header.frame_id,Time())
            observations=[]
            cell_size=self.profile.scan_occupancy_resolution_m
            def tf_values(tf):
                t=tf.transform.translation;q=tf.transform.rotation
                return (t.x,t.y,t.z,q.x,q.y,q.z,q.w)
            _,c,s,ox,oy,resolution,mask=self.static_mask()
            cells=scan_occupied_cells(msg.ranges,msg.range_min,msg.range_max,msg.angle_min,
                msg.angle_increment,tf_values(transform),tf_values(to_map),
                (c,s,ox,oy,resolution),mask,cell_size)
            for ix,iy in cells:
                cell=f'cell:{cell_size:g}:{ix}:{iy}'
                x,y=(float(ix)+.5)*cell_size,(float(iy)+.5)*cell_size
                box=MetricBox(Vec3(x,y,self.profile.height_m/2),
                              Vec3(cell_size+2*self.profile.scan_obstacle_padding_m,
                                   cell_size+2*self.profile.scan_obstacle_padding_m,self.profile.height_m),
                              Covariance3((.0004,0.,0.,0.,.0004,0.,0.,0.,.01)))
                observations.append(Observation('scan',f'{capture.ns}:{cell}',cell,
                    capture,Stamp(time.monotonic_ns(),'steady',0),
                    Stamp(capture.ns+int(self.profile.sensor_timeout_s*1e9),self.clock_id,self.epoch),
                    self.profile.tracking_frame,0,box,1.,(),(f'scan:{capture.ns}:{cell}',),
                    velocity_observable=False,spatial_occupancy=True))
            self.fusion.ingest(tuple(observations),now)
            inv=self.tf.lookup_transform(msg.header.frame_id,self.profile.tracking_frame,Time())
            def free_many(boxes):
                t=inv.transform.translation;q=inv.transform.rotation
                return scan_boxes_free(
                    [(b.center_m.x,b.center_m.y,b.size_m.x,b.size_m.y) for b in boxes],
                    (t.x,t.y,t.z,q.x,q.y,q.z,q.w),msg.ranges,msg.range_min,msg.range_max,
                    msg.angle_min,msg.angle_increment,cell_size)
            self.fusion.clear_observed_free(now,free_many=free_many)
            body_tf=self.tf.lookup_transform(self.base_frame,msg.header.frame_id,Time())
            coverage=scan_coverage(msg.ranges,msg.range_min,msg.range_max,msg.angle_min,msg.angle_increment,
                                   yaw(body_tf.transform.rotation))
            self.health_registry.record('scan',capture,now,self.base_frame,coverage,True,0)
            self.scan_at=capture.ns*1e-9;self.scan_count+=1
        except Exception as exc:
            self.errors+=1
            self.last_error=str(exc)
            self.get_logger().debug(f'scan rejected: {exc}')

    def vision(self,msg):
        self.accept_observations(self.vision_adapter,msg)

    def camera_info(self,adapter,msg):
        sensor=adapter.options['sensor_id']
        old=self.calibrations.records.get(sensor)
        epoch=old.calibration_epoch if old else 0
        fields=(sensor,msg.header.frame_id,epoch,msg.width,msg.height,tuple(msg.k),
                msg.distortion_model,tuple(msg.d),adapter.options.get('depth_unit_m'))
        try:
            candidate=CameraCalibration(*fields)
            if old and candidate!=old:candidate=replace(candidate,calibration_epoch=epoch+1)
            self.calibrations.register(candidate)
            adapter.options['calibration_epoch']=candidate.calibration_epoch
        except (ValueError,KeyError) as error:self.get_logger().warning(str(error))

    def accept_observations(self,adapter,msg):
        try:
            observations=adapter.normalize(msg)
            now=self.stamp();data=adapter.last_packet
            capture=Stamp(int(data['stamp_ns']),now.clock,now.epoch)
            sensor=data['sensor_id'];epoch=int(data['calibration_epoch'])
            if adapter.options.get('camera_info_topic'):self.calibrations.calibration(sensor,epoch)
            old=self.health_registry.records.get(sensor)
            if old and (epoch<old.calibration_epoch or (old.stamp.clock,old.stamp.epoch)==(now.clock,now.epoch) and capture.ns<=old.stamp.ns):
                return
            coverage=tuple(BearingCone(Vec3(math.cos(c[0]),math.sin(c[0]),0.),c[1])
                for c in adapter.options.get('coverage_body_yaw_half_angle',[]))
            depth=bool(observations) and all(isinstance(o.geometry,MetricBox) for o in observations)
            self.fusion.ingest(observations,now)
            self.health_registry.record(sensor,capture,now,self.base_frame,coverage,depth,epoch)
            self.fusion.resolve_unassociated(data['sensor_id'],data.get('resolved_measurement_ids',()),capture,now)
            self.vision_count+=1
        except Exception as exc:
            self.errors+=1;self.get_logger().warning(f'observation rejected: {exc}')

    def tick(self):
        processing_start=time.monotonic()
        processing_cpu_start=time.thread_time()
        self.process_envelope()
        now=self.stamp();seconds=now.ns*1e-9
        try:
            tf=self.tf.lookup_transform(self.profile.tracking_frame,'map',Time())
            p=tf.transform.translation
            jumped=self.execution.localization((p.x,p.y,yaw(tf.transform.rotation)),
                self.get_parameter('localization_jump_m').value,self.get_parameter('localization_jump_rad').value)
            if jumped and self.path:
                self.path=();self.last_error='LOCALIZATION_CHANGED_REQUIRES_PATH_REFRESH'
        except Exception:pass
        self.execution.version=replace(self.execution.version,clock_epoch=now.epoch)
        self.process_plan()
        self.fusion.version=self.execution.version
        scan_start=time.monotonic()
        self.process_scans(now)
        scan_elapsed=time.monotonic()-scan_start
        with self.odom_lock:
            robot=self.robot;odom_at=self.odom_at
        scan_at=self.scan_at
        now=self.stamp()
        region=(robot.x,robot.y,max(self.profile.max_speed_m_s,math.hypot(robot.vx,robot.vy))*
                self.profile.prediction_horizon_s) if robot else None
        self.fusion.sensors=self.health_registry.health(now)
        health=SensorHealthArray();health.stamp=self.get_clock().now().to_msg()
        for source in self.fusion.sensors:
            h=HealthMessage();h.sensor_id=source.sensor_id;h.state=source.health.value
            h.frame_id=source.frame_id;h.capture_stamp=Time(nanoseconds=source.stamp.ns).to_msg()
            h.valid_until=Time(nanoseconds=source.valid_until.ns).to_msg();h.calibration_epoch=source.calibration_epoch
            h.depth_available=source.depth_available;h.reason=source.reason
            h.coverage_yaw=[math.atan2(c.direction.y,c.direction.x) for c in source.coverage]
            h.coverage_half_angle=[c.half_angle_rad for c in source.coverage];health.sensors.append(h)
        self.health_pub.publish(health)
        snapshot_start=time.monotonic()
        world=self.fusion.snapshot(now,region)
        risk_start=time.monotonic()
        risk=evaluate_risk(world,robot,self.path,self.profile) if robot else None
        risk_elapsed=time.monotonic()-risk_start
        self.last_risk=risk
        self.last_world=world;self.last_robot=robot
        seconds=self.get_clock().now().nanoseconds*1e-9
        fresh=scan_at is not None and odom_at is not None and self.map is not None
        fresh=fresh and self.health_registry.required_valid(now) and (self.observation_only or self.profile.ready(now))
        self.last_inputs_valid=fresh;self.last_evaluation_epoch=now.epoch
        blocking=[]
        if risk:
            for identifier in dict.fromkeys((*risk.obstacle_ids,*risk.immediate_obstacle_ids)):
                track=self.fusion.tracks[identifier];o=track.observation;b=o.geometry
                blocking.append({'id':identifier,'center_m':asdict(b.center_m),'size_m':asdict(b.size_m),
                                 'velocity_m_s':asdict(track.velocity),'age_s':seconds-o.capture_stamp.ns*1e-9})
        def number(v):return v if math.isfinite(v) else None
        result={'stamp_ns':now.ns,'epoch':now.epoch,'observation_only':self.observation_only,'inputs_valid':fresh,
                'scan_count':self.scan_count,'vision_count':self.vision_count,'errors':self.errors,
                'last_error':self.last_error,'scan_age_s':seconds-scan_at if scan_at is not None else None,
                'odom_age_s':seconds-odom_at if odom_at is not None else None,
                'required_sensors_valid':self.health_registry.required_valid(now),
                'envelope_ready':self.profile.ready(now),
                'envelope_reason':self.profile.envelope.reason if self.profile.envelope else 'NO_ENVELOPE',
                'envelope_age_s':(now.ns-self.profile.envelope.stamp.sec*10**9-self.profile.envelope.stamp.nanosec)*1e-9 if self.profile.envelope else None,
                'tracks':len(world.tracks),'unassociated':len(world.unassociated),'path_revision':self.path_revision,
                'predicted_tracks':sum(has_predictions(t) for t in world.tracks),
                'processing_wall_s':time.monotonic()-processing_start,'blocking_tracks':blocking,
                'geometry_backend':'cpp',
                'processing_cpu_s':time.thread_time()-processing_cpu_start,
                'processing_stages_s':{'scan':scan_elapsed,'snapshot':risk_start-snapshot_start,'risk':risk_elapsed},
                'risk':{k:number(v) if isinstance(v,float) else v for k,v in asdict(risk).items()} if risk else None}
        if self.scan_timing is not None:
            result['scan_timing']=self.scan_timing.snapshot()
            result['scan_timing']['source_topic']=self.get_parameter('scan_topic').value
            result['scan_timing']['observation_pre_serialize']={
                'ros_ns':self.get_clock().now().nanoseconds,'steady_ns':time.monotonic_ns()}
            result['tf_backend']='cpp' if self.native_tf else 'python'
            if self.native_tf:result['tf_backend_diagnostics']=self.tf.diagnostics()
        self.publisher.publish(String(data=json.dumps(result,allow_nan=False)))


def main():
    rclpy.init();node=PolicyObserver()
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()
