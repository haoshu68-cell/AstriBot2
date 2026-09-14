"""ROS observation adapter; this node has no command or planning publishers."""
import json
import math
import time
import threading
from dataclasses import asdict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory

from .contracts import (BearingCone, Covariance3, ImageBox, MetricBox, Observation,
                        Stamp, Vec3, Version)
from .fusion import ConservativeFusion
from .profile import Profile
from .protection import scan_usable
from .risk import RobotState, evaluate_risk


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
        self.profile=Profile.load(self.get_parameter('profile').value)
        self.profile.require_environment(self.get_parameter('use_sim_time').value)
        self.clock_id='sim' if self.get_parameter('use_sim_time').value else 'ros'
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.fusion=ConservativeFusion(self.profile)
        self.robot=None;self.odom_at=None;self.scan_at=None;self.map=None;self.path=()
        self.epoch=0;self.last_time=None;self.errors=0;self.scan_count=0;self.vision_count=0
        self.path_revision=0;self.last_risk=None
        self.last_inputs_valid=False;self.last_evaluation_epoch=0
        self.last_error=''
        self.pending_scans=[]
        self.pending_plan=None
        self.scan_lock=threading.Lock()
        self.odom_lock=threading.Lock()
        self.processing_group=MutuallyExclusiveCallbackGroup()
        self.publisher=self.create_publisher(String,'/navigation_policy/observation',10)
        self.create_subscription(LaserScan,self.get_parameter('scan_topic').value,self.scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map',self.mapping,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        plan_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if not self.observation_only else 10
        self.create_subscription(Path,self.get_parameter('plan_topic').value,self.plan,plan_qos)
        self.create_subscription(String,self.get_parameter('vision_topic').value,self.vision,10,
                                 callback_group=self.processing_group)
        self.create_timer(.1,self.tick,callback_group=self.processing_group)

    def stamp(self):
        ns=self.get_clock().now().nanoseconds
        if self.last_time is not None and ns<self.last_time:
            self.epoch+=1;self.scan_at=None;self.odom_at=None;self.path=();self.robot=None
            with self.scan_lock:self.pending_scans=[]
            self.pending_plan=None
        self.last_time=ns
        return Stamp(ns,self.clock_id,self.epoch)

    def odom(self,msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation;v=msg.twist.twist
        values=(p.x,p.y,yaw(q),v.linear.x,v.linear.y,v.angular.z)
        if all(math.isfinite(x) for x in values) and msg.header.frame_id==self.profile.tracking_frame:
            with self.odom_lock:
                self.robot=RobotState(*values)
                self.odom_at=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def mapping(self,msg):
        if msg.header.frame_id=='map' and msg.info.resolution>0:self.map=msg

    def plan(self,msg):
        self.pending_plan=msg

    def process_plan(self):
        msg=self.pending_plan
        if msg is None:return
        try:
            transform=self.tf.lookup_transform(self.profile.tracking_frame,msg.header.frame_id,Time())
            self.path=tuple(self.point((p.pose.position.x,p.pose.position.y,p.pose.position.z),transform)[:2] for p in msg.poses)
            self.path_revision+=1
            self.fusion.version=Version('observed-route',self.path_revision,0,0)
            if self.pending_plan is msg:self.pending_plan=None
        except Exception:
            self.path=();self.errors+=1

    @staticmethod
    def point(xyz,transform):
        v=rotate(Vec3(*xyz),transform.transform.rotation);t=transform.transform.translation
        return v.x+t.x,v.y+t.y,v.z+t.z

    def static_at(self,x,y):
        if self.map is None:return False
        info=self.map.info;origin=info.origin.position;theta=yaw(info.origin.orientation)
        dx,dy=x-origin.x,y-origin.y;c,s=math.cos(theta),math.sin(theta)
        ix=int(math.floor((c*dx+s*dy)/info.resolution));iy=int(math.floor((-s*dx+c*dy)/info.resolution))
        for a in range(ix-2,ix+3):
            for b in range(iy-2,iy+3):
                if 0<=a<info.width and 0<=b<info.height and self.map.data[b*info.width+a]>=65:return True
        return False

    def scan(self,msg):
        if not scan_usable(msg.ranges,msg.range_min,msg.range_max,msg.angle_min,msg.angle_increment,
                           self.profile.scan_min_valid_fraction):return
        with self.scan_lock:
            self.pending_scans.append(msg)
            self.pending_scans=self.pending_scans[-5:]

    def process_scans(self, now):
        if self.map is None:return
        ready=[];pending=[]
        with self.scan_lock:
            batch=self.pending_scans;self.pending_scans=[]
        for msg in batch:
            capture=Time.from_msg(msg.header.stamp)
            age=(now.ns-capture.nanoseconds)*1e-9
            if age>self.profile.sensor_timeout_s:continue
            if age>=0 and all(self.tf.can_transform(frame,msg.header.frame_id,capture)
                            for frame in (self.profile.tracking_frame,'map')):
                ready.append(msg)
            else:pending.append(msg)
        with self.scan_lock:
            self.pending_scans=(pending+self.pending_scans)[-5:]
        if ready:self.process_scan(ready[-1])

    def process_scan(self,msg):
        try:
            now=self.stamp();capture=Stamp(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,self.clock_id,self.epoch)
            if not 0<=now.since(capture)<=self.profile.sensor_timeout_s*1e9:
                self.last_error=f'scan capture age {now.since(capture)*1e-9:.3f}s exceeds budget'
                return
            transform=self.tf.lookup_transform(self.profile.tracking_frame,msg.header.frame_id,Time.from_msg(msg.header.stamp))
            to_map=self.tf.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp))
            clusters=[];current=[]
            for i,r in enumerate(msg.ranges):
                if not math.isfinite(r) or not msg.range_min<=r<msg.range_max-.05:
                    if current:clusters.append(current);current=[]
                    continue
                angle=msg.angle_min+i*msg.angle_increment
                xyz=(r*math.cos(angle),r*math.sin(angle),0.)
                mx,my,_=self.point(xyz,to_map)
                if self.static_at(mx,my):
                    if current:clusters.append(current);current=[]
                    continue
                x,y,_=self.point(xyz,transform)
                if current and math.dist(current[-1],(x,y))>.25:clusters.append(current);current=[]
                current.append((x,y))
            if current:clusters.append(current)
            observations=[]
            for i,cluster in enumerate(clusters):
                xs,ys=zip(*cluster);x,y=(min(xs)+max(xs))/2,(min(ys)+max(ys))/2
                box=MetricBox(Vec3(x,y,self.profile.height_m/2),Vec3(max(.08,max(xs)-min(xs)),max(.08,max(ys)-min(ys)),self.profile.height_m),
                              Covariance3((.0004,0.,0.,0.,.0004,0.,0.,0.,.01)))
                observations.append(Observation('scan',f'{capture.ns}:{i}',None,capture,Stamp(time.monotonic_ns(),'steady',0),
                    Stamp(capture.ns+int(self.profile.sensor_timeout_s*1e9),self.clock_id,self.epoch),self.profile.tracking_frame,0,box,1.,(),(f'scan:{capture.ns}:{i}',),
                    velocity_observable=False))
            self.fusion.ingest(tuple(observations),now)
            inv=self.tf.lookup_transform(msg.header.frame_id,self.profile.tracking_frame,Time.from_msg(msg.header.stamp))
            def free_at(box):
                for dx,dy in ((0,0),(-1,-1),(-1,1),(1,-1),(1,1)):
                    x,y,_=self.point((box.center_m.x+dx*box.size_m.x/2,box.center_m.y+dy*box.size_m.y/2,0.),inv)
                    distance=math.hypot(x,y);a=math.atan2(y,x);index=int(round((a-msg.angle_min)/msg.angle_increment))
                    if not msg.range_min<=distance<msg.range_max-.2 or not 0<=index<len(msg.ranges):return False
                    observed=msg.ranges[index]
                    if math.isnan(observed) or observed==-math.inf:return False
                    if min(observed,msg.range_max)<distance+.15:return False
                return True
            self.fusion.clear_observed_free(now,free_at)
            self.scan_at=capture.ns*1e-9;self.scan_count+=1
        except Exception as exc:
            self.errors+=1
            self.last_error=str(exc)
            self.get_logger().debug(f'scan rejected: {exc}')

    def vision(self,msg):
        try:
            data=json.loads(msg.data)
            if data.get('schema_version')!=1:raise ValueError('vision schema_version')
            now=self.stamp();capture=Stamp(int(data['stamp_ns']),self.clock_id,self.epoch)
            observations=[]
            for item in data['observations']:
                frame=data['frame_id'];kind=item['kind']
                if kind=='metric_box':
                    transform=self.tf.lookup_transform(self.profile.tracking_frame,frame,Time(nanoseconds=capture.ns))
                    center=self.point(item['center_m'],transform);size=Vec3(*item['size_m']);q=transform.transform.rotation
                    if min(size.x,size.y,size.z)<=0:raise ValueError('positive metric dimensions required')
                    axes=[rotate(Vec3(size.x,0,0),q),rotate(Vec3(0,size.y,0),q),rotate(Vec3(0,0,size.z),q)]
                    extent=Vec3(*[sum(abs(getattr(v,k)) for v in axes) for k in ('x','y','z')])
                    # A scalar isotropic uncertainty is explicitly supplied, not inferred from class confidence.
                    variance=float(item['position_variance_m2'])
                    velocity=None;velocity_covariance=None
                    if 'velocity_m_s' in item or 'velocity_variance_m2_s2' in item:
                        velocity=rotate(Vec3(*item['velocity_m_s']),q)
                        vv=float(item['velocity_variance_m2_s2'])
                        velocity_covariance=Covariance3((vv,0.,0.,0.,vv,0.,0.,0.,vv))
                    geometry=MetricBox(Vec3(*center),extent,Covariance3((variance,0.,0.,0.,variance,0.,0.,0.,variance)),
                                       velocity,velocity_covariance)
                    frame=self.profile.tracking_frame
                elif kind=='image_box':
                    geometry=ImageBox(data['sensor_id'],*item['image_size_px'],*item['box_xyxy_px'])
                elif kind=='bearing_cone':geometry=BearingCone(Vec3(*item['direction']),item['half_angle_rad'])
                else:raise ValueError('unsupported vision geometry')
                observations.append(Observation(data['sensor_id'],item['measurement_id'],item.get('track_id'),capture,
                    Stamp(time.monotonic_ns(),'steady',0),Stamp(capture.ns+int(self.profile.sensor_timeout_s*1e9),self.clock_id,self.epoch),
                    frame,int(data['calibration_epoch']),geometry,float(item['geometry_quality']),
                    tuple((str(k),float(v)) for k,v in item.get('classes',{}).items()),tuple(item['provenance']),
                    velocity_observable=bool(item.get('track_id'))))
            self.fusion.ingest(tuple(observations),now)
            self.fusion.resolve_unassociated(data['sensor_id'],data.get('resolved_measurement_ids',()),capture,now)
            self.vision_count+=1
        except Exception as exc:
            self.errors+=1;self.get_logger().warning(f'vision rejected: {exc}')

    def tick(self):
        processing_start=time.monotonic()
        now=self.stamp();seconds=now.ns*1e-9
        self.process_plan()
        self.process_scans(now)
        with self.odom_lock:
            robot=self.robot;odom_at=self.odom_at
        scan_at=self.scan_at
        now=self.stamp()
        region=(robot.x,robot.y,max(self.profile.max_speed_m_s,math.hypot(robot.vx,robot.vy))*
                self.profile.prediction_horizon_s) if robot else None
        world=self.fusion.snapshot(now,region)
        risk=evaluate_risk(world,robot,self.path,self.profile) if robot else None
        self.last_risk=risk
        seconds=self.get_clock().now().nanoseconds*1e-9
        fresh=(scan_at is not None and odom_at is not None and self.map is not None and
               0<=seconds-scan_at<=self.profile.sensor_timeout_s and 0<=seconds-odom_at<=self.profile.sensor_timeout_s)
        self.last_inputs_valid=fresh;self.last_evaluation_epoch=now.epoch
        blocking=[]
        if risk:
            for identifier in risk.obstacle_ids:
                track=self.fusion.tracks[identifier];o=track.observation;b=o.geometry
                blocking.append({'id':identifier,'center_m':asdict(b.center_m),'size_m':asdict(b.size_m),
                                 'velocity_m_s':asdict(track.velocity),'age_s':seconds-o.capture_stamp.ns*1e-9})
        def number(v):return v if math.isfinite(v) else None
        result={'stamp_ns':now.ns,'epoch':now.epoch,'observation_only':self.observation_only,'inputs_valid':fresh,
                'scan_count':self.scan_count,'vision_count':self.vision_count,'errors':self.errors,
                'last_error':self.last_error,'scan_age_s':seconds-scan_at if scan_at is not None else None,
                'odom_age_s':seconds-odom_at if odom_at is not None else None,
                'tracks':len(world.tracks),'unassociated':len(world.unassociated),'path_revision':self.path_revision,
                'predicted_tracks':sum(bool(t.predictions) for t in world.tracks),
                'processing_wall_s':time.monotonic()-processing_start,'blocking_tracks':blocking,
                'risk':{k:number(v) if isinstance(v,float) else v for k,v in asdict(risk).items()} if risk else None}
        self.publisher.publish(String(data=json.dumps(result,allow_nan=False)))


def main():
    rclpy.init();node=PolicyObserver()
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()
