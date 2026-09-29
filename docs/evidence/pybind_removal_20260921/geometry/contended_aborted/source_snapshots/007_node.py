"""Read-only geometry producer; no robot command publishers/services/actions."""
import copy
import hashlib
import json
import math
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Polygon, Point32
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import GetPlanningScene
from rcl_interfaces.srv import GetParameters
from rosidl_runtime_py.convert import message_to_ordereddict
from astribot_navigation_msgs.msg import RobotGeometryState, EnvelopeSlice
from .model import RobotModel, Shape
from .polygon import hull,inflate
from .state import JointSnapshot
from .projection_contract import PARAMETERS,projection_ceiling


def stamp(value): return Time(sec=int(value//10**9),nanosec=int(value%10**9))
def ns(value): return value.sec*10**9+value.nanosec
def polygon(points):
    # Float32 ROS transport may turn almost-collinear vertices inward. Add a
    # bounded serialization reserve, then canonicalize the transmitted values.
    scale=max(1.,float(np.max(np.abs(points))))
    points=hull(np.asarray(inflate(points,8*np.finfo(np.float32).eps*scale),dtype=np.float32))
    return Polygon(points=[Point32(x=float(x),y=float(y),z=0.) for x,y in points])
def pose_matrix(pose):
    p,q=pose.position,pose.orientation;out=np.eye(4)
    out[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();out[:3,3]=[p.x,p.y,p.z]
    return out


class GeometryNode(Node):
    def __init__(self):
        super().__init__('robot_geometry_state')
        self.declare_parameter('base_frame','astribot_torso_base')
        self.declare_parameter('hold_error_rad',.003)
        self.declare_parameter('model_padding_m',.01)
        self.declare_parameter('max_joint_age_s',.3)
        self.declare_parameter('max_joint_skew_s',.1)
        self.frame=self.get_parameter('base_frame').value
        self.error=self.get_parameter('hold_error_rad').value;self.padding=self.get_parameter('model_padding_m').value
        if not 0<self.error<=.025 or not 0<=self.padding<=.1:raise ValueError('invalid geometry margins')
        self.model=None;self.samples=None;self.sequence=0;self.source=str(uuid.uuid4())
        self.urdf_hash='';self.params_future=None;self.scene_future=None
        self.attachments=None;self.attachment_revision='';self.attachment_stamp=-1;self.attachment_ids=[]
        self.last_params=-float('inf');self.last_scene=-float('inf');self.error_reason='MODEL_UNAVAILABLE'
        self.params=self.create_client(GetParameters,'/robot_state_publisher/get_parameters')
        self.scene=self.create_client(GetPlanningScene,'/get_planning_scene')
        self.coverage=self.create_client(GetParameters,'/pointcloud_slice_scan_node/get_parameters')
        self.coverage_future=None;self.coverage_at=-1;self.coverage_max=None;self.last_coverage=-float('inf')
        self.pub=self.create_publisher(RobotGeometryState,'/navigation/geometry_state',10)
        self.attached_pub=self.create_publisher(PlanningScene,'/navigation/attached_geometry',10)
        self.filter_revision='';self.filter_at=-math.inf
        self.create_subscription(String,'/navigation/attachment_filter_applied',self.filter_ack,10)
        self.worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='robot_geometry')
        self.work=None;self.work_context=None;self.input_generation=0;self.next_sample=-math.inf
        self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
        # FK/polygon construction takes longer than a joint or /clock period.
        # Keep acquisition callbacks responsive; only immutable snapshots leave
        # the executor. A finished calculation cannot renew its source stamps.
        self.create_timer(.02,self.tick)

    def joints(self,msg):
        if self.samples is None:return
        try:self.samples.receive(msg.name,msg.position,ns(msg.header.stamp),self.get_clock().now().nanoseconds)
        except ValueError as error:
            self.error_reason=str(error);self.samples.clear();self.input_generation+=1

    def filter_ack(self,msg):
        self.filter_revision=msg.data;self.filter_at=time.monotonic()

    def refresh_inputs(self,now):
        wall=time.monotonic()
        # Discovery can expose a service before the first response is deliverable.
        # Retry bounded read-only queries without refreshing stale evidence.
        for future_name,client,last,timeout in (
            ('params_future',self.params,self.last_params,2.),
            ('scene_future',self.scene,self.last_scene,.5),
            ('coverage_future',self.coverage,self.last_coverage,.5)):
            future=getattr(self,future_name)
            if future is not None and not future.done() and wall-last>timeout:
                client.remove_pending_request(future);future.cancel();setattr(self,future_name,None)
        if self.coverage_future is not None and self.coverage_future.done():
            response=self.coverage_future.result();self.coverage_future=None;self.coverage_max=None
            from rclpy.parameter import parameter_value_to_python
            values=dict(zip(PARAMETERS,map(parameter_value_to_python,response.values)))
            self.coverage_max=projection_ceiling(values,self.frame);self.coverage_at=now
        if self.coverage_future is None and wall-self.last_coverage>.5 and self.coverage.service_is_ready():
            self.coverage_future=self.coverage.call_async(GetParameters.Request(names=list(PARAMETERS)))
            self.last_coverage=wall
        if self.params_future is not None and self.params_future.done():
            response=self.params_future.result();self.params_future=None
            urdf=response.values[0].string_value
            sha=hashlib.sha256(urdf.encode()).hexdigest()
            if sha!=self.urdf_hash:
                self.model=None;self.samples=None
                model=RobotModel(urdf,self.frame)
                self.model=model;self.urdf_hash=sha
                self.samples=JointSnapshot(model.required,
                    round(self.get_parameter('max_joint_age_s').value*1e9),
                    round(self.get_parameter('max_joint_skew_s').value*1e9))
        if self.params_future is None and wall-self.last_params>2. and self.params.service_is_ready():
            self.params_future=self.params.call_async(GetParameters.Request(names=['robot_description']))
            self.last_params=wall
        if self.scene_future is not None and self.scene_future.done():
            response=self.scene_future.result();self.scene_future=None
            items=copy.deepcopy(response.scene.robot_state.attached_collision_objects)
            self.attachments=None;self.attachment_stamp=-1
            for item in items:item.object.header.stamp=Time()
            revision=hashlib.sha256(json.dumps([message_to_ordereddict(x) for x in items],sort_keys=True).encode()).hexdigest()
            bodies=[]
            for item in items:
                obj=item.object
                if obj.header.frame_id not in ('',item.link_name) or obj.meshes or obj.planes or len(obj.primitives)!=len(obj.primitive_poses):
                    raise ValueError('unsupported attachment geometry/frame')
                for primitive,pose in zip(obj.primitives,obj.primitive_poses):
                    if primitive.type==primitive.BOX:kind='box';dimensions=list(primitive.dimensions)
                    elif primitive.type==primitive.SPHERE:kind='sphere';dimensions=list(primitive.dimensions)
                    elif primitive.type==primitive.CYLINDER:
                        kind='cylinder';dimensions=[primitive.dimensions[1],primitive.dimensions[0]]
                    else:raise ValueError('unsupported attachment primitive')
                    bodies.append(Shape(item.link_name,kind,dimensions,pose_matrix(obj.pose)@pose_matrix(pose)))
            self.attachments=bodies;self.attachment_revision=revision;self.attachment_stamp=now
            self.attachment_ids=[x.object.id for x in items]
            snapshot=PlanningScene(name=revision)
            snapshot.robot_state.joint_state.header.stamp=stamp(now)
            snapshot.robot_state.attached_collision_objects=items
            self.attached_pub.publish(snapshot)
        if self.scene_future is None and wall-self.last_scene>.2 and self.scene.service_is_ready():
            request=GetPlanningScene.Request();request.components.components=4
            self.scene_future=self.scene.call_async(request);self.last_scene=wall

    @staticmethod
    def calculate(model,q,errors,attachments,padding,msg):
        geometry=model.geometry(q,errors,attachments,padding)
        msg.physical_footprint=polygon(geometry['physical']);msg.reserved_footprint=polygon(geometry['reserved'])
        msg.height_m=geometry['height']
        msg.height_slices=[EnvelopeSlice(z_min_m=s['z_min'],z_max_m=s['z_max'],footprint=polygon(s['footprint'])) for s in geometry['slices']]
        return msg

    def new_message(self,now):
        msg=RobotGeometryState();msg.header.frame_id=self.frame;msg.published_at=stamp(now);msg.source_id=self.source
        self.sequence+=1;msg.sequence=self.sequence
        return msg

    def tick(self):
        now=self.get_clock().now().nanoseconds
        try:
            self.refresh_inputs(now)
            if self.work is not None:
                if not self.work.done():return
                work=self.work;self.work=None
                msg=work.result()
                generation,model,revision,epoch=self.work_context
                if (generation!=self.input_generation or model is not self.model or self.attachments is None or
                    revision!=self.attachment_revision or epoch!=self.samples.epoch):
                    raise ValueError('GEOMETRY_INPUT_CHANGED_DURING_COMPUTE')
                if not ns(msg.header.stamp)<=now<ns(msg.valid_until):raise ValueError('GEOMETRY_COMPUTE_EXCEEDED_SOURCE_LEASE')
                if self.coverage_max is None or not 0<=now-self.coverage_at<=1_500_000_000:
                    raise ValueError('HEIGHT_FILTER_CONFIGURATION_UNAVAILABLE')
                if msg.height_m>self.coverage_max:raise ValueError('GEOMETRY_EXCEEDS_CONFIGURED_HEIGHT_PROJECTION')
                filter_age=time.monotonic()-self.filter_at
                if self.filter_revision!=revision or filter_age>.5:
                    raise ValueError('ATTACHMENT_FILTER_UNCONFIRMED: '
                        f'revision_match={self.filter_revision==revision} wall_age_s={filter_age:.6f}')
                msg.published_at=stamp(now);msg.complete=True;msg.attachment_state_confirmed=True;msg.reason='GEOMETRY_CURRENT'
                self.pub.publish(msg)
            if time.monotonic()<self.next_sample:return
            self.next_sample=time.monotonic()+.1
            if self.model is None:raise ValueError('MODEL_UNAVAILABLE')
            q,times,valid_until=self.samples.snapshot(now)
            if self.attachments is None or not 0<=now-self.attachment_stamp<=500_000_000:raise ValueError('ATTACHMENT_STATE_UNAVAILABLE')
            errors={name:self.error for name in self.model.required}
            msg=self.new_message(now)
            msg.header.stamp=stamp(min(times));msg.valid_until=stamp(min(valid_until,self.attachment_stamp+500_000_000))
            msg.clock_epoch=self.samples.epoch;msg.model_revision=self.model.revision
            msg.attachment_revision=self.attachment_revision;msg.attachment_ids=self.attachment_ids
            msg.joints.name=list(self.model.required);msg.joints.position=[q[n] for n in msg.joints.name]
            msg.joints.header=msg.header;msg.joint_source_stamps=[stamp(t) for t in times]
            msg.joint_position_error_bounds=[errors[n] for n in msg.joints.name]
            self.work_context=(self.input_generation,self.model,self.attachment_revision,self.samples.epoch)
            self.work=self.worker.submit(self.calculate,self.model,q,errors,tuple(self.attachments),self.padding,msg)
        except Exception as error:
            msg=self.new_message(now);msg.reason=str(error);self.pub.publish(msg)

    def destroy_node(self):
        self.worker.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init();node=GeometryNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
