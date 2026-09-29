"""Isolated observer wire replay with explicit captured Python implementation."""
import json
import hashlib
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import uuid

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, PoseStamped, Point32
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from sensor_msgs.msg import LaserScan, CameraInfo, PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from rcl_interfaces.srv import GetParameters
from astribot_navigation_msgs.msg import RobotEnvelope, NavigationEnvelopeV2, EnvelopeApplyStatus, SensorHealthArray, NavigationExecutionStatus

HERE=Path(__file__).resolve().parent
REF=HERE/'reference/policy_observer'
BINDING=Path(os.environ.get('POLICY_OBSERVER_GEOMETRY_BINDING','/tmp/codex_geometry_validation_20260921/baseline_build/_geometry_native.cpython-310-x86_64-linux-gnu.so'))
RUN_REFERENCE="""import importlib.util,sys,types
from pathlib import Path
reference=Path(sys.argv.pop(1));binding=Path(sys.argv.pop(1))
package=types.ModuleType('astribot_s1_robot_geometry');package.__path__=[str(reference.parent/'policy_risk/astribot_s1_robot_geometry')];sys.modules[package.__name__]=package
spec=importlib.util.spec_from_file_location('astribot_s1_robot_geometry._geometry_native',binding);native=importlib.util.module_from_spec(spec);sys.modules[spec.name]=native;spec.loader.exec_module(native)
sys.path.insert(0,str(reference.parent))
from policy_observer import observer_node
assert Path(observer_node.__file__).resolve()==reference/'observer_node.py'
print('ORACLE_FILE='+observer_node.__file__,flush=True)
observer_node.main()
"""

def stamp(ns):return Time(sec=ns//10**9,nanosec=ns%10**9)

def transform(parent,child,x=0.,y=0.):
    t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
    t.transform.rotation.w=1.;t.transform.translation.x=float(x);t.transform.translation.y=float(y)
    return t

class Runtime:
    def __init__(self,impl,directory,params=None):
        self.name='policy_'+uuid.uuid4().hex[:10];self.ns=1_000_000_000
        self.domain=int(os.environ.get('POLICY_OBSERVER_DOMAIN','164'))
        self.context=Context();os.environ['ROS_LOCALHOST_ONLY']='1';rclpy.init(context=self.context,domain_id=self.domain)
        self.node=Node(self.name+'_probe',context=self.context);self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.outputs=[];self.messages=[];self.health=[];self.acks=[]
        topics=['clock','tf','tf_static','scan_from_cloud','odom','map','plan','navigation_policy/vision_observations','navigation_policy/observation','navigation/sensor_health','navigation/execution_status','navigation/robot_envelope','navigation/envelope_v2','navigation/envelope_applied','camera_points','camera_info','camera_vision']
        self.topic=lambda n:'/'+self.name+'/'+n
        self.clock=self.node.create_publisher(Clock,self.topic('clock'),10)
        self.tf=self.node.create_publisher(TFMessage,self.topic('tf_static'),QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scan_pub=self.node.create_publisher(LaserScan,self.topic('scan_from_cloud'),qos_profile_sensor_data)
        self.odom_pub=self.node.create_publisher(Odometry,self.topic('odom'),qos_profile_sensor_data)
        self.map_pub=self.node.create_publisher(OccupancyGrid,self.topic('map'),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.plan_pub=self.node.create_publisher(RosPath,self.topic('plan'),10)
        self.vision_pub=self.node.create_publisher(String,self.topic('navigation_policy/vision_observations'),10)
        self.vision_source_pub=self.node.create_publisher(String,self.topic('camera_vision'),qos_profile_sensor_data)
        self.envelope_pub=self.node.create_publisher(RobotEnvelope,self.topic('navigation/robot_envelope'),1)
        self.v2_pub=self.node.create_publisher(NavigationEnvelopeV2,self.topic('navigation/envelope_v2'),10)
        self.info_pub=self.node.create_publisher(CameraInfo,self.topic('camera_info'),qos_profile_sensor_data)
        self.cloud_pub=self.node.create_publisher(PointCloud2,self.topic('camera_points'),qos_profile_sensor_data)
        self.task_pub=self.node.create_publisher(NavigationExecutionStatus,self.topic('navigation/execution_status'),QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(String,self.topic('navigation_policy/observation'),lambda m:self.outputs.append(json.loads(m.data)),100)
        self.node.create_subscription(SensorHealthArray,self.topic('navigation/sensor_health'),self.health.append,100)
        self.node.create_subscription(EnvelopeApplyStatus,self.topic('navigation/envelope_applied'),self.acks.append,100)
        self.ready=self.node.create_client(GetParameters,'/'+self.name+'/get_parameters')
        binary=Path(os.environ.get('POLICY_OBSERVER_CPP','/tmp/codex_policy_integration_20260921/build/policy_observer_cpp'))
        command=[str(binary)] if impl=='cpp' else [sys.executable,'-c',RUN_REFERENCE,str(REF),str(BINDING)]
        if os.environ.get('POLICY_OBSERVER_CPU'):command=['taskset','-c',os.environ['POLICY_OBSERVER_CPU'],*command]
        if impl=='cpp':assert binary.is_file(),'native observer executable required'
        else:assert BINDING.is_file(),'explicit historical geometry binding required'
        command+=['--ros-args','-r','__node:='+self.name,'-p','use_sim_time:=true','-p','profile:='+str(REF/'simulation.json')]
        for topic in topics:command+=['-r','/'+topic+':='+self.topic(topic)]
        for key,value in (params or {}).items():command+=['-p',key+':='+(json.dumps(value) if isinstance(value,str) else str(value))]
        env=dict(os.environ,ROS_DOMAIN_ID=str(self.domain),ROS_LOCALHOST_ONLY='1')
        env.pop('ASTRIBOT_NAV_NATIVE_KERNELS',None);env.pop('ASTRIBOT_FUSION_NATIVE_SNAPSHOT',None)
        self.log_path=directory/(self.name+'.log');self.log=self.log_path.open('w')
        self.proc=subprocess.Popen(command,env=env,stdout=self.log,stderr=subprocess.STDOUT)
        try:
            self.until(lambda:self.ready.service_is_ready() and self.clock.get_subscription_count()>0)
            child=Path('/proc/'+str(self.proc.pid)+'/environ').read_bytes().split(b'\0')
            assert ('ROS_DOMAIN_ID='+str(self.domain)).encode() in child and b'ROS_LOCALHOST_ONLY=1' in child
            self.until(lambda:self.scan_pub.get_subscription_count()>0 and self.map_pub.get_subscription_count()>0)
            # Service discovery alone can precede the end of the constructor.
            # Require an executor response, then anchor the simulated clock
            # before advancing one complete timer period for the first sample.
            configured=self.ready.call_async(GetParameters.Request(names=['use_sim_time']))
            self.until(configured.done)
            assert configured.result().values[0].bool_value
            self.clock.publish(Clock(clock=stamp(self.ns)));self.drain(.05)
            self.tick()
        except BaseException:self.close();raise

    def until(self,predicate,timeout=6.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            assert self.proc.poll() is None,self.log_path.read_text()
            if predicate():return
            self.executor.spin_once(timeout_sec=.01)
        raise AssertionError('observer deadline: '+self.log_path.read_text())

    def drain(self,seconds=.08):
        end=time.monotonic()+seconds
        while time.monotonic()<end:self.executor.spin_once(timeout_sec=.005)

    def tick(self,advance=100_000_000):
        previous=len(self.outputs);self.ns+=advance
        if advance<0:
            self.clock.publish(Clock(clock=stamp(self.ns)));self.drain(.05)
            self.ns+=100_000_000
        # A repeated stamp may be needed until clock delivery is established.
        def output():
            self.clock.publish(Clock(clock=stamp(self.ns)))
            return len(self.outputs)>previous and self.outputs[-1]['stamp_ns']==self.ns
        self.until(output);return self.outputs[-1]

    def seed(self,scan_distance=math.inf,scan_frame='scan'):
        self.tf.publish(TFMessage(transforms=[transform('map','odom'),transform('odom','astribot_torso_base'),transform('astribot_torso_base','scan')]))
        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=40;grid.info.height=40;grid.info.resolution=.1;grid.info.origin.position.x=-2.;grid.info.origin.position.y=-2.;grid.info.origin.orientation.w=1.;grid.data=[0]*1600;self.map_pub.publish(grid)
        self.drain(.15);self.sensors(scan_distance,scan_frame)

    def sensors(self,scan_distance=math.inf,scan_frame='scan',capture=None):
        ns=self.ns if capture is None else capture
        odom=Odometry();odom.header.frame_id='odom';odom.header.stamp=stamp(ns);odom.pose.pose.orientation.w=1.;self.odom_pub.publish(odom)
        self.send_scan(scan_distance,scan_frame,ns)
        self.drain()

    def send_scan(self,distance=math.inf,frame='scan',capture=None):
        scan=LaserScan();scan.header.frame_id=frame;scan.header.stamp=stamp(self.ns if capture is None else capture);scan.range_min=.05;scan.range_max=5.;scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360;scan.ranges=[distance]*360;self.scan_pub.publish(scan)

    def close(self):
        if hasattr(self,'log_path'):self.log_path.with_suffix('.outputs.json').write_text(json.dumps(self.outputs,indent=2))
        if hasattr(self,'proc') and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:self.proc.wait(timeout=4.)
            except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait();raise AssertionError('owned observer failed to exit')
        if hasattr(self,'log'):self.log.close()
        self.executor.shutdown();self.node.destroy_node();self.context.shutdown()

@pytest.fixture
def runtime_config(request):return getattr(request,'param',{})

@pytest.fixture(params=['python','cpp'])
def runtime(request,tmp_path,runtime_config):
    instance=Runtime(request.param,tmp_path,runtime_config)
    try:yield instance
    finally:instance.close()

def test_startup_requires_sources_but_has_no_motion_authority(runtime):
    value=runtime.outputs[-1]
    assert value['observation_only'] and not value['inputs_valid'] and value['risk'] is None
    assert value['tracks']==0 and value['envelope_reason']=='NO_ENVELOPE'
    publishers=[]
    def graph_ready():
        try:
            current=runtime.node.get_publisher_names_and_types_by_node(runtime.name,'/')
        except rclpy.node.NodeNameNonExistentError:
            return False
        required={runtime.topic('navigation_policy/observation'),runtime.topic('navigation/sensor_health')}
        if not required<={name for name,_ in current}:return False
        publishers[:]=current;return True
    runtime.until(graph_ready)
    allowed={runtime.topic('navigation_policy/observation'),runtime.topic('navigation/sensor_health'),'/parameter_events','/rosout'}
    assert {name for name,_ in publishers}<=allowed

def test_fresh_empty_scan_stale_and_recovery(runtime):
    runtime.seed();value=runtime.tick();assert value['scan_count']==1 and value['inputs_valid'],value
    assert value['risk']['blocked'] is False and value['risk']['immediate'] is False
    value=runtime.tick(400_000_000);assert not value['inputs_valid']
    runtime.sensors();assert runtime.tick()['inputs_valid']

def test_scan_obstacle_and_clearing_evidence(runtime):
    runtime.seed(.3);first=runtime.tick();assert first['tracks']>0 and first['risk']['blocked']
    runtime.tick(400_000_000);runtime.sensors();cleared=runtime.tick()
    assert cleared['tracks']==0 and not cleared['risk']['blocked']

def test_latest_transformable_scan_and_delayed_tf(runtime):
    runtime.seed(scan_frame='late_scan');assert runtime.tick()['scan_count']==0
    runtime.tf.publish(TFMessage(transforms=[transform('astribot_torso_base','late_scan')]))
    runtime.drain();assert runtime.tick()['scan_count']==1

def test_clock_rollback_revokes_sources_and_risk(runtime):
    runtime.seed(.6);assert runtime.tick()['inputs_valid']
    value=runtime.tick(-800_000_000)
    assert value['epoch']==1 and value['risk'] is None and not value['inputs_valid']
    assert value['scan_age_s'] is None and value['odom_age_s'] is None

def test_plan_waits_for_frame_and_counts_success_once(runtime):
    msg=RosPath();msg.header.frame_id='route';pose=PoseStamped();pose.pose.position.x=1.;msg.poses=[pose]
    runtime.plan_pub.publish(msg);runtime.drain();first=runtime.tick();assert first['path_revision']==0 and first['errors']>=1
    runtime.tf.publish(TFMessage(transforms=[transform('map','odom'),transform('odom','route')]))
    runtime.drain();value=runtime.tick();assert value['path_revision']==1
    assert runtime.tick()['path_revision']==1

def test_image_observation_remains_uncertain_and_duplicate_rejects(runtime):
    runtime.seed();runtime.tick();runtime.sensors()
    packet=dict(schema_version=1,sensor_id='camera',stamp_ns=runtime.ns,frame_id='optical',calibration_epoch=0,observations=[dict(kind='image_box',measurement_id='image1',image_size_px=[100,100],box_xyxy_px=[1.,2.,10.,20.],geometry_quality=1.,provenance=['camera:image1'])])
    runtime.vision_pub.publish(String(data=json.dumps(packet)));runtime.drain();first=runtime.tick()
    assert first['vision_count']==1 and first['unassociated']==1 and first['risk']['uncertain']
    runtime.vision_pub.publish(String(data=json.dumps(packet)));runtime.drain();second=runtime.tick()
    assert second['vision_count']==1 and second['errors']>first['errors']

def test_future_scan_waits_for_capture_time(runtime):
    runtime.seed()
    runtime.tick();initial=runtime.outputs[-1]['scan_count']
    runtime.sensors(capture=runtime.ns+200_000_000)
    assert runtime.tick()['scan_count']==initial
    value=runtime.tick();assert value['scan_count']==initial+1 and value['inputs_valid']

def test_scan_mailbox_uses_last_ready_arrival_not_max_timestamp(runtime):
    runtime.seed();runtime.tick();initial=runtime.outputs[-1]['scan_count']
    runtime.send_scan(capture=runtime.ns)
    runtime.send_scan(capture=runtime.ns-50_000_000)
    runtime.drain();value=runtime.tick()
    assert value['scan_count']==initial+1 and value['scan_age_s']==pytest.approx(.15)

def test_scan_mailbox_discards_older_than_five_pending(runtime):
    runtime.seed();runtime.tick();initial=runtime.outputs[-1]['scan_count']
    # Best-effort depth five also bounds the input; drain each publish so the
    # behavior under test is the observer mailbox, not DDS queue loss.
    runtime.send_scan();runtime.drain(.02)
    for _ in range(5):runtime.send_scan(frame='missing');runtime.drain(.02)
    assert runtime.tick()['scan_count']==initial

@pytest.mark.parametrize('age',[299_999_999,300_000_000,300_000_001])
def test_scan_age_exact_boundary(runtime,age):
    runtime.seed();value=runtime.tick(age)
    accepted=age*1e-9<=.3
    assert value['scan_count']==int(accepted)

@pytest.mark.parametrize('distance',[math.nan,-math.inf,0.])
def test_invalid_scan_keeps_previous_source_stamp(runtime,distance):
    runtime.seed();runtime.tick();previous=runtime.outputs[-1]
    runtime.send_scan(distance);runtime.drain();value=runtime.tick()
    assert value['scan_count']==previous['scan_count'] and value['scan_age_s']>previous['scan_age_s']

def test_empty_plan_replaces_path_without_requiring_poses(runtime):
    runtime.seed();runtime.tick();msg=RosPath();msg.header.frame_id='odom';runtime.plan_pub.publish(msg);runtime.drain()
    assert runtime.tick()['path_revision']==1

def test_resolve_error_retains_prior_partial_transaction(runtime):
    runtime.seed();runtime.tick()
    packet=dict(schema_version=1,sensor_id='camera',stamp_ns=runtime.ns,frame_id='optical',calibration_epoch=0,observations=[dict(kind='image_box',measurement_id='image1',image_size_px=[100,100],box_xyxy_px=[1.,2.,10.,20.],geometry_quality=1.,provenance=['camera:image1'])])
    runtime.vision_pub.publish(String(data=json.dumps(packet)));runtime.drain();first=runtime.tick();assert first['unassociated']==1
    packet.update(stamp_ns=runtime.ns,observations=[],resolved_measurement_ids=['image1',[]]);packet.pop('frame_id')
    runtime.vision_pub.publish(String(data=json.dumps(packet)));runtime.drain();value=runtime.tick()
    assert value['unassociated']==0 and value['vision_count']==1 and value['errors']==first['errors']+1

@pytest.mark.parametrize('malformed',[dict(schema_version=2),dict(stamp_ns=-1),dict(calibration_epoch=-1)])
def test_bad_vision_does_not_synthesize_clear_evidence(runtime,malformed):
    runtime.seed(.3);initial=runtime.tick()
    packet=dict(schema_version=1,sensor_id='camera',stamp_ns=runtime.ns,frame_id='odom',calibration_epoch=0,observations=[]);packet.update(malformed)
    runtime.vision_pub.publish(String(data=json.dumps(packet)));runtime.drain();value=runtime.tick()
    assert value['errors']==initial['errors']+1 and value['vision_count']==0 and value['tracks']==initial['tracks']

def legacy_envelope(runtime,epoch=1):
    profile=json.loads((REF/'simulation.json').read_text())
    result=RobotEnvelope(stamp=stamp(runtime.ns),epoch=epoch,lease_s=.5,frame_id='astribot_torso_base',posture_id='transport',transport_ready=True,reason='READY')
    for key in ('half_length_m','half_width_m','height_m','payload_mass_kg','max_speed_m_s','max_angular_speed_rad_s','max_acceleration_m_s2','brake_deceleration_m_s2'):setattr(result,key,float(profile[key]))
    return result

def fixed_envelope(runtime,epoch=1):
    m=NavigationEnvelopeV2();m.header.stamp=stamp(runtime.ns);m.header.frame_id='astribot_torso_base';m.valid_until=stamp(runtime.ns+500_000_000)
    m.coordinator_session_id='session';m.request_id='request';m.hold_id='hold';m.epoch=epoch;m.reference_state_sequence=5;m.source_state_sequence=6
    m.model_revision='model';m.attachment_revision='attachment';m.mode=m.FIXED_POSTURE;m.limits=legacy_envelope(runtime,epoch)
    m.reserved_footprint.points=[Point32(x=x,y=y,z=0.) for x,y in [(-.3,-.3),(.3,-.3),(.3,.3),(-.3,.3)]]
    xy=[[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]
    m.installed_footprint.points=[Point32(x=x,y=y,z=0.) for x,y in xy];m.clearance_m=.08
    m.installed_geometry_hash=hashlib.sha256(json.dumps(dict(frame=m.header.frame_id,clearance=m.clearance_m,vertices=xy),sort_keys=True).encode()).hexdigest()
    m.navigation_allowed=True;m.reason='READY';return m

def test_legacy_full_width_epoch_expiry_and_stale_retention(runtime):
    runtime.seed();runtime.envelope_pub.publish(legacy_envelope(runtime,2**64-1));runtime.drain()
    assert runtime.tick()['envelope_ready']
    old=legacy_envelope(runtime,1);old.reason='OLD';runtime.envelope_pub.publish(old);runtime.drain()
    assert runtime.tick()['envelope_reason']=='READY'
    value=runtime.tick(400_000_000);assert not value['envelope_ready'] and value['envelope_reason']=='READY'

@pytest.mark.parametrize('runtime_config',[{'navigation_geometry_mode':'fixed_v2'}],indirect=True)
def test_fixed_envelope_apply_matching_heartbeat_revoke_and_recover(runtime):
    runtime.seed();m=fixed_envelope(runtime,2**64-1);runtime.v2_pub.publish(m);runtime.drain()
    assert not runtime.acks  # First configuration must wait for processing.
    assert runtime.tick()['envelope_ready'];runtime.until(lambda:bool(runtime.acks))
    assert runtime.acks[-1].envelope_epoch==2**64-1 and runtime.acks[-1].installed_geometry_hash==m.installed_geometry_hash
    count=len(runtime.acks);m.header.stamp=stamp(runtime.ns);m.valid_until=stamp(runtime.ns+500_000_000);m.source_state_sequence+=1
    runtime.v2_pub.publish(m);runtime.until(lambda:len(runtime.acks)>count)
    assert len(runtime.outputs)>0 and runtime.outputs[-1]['stamp_ns']==runtime.ns
    assert runtime.tick()['envelope_ready']
    runtime.until(lambda:len(runtime.acks)>=3)  # Drain the previous apply ACK.
    count=len(runtime.acks);m.installed_geometry_hash='invalid';runtime.v2_pub.publish(m);runtime.drain()
    assert len(runtime.acks)==count
    value=runtime.tick();assert not value['envelope_ready'] and value['envelope_reason']=='NO_ENVELOPE'
    m=fixed_envelope(runtime,2**64-1);runtime.v2_pub.publish(m);runtime.drain();assert runtime.tick()['envelope_ready']
    assert not runtime.tick(500_000_000)['envelope_ready']

POINT_CLOUD_SOURCE=dict(adapter='pointcloud_boxes',topic='/camera_points',sensor_id='depth',required=True,camera_info_topic='/camera_info',depth_unit_m=.001,coverage_body_yaw_half_angle=[[0.,.5]],max_points='32_768',position_variance_m2='0.0025')

@pytest.mark.parametrize('runtime_config',[{'observation_sources':json.dumps([POINT_CLOUD_SOURCE])}],indirect=True)
def test_humble_camera_info_retains_original_coefficient_rejection(runtime):
    runtime.seed();assert not runtime.tick()['required_sensors_valid']
    cloud=PointCloud2();cloud.header.frame_id='odom';cloud.header.stamp=stamp(runtime.ns);cloud.width=2;cloud.height=1;cloud.point_step=12;cloud.row_step=24
    cloud.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))];cloud.data=struct.pack('<ffffff',.7,-.1,.2,.8,.1,.4)
    runtime.cloud_pub.publish(cloud);runtime.drain();first=runtime.tick();assert first['vision_count']==0 and first['errors']==1
    info=CameraInfo();info.header.frame_id='optical';info.width=2**32-1;info.height=10;info.k=[1.,0.,0.,0.,1.,0.,0.,0.,1.];info.distortion_model='plumb_bob'
    runtime.info_pub.publish(info);runtime.drain();runtime.sensors();cloud.header.stamp=stamp(runtime.ns);runtime.cloud_pub.publish(cloud);runtime.drain()
    # Humble's Python wire type exposes K as numpy.float64 values. The existing
    # strict coefficient contract rejects them; a migration must not silently
    # make this required source valid. The separate typed registry tests cover
    # accepted built-in float coefficients and epoch changes.
    value=runtime.tick();assert value['vision_count']==0 and not value['inputs_valid'] and value['errors']==2
    info.width=123;runtime.info_pub.publish(info);runtime.drain();runtime.sensors();cloud.header.stamp=stamp(runtime.ns);runtime.cloud_pub.publish(cloud);runtime.drain();assert runtime.tick()['vision_count']==0
    assert all(all(s.state=='UNAVAILABLE' and s.reason=='NO_CURRENT_DATA' for s in h.sensors if s.sensor_id=='depth') for h in runtime.health)

@pytest.mark.parametrize('runtime_config',[{'observation_sources':json.dumps([{k:v for k,v in POINT_CLOUD_SOURCE.items() if k!='camera_info_topic'}])}],indirect=True)
def test_pointcloud_without_camera_info_produces_metric_evidence(runtime):
    runtime.seed();assert not runtime.tick()['required_sensors_valid']
    cloud=PointCloud2();cloud.header.frame_id='odom';cloud.header.stamp=stamp(runtime.ns);cloud.width=2;cloud.height=1;cloud.point_step=12;cloud.row_step=24
    cloud.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))];cloud.data=struct.pack('<ffffff',.7,-.1,.2,.8,.1,.4)
    runtime.sensors();runtime.cloud_pub.publish(cloud);runtime.drain();value=runtime.tick()
    assert value['vision_count']==1 and value['inputs_valid'] and value['tracks']==1
    runtime.until(lambda:any(any(s.sensor_id=='depth' and s.calibration_epoch==0 and s.depth_available for s in h.sensors) for h in runtime.health))

@pytest.mark.parametrize('runtime_config',[{'observation_sources':json.dumps([dict(adapter='vision_json',topic='/camera_vision',coverage_body_yaw_half_angle=v)])} for v in (None,{'front':[0.,.5]},[[0.]])],indirect=True)
def test_malformed_coverage_rejects_packet_after_startup(runtime):
    pub=runtime.vision_source_pub
    runtime.until(lambda:pub.get_subscription_count()>0)
    packet=dict(schema_version=1,sensor_id='camera',stamp_ns=runtime.ns,calibration_epoch=0,observations=[])
    pub.publish(String(data=json.dumps(packet)));runtime.until(lambda:'observation rejected:' in runtime.log_path.read_text());value=runtime.tick()
    assert value['vision_count']==0 and value['errors']==1

@pytest.mark.parametrize('sensor',[None,False,123,[],{}])
def test_malformed_empty_sensor_rejected_without_fabricating_health(runtime,sensor):
    runtime.seed();runtime.tick()
    packet=dict(schema_version=1,sensor_id=sensor,stamp_ns=runtime.ns,calibration_epoch=0,observations=[])
    runtime.vision_pub.publish(String(data=json.dumps(packet)));runtime.drain();value=runtime.tick()
    assert value['vision_count']==0 and value['errors']==1
    assert all(all(s.sensor_id=='scan' for s in h.sensors) for h in runtime.health)

def test_health_expiry_outside_ros_time_does_not_publish_wrapped_seconds(runtime):
    previous=len(runtime.outputs)
    runtime.ns=(2**31-1)*10**9+900_000_000
    runtime.clock.publish(Clock(clock=stamp(runtime.ns)))
    deadline=time.monotonic()+5.
    while runtime.proc.poll() is None and time.monotonic()<deadline:runtime.executor.spin_once(timeout_sec=.01)
    assert runtime.proc.poll() not in (None,0),runtime.log_path.read_text()
    assert all(value['stamp_ns']<runtime.ns for value in runtime.outputs[previous:])

@pytest.mark.parametrize('epoch,width,xmax',[(2**63,100,10.),(2**64-1,100,10.),(0,2**64+1,10.),(0,10**399,10.),(0,2**64-1,2**64-1),(0,2**64,float(2**64))])
def test_large_json_epoch_and_image_bounds_reach_original_output(runtime,epoch,width,xmax):
    packet=dict(schema_version=1,sensor_id='camera',stamp_ns=runtime.ns,frame_id='optical',calibration_epoch=epoch,
        observations=[dict(kind='image_box',measurement_id='large',image_size_px=[width,100],box_xyxy_px=[0.,0.,xmax,10.],geometry_quality=1.,provenance=['camera:large'])])
    runtime.vision_pub.publish(String(data=json.dumps(packet)));runtime.drain();value=runtime.tick()
    assert value['vision_count']==1 and value['errors']==0 and value['unassociated']==1
    runtime.until(lambda:any(any(s.sensor_id=='camera' and s.calibration_epoch==epoch for s in h.sensors) for h in runtime.health))

def test_json_epoch_over_wire_range_fails_at_health_output(runtime):
    packet=dict(schema_version=1,sensor_id='camera',stamp_ns=runtime.ns,frame_id='optical',calibration_epoch=2**64,
        observations=[dict(kind='image_box',measurement_id='large',image_size_px=[100,100],box_xyxy_px=[0.,0.,10.,10.],geometry_quality=1.,provenance=['camera:large'])])
    runtime.vision_pub.publish(String(data=json.dumps(packet)));runtime.drain()
    assert runtime.proc.poll() is None and 'observation rejected:' not in runtime.log_path.read_text()
    previous=len(runtime.outputs);runtime.ns+=100_000_000;runtime.clock.publish(Clock(clock=stamp(runtime.ns)))
    deadline=time.monotonic()+5.
    while runtime.proc.poll() is None and time.monotonic()<deadline:
        # Humble's executor retrieves a completed callback exception on its
        # next ready callback. Keep the clock unchanged and provide that wakeup.
        runtime.clock.publish(Clock(clock=stamp(runtime.ns)))
        runtime.executor.spin_once(timeout_sec=.01)
    assert runtime.proc.poll() not in (None,0),runtime.log_path.read_text()
    assert all(value['stamp_ns']<runtime.ns for value in runtime.outputs[previous:])


def image_packet(runtime,sensor='camera',epoch=0):
    return dict(schema_version=1,sensor_id=sensor,stamp_ns=runtime.ns,frame_id='optical',calibration_epoch=epoch,
        observations=[dict(kind='image_box',measurement_id='image',image_size_px=[100,100],
            box_xyxy_px=[0.,0.,10.,10.],geometry_quality=1.,provenance=['camera:image'])])


@pytest.mark.parametrize('field,value',[
    ('ignored','unused\ud800'),('measurement_id','image\udfff'),
    ('provenance','origin\ud800'),('frame_id','optical\ud800'),
    ('measurement_id','image\u0000Sd800'),('sensor_id','camera\U00010000'),
])
def test_json_string_codepoints_preserved_until_their_consumer(runtime,field,value):
    packet=image_packet(runtime)
    if field in ('ignored','frame_id','sensor_id'):packet[field]=value
    elif field=='provenance':packet['observations'][0][field]=[value]
    else:packet['observations'][0][field]=value
    runtime.vision_pub.publish(String(data=json.dumps(packet)));runtime.drain();result=runtime.tick()
    assert result['vision_count']==1 and result['errors']==0 and result['unassociated']==1


@pytest.mark.parametrize('sensor,epoch',[
    ('camera\ud800',0),('camera\udfff',0),('camera\u0000\ud800',0),('camera\ud800',2**64),
])
def test_surrogate_sensor_fails_at_health_serialization_after_numeric_checks(runtime,sensor,epoch):
    runtime.vision_pub.publish(String(data=json.dumps(image_packet(runtime,sensor,epoch))));runtime.drain()
    assert runtime.proc.poll() is None and 'observation rejected:' not in runtime.log_path.read_text()
    previous=len(runtime.outputs);runtime.ns+=100_000_000;deadline=time.monotonic()+5.
    while runtime.proc.poll() is None and time.monotonic()<deadline:
        runtime.clock.publish(Clock(clock=stamp(runtime.ns)));runtime.executor.spin_once(timeout_sec=.01)
    assert runtime.proc.poll() not in (None,0),runtime.log_path.read_text()
    log=runtime.log_path.read_text()
    if epoch:
        assert 'calibration_epoch' in log or 'ROS unsigned integer out of range' in log
        assert 'UnicodeEncodeError' not in log
    else:assert 'UnicodeEncodeError' in log
    assert all(value['stamp_ns']<runtime.ns for value in runtime.outputs[previous:])


def test_health_wire_nul_matches_original_generated_message_converter(runtime):
    raw=[]
    runtime.node.create_subscription(SensorHealthArray,runtime.topic('navigation/sensor_health'),raw.append,10,raw=True)
    runtime.vision_pub.publish(String(data=json.dumps(image_packet(runtime,'a\u0000tail'))));runtime.drain()
    result=runtime.tick();assert result['vision_count']==1 and result['errors']==0
    # Plain CDR: encapsulation (4), stamp (8), sequence length (4), then first
    # sensor string length/data. Observe raw bytes: Python deserialization itself
    # truncates at NUL and would hide a native wire mismatch.
    runtime.until(lambda:any(len(v)>20 and v[20:21]==b'a' for v in raw))
    wire=next(v for v in reversed(raw) if len(v)>20 and v[20:21]==b'a')
    assert wire[:4]==b'\x00\x01\x00\x00'
    length=struct.unpack_from('<I',wire,16)[0]
    assert wire[20:20+length]==b'a\0'


@pytest.mark.parametrize('frame',['bad\ud800','bad\u0000frame'])
def test_tf_string_conversion_precedes_lookup_and_metric_validation(runtime,frame):
    packet=dict(schema_version=1,sensor_id='camera',stamp_ns=runtime.ns,frame_id=frame,calibration_epoch=0,
        observations=[dict(kind='metric_box',measurement_id='bad_tf',center_m=['also invalid'],size_m=[1.,1.,1.],
            position_variance_m2=.01,geometry_quality=1.,provenance=['bad_tf'])])
    runtime.vision_pub.publish(String(data=json.dumps(packet)))
    runtime.until(lambda:'observation rejected:' in runtime.log_path.read_text())
    log=runtime.log_path.read_text()
    if '\0' in frame:assert 'embedded null character' in log
    else:assert 'surrogates not allowed' in log or 'UnicodeEncodeError' in log
    result=runtime.tick();assert result['errors']==1 and result['vision_count']==0
