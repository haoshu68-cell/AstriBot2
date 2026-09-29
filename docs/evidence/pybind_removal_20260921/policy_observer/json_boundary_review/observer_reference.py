"""Frozen adapter/observer methods and real generated messages; no ROS init/node."""
import ast,dataclasses,importlib.util,json,math,os,sys,threading,time,types
from pathlib import Path
from rclpy.time import Time
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from astribot_navigation_msgs.msg import SensorHealth as HealthMessage,SensorHealthArray
REF=Path('/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation_policy_native/test/reference/policy_observer')
NAME='_bounded_json_review';pkg=types.ModuleType(NAME);pkg.__path__=[str(REF)];sys.modules[NAME]=pkg
os.environ.pop('ASTRIBOT_NAV_NATIVE_KERNELS',None);os.environ.pop('ASTRIBOT_FUSION_NATIVE_SNAPSHOT',None)
for n in ('contracts','ports','profile','execution_context','sensor_health','fusion','robot_envelope','observation_adapters'):
    spec=importlib.util.spec_from_file_location(NAME+'.'+n,REF/(n+'.py'));m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)
C=sys.modules[NAME+'.contracts'];F=sys.modules[NAME+'.fusion'];F._snapshot_tracks_native=None
E=sys.modules[NAME+'.execution_context'];H=sys.modules[NAME+'.sensor_health'];P=sys.modules[NAME+'.profile'];R=sys.modules[NAME+'.robot_envelope'];A=sys.modules[NAME+'.observation_adapters']
cls=next(n for n in ast.parse((REF/'observer_node.py').read_text()).body if isinstance(n,ast.ClassDef) and n.name=='PolicyObserver')
cls=ast.ClassDef(name='Observer',bases=[],keywords=[],body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('accept_observations','tick')],decorator_list=[])
ns=dict(json=json,math=math,time=time,asdict=dataclasses.asdict,replace=dataclasses.replace,Time=Time,String=String,
        HealthMessage=HealthMessage,SensorHealthArray=SensorHealthArray,has_predictions=lambda t:False)
for n in ('Stamp','BearingCone','Vec3','MetricBox'):ns[n]=getattr(C,n)
exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(REF/'observer_node.py'),'exec'),ns)
class Publisher:
    def __init__(self):self.items=[];self.serialized=[]
    def publish(self,m):
        self.serialized.append(len(serialize_message(m)));self.items.append(m)
node=ns['Observer']();node.profile=R.EnvelopeProfile(P.Profile.load(REF/'simulation.json'))
node.fusion=F.ConservativeFusion(node.profile);node.execution=E.ExecutionContext();node.health_registry=H.SensorHealthRegistry(node.profile.sensor_timeout_s);node.calibrations=H.CameraCalibrationRegistry()
node.stamp=lambda:C.Stamp(1_000_000_000,'ros',0);node.get_clock=lambda:types.SimpleNamespace(now=lambda:Time(nanoseconds=1_000_000_000))
node.tf=types.SimpleNamespace(lookup_transform=lambda *a:(_ for _ in ()).throw(LookupError('no TF needed')))
node.process_envelope=lambda:None;node.process_plan=lambda:None;node.process_scans=lambda _:None
node.robot=None;node.odom_at=None;node.scan_at=None;node.map=None;node.path=();node.odom_lock=threading.Lock();node.observation_only=True
node.base_frame='base';node.errors=0;node.scan_count=0;node.vision_count=0;node.path_revision=0;node.last_error=''
node.health_pub=Publisher();node.publisher=Publisher();logs=[];node.get_logger=lambda:types.SimpleNamespace(warning=lambda m:logs.append(str(m)))
case=sys.argv[1];epoch=0;width=640;sensor='cam';measurement='m';extra={}
if case=='epoch_int64_plus':epoch=2**63
elif case=='epoch_uint64_max':epoch=2**64-1
elif case=='epoch_uint64_plus':epoch=2**64
elif case=='image_uint64_plus':width=2**64
elif case=='image_400_digits':width=10**399
elif case=='ignored_surrogate':extra['ignored']='\ud800'
elif case=='measurement_surrogate':measurement='m\ud800'
elif case=='sensor_surrogate':sensor='cam\ud800'
packet=dict(schema_version=1,sensor_id=sensor,stamp_ns=1_000_000_000,frame_id='camera',calibration_epoch=epoch,
 observations=[dict(kind='image_box',measurement_id=measurement,image_size_px=[width,480],box_xyxy_px=[0.,0.,1.,1.],geometry_quality=1.,provenance=['origin'])],**extra)
adapter=A.VisionAdapter(node.profile,node.tf,node.stamp,{'coverage_body_yaw_half_angle':[[0.,math.pi]]})
node.accept_observations(adapter,String(data=json.dumps(packet)))
result=dict(case=case,errors_after_accept=node.errors,vision_count=node.vision_count,unassociated=len(node.fusion.unassociated),health_records=len(node.health_registry.records),logs=logs)
try:node.tick();result['tick']='completed'
except Exception as error:result['tick']=type(error).__name__+': '+str(error)
result['health_published']=len(node.health_pub.items);result['observation_published']=len(node.publisher.items)
if node.publisher.items:result['output_unassociated']=json.loads(node.publisher.items[-1].data)['unassociated']
print(json.dumps(result,ensure_ascii=True),flush=True)
