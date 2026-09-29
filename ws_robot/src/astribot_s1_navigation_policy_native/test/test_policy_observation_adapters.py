"""Differential built-in adapter replay against Git 965bf057 and installed Humble read_points."""
import dataclasses
import copy
import enum
import importlib.util
import hashlib
import json
import math
import os
from pathlib import Path
import random
import struct
import subprocess
import sys
import types
import pytest

HERE=Path(__file__).resolve().parent
REF=HERE/'reference/policy_observation_adapters'
PROBE=Path(os.environ.get('POLICY_OBSERVATION_ADAPTERS_PROBE','/tmp/codex_policy_observation_adapters_20260921/policy_observation_adapters_probe'))
PKG='_frozen_policy_observation_adapters'

def load(name,file):
    spec=importlib.util.spec_from_file_location(name,file);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m

class Message:
    def __init__(self,**kw): self.__dict__.update(kw)
class Time:
    def __init__(self,*,nanoseconds): self.nanoseconds=nanoseconds
class PointField(Message):
    INT8=1;UINT8=2;INT16=3;UINT16=4;INT32=5;UINT32=6;FLOAT32=7;FLOAT64=8
class PointCloud2(Message): pass
class String(Message): pass

@pytest.fixture(scope='module')
def authority():
    names=['rclpy','rclpy.time','std_msgs','std_msgs.msg','sensor_msgs','sensor_msgs.msg','sensor_msgs_py','sensor_msgs_py.point_cloud2',PKG,PKG+'.contracts',PKG+'.observation_adapters']
    previous={name:sys.modules.get(name) for name in names}
    for name in ['rclpy','std_msgs','sensor_msgs','sensor_msgs_py',PKG]:
        m=types.ModuleType(name);m.__path__=[str(REF)];sys.modules[name]=m
    for name,attrs in [('rclpy.time',dict(Time=Time)),('std_msgs.msg',dict(String=String,Header=Message)),('sensor_msgs.msg',dict(PointCloud2=PointCloud2,PointField=PointField))]:
        m=types.ModuleType(name);m.__dict__.update(attrs);sys.modules[name]=m
    C=load(PKG+'.contracts',REF/'contracts.py');load('sensor_msgs_py.point_cloud2',REF/'point_cloud2.py');A=load(PKG+'.observation_adapters',REF/'observation_adapters.py')
    yield C,A
    for name,old in previous.items():
        if old is None:sys.modules.pop(name,None)
        else:sys.modules[name]=old


def canonical(value):
    if dataclasses.is_dataclass(value):return {f.name:canonical(getattr(value,f.name)) for f in dataclasses.fields(value)}
    if isinstance(value,enum.Enum):return value.value
    if isinstance(value,(tuple,list)):return [canonical(v) for v in value]
    if isinstance(value,dict):return {k:canonical(v) for k,v in value.items()}
    if isinstance(value,float) and not math.isfinite(value):return {'nonfinite':'nan' if math.isnan(value) else ('inf' if value>0 else '-inf')}
    return value


def stamp(ns=100,clock='ros',epoch=4):return dict(ns=ns,clock=clock,epoch=epoch)
def vision(**updates):
    data=dict(schema_version=1,sensor_id='camera',stamp_ns=100,frame_id='sensor',calibration_epoch=7,observations=[dict(kind='metric_box',measurement_id='m',track_id='track',center_m=[1.,2.,3.],size_m=[.2,.4,.6],position_variance_m2=.01,geometry_quality=.8,classes={'person':.6,'unknown':.3},provenance=['frame:100'])])
    data.update(updates);return data

def cloud(points,fmt='fff',*,bigendian=False,dense=False,width=None,height=1,padding=0,row_padding=0,fields=None):
    types_by_fmt={'b':1,'B':2,'h':3,'H':4,'i':5,'I':6,'f':7,'d':8}
    sizes=[struct.calcsize(c) for c in fmt];offsets=[0,sizes[0],sizes[0]+sizes[1]];step=sum(sizes)+padding
    data=b''.join(struct.pack(('>' if bigendian else '<')+fmt,*p)+bytes(padding) for p in points)
    width=len(points) if width is None else width
    return dict(width=width,height=height,point_step=step,row_step=width*step+row_padding,is_bigendian=bigendian,is_dense=dense,data=list(data),
                fields=fields or [dict(name=n,offset=offset,datatype=types_by_fmt[f],count=1) for n,offset,f in zip('xyz',offsets,fmt)],
                sec=0,nanosec=100,frame_id='sensor')


def reference(case,authority):
    C,A=authority;calls=[];now_index=0;steady_index=0
    now_values=case.get('now',[stamp(100),stamp(200,'other',9)]);steady=case.get('steady',[1000,1001,1002,1003,1004])
    def now():
        nonlocal now_index
        calls.append(['now',now_index]);v=now_values[min(now_index,len(now_values)-1)];now_index+=1
        if v.get('error'):raise RuntimeError(v['error'])
        return C.Stamp(**v)
    def steady_now():
        nonlocal steady_index
        calls.append(['steady',steady_index]);v=steady[min(steady_index,len(steady)-1)];steady_index+=1;return v
    class TF:
        def lookup_transform(self,target,source,time):
            calls.append(['tf',target,source,time.nanoseconds]);t=case.get('transform',dict(translation=[0.,0.,0.],rotation=[0.,0.,0.,1.]))
            if t.get('error'):raise RuntimeError(t['error'])
            return Message(transform=Message(translation=Message(**dict(zip('xyz',t['translation']))),rotation=Message(**dict(zip('xyzw',t['rotation'])))))
    old=A.time.monotonic_ns;A.time.monotonic_ns=steady_now
    options=copy.deepcopy(case.get('raw_options',case.get('options',{})));profile=types.SimpleNamespace(sensor_timeout_s=case.get('timeout',.5),tracking_frame=case.get('tracking_frame','odom'))
    adapter=A.make_adapter(case.get('adapter','vision_json'),profile,TF(),now,options)
    results=[];packets={}
    try:
        for op in case['operations']:
            packet=None
            try:
                if op.get('action')=='integer_dump':
                    value=json.dumps(int(op['hex'],16));packet=None
                elif op.get('action')=='json':
                    value=json.loads(op['data']);packet=None
                elif op.get('action')=='metadata':
                    d=adapter.last_packet;value=dict(stamp_ns=int(d['stamp_ns']),calibration_epoch=int(d['calibration_epoch']),sensor_id=d['sensor_id'],frame_id=d.get('frame_id','') if isinstance(d.get('frame_id',''),str) else '',resolved_measurement_ids=d.get('resolved_measurement_ids',[]));packet=None
                elif op.get('action')=='calibration':
                    adapter.options['calibration_epoch']=op['value'];value=None;packet=None
                else:
                    if case.get('adapter','vision_json')=='vision_json':packet=String(data=op.get('data',json.dumps(op.get('packet',vision()))))
                    else:
                        key=op.get('packet_id');packet=packets.get(key) if op.get('reuse') else None
                        if packet is None:
                            p=op['packet'];packet=PointCloud2(**{k:v for k,v in p.items() if k not in ('sec','nanosec','frame_id','fields','data')},
                                header=Message(stamp=Message(sec=p['sec'],nanosec=p['nanosec']),frame_id=p['frame_id']),fields=[PointField(**f) for f in p['fields']],data=bytearray(p['data']))
                            if key:packets[key]=packet
                    value=adapter.normalize(packet)
                result=dict(ok=True,result=canonical(value))
            except C.ContractError as e:result=dict(ok=False,type='ContractError',code=e.code.value,field=e.field)
            except Exception as e:result=dict(ok=False,type=type(e).__name__)
            result['last_packet']=canonical(adapter.last_packet);result['calls']=calls.copy()
            if isinstance(packet,PointCloud2):result['data']=list(packet.data)
            results.append(result)
        return results
    finally:A.time.monotonic_ns=old


def native(case,env=None):
    assert PROBE.is_file(),'native observation adapter executable has not been implemented'
    transport=copy.deepcopy(case)
    if transport.get('adapter','vision_json')=='vision_json':
        for op in transport['operations']:
            if 'packet' in op and 'data' not in op:op['data']=json.dumps(op['packet'])
    p=subprocess.run([str(PROBE)],input=json.dumps(transport,allow_nan=False),text=True,capture_output=True,check=True,env=env)
    return json.loads(p.stdout)


def compare(a,b,path=''):
    if isinstance(a,float):
        assert b==pytest.approx(a,rel=2e-12,abs=2e-12),(path,a,b)
        if a==0. and b==0.:assert math.copysign(1.,a)==math.copysign(1.,b),(path,a,b)
    elif isinstance(a,dict):
        assert set(a)==set(b),(path,a,b)
        for k in a:compare(a[k],b[k],path+'.'+k)
    elif isinstance(a,list):
        assert len(a)==len(b),(path,a,b)
        for i,(x,y) in enumerate(zip(a,b)):compare(x,y,path+f'[{i}]')
    else:assert a==b,(path,a,b)


def check(case,authority):compare(reference(case,authority),native(case))


def test_clock_tf_rotation_velocity_and_metadata(authority):
    p=vision();p['observations'][0].update(velocity_m_s=[.1,-.2,.3],velocity_variance_m2_s2=.04)
    p['resolved_measurement_ids']=['old'];q=[.18257418583505536,.3651483716701107,.5477225575051661,.7302967433402214]
    check(dict(options={'sensor_id':'camera'},transform=dict(translation=[.5,-.3,.7],rotation=q),operations=[dict(packet=p)]),authority)


def test_image_bearing_empty_and_rejection_last_packet(authority):
    items=[dict(kind='image_box',measurement_id='i',image_size_px=[640,480],box_xyxy_px=[1.,2.,40.,50.],geometry_quality=.5,provenance=['i']),
           dict(kind='bearing_cone',measurement_id='b',direction=[1.,0.,0.],half_angle_rad=.2,geometry_quality=1.,provenance=['b'])]
    check(dict(options={'sensor_id':'camera'},operations=[dict(packet=vision(observations=items)),dict(packet=vision(schema_version=2)),dict(packet=vision(sensor_id='wrong')),dict(data='{bad'),dict(packet=vision(observations=[]))]),authority)


def test_cloud_types_stride_and_calibration_update(authority):
    for fmt in ('bbb','BBB','hhh','HHH','iii','III','fff','ddd','fdi'):
        p=cloud([(1,2,3),(4,6,9)],fmt,padding=5)
        check(dict(adapter='pointcloud_boxes',options={'sensor_id':'lidar'},operations=[dict(packet=p),dict(action='calibration',value=9),dict(packet=p)]),authority)


def test_cloud_bigendian_inplace_repeat_and_row_step(authority):
    p=cloud([(1.,2.,3.),(4.,5.,6.),(7.,8.,9.),(10.,11.,12.)],bigendian=True,width=2,height=2,row_padding=19)
    check(dict(adapter='pointcloud_boxes',options={'sensor_id':'lidar'},operations=[dict(packet=p,packet_id='p'),dict(packet_id='p',reuse=True)]),authority)


def test_cloud_dense_nan_infinity_and_no_free_space(authority):
    for dense in (False,True):
        for points in ([(math.nan,0.,1.)],[(math.nan,0.,1.),(1.,2.,3.)],[(1.,2.,3.),(math.nan,0.,1.)],[(math.inf,0.,1.)],[]):
            check(dict(adapter='pointcloud_boxes',options={'sensor_id':'lidar'},operations=[dict(packet=cloud(points,dense=dense))]),authority)


def test_vision_budgets_unicode_and_failure_call_order(authority):
    p=vision(sensor_id='摄像头');wire=json.dumps(p,ensure_ascii=False)
    for limit in (len(wire)-1,len(wire)):
        check(dict(options={'max_packet_bytes':limit},operations=[dict(data=wire)]),authority)
    check(dict(options={'max_observations':0},operations=[dict(packet=vision())]),authority)
    check(dict(now=[stamp(),dict(error='second clock failure')],operations=[dict(packet=vision())]),authority)
    check(dict(transform=dict(error='transform unavailable'),operations=[dict(packet=vision())]),authority)


@pytest.mark.parametrize('seed',range(10))
def test_random_quaternion_geometry_and_order(seed,authority):
    r=random.Random(seed);ops=[]
    q=[r.uniform(-1,1) for _ in range(4)];length=math.sqrt(sum(v*v for v in q));q=[v/length for v in q]
    for index in range(15):
        p=vision();item=p['observations'][0];item.update(center_m=[r.uniform(-4,4) for _ in range(3)],size_m=[r.uniform(.001,2) for _ in range(3)],position_variance_m2=r.uniform(0,.5),measurement_id=str(index))
        if index%2:item.update(velocity_m_s=[r.uniform(-2,2) for _ in range(3)],velocity_variance_m2_s2=r.uniform(0,.5))
        ops.append(dict(packet=p))
    check(dict(transform=dict(translation=[.3,-.4,.6],rotation=q),operations=ops),authority)


def test_cloud_unused_fields_duplicates_layout_and_small_buffer(authority):
    base=cloud([(1.,2.,3.)],padding=8)
    cases=[]
    for extra in [dict(name='unused',offset=12,datatype=99,count=1),dict(name='unused',offset=12,datatype=7,count=0),
                  dict(name='unused',offset=12,datatype=7,count=2),dict(name='x',offset=12,datatype=7,count=1),
                  dict(name='unused',offset=19,datatype=7,count=1),dict(name='',offset=12,datatype=7,count=1)]:
        cases.append(base|dict(fields=base['fields']+[extra]))
    cases += [base|dict(data=base['data'][:-1]),base|dict(point_step=3),base|dict(fields=base['fields'][:2]),
              base|dict(fields=[base['fields'][0]|dict(count=2)]+base['fields'][1:]),
              base|dict(fields=list(reversed(base['fields'])))]
    for packet in cases:check(dict(adapter='pointcloud_boxes',options={'sensor_id':'lidar'},operations=[dict(packet=packet)]),authority)


def test_cloud_actual_row_padding_and_overlapping_endian_fields(authority):
    p=cloud([(1.,2.,3.),(4.,5.,6.),(7.,8.,9.),(10.,11.,12.)],width=2,height=2,row_padding=12)
    p['data']=p['data'][:24]+[0]*12+p['data'][24:]
    check(dict(adapter='pointcloud_boxes',options={'sensor_id':'lidar'},operations=[dict(packet=p)]),authority)
    p=cloud([(1.,2.,3.),(4.,5.,6.)],bigendian=True);p['fields'][1]['offset']=0
    check(dict(adapter='pointcloud_boxes',options={'sensor_id':'lidar'},operations=[dict(packet=p)]),authority)


def test_cloud_point_and_generated_string_budget_boundaries(authority):
    packet=cloud([(-0.,2.,3.),(4.,5.,6.)],'ddd')
    base=dict(adapter='pointcloud_boxes',options={'sensor_id':'摄像头'},operations=[dict(packet=packet)])
    for limit in (0,1,2,3):check(base|dict(options=base['options']|dict(max_points=limit)),authority)
    parsed=reference(base,authority)[0]['last_packet'];size=len(json.dumps(parsed))
    for limit in (size-1,size,size+1):check(base|dict(options=base['options']|dict(max_packet_bytes=limit)),authority)
    check(base|dict(options={}),authority)


def test_vision_field_failures_and_partial_processing_state(authority):
    cases=[]
    for field,value in [('size_m',[0.,1.,1.]),('center_m',[True,1.,1.]),('position_variance_m2',-1.),
                        ('velocity_m_s',[1.,0.,0.]),('velocity_variance_m2_s2',.1),('geometry_quality',1.1),
                        ('provenance',[]),('provenance',['x','x']),('classes',[]),('classes',{'x':1.1}),
                        ('track_id',''),('track_id',0),('measurement_id','')]:
        p=vision();p['observations'][0][field]=value;cases.append(dict(packet=p))
    p=vision();p['observations'].append(p['observations'][0]|dict(kind='unsupported'));cases.append(dict(packet=p))
    p=vision();del p['observations'][0]['measurement_id'];cases.append(dict(packet=p))
    check(dict(operations=cases),authority)


def test_json_schema_coercions_nonfinite_and_empty_iterables(authority):
    ops=[dict(packet=vision(schema_version=value)) for value in (True,1.,'1',0,None)]
    ops += [dict(packet=vision(stamp_ns=value,calibration_epoch=' 7 ')) for value in ('1_00',100.9,True,-1)]
    ops += [dict(packet=vision(observations=value)) for value in ({},'',None)]
    noframe=vision(observations=[]);del noframe['frame_id'];ops.append(dict(packet=noframe))
    for token in ('NaN','Infinity','-Infinity','1e999'):
        body=json.dumps(vision());body=body.replace('0.01',token);ops.append(dict(data=body))
    ops += [dict(data='\ufeff'+json.dumps(vision())),dict(data='[]'),dict(data='null')]
    check(dict(operations=ops),authority)


def test_packet_rejections_do_not_replace_last_packet_or_consume_clock(authority):
    first=vision();large=vision(observations=first['observations']*3)
    check(dict(options=dict(max_observations=2,sensor_id='camera'),operations=[dict(packet=first),dict(packet=large),dict(packet=vision(sensor_id='wrong')),dict(data='not-json'),dict(packet=vision(schema_version=0))]),authority)
    check(dict(adapter='pointcloud_boxes',options=dict(sensor_id='lidar',max_points=2),operations=[dict(packet=cloud([(1.,2.,3.)])),dict(packet=cloud([])),dict(packet=cloud([(1.,2.,3.)]*3)),dict(packet=cloud([(math.nan,0.,0.)]))]),authority)


def test_nonunit_quaternion_signed_zero_and_source_time_limits(authority):
    for q in ([0.,0.,0.,0.],[.1,.2,.3,.4],[-0.,0.,-0.,1.],[1.,0.,0.,0.]):
        p=vision();p['observations'][0]['center_m']=[-0.,-0.,-0.]
        check(dict(transform=dict(translation=[-0.,-0.,-0.],rotation=q),operations=[dict(packet=p)]),authority)
    check(dict(steady=[-1],operations=[dict(packet=vision())]),authority)
    check(dict(operations=[dict(packet=vision(stamp_ns=2147483647999999999))]),authority)


def test_last_metadata_empty_frame_and_deferred_resolution_values(authority):
    p=vision(observations=[],resolved_measurement_ids=['old',42,{},['x']]);del p['frame_id']
    check(dict(operations=[dict(packet=p),dict(action='metadata')]),authority)
    check(dict(adapter='pointcloud_boxes',options=dict(sensor_id='lidar'),operations=[dict(packet=cloud([(1.,2.,3.)])),dict(action='metadata')]),authority)


def test_explicit_native_factory_extension_and_default_monotonic_receipts():
    result=native(dict(operations=[dict(action='registry')]))[0]['result']
    assert result==dict(custom_kind=True,custom_factory=True,duplicate_rejected=True,python_plugin_rejected=True,mismatched_kind_rejected=True)
    p=vision();p['observations']*=2
    output=native(dict(use_default_steady=True,operations=[dict(packet=p)]))[0]
    assert output['ok'];a,b=[o['received_at'] for o in output['result']]
    assert a['clock']==b['clock']=='steady' and a['epoch']==b['epoch']==0 and 0<=a['ns']<=b['ns']


def test_constructor_argument_expansion_and_error_order(authority):
    image=dict(kind='image_box',measurement_id='i',image_size_px=[640,480,1.],box_xyxy_px=[2.,40.,50.],geometry_quality=.5,provenance=['i'])
    ops=[dict(packet=vision(observations=[image])),dict(packet=vision(observations=[image],frame_id=42))]
    image_bad=image|dict(image_size_px=[0,'bad'],box_xyxy_px=[1.,2.,3.,4.])
    ops.append(dict(packet=vision(observations=[image_bad])))
    for changes in [dict(sensor_id=' ',observations=[vision()['observations'][0]|dict(measurement_id=42)]),
                    dict(observations=[vision()['observations'][0]|dict(geometry_quality=2.,provenance=[42])]),
                    dict(observations=[vision()['observations'][0]|dict(provenance=['x','x',42])])]:
        ops.append(dict(packet=vision(**changes)))
    cone=dict(kind='bearing_cone',measurement_id='b',direction=[2.,0.,0.],half_angle_rad=True,geometry_quality=1.,provenance=['b'])
    ops.append(dict(packet=vision(observations=[cone])))
    check(dict(operations=ops),authority)


def test_full_uint32_image_dimensions(authority):
    item=dict(kind='image_box',measurement_id='wide',image_size_px=[4294967295,4294967295],box_xyxy_px=[0.,0.,4294967295.,4294967295.],geometry_quality=1.,provenance=['wide'])
    check(dict(operations=[dict(packet=vision(observations=[item]))]),authority)


def test_point_field_expansion_error_precedence(authority):
    p=cloud([(1.,2.,3.)],padding=8)
    cases=[p['fields']+[dict(name='oversize',offset=100,datatype=7,count=1),dict(name='bad',offset=0,datatype=99,count=1)],
           p['fields']+[dict(name='a',offset=12,datatype=7,count=2),dict(name='a_0',offset=12,datatype=7,count=1)],
           p['fields']+[dict(name='a_1',offset=12,datatype=7,count=1),dict(name='a',offset=12,datatype=7,count=2)],
           p['fields']+[dict(name='a',offset=12,datatype=7,count=2),dict(name='a_01',offset=12,datatype=7,count=1)]]
    for fields in cases:check(dict(adapter='pointcloud_boxes',options={'sensor_id':'lidar'},operations=[dict(packet=p|dict(fields=fields))]),authority)


def test_numeric_string_conversion_semantics(authority):
    ops=[]
    for value in [' 1_0e-3 ','.001','1.','+.01','0x1p-1','nan(payload)','Inf','-Infinity','\u3000٠.١\u3000','١_٢e-٣','1__0','_1','1_', '.', '']:
        p=vision();p['observations'][0]['position_variance_m2']=value;ops.append(dict(packet=p))
    for value in ['\u3000١_٢\u3000','１２３','+','1__0']:
        ops.append(dict(packet=vision(calibration_epoch=value)))
    check(dict(operations=ops),authority)


def test_seeded_stateful_mixed_geometry_and_fault_replay(authority):
    r=random.Random(694123)
    base=[vision()['observations'][0],
          dict(kind='image_box',measurement_id='image',image_size_px=[640,480],box_xyxy_px=[1.,2.,40.,50.],geometry_quality=.5,provenance=['image']),
          dict(kind='bearing_cone',measurement_id='bearing',direction=[1.,0.,0.],half_angle_rad=.2,geometry_quality=1.,provenance=['bearing'])]
    common={'measurement_id':['',12,'new'],'track_id':[None,'','id',False], 'geometry_quality':[-1.,0.,2.,' 1_0e-2 ',None],
            'provenance':[[],['x','x'],[12],['x',12],'abc',{}], 'classes':[None,[],{'a':.7,'b':.4},{'':0.},{'a':'bad'}]}
    specific=[{'center_m':[[0.,1.],[True,1.,2.],[0.,None,1.]],'size_m':[[1.,0.,2.],[-1.,1.,1.]],'position_variance_m2':[-1.,'nan','bad'], 'velocity_m_s':[[1.,2.,3.]]},
              {'image_size_px':[[0,480],[640.,480],[640,480,0]],'box_xyxy_px':[[1.,2.,0.,3.],[False,2.,4.,5.],[0.,0.,640.,480.]]},
              {'direction':[[0.,0.,0.],[1.,0.],[True,0.,0.]],'half_angle_rad':[-1.,4.,True,'bad']}]
    ops=[]
    for i in range(600):
        index=r.randrange(3);item=base[index].copy();choices=common|specific[index]
        for _ in range(r.randrange(1,4)):
            key=r.choice(list(choices));item[key]=r.choice(choices[key])
        if i%17==0:item=base[index].copy()
        ops.append(dict(packet=vision(observations=[item])))
    check(dict(operations=ops),authority)


def test_frozen_authority_bytes():
    expected={
        'contracts.py':'6a55bc374c1ae146d33e13ad7d7a0bb98514007cbf8816cb1ad09c1015d019b5',
        'observation_adapters.py':'2585066e21070b4cfd102741d360e52d54b0403363b77848324052f538e25505',
        'point_cloud2.py':'79582d403a689d88468e064dcfc39a4d6748de3459bc0492084a4abb0831e7eb',
        'numpy_compat.py':'0959235b3e47d6d764abaa6f1121458579cfb928a8e67db1acc23aacab7290dc'}
    for name,digest in expected.items():assert hashlib.sha256((REF/name).read_bytes()).hexdigest()==digest


def test_raw_options_defer_vision_conversion_and_unused_fields(authority):
    for options in [dict(max_packet_bytes=None),dict(max_packet_bytes='bad'),dict(max_observations='bad'),
                    dict(max_points='unused bad value',calibration_epoch=[],position_variance_m2={}),
                    dict(max_packet_bytes='１_０００',max_observations=' １ '),None,[],{},'',0,False,[1],'text',1]:
        check(dict(raw_options=options,operations=[dict(packet=vision()),dict(data='{bad'),dict(packet=vision(observations=[]))]),authority)


def test_raw_options_sensor_truth_and_python_equality(authority):
    for sensor in [None,False,0,True,1,1.5,'',[],{},['x'],{'a':1,'b':2}]:
        check(dict(raw_options=dict(sensor_id=sensor),operations=[dict(packet=vision(sensor_id=sensor,observations=[])),dict(packet=vision(sensor_id=sensor)),dict(packet=vision())]),authority)
    check(dict(raw_options={'sensor_id':True},operations=[dict(packet=vision(sensor_id=1,observations=[]))]),authority)
    check(dict(raw_options={'sensor_id':{'a':1,'b':2}},operations=[dict(packet=vision(sensor_id={'b':2,'a':1},observations=[]))]),authority)
    check(dict(raw_options={'sensor_id':9007199254740993},operations=[dict(packet=vision(sensor_id=9007199254740992.,observations=[]))]),authority)


def test_raw_cloud_options_conversion_order_and_byte_mutation(authority):
    p=cloud([(1.,2.,3.)],bigendian=True)
    for options in [dict(max_points=None),dict(max_points='bad'),dict(sensor_id='lidar',calibration_epoch='bad',position_variance_m2=None),
                    dict(sensor_id='lidar',calibration_epoch=' １_２ ',position_variance_m2=' .0_1 ',max_points='１'),
                    dict(sensor_id='lidar',position_variance_m2=None),dict(sensor_id='lidar',max_packet_bytes=None),
                    dict(sensor_id='lidar',max_observations='bad'),{},None,[],[1],'text',
                    dict(sensor_id=None),dict(sensor_id=1),dict(sensor_id=False),dict(sensor_id=['x']),dict(sensor_id={'a':1})]:
        check(dict(adapter='pointcloud_boxes',raw_options=options,operations=[dict(packet=p),dict(packet=cloud([]))]),authority)


def test_raw_calibration_update_overrides_bad_prior_option(authority):
    check(dict(adapter='pointcloud_boxes',raw_options=dict(sensor_id='lidar',calibration_epoch='bad'),operations=[dict(packet=cloud([(1.,2.,3.)])),dict(action='calibration',value=9),dict(packet=cloud([(1.,2.,3.)]))]),authority)


def test_bigint_calibration_retains_exact_epoch(authority):
    for value in [2**63,2**64-1,2**64,2**64+1,10**399,-2**64]:
        check(dict(operations=[dict(packet=vision(calibration_epoch=value))]),authority)
    for value in [' １２_３４５６７８９０１２３４５６７８９０１２３ ',1e100,-1e100]:
        check(dict(operations=[dict(packet=vision(calibration_epoch=value))]),authority)


def test_bigint_image_exact_mixed_pixel_boundaries(authority):
    item=dict(kind='image_box',measurement_id='wide',image_size_px=[100,100],box_xyxy_px=[0.,0.,10.,10.],geometry_quality=1.,provenance=['wide'])
    cases=[(2**64-1,float(2**64)),(2**64,float(2**64)),(2**64-1,2**64-1),
           (10**399,1.),(10**399,10**398),(2**53+1,2**53+1),(2**53,2**53+1),
           (2**64+1,2**64+1),(2**64+1,2**64+2),(-2**64,1.),(True,1.)]
    for width,xmax in cases:
        p=vision(observations=[item|dict(image_size_px=[width,100],box_xyxy_px=[0.,0.,xmax,10.])])
        check(dict(operations=[dict(packet=p)]),authority)


def test_bigint_unknown_fields_and_transport_marker_collision(authority):
    p=vision(observations=[],unknown=[2**64+1,-2**64-1,10**399,{'bytes':[49,50,51],'subtype':4409680},'__integer_json__0','__native_nonfinite__0'])
    check(dict(operations=[dict(packet=p)]),authority)
    raw=json.dumps(vision(observations=[]))[:-1]+',"same":18446744073709551617,"same":18446744073709551619}'
    check(dict(operations=[dict(data=raw)]),authority)


def test_bigint_raw_and_typed_option_budgets(authority):
    for value in [2**64+1,10**399,str(10**399),-2**64]:
        for name in ['max_packet_bytes','max_observations']:
            check(dict(raw_options={name:value},operations=[dict(packet=vision())]),authority)
        check(dict(adapter='pointcloud_boxes',raw_options=dict(sensor_id='lidar',max_points=value,calibration_epoch=2**64+1),operations=[dict(packet=cloud([(1.,2.,3.)]))]),authority)
    check(dict(adapter='pointcloud_boxes',raw_options=dict(sensor_id='lidar',calibration_epoch=2**64+1),operations=[dict(packet=cloud([(1.,2.,3.)])),dict(action='calibration',value=10**399),dict(packet=cloud([(1.,2.,3.)]))]),authority)
    check(dict(raw_options={'sensor_id':2**64+1},operations=[dict(packet=vision(sensor_id=2**64,observations=[])),dict(packet=vision(sensor_id=2**64+1,observations=[]))]),authority)


def test_json_integer_digit_limit_and_error_order(authority):
    assert sys.get_int_max_str_digits()==4300
    base=json.dumps(vision(observations=[]))[:-1]
    huge='7'*4301
    ops=[dict(data=base+',"ignored":'+('7'*4300)+'}'),dict(data=base+',"ignored":'+huge+'}'),
         dict(data='{"invalid": ,"ignored":'+huge+'}'),dict(data='{"ignored":'+huge+',"invalid": }'),
         dict(data=base+',"ignored":"'+huge+'"}'),dict(packet=vision(calibration_epoch=huge)),
         dict(data=base+',"ignored":1e999}'),dict(data=base+',"ignored":-Infinity}')]
    check(dict(operations=ops),authority)


def test_lossless_json_codec_collisions_duplicates_and_syntax(authority):
    huge='9'*4301
    texts=[
        '{"__integer_json__":0,"__integer_json___":1,"value":18446744073709551617}',
        '{"bytes":[49,50],"subtype":4409680,"value":-18446744073709551617}',
        '{"x":18446744073709551617,"x":18446744073709551619}',
        '{"x":"18446744073709551617","y":"__integer_json__","z":18446744073709551617}',
        '[1.5980597012435153e+306,-3.0783420417682518e+305,5e-324,1e-999]',
        '[NaN,Infinity,-Infinity,1e999,-1e999,18446744073709551617]',
        '{18446744073709551617:1}','{NaN:1}','[018446744073709551617]',
        '18446744073709551617x','[1e+]', '{"bad":,'+huge+'}',
        '{"large":'+huge+',"large":1}',huge+'x']
    check(dict(operations=[dict(action='json',data=text) for text in texts]),authority)


def test_lossless_json_exact_random_integer_and_float_roundtrip(authority):
    r=random.Random(1946641)
    values=[r.randrange(-(1<<bits),1<<bits) for bits in [0,32,53,63,64,65,100,1024,4096,14000] for _ in range(20)]
    values.extend(math.ldexp(r.uniform(-1,1),r.randrange(-1074,1024)) for _ in range(1000))
    case=dict(operations=[dict(action='json',data=json.dumps(values))])
    expected=reference(case,authority);got=native(case)
    assert expected==got


def test_bigint_budget_uses_identical_vision_wire_text(authority):
    p=vision(calibration_epoch=2**64+1);size=len(json.dumps(p))
    for budget in [size-1,size,size+1,10**399]:
        check(dict(raw_options=dict(max_packet_bytes=budget),operations=[dict(packet=p)]),authority)


@pytest.mark.parametrize('limit',[0,640,4300])
def test_configured_integer_digit_limit_subprocesses(limit):
    size=4301 if limit==0 else limit+1
    huge='7'*size
    case=dict(operations=[dict(action='json',data=huge),dict(action='json',data='7'*(size-1)),
                          dict(packet=vision(calibration_epoch=huge)),dict(action='integer_dump',hex=hex(10**(size-1))[2:])])
    env=dict(os.environ,PYTHONINTMAXSTRDIGITS=str(limit))
    script="""import importlib.util,json,sys
from pathlib import Path
spec=importlib.util.spec_from_file_location('configured_reference',Path(sys.argv[1]));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
fixture=m.authority.__wrapped__();a=next(fixture)
try: print(json.dumps(m.reference(json.load(sys.stdin),a)))
finally:
 try: next(fixture)
 except StopIteration: pass
"""
    source=subprocess.run([sys.executable,'-c',script,str(Path(__file__).resolve())],input=json.dumps(case),text=True,capture_output=True,check=True,env=env)
    previous=sys.get_int_max_str_digits();sys.set_int_max_str_digits(0)
    try:compare(json.loads(source.stdout),native(case,env=env))
    finally:sys.set_int_max_str_digits(previous)


def test_invalid_integer_digit_limit_environment_rejects_process():
    for value in ['639','-1','640 ','6_40','６４０','2147483648']:
        env=dict(os.environ,PYTHONINTMAXSTRDIGITS=value)
        source=subprocess.run([sys.executable,'-c','pass'],text=True,capture_output=True,env=env)
        target=subprocess.run([str(PROBE)],input='{"operations":[]}',text=True,capture_output=True,env=env)
        assert source.returncode!=0 and target.returncode!=0,(value,source.returncode,target.returncode)
