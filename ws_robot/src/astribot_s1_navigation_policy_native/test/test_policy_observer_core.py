"""Frozen observer methods plus exact task/map/localization identity transitions."""
import ast
from array import array
import dataclasses
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import types

import pytest

HERE=Path(__file__).resolve().parent
REF=HERE/'reference/policy_observer'
NAME='_frozen_policy_observer'
pkg=types.ModuleType(NAME);pkg.__path__=[str(REF)];sys.modules[NAME]=pkg
os.environ.pop('ASTRIBOT_NAV_NATIVE_KERNELS',None)
for name in ('contracts','execution_context'):
    spec=importlib.util.spec_from_file_location(NAME+'.'+name,REF/(name+'.py'))
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
C=sys.modules[NAME+'.contracts'];E=sys.modules[NAME+'.execution_context']
source=ast.parse((REF/'observer_node.py').read_text())
selected=[]
for node in source.body:
    if isinstance(node,ast.FunctionDef) and node.name=='yaw':selected.append(node)
    if isinstance(node,ast.ClassDef) and node.name=='PolicyObserver':
        selected.append(ast.ClassDef(name='FrozenObserver',bases=[],keywords=[],body=[f for f in node.body if isinstance(f,ast.FunctionDef) and f.name in ('mapping','static_mask','static_at','point')],decorator_list=[]))
namespace={'math':math};exec(compile(ast.fix_missing_locations(ast.Module(body=selected,type_ignores=[])),str(REF/'observer_node.py'),'exec'),namespace)
Observer=namespace['FrozenObserver']
S=types.SimpleNamespace

def encoded(value):
    if isinstance(value,float) and not math.isfinite(value):return 'NaN' if math.isnan(value) else ('Infinity' if value>0 else '-Infinity')
    if isinstance(value,dict):return {k:encoded(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [encoded(v) for v in value]
    return value

def reference(operations):
    state=Observer();state.execution=E.ExecutionContext();state.map=None;out=[]
    for op in operations:
        row={};action=op['action'];context=state.execution
        try:
            if action=='task':context.task(op['id'],op['state'],op['sequence'])
            elif action=='path':context.path()
            elif action=='localization':row['changed']=context.localization(op['pose'],op['position_limit'],op['angle_limit'])
            elif action=='version':context.version=C.Version(*op['version'])
            elif action=='map':
                origin=op['origin'];previous=context.version
                msg=S(header=S(frame_id=op['frame']),info=S(width=op['width'],height=op['height'],resolution=op['resolution'],origin=S(position=S(x=origin[0],y=origin[1],z=origin[2]),orientation=S(x=origin[3],y=origin[4],z=origin[5],w=origin[6]))),data=array('b',op['cells']))
                state.mapping(msg);row['accepted']=state.map is msg
                if row['accepted']:row['changed']=previous.map_epoch!=context.version.map_epoch
            elif action=='lookup':
                row['values']=[state.static_at(*p) for p in op['points']]
                if state.map is not None:row['mask']=state.static_mask()[-1].tolist()
            elif action=='point':
                t=op['transform'];tf=S(transform=S(translation=S(x=t[0],y=t[1],z=t[2]),rotation=S(x=t[3],y=t[4],z=t[5],w=t[6])))
                row['point']=list(state.point(op['point'],tf))
        except Exception as error:row={'error':str(error)}
        row['version']=list(dataclasses.astuple(context.version));out.append(row)
    return out

@pytest.fixture(scope='session')
def probe():
    executable=Path(os.environ.get('POLICY_OBSERVER_CORE_PROBE','/tmp/codex_policy_observer_20260921/policy_observer_core_probe'))
    assert executable.is_file(),f'Native observer core executable required: {executable}'
    p=subprocess.Popen([str(executable)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    def call(ops):
        p.stdin.write(json.dumps(encoded({'operations':ops}))+'\n');p.stdin.flush();return json.loads(p.stdout.readline())
    yield call
    p.stdin.close();p.wait(timeout=10);assert p.returncode==0

def compare(probe,ops):
    want=reference(ops);got=probe(ops);assert len(got)==len(want)
    for index,(a,b) in enumerate(zip(got,want)):
        assert a.keys()==b.keys(),(index,ops[index],a,b)
        for key in a:
            if key=='point':assert a[key]==pytest.approx(b[key],rel=2e-14,abs=2e-14)
            else:assert a[key]==b[key],(index,ops[index],a,b)
    return got

def test_frozen_authority_hashes():
    manifest=json.loads((REF/'manifest.json').read_text())
    for name,record in manifest['files'].items():
        path=REF/(name if name.endswith('.json') else name+'.py')
        assert hashlib.sha256(path.read_bytes()).hexdigest()==record['sha256'],name

def mapping(**kwargs):
    return dict(dict(action='map',frame='map',width=3,height=2,resolution=.1,origin=[1.,-2.,0.,0.,0.,0.,1.],cells=[-1,64,65,100,0,1]),**kwargs)

def test_map_unknown_edges_rotation_and_retention(probe):
    ops=[dict(action='lookup',points=[[0.,0.]])]
    for angle in (0.,math.pi/2,-math.pi/3):
        m=mapping(origin=[1.,-2.,0.,0.,0.,math.sin(angle/2),math.cos(angle/2)])
        ops += [m,dict(action='lookup',points=[[1.+x*.1,-2.+y*.1] for x in range(-4,8) for y in range(-4,8)])]
        for bad in (dict(frame='odom'),dict(resolution=0.),dict(cells=[]),dict(width=4),dict(origin=[math.nan,0.,0.,0.,0.,0.,1.])):
            ops += [mapping(**bad),dict(action='lookup',points=[[1.,-2.]])]
    compare(probe,ops)

def test_map_identity_ignores_z_stamp_but_retains_full_metadata(probe):
    base=mapping();ops=[base,base,mapping(origin=[1.,-2.,math.nan,0.,0.,0.,1.]),mapping(origin=[1.,-2.,0.,-0.,0.,0.,1.]),mapping(cells=[-1,64,65,100,0,2]),base,mapping(origin=[1.,-2.,0.,0.,0.,0.,-1.])]
    out=compare(probe,ops);assert [r['version'][2] for r in out]==[1,1,1,1,2,3,4]

@pytest.mark.parametrize('terminal',['SUCCEEDED','FAILED','CANCELED','PREEMPTED'])
def test_task_sequence_path_and_terminal_owner(probe,terminal):
    compare(probe,[dict(action='task',id='a',state='EXECUTING',sequence=0),dict(action='path'),dict(action='task',id='b',state='EXECUTING',sequence=0),dict(action='task',id='b',state=terminal,sequence=1),dict(action='task',id='a',state=terminal,sequence=2),dict(action='task',id='b',state='EXECUTING',sequence=3)])

def test_localization_strict_norm_boundary(probe):
    op=lambda pose:dict(action='localization',pose=pose,position_limit=.5,angle_limit=.15)
    result=compare(probe,[op([0.,0.,0.]),op([.3653736529718966,.34132403037871817,0.])])
    assert result[-1]['changed'] is True

def test_localization_nextafter_default_and_wraparound(probe):
    rng=random.Random(58493);ops=[]
    for limit in (.2,.5):
        for _ in range(800):
            angle=rng.uniform(-math.pi,math.pi);x=limit*math.cos(angle);y=limit*math.sin(angle)
            x=math.nextafter(x,rng.choice([-math.inf,math.inf]))
            ops.extend([dict(action='localization',pose=[0.,0.,0.],position_limit=limit,angle_limit=.15),dict(action='localization',pose=[x,y,0.],position_limit=limit,angle_limit=.15)])
    for a in (.15,math.nextafter(.15,math.inf),2*math.pi,2*math.pi+.15):
        ops.extend([dict(action='localization',pose=[0.,0.,0.],position_limit=.2,angle_limit=.15),dict(action='localization',pose=[0.,0.,a],position_limit=.2,angle_limit=.15)])
    compare(probe,ops)

def test_seeded_rotated_maps_and_threshold_cells(probe):
    rng=random.Random(40816);ops=[]
    for _ in range(80):
        width=rng.randrange(1,12);height=rng.randrange(1,12);angle=rng.uniform(-math.pi,math.pi)
        ops.extend([mapping(width=width,height=height,origin=[0.,0.,0.,0.,0.,math.sin(angle/2),math.cos(angle/2)],resolution=.2,cells=[rng.choice([-1,0,1,64,65,100]) for _ in range(width*height)]),dict(action='lookup',points=[[rng.uniform(-3,3),rng.uniform(-3,3)] for _ in range(100)])])
    compare(probe,ops)

def test_randomized_identity_sequences(probe):
    rng=random.Random(980114);ops=[]
    for i in range(3000):
        action=rng.randrange(5)
        if action==0:ops.append(dict(action='task',id=rng.choice(['a','b','idle']),state=rng.choice(['EXECUTING','SUCCEEDED','FAILED','CANCELED','PREEMPTED','PENDING']),sequence=rng.randrange(max(1,i))))
        elif action==1:ops.append(dict(action='path'))
        elif action==2:ops.append(mapping(cells=[rng.choice([-1,0,64,65,100]) for _ in range(6)]))
        elif action==3:ops.append(dict(action='localization',pose=[rng.uniform(-1,1),rng.uniform(-1,1),rng.uniform(-20,20)],position_limit=.2,angle_limit=.15))
        else:ops.append(dict(action='version',version=['a',i,i//2,i//3,i//4,i//5]))
    compare(probe,ops)

@pytest.mark.parametrize('angle',[0.,.01,math.pi/2,math.pi,-math.pi/2])
def test_raw_quaternion_point_transform(probe,angle):
    compare(probe,[dict(action='point',point=[1.,-2.,.5],transform=[3.,4.,5.,0.,0.,math.sin(angle/2),math.cos(angle/2)]),dict(action='point',point=[math.nan,0.,0.],transform=[0.,0.,0.,0.,0.,0.,1.])])

@pytest.mark.parametrize('sequence',[2**63-1,2**63,2**64-1])
def test_wire_uint64_task_sequence(probe,sequence):
    compare(probe,[dict(action='task',id='a',state='EXECUTING',sequence=sequence-1),
                   dict(action='path'),dict(action='task',id='b',state='EXECUTING',sequence=sequence),
                   dict(action='task',id='b',state='SUCCEEDED',sequence=sequence-1),
                   dict(action='task',id='a',state='EXECUTING',sequence=-1),
                   dict(action='task',id='b',state='SUCCEEDED',sequence=sequence)])

@pytest.mark.parametrize('counter',[2**63-1,2**63,2**64-1])
def test_wire_uint64_context_version(probe,counter):
    compare(probe,[dict(action='version',version=['wire',counter,counter,counter,counter,counter]),
                   dict(action='task',id='next',state='EXECUTING',sequence=0)])

def test_wire_counter_overflow_is_explicit_and_leaves_context_unchanged(probe):
    # Beyond uint64 is outside the wire domain, unlike Python's unbounded ints.
    maximum=2**64-1
    local=lambda p:dict(action='localization',pose=p,position_limit=.2,angle_limit=.15)
    ops=[dict(action='version',version=['wire',maximum,maximum,maximum,maximum,maximum]),
         dict(action='path'),mapping(),local([0.,0.,0.]),local([1.,0.,0.]),
         dict(action='version',version=['wire',0,0,maximum,0,0]),mapping(),local([1.,0.,0.])]
    rows=probe(ops)
    assert rows[1]['error']=='path_revision exceeds uint64 wire range'
    assert rows[2]['error']=='map_epoch exceeds uint64 wire range'
    assert rows[4]['error']=='localization_epoch exceeds uint64 wire range'
    assert rows[1]['version']==rows[2]['version']==rows[4]['version']==rows[0]['version']
    assert rows[6]['changed'] and rows[7]['changed']
    assert rows[7]['version']==['wire',0,1,maximum,1,0]

def test_wire_signed_task_negative_inputs_still_ignore(probe):
    compare(probe,[dict(action='task',id='a',state='EXECUTING',sequence=-2**63),
                   dict(action='task',id='a',state='EXECUTING',sequence=-1),
                   dict(action='task',id='a',state='EXECUTING',sequence=0),
                   dict(action='task',id='a',state='SUCCEEDED',sequence=-1),
                   dict(action='task',id='a',state='SUCCEEDED',sequence=1)])
