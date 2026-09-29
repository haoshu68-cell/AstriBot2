"""Full path semantic identity and bounded-age evidence; no CDR byte hashing."""
import copy
import dataclasses
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import subprocess
from types import SimpleNamespace as S

import pytest

REF=Path(__file__).resolve().parent/'reference/policy_behavior'
os.environ.pop('ASTRIBOT_NAV_NATIVE_KERNELS',None)
spec=importlib.util.spec_from_file_location('_frozen_path_evidence',REF/'path_evidence.py');oracle=importlib.util.module_from_spec(spec);spec.loader.exec_module(oracle)
PROFILE=dict(path_risk_timeout_s=.5,max_speed_m_s=.35,clearance_margin_m=.08,payload_extra_margin_m=.03)
PATH=dict(header=['map',1,7],poses=[dict(header=['map',1,9],geometry=[1.,2.,3.,0.,0.,0.,1.])])
def header(j):return S(frame_id=j[0],stamp=S(sec=j[1],nanosec=j[2]))
def path(j):return S(header=header(j['header']),poses=[S(header=header(p['header']),pose=S(position=S(**dict(zip(('x','y','z'),p['geometry'][:3]))),orientation=S(**dict(zip(('x','y','z','w'),p['geometry'][3:]))))) for p in j['poses']])
def encoded(x):
    if isinstance(x,float) and not math.isfinite(x):return 'NaN' if math.isnan(x) else ('Infinity' if x>0 else '-Infinity')
    if isinstance(x,dict):return {k:encoded(v) for k,v in x.items()}
    if isinstance(x,(tuple,list)):return [encoded(v) for v in x]
    return x
def reference(j):
    try:
        if j['op']=='identity':return dict(same=oracle.path_identity(path(j['left']))==oracle.path_identity(path(j['right'])))
        key=None if j['path'] is None else oracle.path_identity(path(j['path']));e=j['evidence']
        evidence=None if e is None else oracle.PathEvidence(oracle.path_identity(path(e['path'])),e['stamp_s'],e['received_wall_s'],e['epoch'],e['known'],e['blocked'],e['distance_m'])
        return encoded(dataclasses.asdict(oracle.assess_path(evidence,key,j['now_s'],j['wall_s'],j['epoch'],j['legacy_blocked'],S(**j['profile']))))
    except Exception as error:return dict(error=str(error))
def check(cases):
    result=subprocess.run([os.environ['POLICY_PATH_EVIDENCE_PROBE']],input=''.join(json.dumps(encoded(c))+'\n' for c in cases),text=True,capture_output=True,check=True)
    got=[json.loads(s) for s in result.stdout.splitlines()];assert len(got)==len(cases)
    for index,(value,case) in enumerate(zip(got,cases)):assert value==reference(case),(index,case,value,reference(case))
def assessment(**kw):
    return dict(dict(op='assess',path=PATH,evidence=dict(path=PATH,stamp_s=1.,received_wall_s=2.,epoch=3,known=True,blocked=True,distance_m=1.),now_s=1.1,wall_s=2.1,epoch=3,legacy_blocked=False,profile=PROFILE),**kw)

def test_every_header_pose_stamp_and_geometry_field_is_identity():
    variants=[copy.deepcopy(PATH),dict(header=PATH['header'],poses=[])]
    for index,value in enumerate(('odom',2,8)):
        p=copy.deepcopy(PATH);p['header'][index]=value;variants.append(p)
        p=copy.deepcopy(PATH);p['poses'][0]['header'][index]=value;variants.append(p)
    for index in range(7):
        for value in (math.nextafter(PATH['poses'][0]['geometry'][index],math.inf),math.nan,math.inf,-math.inf):
            p=copy.deepcopy(PATH);p['poses'][0]['geometry'][index]=value;variants.append(p)
    p=copy.deepcopy(PATH);p['poses'][0]['geometry'][3]=-0.;variants.append(p)
    check([dict(op='identity',left=PATH,right=p) for p in variants])

def test_fallback_precedes_invalid_age_and_source_gate():
    cases=[assessment(evidence=None,now_s=math.nan,legacy_blocked=v) for v in (False,True)]
    for change in (dict(path=None),dict(epoch=4),dict(path=dict(PATH,header=['other',1,7]))):cases.append(assessment(now_s=math.nan,**change))
    check(cases)

def test_exact_lease_future_unknown_clear_and_distance_boundaries():
    cases=[]
    for blocked in (False,True):
        for known in (False,True):
            for age in (-1e-16,0.,math.nextafter(.5,0.),.5,math.nextafter(.5,math.inf),1.):
                for distance in (-1.,0.,.01,1.,math.nan,math.inf):
                    c=assessment(now_s=age,wall_s=age);c['evidence']=dict(c['evidence'],stamp_s=0.,received_wall_s=0.,known=known,blocked=blocked,distance_m=distance);cases.append(c)
    check(cases)

@pytest.mark.parametrize('seed',[119,71359,20260921])
def test_random_independent_ros_and_wall_ages(seed):
    rng=random.Random(seed);cases=[]
    for _ in range(2000):
        c=assessment(now_s=rng.uniform(-1.,3.),wall_s=rng.uniform(0.,4.),legacy_blocked=rng.choice([False,True]));c['evidence']=dict(c['evidence'],known=rng.choice([False,True]),blocked=rng.choice([False,True]),distance_m=rng.uniform(-.1,10.));c['profile']=dict(PROFILE,max_speed_m_s=rng.uniform(.01,1.),payload_extra_margin_m=rng.uniform(0.,.3));cases.append(c)
    check(cases)
