"""Catch lost polygon shape, interval coverage, and changed rejection boundaries."""
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
REF = HERE / 'reference' / 'policy_risk'
sys.path.insert(0, str(REF))
os.environ.pop('ASTRIBOT_NAV_NATIVE_KERNELS', None)
from astribot_s1_navigation_policy.swept_geometry import clearance_many
from astribot_s1_navigation_policy.continuous_sweep import motion_clearance

RECT = dict(half_length_m=.5, half_width_m=.3, clearance_margin_m=.04,
            payload_extra_margin_m=.01)
POLY = dict(RECT, half_length_m=.9, half_width_m=.6,
            footprint_xy=[[-.5, -.3], [.9, -.3], [.9, .15], [.1, .6], [-.5, .3]])

def encoded(value):
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else ('Infinity' if value > 0 else '-Infinity')
    if isinstance(value, dict): return {k: encoded(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)): return [encoded(v) for v in value]
    return value

@pytest.fixture(scope='session')
def probe():
    value = Path(os.environ.get('POLICY_SWEEP_PROBE', '/tmp/codex_policy_risk_20260921/policy_sweep_probe'))
    assert value.is_file(), f'Native sweep executable required: {value}'
    process = subprocess.Popen([str(value)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    def call(payload):
        process.stdin.write(json.dumps(encoded(payload), allow_nan=False)+'\n');process.stdin.flush()
        return json.loads(process.stdout.readline())
    yield call
    process.stdin.close();process.wait(timeout=10)
    assert process.returncode == 0

def compare(probe, data):
    profile = SimpleNamespace(**data['profile'])
    boxes = np.asarray(data['lower'],dtype=float).reshape((-1,2)), np.asarray(data['upper'],dtype=float).reshape((-1,2))
    try:
        if data['op'] == 'clearance':
            want = clearance_many(np.asarray(data['x']),np.asarray(data['y']),np.asarray(data['yaw']),*boxes,profile,data.get('sampling_margin',0.))
        else:
            want = motion_clearance(data['command'],data['begin'],data['end'],*boxes,profile,data['origin'])
    except ValueError:
        assert 'error' in probe(data)
        return
    got = probe(data)
    assert 'error' not in got, got
    got = np.asarray([float(v) for v in got['values']])
    np.testing.assert_allclose(got,want,rtol=2e-11,atol=2e-12)
    np.testing.assert_array_equal(got>0,want>0)
    return got

@pytest.mark.parametrize('profile',[RECT,POLY])
def test_contact_containment_and_distance(probe,profile):
    # Removing filled-polygon containment or the 1e-12 reserve admits contact.
    data=dict(op='clearance',profile=profile,x=[0.]*5,y=[0.]*5,yaw=[0.]*5,
              lower=[[0,0],[.2,-.01],[2,0],[.95,0],[-1,-1]],
              upper=[[0,0],[.21,.01],[2.1,.1],[.95,0],[-.9,-.9]])
    out=compare(probe,data)
    assert out[0]<0 and out[1]<0 and out[2]>0
    if profile is POLY: assert out[3]<=0

@pytest.mark.parametrize('profile',[RECT,POLY])
def test_sweep_crossing_origin_and_empty(probe,profile):
    data=dict(op='sweep',profile=profile,command=[3.,0.,0.],origin=[0.,0.,0.],
              begin=[0.],end=[2.],lower=[[2.5,-.05]],upper=[[2.6,.05]])
    assert compare(probe,data)[0]<0
    data.update(origin=[10.,-4.,math.pi/2],lower=[[9.95,-1.5]],upper=[[10.05,-1.4]])
    assert compare(probe,data)[0]<0
    data.update(begin=[],end=[],lower=[],upper=[])
    assert len(compare(probe,data))==0

@pytest.mark.parametrize('profile',[RECT,POLY])
def test_randomized_clearance_and_rotating_sweep(probe,profile):
    rng=random.Random(740129)
    for sweep in (False,True):
        for _ in range(12):
            n=23
            low=[[rng.uniform(-3,3),rng.uniform(-3,3)] for _ in range(n)]
            high=[[x+rng.uniform(0,.5),y+rng.uniform(0,.5)] for x,y in low]
            data=dict(op='sweep' if sweep else 'clearance',profile=profile,lower=low,upper=high)
            if sweep:
                start=[rng.uniform(0,2) for _ in range(n)]
                data.update(command=[rng.uniform(-1.5,1.5),rng.uniform(-1.5,1.5),rng.choice([0.,1e-6,1e-6-1e-16,rng.uniform(-3,3)])],
                            origin=[rng.uniform(-1,1),rng.uniform(-1,1),rng.uniform(-math.pi,math.pi)],begin=start,end=[v+rng.uniform(0,1) for v in start])
            else:
                data.update(x=[rng.uniform(-2,2) for _ in range(n)],y=[rng.uniform(-2,2) for _ in range(n)],
                            yaw=[rng.uniform(-math.pi,math.pi) for _ in range(n)],sampling_margin=rng.uniform(0,.2))
            compare(probe,data)

@pytest.mark.parametrize('field,value',[('command',[float('nan'),0,0]),('origin',[0,0,float('inf')]),
    ('begin',[-.1]),('end',[-.1]),('lower',[[float('nan'),0.]]),('upper',[[-2.,0.]])])
@pytest.mark.parametrize('profile',[RECT,POLY])
def test_invalid_intervals_never_certify_safe(probe,field,value,profile):
    data=dict(op='sweep',profile=profile,command=[.1,0.,0.],origin=[0.,0.,0.],begin=[0.],end=[1.],lower=[[-1.,0.]],upper=[[1.,1.]])
    data[field]=value
    compare(probe,data)
