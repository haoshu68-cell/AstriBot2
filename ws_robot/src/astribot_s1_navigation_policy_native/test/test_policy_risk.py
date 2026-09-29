"""Same-input forecast tests: owner ordering, measured motion and uncertainty."""
from dataclasses import asdict
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

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'reference'/'policy_risk'))
os.environ.pop('ASTRIBOT_NAV_NATIVE_KERNELS',None)
from astribot_s1_navigation_policy.contracts import Covariance3, MetricBox, Stamp, Vec3, Version, Observation, ImageBox
from astribot_s1_navigation_policy.ports import Prediction, PredictionModel, TrackedObstacle, WorldSnapshot
from astribot_s1_navigation_policy.risk import RobotState, evaluate_risk
from astribot_s1_navigation_policy.world_geometry import prediction_rows, final_prediction, has_predictions, prediction_count

PROFILE=dict(half_length_m=.5,half_width_m=.3,clearance_margin_m=.04,payload_extra_margin_m=.01,
    max_speed_m_s=.4,reaction_time_s=.3,brake_deceleration_m_s2=.6,angular_brake_deceleration_rad_s2=1.,prediction_horizon_s=2.)
FOOTPRINT=[[-.5,-.3],[.9,-.3],[.9,.15],[.1,.6],[-.5,.3]]

def metric(v):
    a,b,c=v.get('variance',[0.,0.,0.])
    return MetricBox(Vec3(*v['center']),Vec3(*v['size']),Covariance3((a,0.,0.,0.,b,0.,0.,0.,c)))

def make_world(data):
    stamp=Stamp(10**9,'ros',0); tracks=[]
    for t in data['tracks']:
        m=t.get('model')
        model=PredictionModel(Vec3(*m['velocity']),m['variance'],tuple(map(tuple,m['steps']))) if m else None
        tracks.append(TrackedObstacle(t['id'],'odom',stamp,metric(t['box']),
            tuple(Prediction(p['ns'],metric(p['box'])) for p in t.get('predictions',[])),('source',),model))
    uncertain=() if not data.get('uncertain') else (Observation('camera','measurement',None,stamp,
        Stamp(0,'steady',0),Stamp(2*10**9,'ros',0),'optical',0,ImageBox('camera',100,100,0.,0.,1.,1.),1.,(),('camera',)),)
    return WorldSnapshot(Version('goal',1,1,1),stamp,'odom',tuple(tracks),uncertain,(),1)

@pytest.fixture(scope='session')
def probe():
    executable=Path(os.environ.get('POLICY_RISK_PROBE','/tmp/codex_policy_risk_20260921/policy_risk_probe'))
    assert executable.is_file(),f'Native risk executable required: {executable}'
    p=subprocess.Popen([str(executable)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    def call(data):
        p.stdin.write(json.dumps(data)+'\n');p.stdin.flush()
        return json.loads(p.stdout.readline())
    yield call
    p.stdin.close();p.wait(timeout=10);assert p.returncode==0

def compare(probe,data):
    w=make_world(data)
    if data['op']=='rows':
        try: want=prediction_rows(w,data.get('include_current',False),data.get('swept',False))
        except ValueError:
            assert 'error' in probe(data);return
        got=probe(data);assert 'error' not in got,got
        rows=got['rows'];assert [r['owner'] for r in rows]==want.owners.tolist()
        assert [r['offset_ns'] for r in rows]==want.offsets_ns.tolist()
        np.testing.assert_allclose(np.asarray([r['lower'] for r in rows]).reshape((-1,2)),want.lower,rtol=3e-14,atol=3e-14)
        np.testing.assert_allclose(np.asarray([r['upper'] for r in rows]).reshape((-1,2)),want.upper,rtol=3e-14,atol=3e-14)
        return rows
    try: want=asdict(evaluate_risk(w,RobotState(*data['robot']),tuple(map(tuple,data['path'])),SimpleNamespace(**data['profile']),data.get('speed_limit')))
    except ValueError:
        assert 'error' in probe(data);return
    got=probe(data);assert 'error' not in got,got
    result=got['risk']
    for key in ('blocked','immediate','moving','uncertain'):assert result[key]==want[key],(key,data,want,result)
    for key in ('obstacle_ids','immediate_obstacle_ids'):assert result[key]==list(want[key]),(data,want,result)
    for key in ('clearance_m','conflict_time_s'):
        assert float(result[key])==pytest.approx(want[key],rel=2e-11,abs=3e-12),(data,want,result)
    return result

def box(x,y,variance=0.):return dict(center=[x,y,.5],size=[.15,.2,1.],variance=[variance,variance,variance])
def data(tracks,**values):
    return dict(op='risk',tracks=tracks,profile=PROFILE,robot=[0.,0.,0.,0.,0.,0.],path=[[0.,0.],[3.,0.]],**values)

@pytest.mark.parametrize('uncertain',[False,True])
def test_empty_world_cannot_discard_unassociated(probe,uncertain):
    result=compare(probe,data([],uncertain=uncertain))
    assert result['blocked']==uncertain and result['uncertain']==uncertain
    assert result['immediate'] is False

@pytest.mark.parametrize('include_current',[False,True])
@pytest.mark.parametrize('swept',[False,True])
def test_mixed_rows_preserve_owners_and_duplicate_zero(probe,include_current,swept):
    b=box(1.,2.,.1)
    tracks=[dict(id='static',box=b),dict(id='parametric',box=b,model=dict(velocity=[.2,-.4,0.],variance=.03,steps=[[10**8,.1],[3*10**8,.3]])),
        dict(id='explicit',box=b,predictions=[dict(ns=0,box=b),dict(ns=10**8,box=box(2.,1.,.2)),dict(ns=3*10**8,box=box(0.,0.))]),
        dict(id='empty-model',box=b,model=dict(velocity=[0.,0.,0.],variance=0.,steps=[]))]
    rows=compare(probe,dict(op='rows',tracks=tracks,include_current=include_current,swept=swept))
    assert sum(r['owner']==2 and r['offset_ns']==0 for r in rows)==(2 if include_current else 1)

def test_measured_speed_survives_a_smaller_proposed_cap(probe):
    t=dict(id='wall',box=box(1.6,0.),model=dict(velocity=[0.,0.,0.],variance=0.,steps=[[5*10**8,.5],[10**9,1.],[2*10**9,2.]]))
    d=data([t],speed_limit=.05);d['robot']=[0.,0.,0.,1.,0.,0.]
    result=compare(probe,d)
    assert result['immediate'] is True and result['blocked'] is True

def test_sweep_immediate_owners_are_separate_from_path_owners(probe):
    t=dict(id='side',box=box(0.,.7),model=dict(velocity=[0.,0.,0.],variance=0.,steps=[[10**9,1.]]))
    d=data([t]);d['robot']=[0.,0.,0.,0.,1.,0.];d['path']=[]
    result=compare(probe,d)
    assert result['immediate'] and result['immediate_obstacle_ids']==['side']
    assert result['obstacle_ids']==[] and not result['blocked']

@pytest.mark.parametrize('limit',[0.,-.1,.40001])
def test_invalid_speed_never_becomes_empty_world_clear(probe,limit):compare(probe,data([],speed_limit=limit))

@pytest.mark.parametrize('polygon',[False,True])
def test_random_worlds_and_paths(probe,polygon):
    rng=random.Random(740130)
    for case in range(100):
        tracks=[]
        for i in range(rng.randrange(0,8)):
            b=box(rng.uniform(-3,3),rng.uniform(-3,3),rng.choice([0.,.0001,.01]))
            t=dict(id=f'obstacle-{i}',box=b)
            if i%3==0:t['model']=dict(velocity=[rng.uniform(-1,1),rng.uniform(-1,1),0.],variance=rng.uniform(0,.03),steps=[[10**8,.1],[3*10**8,.3],[7*10**8,.7],[2*10**9,2.]])
            elif i%3==1:t['predictions']=[dict(ns=ns,box=box(rng.uniform(-3,3),rng.uniform(-3,3),rng.uniform(0,.02))) for ns in (0,3*10**8,2*10**9)]
            tracks.append(t)
        d=data(tracks,uncertain=case%7==0)
        d['profile']=dict(PROFILE,**(dict(footprint_xy=FOOTPRINT,half_length_m=.9,half_width_m=.6) if polygon else {}))
        d['robot']=[rng.uniform(-.5,.5),rng.uniform(-.5,.5),rng.uniform(-math.pi,math.pi),rng.uniform(-.8,.8),rng.uniform(-.8,.8),rng.uniform(-1.5,1.5)]
        d['path']=rng.choice([[],[[0.,0.]],[[0.,0.],[0.,0.],[1.,1.],[1.,1.]],[[0.,0.],[3.,0.],[3.,2.]]])
        compare(probe,d)

def test_final_prediction_preserves_zero_velocity_signed_center_and_uncertainty(probe):
    tracks=[dict(id='zero',box=box(-0.,-0.),model=dict(velocity=[0.,0.,0.],variance=.1,steps=[[10**9,1.]])),
        dict(id='moving',box=box(2.,3.,.02),model=dict(velocity=[.2,-.3,.1],variance=.3,steps=[[10**9,1.]])),
        dict(id='empty',box=box(-0.,1.),model=dict(velocity=[0.,0.,0.],variance=.1,steps=[])),
        dict(id='explicit',box=box(1.,2.),predictions=[dict(ns=10**9,box=box(5.,-2.))])]
    d=dict(op='meta',tracks=tracks);w=make_world(d);got=probe(d)
    assert 'error' not in got,got
    for t,out in zip(w.tracks,got['meta']):
        final=final_prediction(t)
        assert out['has_predictions']==has_predictions(t)
        assert out['prediction_count']==prediction_count(t)
        expected=[final.center_m.x,final.center_m.y,final.center_m.z]
        assert out['center']==expected
        assert [math.copysign(1,x) for x in out['center']]==[math.copysign(1,x) for x in expected]
        assert out['covariance']==list(final.position_covariance_m2.values)

@pytest.mark.parametrize('swept',[False,True])
def test_overflowed_predictions_reject_instead_of_clear(probe,swept):
    t=dict(id='overflow',box=box(1.,2.),model=dict(velocity=[1e308,0.,0.],variance=0.,steps=[[10**10,10.]]))
    compare(probe,dict(op='rows',tracks=[t],swept=swept))

@pytest.mark.parametrize('path',[[],[[0.,0.],[3.,0.],[3.,2.]]])
def test_finite_measured_velocity_can_overflow_forecast_distance(probe,path):
    t=dict(id='far',box=box(10.,0.),model=dict(velocity=[0.,0.,0.],variance=0.,steps=[[2*10**9,2.]]))
    d=data([t]);d['robot']=[0.,0.,0.,1e308,1e308,0.];d['path']=path
    result=compare(probe,d)
    assert result['immediate']

@pytest.mark.parametrize('parametric',[False,True])
def test_moving_classification_uses_original_norm_at_point_one_boundary(probe,parametric):
    dx,dy=.05142229122650155,-.08576565725870038
    t=dict(id='moving-boundary',box=box(0.,0.))
    if parametric:t['model']=dict(velocity=[dx,dy,0.],variance=0.,steps=[[10**9,1.]])
    else:t['predictions']=[dict(ns=10**9,box=box(dx,dy))]
    d=data([t]);result=compare(probe,d)
    assert result['moving']
