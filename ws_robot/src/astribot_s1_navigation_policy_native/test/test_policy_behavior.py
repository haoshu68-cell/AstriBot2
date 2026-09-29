"""Yield state transitions against frozen default Python, including live limits."""
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

HERE=Path(__file__).resolve().parent;REF=HERE/'reference/policy_behavior';NAME='_frozen_policy_behavior'
pkg=types.ModuleType(NAME);pkg.__path__=[str(REF)];sys.modules[NAME]=pkg
os.environ.pop('ASTRIBOT_NAV_NATIVE_KERNELS',None)
spec=importlib.util.spec_from_file_location(NAME+'.behavior',REF/'behavior.py');oracle=importlib.util.module_from_spec(spec);sys.modules[spec.name]=oracle;spec.loader.exec_module(oracle)
BASE=dict(max_speed_m_s=.35,narrow_speed_m_s=.15,reaction_time_s=1.,brake_deceleration_m_s2=.5,linear_stop_delay_s=0.,wait_budget_s=8.,clear_hold_s=.6,angular_brake_deceleration_rad_s2=3.2)

def risk(blocked=False,immediate=False,uncertain=False,conflict_time_s=math.inf):return dict(blocked=blocked,immediate=immediate,uncertain=uncertain,conflict_time_s=conflict_time_s)
def operation(now,r=None,valid=True,**kw):return dict(now=now,risk=r if r is not None else risk(),valid=valid,**kw)
def encoded(x):
    if isinstance(x,float) and not math.isfinite(x):return 'NaN' if math.isnan(x) else ('Infinity' if x>0 else '-Infinity')
    if isinstance(x,dict):return {k:encoded(v) for k,v in x.items()}
    if isinstance(x,list):return [encoded(v) for v in x]
    return x

def check(operations,profile=None):
    p=types.SimpleNamespace(**(BASE if profile is None else profile));selector=oracle.YieldPolicy(p);expected=[]
    for op in operations:
        for k,v in op.get('profile',{}).items():setattr(p,k,v)
        try:value=dataclasses.asdict(selector.select(None if op['risk'] is None else types.SimpleNamespace(**op['risk']),op['valid'],op['now']))
        except Exception as e:value=dict(error=str(e))
        value['state']={k:getattr(selector,k) for k in ('blocked_at','clear_at','last_time','held','episode','in_episode')};expected.append(encoded(value))
    probe=Path(os.environ['POLICY_BEHAVIOR_PROBE']);result=subprocess.run([str(probe)],input=json.dumps(encoded(dict(profile=BASE if profile is None else profile,operations=operations)))+'\n',text=True,capture_output=True,check=True)
    actual=json.loads(result.stdout);assert len(actual)==len(expected)
    for index,(a,b) in enumerate(zip(actual,expected)):assert a==b,(index,operations[index],a,b)

def test_live_envelope_limits_preserve_episode_and_recompute_stopping_gate():
    check([operation(0),operation(.6),operation(.7,risk(blocked=True,conflict_time_s=2.3)),operation(.8,risk(blocked=True,conflict_time_s=2.3),profile=dict(max_speed_m_s=.1,brake_deceleration_m_s2=.1)),operation(.9,risk()),operation(1.5,risk()),operation(1.6,risk())])

def test_clear_hold_wait_budget_clock_rollback_and_invalid_sources():
    ops=[operation(0),operation(math.nextafter(.6,0)),operation(.6),operation(.7,risk(uncertain=True))]
    ops += [operation(t,risk(immediate=True)) for t in (8.7,math.nextafter(8.7,math.inf),10.)]
    ops += [dict(now=10.1,risk=None,valid=True),operation(11.,valid=False),operation(12),operation(12.6),operation(-1),operation(math.nan),operation(math.inf),operation(-.4)]
    check(ops)

@pytest.mark.parametrize('seed',[17,803,20260921])
def test_seeded_persistent_behavior_with_live_limits(seed):
    rng=random.Random(seed);ops=[];now=0.
    for _ in range(2000):
        now+=rng.choice([-.7,0.,.03,.1,.6,8.1]);r=risk(rng.choice([True,False]),rng.random()<.08,rng.random()<.1,rng.choice([0.,1.,2.2,2.3,10.,math.inf,math.nan]))
        op=operation(now,r,rng.random()>.05)
        if rng.random()<.07:op['profile']=dict(max_speed_m_s=rng.choice([0.,.1,.35]),brake_deceleration_m_s2=rng.choice([.1,.5]),linear_stop_delay_s=rng.choice([0.,.13]),narrow_speed_m_s=rng.choice([.05,.15]))
        ops.append(op)
    check(ops)

def test_horizon_rounding_and_zero_speed_keep_python_evaluation_order():
    rng=random.Random(822);ops=[];now=0.
    for _ in range(1000):
        p=dict(BASE,max_speed_m_s=rng.choice([0.,rng.random()]),reaction_time_s=rng.random()*3,brake_deceleration_m_s2=.01+rng.random(),linear_stop_delay_s=rng.random())
        horizon=p['reaction_time_s']+((p['max_speed_m_s']/p['brake_deceleration_m_s2']+p['linear_stop_delay_s']) if p['max_speed_m_s']>0 else 0.)
        for conflict in (math.nextafter(horizon+.5,-math.inf),horizon+.5,math.nextafter(horizon+.5,math.inf)):
            now+=1.;ops.append(operation(now,risk(blocked=True,conflict_time_s=conflict),profile=p))
    check(ops)

def test_frozen_behavior_sources():
    for name,item in json.loads((REF/'manifest.json').read_text())['files'].items():assert hashlib.sha256((REF/(name+'.py')).read_bytes()).hexdigest()==item['sha256']
