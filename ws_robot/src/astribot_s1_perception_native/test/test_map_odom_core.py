"""Pure-input differential against a byte-frozen, explicitly loaded oracle."""
import importlib.util
import math
import os
from pathlib import Path
import random
import subprocess
import sys

import pytest

HERE=Path(__file__).resolve().parent
SPEC=importlib.util.spec_from_file_location('map_odom_frozen_math',
    HERE/'reference/astribot_s1_perception/map_odom_decompose.py')
REF=importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name]=REF
SPEC.loader.exec_module(REF)


def probe(lines):
    binary=os.environ.get('MAP_ODOM_CORE_PROBE','/tmp/codex_map_odom_20260921/build/map_odom_core_probe')
    result=subprocess.run([binary],input='\n'.join(lines)+'\n',text=True,
                          capture_output=True,check=True)
    return [line.split() for line in result.stdout.splitlines()]


def n(value): return repr(float(value))


@pytest.mark.parametrize('now,stamp,limit', [
    (2,1,1),(math.nextafter(2,math.inf),1,1),(math.nextafter(2,-math.inf),1,1),
    (0,0,1),(1,-1,1),(1,0,0),(1,100,-1),
    (0.95,1,1),(math.nextafter(.95,math.inf),1,1),(math.nextafter(.95,-math.inf),1,1),
    (1e9,1e9+.05,1),(1e9+1,1e9,1),(1e9+1.0000001,1e9,1),
    (100,1,float('nan')),(0,1,float('nan')),(1,1,float('inf')),
    (float('nan'),1,0),(1,float('inf'),0),(float('inf'),1,1),
])
def test_age_boundaries(now,stamp,limit):
    actual=probe(['age '+ ' '.join(map(n,[now,stamp,limit]))])[0]
    try: expected=REF.check_source_age(now,stamp,limit)
    except REF.DecompositionError:
        assert actual==['error']; return
    assert actual[0]=='age'
    assert float(actual[1])==expected.age_sec
    assert bool(int(actual[2]))==expected.stale


def test_random_composition_sequence_and_stats():
    rng=random.Random(20260921)
    source=REF.MapOdomDecomposer()
    inputs=[]; expected=[]
    for _ in range(600):
        a=REF.Pose2D(*(rng.uniform(-100,100) for _ in range(3)))
        b=REF.Pose2D(*(rng.uniform(-100,100) for _ in range(3)))
        inputs.append('update '+' '.join(map(n,[a.x,a.y,a.theta,b.x,b.y,b.theta])))
        p,jumped,jump=source.update(a,b)
        expected.append([p.x,p.y,p.theta,int(jumped),jump,source.stats.updates,
                         source.stats.jumps,source.stats.max_jump_m])
    for got,want in zip(probe(inputs),expected):
        assert got[0]=='update'
        assert list(map(float,got[1:]))==pytest.approx(want,rel=2e-13,abs=2e-12)


def test_threshold_tilt_and_update_not_rejected():
    lines=['reset .3 .1','tilt .1 0','tilt .10000000000000002 0','tilt nan 0',
           'update 0 0 0 0 0 0','update .3 0 0 0 0 0','update .3 0 1.5 0 0 0',
           'update .7 0 1.5 0 0 0']
    result=probe(lines)
    assert result[1:4]==[['tilt','0','0'],['tilt','1','1'],['tilt','0','1']]
    assert [int(x[4]) for x in result[4:]]==[0,0,0,1]
    assert int(result[-1][6])==4 and int(result[-1][7])==1


@pytest.mark.parametrize('value', [-0.,0.,math.pi,-math.pi,3*math.pi,-3*math.pi,1e100])
def test_wrap_and_quaternion(value):
    rows=probe(['wrap '+n(value),'quat '+n(value)])
    assert float(rows[0][1])==REF.wrap_angle(value)
    assert math.copysign(1,float(rows[0][1]))==math.copysign(1,REF.wrap_angle(value))
    assert tuple(map(float,rows[1][1:]))==REF.quaternion_from_yaw(value)


@pytest.mark.parametrize('z,w', [(0,0),(-0.,1.),(.4,.5),(float('inf'),1),(.1,float('inf'))])
def test_non_normalized_yaw(z,w):
    result=probe(['yaw '+n(z)+' '+n(w)])[0]
    assert float(result[1])==REF.yaw_from_quaternion(z,w)


def test_invalid_inputs_and_extreme_overflow():
    rows=probe(['reset 0 .1','reset nan .1','reset inf nan','tilt 1 1',
                'update nan 0 0 0 0 0','update 1.7e308 1.7e308 .7 1.7e308 1.7e308 .7'])
    assert rows[:3]==[['error'],['error'],['reset']]
    assert rows[3]==['tilt','0','0']
    assert rows[4:]==[['error'],['error']]
