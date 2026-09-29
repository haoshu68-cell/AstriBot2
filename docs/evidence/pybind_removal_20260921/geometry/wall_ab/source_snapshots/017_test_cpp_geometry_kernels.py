"""Differential checks via a test executable, never a runtime Python bridge."""
import json
import os
from pathlib import Path
import subprocess
import numpy as np
import pytest
from astribot_s1_robot_geometry import polygon


@pytest.fixture(scope='module',params=['-O2','-O3'])
def probe(tmp_path_factory,request):
    root=Path(__file__).parents[1]
    executable=tmp_path_factory.mktemp('geometry_cpp')/'probe'
    subprocess.run(['g++','-std=c++17',request.param,'-Wall','-Wextra','-Werror',
                    '-I'+str(root/'include'),str(root/'test/geometry_kernels_probe.cpp'),
                    '-lcrypto','-o',str(executable)],check=True)
    process=subprocess.Popen([str(executable)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    def call(**kwargs):
        process.stdin.write(json.dumps(kwargs)+'\n');process.stdin.flush()
        result=json.loads(process.stdout.readline())
        if 'error' in result:raise ValueError(result['error'])
        return result['value']
    yield call
    process.stdin.close();process.wait(timeout=5)


def test_cpp_polygon_matches_python_oracle(probe):
    rng=np.random.default_rng(125)
    for _ in range(30):
        points=polygon._hull_reference(rng.normal(size=(32,2)))
        np.testing.assert_array_equal(probe(op='hull',points=points.tolist()),points)
        for radius in (0.,1e-6,.03):
            actual=probe(op='inflate',points=points.tolist(),radius=radius)
            expected=polygon.inflate(points,radius)
            assert polygon.contains(actual,expected) and polygon.contains(expected,actual)
        rows=rng.normal(size=(25,7));rows[:,5:7]=rows[:,3:5]+np.abs(rows[:,5:7])
        expected=polygon._box_distance_many_reference(points,*rows[:,:3].T,rows[:,3:5],rows[:,5:7])
        np.testing.assert_allclose(probe(op='distance',points=points.tolist(),rows=rows.tolist()),expected,atol=1e-12)
        assert probe(op='hash',points=points.tolist(),frame='base',clearance=.08)==polygon.geometry_hash(points,'base',.08)


def test_cpp_hash_preserves_json_number_and_negative_zero(probe):
    for value in (1e-6,-0.,1e-5,1e-4,1e6):
        points=[[value,-.3],[value+.5,-.3],[value+.5,.3],[value,.3]]
        assert probe(op='hash',points=points,frame='base',clearance=.00001)==polygon.geometry_hash(points,'base',.00001)


def test_cpp_invalid_polygon_rejects(probe):
    for points in ([[0,0],[1,1],[0,1],[1,0]],[[0,0],[1,0],[.5,.1],[1,1],[0,1]]):
        with pytest.raises(ValueError):probe(op='validate',points=points)


def test_cpp_scan_projection_matches_scalar(probe):
    from test_scan_projection import reference
    rng=np.random.default_rng(19374)
    for _ in range(20):
        transforms=[]
        for _ in range(2):
            q=rng.normal(size=4);q/=np.linalg.norm(q)
            transforms.append(np.r_[rng.uniform(-2,2,3),q].tolist())
        ranges=rng.uniform(.05,20.,723).tolist();theta=rng.uniform(-np.pi,np.pi)
        info=[np.cos(theta),np.sin(theta),-3.,-3.,.05];mask=rng.random((100,110))<.25
        args=(ranges,.1,20.,-np.pi,2*np.pi/723,*transforms,info,mask,.05)
        actual=probe(op='scan_cells',ranges=ranges,lo=.1,hi=20.,start=-np.pi,step=2*np.pi/723,
                     tf=transforms[0],map_tf=transforms[1],info=info,mask=mask.tolist(),resolution=.05)
        np.testing.assert_array_equal(actual,reference(*args))


def test_cpp_free_space_retains_occluded_boxes(probe):
    tf=[0.,0.,0.,0.,0.,0.,1.];ranges=[10.]*720
    args=dict(op='scan_free',boxes=[[2.,0.,.1,.1]],tf=tf,ranges=ranges,lo=.1,hi=10.,start=-np.pi,step=2*np.pi/720,resolution=.05)
    assert probe(**args)==[True]
    ranges[360]=1.
    assert probe(**args)==[False]

def test_cpp_float32_serialization_retains_outward_reserve(probe):
    points=polygon.hull([[-.3,-.3],[.3,-.3],[.8,.2],[.3,.6],[-.3,.3]])
    scale=max(1.,float(np.max(np.abs(points))))
    expected=polygon.hull(np.asarray(polygon.inflate(points,8*np.finfo(np.float32).eps*scale),dtype=np.float32))
    actual=probe(op='serialize',points=points.tolist())
    np.testing.assert_array_equal(actual,expected)

def test_cpp_release_float32_storage_before_canonicalization(probe):
    fixture=json.loads((Path(__file__).parent/'data/float32_serialization.json').read_text())
    points=polygon.inflate(fixture['points'],fixture['clearance'])
    expected=polygon.hull(np.asarray(polygon.inflate(points,8*np.finfo(np.float32).eps),dtype=np.float32))
    actual=probe(op='serialize',points=points.tolist())
    np.testing.assert_array_equal(actual,expected)
