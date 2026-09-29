"""Differential tests against the frozen pre-migration scalar/NumPy kernels."""
import math
import numpy as np
import pytest
from astribot_s1_robot_geometry._geometry_native import scan_boxes_free, box_distance
from astribot_s1_robot_geometry.polygon import hull, box_distance_many, _box_distance_many_reference
from astribot_s1_navigation_policy.scan_occupancy import angular_box_free


def test_native_hull_preserves_outward_vertices_without_epsilon():
    from astribot_s1_robot_geometry.polygon import _hull_reference
    rng=np.random.default_rng(719)
    for scale in (1e-6,.01,1.,1000.):
        for n in (3,10,256,10000):
            points=rng.normal(size=(n,2))*scale
            points=np.vstack((points,points[::3]))
            np.testing.assert_array_equal(hull(points),_hull_reference(points))
    points=[[0,0],[1,0],[1+1e-12,.5],[1,1],[0,1]]
    np.testing.assert_array_equal(hull(points),_hull_reference(points))
    assert len(hull(points))==5
    for points in ([[0,0],[1,1]],[[0,0],[1,1],[2,2]],[[0,0],[1,1],[math.nan,1]]):
        with pytest.raises(ValueError):hull(points)


def scalar_free(boxes, transform, ranges, lo, hi, amin, inc, resolution):
    tx,ty,_,qx,qy,qz,qw=transform
    result=[]
    for cx,cy,sx,sy in boxes:
        corners=[]
        for dx,dy in ((-1,-1),(-1,1),(1,-1),(1,1)):
            x,y=cx+dx*sx/2,cy+dy*sy/2
            rx,ry,rz=-2*qz*y,2*qz*x,2*(qx*y-qy*x)
            corners.append((x+qw*rx+qy*rz-qz*ry+tx,y+qw*ry+qz*rx-qx*rz+ty))
        result.append(angular_box_free(corners,ranges,lo,hi,amin,inc,resolution))
    return np.asarray(result)


@pytest.mark.parametrize('full',[True,False])
@pytest.mark.parametrize('scan_kind',['free','occluded','unknown','mixed'])
def test_free_space_equivalence(full,scan_kind):
    rng=np.random.default_rng(619)
    n=720 if full else 400;inc=2*math.pi/720
    boxes=np.column_stack((rng.uniform(-6,6,(3000,2)),rng.uniform(.001,.4,(3000,2))))
    boxes=np.vstack((boxes,[[0,0,.05,.05],[-1,0,.05,.05],[.12,0,.05,.05],[9.79,0,.001,.001]]))
    ranges=np.full(n,math.inf)
    if scan_kind=='occluded':ranges[:]=1.
    if scan_kind=='unknown':ranges[:]=math.nan
    if scan_kind=='mixed':
        ranges=rng.uniform(.1,10,n)
        ranges[::7]=math.inf;ranges[::11]=math.nan;ranges[::13]=-math.inf
    for transform in ([0,0,0,0,0,0,1],[.4,-.2,.3,.1,-.2,.3,math.sqrt(.86)]):
        actual=scan_boxes_free(boxes,transform,ranges,.1,10.,-math.pi,inc,.05)
        expected=scalar_free(boxes,transform,ranges,.1,10.,-math.pi,inc,.05)
        np.testing.assert_array_equal(actual,expected)


def test_partial_coverage_and_bracket_occlusion():
    box=[[1.,0.,.05,.05]];tf=[0,0,0,0,0,0,1]
    scan=np.full(720,math.inf);args=(.1,10.,-math.pi,2*math.pi/720,.05)
    assert scan_boxes_free(box,tf,scan,*args)[0]
    scan[357]=1.
    assert not scan_boxes_free(box,tf,scan,*args)[0]
    assert not scan_boxes_free([[-1.,0.,.05,.05]],tf,np.full(360,math.inf),.1,10.,-math.pi/2,math.pi/360,.05)[0]


def test_distance_convex_filled_and_broadcast():
    rng=np.random.default_rng(917)
    for _ in range(40):
        polygon=hull(rng.normal(size=(64,2)))
        pose=rng.uniform(-4,4,(800,3));lo=rng.uniform(-4,4,(800,2))
        hi=lo+rng.uniform(0,1,(800,2));hi[::7]=lo[::7]
        actual=box_distance_many(polygon,*pose.T,lo,hi)
        expected=_box_distance_many_reference(polygon,*pose.T,lo,hi)
        np.testing.assert_allclose(actual,expected,atol=2e-12,rtol=2e-12)
    polygon=np.asarray([[-1,-1],[1,-1],[1,1],[-1,1]])
    assert box_distance_many(polygon,0.,0.,0.,[0.,0.],[.1,.1])<0
    assert box_distance_many(polygon,0.,0.,0.,[1.,0.],[1.,0.])<=0
    result=box_distance_many(polygon,[[0.],[5.]],0.,0.,[[0.,0.],[2.,2.]],[[0.,0.],[2.,2.]])
    assert result.shape==(2,2)
    assert box_distance(polygon,np.empty((0,7))).size==0


@pytest.mark.parametrize('bad',[math.nan,math.inf,-math.inf])
def test_invalid_geometry_rejected(bad):
    with pytest.raises(ValueError):
        scan_boxes_free([[bad,0,.1,.1]],[0,0,0,0,0,0,1],[math.inf]*10,.1,10.,0.,.1,.05)
    with pytest.raises(ValueError):
        box_distance([[-1,-1],[1,-1],[1,1],[-1,1]],[[0,0,bad,0,0,1,1]])


def test_clear_batch_does_not_remove_fresh_occupancy():
    from astribot_s1_navigation_policy.fusion import ConservativeFusion, _Track
    from astribot_s1_navigation_policy.profile import Profile
    from astribot_s1_navigation_policy.contracts import Observation,MetricBox,Vec3,Covariance3,Stamp
    from pathlib import Path
    profile=Profile.load(str(Path(__file__).resolve().parents[2]/'astribot_s1_navigation_policy/config/simulation.json'))
    fusion=ConservativeFusion(profile)
    box=MetricBox(Vec3(1,0,.5),Vec3(.05,.05,1),Covariance3((0.,)*9))
    for identifier,capture in [('stale',1_000_000_000),('fresh',1_900_000_000)]:
        t=Stamp(capture,'sim',0)
        obs=Observation('scan',identifier,identifier,t,Stamp(1,'steady',0),
            Stamp(capture+300_000_000,'sim',0),'odom',0,box,1.,(),(identifier,),
            velocity_observable=False,spatial_occupancy=True)
        fusion.tracks[identifier]=_Track(identifier,obs,Vec3(0,0,0))
    with pytest.raises(ValueError):
        fusion.clear_observed_free(Stamp(2_000_000_000,'sim',0),free_many=lambda boxes: [])
    assert len(fusion.tracks)==2
    fusion.clear_observed_free(Stamp(2_000_000_000,'sim',0),free_many=lambda boxes:[True]*len(boxes))
    assert set(fusion.tracks)=={'fresh'}


def test_cpp_joint_cache_queues_future_partial_frames():
    from astribot_s1_robot_geometry.state import JointSnapshot
    state=JointSnapshot(['a','b'])
    state.receive(['a','b'],[1.,2.],1_000_000_000,1_000_000_000)
    state.receive(['a'],[1.1],1_016_000_000,1_000_000_000)
    state.receive(['b'],[2.1],1_017_000_000,1_000_000_000)
    assert state.snapshot(1_000_000_000)==({'a':1.,'b':2.},[1_000_000_000]*2,1_300_000_000)
    q,times,deadline=state.snapshot(1_016_000_000)
    assert q=={'a':1.1,'b':2.} and times==[1_016_000_000,1_000_000_000] and deadline==1_300_000_000
    assert state.snapshot(1_017_000_000)[0]=={'a':1.1,'b':2.1}
    with pytest.raises(ValueError,match='STALE'):state.snapshot(1_317_000_001)


def test_cpp_joint_future_does_not_extend_lease_and_rollback_clears():
    from astribot_s1_robot_geometry.state import JointSnapshot
    state=JointSnapshot(['a'])
    state.receive(['a'],[1.],1_000_000_000,1_000_000_000)
    state.receive(['a'],[99.],9_000_000_000,1_000_000_000)
    with pytest.raises(ValueError,match='STALE'):state.snapshot(1_300_000_001)
    with pytest.raises(ValueError,match='INCOMPLETE'):state.snapshot(500_000_000)
    assert state.epoch==1
    with pytest.raises(ValueError,match='INCOMPLETE'):state.snapshot(9_000_000_000)


def test_cpp_joint_malformed_source_and_queue_limit():
    from astribot_s1_robot_geometry.state import JointSnapshot
    state=JointSnapshot(['a'])
    with pytest.raises(ValueError,match='source'):state.receive(['a','a'],[1.,2.],1,1)
    with pytest.raises(ValueError,match='source'):state.receive(['a'],[math.nan],1,1)
    for i in range(64):state.receive(['a'],[1.],10_000_000_000+i,1)
    with pytest.raises(ValueError,match='OVERFLOW'):state.receive(['a'],[1.],20_000_000_000,1)
    state.clear()
    with pytest.raises(ValueError,match='INCOMPLETE'):state.snapshot(20_000_000_000)
