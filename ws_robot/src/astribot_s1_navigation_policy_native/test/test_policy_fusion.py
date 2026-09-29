"""Frozen Python differential oracle for the persistent native fusion core.

Run: POLICY_FUSION_PROBE=/tmp/codex_policy_fusion_20260921/policy_fusion_probe pytest -q <this file>
The frozen modules are test-owned byte copies and never installed as production.
"""
import dataclasses
import enum
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

HERE = Path(__file__).resolve().parent
PKG = '_frozen_policy_fusion'
pkg = types.ModuleType(PKG)
pkg.__path__ = [str(HERE / 'reference/policy_fusion')]
sys.modules[PKG] = pkg
for name in ('contracts', 'ports', 'fusion'):
    spec = importlib.util.spec_from_file_location(PKG + '.' + name, HERE / 'reference/policy_fusion' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
C = sys.modules[PKG + '.contracts']
F = sys.modules[PKG + '.fusion']
F._snapshot_tracks_native = None
BASELINE_BINDING = os.environ.get('POLICY_FUSION_BASELINE_GEOMETRY_BINDING')
if BASELINE_BINDING:
    binding_spec = importlib.util.spec_from_file_location('_geometry_native', BASELINE_BINDING)
    binding = importlib.util.module_from_spec(binding_spec)
    binding_spec.loader.exec_module(binding)
    F._snapshot_tracks_native = binding.snapshot_tracks
    os.environ['ASTRIBOT_FUSION_NATIVE_SNAPSHOT'] = '1'
else:
    os.environ.pop('ASTRIBOT_FUSION_NATIVE_SNAPSHOT', None)
PROBE = Path(os.environ.get('POLICY_FUSION_PROBE', '/tmp/codex_policy_fusion_20260921/policy_fusion_probe'))
DEFAULTS = dict(sensor_timeout_s=.5, track_memory_s=1., association_distance_m=.5,
                velocity_confirmation_s=.4, velocity_fit_window_s=1.,
                velocity_fit_max_residual_m=.06, min_tracked_speed_m_s=.1,
                max_obstacle_speed_m_s=2., stationary_velocity_variance_m2_s2=.0004,
                prediction_horizon_s=2.8, prediction_step_s=.2, half_length_m=.31,
                half_width_m=.31, clearance_margin_m=.08, payload_extra_margin_m=0.)


def stamp(ns, clock='ros', epoch=0):
    return dict(ns=ns, clock=clock, epoch=epoch)


def observation(ns, measurement='m', sensor='a', source='track', center=(0., 0., .5), **kw):
    g = dict(kind='MetricBox', center_m=list(center), size_m=[.2, .3, 1.],
             position_covariance_m2=[.01, 0., 0., 0., .02, 0., 0., 0., .03],
             velocity_m_s=None, velocity_covariance_m2_s2=None)
    g.update(kw.pop('geometry', {}))
    d = dict(sensor_id=sensor, measurement_id=measurement, source_track_id=source,
             capture_stamp=stamp(ns), received_at=stamp(ns, 'steady'), valid_until=stamp(ns+500000001),
             frame_id='odom', calibration_epoch=0, geometry=g, geometry_quality=1.,
             class_probabilities=[['unknown', .5]], provenance=[sensor + ':' + measurement],
             velocity_observable=True, spatial_occupancy=False)
    d.update(kw)
    return d


def decode_geometry(g):
    if g['kind'] == 'MetricBox':
        return C.MetricBox(C.Vec3(*g['center_m']), C.Vec3(*g['size_m']), C.Covariance3(tuple(g['position_covariance_m2'])),
                           None if g.get('velocity_m_s') is None else C.Vec3(*g['velocity_m_s']),
                           None if g.get('velocity_covariance_m2_s2') is None else C.Covariance3(tuple(g['velocity_covariance_m2_s2'])))
    if g['kind'] == 'BearingCone':
        return C.BearingCone(C.Vec3(*g['direction']), g['half_angle_rad'])
    return C.ImageBox(**{k:v for k,v in g.items() if k != 'kind'})


def decode_observation(d):
    d = d.copy()
    for key in ('capture_stamp', 'received_at', 'valid_until'):
        d[key] = C.Stamp(**d[key])
    d['geometry'] = decode_geometry(d['geometry'])
    d['class_probabilities'] = tuple(tuple(v) for v in d['class_probabilities'])
    d['provenance'] = tuple(d['provenance'])
    return C.Observation(**d)


def contract(kind, v):
    if kind == 'Observation': return decode_observation(v)
    if kind == 'MetricBox': return decode_geometry(v)
    if kind == 'Covariance3': return C.Covariance3(tuple(v))
    if kind == 'Vec3': return C.Vec3(*v)
    if kind == 'BearingCone': return C.BearingCone(C.Vec3(*v['direction']), v['half_angle_rad'])
    if kind == 'Stamp': return C.Stamp(**v)
    if kind == 'Version': return C.Version(**v)
    if kind == 'MotionLimits': return C.MotionLimits(**v)
    if kind == 'Decision':
        d=v.copy()
        for field,cls in [('version',C.Version),('issued_at',C.Stamp),('valid_until',C.Stamp),('limits',C.MotionLimits)]: d[field]=cls(**d[field])
        for field,cls in [('motion',C.Motion),('planning',C.Planning),('trigger',C.Trigger)]: d[field]=cls(d[field])
        return C.Decision(**d)
    if kind == 'ExecutionContext':
        return C.ExecutionContext(C.Version(**v['version']),C.Stamp(**v['now']),v['required_inputs_valid'],v['motion_enabled'],
                                  frozenset(C.Planning(p) for p in v['allowed_planning']),v['retreat_enabled'],C.MotionLimits(**v['baseline_limits']))
    P=sys.modules[PKG+'.ports']
    if kind == 'Prediction': return P.Prediction(v['offset_ns'],decode_geometry(v['geometry']))
    if kind == 'PredictionModel': return P.PredictionModel(C.Vec3(*v['velocity']),v['variance_m2_s2'],tuple(tuple(t) for t in v['steps']))
    if kind == 'TrackedObstacle':
        return P.TrackedObstacle(v['fused_track_id'],v['frame_id'],C.Stamp(**v['stamp']),decode_geometry(v['geometry']),
            tuple(contract('Prediction', p) for p in v['predictions']),tuple(v['provenance']),
            None if v.get('prediction_model') is None else contract('PredictionModel',v['prediction_model']))
    if kind == 'WorldSnapshot':
        return P.WorldSnapshot(C.Version(**v['version']),C.Stamp(**v['stamp']),v['frame_id'],tuple(contract('TrackedObstacle',t) for t in v['tracks']),
            tuple(decode_observation(o) for o in v['unassociated']),(),v['observation_seq'])
    raise AssertionError(kind)


def encode(value):
    if dataclasses.is_dataclass(value):
        return {f.name: encode(getattr(value, f.name)) for f in dataclasses.fields(value)}
    if isinstance(value, enum.Enum): return value.value
    if isinstance(value, (tuple, list)): return [encode(v) for v in value]
    return value


def reference(case):
    f = F.ConservativeFusion(types.SimpleNamespace(**(DEFAULTS | case.get('profile', {}))), case.get('frame_id', 'odom'))
    out = []
    for op in case['operations']:
        try:
            action = op['action']
            now = C.Stamp(**op['now']) if 'now' in op else None
            result = None
            if action in ('update', 'ingest'):
                result = getattr(f, action)(tuple(decode_observation(d) for d in op['observations']), now)
            elif action == 'snapshot':
                region=op.get('region')
                if 'region_velocity' in op: region=(0.,0.,max(f.profile.max_obstacle_speed_m_s,math.hypot(*op['region_velocity']))*f.profile.prediction_horizon_s)
                result = f.snapshot(now, region)
            elif action == 'resolve': f.resolve_unassociated(op['sensor_id'], op['measurement_ids'], C.Stamp(**op['capture']), now)
            elif action == 'clear':
                if 'free_rule' in op:
                    x,covariance=op['free_rule']
                    f.clear_observed_free(now,free_many=lambda boxes: [b.center_m.x>=x and b.position_covariance_m2.values[0]<covariance for b in boxes])
                else:
                    flags = op['flags']
                    f.clear_observed_free(now, free_many=lambda boxes: flags)
            elif action == 'norm': result=math.hypot(*op['value'])
            elif action == 'check_contract': contract(op['type'],op['value'])
            elif action == 'check_executable': C.check_executable(contract('Decision',op['decision']),contract('ExecutionContext',op['context']))
            elif action == 'check_fresh': decode_observation(op['observation']).check_fresh(now,op['max_age_ns'],op.get('future_tolerance_ns',0))
            elif action == 'set_version': f.version = C.Version(**op['version'])
            elif action == 'set_envelope_bounds':
                f.profile.half_length_m, f.profile.half_width_m = op['bounds']
            elif action == 'validate':
                cls = getattr(C, op['type'])
                if op['type'] == 'Observation': result = decode_observation(op['value'])
                elif op['type'] == 'Covariance3': result = cls(tuple(op['value']))
                elif op['type'] == 'Vec3': result = cls(*op['value'])
                elif op['type'] == 'Stamp': result = cls(**op['value'])
                elif op['type'] == 'MetricBox': result = decode_geometry(op['value'])
            out.append(dict(ok=True, result=encode(result)))
        except C.ContractError as e: out.append(dict(ok=False, code=e.code.value, field=e.field))
    return out


def native(case):
    assert PROBE.is_file(), 'persistent native ConservativeFusion executable has not been implemented'
    r = subprocess.run([str(PROBE)], input=json.dumps(case), text=True, capture_output=True, check=True)
    return json.loads(r.stdout)


def compare(a, b, path=''):
    if isinstance(a, float):
        assert b == pytest.approx(a, rel=2e-12, abs=2e-12), (path, a, b)
        if a == 0. and b == 0.: assert math.copysign(1., a) == math.copysign(1., b), (path, a, b)
    elif isinstance(a, dict):
        assert set(a) == set(b), (path, a, b)
        for k in a: compare(a[k], b[k], path + '.' + k)
    elif isinstance(a, list):
        assert len(a) == len(b), (path, a, b)
        for i,(x,y) in enumerate(zip(a,b)): compare(x,y,path+f'[{i}]')
    else: assert a == b, (path, a, b)


def check(ops, **kw):
    case = dict(operations=ops, **kw)
    compare(reference(case), native(case))


def update(ns, observations, action='update', epoch=0):
    return dict(action=action, now=stamp(ns, epoch=epoch), observations=observations)


def snapshot(ns, **kw): return dict(action='snapshot', now=stamp(ns), **kw)


def test_persistent_motion_static_dedup_and_epoch():
    ops = [update(i*100000000, [observation(i*100000000, str(i), center=(i*.03, 0., .5))]) for i in range(12)]
    ops += [update(1100000000, [observation(1100000000, '11', center=(9., 0., .5))]),
            snapshot(1200000000, region=[100., 100., 0.]), snapshot(2500000000), snapshot(5000000000),
            dict(action='snapshot', now=stamp(0, epoch=1)), update(0, [observation(0, 'epoch', capture_stamp=stamp(0,epoch=1), valid_until=stamp(500000000,epoch=1))], epoch=1)]
    check(ops)


def test_union_provenance_identity_and_source_order():
    obs = [observation(0,'first',center=(0.,0.,.5)), observation(0,'shared',sensor='b',center=(.03,0.,.5),provenance=['a:first']),
           observation(0,'independent',sensor='b',center=(.2,0.,.8)), observation(0,'separate',source='different',center=(0.,0.,.5))]
    check([update(0,obs), update(200000000,[observation(200000000,'later')]),
           update(250000000,[observation(100000000,'out-of-order')]), snapshot(300000000)])


def test_occupancy_unassociated_and_fresh_clear():
    image = dict(kind='ImageBox',camera_id='cam',image_width_px=640,image_height_px=480,xmin_px=1.,ymin_px=2.,xmax_px=8.,ymax_px=9.)
    a = observation(0,'pixel'); a['geometry']=image
    b = observation(0,'cell',source='cell',center=(1.,0.,.5),velocity_observable=False,spatial_occupancy=True)
    c = observation(0,'moving',center=(2.,0.,.5),geometry=dict(velocity_m_s=[.3,0.,0.],velocity_covariance_m2_s2=[.01,0.,0.,0.,.02,0.,0.,0.,.03]))
    check([update(0,[a,b,c]),snapshot(500000000), dict(action='clear',now=stamp(500000001),flags=[True,False]),snapshot(600000000),
           dict(action='resolve',now=stamp(600000000),capture=stamp(100000000),sensor_id='a',measurement_ids=['pixel']),snapshot(600000000),
           dict(action='clear',now=stamp(600000000),flags=[]),snapshot(3000000000)])


def test_transaction_preflight_and_contract_invalid_cases():
    bad = observation(0,'bad',frame_id='map')
    future = observation(100)
    ops=[update(0,[observation(0,'ok'),bad]),snapshot(0),update(0,[future]), update(500000001,[observation(0)]),
         dict(action='validate',type='Covariance3',value=[1.,2.,0.,0.,1.,0.,0.,0.,1.]),
         dict(action='validate',type='Covariance3',value=[1.,2.,0.,2.,1.,0.,0.,0.,1.]),
         dict(action='validate',type='Stamp',value=stamp(-1)),
         dict(action='validate',type='Observation',value=observation(0,geometry_quality=1.1)),
         dict(action='validate',type='Observation',value=observation(0,provenance=['x','x'])),
         dict(action='validate',type='Observation',value=observation(0,velocity_observable=True,spatial_occupancy=True)),
         dict(action='validate',type='Observation',value=observation(0,geometry=dict(velocity_m_s=[0.,0.,0.])))]
    check(ops)


@pytest.mark.parametrize('seed', range(20))
def test_random_stateful_sequences(seed):
    r=random.Random(seed); ops=[]; t=0
    for step in range(65):
        t += r.choice([0,10000000,50000000,100000000])
        observations=[]
        for i in range(r.randrange(5)):
            sensor=r.choice(['lidar','camera']); source=r.choice(['a','b',None]); spatial=r.random()<.15
            observations.append(observation(t-r.choice([0,0,10000000]) if t else 0,str(step)+':'+str(i),sensor,source if not spatial else 'cell',
                center=(r.uniform(-2,2),r.uniform(-2,2),.5),velocity_observable=not spatial,spatial_occupancy=spatial,
                geometry_quality=r.choice([0.,.5,1.]),provenance=[r.choice(['p','q','r'])+str(step)]))
        ops.append(update(t, observations))
        if step%7==0: ops.append(snapshot(t+50000000,region=[r.uniform(-3,3),r.uniform(-3,3),r.uniform(0,3)]))
    ops.append(snapshot(t+5000000000))
    check(ops)


def test_live_envelope_updates_region_without_resetting_state():
    cell = observation(0, 'cell', source='cell', center=(4.,0.,.5), velocity_observable=False, spatial_occupancy=True)
    check([update(0,[cell]),snapshot(0,region=[0.,0.,0.]),
           dict(action='set_envelope_bounds',bounds=[5.,.31]),snapshot(0,region=[0.,0.,0.]),
           dict(action='set_envelope_bounds',bounds=[.31,.31]),snapshot(0,region=[0.,0.,0.])])


def test_association_gate_and_provenance_distance_boundaries():
    for distance in (.05-1e-12,.05,.05+1e-12,.5-1e-12,.5,.5+1e-12):
        check([update(0,[observation(0,'a',source=None),observation(0,'b',sensor='other',source=None,
                    center=(distance,0.,.5),provenance=['a:a'])])])
    for distance in (.6-1e-12,.6,.6+1e-12):
        check([update(0,[observation(0,'a',source=None)]),
               update(500000000,[observation(500000000,'b',source=None,center=(distance,0.,.5))])])


def test_measured_velocity_fitting_residual_speed_cap_and_shape_reset():
    for speed,noise in [(0.,0.),(.1,0.),(.099,0.),(3.,0.),(.2,.25)]:
        ops=[]
        for i in range(8):
            x=i*.1*speed+(noise if i%2 else 0.)
            ops.append(update(i*100000000,[observation(i*100000000,str(i),center=(x,0.,.5))]))
        ops += [snapshot(800000000),update(800000000,[observation(800000000,'reset',center=(8*.1*speed,0.,.5),geometry=dict(size_m=[.8,.3,1.]))]),snapshot(900000000)]
        check(ops)
    measured=dict(velocity_m_s=[9.,-2.,1.],velocity_covariance_m2_s2=[.04,0.,0.,0.,.03,0.,0.,0.,.08])
    check([update(0,[observation(0,'direct',geometry=measured)]),snapshot(100000000),snapshot(1000000001),snapshot(4000000000)])


def test_seen_expiry_is_after_ingest_and_ids_survive_epoch_reset():
    check([update(0,[observation(0,'same')]),update(2100000000,[observation(2100000000,'same',center=(3.,0.,.5))]),
           update(2200000000,[observation(2200000000,'same',center=(3.,0.,.5))]),
           dict(action='snapshot',now=stamp(0,epoch=1)),
           update(0,[observation(0,'same',capture_stamp=stamp(0,epoch=1),valid_until=stamp(500000000,epoch=1))],epoch=1)])


def test_batch_same_sensor_assignments_and_identity_priority():
    start=[observation(0,'0',source='a',center=(0.,0.,.5)),observation(0,'1',source=None,center=(.4,0.,.5))]
    follow=[observation(100000000,'2',source='a',center=(.39,0.,.5)),observation(100000000,'3',source='a',center=(.38,0.,.5))]
    check([update(0,start),update(100000000,follow),snapshot(200000000)])


def test_unknown_order_strict_resolution_clock_mismatch_and_version():
    cone=dict(kind='BearingCone',direction=[1.,0.,0.],half_angle_rad=.5)
    a=observation(0,'a');a['geometry']=cone
    b=observation(0,'b',geometry_quality=0.)
    ops=[update(0,[a,b]),dict(action='resolve',now=stamp(0),capture=stamp(0),sensor_id='a',measurement_ids=['a','b']),snapshot(0),
         dict(action='resolve',now=stamp(1),capture=stamp(1),sensor_id='a',measurement_ids=['a']),snapshot(1),
         dict(action='resolve',now=stamp(500000002),capture=stamp(1),sensor_id='a',measurement_ids=['b']),
         update(10,[observation(10,'bad-clock',capture_stamp=stamp(10,epoch=1),valid_until=stamp(500000000,epoch=1))]),snapshot(10),
         dict(action='set_version',version=dict(goal_id='new',path_revision=7,map_epoch=8,envelope_epoch=9,localization_epoch=10,clock_epoch=11)),
         dict(action='snapshot',now=stamp(10,'other-clock',0))]
    check(ops)


@pytest.mark.parametrize('horizon,step',[(.5,.2),(.7,.2),(1.,.4),(1.25,.5),(1.75,.5),(.000000009,.000000003)])
def test_prediction_rounding_and_large_capture_stamps(horizon,step):
    ns=8000000000000000000
    check([update(ns,[observation(ns)]),snapshot(ns+1)], profile=dict(prediction_horizon_s=horizon,prediction_step_s=step))


def test_boundary_freshness_preflight_preserves_previous_epoch():
    ops=[update(0,[observation(0)]),update(500000000,[observation(0,'at-limit')]),
         update(500000001,[observation(0,'past-limit')]),
         update(499999999,[observation(0,'expiry',valid_until=stamp(499999999))]),snapshot(500000001),
         update(0,[observation(0,'bad-frame',capture_stamp=stamp(0,epoch=1),valid_until=stamp(500000000,epoch=1),frame_id='map')],epoch=1),
         snapshot(600000000)]
    check(ops)


def test_covariance_scaled_psd_and_observation_invalid_contracts():
    matrices=[[0.]*9,[1e200,0.,0.,0.,1e200,0.,0.,0.,1e200],
              [1.,-0.,0.,0.,1.,0.,0.,0.,0.],[-1.,0.,0.,0.,1.,0.,0.,0.,1.],
              [1.,-.9,-.9,-.9,1.,-.9,-.9,-.9,1.],
              [1.,0.,0.,1e-11,1.,0.,0.,0.,1.]]
    ops=[dict(action='validate',type='Covariance3',value=m) for m in matrices]
    for changes in [dict(sensor_id=' '),dict(measurement_id=''),dict(source_track_id='\u3000'),dict(calibration_epoch=-1),
                    dict(received_at=stamp(0,'ros')),dict(valid_until=stamp(0)),dict(class_probabilities=[['x',.6],['y',.5]]),
                    dict(class_probabilities=[['x',.1],['x',.1]]),dict(provenance=[]),dict(geometry=dict(size_m=[0.,1.,1.]))]:
        ops.append(dict(action='validate',type='Observation',value=observation(0,**changes)))
    check(ops)


def test_signed_zero_preserved_for_zero_velocity_snapshot():
    check([update(0,[observation(0,center=(-0.,-0.,-0.))]),snapshot(1000000000),snapshot(4000000000)])


def decision():
    return dict(decision_id='d',episode_id='e',version=dict(goal_id='g',path_revision=1,map_epoch=2,envelope_epoch=3),
                issued_at=stamp(100,'steady'),valid_until=stamp(200,'steady'),motion='CONTINUE',planning='NONE',trigger='NONE',
                limits=dict(linear_speed_m_s=.2,angular_speed_rad_s=.3,linear_accel_m_s2=.4,angular_accel_rad_s2=.5),reason='valid')


def test_typed_decision_and_execution_authority_contracts():
    d=decision()
    ctx=dict(version=d['version'],now=stamp(150,'steady'),required_inputs_valid=True,motion_enabled=True,
             allowed_planning=['LOCAL'],retreat_enabled=False,baseline_limits=d['limits'])
    ops=[dict(action='check_contract',type='Decision',value=d),dict(action='check_executable',decision=d,context=ctx)]
    for changes in [dict(motion='STOP'),dict(motion='HOLD'),dict(motion='RETREAT'),dict(motion='FOLLOW_COMMITTED_PATH'),
                    dict(planning='LOCAL'),dict(planning='LOCAL',trigger='PATH_RISK'),dict(request_id='unexpected'),
                    dict(committed_path_revision=1),dict(issued_at=stamp(100)),dict(valid_until=stamp(100,'steady')),
                    dict(decision_id=''),dict(reason=' '),dict(episode_id='')]:
        ops.append(dict(action='check_contract',type='Decision',value=d|changes))
    for changes in [dict(motion_enabled=False),dict(required_inputs_valid=False),dict(now=stamp(99,'steady')),
                    dict(now=stamp(200,'steady')),dict(version=d['version']|dict(path_revision=2)),
                    dict(baseline_limits=d['limits']|dict(angular_accel_rad_s2=.4)),dict(now=stamp(150,'steady',1))]:
        ops.append(dict(action='check_executable',decision=d,context=ctx|changes))
    planning=d|dict(planning='GLOBAL',trigger='PATH_RISK',request_id='r')
    retreat=d|dict(motion='RETREAT',committed_path_revision=1)
    stop=d|dict(motion='STOP',limits=d['limits']|dict(linear_speed_m_s=0.,angular_speed_rad_s=0.))
    ops += [dict(action='check_executable',decision=planning,context=ctx),dict(action='check_executable',decision=retreat,context=ctx),
            dict(action='check_executable',decision=stop,context=ctx|dict(required_inputs_valid=False))]
    check(ops)


def test_prediction_track_and_world_invariants():
    g=observation(0)['geometry'];p=dict(offset_ns=0,geometry=g)
    m=dict(velocity=[0.,0.,0.],variance_m2_s2=.01,steps=[[100000000,.1],[200000000,.2]])
    t=dict(fused_track_id='track',frame_id='odom',stamp=stamp(0),geometry=g,predictions=[],provenance=['a'],prediction_model=m)
    w=dict(version=decision()['version'],stamp=stamp(0),frame_id='odom',tracks=[t],unassociated=[],observation_seq=0)
    cases=[('Prediction',p),('Prediction',p|dict(offset_ns=-1)),('PredictionModel',m),('PredictionModel',m|dict(variance_m2_s2=-1)),
           ('PredictionModel',m|dict(steps=[[0,0.]])),('PredictionModel',m|dict(steps=[[100000000,.2]])),
           ('PredictionModel',m|dict(steps=[[100000000,.1],[100000000,.1]])),
           ('TrackedObstacle',t),('TrackedObstacle',t|dict(predictions=[p])),('TrackedObstacle',t|dict(prediction_model=None,predictions=[p,p])),
           ('TrackedObstacle',t|dict(provenance=[])),('TrackedObstacle',t|dict(provenance=['a','a'])),
           ('WorldSnapshot',w),('WorldSnapshot',w|dict(tracks=[t,t])),('WorldSnapshot',w|dict(frame_id='map')),
           ('WorldSnapshot',w|dict(tracks=[t|dict(stamp=stamp(1))])),('WorldSnapshot',w|dict(observation_seq=-1)),
           ('BearingCone',dict(direction=[2.,0.,0.],half_angle_rad=.2)),('BearingCone',dict(direction=[1.,0.,0.],half_angle_rad=4.)),
           ('MotionLimits',decision()['limits']|dict(linear_speed_m_s=-1)),('Version',decision()['version']|dict(clock_epoch=-1))]
    check([dict(action='check_contract',type=kind,value=v) for kind,v in cases])


def test_observation_freshness_all_boundaries():
    o=observation(100,valid_until=stamp(200))
    check([dict(action='check_fresh',now=stamp(ns),observation=o,max_age_ns=age,future_tolerance_ns=future)
           for ns,age,future in [(99,100,0),(99,100,1),(100,100,0),(199,99,0),(199,98,0),(200,100,0),(100,-1,0),(100,100,-1)]])


def test_default_snapshot_overflow_contracts_and_conservative_relevance():
    # Existing optional snapshot kernel has stricter exceptional arithmetic than
    # the default Python path. Native persistent core retains the default result.
    original = F._snapshot_tracks_native
    F._snapshot_tracks_native = None
    try:
        moving=observation(0,center=(1e308,0.,.5),geometry=dict(velocity_m_s=[1e308,0.,0.],
            velocity_covariance_m2_s2=[.01,0.,0.,0.,.01,0.,0.,0.,.01]))
        check([update(0,[moving]),snapshot(2000000000)])
        variance=observation(0,geometry=dict(velocity_m_s=[0.,0.,0.],
            velocity_covariance_m2_s2=[1e308,0.,0.,0.,1e308,0.,0.,0.,1e308]))
        check([update(0,[variance]),snapshot(2000000000)])
        enormous=observation(0,center=(1e308,0.,.5),geometry=dict(position_covariance_m2=[1e308,0.,0.,0.,1e308,0.,0.,0.,1e308]))
        check([update(0,[enormous]),snapshot(0,region=[0.,0.,0.])])
        check([update(0,[observation(0)]),snapshot(0,region_velocity=[1e308,0.])])
    finally:
        F._snapshot_tracks_native=original
    if original:
        with pytest.raises(ValueError,match='snapshot region overflow'):
            reference(dict(operations=[update(0,[enormous]),snapshot(0,region=[0.,0.,0.])]))


def test_clear_callback_receives_memory_clamped_box_without_snapshot_variance():
    direct=observation(0,'moving',center=(0.,0.,.5),geometry=dict(velocity_m_s=[.3,0.,0.],velocity_covariance_m2_s2=[.01,0.,0.,0.,.01,0.,0.,0.,.01]))
    cell=observation(0,'cell',center=(.1,0.,.5),source='cell',velocity_observable=False,spatial_occupancy=True)
    check([update(0,[direct,cell]),dict(action='clear',now=stamp(1500000000),free_rule=[.2,.02]),snapshot(1500000000)])


def test_bearing_norm_nextafter_admission_boundaries():
    directions=[[-0.9770489201975666,-0.20631118122899542,0.05303870324268472]]
    rng=random.Random(1946)
    for i in range(8000):
        v=[rng.uniform(-1.,1.) for _ in range(3)];length=math.hypot(*v)
        scale=rng.choice([1.-1e-6,1.+1e-6]);v=[x/length*scale for x in v]
        if i%3:
            k=rng.randrange(3);v[k]=math.nextafter(v[k],rng.choice([-math.inf,math.inf]))
        directions.append(v)
    check([dict(action='check_contract',type='BearingCone',value=dict(direction=v,half_angle_rad=.3)) for v in directions])


def test_exact_norm_association_provenance_and_snapshot_gates(monkeypatch):
    # The old optional native snapshot's libm thresholds differ here. Match the
    # default Python decision exactly, retaining the original gate constants.
    monkeypatch.setattr(F,'_snapshot_tracks_native',None)
    check([update(0,[observation(0,'first',source=None),observation(0,'second',sensor='b',source=None,
           center=(.3653736529718966,.34132403037871817,.5))])])
    check([update(0,[observation(0,'first',source=None),observation(0,'second',sensor='b',source=None,
           center=(-.035354137271507,.03535654080629834,.5),provenance=['a:first'])])])
    dx,dy=.041467329541260464,-.11260755117094283
    positions=[(dx,dy),(-dx,-dy),(dx,dy),(-dx,-dy),(0.,0.)]
    check([update(i*100000000,[observation(i*100000000,str(i),center=(x,y,.5))]) for i,(x,y) in enumerate(positions)])
    far=observation(0,'cell',source='cell',velocity_observable=False,spatial_occupancy=True,
                    center=(.6166499622954075,-.6281969542138451,.5),geometry=dict(position_covariance_m2=[0.]*9))
    check([update(0,[far]),snapshot(0,region=[0.,0.,0.])])


def test_norm_subnormal_and_threshold_values_match_exactly():
    values=[[-9.91903831256525e-310,-5.31992920150296e-310,-1.30473441461663e-310],
            [-4.9444771403072e-310,1.063166560804166e-309],[-6.53683555240816e-310,-3.346451632742e-311],
            [-3.25496464583843e-309,1.605603980145205e-309]]
    rng=random.Random(9814)
    for i in range(3000):
        exponent=rng.randrange(-1074,1023);v=[math.ldexp(rng.uniform(-1.,1.),exponent) for _ in range(rng.choice([2,3]))]
        if math.isfinite(math.hypot(*v)): values.append(v)
    case=dict(operations=[dict(action='norm',value=v) for v in values])
    assert native(case)==reference(case)

@pytest.mark.parametrize('counter',[2**63-1,2**63,2**64-1])
def test_wire_uint64_version_roundtrip(counter):
    value=dict(goal_id='wire',path_revision=counter,map_epoch=counter,envelope_epoch=counter,
               localization_epoch=counter,clock_epoch=counter)
    check([dict(action='check_contract',type='Version',value=value),
           dict(action='set_version',version=value),snapshot(0)])

@pytest.mark.parametrize('field',['path_revision','map_epoch','envelope_epoch','localization_epoch','clock_epoch'])
def test_wire_version_negative_preserves_field_order(field):
    value=dict(goal_id='wire',path_revision=2**64-1,map_epoch=2**64-1,envelope_epoch=2**64-1,
               localization_epoch=2**64-1,clock_epoch=2**64-1)
    value[field]=-1
    bad_goal=dict(value,goal_id='')
    check([dict(action='check_contract',type='Version',value=value),dict(action='check_contract',type='Version',value=bad_goal)])

@pytest.mark.parametrize('dimension',[2**31-1,2**31,2**32-1])
def test_wire_uint32_image_dimensions(dimension):
    obs=observation(0)
    obs['geometry']=dict(kind='ImageBox',camera_id='front',image_width_px=dimension,image_height_px=dimension,
                         xmin_px=0.,ymin_px=0.,xmax_px=float(dimension),ymax_px=float(dimension))
    check([dict(action='validate',type='Observation',value=obs),update(0,[obs]),snapshot(0)])

@pytest.mark.parametrize('counter',[2**63-1,2**63,2**64-1])
@pytest.mark.parametrize('motion',['FOLLOW_COMMITTED_PATH','RETREAT'])
def test_wire_uint64_committed_revision(counter,motion):
    d=decision();d.update(motion=motion,committed_path_revision=counter)
    d['version']['path_revision']=counter
    ctx=dict(version=d['version'],now=stamp(150,'steady'),required_inputs_valid=True,motion_enabled=True,
             allowed_planning=['LOCAL'],retreat_enabled=True,baseline_limits=d['limits'])
    check([dict(action='check_contract',type='Decision',value=d),dict(action='check_executable',decision=d,context=ctx)])

def test_wire_negative_committed_does_not_alias_uint64_max():
    d=decision();d.update(motion='FOLLOW_COMMITTED_PATH',committed_path_revision=-1)
    d['version']['path_revision']=2**64-1
    check([dict(action='check_contract',type='Decision',value=d),
           dict(action='check_contract',type='Decision',value=d|dict(decision_id=''))])

@pytest.mark.parametrize('field',['image_width_px','image_height_px'])
def test_wire_image_negative_dimension_error_order(field):
    obs=observation(0)
    obs['geometry']=dict(kind='ImageBox',camera_id='front',image_width_px=2**32-1,image_height_px=2**32-1,
                         xmin_px=0.,ymin_px=0.,xmax_px=1.,ymax_px=1.)
    obs['geometry'][field]=-1
    check([dict(action='validate',type='Observation',value=obs)])
    obs['geometry']['camera_id']=''
    check([dict(action='validate',type='Observation',value=obs)])

def test_wire_counter_invalid_type_and_goal_error_order():
    ops=[]
    for bad in (True,1.5,-1):
        version=dict(goal_id='wire',path_revision=2**64-1,map_epoch=2**64-1,envelope_epoch=bad,
                     localization_epoch=2**64-1,clock_epoch=2**64-1)
        ops.extend([dict(action='check_contract',type='Version',value=version),
                    dict(action='check_contract',type='Version',value=version|dict(goal_id=''))])
        d=decision();d.update(motion='FOLLOW_COMMITTED_PATH',committed_path_revision=bad)
        d['version']['path_revision']=2**64-1
        ops.extend([dict(action='check_contract',type='Decision',value=d),
                    dict(action='check_contract',type='Decision',value=d|dict(decision_id=''))])
    check(ops)


@pytest.mark.parametrize('epoch', [2**63, 2**64-1, 2**64, 2**64+1, 10**399])
def test_json_integer_calibration_epoch_preserved_in_metric_and_image(epoch):
    image = dict(kind='ImageBox', camera_id='cam', image_width_px=640, image_height_px=480,
                 xmin_px=0., ymin_px=0., xmax_px=1., ymax_px=1.)
    observations = [observation(0, 'metric', calibration_epoch=epoch),
                    dict(observation(0, 'image', sensor='cam', source=None, calibration_epoch=epoch), geometry=image)]
    check([update(0, observations), snapshot(1), snapshot(500000002)])


@pytest.mark.parametrize('width', [2**63, 2**64-1, 2**64, 10**399])
def test_json_integer_image_dimensions_preserved(width):
    image = dict(kind='ImageBox', camera_id='cam', image_width_px=width, image_height_px=width+1,
                 xmin_px=-0.0, ymin_px=0, xmax_px=1, ymax_px=1.0)
    check([update(0, [dict(observation(0, source=None), geometry=image)]), snapshot(1)])


@pytest.mark.parametrize('width,xmin,xmax,expected', [
    (2**64-1, 0, float(2**64), 'image.box.x'),
    (2**64, 0, float(2**64), None),
    (2**64-1, 0, 2**64-1, None),
    (10**399, 0, 10**398, 'xmax_px'),
    (10**399, 0, (1<<1024)-(1<<970)-1, None),
    (10**399, 0, (1<<1024)-(1<<970), 'xmax_px'),
    (10**399, -((1<<1024)-(1<<970)), 1, 'xmin_px'),
    (10**399, -1, 10**398, 'xmin_px'),
    (2**64, float(2**64), 2**64+1, 'image.box.x'),
    (2**64+1, float(2**64), 2**64+1, None),
    (0, 10**398, 10**399, 'image.width'),
])
def test_json_integer_exact_image_pixel_boundaries(width, xmin, xmax, expected):
    image = dict(kind='ImageBox', camera_id='cam', image_width_px=width, image_height_px=10,
                 xmin_px=xmin, ymin_px=0., xmax_px=xmax, ymax_px=1.)
    case = dict(operations=[dict(action='validate', type='Observation', value=dict(observation(0), geometry=image))])
    expected_result = reference(case)
    assert expected_result[0]['ok'] is (expected is None)
    if expected is not None:
        assert expected_result[0]['field'] == expected
    actual = native(case)
    compare(expected_result, actual)
    if expected is None:
        geometry = actual[0]['result']['geometry']
        assert type(geometry['xmin_px']) is type(xmin)
        assert type(geometry['xmax_px']) is type(xmax)


def test_json_integer_image_random_exact_integer_float_edges():
    rng = random.Random(921640)
    ops = []
    for _ in range(500):
        exponent = rng.randrange(0, 1024)
        edge = math.ldexp(rng.uniform(1., 1.999), exponent)
        if not math.isfinite(edge):
            continue
        floor = int(edge)
        width = floor + rng.choice([-1, 0, 1])
        xmax = rng.choice([edge, floor, floor+1])
        image = dict(kind='ImageBox', camera_id='cam', image_width_px=width, image_height_px=2,
                     xmin_px=rng.choice([0., edge, max(0, floor-1)]), ymin_px=0., xmax_px=xmax, ymax_px=1.)
        ops.append(dict(action='validate', type='Observation', value=dict(observation(0), geometry=image)))
    check(ops)
