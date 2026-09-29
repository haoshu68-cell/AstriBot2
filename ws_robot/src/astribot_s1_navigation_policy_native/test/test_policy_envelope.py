"""Exact stateful envelope differential against Git 965bf057 (test-only oracle)."""
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import struct
import subprocess
import sys
from types import ModuleType, SimpleNamespace

import pytest

REFERENCE = Path(__file__).parent / 'reference/policy_envelope'
PACKAGE = '_frozen_policy_envelope_20260921'
package = ModuleType(PACKAGE)
package.__path__ = [str(REFERENCE)]
sys.modules[PACKAGE] = package

def module(name):
    spec = importlib.util.spec_from_file_location(PACKAGE + '.' + name, REFERENCE / (name + '.py'))
    result = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = result
    spec.loader.exec_module(result)
    return result

contracts, profile, polygon, oracle = (module(n) for n in ('contracts', 'profile', 'polygon', 'robot_envelope'))
assert polygon._native_hull is None and polygon._native_box_distance is None
BASE = profile.Profile.load(REFERENCE / 'simulation.json')
FIELDS = oracle.FIELDS
STATS = dict(cases=0, operations=0)

def teardown_module():
    if os.environ.get("POLICY_ENVELOPE_STATS"):
        Path(os.environ["POLICY_ENVELOPE_STATS"]).write_text(json.dumps(STATS, indent=2)+"\n")

def number(x):
    return {'nan': math.nan, 'inf': math.inf, '-inf': -math.inf}.get(x, x) if isinstance(x, str) else x

def wire(x):
    return struct.unpack('f', struct.pack('f', number(x)))[0]

def stamp(ns=1_000_000_000, clock='ros', epoch=0):
    return dict(ns=ns, clock=clock, epoch=epoch)

def legacy(ns=1_000_000_000, **kw):
    result = dict(stamp=ns, epoch=1, lease_s=.5, frame_id=BASE.base_frame,
                  posture_id='transport', transport_ready=True, reason='READY',
                  **{k: getattr(BASE, k) for k in FIELDS})
    result.update(kw)
    return result

def points(x):
    return [[wire(v) for v in (p if len(p) == 3 else list(p) + [0.])] for p in x]

def fixed(ns=1_000_000_000, **kw):
    reserved = points([[-.3, -.3], [.3, -.3], [.3, .3], [-.3, .3]])
    installed = points([[-.5, -.5], [.5, -.5], [.5, .5], [-.5, .5]])
    result = dict(header_stamp=ns, valid_until=ns+500_000_000, frame_id=BASE.base_frame,
                  coordinator_session_id='session', request_id='request', hold_id='hold', epoch=1,
                  clock_epoch=0, reference_state_sequence=5, source_state_sequence=6,
                  model_revision='model', attachment_revision='attachment', mode=1,
                  limits=legacy(ns), reserved_footprint=reserved, installed_footprint=installed,
                  clearance_m=.08, navigation_allowed=True, reason='READY')
    result.update(kw)
    for name in ('reserved_footprint', 'installed_footprint'):
        result[name] = points(result[name])
    if 'installed_geometry_hash' not in result:
        result['installed_geometry_hash'] = polygon.geometry_hash(
            [p[:2] for p in result['installed_footprint']], result['frame_id'], result['clearance_m'])
    return result

def msg_stamp(ns):
    sec, nanosec = divmod(ns, 10**9)
    return SimpleNamespace(sec=sec, nanosec=nanosec)

def legacy_msg(d):
    result = {k: number(v) for k, v in d.items()}
    result['stamp'] = msg_stamp(d['stamp'])
    return SimpleNamespace(**result)

def fixed_msg(d):
    result = {k: number(v) for k, v in d.items() if k not in ('header_stamp', 'frame_id')}
    result['header'] = SimpleNamespace(stamp=msg_stamp(d['header_stamp']), frame_id=d['frame_id'])
    result['valid_until'] = msg_stamp(d['valid_until'])
    result['limits'] = legacy_msg(d['limits'])
    result['FIXED_POSTURE'] = 1
    for name in ('reserved_footprint', 'installed_footprint'):
        result[name] = SimpleNamespace(points=[SimpleNamespace(x=wire(p[0]), y=wire(p[1]), z=wire(p[2])) for p in d[name]])
    return SimpleNamespace(**result)

def encoded_number(x):
    return 'nan' if math.isnan(x) else ('inf' if x == math.inf else ('-inf' if x == -math.inf else x))

def state(p, now):
    e = p.envelope
    return dict(ready=bool(p.ready(now)), values={k: getattr(p, k) for k in FIELDS},
                envelope=None if e is None else {**{k: getattr(e, k) for k in FIELDS},
                **{k: getattr(e, k) for k in ('epoch', 'lease_s', 'frame_id', 'posture_id', 'transport_ready', 'reason')},
                'stamp': e.stamp.sec*10**9+e.stamp.nanosec},
                polygon=None if getattr(p, 'footprint_xy', None) is None else p.footprint_xy.tolist(),
                stopping=[encoded_number(p.stopping_distance(number(x))) for x in (0., -.1, .1, 1., 'inf', 'nan')])

def reference(case):
    p = (oracle.FixedEnvelopeProfile if case['fixed'] else oracle.EnvelopeProfile)(BASE)
    alias = 'astribot_s1_robot_geometry.polygon'
    previous = sys.modules.get(alias)
    sys.modules[alias] = polygon
    result = []
    try:
        for op in case['ops']:
            now = contracts.Stamp(**op['now'])
            try:
                if op['op'] == 'accept':
                    value = p.accept((fixed_msg if case['fixed'] else legacy_msg)(op['message']), now)
                elif op['op'] == 'confirm':
                    value = p.confirms_applied(fixed_msg(op['message']), now)
                else:
                    value = p.ready(now)
                row = dict(value=bool(value))
            except (ValueError, AttributeError) as error:
                row = dict(error=str(error))
            row['state'] = state(p, now)
            result.append(row)
    finally:
        if previous is None: sys.modules.pop(alias, None)
        else: sys.modules[alias] = previous
    return result

def check(expected, actual, path=''):
    if isinstance(expected, dict):
        assert expected.keys() == actual.keys(), path
        for k in expected: check(expected[k], actual[k], path+'/'+k)
    elif isinstance(expected, list):
        assert len(expected) == len(actual), path
        for i,(a,b) in enumerate(zip(expected, actual)): check(a,b,path+'/'+str(i))
    elif isinstance(expected, float):
        assert expected == actual, f'{path}: {actual!r} != {expected!r}'
        if expected == 0.: assert math.copysign(1., expected) == math.copysign(1., actual), path
    else:
        assert expected == actual, f'{path}: {actual!r} != {expected!r}'

def run(ops, fixed_mode=False):
    case = dict(fixed=fixed_mode, profile_path=str(REFERENCE / 'simulation.json'), ops=ops)
    binary = os.environ.get('POLICY_ENVELOPE_PROBE')
    assert binary and Path(binary).is_file(), 'native policy envelope probe has not been implemented/built'
    completed = subprocess.run([binary], input=json.dumps(case, allow_nan=False), text=True, capture_output=True, check=True)
    actual = json.loads(completed.stdout)
    STATS['cases'] += 1
    STATS['operations'] += len(ops)
    expected = reference(case)
    try: check(expected, actual)
    except AssertionError:
        if os.environ.get('POLICY_ENVELOPE_FAILURE'):
            Path(os.environ['POLICY_ENVELOPE_FAILURE']).write_text(json.dumps(case, indent=2))
        raise
    return actual

def accept(msg=None, ns=1_000_000_000, **kw):
    return dict(op='accept', message=legacy() if msg is None else msg, now=stamp(ns, **kw))

def query(ns=1_000_000_000, **kw):
    return dict(op='ready', now=stamp(ns, **kw))

def confirm(msg, ns=1_000_000_000, **kw):
    return dict(op='confirm', message=msg, now=stamp(ns, **kw))

def test_frozen_bytes():
    for name, metadata in json.loads((REFERENCE/'manifest.json').read_text())['files'].items():
        assert hashlib.sha256((REFERENCE/name).read_bytes()).hexdigest() == metadata['sha256']

def test_legacy_boundaries_source_receipt_and_clock():
    ops = [query(), accept()]
    ops += [query(n) for n in (999_999_999, 1_000_000_000, 1_499_999_999, 1_500_000_000, 1_500_000_001)]
    ops += [query(epoch=1), query(clock='steady'), accept(legacy(ns=900_000_000), ns=1_000_000_000), query(1_400_000_000), query(1_400_000_001)]
    run(ops)

@pytest.mark.parametrize('ns', [0, 2**53+51, 2_147_483_647_499_999_999])
def test_large_source_nanoseconds(ns):
    run([accept(legacy(ns=ns),ns=ns),query(ns+500_000_000),query(ns+500_000_001)])
    run([accept(fixed(ns=ns),ns=ns),confirm(fixed(ns=ns),ns=ns+300_000_000),query(ns+499_999_999),query(ns+500_000_000)], True)

@pytest.mark.parametrize('field,bad', [('posture_id',''),('frame_id','wrong'),('lease_s',0.),('lease_s','nan'),('lease_s',.5000000000000001)] + [(k,'inf') for k in FIELDS] + [(k,0.) for k in FIELDS if k!='payload_mass_kg'] + [('half_length_m',.1),('max_speed_m_s',.5)])
def test_validation_error_order_and_legacy_preserves(field,bad):
    msg = legacy(**{field:bad})
    run([accept(),accept(msg),query()])
    v = fixed(limits=msg)
    result = run([accept(fixed()),accept(v),query()],True)
    assert result[1]['state']['envelope'] is None and result[1]['state']['polygon'] is not None

def test_legacy_epoch_reorder_and_refresh():
    ops = [accept(),accept(legacy(epoch=0)),accept(legacy(max_speed_m_s=.2)),
           accept(legacy(ns=900_000_000,lease_s=.4,reason='refresh',transport_ready=False)),query(),
           accept(legacy(ns=2_000_000_000,epoch=2)),accept(legacy(ns=0,epoch=2)),
           accept(legacy(epoch=2,max_speed_m_s=.2)),query()]
    run(ops)

def test_validation_numeric_pass_precedes_baseline():
    run([accept(legacy(half_length_m=.1,brake_deceleration_m_s2='nan'))])

@pytest.mark.parametrize('mutate,error', [
    (lambda m:m.update(frame_id='bad'),'invalid V2 frame/mode'),
    (lambda m:m.update(mode=0),'invalid V2 frame/mode'),
    (lambda m:m.update(coordinator_session_id=''),'missing V2 identity'),
    (lambda m:m.update(hold_id=''),'missing V2 identity'),
    (lambda m:m.update(clearance_m=.07999999999999999),'V2 clearance undercut'),
    (lambda m:m.update(clearance_m='inf'),'V2 clearance undercut'),
    (lambda m:m.update(reserved_footprint=[]),'invalid polygon vertices'),
    (lambda m:m.update(installed_geometry_hash='bad'),'V2 hash mismatch'),
    (lambda m:m.update(header_stamp=1_000_000_001),'V2 stale heartbeat'),
    (lambda m:m.update(header_stamp=699_999_999),'V2 stale heartbeat')])
def test_v2_revokes_keeps_footprint(mutate,error):
    bad=fixed();mutate(bad)
    rows=run([accept(fixed()),accept(bad),confirm(fixed()),query()],True)
    assert rows[1]['error']==error
    assert rows[1]['state']['polygon']==rows[0]['state']['polygon']
    assert not rows[2]['value']

def test_v2_limits_deadline_accept_and_ready_are_distinct():
    msgs=[fixed(valid_until=0),fixed(valid_until=9_000_000_000),
          fixed(limits=legacy(ns=0)),fixed(navigation_allowed=False),
          fixed(limits=legacy(transport_ready=False)),fixed(header_stamp=700_000_000)]
    run([op for m in msgs for op in (accept(m),confirm(m),query())],True)

@pytest.mark.parametrize('ns,until,expected',[(700_000_000,1_000_000_001,True),(699_999_999,1_000_000_001,False),
    (1_000_000_001,1_100_000_000,False),(1_000_000_000,1_000_000_000,False),
    (1_000_000_000,1_500_000_000,True),(1_000_000_000,1_500_000_001,False)])
def test_confirmation_deadline_edges(ns,until,expected):
    m=fixed(header_stamp=ns,valid_until=until)
    rows=run([accept(fixed()),confirm(m)],True)
    assert rows[-1]['value'] is expected

def test_confirmation_is_not_accept_or_receipt_refresh():
    later=fixed(ns=1_400_000_000)
    ops=[confirm(fixed()),accept(fixed()),confirm(fixed(),clock='steady'),confirm(fixed(),epoch=1),
         confirm(fixed(),ns=999_999_999),confirm(later,ns=1_400_000_000),query(1_500_000_000),query(1_500_000_001)]
    rows=run(ops,True)
    assert rows[5]['value'] and not rows[6]['value']

@pytest.mark.parametrize('field,new', [('coordinator_session_id','other'),('epoch',2),('clock_epoch',2),
    ('request_id','other'),('hold_id','other'),('reference_state_sequence',7),('model_revision','other'),
    ('attachment_revision','other'),('mode',0),('frame_id','other'),('clearance_m',.081),('installed_geometry_hash','other')])
def test_configuration_identity_outer_fields(field,new):
    m=fixed();m[field]=new
    rows=run([accept(fixed()),confirm(m)],True)
    assert rows[-1]['value'] is False

@pytest.mark.parametrize('field',list(FIELDS)+['epoch','posture_id','frame_id','lease_s'])
def test_configuration_identity_nested_fields(field):
    m=fixed();v=m['limits'][field];m['limits'][field]=v+'new' if isinstance(v,str) else v+.001
    if field=='epoch':m['limits'][field]=2
    assert not run([accept(fixed()),confirm(m)],True)[-1]['value']

@pytest.mark.parametrize('name',['reserved_footprint','installed_footprint'])
@pytest.mark.parametrize('axis',[0,1,2])
def test_configuration_identity_exact_float32_xyz(name,axis):
    m=fixed();m[name][0][axis]=wire(m[name][0][axis]+1e-7)
    assert not run([accept(fixed()),confirm(m)],True)[-1]['value']

def test_configuration_excludes_heartbeat_metadata():
    m=fixed(reason='other',navigation_allowed=False,source_state_sequence=987)
    m['limits'].update(reason='other',transport_ready=False,stamp=1_100_000_000)
    assert run([accept(fixed()),confirm(m)],True)[-1]['value']

def test_v2_epoch_session_order_and_revoke_order():
    old=fixed(epoch=0,header_stamp=0)
    badold=copy.deepcopy(old);badold['installed_geometry_hash']='bad'
    mutation=fixed(limits=legacy(max_speed_m_s=.2))
    allowed=fixed(request_id='new',hold_id='new',limits=legacy(epoch=7,posture_id='new',lease_s=.4))
    rows=run([accept(fixed()),accept(old),confirm(fixed()),accept(allowed),confirm(allowed),
              accept(mutation),accept(fixed()),accept(badold),accept(fixed(epoch=0,coordinator_session_id='new'))],True)
    assert rows[1]['value'] is False and rows[2]['value'] is True
    assert rows[3]['value'] and rows[4]['value']
    assert rows[5]['error']=='V2 mutated epoch'

def test_v2_polygon_validation_and_broadphase_order():
    cases=[]
    for pts in ([],[[0,0],[1,1]],[[0,0],[1,1],[2,2]],
                [[-.3,-.3],[.3,-.3],[0,0],[.3,.3],[-.3,.3]],
                [[-.3,-.3],[.3,.3],[.3,-.3],[-.3,.3]],
                [[-.3,-.3],[.3,-.3],[.3,.3],[-.3,.3],[-.3,-.3]],
                [['nan',-.3],[.3,-.3],[.3,.3],[-.3,.3]]):
        m=fixed();m['reserved_footprint']=[p+[0.] for p in pts];cases.append(m)
    cases.append(fixed(reserved_footprint=[[-.32,-.3],[.32,-.3],[.32,.3],[-.32,.3]]))
    too_small=fixed();too_small['installed_footprint']=too_small['reserved_footprint'];cases.append(too_small)
    run([op for m in cases for op in (accept(fixed()),accept(m))],True)

def test_polygon_clockwise_rotation_canonical_and_signed_zero():
    m=fixed(reserved_footprint=[[-0.,-.3],[.3,-.3],[.3,.3],[-0.,.3]])
    reversed_m=copy.deepcopy(m);reversed_m['reserved_footprint'].reverse()
    rows=run([accept(m),confirm(m),confirm(reversed_m),accept(reversed_m)],True)
    assert rows[1]['value'] and not rows[2]['value']
    changed=copy.deepcopy(m);changed['reserved_footprint'][0][0]=0.
    assert run([accept(m),confirm(changed)],True)[-1]['value']

@pytest.mark.parametrize('seed',[7,29,51,98])
def test_random_state_transitions(seed):
    rng=random.Random(seed);ops=[];ns=1_000_000_000
    for i in range(100):
        ns+=rng.randrange(0,25_000_000)
        m=fixed(ns=ns,epoch=rng.randrange(4),coordinator_session_id=rng.choice(['a','b']))
        choice=rng.randrange(12)
        if choice==0:m['header_stamp']-=300_000_001
        elif choice==1:m['navigation_allowed']=False
        elif choice==2:m['limits']['transport_ready']=False
        elif choice==3:m['limits']['max_speed_m_s']=.2
        elif choice==4:m['valid_until']=ns
        elif choice==5:m['installed_geometry_hash']='bad'
        elif choice==6:m['limits']['stamp']-=500_000_001
        elif choice==7:m['reserved_footprint'][0][2]=wire(rng.random())
        elif choice==8:m['limits']['lease_s']=.1
        ops.extend([accept(m,ns=ns),confirm(m,ns=ns),query(ns+rng.choice([0,100_000_000,500_000_000,500_000_001]))])
    run(ops,True)

def test_subnanosecond_and_rounding_lease_boundaries():
    for lease in [math.nextafter(1e-9, 0.), 1e-9, math.nextafter(1e-9,math.inf),.1000000009]:
        ttl=int(lease*1e9)
        run([accept(legacy(lease_s=lease)),query(1_000_000_000+ttl),query(1_000_000_001+ttl)])

def test_negative_wire_source_and_ros_maximum():
    run([accept(legacy(ns=-1),ns=0),query(499_999_999),query(500_000_000)])
    maximum=2_147_483_647_999_999_999
    m=fixed(ns=maximum-300_000_000,valid_until=maximum)
    run([accept(m,ns=maximum-1),confirm(m,ns=maximum-1),query(maximum),
         query(9_223_372_036_854_775_807)],True)
    run([accept(legacy(ns=-2_147_483_648_000_000_000),ns=9_223_372_036_854_775_807)])

def test_wire_uint64_configuration_identity():
    m=fixed(epoch=2**64-1,clock_epoch=2**64-1,reference_state_sequence=2**64-1,source_state_sequence=2**64-1)
    m['limits']['epoch']=2**64-1
    rows=run([accept(m),confirm(m),accept(fixed()),confirm(m)],True)
    assert rows[0]['value'] and rows[1]['value'] and not rows[2]['value'] and rows[3]['value']

def test_broadphase_exact_equality_and_float32_neighbors():
    x=wire(.310003)
    threshold=x-1e-6
    ops=[]
    for epoch,limit in enumerate([math.nextafter(threshold,0.),threshold,math.nextafter(threshold,math.inf)], 1):
        m=fixed(epoch=epoch,reserved_footprint=[[-x,-.3],[x,-.3],[x,.3],[-x,.3]],limits=legacy(half_length_m=limit))
        ops.extend([accept(m),query()])
    rows=run(ops,True)
    assert rows[0].get('error')=='V2 broad-phase undercut'
    assert rows[-2]['value']

def test_containment_float32_neighbors():
    required=wire(.3)+.08/math.cos(math.pi/32)-2e-6
    center=polygon.np.float32(required)
    ops=[]
    for x in [polygon.np.nextafter(center,polygon.np.float32(-math.inf)),center,
              polygon.np.nextafter(center,polygon.np.float32(math.inf))]:
        m=fixed(installed_footprint=[[-.5,-.5],[float(x),-.5],[float(x),.5],[-.5,.5]])
        ops.append(accept(m))
    rows=run(ops,True)
    assert rows[0].get('error')=='V2 installed footprint undercut'
    assert rows[-1]['value']

def test_hash_collision_still_compares_exact_geometry():
    original=fixed();modified=fixed()
    modified['installed_footprint'][1][0]=wire(.5+1e-7)
    assert polygon.geometry_hash([p[:2] for p in modified['installed_footprint']],BASE.base_frame,.08)==original['installed_geometry_hash']
    rows=run([accept(original),confirm(modified),accept(modified),confirm(modified),confirm(original)],True)
    assert [row['value'] for row in rows]==[True,False,True,True,False]

def test_float32_roundtrip_identity_and_ignored_z_validation():
    m=fixed();rounded=copy.deepcopy(m)
    rounded['reserved_footprint'][0][0]+=1e-10
    assert run([accept(m),confirm(rounded)],True)[-1]['value']
    m['reserved_footprint'][0][2]='nan'
    rows=run([accept(m),confirm(m),query()],True)
    assert rows[0]['value'] and not rows[1]['value'] and rows[2]['value']

def test_wire_vertex_count_bounds():
    ops=[]
    for size in [3,4,32,128,256,257]:
        vertices=[[.3*math.cos(i*2*math.pi/size),.3*math.sin(i*2*math.pi/size)] for i in range(size)]
        m=fixed(reserved_footprint=vertices)
        ops.append(accept(m))
    rows=run(ops,True)
    assert all(r['value'] for r in rows[:-1])
    assert rows[-1]['error']=='invalid polygon vertices'

@pytest.mark.parametrize('seed',[100,207])
def test_random_float32_geometry_and_hashes(seed):
    rng=random.Random(seed);ops=[]
    for i in range(120):
        count=rng.randrange(3,16);radius=rng.uniform(.04,.4);yaw=rng.uniform(-math.pi,math.pi)
        cx,cy=rng.uniform(-.1,.1),rng.uniform(-.1,.1)
        vertices=[[cx+radius*math.cos(j*2*math.pi/count+yaw),cy+radius*math.sin(j*2*math.pi/count+yaw)] for j in range(count)]
        if rng.choice([False,True]): vertices.reverse()
        k=rng.randrange(count);vertices=vertices[k:]+vertices[:k]
        # Both footprints are serialized through float32 before identity/containment.
        m=fixed(epoch=i+1,reserved_footprint=vertices,
                installed_footprint=[[-1.,-1.],[1.,-1.],[1.,1.],[-1.,1.]],
                limits=legacy(half_length_m=.6,half_width_m=.6),clearance_m=rng.uniform(.08,.2))
        ops.extend([accept(m),confirm(m),query()])
    rows=run(ops,True)
    assert all(row['value'] for row in rows)

def test_clock_epoch_switch_and_receipt_rollback_on_accept():
    old=fixed();new=fixed(ns=900_000_000)
    rows=run([accept(old),query(epoch=1),confirm(old,epoch=1),
              accept(new,ns=950_000_000,epoch=1),query(950_000_000,epoch=1),
              confirm(new,ns=950_000_000,epoch=0),confirm(new,ns=950_000_000,epoch=1),
              query(949_999_999,epoch=1)],True)
    assert rows[3]['value'] and rows[4]['value'] and rows[6]['value'] and not rows[7]['value']

def test_mutated_hash_at_same_epoch_versus_new_epoch():
    changed=fixed(installed_footprint=[[-.6,-.6],[.6,-.6],[.6,.6],[-.6,.6]])
    newer=copy.deepcopy(changed);newer['epoch']=2
    rows=run([accept(fixed()),accept(changed),accept(fixed()),accept(newer),confirm(newer)],True)
    assert rows[1]['error']=='V2 mutated epoch' and rows[-1]['value']

def test_signed_zero_payload_preserves_wire_value():
    rows=run([accept(legacy(payload_mass_kg=-0.)),accept(legacy(payload_mass_kg=0.))])
    assert math.copysign(1.,rows[0]['state']['values']['payload_mass_kg'])==-1.
    assert math.copysign(1.,rows[1]['state']['values']['payload_mass_kg'])==1.

def test_hash_signed_zero_identity_is_preserved():
    m=fixed(reserved_footprint=[[.1,-.1],[.2,-.1],[.2,.1],[.1,.1]],
            installed_footprint=[[-0.,-.3],[.4,-.3],[.4,.3],[-0.,.3]])
    other=fixed(reserved_footprint=m['reserved_footprint'],
                installed_footprint=[[0.,-.3],[.4,-.3],[.4,.3],[0.,.3]])
    assert m['installed_geometry_hash']!=other['installed_geometry_hash']
    rows=run([accept(m),confirm(m),confirm(other),accept(other)],True)
    assert rows[0]['value'] and rows[1]['value'] and not rows[2]['value']
    assert rows[-1]['error']=='V2 mutated epoch'
