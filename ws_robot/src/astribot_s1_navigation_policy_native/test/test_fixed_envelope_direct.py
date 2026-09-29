"""Replay the fixed-posture transaction through Python and a standalone C++ process."""
from pathlib import Path as _ReferencePath
import sys as _reference_sys
_reference_sys.path.insert(0, str(_ReferencePath(__file__).resolve().parents[4] / "tools/migration/python_reference"))
from reference_bootstrap import REFERENCE_ROOT, enable as _enable_references
_enable_references()

import copy
import json
import math
import os
from pathlib import Path
import subprocess

import pytest
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from astribot_navigation_msgs.msg import ArmHoldStatus, EnvelopeApplyStatus, RobotGeometryState
from astribot_navigation_msgs.srv import SetFixedEnvelope
from astribot_s1_navigation_policy.fixed_envelope import FixedEnvelope, CONSUMERS
from astribot_s1_navigation_policy.profile import Profile
from astribot_s1_robot_geometry.node import polygon, stamp
from astribot_s1_robot_geometry.polygon import inflate

PROFILE = Path(__file__).resolve().parents[2] / 'astribot_s1_navigation_policy/config/simulation.json'
NOW = 10_000_000_000


def fixture():
    baseline = Profile.load(PROFILE)
    state = RobotGeometryState()
    state.header.frame_id = baseline.base_frame
    state.header.stamp = stamp(NOW)
    state.valid_until = stamp(NOW + 300_000_000)
    state.source_id = 'source'; state.sequence = 1
    state.model_revision = 'model'; state.attachment_revision = 'empty'
    state.complete = True; state.attachment_state_confirmed = True
    state.joints.name = ['arm']; state.joints.position = [0.0]
    state.joint_source_stamps = [stamp(NOW)]; state.joint_position_error_bounds = [.003]
    state.physical_footprint = polygon([[-.31,-.31],[.31,-.31],[.7,.5],[-.31,.31]])
    state.reserved_footprint = polygon(inflate([(p.x,p.y) for p in state.physical_footprint.points], .02))
    state.height_m = 1.7
    hold = ArmHoldStatus(owner_id='task', hold_id='hold', lease_s=.3,
                        hold_confirmed=True, attachment_revision='empty')
    hold.header.stamp = stamp(NOW)
    request = SetFixedEnvelope.Request(request_id='request', hold_id='hold', geometry_sequence=1)
    request.limits.frame_id = baseline.base_frame
    request.limits.posture_id = 'arbitrary_non_home'; request.limits.lease_s = .3
    from astribot_s1_navigation_policy.robot_envelope import FIELDS
    for field in FIELDS:
        setattr(request.limits, field, getattr(baseline, field))
    return state, hold, request


def event(op, message=None, now=NOW, **kw):
    result = dict(op=op, now=now, **kw)
    if message is not None:
        result['message'] = message_to_ordereddict(message)
    return result


def setup_events():
    state, hold, request = fixture()
    return [event('state',state), event('hold',hold), event('propose',request,stopped=True)]


def snapshot(core, error=''):
    return dict(error=error, epoch=core.epoch, fault=core.fault,
                acks=dict(sorted(core.acks.items())),
                output=message_to_ordereddict(core.output) if core.output is not None else None)


def oracle(events):
    core = FixedEnvelope(Profile.load(PROFILE)); core.session = 'test-session'; core.epoch = 100
    records = []
    kinds = dict(state=RobotGeometryState, hold=ArmHoldStatus, propose=SetFixedEnvelope.Request)
    for step in events:
        op, now = step['op'], step['now']; error = ''
        try:
            if op in kinds:
                message = kinds[op](); set_message_fields(message, copy.deepcopy(step['message']))
                if op == 'state': core.state(message,now)
                elif op == 'hold': core.hold = message
                else: core.propose(message,now,step['stopped'])
            elif op == 'ack':
                current = core.output
                message = EnvelopeApplyStatus(coordinator_session_id=core.session,
                    envelope_epoch=current.epoch if current else 0,
                    installed_geometry_hash=current.installed_geometry_hash if current else '',
                    consumer_id=step['consumer'], applied=True)
                message.header.stamp = stamp(step.get('stamp',now))
                for field,value in step.get('override',{}).items(): setattr(message,field,value)
                core.acknowledge(message,now)
            elif op == 'revoke': core.revoke(step['reason'])
            elif op == 'tick': core.tick(now)
            else: raise AssertionError(op)
        except ValueError as exc:
            error = str(exc)
        records.append(snapshot(core,error))
    return records


def compare(actual, expected, path=''):
    if isinstance(expected,dict):
        assert actual.keys() == expected.keys(), path
        for key in expected: compare(actual[key],expected[key],path+'.'+key)
    elif isinstance(expected,list):
        assert len(actual) == len(expected), path
        for i,(a,e) in enumerate(zip(actual,expected)): compare(a,e,path+f'[{i}]')
    elif isinstance(expected,float):
        assert actual == pytest.approx(expected,rel=1e-13,abs=1e-12), path
    else:
        assert actual == expected, path


def replay(events):
    binary = Path(os.environ.get('FIXED_ENVELOPE_PROBE','/tmp/astribot_fixed_v2_build/fixed_envelope_probe'))
    assert binary.is_file(), 'Build the direct C++ replay executable; bindings cannot satisfy this test'
    result = subprocess.run([str(binary),str(PROFILE)],input=json.dumps(events),
                            capture_output=True,text=True,timeout=15)
    assert result.returncode == 0, result.stderr
    compare(json.loads(result.stdout), oracle(events))


def all_acks(now=NOW):
    return [event('ack',consumer=name,now=now) for name in sorted(CONSUMERS)]


def test_handshake_expiry_and_latched_fault():
    state,hold,_ = fixture()
    changed = copy.deepcopy(state); changed.sequence = 2; changed.joints.position = [.004]
    recovered = copy.deepcopy(state); recovered.sequence = 3
    replay(setup_events()+all_acks()+[event('tick'),event('state',changed),event('tick'),
           event('state',recovered),event('tick'),event('tick',now=NOW-1)])


@pytest.mark.parametrize('consumer',sorted(CONSUMERS))
@pytest.mark.parametrize('override,offset',[
    ({'coordinator_session_id':'wrong'},0),({'envelope_epoch':0},0),
    ({'installed_geometry_hash':'wrong'},0),({'applied':False},0),
    ({},1),({},-500_000_001),({},-500_000_000),({},0)])
def test_ack_identity_time_and_revocation(consumer,override,offset):
    replay(setup_events()+all_acks()+[event('tick'),
        event('ack',consumer=consumer,override=override,stamp=NOW+offset),event('tick')])


@pytest.mark.parametrize('field,value',[
    ('complete',False),('attachment_state_confirmed',False),('source_id',''),
    ('model_revision',''),('attachment_revision',''),('header.frame_id','wrong'),
    ('header.stamp',NOW+1),('valid_until',NOW),('valid_until',NOW+500_000_001),
    ('joints.name',[]),('joints.name',['arm','arm']),('joints.position',[]),
    ('joint_source_stamps',[]),('joint_source_stamps',[NOW-1]),
    ('joint_position_error_bounds',[]),('joint_position_error_bounds',[0.]),
    ('joint_position_error_bounds',[.02500001]),('height_m',0.),
])
def test_state_boundaries(field,value):
    state,hold,request = fixture()
    if field in ('header.stamp','valid_until'): value=stamp(value)
    if field=='joint_source_stamps': value=[stamp(t) for t in value]
    obj=state
    parts=field.split('.')
    for key in parts[:-1]: obj=getattr(obj,key)
    setattr(obj,parts[-1],value)
    replay([event('state',state),event('hold',hold),event('propose',request,stopped=True),event('tick')])


@pytest.mark.parametrize('field,value',[
    ('source_id','restart'),('clock_epoch',1),('model_revision','changed'),
    ('attachment_revision','changed'),('joints.name',['other']),
    ('joints.position',[.003]),('joints.position',[.0030000001]),
])
def test_source_changes_and_hold_error_boundary(field,value):
    state,_,_=fixture();state.sequence=2
    obj=state;parts=field.split('.')
    for key in parts[:-1]: obj=getattr(obj,key)
    setattr(obj,parts[-1],value)
    replay(setup_events()+all_acks()+[event('state',state),event('tick')])


@pytest.mark.parametrize('offset',[0,299_999_999,300_000_000,300_000_001,-1])
def test_source_and_hold_deadline_are_never_renewed(offset):
    replay(setup_events()+all_acks()+[event('tick',now=NOW+offset)])


def test_future_ack_does_not_extend_prior_evidence():
    state,hold,_=fixture();later=NOW+500_000_000
    state.sequence=2;state.header.stamp=stamp(later);state.joint_source_stamps=[stamp(later)]
    state.valid_until=stamp(later+300_000_000);hold.header.stamp=stamp(later)
    replay(setup_events()+all_acks()+[event('ack',consumer='planner',stamp=NOW+1),
        event('state',state,now=later),event('hold',hold,now=later)]+
        [event('ack',consumer=c,now=later) for c in sorted(CONSUMERS-{'planner'})]+[event('tick',now=later)])


def test_history_eviction_duplicate_source_and_explicit_hold():
    state,hold,request=fixture();events=setup_events()+all_acks()
    duplicate=copy.deepcopy(state);duplicate.complete=False;duplicate.reason='duplicate must not revoke'
    old=copy.deepcopy(duplicate);old.sequence=0
    events += [event('state',duplicate),event('state',old),event('tick')]
    for sequence in range(2,11):
        state.sequence=sequence; events.append(event('state',state))
    request.geometry_sequence=1
    events += [event('propose',request,stopped=True),event('revoke',reason='TASK_HOLD'),event('tick')]
    replay(events)


@pytest.mark.parametrize('field,value',[
    ('owner_id',''),('hold_id','wrong'),('attachment_revision','wrong'),('hold_confirmed',False),
    ('lease_s',0.),('lease_s',.50000001),('lease_s',.5)])
def test_hold_owner_lease_and_identity(field,value):
    state,hold,request=fixture();setattr(hold,field,value)
    replay([event('state',state),event('hold',hold),event('propose',request,stopped=True),event('tick')])


@pytest.mark.parametrize('field,value',[
    ('request_id',''),('hold_id',''),('geometry_sequence',0),
    ('limits.posture_id',''),('limits.frame_id','wrong'),('limits.lease_s',0.),
    ('limits.max_speed_m_s',10.),('limits.brake_deceleration_m_s2',10.)])
def test_request_validation_preserved(field,value):
    state,hold,request=fixture();parts=field.split('.');obj=request
    for name in parts[:-1]:obj=getattr(obj,name)
    setattr(obj,parts[-1],value)
    replay([event('state',state),event('hold',hold),event('propose',request,stopped=True),event('tick')])
