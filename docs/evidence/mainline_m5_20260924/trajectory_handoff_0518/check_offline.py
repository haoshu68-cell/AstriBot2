#!/usr/bin/env python3
"""Execute the actual verifier block using synthetic journal events; no ROS."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import textwrap

here=Path(__file__).resolve().parent
verifier=Path('/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/verify_full_transfer.py')
source=verifier.read_text();ast.parse(source)
start=source.index('        # Prove returned/adopted/sent suffix identity')
end=source.index("        report['final_stop']=stop()",start)
block=source[start:end]
assert source[:start]+source[end:]==(here/'verify_full_transfer.py.before').read_text(), 'Existing acceptance or cleanup code changed'
code=compile(textwrap.dedent(block),str(verifier)+':trajectory_handoff','exec')
records=[]
for operation,names,physical_stage in [('PICK',('LIFT','TRANSPORT_POSTURE'),'ATTACH_CONFIRM'),('PLACE',('RETREAT','STOW'),'DETACH_CONFIRM')]:
    context='synthetic_parent_op_'+('1_PICK' if operation=='PICK' else '2_PLACE')
    transaction='synthetic_lease:'+operation
    # Prefix active submissions and hold responses must not be mistaken for suffix evidence.
    records.append(dict(event='child_trajectory_submission',details=dict(context=context,transaction='',stage_id='PREGRASP:prefix',stage_index=0,stage_generation=1,controller='arm_left_controller',sent_digest='0'*64,point_count=2,joint_names=['arm_left_joint1'])))
    records.append(dict(event='physical_submission',details=dict(stage=physical_stage,context=context,transaction=transaction)))
    records.append(dict(event='physical_applied_scene_submission',details=dict(transaction=transaction)))
    stages=[]
    for index,name in enumerate(names,4):
        digest=hashlib.sha256((operation+name).encode()).hexdigest()
        stages.append(dict(stage_id=name+':synthetic',stage_index=index,returned_digest=digest,adopted_digest=digest,point_count=3,joint_names=['arm_left_joint1','arm_left_joint2']))
    records.append(dict(event='payload_suffix_adopted',details=dict(context=context,transaction=transaction,start_index=4,transport_replanned=operation=='PICK',stages=stages)))
    records.append(dict(event='payload_transaction_confirmed',details=dict(transaction=transaction)))
    for stage in stages:
        index=stage['stage_index'];child_uuid=hashlib.sha256((context+str(index)).encode()).hexdigest()[:32]
        metadata=dict(context=context,transaction=transaction,stage_id=stage['stage_id'],stage_index=index,stage_generation=index+10,controller='arm_left_controller',sent_digest=stage['adopted_digest'],point_count=stage['point_count'],joint_names=stage['joint_names'])
        records.append(dict(event='child_submission',details=dict(controller='arm_left_controller')))
        records.append(dict(event='child_trajectory_submission',details=metadata))
        children=['arm_left_controller:'+child_uuid]
        for hold in ('arm_right_controller','head_controller','torso_controller','gripper_left_controller','gripper_right_controller'):
            hold_uuid=hashlib.sha256((context+str(index)+hold).encode()).hexdigest()[:32]
            records.append(dict(event='child_response',details=dict(controller=hold,uuid=hold_uuid)))
            records.append(dict(event='child_terminal',details=dict(controller=hold,uuid=hold_uuid,success=True,result_code=4)))
            children.append(hold+':'+hold_uuid)
        records.append(dict(event='child_response',details=dict(metadata,uuid=child_uuid)))
        records.append(dict(event='child_terminal',details=dict(controller='arm_left_controller',uuid=child_uuid,success=True,result_code=4)))
        records.append(dict(event='stage_confirmed',details=dict(context=context,stage_id=stage['stage_id'],index=index,children=';'.join(children)+';')))
(here/'synthetic_journal.json').write_text(json.dumps(dict(scope='Entirely synthetic event contract fixture; no runtime or trajectory computation evidence',events=records),indent=2)+'\n')
cases=[
    ('complete_pick_place',None),
    ('pick_no_replan',None),
    ('missing_adoption','TRAJECTORY_SUFFIX_ADOPTION_REQUIRED:PICK'),
    ('duplicate_adoption','TRAJECTORY_SUFFIX_ADOPTION_REQUIRED:PICK'),
    ('wrong_context','TRAJECTORY_SUFFIX_BINDING_MISMATCH:PICK'),
    ('returned_adopted_mismatch','TRAJECTORY_SUFFIX_CONTENT_MISMATCH:PICK:LIFT'),
    ('sent_digest_mismatch','TRAJECTORY_SENT_CONTENT_MISMATCH:PICK:LIFT'),
    ('point_count_mismatch','TRAJECTORY_SENT_CONTENT_MISMATCH:PICK:LIFT'),
    ('joint_order_mismatch','TRAJECTORY_SENT_CONTENT_MISMATCH:PICK:LIFT'),
    ('wrong_transaction','TRAJECTORY_ACTIVE_SUBMISSION_REQUIRED:PICK:LIFT'),
    ('response_generation_mismatch','TRAJECTORY_RESPONSE_BINDING_MISMATCH:PICK:LIFT'),
    ('terminal_wrong_uuid','TRAJECTORY_COMPLETION_REQUIRED:PICK:LIFT'),
    ('terminal_failed','TRAJECTORY_COMPLETION_BINDING_MISMATCH:PICK:LIFT'),
    ('stage_wrong_uuid','TRAJECTORY_COMPLETION_BINDING_MISMATCH:PICK:LIFT'),
    ('terminal_before_response','TRAJECTORY_COMPLETION_BINDING_MISMATCH:PICK:LIFT'),
    ('missing_place_stage','TRAJECTORY_SUFFIX_BINDING_MISMATCH:PLACE'),
    ('place_replanned','TRAJECTORY_SUFFIX_BINDING_MISMATCH:PLACE'),
]
results=[]
for name,want in cases:
    rows=copy.deepcopy(records)
    adoption=next(r for r in rows if r['event']=='payload_suffix_adopted')
    place=next(r for r in rows if r['event']=='payload_suffix_adopted' and r['details']['context'].endswith('PLACE'))
    send=next(r for r in rows if r['event']=='child_trajectory_submission' and r['details']['stage_index']==4)
    response=next(r for r in rows if r['event']=='child_response' and r['details'].get('stage_index')==4)
    terminal=next(r for r in rows if r['event']=='child_terminal' and r['details']['controller']=='arm_left_controller')
    confirmed=next(r for r in rows if r['event']=='stage_confirmed')
    if name=='pick_no_replan':adoption['details']['transport_replanned']=False
    if name=='missing_adoption':rows.remove(adoption)
    if name=='duplicate_adoption':rows.insert(rows.index(adoption),copy.deepcopy(adoption))
    if name=='wrong_context':adoption['details']['context']='foreign_operation'
    if name=='returned_adopted_mismatch':adoption['details']['stages'][0]['returned_digest']='f'*64
    if name=='sent_digest_mismatch':send['details']['sent_digest']='e'*64
    if name=='point_count_mismatch':send['details']['point_count']+=1
    if name=='joint_order_mismatch':send['details']['joint_names']=list(reversed(send['details']['joint_names']))
    if name=='wrong_transaction':send['details']['transaction']='foreign_transaction'
    if name=='response_generation_mismatch':response['details']['stage_generation']+=1
    if name=='terminal_wrong_uuid':terminal['details']['uuid']='a'*32
    if name=='terminal_failed':terminal['details']['success']=False
    if name=='stage_wrong_uuid':confirmed['details']['children']=confirmed['details']['children'].replace(response['details']['uuid'],'a'*32)
    if name=='terminal_before_response':rows.remove(terminal);rows.insert(rows.index(response),terminal)
    if name=='missing_place_stage':place['details']['stages'].pop()
    if name=='place_replanned':place['details']['transport_replanned']=True
    physical=[]
    for stage in ('ATTACH_CONFIRM','DETACH_CONFIRM'):
        si,submission=next((i,r['details']) for i,r in enumerate(rows) if r['event']=='physical_submission' and r['details']['stage']==stage)
        transaction=submission['transaction']
        ai=next(i for i,r in enumerate(rows) if r['event']=='physical_applied_scene_submission' and r['details']['transaction']==transaction)
        ci=next(i for i,r in enumerate(rows) if r['event']=='payload_transaction_confirmed' and r['details']['transaction']==transaction)
        physical.append(dict(submission=submission,submission_index=si,application_index=ai,confirmation_index=ci))
    env=dict(world_journal=rows,attached=physical[0],detached=physical[1],report={})
    actual=None
    try:exec(code,env)
    except RuntimeError as error:actual=str(error)
    assert actual==want,(name,actual,want)
    if want is None:
        operations=env['report']['trajectory_handoff']['operations']
        assert len(operations)==2 and all(len(p['stages'])==2 for p in operations)
    results.append(dict(name=name,expected_error=want,observed_error=actual,passed=True))
output=dict(scope='Offline execution of production success-path block using synthetic journal only; NOT ROS, simulation, controller execution, or hardware acceptance',verifier_sha256=hashlib.sha256(verifier.read_bytes()).hexdigest(),syntax_parse=True,all_existing_code_unchanged=True,cases=results)
(here/'offline_results.json').write_text(json.dumps(output,indent=2)+'\n')
print(json.dumps(dict(passed=len(results),failed=0,syntax_parse=True,all_existing_code_unchanged=True)))
