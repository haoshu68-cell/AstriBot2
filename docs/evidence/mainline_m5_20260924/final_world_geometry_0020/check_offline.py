#!/usr/bin/env python3
"""Execute only the production final-geometry block; no ROS imports or processes."""
import ast
import copy
import json
from pathlib import Path
import tempfile
import textwrap
from types import SimpleNamespace

here=Path(__file__).resolve().parent
verifier=Path('/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/verify_full_transfer.py')
source_text=verifier.read_text()
ast.parse(source_text)
start=source_text.index('        # Bind retained collision geometry')
end=source_text.index("        report['final_full_scene']",start)
code=compile(textwrap.dedent(source_text[start:end]),str(verifier)+':final_geometry','exec')
fixture=json.loads((here/'scene21_geometry_fixture.json').read_text())
raw=fixture['source_observation']['record']
ledger=fixture['confirmed_attachment_state']['record']
identity=fixture['identity']
binding=identity['lease_epoch']
submission=fixture['journal_link']['physical_submission']['record']
application=fixture['journal_link']['physical_applied_scene_submission']['record']
confirmed=copy.deepcopy(application)
confirmed['event']='payload_transaction_confirmed'
confirmed['details']=dict(transaction=application['details']['transaction'],stage='ATTACH_CONFIRM',command_id=application['details']['command_id'],attachment_revision=ledger['message']['attachment_revision'])
detach_submission=copy.deepcopy(submission)
detach_submission['details'].update(stage='DETACH_CONFIRM',transaction=binding[0]+':payload:2:3')
detach_application=copy.deepcopy(application)
detach_application['details'].update(transaction=detach_submission['details']['transaction'],command_id=application['details']['command_id']+1,source_revision=application['details']['source_revision']+1,source_sequence=application['details']['source_sequence']+100)
detach_confirmed=copy.deepcopy(confirmed)
detach_confirmed['details'].update(transaction=detach_submission['details']['transaction'],stage='DETACH_CONFIRM',command_id=detach_application['details']['command_id'],attachment_revision='offline-synthetic-detached-revision')
records=[submission,application,confirmed,detach_submission,detach_application,detach_confirmed]
final=dict(source_epoch=raw['message']['source_epoch'],clock_epoch=raw['message']['clock_epoch'],source_revision=detach_application['details']['source_revision'],attachment_revision=detach_confirmed['details']['attachment_revision'],ledger_epoch=ledger['message']['ledger_epoch'])
expected=raw['message']['objects'][0]['object']
results=[]
with tempfile.TemporaryDirectory(prefix='m5_final_geometry_') as tmp:
    journal=Path(tmp)/'journal.jsonl'
    for name,want in [('real_conservative_shape_with_synthetic_completed_transactions',None),('dimension_tampered','FINAL_WORLD_TARGET_GEOMETRY_MISMATCH'),('nominal_size_regression','FINAL_WORLD_TARGET_GEOMETRY_MISMATCH'),('source_missing','FINAL_WORLD_BOUND_SOURCE_REQUIRED'),('source_wrong_revision','FINAL_WORLD_BOUND_SOURCE_REQUIRED'),('ledger_wrong_revision','FINAL_WORLD_BOUND_LEDGER_REQUIRED'),('detach_wrong_revision','FINAL_WORLD_PAYLOAD_TRANSITION_MISMATCH')]:
        payload=copy.deepcopy([raw,ledger]);obj=copy.deepcopy(expected);case_records=copy.deepcopy(records)
        obj['header']['frame_id']='astribot_torso_base'
        if name=='dimension_tampered':obj['primitives'][0]['dimensions'][0]+=.001
        if name=='nominal_size_regression':obj['primitives'][0]['dimensions']=[.06,.06,.12]
        if name=='source_missing':payload=[payload[1]]
        if name=='source_wrong_revision':payload[0]['message']['revision']+=1
        if name=='ledger_wrong_revision':payload[1]['message']['observation']['revision']+=1
        if name=='detach_wrong_revision':case_records[4]['details']['source_revision']+=1
        journal.write_text(''.join(json.dumps(r)+'\n' for r in case_records))
        env=dict(journal=journal,journal_start=0,json=json,binding=binding,task_id=identity['task_id'],final=final,payload_records=payload,args=SimpleNamespace(session=raw['message']['session_id'],source=raw['message']['source_id']),goal=SimpleNamespace(object_id=expected['id']),report={},obj=obj)
        actual=None
        try:exec(code,env)
        except RuntimeError as error:actual=str(error)
        assert actual==want,(name,actual,want)
        if want is None:
            assert env['report']['final_world_geometry_source']['independent_pre_detach_scene'] is False
            assert env['report']['final_world_geometry_source']['expected_primitives']==expected['primitives']
        results.append(dict(name=name,expected_error=want,observed_error=actual,passed=True))
(here/'offline_results.json').write_text(json.dumps(dict(scope='Offline production block only; real scene21 geometry plus explicitly synthetic successful ATTACH/DETACH journal confirmations; NOT a successful scene21 or full transfer',cases=results),indent=2)+'\n')
print(json.dumps(dict(passed=len(results),failed=0,syntax_parse=True)))
