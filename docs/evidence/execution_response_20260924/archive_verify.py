#!/usr/bin/env python3
"""Offline archive construction/verification. Reads original evidence; writes only this directory."""
from pathlib import Path
import ast, datetime, hashlib, json, math, shutil, statistics
A=Path(__file__).resolve().parent
E=Path('/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924')
S=E/'execution_response_scene03_world91'
R=Path('/home/yjh/WorkSpace/astribot_sdk_ros2')
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
 return h.hexdigest()
def write(p,d):
 p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
manifest=[]
def cp(p,dest=None,kind='byte_identical_source_copy'):
 q=A/(dest if dest else p.relative_to(E));q.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,q)
 assert sha(q)==sha(p)
 manifest.append(dict(archive=str(q.relative_to(A)),source=str(p),bytes=p.stat().st_size,sha256=sha(p),kind=kind))
for folder in ['observer_precheck_order_fix','ack_contract_fix']:
 for p in sorted((E/folder).iterdir()):
  if p.is_file():cp(p)
for n in ['final_preparation_manifest.json','resolved_runtime_manifest.json','preparation_manifest.json','candidate_identity.json','new_world91/preflight.json','new_world91/run.patch']:cp(E/n)
for p in sorted(S.iterdir()):
 if p.is_file() and (p.name.startswith('used_') or p.name.endswith('_identity.json') or p.name.endswith('_runtime_manifest.json') or p.name in ['result.json','summary.json','owner.json','executor_owner.json','observer_owner.json','cold_start.json','own_lease_journal.json','post_cleanup_identity_audit.json','runtime_manifest.json','actual_world.json','mtc_parameters_pre_action.json','trajectory_capture_manifest.json','query_env.sh']):cp(p)
for n in ['session.json','env.sh','map_manager.yaml','mapping_runtime.yaml','nav_udp.xml','voxel_session_adapter.yaml','payload_ledger.jsonl']:cp(S/'stack'/n)
cp(S/'fixtures/registry.json')
for n in ['execution_response_scene01','execution_response_scene02']:
 cp(E/n/'result.json')
 p=E/n/'first_stage/result.json';d=json.loads(p.read_text());keys=['passed','error','started_wall','finished_wall','parent_terminal','cleanup_complete','cleanup_errors','resource_disposition','cleanup_stop','events','owned_hold_identity','hold_handle_unresolved']
 write(A/n/'first_stage_failure_extract.json',dict(kind='derived_complete_selected_fields',source=str(p),source_sha256=sha(p),json_pointers=['/'+k for k in keys if k in d],data={k:d[k] for k in keys if k in d}))
p=S/'first_stage/result.json';d=json.loads(p.read_text())
keys=['passed','mode','owner','evidence_layer','navigation_goals_sent','task_id','context_id','parent_request','identity_parameters','initial_stop','initial_ledger','executor_binary','mtc_parameters','goal_source','initial_joints','held_joints','stops','scenarios','controller_action_graph','navigation_revocation_ack','first_stage_evidence','parent_terminal','cleanup_complete','cleanup_errors','resource_disposition','cleanup_stop','events','envelopes','acks','commands','motion','feedback_records','owned_hold_identity','hold_handle_unresolved','pending_hold_uuid']
write(A/'execution_response_scene03_world91/first_stage_evidence_extract.json',dict(kind='derived_complete_selected_fields_no_array_truncation',source=str(p),source_sha256=sha(p),source_bytes=p.stat().st_size,json_pointers=['/'+k for k in keys],data={k:d[k] for k in keys}))
# Current read-only helper snapshot is labelled separately: no claim it was independently frozen during execution.
cp(R/'tools/sim/verify_fixed_navigation.py','dependencies_at_archive/verify_fixed_navigation.py','current_dependency_snapshot_not_execution_time_freeze')
cp(R/'runs/mainline_20260924/scene_binding_first_stage/capture_overlay.bash','dependencies_at_archive/capture_overlay.bash','current_dependency_snapshot_not_execution_time_freeze')
cp(R/'ws_robot/src/astribot_s1_description/config/sim_initial_transport_ready.yaml','dependencies_at_archive/sim_initial_transport_ready.yaml','current_dependency_snapshot_not_execution_time_freeze')
j=json.loads((S/'own_lease_journal.json').read_text());terminal=d['parent_terminal'];lease=d['owned_hold_identity'][0]
assert terminal['status']==5 and terminal['result']['resources_released'] and terminal['result']['reason']=='TASK_CANCELED' and terminal['result']['lease_id']==lease
assert j[-1]['event']=='resource_handoff_committed' and j[-1]['phase']==0 and not j[-1]['side_effects'] and j[-1]['lease_id']==lease
assert all(x['lease_id']==lease for x in j)
assert d['resource_disposition']=='RELEASE_CONFIRMED' and d['cleanup_complete'] and not d['cleanup_errors'] and not d['hold_handle_unresolved']
# Recover pre-cancel six-consumer evidence using only ACKs already received at each envelope callback.
cons={'controller','global_costmap','local_costmap','planner','policy','protection'}
cancel_feedback=next(x for x in d['feedback_records'] if x['feedback']['phase']=='4')
accepted=None
for e in d['envelopes']:
 if not e['allowed'] or e['wall']>=cancel_feedback['wall'] or not e['stamp']<=e['ros_s']<e['until']:continue
 latest={}
 for a in d['acks']:
  if a['wall']<=e['wall'] and (a['session'],a['epoch'],a['hash'])==(e['session'],e['epoch'],e['hash']):latest[a['consumer']]=a
 good={k:a for k,a in latest.items() if a['applied'] and 0<=e['wall']-a['wall']<.5 and 0<=e['ros_s']-a['stamp']<.5}
 if cons<=good.keys():accepted=dict(envelope=e,acks={k:good[k] for k in sorted(cons)});break
assert accepted is not None
# Recompute stop metrics from complete saved odometry/final-command rows without ROS.
mod=ast.parse((R/'tools/sim/verify_fixed_navigation.py').read_text());fn=next(x for x in mod.body if isinstance(x,ast.FunctionDef) and x.name=='measured_stop');ns={'math':math,'statistics':statistics};exec(compile(ast.Module(body=[fn],type_ignores=[]),'measured_stop_snapshot','exec'),ns)
stopchecks=[]
for saved in [d['initial_stop'],*d['stops'],d['cleanup_stop']]:
 rows=[r for r in d['motion'] if r['t']<=saved['end_ros_s']+1e-9];tail=rows[-1]
 check=ns['measured_stop'](rows,saved['start_ros_s']-.001,tail['wall'],tail['t'])
 assert check['passed'] and check['samples']==saved['samples']
 stopchecks.append(dict(recorded=saved,recomputed=check,recompute_time_basis='last sample receipt; historical callback-now not separately serialized'))
# Recheck exact PID/start-tick identities. No process control or ROS graph calls.
audit=json.loads((S/'post_cleanup_identity_audit.json').read_text());checks=[]
for a in audit:
 stat=Path('/proc')/str(a['pid'])/'stat';ticks=None
 try:ticks=stat.read_text().rsplit(')',1)[1].split()[19]
 except FileNotFoundError:pass
 live=ticks==str(a['start_ticks']);assert not live
 checks.append(dict(source=a['source'],pid=a['pid'],recorded_start_ticks=a['start_ticks'],current_start_ticks=ticks,same_process_alive=live))
session=json.loads((S/'stack/session.json').read_text());assert session['state']=='stopped' and not session['remaining_owned_pids']
stackpids=[session['supervisor_pid'],*[x['pid'] for x in session['children']]]
stackcheck=[dict(pid=p,pid_exists=Path('/proc',str(p)).exists()) for p in stackpids];assert all(not x['pid_exists'] for x in stackcheck)
fixchecks={}
for n in ['observer_precheck_order_fix/offline_order_check.json','ack_contract_fix/result.json']:
 x=json.loads((E/n).read_text());assert x['passed'];fixchecks[n]=dict(recorded_passed=True,evidence_layer=x.get('evidence_layer',x.get('scope')),rerun=False)
assert sha(E/'ack_contract_fix/after.py')==sha(S/'used_verify_first_stage.py')
prep=json.loads((E/'final_preparation_manifest.json').read_text());sourcechecks=[]
for f,h in prep['source_sha256'].items():
 used=S/('used_'+Path(f).name)
 if used.exists():sourcechecks.append(dict(source=f,frozen=str(used),matches_preparation=sha(used)==h))
assert all(x['matches_preparation'] for x in sourcechecks)
video=S/'first_stage.mp4';write(A/'external_artifacts.json',dict(kind='derived_reference_only',video=dict(path=str(video),sha256=sha(video),bytes=video.stat().st_size,decoded_in_this_audit=False),omitted=['large bags','video bytes','full logs','full source result (selected complete fields retained with original SHA)']))
write(A/'verification.json',dict(kind='derived_independent_offline_audit',checked_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_result_sha256=sha(p),parent_terminal=terminal,journal_last=j[-1],pre_cancel_six_consumers=accepted,first_cancel_feedback=cancel_feedback,ack_total=len(d['acks']),ack_total_is_not_valid_hold_count=True,stop_recomputations=stopchecks,identity_checks=checks,stack_pid_checks=stackcheck,session_stopped=True,source_freeze_checks=sourcechecks,recorded_offline_redgreen=fixchecks,typed_hold_scope='held_joints is JointState, not ArmHoldStatus; no raw typed Hold separately serialized. Corroborated indirectly by coordinator READY_FIXED envelope bound to own hold_id and six ACK plus native hold_confirmed journal, not independent typed observation.',passed=True))
write(A/'copy_manifest.json',dict(kind='derived_copy_manifest',copied_files=manifest,all_copy_hashes_match=True))
(A/'COLCON_IGNORE').write_text('')
print(json.dumps(dict(copied_files=len(manifest),bytes=sum(x['bytes'] for x in manifest),six_ack_epoch=accepted['envelope']['epoch'],six_ack_hash=accepted['envelope']['hash'],identities=len(checks),stop_windows=len(stopchecks),all_passed=True),indent=2))
