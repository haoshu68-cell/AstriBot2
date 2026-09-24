"""Lightweight world91 sidecar audit. Reuses owner decode evidence; never decodes/encodes."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'tools/vision'))
from analyze_m5_capture import statistics

SOURCE=Path('/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene03_world91')
OUT=Path(__file__).parent
rows=[json.loads(line) for line in (SOURCE/'first_stage.frames.jsonl').read_text().splitlines()]
video=json.loads((SOURCE/'first_stage.json').read_text())
result=json.loads((SOURCE/'result.json').read_text())
probe=json.loads((SOURCE/'first_stage/result.json').read_text())
summary=json.loads((SOURCE/'summary.json').read_text())
feedback={reason:next(r for r in probe['feedback_records'] if r['feedback']['reason']==reason)
          for reason in ['EXECUTING_FIRST_MTC_STAGE','FIRST_STAGE_HOLD_CONFIRMED','TASK_CANCELED']}
start=feedback['EXECUTING_FIRST_MTC_STAGE'];hold=feedback['FIRST_STAGE_HOLD_CONFIRMED'];cancel=feedback['TASK_CANCELED']
begin_ns=round(start['wall']*1e9);end_ns=round(hold['wall']*1e9)
begin_source=round(start['ros_s']*1e9);end_source=round(hold['ros_s']*1e9)
in_receive=[r for r in rows if begin_ns<=r['overview']['receive_monotonic_ns']<=end_ns]
in_source=[r for r in rows if begin_source<=r['overview']['source_ns']<=end_source]
assert len(rows)==video['frames']==result['video_decode']['decoded_frames']==summary['video_decode']['decoded_frames']==515
assert result['video_decode']['passed'] and [r['frame_index'] for r in rows]==list(range(len(rows)))
by_sequence={r['receive_seq']:r for r in video['status_events']}
for row in rows:
    if row['status']['receive_seq']:assert row['status']==by_sequence[row['status']['receive_seq']]
    assert row['overview']['receive_monotonic_ns']<=row['encode_started_monotonic_ns']<=row['encode_finished_monotonic_ns']

def metrics(frames):
    received=[r['overview']['receive_monotonic_ns'] for r in frames]
    stamps=[r['overview']['source_ns'] for r in frames]
    gap=[(b-a)/1e6 for a,b in zip(received,received[1:])]
    source_gap=[(b-a)/1e6 for a,b in zip(stamps,stamps[1:])]
    head=[r for r in frames if r['head'] is not None]
    top=[]
    for i in sorted(range(len(gap)),key=lambda n:gap[n],reverse=True)[:5]:
        top.append(dict(from_frame=frames[i]['frame_index'],to_frame=frames[i+1]['frame_index'],
            receive_gap_ms=gap[i],source_gap_ms=source_gap[i],source_endpoints_ns=stamps[i:i+2],
            receive_endpoints_ns=received[i:i+2]))
    return dict(frames=len(frames),frame_indices=[r['frame_index'] for r in frames],
        source_range_ns=[stamps[0],stamps[-1]],receive_range_ns=[received[0],received[-1]],
        source_span_s=(stamps[-1]-stamps[0])/1e9,receive_span_s=(received[-1]-received[0])/1e9,
        receive_gap_ms=statistics(gap),source_gap_ms=statistics(source_gap),
        receive_gaps_over_250ms=sum(x>250 for x in gap),source_duplicates=sum(x==0 for x in source_gap),
        source_regressions=sum(x<0 for x in source_gap),top_receive_gaps=top,
        head_present=len(head),head_missing=len(frames)-len(head),
        head_cache_residence_ms=statistics([(r['overview']['receive_monotonic_ns']-r['head']['receive_monotonic_ns'])/1e6 for r in head]),
        head_source_age_vs_latest_clock_ms=statistics([(r['overview']['observed_clock']['source_ns']-r['head']['source_ns'])/1e6
            for r in head if r['overview']['observed_clock'] is not None]),
        head_consecutive_reused_receipt_count=sum(a['head'] is not None and b['head'] is not None and
            a['head']['receive_monotonic_ns']==b['head']['receive_monotonic_ns'] for a,b in zip(frames,frames[1:])),
        encode_call_ms=statistics([(r['encode_finished_monotonic_ns']-r['encode_started_monotonic_ns'])/1e6 for r in frames]),
        shown_status_reasons=dict(Counter((r['status']['raw_status'] or {}).get('reason','NO_STATUS_YET') for r in frames)))

jtc=[r for r in probe['jtc_records'] if start['ros_s']<=r['ros_s']<=hold['ros_s']]
positions=[r['message']['feedback']['positions'] for r in jtc]
status_changes=[];previous=None
for event in video['status_events']:
    reason=event['raw_status']['reason']
    if reason!=previous:
        status_changes.append(event);previous=reason
data=dict(schema='astribot.m5.world91_video_metadata/1',source_directory=str(SOURCE),
    evidence_scope='New world91 limited first-stage execution/cancel/release; scene02 domain90 failure is unchanged',
    source_sha256={name:hashlib.sha256((SOURCE/name).read_bytes()).hexdigest() for name in
        ['first_stage.frames.jsonl','first_stage.json','first_stage/result.json','result.json','summary.json',
         'used_record_transport_demo.py','post_cleanup_identity_audit.json']},
    analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    original_video=str(SOURCE/'first_stage.mp4'),
    video_integrity=dict(owner_decoded_frames=515,sidecar_rows=len(rows),summary_frames=video['frames'],
        contiguous_indices=True,status_references_exact=True,additional_decode_performed=False,reencode_performed=False,
        recorder_returncode=result['recorder_returncode']),
    timing=dict(first_execution_feedback=start,first_hold_feedback=hold,first_cancel_feedback=cancel,
        execution_steady_duration_s=(end_ns-begin_ns)/1e9,execution_ros_duration_s=(end_source-begin_source)/1e9,
        post_cancel_to_last_frame_s=rows[-1]['overview']['receive_monotonic_ns']/1e9-cancel['wall'],
        recorder_loop_wall_s=video['wall_duration_s'],whole_playback_s=len(rows)/video['encoding_fps'],
        execution_received_frames_playback_s=len(in_receive)/video['encoding_fps'],
        first_receive_after_execution_ms=(in_receive[0]['overview']['receive_monotonic_ns']-begin_ns)/1e6,
        last_receive_before_hold_ms=(end_ns-in_receive[-1]['overview']['receive_monotonic_ns'])/1e6),
    whole=metrics(rows),execution_by_receive=metrics(in_receive),execution_by_source=metrics(in_source),
    status_transitions=status_changes,
    jtc=dict(samples=len(jtc),actual_position_ranges_rad=[max(v[i] for v in positions)-min(v[i] for v in positions) for i in range(7)],
        max_tracking_error_rad=max(abs(v) for r in jtc for v in r['message']['error']['positions']),
        max_actual_velocity_rad_s=max(abs(v) for r in jtc for v in r['message']['feedback']['velocities']),
        last_source_ros_s=jtc[-1]['ros_s'],last_max_error_rad=max(abs(v) for v in jtc[-1]['message']['error']['positions'])),
    control_result=dict(probe_passed=probe['passed'],owner_passed=result['passed'],resource_disposition=probe['resource_disposition'],
        parent_terminal=probe['parent_terminal'],cleanup_complete=probe['cleanup_complete'],cleanup_stop=probe['cleanup_stop'],
        ack_records=len(probe['acks']),limits=summary['limitations']),
    limits=['Owner already decoded full video; this audit independently reads metadata without a second decode.',
        'Source-time and receive-time selection are separate. Status overlay is not synchronized to camera acquisition.',
        'Fixed 10fps playback compresses real gaps. No interpolation or manufactured frames are used.',
        'Cache age is not capture-to-display latency, and video gaps are not raw-depth topic gaps.',
        'Limited first-stage control success is separate from image-gap budgets and full PICK/PLACE or M3 acceptance.'])
(OUT/'video_metadata_analysis.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:data[k] for k in ['video_integrity','timing','execution_by_receive','execution_by_source','jtc']},ensure_ascii=False,indent=2))
