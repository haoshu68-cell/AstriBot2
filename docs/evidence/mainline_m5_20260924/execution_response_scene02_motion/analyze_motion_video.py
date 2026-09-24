"""Audit original motion video/sidecar; one decode, selected unmodified PNGs, no re-encode."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import cv2

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'tools/vision'))
from analyze_m5_capture import statistics

SOURCE = Path('/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene02')
OUT = Path(__file__).parent
rows = [json.loads(line) for line in (SOURCE/'first_stage.frames.jsonl').read_text().splitlines()]
video = json.loads((SOURCE/'first_stage.json').read_text())
result = json.loads((SOURCE/'result.json').read_text())
probe = json.loads((SOURCE/'first_stage/result.json').read_text())
executing = next(r for r in probe['feedback_records'] if r['feedback']['reason']=='EXECUTING_FIRST_MTC_STAGE')
holding = next(r for r in probe['feedback_records'] if r['feedback']['reason']=='FIRST_STAGE_HOLD_CONFIRMED')
begin,end = executing['wall']*1e9,holding['wall']*1e9
source_begin,source_end = round(executing['ros_s']*1e9),round(holding['ros_s']*1e9)
selected_receive = [r for r in rows if begin<=r['overview']['receive_monotonic_ns']<=end]
selected_source = [r for r in rows if source_begin<=r['overview']['source_ns']<=source_end]
status_by_seq = {r['receive_seq']:r for r in video['status_events']}
for row in rows:
    if row['status']['receive_seq']:
        assert row['status']==status_by_seq[row['status']['receive_seq']]
assert [r['frame_index'] for r in rows]==list(range(len(rows)))

def metrics(frames):
    overview=[r['overview'] for r in frames]
    receives=[r['receive_monotonic_ns'] for r in overview]
    stamps=[r['source_ns'] for r in overview]
    gap=[(b-a)/1e6 for a,b in zip(receives,receives[1:])]
    source_gap=[(b-a)/1e6 for a,b in zip(stamps,stamps[1:])]
    head=[r for r in frames if r['head'] is not None]
    cache=[(r['overview']['receive_monotonic_ns']-r['head']['receive_monotonic_ns'])/1e6 for r in head]
    head_age=[(r['overview']['observed_clock']['source_ns']-r['head']['source_ns'])/1e6
              for r in head if r['overview']['observed_clock'] is not None]
    encode=[(r['encode_finished_monotonic_ns']-r['encode_started_monotonic_ns'])/1e6 for r in frames]
    top=[]
    for i in sorted(range(len(gap)),key=lambda n:gap[n],reverse=True)[:5]:
        top.append(dict(from_frame=frames[i]['frame_index'],to_frame=frames[i+1]['frame_index'],
            receive_gap_ms=gap[i],source_gap_ms=source_gap[i],
            source_start_ns=stamps[i],source_end_ns=stamps[i+1],
            receive_start_ns=receives[i],receive_end_ns=receives[i+1]))
    return dict(frames=len(frames),frame_indices=[r['frame_index'] for r in frames],
        source_range_ns=[stamps[0],stamps[-1]],receive_range_ns=[receives[0],receives[-1]],
        receive_gap_ms=statistics(gap),source_gap_ms=statistics(source_gap),
        receive_gaps_over_250ms=sum(g>250 for g in gap),
        source_duplicates=sum(g==0 for g in source_gap),source_regressions=sum(g<0 for g in source_gap),
        top_receive_gaps=top,head_present_frames=len(head),
        head_cache_residence_ms=statistics(cache),head_age_vs_latest_clock_ms=statistics(head_age),
        head_same_receipt_consecutive=sum(a['head'] is not None and b['head'] is not None and
            a['head']['receive_monotonic_ns']==b['head']['receive_monotonic_ns'] for a,b in zip(frames,frames[1:])),
        visible_status_reasons=dict(Counter((r['status']['raw_status'] or {}).get('reason','NO_STATUS_YET') for r in frames)),
        encode_call_ms=statistics(encode))

keyframes=[]
for target in [source_begin-100_000_000,source_begin+2_000_000_000,source_begin+4_000_000_000,
               source_begin+6_000_000_000,source_end,rows[-1]['overview']['source_ns']]:
    row=min(rows,key=lambda r:abs(r['overview']['source_ns']-target))
    if row['frame_index'] not in [x['frame_index'] for x in keyframes]:
        keyframes.append(dict(frame_index=row['frame_index'],source_ns=row['overview']['source_ns'],
            receive_monotonic_ns=row['overview']['receive_monotonic_ns'],
            head_source_ns=row['head']['source_ns'] if row['head'] else None,
            status=row['status']['raw_status'],file=f"frame_{row['frame_index']:03d}.png"))
key_by_index={r['frame_index']:r for r in keyframes}
reader=cv2.VideoCapture(str(SOURCE/'first_stage.mp4'))
decoded=0
while True:
    ok,picture=reader.read()
    if not ok:break
    if decoded in key_by_index:
        target=OUT/key_by_index[decoded]['file']
        assert cv2.imwrite(str(target),picture)
        key_by_index[decoded]['sha256']=hashlib.sha256(target.read_bytes()).hexdigest()
    decoded+=1
reader.release()
assert decoded==len(rows)==video['frames']==result['video_decode']['decoded_frames']==383
assert all('sha256' in x for x in keyframes)

jtc=[r for r in probe['jtc_records'] if executing['ros_s']<=r['ros_s']<=holding['ros_s']]
position_rows=[r['message']['feedback']['positions'] for r in jtc]
jtc_ranges=[max(v[i] for v in position_rows)-min(v[i] for v in position_rows) for i in range(7)]
data=dict(schema='astribot.m5.motion_video_analysis/1',source_directory=str(SOURCE),
    source_sha256={name:hashlib.sha256((SOURCE/name).read_bytes()).hexdigest() for name in
        ['first_stage.mp4','first_stage.frames.jsonl','first_stage.json','first_stage/result.json','result.json']},
    analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    decode=dict(actual_decoded_frames=decoded,sidecar_rows=len(rows),summary_frames=video['frames'],
        contiguous_indices=True,status_references_exact=True,decode_passes_this_analysis=1,reencode=False,
        recorder_returncode=result['recorder_returncode']),
    stage_window=dict(begin_feedback=executing,end_feedback=holding,
        steady_duration_sec=(end-begin)/1e9,ros_duration_sec=(source_end-source_begin)/1e9,
        same_clock_epoch_assumption='No source regression observed in selected sidecar; consult full observer /clock'),
    whole_video=metrics(rows),during_execution_by_receive=metrics(selected_receive),
    during_execution_by_source=metrics(selected_source),
    boundaries=dict(first_receive_after_execute_delay_ms=(selected_receive[0]['overview']['receive_monotonic_ns']-begin)/1e6,
        last_receive_to_hold_ms=(end-selected_receive[-1]['overview']['receive_monotonic_ns'])/1e6,
        first_source_after_execute_ms=(selected_source[0]['overview']['source_ns']-source_begin)/1e6,
        last_source_before_hold_ms=(source_end-selected_source[-1]['overview']['source_ns'])/1e6),
    jtc=dict(samples=len(jtc),first_source_ros_s=jtc[0]['ros_s'],last_source_ros_s=jtc[-1]['ros_s'],
        actual_position_range_rad=jtc_ranges,max_measured_velocity_rad_s=max(abs(v) for r in jtc for v in r['message']['feedback']['velocities']),
        max_tracking_error_rad=max(abs(v) for r in jtc for v in r['message']['error']['positions']),
        final_max_tracking_error_rad=max(abs(v) for v in jtc[-1]['message']['error']['positions']),
        final_max_abs_velocity_rad_s=max(abs(v) for v in jtc[-1]['message']['feedback']['velocities'])),
    playback=dict(fps=video['encoding_fps'],whole_duration_sec=decoded/video['encoding_fps'],
        recorder_loop_wall_sec=video['wall_duration_s'],meaning='Fixed playback fps; not actual execution duration'),
    keyframes=keyframes,
    acceptance=dict(first_stage_hold_feedback='OBSERVED',complete_pick='NOT_VALIDATED',
        cleanup_complete=probe['cleanup_complete'],resource_disposition=probe['resource_disposition'],
        error=probe['error'],owner_process_teardown_complete=result['cleanup_complete'],
        stop_cancel_release='UNPROVEN; owner teardown is not a release acknowledgement',
        ack_count_in_probe=len(probe['acks'])),
    limits=['Frames sample motion; source/receive gaps remain and no interpolation is used.',
        'Source-window selection and receive-window selection differ; status overlay is latest received state, not capture-synchronized truth.',
        'Head cache residence is receiver cache age, not physical capture-to-display latency.',
        'No complete PICK/place/cancel/release acceptance is inferred from first-stage Hold.'])
(OUT/'motion_video_analysis.json').write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({k:data[k] for k in ['decode','stage_window','during_execution_by_receive','boundaries','jtc','playback','acceptance','keyframes']},ensure_ascii=False,indent=2))
