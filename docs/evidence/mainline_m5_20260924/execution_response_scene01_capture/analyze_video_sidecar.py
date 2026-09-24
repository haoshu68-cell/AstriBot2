"""One-scene offline sidecar audit; no ROS, video decode or re-encode."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'tools/vision'))
from analyze_m5_capture import statistics

SOURCE = Path('/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene01')
OUTPUT = Path(__file__).with_name('video_sidecar_analysis.json')
rows = [json.loads(line) for line in (SOURCE/'first_stage.frames.jsonl').read_text().splitlines()]
video = json.loads((SOURCE/'first_stage.json').read_text())
result = json.loads((SOURCE/'result.json').read_text())
probe = json.loads((SOURCE/'first_stage/result.json').read_text())
preflight = json.loads((SOURCE/'observer_pre_action.json').read_text())
assert [r['frame_index'] for r in rows] == list(range(len(rows)))
assert len(rows) == video['frames'] == result['video_decode']['decoded_frames'] == 307
assert result['video_decode']['passed']
overview = [r['overview'] for r in rows]
heads = [r['head'] for r in rows if r['head'] is not None]
first_head_row = next(r for r in rows if r['head'] is not None)
received = [r['receive_monotonic_ns'] for r in overview]
sources = [r['source_ns'] for r in overview]
receive_gaps = [(b-a)/1e6 for a,b in zip(received,received[1:])]
source_gaps = [(b-a)/1e6 for a,b in zip(sources,sources[1:])]
head_residence = [(r['overview']['receive_monotonic_ns']-r['head']['receive_monotonic_ns'])/1e6
                  for r in rows if r['head'] is not None]
head_ros_age = [(r['overview']['observed_clock']['source_ns']-r['head']['source_ns'])/1e6
                for r in rows if r['head'] is not None and r['overview']['observed_clock'] is not None]
overview_ros_age = [(r['observed_clock']['source_ns']-r['source_ns'])/1e6
                    for r in overview if r['observed_clock'] is not None]
clock_age = [(r['receive_monotonic_ns']-r['observed_clock']['receive_monotonic_ns'])/1e6
             for r in overview if r['observed_clock'] is not None]
encode = [(r['encode_finished_monotonic_ns']-r['encode_started_monotonic_ns'])/1e6 for r in rows]
preencode = [(r['encode_started_monotonic_ns']-r['overview']['receive_monotonic_ns'])/1e6 for r in rows]
assert all(v>=0 for v in head_residence+clock_age+encode+preencode)
by_sequence = {r['receive_seq']:r for r in video['status_events']}
for row in rows:
    if row['status']['receive_seq']:
        assert row['status'] == by_sequence[row['status']['receive_seq']]
    assert row['encode_started_monotonic_ns'] >= row['overview']['receive_monotonic_ns']
    assert row['encode_finished_monotonic_ns'] >= row['encode_started_monotonic_ns']
raw_start,raw_end = result['head_raw']['started_steady'],result['head_raw']['ended_steady']
top_gaps = []
for i in sorted(range(len(receive_gaps)), key=lambda n:receive_gaps[n], reverse=True)[:10]:
    top_gaps.append(dict(from_frame=i,to_frame=i+1,receive_gap_ms=receive_gaps[i],
        source_gap_ms=source_gaps[i],from_source_ns=sources[i],to_source_ns=sources[i+1],
        from_receive_monotonic_ns=received[i],to_receive_monotonic_ns=received[i+1],
        overlaps_raw_bag_process_window=received[i]/1e9<raw_end and received[i+1]/1e9>raw_start))
same_source = same_receipt = 0
for a,b in zip(rows,rows[1:]):
    if a['head'] is not None and b['head'] is not None:
        same_source += a['head']['source_ns']==b['head']['source_ns']
        same_receipt += a['head']['receive_monotonic_ns']==b['head']['receive_monotonic_ns']
data = {
    'schema':'astribot.m5.video_sidecar_analysis/1',
    'scope':'Actual stationary scene metadata audit. No Action was sent; not motion/Hold acceptance.',
    'source_directory':str(SOURCE),
    'source_sha256':{name:hashlib.sha256((SOURCE/name).read_bytes()).hexdigest() for name in
        ['first_stage.frames.jsonl','first_stage.json','result.json','first_stage/result.json',
         'observer_pre_action.json','head_raw_summary.json','m5_observer/manifest.json',
         'post_cleanup_identity_audit.json']},
    'analyzer_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    'video_integrity':dict(sidecar_rows=len(rows),summary_frames=video['frames'],
        owner_decoded_frames=result['video_decode']['decoded_frames'],owner_decode_passed=True,
        independently_decoded_in_this_analysis=False,contiguous_indices=True,
        status_references_exact=True,recorder_returncode=result['recorder_returncode'],
        stop_interpretation='Owner-controlled SIGINT; retained finally output, not normal duration completion'),
    'coverage':dict(overview_first_source_ns=sources[0],overview_last_source_ns=sources[-1],
        source_span_sec=(sources[-1]-sources[0])/1e9,
        first_receive_monotonic_ns=received[0],last_receive_monotonic_ns=received[-1],
        receive_span_sec=(received[-1]-received[0])/1e9,
        recorder_loop_wall_sec=video['wall_duration_s'],encoded_playback_sec=len(rows)/video['encoding_fps'],
        native_status_events=len(video['status_events']),
        native_status_reasons=dict(Counter(r['raw_status']['reason'] for r in video['status_events'])),
        drawn_status_reasons=dict(Counter((r['status']['raw_status'] or {}).get('reason','NO_STATUS_YET') for r in rows)),
        pre_action_receipt_receive_sec=preflight['wall'],probe_error=probe['error'],
        feedback_count=len(probe['feedback_records']),envelope_count=len(probe['envelopes']),
        ack_count=len(probe['acks']),pending_hold_uuid=probe['pending_hold_uuid'],
        action_result='NOT_SENT',actual_motion_validation='NOT_RUN',hold_acceptance='NOT_RUN'),
    'overview':dict(receive_gap_ms=statistics(receive_gaps),source_gap_ms=statistics(source_gaps),
        receive_gaps_over_250ms=sum(g>250 for g in receive_gaps),
        duplicate_source_intervals=sum(g==0 for g in source_gaps),
        source_regressions=sum(g<0 for g in source_gaps),source_age_vs_latest_observed_clock_ms=statistics(overview_ros_age),
        negative_ros_age_samples=sum(x<0 for x in overview_ros_age),
        observed_clock_residence_ms=statistics(clock_age),top_receive_gaps=top_gaps),
    'head_inset':dict(present_frames=len(heads),missing_frames=len(rows)-len(heads),
        first_present_frame_index=first_head_row['frame_index'],
        first_present_overview_source_ns=first_head_row['overview']['source_ns'],
        first_present_after_first_overview_receive_sec=(first_head_row['overview']['receive_monotonic_ns']-received[0])/1e9,
        unique_source_stamps=len({r['source_ns'] for r in heads}),
        consecutive_reused_source_count=same_source,consecutive_reused_receipt_count=same_receipt,
        cache_residence_at_overview_receive_ms=statistics(head_residence),
        source_age_vs_overview_latest_clock_ms=statistics(head_ros_age),
        negative_ros_age_samples=sum(x<0 for x in head_ros_age),
        source_regressions_in_visible_samples=sum(b['source_ns']<a['source_ns'] for a,b in zip(heads,heads[1:])),
        scope='Only head samples selected for inset; not the full head topic, and cache residence is not capture-to-display latency'),
    'recorder_work':dict(encode_call_ms=statistics(encode),receive_to_encode_start_ms=statistics(preencode)),
    'raw_bag':dict(process_elapsed_sec=raw_end-raw_start,returncode=result['head_raw']['returncode'],
        actual_bag_record_span_sec=result['head_raw']['metadata']['rosbag2_bagfile_information']['duration']['nanoseconds']/1e9,
        not_activated=result['head_raw']['not_activated'],missing_topics=result['head_raw']['missing_topics'],
        exact_stamp_tuple_audit='NOT_PERFORMED',scope='Seven baseline topics only; not full M3 source/model input'),
    'limits':['No video re-encode or additional decode was performed; decoded frame count is owner evidence.',
        'Playback time is fixed-fps and not wall time. Receive gaps characterize this callback, not proven rendering/DDS causes.',
        'Latest observed clock can lag; ROS age and steady cache residence are different clocks and meanings.',
        'No action was sent, so no motion-window source coverage or Hold acceptance can be inferred.']}
OUTPUT.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({k:data[k] for k in ['video_integrity','coverage','head_inset','recorder_work']},indent=2,ensure_ascii=False))
print('overview receive gaps',data['overview']['receive_gap_ms'])
print('overview source gaps',data['overview']['source_gap_ms'])
print('overview ROS age',data['overview']['source_age_vs_latest_observed_clock_ms'])
