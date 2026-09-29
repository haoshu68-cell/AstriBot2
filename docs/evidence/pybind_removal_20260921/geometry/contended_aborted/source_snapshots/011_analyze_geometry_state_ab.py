#!/usr/bin/env python3
"""Aggregate completed geometry A/B raw records without treating skipped inputs as loss."""
import json
from pathlib import Path
import sys
import numpy as np

root=Path(sys.argv[1]);rows=json.loads((root/'summary.json').read_text())
if len(rows)!=12:raise SystemExit(f'Expected 12 complete rounds; found {len(rows)}')

def percentile(values):
    return dict(zip(('p50','p95','p99'),map(float,np.percentile(values,[50,95,99])))) if values else None

results=[]
for scenario in ('empty','offset_payload'):
    for variant in ('python','cpp'):
        runs=[r for r in rows if r['scenario']==scenario and r['variant']==variant]
        delays=[];receipt_delays=[];interarrival=[];resources=[];frames=[];load=[]
        for run in runs:
            raw=json.loads((root/(run['id']+'.json')).read_text());frames.extend(raw['frames']);resources.extend(raw['resource_samples'])
            inputs={p['source_ns']:p for p in raw['inputs']}
            stamps=[]
            for f in raw['frames']:
                if not f['complete']:continue
                source=f['header']['stamp']['sec']*10**9+f['header']['stamp']['nanosec']
                published=f['published_at']['sec']*10**9+f['published_at']['nanosec']
                assert f['matched_input_sequence']==inputs[source]['sequence']
                delays.append((published-source)/1e6)
                receipt_delays.append((f['receive_monotonic_ns']-inputs[source]['send_monotonic_ns'])/1e6)
                stamps.append(f['receive_monotonic_ns'])
            interarrival.extend((np.diff(stamps)/1e6).tolist())
            load.append({'group':run['group'],'input_count':run['published_input_count'],'input_hz':run['published_input_hz'],
                         'complete_frames':run['complete_frames'],'output_hz':run['complete_output_hz'],'cpu_percent_one_core':run['cpu_percent_one_core'],
                         'cpu_ms_per_complete_frame':1000*run['cpu_seconds']/run['complete_frames'],'rss_peak_mib':run['rss_peak_bytes']/1024**2})
        complete=sum(r['complete_frames'] for r in runs);duration=sum(r['measurement_s'] for r in runs);cpu=sum(r['cpu_seconds'] for r in runs)
        result={'scenario':scenario,'variant':variant,'rounds':len(runs),'measurement_s':duration,'complete_frames':complete,
                'incomplete_frames':sum(r['incomplete_frames'] for r in runs),'output_sequence_gaps':sum(r['output_sequence_gaps'] for r in runs),
                'validation_failure_count':sum(len(r['validation_failures']) for r in runs),'exact_float32_wire_frames':sum(r['exact_float32_wire_frames'] for r in runs),
                'matched_original_source_frames':sum(r['source_matched_complete_frames'] for r in runs),
                'cpu_percent_one_core':100*cpu/duration,'cpu_ms_per_complete_frame':1000*cpu/complete,
                'cpu_percent_per_run':[r['cpu_percent_one_core'] for r in runs],
                'rss_mib':percentile([r['rss_bytes']/1024**2 for r in resources]),
                'complete_output_hz':complete/duration,'output_interarrival_ms':percentile(interarrival),'maximum_output_interarrival_ms':max(interarrival),
                'source_to_publish_ms':percentile(delays),'source_to_receive_ms':percentile(receipt_delays),
                'performance_comparison_eligible':all(r['performance_comparison_eligible'] for r in runs),'runs':load}
        results.append(result)
comparisons=[]
for scenario in ('empty','offset_payload'):
    a,b=[next(r for r in results if r['scenario']==scenario and r['variant']==v) for v in ('python','cpp')]
    comparisons.append({'scenario':scenario,'eligible':a['performance_comparison_eligible'] and b['performance_comparison_eligible'],
                        'cpp_cpu_percent_change':100*(b['cpu_percent_one_core']/a['cpu_percent_one_core']-1),
                        'cpp_cpu_per_complete_frame_percent_change':100*(b['cpu_ms_per_complete_frame']/a['cpu_ms_per_complete_frame']-1),
                        'cpp_rss_median_percent_change':100*(b['rss_mib']['p50']/a['rss_mib']['p50']-1),
                        'cpp_output_rate_percent_change':100*(b['complete_output_hz']/a['complete_output_hz']-1)})
out={'groups':results,'comparisons':comparisons,
     'limits':['Steady-state controlled URDF fixture, fixed arm=0.2, 50Hz joint input, two attachment scenarios; not full robot or simulation acceptance.',
               'RSS/CPU from only each owned producer PID; CPU includes all producer threads and is normalized to one core.',
               'Original acquisition stamps match exactly. Latency includes the 20ms ROS timer and latest-state sampling phase; it is not isolated geometry kernel time.',
               'No output sequence gaps and no incomplete measured frames are necessary for a comparison; this test cannot prove all DDS delivery with an independent writer-side trace.',
               'Unselected 50Hz input samples are intentional latest-state sampling into roughly 10Hz geometry jobs, not missing output frames.',
               'Three interleaved rounds per scenario/variant; CPU clock-tick resolution is 0.01s. No confidence intervals or hardware timing guarantee.']}
(root/'aggregate.json').write_text(json.dumps(out,indent=2)+'\n')
(root/'analyze_script.py').write_bytes(Path(__file__).read_bytes())
print(json.dumps(out,indent=2))
