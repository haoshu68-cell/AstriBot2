#!/usr/bin/env python3
"""Read-only analysis of the owned N4 diagnostic trace; starts no ROS nodes."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics


def distribution(values):
    values=sorted(v for v in values if isinstance(v,(float,int)) and math.isfinite(v))
    if not values:return {'n':0}
    return {'n':len(values),'p50':statistics.median(values),
            'p95':values[max(0,math.ceil(.95*len(values))-1)],'max':values[-1]}


def summarize(rows):
    observations=[r['data']for r in rows if r['source']=='observation']
    states=[r['data']for r in rows if r['source']=='state']
    scans=[r for r in rows if r['source']=='scan_source']
    answer={'observation_samples':len(observations),'state_samples':len(states),
            'state_reasons':dict(Counter(d['reason']for d in states)),
            'observation_stages_ms':{},'state_stages_ms':{},'scan_age_ms':distribution([d['scan_age_s']*1000for d in observations if d.get('scan_age_s')is not None])}
    if observations:answer['tracks']={'min':min(d['tracks']for d in observations),'max':max(d['tracks']for d in observations)}
    for label,data in [('observation',observations),('state',states)]:
        answer[label+'_processing_ms']=distribution([d['processing_wall_s']*1000for d in data])
        keys={k for d in data for k in d.get('processing_stages_s',{})}
        for key in sorted(keys):answer[label+'_stages_ms'][key]=distribution([d['processing_stages_s'][key]*1000for d in data if key in d.get('processing_stages_s',{})])
    rejected=[d for d in states if not d.get('coverage_ok',True)]
    answer['coverage_rejection_samples']=len(rejected)
    answer['rejected_health']=dict(Counter(h['sensor_id']+':'+h['state']+':'+h['reason'] for d in rejected for h in d.get('coverage_health',[])if h.get('required')and h['state']!='VALID'))
    answer['stopped_coverage_rejections']=sum(math.hypot(d['coverage_motion']['vx'],d['coverage_motion']['vy'])<.01 and abs(d['coverage_motion']['wz'])<=.02 for d in rejected if d.get('coverage_motion'))
    if scans:
        ages=[(r['ros_s']-r['capture_s'])*1000for r in scans]
        answer['independent_scan_probe']={'samples':len(scans),'receipt_age_ms':distribution(ages),
            'negative_age_samples':sum(v<0for v in ages),
            'tf_unavailable_samples':sum(not r.get('tf_at_capture')or any(not value for value in r['tf_at_capture'].values())for r in scans),
            'boundary':'Probe TF buffer and receive time are not the policy worker buffer or internal queue time.'}
    return answer


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace',type=Path);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--navigation-result',type=Path)
    args=parser.parse_args();raw=args.trace.read_bytes();rows=[];incomplete_tail=False
    lines=raw.splitlines()
    for index,line in enumerate(lines):
        try:row=json.loads(line)
        except ValueError:
            if index==len(lines)-1 and not raw.endswith(b'\n'):incomplete_tail=True;continue
            raise
        if row.get('source')in ('state','observation','scan_source'):rows.append(row)
    report={'trace':str(args.trace.resolve()),'trace_sha256':hashlib.sha256(raw).hexdigest(),'sampled_trace':True,
            'percentile':'nearest rank ceil(.95*n); sampled maximum is not a timing bound',
            'incomplete_tail_ignored':incomplete_tail,'overall':summarize(rows),'phases':{},
            'limitations':['observer/state stage timings overlap; do not add them','no raw scan delivery or TF waiting timing inferred from this trace','correlation of track count and runtime is not full root-cause proof','no runtime or configuration changes; no new simulation']}
    if args.navigation_result:
        raw_result=args.navigation_result.read_bytes();result=json.loads(raw_result)
        report['navigation_result']={'path':str(args.navigation_result.resolve()),'sha256':hashlib.sha256(raw_result).hexdigest(),
                                      'passed':result['passed'],'error':result.get('error'),'cleanup_complete':result.get('cleanup_complete')}
        starts=[e for e in result['events']if e['kind']=='nav_started']
        ends=[e for e in result['events']if e['kind']=='nav_terminal']
        for index,start in enumerate(starts):
            end=next((e for e in ends if e['wall']>=start['wall']),None)
            selected=[r for r in rows if start['wall']<=r['wall'] and (end is None or r['wall']<=end['wall'])]
            report['phases']['goal_'+str(index+1)]={'start':start,'terminal':end,'summary':summarize(selected)}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'rows':len(rows),'output':str(args.output),'phases':list(report['phases'])}))


if __name__=='__main__':main()
