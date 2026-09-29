#!/usr/bin/env python3
"""Assess sampled health without converting capture completion into acceptance."""
import argparse
from collections import Counter
import json
from pathlib import Path


def analyze(folder, warmup=2.):
    summary=json.loads((folder/'summary.json').read_text())
    events=json.loads((folder/'health_events.json').read_text())
    result={'scope': 'sampled camera health; no continuous-time or obstacle coverage proof',
            'scenario': summary.get('scenario', 'see parent session manifest'),
            'duration_wall_sec':summary['duration_wall_sec'], 'warmup_sec':warmup, 'cameras':{}}
    for camera in ('head_rgbd','torso_rgbd','left_wrist_rgbd','right_wrist_rgbd'):
        selected=[row for row in events.get(camera,[]) if row['wall_elapsed']>=warmup]
        ages=sorted(row['age_sec'] for row in selected)
        invalid=[row for row in selected if not row['valid']]
        gaps=[b['wall_elapsed']-a['wall_elapsed'] for a,b in zip(selected,selected[1:])]
        row=summary['cameras'].get(camera,{})
        result['cameras'][camera]={'health_samples':len(selected),
            'states':dict(Counter(x['state'] for x in selected)), 'invalid_samples':len(invalid),
            'max_health_gap_wall_sec':max(gaps, default=None),
            'age_ros_sec':{key:ages[min(int((len(ages)-1)*fraction),len(ages)-1)] if ages else None
                for key,fraction in [('p50',.5),('p95',.95),('p99',.99),('max',1.)]},
            'source_epochs':sorted(set(x['source_epoch'] for x in selected)),
            'first_invalid':invalid[:5], 'exact_stamp_groups':row.get('exact_stamp_groups',0),
            'cloud_messages':row.get('cloud_messages',0),
            'passed_sampled_freshness':bool(selected) and not invalid and max(gaps, default=1.)<=.25
                and selected[0]['wall_elapsed']<=warmup+.25
                and selected[-1]['wall_elapsed']>=summary['duration_wall_sec']-.25}
    result['capture_completed']=summary.get('capture_completed', True) and not summary.get('interrupted', False)
    result['passed_sampled_freshness']=result['capture_completed'] and all(x['passed_sampled_freshness'] for x in result['cameras'].values())
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():raise RuntimeError('Preserve previous analysis')
    result=analyze(args.capture);args.output.write_text(json.dumps(result,indent=2))
    print(json.dumps({k:{'states':v['states'],'max_gap':v['max_health_gap_wall_sec'],
        'passed':v['passed_sampled_freshness']} for k,v in result['cameras'].items()},indent=2))
    raise SystemExit(0 if result['passed_sampled_freshness'] else 1)
