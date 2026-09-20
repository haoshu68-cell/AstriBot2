#!/usr/bin/env python3
"""Read-only timing diagnostics; fitted time shifts never certify geometry."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import numpy as np


def stats(values):
    return dict(n=len(values), **{f'p{q}':float(np.percentile(values,q))
        for q in (50,95,99,100)}) if len(values) else dict(n=0)


def analyze(directory):
    poses=[]
    for line in (directory/'poses.jsonl').open():
        row=json.loads(line)
        if row.get('source')=='physics_post_update' and 'physics' in row:
            poses.append(row)
    poses.sort(key=lambda r:r['physics']['stamp_ns'])
    result=dict(evidence='offline timing diagnosis; no shifted sample used for acceptance',samples=len(poses))
    if not poses:return result
    stamp=np.array([r['physics']['stamp_ns']*1e-9 for r in poses])
    physical=np.array([r['physics']['expected_world_xyzw'][:3] for r in poses])
    tf=np.array([np.array(r['expected_matrix'])[:3,3] for r in poses])
    velocity=np.zeros(len(poses));dt=np.diff(stamp)
    velocity[1:]=np.linalg.norm(np.diff(physical,axis=0),axis=1)/np.maximum(dt,1e-9)
    moving=(velocity>.05)&(np.r_[0.,dt]<.06)&(stamp>stamp[0]+.05)&(stamp<stamp[-1]-.05)
    result.update(physics_payload_translation_m=stats([r['physics']['position_error_m'] for r in poses]),
        tf_target_translation_m=stats([r['tf_target_vs_physics_target_m'] for r in poses]),
        moving_tf_target_translation_m=stats(np.linalg.norm(tf[moving]-physical[moving],axis=1)),
        moving_samples=int(moving.sum()))
    # Report the full sweep, including zero shift. A lower fitted residual is
    # evidence for further timestamp investigation, not an authorized correction.
    sweep=[]
    if moving.any():
        for delay in np.arange(-.025,.02501,.001):
            delayed=np.stack([np.interp(stamp[moving]-delay,stamp,physical[:,axis]) for axis in range(3)],axis=1)
            error=np.linalg.norm(tf[moving]-delayed,axis=1)
            sweep.append(dict(delay_s=round(float(delay),6),rms_m=float(np.sqrt(np.mean(error**2))),
                p95_m=float(np.percentile(error,95))))
        result['delay_sweep']=sweep;result['best_delay_by_rms']=min(sweep,key=lambda row:row['rms_m'])
    times=defaultdict(set)
    for line in (directory/'sources.jsonl').open():
        r=json.loads(line)
        if r['kind']=='tf':
            for t in r['message']['transforms']:
                s=t['header']['stamp'];times[t['child_frame_id']].add(s['sec']+s['nanosec']*1e-9)
        elif r['kind']=='joints':
            s=r['message']['header']['stamp'];times['/joint_states'].add(s['sec']+s['nanosec']*1e-9)
    result['source_intervals_s']={k:stats(np.diff(sorted(v))) for k,v in times.items()
        if k=='/joint_states' or 'arm_left_link' in k or k=='astribot_torso_base'}
    worst=max(poses,key=lambda r:r['tf_target_vs_physics_target_m'])
    result['worst_sample']=worst
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--output',required=True,type=Path)
    args=p.parse_args();result=analyze(args.directory)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('delay_sweep','worst_sample','source_intervals_s')}))


if __name__=='__main__':main()
