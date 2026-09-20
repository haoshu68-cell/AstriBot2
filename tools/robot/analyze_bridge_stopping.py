#!/usr/bin/env python3
"""Analyze recorded zero-command stopping using source-time positions, never SDK twist."""
import argparse
import json
import math
from pathlib import Path
import numpy as np


def positions(data,key,origin):
    rows=data[key]
    t=np.array([r['stamp'] for r in rows]);xy=np.array([[r['x'],r['y']] for r in rows]);yaw=np.unwrap([r['yaw'] for r in rows])
    if key=='odom':
        d=xy-np.array([origin['x'],origin['y']]);c,s=math.cos(origin['yaw']),math.sin(origin['yaw'])
        xy=np.column_stack((c*d[:,0]+s*d[:,1],-s*d[:,0]+c*d[:,1]))
    return t,np.column_stack((xy,yaw))


def slope(t,q):
    if len(t)<3 or t[-1]-t[0]<.15:return None
    x=t-t.mean();return float(np.dot(x,q-q.mean())/np.dot(x,x))


def analyze(path):
    result=json.loads(path.read_text());data=json.loads((path.parent/'samples.json').read_text());case=result['case'];axis=case['axis'];sign=case['sign']
    out={'path':str(path.parent),'case':case,'success':result['success'],'return_error':result.get('return_error'),'failure':result.get('failure'),'sources':{}}
    if 'zero' not in result:return out
    zero=result['zero']['wall'];coast_end=result['zero']['wall']+result['coast_end']-result['zero']['t']
    for key in ['odom','sdk']:
        t,q=positions(data,key,result['origin']);mask=(t>=zero)&(t<=coast_end)
        if not (t[0]<zero<t[-1]) or sum(mask)<10:continue
        q0=np.array([np.interp(zero,t,q[:,i]) for i in range(3)])
        after=q[mask]-q0;forward=sign*after[:,axis];before=(t>=zero-.4)&(t<=zero)
        speed=slope(t[before],q[before,axis])
        nearzero=0. if speed is None else abs(speed)
        baseline=np.array([r['phase']=='baseline' for r in data[key]])
        noise=float(np.ptp(q[baseline,axis])) if sum(baseline)>=3 else None
        peak=max(0.,float(forward.max()));tail=after[-5:].mean(axis=0)
        out['sources'][key]={'pre_zero_speed':speed,'pre_zero_samples':int(sum(before)),
            'peak_forward':peak,'final_forward':float(sign*tail[axis]),
            'rebound_from_peak':max(0.,peak-float(sign*tail[axis])),
            'peak_euclidean_m':float(np.linalg.norm(after[:,:2],axis=1).max()),
            'peak_cross_axis_m':float(np.max(np.abs(after[:,1-axis]))) if axis<2 else None,
            'peak_yaw_rad':float(np.max(np.abs(after[:,2]))),
            'peak_delay_s':float(t[mask][np.argmax(forward)]-zero),
            'baseline_peak_to_peak':noise,'response_ratio':sign*speed/case['speed'] if speed is not None else None,
            'effective_stop_time_s':peak/nearzero if nearzero>.005 else None}
        moving=np.array([r['phase']=='outbound' for r in data[key]])
        if key=='odom' and sum(moving)>2:
            travel=q[moving]-q[moving][0]
            out['tracking']={'cross_track_max_m':float(np.max(np.abs(travel[:,1-axis]))) if axis<2 else None,
                'heading_max_deg':float(np.max(np.abs(travel[:,2]))*180/math.pi),
                'translation_max_m':float(np.max(np.linalg.norm(travel[:,:2],axis=1)))}
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory');args=p.parse_args();w=Path(args.directory)
    results=[analyze(p) for p in sorted(w.glob('**/result.json')) if 'case' in json.loads(p.read_text())]
    (w/'stopping_results.json').write_text(json.dumps(results,indent=2))
    for r in results:
        a=r['sources'].get('odom',{});b=r['sources'].get('sdk',{})
        print(r['case']['direction'],r['case']['speed'],'pass',r['success'],'v0',a.get('pre_zero_speed'),'SLAM peak',a.get('peak_forward'),'SDK peak',b.get('peak_forward'),'return',r['return_error'],'error',r['failure'])

if __name__=='__main__':main()
