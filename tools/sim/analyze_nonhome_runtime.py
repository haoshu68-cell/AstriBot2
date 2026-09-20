#!/usr/bin/env python3
"""Read-only source-age and known-wall diagnostics from the runtime probe.

Clearance is reserved-polygon versus fixture geometry, not a contact force or
external accuracy measurement. The wall fixture must be identified separately.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import numpy as np


def stats(values):
    return dict(n=len(values), **{f'p{q}':float(np.percentile(values,q))
        for q in (0,50,95,100)}) if values else dict(n=0)


def analyze(path, wall_width=None):
    observations=[];policies=[];health=Counter();geometry=Counter()
    polygon=None;pose=None;clearance=[];angular=0;latest_epoch=None;epochs=set()
    if wall_width is not None:
        from astribot_s1_robot_geometry.polygon import box_distance_many
        lower=np.array([[-.6,wall_width/2],[-.6,-wall_width/2-.1]])
        upper=np.array([[.6,wall_width/2+.1],[.6,-wall_width/2]])
    for line in path.open():
        row=json.loads(line);kind=row['kind'];m=row['message']
        if 'data' in m and kind in ('observation','policy'):m=json.loads(m['data'])
        if kind=='observation' and m.get('envelope_ready'):observations.append(m)
        elif kind=='policy':policies.append(m)
        elif kind=='sensor_health':
            for sensor in m['sensors']:health[(sensor['sensor_id'],sensor['reason'])]+=1
        elif kind=='geometry':geometry[m['reason']]+=1
        elif kind=='envelope' and m['navigation_allowed']:
            latest_epoch=(m['coordinator_session_id'],m['epoch']);epochs.add(latest_epoch)
            polygon=np.array([(p['x'],p['y']) for p in m['reserved_footprint']['points']])
        elif kind=='odom':
            p=m['pose']['pose'];q=p['orientation'];x=p['position']['x'];y=p['position']['y']
            yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
            pose=(x,y,yaw)
            if wall_width is not None and polygon is not None:
                clearance.append(float(min(box_distance_many(polygon,x,y,yaw,lower,upper))))
        elif kind=='raw_command' and pose is not None and abs(pose[0])<=.6:
            angular+=int(math.hypot(m['linear']['x'],m['linear']['y'])<.005 and
                         abs(m['angular']['z'])>1e-6)
    result=dict(evidence='recorded simulation receipt; source time retained',
        ready_observation_cycles=len(observations),
        valid_ready_observation_cycles=sum(x['inputs_valid'] for x in observations),
        scan_age_s=stats([x['scan_age_s'] for x in observations if x.get('scan_age_s') is not None]),
        processing_wall_s=stats([x['processing_wall_s'] for x in observations]),
        processing_cpu_s=stats([x['processing_cpu_s'] for x in observations]),
        tracks=stats([x['tracks'] for x in observations]),
        geometry_reasons=dict(geometry),sensor_reasons={str(k):v for k,v in health.items()},
        policy_reasons=dict(Counter(x['reason'] for x in policies)),
        installed_sessions_and_epochs=sorted(epochs))
    if wall_width is not None:
        result['known_wall_check']=dict(width_m=wall_width,clearance_m=stats(clearance),
            angular_commands_below_translation_gate_in_wall_strip=angular,
            required_clearance_per_side_m=.08,
            evidence='all recorded poses after envelope installation against known wall boxes; not contact proof')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('messages',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--wall-width',type=float)
    args=p.parse_args()
    if args.wall_width is not None and not 0<args.wall_width<10:p.error('invalid wall width')
    result=analyze(args.messages,args.wall_width)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
