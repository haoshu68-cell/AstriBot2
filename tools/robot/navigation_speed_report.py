#!/usr/bin/env python3
"""Compare recorded velocity commands with SLAM position derivatives; never controls ROS."""
import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def summary(values):
    if not values:
        return {'samples': 0}
    ordered = sorted(abs(v) for v in values)
    return {'samples': len(values), 'bias': sum(values)/len(values),
            'rms': math.sqrt(sum(v*v for v in values)/len(values)),
            'abs_p95': ordered[math.ceil(.95*len(ordered))-1], 'abs_max': ordered[-1]}


def load(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def velocities(samples):
    segments = []; current = []; seen = set()
    quality = {'duplicates': 0, 'goal_boundaries': 0, 'time_or_pose_breaks': 0, 'source_samples': len(samples)}
    for row in samples:
        stamp = row.get('stamp_ns', round(row['stamp']*1e9))
        key = (row['goal'], stamp)
        if key in seen:
            quality['duplicates'] += 1
            continue
        seen.add(key)
        r = dict(row, time=stamp/1e9)
        if current:
            prev = current[-1]; dt = r['time']-prev['time']
            speed = math.hypot(r['pose'][0]-prev['pose'][0], r['pose'][1]-prev['pose'][1])/dt if dt > 0 else math.inf
            turn = abs(wrap(r['pose'][2]-prev['pose'][2]))/dt if dt > 0 else math.inf
            if row['goal'] != prev['goal'] or not 0 < dt <= .5 or speed > 2 or turn > 4:
                segments.append(current); current = []
                quality['goal_boundaries' if row['goal'] != prev['goal'] else 'time_or_pose_breaks'] += 1
        current.append(r)
    if current:
        segments.append(current)
    result = []
    for segment in segments:
        for a, mid, z in zip(segment, segment[1:], segment[2:]):
            if not .08 <= z['time']-a['time'] <= .45:
                continue
            ts = [r['time']-mid['time'] for r in (a, mid, z)]; mean = sum(ts)/3
            centered = [t-mean for t in ts]; den = sum(t*t for t in centered)
            angles = [a['pose'][2], a['pose'][2]+wrap(mid['pose'][2]-a['pose'][2])]
            angles.append(angles[1]+wrap(z['pose'][2]-mid['pose'][2]))
            v = [sum(t*r['pose'][axis] for t,r in zip(centered,(a,mid,z)))/den for axis in (0,1)]
            v.append(sum(t*angle for t,angle in zip(centered,angles))/den)
            c,s = math.cos(mid['pose'][2]), math.sin(mid['pose'][2])
            result.append({'time': mid['time'], 'begin': a['time'], 'end': z['time'],
                           'goal': mid['goal'], 'phase': mid.get('phase','UNKNOWN'),
                           'post_action': mid.get('post_action',False),
                           'map': v, 'body': [c*v[0]+s*v[1], -s*v[0]+c*v[1], v[2]]})
    quality['velocity_samples'] = len(result)
    return result, quality


class Commands:
    def __init__(self, rows):
        self.rows = sorted(rows, key=lambda r:r.get('ros_ns', round(r['ros']*1e9)))
        self.times = [r.get('ros_ns', round(r['ros']*1e9))/1e9 for r in self.rows]

    def average(self, begin, end, goal):
        index = bisect_right(self.times, begin)-1
        if index < 0 or end <= begin:
            return None
        total = [0.,0.,0.]; cursor = begin
        while cursor < end-1e-8:
            r = self.rows[index]
            if r['goal'] != goal:
                return None
            next_time = self.times[index+1] if index+1 < len(self.times) else math.inf
            stop = min(end,next_time)
            if stop-self.times[index] > .35:
                return None
            for axis in range(3): total[axis] += r['v'][axis]*(stop-cursor)
            cursor = stop; index += 1
        return [v/(end-begin) for v in total]


def compare(rows, commands, frame, lag=0.):
    pairs = [(r,commands.average(r['begin']-lag,r['end']-lag,r['goal'])) for r in rows]
    return [(r,c,r[frame]) for r,c in pairs if c is not None]


def errors(pairs):
    return {'linear_vector_mps': summary([math.hypot(v[0]-c[0],v[1]-c[1]) for _,c,v in pairs]),
            'vx_mps': summary([v[0]-c[0] for _,c,v in pairs]),
            'vy_mps': summary([v[1]-c[1] for _,c,v in pairs]),
            'yaw_rate_radps': summary([v[2]-c[2] for _,c,v in pairs])}


def command_dynamics(commands, step=.1):
    """Compare command smoothness at a fixed cadence, without amplifying receive jitter."""
    accelerations = [[] for _ in range(3)]
    jerks = [[] for _ in range(3)]
    bins = 0
    for goal in dict.fromkeys(r['goal'] for r in commands.rows):
        series = Commands([r for r in commands.rows if r['goal'] == goal])
        if len(series.times) < 2:
            continue
        previous = previous_accel = None
        for index in range(int((series.times[-1]-series.times[0])/step)):
            begin = series.times[0]+index*step
            velocity = series.average(begin, begin+step, goal)
            if velocity is None:
                previous = previous_accel = None
                continue
            bins += 1
            if previous is not None:
                accel = [(v-p)/step for v,p in zip(velocity,previous)]
                for axis,value in enumerate(accel):
                    accelerations[axis].append(value)
                if previous_accel is not None:
                    for axis,(a,p) in enumerate(zip(accel,previous_accel)):
                        jerks[axis].append((a-p)/step)
                previous_accel = accel
            previous = velocity
    return {
        'sample_hz': 1./step, 'averaging_window_s': step, 'valid_bins': bins,
        'measurement': 'Time-averaged command stream using ROS receive timestamps; not physical acceleration or jerk. Averaging suppresses short peaks; these values do not prove hard acceleration/jerk limits.',
        'acceleration': {name: summary(values) for name,values in
                         zip(('x_mps2','y_mps2','yaw_radps2'), accelerations)},
        'jerk': {name: summary(values) for name,values in
                zip(('x_mps3','y_mps3','yaw_radps3'), jerks)},
    }


def fit_lag(rows, commands, frame, axes):
    common = [r for r in rows if not r['post_action'] and
              commands.average(r['begin']-1.,r['end'],r['goal']) is not None]
    if len(common) < 20:
        return {'status':'insufficient_common_window'}
    fits = []
    for step in range(51):
        lag = step*.02; pairs = compare(common,commands,frame,lag)
        cs = [c[a] for _,c,v in pairs for a in axes]; vs = [v[a] for _,c,v in pairs for a in axes]
        energy = sum(c*c for c in cs)
        if not cs or energy < 1e-6:
            continue
        gain = sum(c*v for c,v in zip(cs,vs))/energy
        mse = sum((v-gain*c)**2 for c,v in zip(cs,vs))/len(cs)
        fits.append({'lag_s':lag,'gain':gain,'mse':mse,'samples':len(pairs)})
    if not fits:
        return {'status':'no_command_excitation'}
    pairs = compare(common,commands,frame)
    # Each component must vary over time; vector x/y differences alone are not excitation.
    varying = max((max(c[a] for _,c,v in pairs)-min(c[a] for _,c,v in pairs)) for a in axes)
    actual_rms = math.sqrt(sum(v[a]**2 for _,c,v in pairs for a in axes)/(len(pairs)*len(axes)))
    if varying < .02 or actual_rms < .005:
        return {'status':'lag_not_identifiable','command_range':varying,'actual_rms':actual_rms}
    best = min(fits,key=lambda f:f['mse'])
    improvement = 1-best['mse']/fits[0]['mse'] if fits[0]['mse'] > 1e-12 else 0.
    return dict(best,status='boundary_or_weak_fit' if best['lag_s']==1. or improvement<.05 else 'estimated',
                improvement_vs_zero_lag=improvement,search_range_s=[0.,1.],resolution_s=.02,
                interpretation='Observed command-to-SLAM lag; includes transport, SDK and localization timing. Gain is a dynamic least-squares fit, not a steady-state calibration.')


def stop_drift(samples, commands):
    result = []
    for axis,name in [((0,1),'linear'),((2,),'yaw')]:
        for i in range(1,len(commands.rows)):
            old,now=commands.rows[i-1:i+1]
            if old['goal']!=now['goal'] or not any(abs(old['v'][a])>1e-6 for a in axis) or any(abs(now['v'][a])>1e-6 for a in axis):
                continue
            zero = commands.times[i]
            resume = next((commands.times[j] for j in range(i+1,len(commands.rows))
                           if commands.rows[j]['goal']!=now['goal'] or any(abs(commands.rows[j]['v'][a])>1e-6 for a in axis)),math.inf)
            rows = [r for r in samples if r['goal']==now['goal'] and zero-.15<=r['stamp']<min(resume,zero+2.2)]
            before = [r for r in rows if r['stamp']<=zero]
            if not before:continue
            start = before[-1]; event={'goal':now['goal'],'channel':name,'zero_time':zero,'zero_pose_age_s':zero-start['stamp'],'drift':{}}
            for duration in (.5,1.,2.):
                tail = [r for r in rows if zero+duration-.12<=r['stamp']<=zero+duration+.12]
                if not tail or resume<zero+duration:continue
                end=min(tail,key=lambda r:abs(r['stamp']-zero-duration))
                event['drift'][str(duration)]={'distance_m':math.hypot(end['pose'][0]-start['pose'][0],end['pose'][1]-start['pose'][1]),
                                               'yaw_deg':math.degrees(wrap(end['pose'][2]-start['pose'][2]))}
            if event['drift']:result.append(event)
    return result


def analyze(directory):
    directory=Path(directory); samples=load(directory/'samples.jsonl'); commands=load(directory/'commands.jsonl')
    status=json.loads((directory/'status.json').read_text()) if (directory/'status.json').exists() else {}
    frames=status.get('command_frames',{'/cmd_vel_nav_body_raw':'body','/cmd_vel_nav_body':'body'})
    rows,quality=velocities(samples)
    report={'measurement':'unique SLAM map TF; 3-point centered least-squares position/yaw derivative',
            'position_jump_exclusion':'Only analysis: split source gaps >0.5s or implied speed >2m/s /4rad/s; no controller changes.',
            'body_projection':'Window midpoint SLAM heading; short-window approximation while rotating.',
            'clock':'Command ROS receive time compared with SLAM source time; receive ages retained separately.',
            'quality':quality,'topics':{},'unclassified_topics':[]}
    aligned=[]
    for topic in sorted({r['topic'] for r in commands}):
        frame=frames.get(topic)
        if frame not in ('body','map'):
            report['unclassified_topics'].append(topic);continue
        series=Commands([r for r in commands if r['topic']==topic]); pairs=compare(rows,series,frame)
        entry={'frame':frame,'zero_lag':errors(pairs),'by_goal':{},'by_phase':{},
               'command_dynamics_10hz':command_dynamics(series),
               'linear_response':fit_lag(rows,series,frame,(0,1)),
               'angular_response':fit_lag(rows,series,frame,(2,)),
               'zero_command_drift':stop_drift(samples,series)}
        for key in ('goal','phase'):
            entry['by_'+key]={name:errors([p for p in pairs if p[0][key]==name and not p[0]['post_action']]) for name in sorted({r[key] for r in rows})}
        for r,c,v in pairs:aligned.append({'topic':topic,'frame':frame,'stamp':r['time'],'goal':r['goal'],'phase':r['phase'],'post_action':r['post_action'],'command':c,'slam_velocity':v})
        report['topics'][topic]=entry
    (directory/'speed_report.json').write_text(json.dumps(report,indent=2)+'\n')
    (directory/'speed_aligned.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in aligned))
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory')
    args=parser.parse_args();report=analyze(args.directory)
    print(json.dumps({'quality':report['quality'],'topics':list(report['topics'])},indent=2))
