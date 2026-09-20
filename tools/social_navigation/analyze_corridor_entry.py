#!/usr/bin/env python3
"""Read-only diagnosis of corridor entry arrival and low-speed response."""
import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path


def analyze(directory):
    events=[json.loads(line) for line in (directory/'protocol/events.jsonl').read_text().splitlines()]
    start=next(e for e in events if e.get('validation_target'))
    end=next(e for e in events if e['wall_time']>start['wall_time'] and
             ('navigation_position_error_m' in e or e['stage'] in ('FAULT','PASS')))
    target=start['validation_target']
    samples=[r for r in json.loads((directory/'samples.json').read_text())
             if start['wall_time']<=r['wall_s']<=end['wall_time']]
    if len(samples)<2:raise ValueError('Missing entry samples')
    if any(not 0<b['sim_s']-a['sim_s']<=.15 for a,b in zip(samples,samples[1:])):
        raise ValueError('Entry samples are not physically continuous')
    commands=[json.loads(line) for line in (directory/'observations.jsonl').read_text().splitlines()
              if json.loads(line)['kind']=='command']
    commands.sort(key=lambda r:r['ros_s']);stamps=[r['ros_s'] for r in commands]
    rows=[]
    for sample in samples:
        row=dict(sample)
        row['xy_error_m']=math.hypot(sample['x']-target[0],sample['y']-target[1])
        row['yaw_error_deg']=math.degrees(abs(math.remainder(sample['yaw']-target[2],2*math.pi)))
        index=bisect_right(stamps,sample['sim_s'])-1
        command=commands[index] if index>=0 and sample['sim_s']-stamps[index]<=.15 else None
        row['command_wz']=command['w'] if command else None
        row['command_v']=command['v'] if command else None
        rows.append(row)
    intervals=[];active=[]
    def close():
        if len(active)>1 and active[-1]['sim_s']-active[0]['sim_s']>=.5:
            intervals.append(dict(start_sim_s=active[0]['sim_s'],end_sim_s=active[-1]['sim_s'],
                duration_s=active[-1]['sim_s']-active[0]['sim_s'],
                yaw_error_start_deg=active[0]['yaw_error_deg'],yaw_error_end_deg=active[-1]['yaw_error_deg'],
                minimum_command_rad_s=min(abs(r['command_wz']) for r in active),
                maximum_measured_rad_s=max(abs(r['wz']) for r in active)))
        active.clear()
    for row in rows:
        stalled=(row['phase']=='REFINE' and row['command_wz'] is not None and
                 abs(row['command_wz'])>=.015 and abs(row['wz'])<.001 and row['yaw_error_deg']>.1)
        if stalled:active.append(row)
        else:close()
    close()
    report=dict(episode=str(directory.resolve()),target=target,
        navigation_succeeded='navigation_position_error_m' in end,
        elapsed_wall_s=end['wall_time']-start['wall_time'],
        elapsed_sim_s=rows[-1]['sim_s']-rows[0]['sim_s'],
        last_xy_error_m=rows[-1]['xy_error_m'],last_yaw_error_deg=rows[-1]['yaw_error_deg'],
        command_coverage=sum(r['command_wz'] is not None for r in rows)/len(rows),
        low_speed_yaw_response_intervals=intervals,
        boundary='Simulation map-to-base TF and odom; Twist event time is receiver ROS time. '
                 'Low response does not identify wheel friction or PID as the unique cause. Duration is diagnostic, not a score.')
    return report,rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episodes',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure,axes=plt.subplots(3,len(args.episodes),figsize=(5*len(args.episodes),9),squeeze=False)
    reports=[]
    for col,directory in enumerate(args.episodes):
        report,rows=analyze(directory);reports.append(report)
        t=[r['sim_s']-rows[0]['sim_s'] for r in rows]
        axes[0,col].plot(t,[1000*r['xy_error_m'] for r in rows]);axes[0,col].axhline(2,color='red',ls='--')
        axes[1,col].plot(t,[r['yaw_error_deg'] for r in rows]);axes[1,col].axhline(.1,color='red',ls='--')
        axes[2,col].plot(t,[r['wz'] for r in rows],label='measured odom')
        if report['command_coverage']>0:
            axes[2,col].plot(t,[r['command_wz'] for r in rows],label='final command',alpha=.7)
        axes[2,col].legend();axes[0,col].set_title(directory.parent.name+'\n'+directory.parent.parent.name)
        for row,label in enumerate(('XY error (mm)','Yaw error (deg)','Yaw rate (rad/s)')):
            axes[row,col].set_ylabel(label);axes[row,col].grid(alpha=.3)
        axes[2,col].set_xlabel('Simulation seconds from entry start')
    figure.tight_layout();figure.savefig(args.output/'entry_response.png',dpi=140);plt.close(figure)
    (args.output/'summary.json').write_text(json.dumps(reports,indent=2)+'\n')
    print(json.dumps(reports,indent=2))


if __name__=='__main__':main()
