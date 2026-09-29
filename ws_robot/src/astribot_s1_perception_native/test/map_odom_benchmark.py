#!/usr/bin/env python3
"""Alternating isolated TF trials, measuring proxy/timer delay, not SLAM accuracy."""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import time

from tf2_msgs.msg import TFMessage
from test_map_odom_ros import Runtime,edge


def proc_stats(pid):
    fields=Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split()
    return ((int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK'),
            int(Path(f'/proc/{pid}/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE'))


def trial(impl,directory,count,rate,input_rate):
    directory.mkdir(parents=True,exist_ok=True)
    r=Runtime(impl,directory,{'use_sim_time':False,'publish_rate':rate,
        'report_period_sec':0.,'tf_timeout_sec':.2})
    samples=[]
    offset=-math.cos(.2)*.25+math.sin(.2)*(-.5)
    def one(index):
        x=10.+index*.001
        expected=x+offset
        now=time.time()
        packet=TFMessage(transforms=[edge('map','aft_mapped',now,x=x,y=-.5,z=1.2,yaw=.4),
            edge('odom','astribot_torso_base',now,x=.25,y=-.5,z=.3,yaw=.2)])
        before=len(r.samples)
        start=time.perf_counter_ns()
        r.tf_pub.publish(packet)
        r.until(lambda:any(abs(s['xyz'][0]-expected)<1e-9 for s in r.samples[before:]),timeout=2.)
        matching=next(s for s in r.samples[before:] if abs(s['xyz'][0]-expected)<1e-9)
        assert matching['xyz'][2]==.9 or abs(matching['xyz'][2]-.9)<1e-12
        result=dict(index=index,publish_ns=start,receive_ns=matching['receive_ns'],
                    source_stamp=now,output_stamp=matching['stamp'],xyz=matching['xyz'])
        until=start/1e9+1./input_rate
        while time.perf_counter()<until:r.executor.spin_once(timeout_sec=min(.002,max(0.,until-time.perf_counter())))
        return result
    try:
        for i in range(8):one(i)
        cpu0,rss0=proc_stats(r.proc.pid);start=time.perf_counter()
        for i in range(count):samples.append(one(i+8))
        elapsed=time.perf_counter()-start;cpu1,rss1=proc_stats(r.proc.pid)
        delays=sorted((s['receive_ns']-s['publish_ns'])/1e6 for s in samples)
        return dict(implementation=impl,count=count,publish_rate_hz=rate,input_rate_hz=input_rate,
            median_ms=statistics.median(delays),p95_ms=delays[int(.95*(count-1))],max_ms=max(delays),
            cpu_ms_per_input=(cpu1-cpu0)*1000/count,rss_start_bytes=rss0,rss_end_bytes=rss1,
            elapsed_s=elapsed,loadavg=os.getloadavg(),samples=samples)
    finally:r.close()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--pairs',type=int,default=4)
    parser.add_argument('--count',type=int,default=200)
    parser.add_argument('--rate',type=float,default=100.)
    parser.add_argument('--input-rate',type=float,default=50.)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    rows=[]
    for pair in range(args.pairs):
        for impl in (['python','cpp'] if pair%2==0 else ['cpp','python']):
            result=trial(impl,args.output/f'pair_{pair}_{impl}',args.count,args.rate,args.input_rate)
            result['pair']=pair;rows.append(result)
            print(json.dumps({k:v for k,v in result.items() if k!='samples'}),flush=True)
            (args.output/'results.json').write_text(json.dumps(rows,indent=2))


if __name__=='__main__':main()
