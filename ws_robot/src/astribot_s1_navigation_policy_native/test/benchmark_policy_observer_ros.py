"""Owned, isolated ROS observer AB/BA replay; no robot/simulation commands."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import time

from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from test_policy_observer_ros import Runtime, stamp, BINDING


def usage(pid):
    fields=Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split()
    cpu=(int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK')
    rss=int(next(line.split()[1] for line in Path(f'/proc/{pid}/status').read_text().splitlines() if line.startswith('VmRSS:')))
    return cpu,rss


def semantic(value):
    return {k:v for k,v in value.items() if not k.startswith('processing_')}


def same(a,b,path=''):
    if isinstance(a,dict):
        assert a.keys()==b.keys(),path
        for k in a:same(a[k],b[k],path+'/'+str(k))
    elif isinstance(a,list):
        assert len(a)==len(b),(path,len(a),len(b))
        for i,(x,y) in enumerate(zip(a,b)):same(x,y,path+'/'+str(i))
    elif isinstance(a,float):assert math.isclose(a,b,rel_tol=2e-12,abs_tol=3e-12),(path,a,b)
    else:assert a==b,(path,a,b)


def percentile(values,q):
    v=sorted(values);index=(len(v)-1)*q;low=int(index);high=min(low+1,len(v)-1)
    return v[low]+(v[high]-v[low])*(index-low)


def run_one(impl,count,args,directory):
    directory.mkdir(parents=True,exist_ok=True)
    runtime=Runtime(impl,directory)
    samples=[];states=[];memory=[]
    try:
        runtime.seed();runtime.tick()
        for i in range(args.warmup+args.samples):
            ns=runtime.ns
            odom=Odometry();odom.header.frame_id='odom';odom.header.stamp=stamp(ns);odom.pose.pose.orientation.w=1.;runtime.odom_pub.publish(odom)
            runtime.send_scan()
            packet=dict(schema_version=1,sensor_id='objects',stamp_ns=ns,frame_id='odom',calibration_epoch=0,observations=[])
            for j in range(count):
                packet['observations'].append(dict(kind='metric_box',measurement_id=f'{ns}:{j}',track_id=f'object-{j}',
                    center_m=[.9+.17*(j%13),(-1 if j%2 else 1)*(.45+.12*(j%7)),.5],size_m=[.12,.16,1.],
                    position_variance_m2=.0004,geometry_quality=1.,provenance=[f'objects:{ns}:{j}']))
            runtime.vision_pub.publish(String(data=json.dumps(packet)))
            runtime.drain(.03)
            if i==args.warmup:cpu_start,rss_start=usage(runtime.proc.pid);window_start=time.monotonic()
            previous=len(runtime.outputs);runtime.ns+=100_000_000;start=time.perf_counter_ns();runtime.clock.publish(Clock(clock=stamp(runtime.ns)))
            runtime.until(lambda:len(runtime.outputs)>previous and runtime.outputs[-1]['stamp_ns']==runtime.ns)
            elapsed=time.perf_counter_ns()-start;result=runtime.outputs[-1]
            assert result['vision_count']==i+1 and result['scan_count']==i+2 and result['errors']==0,result
            assert result['tracks']==count and result['inputs_valid'],result
            if i>=args.warmup:
                samples.append(dict(delivery_wall_ns=elapsed,processing_wall_s=result['processing_wall_s'],processing_cpu_s=result['processing_cpu_s'],stages_s=result['processing_stages_s']))
                states.append(semantic(result))
                if (i-args.warmup)%20==0:memory.append(dict(sample=i-args.warmup,rss_kib=usage(runtime.proc.pid)[1]))
        cpu_end,rss_end=usage(runtime.proc.pid);window_s=time.monotonic()-window_start
        return dict(implementation=impl,tracks=count,samples=samples,states=states,rss_samples=memory,
            process_cpu_s=cpu_end-cpu_start,window_s=window_s,starting_rss_kib=rss_start,ending_rss_kib=rss_end,
            cpu_affinity=Path(f'/proc/{runtime.proc.pid}/status').read_text().split('Cpus_allowed_list:\t')[1].splitlines()[0])
    finally:runtime.close()


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--samples',type=int,default=100);p.add_argument('--warmup',type=int,default=20);p.add_argument('--pairs',type=int,default=4)
    p.add_argument('--workloads',type=int,nargs='+',default=[8,64]);args=p.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    cpus=sorted(os.sched_getaffinity(0));assert len(cpus)>=2
    os.sched_setaffinity(0,{cpus[0]});os.environ['POLICY_OBSERVER_CPU']=str(cpus[1])
    rows=[]
    for count in args.workloads:
        reference=None
        for pair in range(args.pairs):
            for impl in (('python','cpp') if pair%2==0 else ('cpp','python')):
                directory=out/f'{count}_{pair}_{impl}';result=run_one(impl,count,args,directory)
                if reference is None:reference=result['states']
                same(reference,result['states'])
                (directory/'result.json').write_text(json.dumps(result,indent=2))
                values=result['samples'];row=dict(tracks=count,pair=pair,implementation=impl,samples=len(values),
                    processing_p50_ms=percentile([s['processing_wall_s']*1e3 for s in values],.5),
                    processing_p95_ms=percentile([s['processing_wall_s']*1e3 for s in values],.95),
                    processing_worst_ms=max(s['processing_wall_s']*1e3 for s in values),
                    processing_cpu_mean_ms=statistics.mean(s['processing_cpu_s']*1e3 for s in values),
                    delivery_p95_ms=percentile([s['delivery_wall_ns']*1e-6 for s in values],.95),
                    process_cpu_per_sample_ms=result['process_cpu_s']*1e3/len(values),
                    ending_rss_mib=result['ending_rss_kib']/1024,rss_growth_mib=(result['ending_rss_kib']-result['starting_rss_kib'])/1024)
                rows.append(row);print(json.dumps(row),flush=True)
    summary={}
    for count in args.workloads:
        summary[count]={}
        for impl in ('python','cpp'):
            group=[r for r in rows if r['tracks']==count and r['implementation']==impl]
            summary[count][impl]={k:statistics.median(r[k] for r in group) for k in group[0] if k not in ('tracks','pair','implementation','samples')}
    binary=Path(os.environ.get('POLICY_OBSERVER_CPP','/tmp/codex_policy_integration_20260921/build/policy_observer_cpp'))
    report=dict(summary=summary,runs=rows,environment=dict(platform=platform.platform(),driver_cpu=cpus[0],observer_cpu=cpus[1],
        binary=str(binary),binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),binding=str(BINDING),binding_sha256=hashlib.sha256(BINDING.read_bytes()).hexdigest(),
        scope='Owned observer processes only; simulated input clock, isolated/remapped topics; no Gazebo or robot. CPU includes ROS callbacks; startup excluded. Default Python geometry binding enabled, optional navigation kernels disabled.'),configuration=vars(args))
    (out/'summary.json').write_text(json.dumps(report,indent=2))


if __name__=='__main__':main()
