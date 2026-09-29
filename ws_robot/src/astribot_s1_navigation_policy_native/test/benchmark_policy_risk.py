"""Alternating in-process risk-kernel timing; excludes JSON/ROS/transport.

Python baseline is the frozen default runtime with its existing geometry binding
loaded explicitly. A pure-Python fallback is not used for the headline numbers.
"""
import argparse
from dataclasses import asdict
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import resource
import statistics
import subprocess
import sys
import time
from types import SimpleNamespace

HERE=Path(__file__).resolve().parent

def normalize(value):
    if isinstance(value,float) and not math.isfinite(value):return 'NaN' if math.isnan(value) else ('Infinity' if value>0 else '-Infinity')
    if isinstance(value,dict):return {k:normalize(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [normalize(v) for v in value]
    return value

def worker(binding):
    sys.path.insert(0,str(HERE/'reference'/'policy_risk'))
    os.environ.pop('ASTRIBOT_NAV_NATIVE_KERNELS',None)
    import astribot_s1_robot_geometry
    name='astribot_s1_robot_geometry._geometry_native'
    spec=importlib.util.spec_from_file_location(name,binding)
    native=importlib.util.module_from_spec(spec);sys.modules[name]=native;spec.loader.exec_module(native)
    from astribot_s1_robot_geometry import polygon
    assert polygon._native_box_distance is native.box_distance
    from astribot_s1_navigation_policy.contracts import Covariance3, MetricBox, Stamp, Vec3, Version
    from astribot_s1_navigation_policy.ports import PredictionModel, TrackedObstacle, WorldSnapshot
    from astribot_s1_navigation_policy.risk import RobotState, evaluate_risk
    payload=json.load(sys.stdin);stamp=Stamp(10**9,'ros',0);tracks=[]
    for t in payload['tracks']:
        b=t['box'];a,bb,c=b['variance'];m=t['model']
        box=MetricBox(Vec3(*b['center']),Vec3(*b['size']),Covariance3((a,0.,0.,0.,bb,0.,0.,0.,c)))
        model=PredictionModel(Vec3(*m['velocity']),m['variance'],tuple(map(tuple,m['steps'])))
        tracks.append(TrackedObstacle(t['id'],'odom',stamp,box,(),('source',),model))
    world=WorldSnapshot(Version('goal',1,1,1),stamp,'odom',tuple(tracks),(),(),1)
    robot=RobotState(*payload['robot']);path=tuple(map(tuple,payload['path']));profile=SimpleNamespace(**payload['profile'])
    first=evaluate_risk(world,robot,path,profile)
    wall=[];cpu=[]
    for i in range(-payload['benchmark']['warmup'],payload['benchmark']['samples']):
        start=time.perf_counter_ns();before=time.process_time_ns()
        result=evaluate_risk(world,robot,path,profile)
        used=time.process_time_ns()-before;elapsed=time.perf_counter_ns()-start
        assert result==first,'unstable risk result'
        if i>=0:wall.append(elapsed);cpu.append(used)
    print(json.dumps(dict(risk=normalize(asdict(first)),benchmark=dict(wall_ns=wall,cpu_ns=cpu,
        peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        ending_rss_kib=int(next(line.split()[1] for line in Path('/proc/self/status').read_text().splitlines() if line.startswith('VmRSS:')))),
        baseline_geometry_binding=str(Path(native.__file__).resolve()),pid=os.getpid(),cpu_affinity=sorted(os.sched_getaffinity(0)))))

def payload(count,samples):
    tracks=[]
    for i in range(count):
        x=.9+.17*(i%13);y=(-1 if i%2 else 1)*(.45+.12*(i%7))
        tracks.append(dict(id=f'obstacle-{i}',box=dict(center=[x,y,.5],size=[.12,.16,1.],variance=[.0004,.0004,.01]),
          model=dict(velocity=[.03*(i%3),-.04*(i%4),0.],variance=.002,steps=[[n*100000000,n*.1] for n in range(1,21)])))
    profile=dict(half_length_m=.9,half_width_m=.6,clearance_margin_m=.04,payload_extra_margin_m=.01,
        footprint_xy=[[-.5,-.3],[.9,-.3],[.9,.15],[.1,.6],[-.5,.3]],max_speed_m_s=.4,reaction_time_s=.3,
        brake_deceleration_m_s2=.6,angular_brake_deceleration_rad_s2=1.,prediction_horizon_s=2.)
    return dict(op='risk',tracks=tracks,profile=profile,robot=[0.,0.,.15,.35,.05,.12],
        path=[[0.,0.],[3.,0.],[3.,2.]],benchmark=dict(warmup=20,samples=samples))

def same_risk(a,b):
    for key in a:
        if key in ('clearance_m','conflict_time_s'):assert math.isclose(float(a[key]),float(b[key]),rel_tol=2e-11,abs_tol=3e-12)
        else:assert a[key]==b[key],(key,a,b)

def percentile(values,q):
    values=sorted(values);i=(len(values)-1)*q;lo=int(i);hi=min(lo+1,len(values)-1)
    return values[lo]+(values[hi]-values[lo])*(i-lo)

def run(args):
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    cpu=args.cpu if args.cpu is not None else min(os.sched_getaffinity(0))
    assert cpu in os.sched_getaffinity(0)
    rows=[]
    for count in (8,64):
        packet=payload(count,args.samples);(out/f'input_{count}.json').write_text(json.dumps(packet,indent=2))
        for pair in range(args.pairs):
            results={}
            for mode in (('python','cpp') if pair%2==0 else ('cpp','python')):
                cmd=[sys.executable,str(Path(__file__).resolve()),'--worker','--geometry-binding',args.geometry_binding] if mode=='python' else [args.probe]
                started=time.time_ns()
                proc=subprocess.run(['taskset','-c',str(cpu),*cmd],input=json.dumps(packet)+'\n',text=True,capture_output=True,timeout=120)
                assert proc.returncode==0,(cmd,proc.stderr)
                result=json.loads(proc.stdout);assert 'error' not in result,result
                result.update(mode=mode,tracks=count,pair=pair,started_unix_ns=started,cpu=cpu,stderr=proc.stderr,returncode=proc.returncode)
                (out/f'{count}_{pair}_{mode}.json').write_text(json.dumps(result,indent=2));results[mode]=result
                bench=result['benchmark'];wall=[v/1e6 for v in bench['wall_ns']];used=[v/1e6 for v in bench['cpu_ns']]
                rows.append(dict(mode=mode,tracks=count,pair=pair,n=len(wall),p50_ms=percentile(wall,.5),p95_ms=percentile(wall,.95),
                    p99_ms=percentile(wall,.99),max_ms=max(wall),cpu_mean_ms=statistics.mean(used),
                    peak_rss_mib=bench['peak_rss_kib']/1024,ending_rss_mib=bench['ending_rss_kib']/1024))
            same_risk(results['python']['risk'],results['cpp']['risk'])
    summary={}
    for count in (8,64):
        summary[count]={}
        for mode in ('python','cpp'):
            selected=[r for r in rows if r['tracks']==count and r['mode']==mode]
            summary[count][mode]={k:statistics.median(r[k] for r in selected) for k in ('p50_ms','p95_ms','p99_ms','cpu_mean_ms','ending_rss_mib','peak_rss_mib')}
            summary[count][mode]['worst_ms']=max(r['max_ms'] for r in selected)
    result=dict(summary=summary,runs=rows,environment=dict(platform=platform.platform(),cpu=cpu,
        probe=args.probe,probe_sha256=hashlib.sha256(Path(args.probe).read_bytes()).hexdigest(),
        geometry_binding=args.geometry_binding,binding_sha256=hashlib.sha256(Path(args.geometry_binding).read_bytes()).hexdigest(),
        python=sys.version,scope='In-process kernel only; default Python native_math disabled, geometry binding enabled; parsing, JSON and ROS excluded.'))
    (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(summary,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--worker',action='store_true')
    parser.add_argument('--geometry-binding',required=True);parser.add_argument('--probe')
    parser.add_argument('--output');parser.add_argument('--samples',type=int,default=150)
    parser.add_argument('--pairs',type=int,default=4);parser.add_argument('--cpu',type=int)
    arguments=parser.parse_args()
    worker(arguments.geometry_binding) if arguments.worker else run(arguments)
