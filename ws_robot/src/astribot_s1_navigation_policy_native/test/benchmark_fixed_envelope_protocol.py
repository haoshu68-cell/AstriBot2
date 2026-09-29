"""Same-input fixed-posture proposal service A/B, independent domain 116.

Paused ROS time holds the same source/hold/odom leases for all requests. Measured
latency is service round trip; it does not include six-consumer apply or motion.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
import time
from benchmark_envelope_protocol import process_stats, percentile
from test_fixed_envelope_protocol import FixedProtocol


def trial(impl,binary,count,logs,index):
    p=FixedProtocol(impl,binary,logs/f'{index}_{impl}.log')
    try:
        p.propose()
        for _ in range(20):assert p.call(p.client,p.req).accepted
        epoch=p.call(p.client,p.req).epoch
        cpu0,rss=process_stats(p.proc.pid);begin=time.perf_counter();latency=[]
        for i in range(count):
            p.req.request_id=f'benchmark-{i}'
            start=time.perf_counter_ns();reply=p.call(p.client,p.req)
            latency.append((time.perf_counter_ns()-start)/1e6)
            assert reply.accepted and reply.epoch==epoch+i+1
            assert reply.reason=='WAITING_FOR:controller,global_costmap,local_costmap,planner,policy,protection'
            if i%100==0:rss=max(rss,process_stats(p.proc.pid)[1])
        elapsed=time.perf_counter()-begin;cpu1,last_rss=process_stats(p.proc.pid)
        message=p.flush()
        assert message.epoch==epoch+count and not message.navigation_allowed
        assert message.request_id==f'benchmark-{count-1}'
        assert message.valid_until==p.state.valid_until
        return dict(implementation=impl,requests=count,elapsed_s=elapsed,cpu_s=cpu1-cpu0,
            sampled_rss_kib=max(rss,last_rss),p50_ms=statistics.median(latency),
            p95_ms=percentile(latency,.95),p99_ms=percentile(latency,.99),max_ms=max(latency),
            requests_per_s=count/elapsed,latency_ms=latency)
    finally:p.close()


from reference_bootstrap import source_or_reference


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--binary',required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--requests',type=int,default=1000)
    parser.add_argument('--pairs',type=int,default=3);args=parser.parse_args()
    assert args.requests>0 and args.pairs>0
    args.output.parent.mkdir(parents=True,exist_ok=True);logs=args.output.parent/'fixed_benchmark_logs';logs.mkdir(exist_ok=True)
    trials=[]
    for pair in range(args.pairs):
        for impl in (('python','cpp') if pair%2==0 else ('cpp','python')):
            result=trial(impl,args.binary,args.requests,logs,len(trials));trials.append(result)
            print({k:v for k,v in result.items() if k!='latency_ms'},flush=True)
    keys=('elapsed_s','cpu_s','sampled_rss_kib','p50_ms','p95_ms','p99_ms','requests_per_s')
    medians={impl:{key:statistics.median(t[key] for t in trials if t['implementation']==impl)
                    for key in keys} for impl in ('python','cpp')}
    native=Path(__file__).resolve().parents[1];packages=native.parent
    paths=[Path(__file__),native/'test/test_fixed_envelope_protocol.py',native/'test/test_fixed_envelope_direct.py',
           native/'test/test_envelope_protocol.py',
           native/'src/fixed_envelope_core.cpp',native/'src/fixed_envelope_node.cpp',native/'src/policy_profile.cpp',
           packages/'astribot_s1_robot_geometry/include/astribot_s1_robot_geometry/geometry_kernels.hpp',
           packages/'astribot_s1_navigation_policy/astribot_s1_navigation_policy/fixed_envelope.py',
           packages/'astribot_s1_navigation_policy/astribot_s1_navigation_policy/fixed_envelope_node.py',
           packages/'astribot_s1_navigation_policy/astribot_s1_navigation_policy/envelope_node.py',
           packages/'astribot_s1_navigation_policy/config/simulation.json']
    report=dict(scope=__doc__,domain=116,localhost_only=True,host=platform.platform(),
        python=platform.python_version(),ros_distro=os.environ.get('ROS_DISTRO'),
        rmw=os.environ.get('RMW_IMPLEMENTATION','ROS default'),
        timestamp_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
        source_sha256={str(source_or_reference(p)):hashlib.sha256(source_or_reference(p).read_bytes()).hexdigest() for p in paths},
        binary_sha256=hashlib.sha256(Path(args.binary).read_bytes()).hexdigest(),
        median_by_implementation=medians,trials=trials)
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(medians,indent=2))


if __name__=='__main__':main()
