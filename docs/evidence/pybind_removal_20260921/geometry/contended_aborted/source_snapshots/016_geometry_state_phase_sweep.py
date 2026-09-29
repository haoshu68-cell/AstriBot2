#!/usr/bin/env python3
"""Paired virtual-clock diagnosis separating source age, compute and completion polling.

For every variant the fixture replays exactly the same integer-nanosecond /clock
and JointState acquisition sequences. Joint packets precede their corresponding
clock update by 5 ms wall time, exercising the existing bounded future queue.
A requested source-age phase is 0/5/10/15 ms. No source stamps are rewritten by
producers; TTL, timer period, model and geometry remain unchanged.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import hashlib
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rosidl_runtime_py.convert import message_to_ordereddict
from benchmark_geometry_state_ab import Fixture,ns,percentiles,digest
from astribot_s1_robot_geometry.polygon import contains,geometry_hash


class PhaseFixture(Fixture):
    def publish(self):
        pass

    def __init__(self):
        super().__init__()
        for timer in list(self.timers):self.destroy_timer(timer)
        self.clock_pub=self.create_publisher(Clock,'/clock',10)

    def until(self,deadline,process):
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError(f'producer exited during replay: {process.returncode}')
            rclpy.spin_once(self,timeout_sec=min(.002,max(0,deadline-time.monotonic())))

    def replay(self,variant,scenario,phase,args):
        self.configure(scenario);root=Path(args.output);identifier=f'{scenario}_phase{phase:02d}_{variant}'
        command=([sys.executable,str(Path(__file__).with_name('geometry_timing_python_wrapper.py'))] if variant=='python' else [args.cpp])
        command+=['--ros-args','-p','base_frame:=base','-p','use_sim_time:=true']
        environment=dict(os.environ);environment['PYTHONPATH']=args.python_root+os.pathsep+environment.get('PYTHONPATH','')
        log=root/(identifier+'.log');clocks=[]
        with log.open('w') as stream:
            process=subprocess.Popen(command,env=environment,stdout=stream,stderr=subprocess.STDOUT)
            try:
                origin=time.monotonic()+.05;base=2_000_000_000;step=20_000_000
                warmup=round(args.warmup/.02);count=round(args.seconds/.02);cycles=warmup+count+3
                for i in range(cycles):
                    expected=origin+i*.02;self.until(expected,process)
                    source=base+i*step-phase*1_000_000
                    message=JointState();message.header.stamp=Time(sec=source//10**9,nanosec=source%10**9);message.name=['arm'];message.position=[.2]
                    self.inputs.append({'sequence':i+1,'source_ns':source,'send_monotonic_ns':time.monotonic_ns()});self.joints.publish(message)
                    self.until(expected+.005,process)
                    now=base+i*step;clock=Clock();clock.clock=Time(sec=now//10**9,nanosec=now%10**9)
                    clocks.append({'clock_ns':now,'send_monotonic_ns':time.monotonic_ns(),'wall_schedule_lateness_ms':1000*(time.monotonic()-(expected+.005))});self.clock_pub.publish(clock)
                    if self.current_ack:self.ack.publish(String(data=self.current_ack))
                self.until(origin+cycles*.02,process)
                mappings=Path(f'/proc/{process.pid}/maps').read_text();(root/(identifier+'.maps')).write_text(mappings)
                actual_environment=Path(f'/proc/{process.pid}/environ').read_bytes().split(b'\0');assert b'ROS_DOMAIN_ID=115' in actual_environment
            finally:
                if process.poll() is None:process.send_signal(signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
        start=base+warmup*step;end=start+count*step
        inputs={r['source_ns']:r for r in self.inputs};events=[];by_source={}
        for line in log.read_text().splitlines():
            if line.startswith('GEOMETRY_TIMING '):
                event=json.loads(line[len('GEOMETRY_TIMING '):]);events.append(event)
                by_source.setdefault(event['source_ns'],{})[event['event']]=event
        outputs=[];timings=[];failures=[];incomplete=[];sequence_gaps=0;previous=None
        for message,received,received_wall in self.outputs:
            if not start<=ns(message.published_at)<end:continue
            frame=message_to_ordereddict(message);frame['receive_steady_ns']=received;outputs.append(frame)
            if previous is not None:sequence_gaps+=max(0,message.sequence-previous-1)
            previous=message.sequence
            if not message.complete:incomplete.append(message.reason);continue
            source=ns(message.header.stamp);sample=inputs.get(source);trace=by_source.get(source,{})
            if sample is None:failures.append('unknown original source');continue
            checks={
                'original_source':len(message.joint_source_stamps)==1 and ns(message.joint_source_stamps[0])==source,
                'joint_values':list(message.joints.name)==['arm'] and list(message.joints.position)==[.2],
                'joint_error_bound':list(message.joint_position_error_bounds)==[.003],
                'height':abs(message.height_m-self.expected['height'])<=1e-12,
                'attachment_ids':list(message.attachment_ids)==[p.object.id for p in self.payload],
                'source_deadline':source<=ns(message.published_at)<ns(message.valid_until)<=source+300_000_000,
                'model_revision':message.model_revision==self.model.revision,
                'attachment_revision':message.attachment_revision==self.expected_revision,
                'attachment_confirmed':message.attachment_state_confirmed,
                'complete_trace':all(k in trace for k in ('submit','compute_begin','compute_end','publish')),
            }
            for name in ('physical','reserved'):
                points=[[p.x,p.y] for p in getattr(message,name+'_footprint').points]
                checks[name+'_exact_wire']=np.array_equal(points,self.expected_wire[name])
                checks[name+'_conservative_containment']=contains(points,self.expected[name],tolerance=0.)
                checks[name+'_hash']=geometry_hash(points,'base',0.)==self.expected_hashes[name]
            checks['height_slices_count']=len(message.height_slices)==len(self.expected['slices'])
            if checks['height_slices_count']:
                for i,(actual,expected) in enumerate(zip(message.height_slices,self.expected['slices'])):
                    checks[f'slice_{i}_height']=actual.z_min_m==expected['z_min'] and actual.z_max_m==expected['z_max']
                    checks[f'slice_{i}_conservative_containment']=contains([[p.x,p.y] for p in actual.footprint.points],expected['footprint'],tolerance=0.)
            for name,passed in checks.items():
                if not passed:failures.append(f'sequence={message.sequence}:{name}')
            frame['checks']=checks;frame['source_sequence']=sample['sequence']
            if checks['complete_trace']:
                t=trace
                timing={'source_sequence':sample['sequence'],'source_ns':source,'source_age_at_submit_ms':(t['submit']['ros_ns']-source)/1e6,
                        'dispatch_ms':(t['compute_begin']['steady_ns']-t['submit']['steady_ns'])/1e6,
                        'compute_ms':(t['compute_end']['steady_ns']-t['compute_begin']['steady_ns'])/1e6,
                        'result_wait_ms':(t['publish']['steady_ns']-t['compute_end']['steady_ns'])/1e6,
                        'submit_to_publish_wall_ms':(t['publish']['steady_ns']-t['submit']['steady_ns'])/1e6,
                        'source_to_publish_ros_ms':(ns(message.published_at)-source)/1e6}
                timings.append(timing)
        metrics={key:percentiles([t[key] for t in timings]) for key in ('source_age_at_submit_ms','dispatch_ms','compute_ms','result_wait_ms','submit_to_publish_wall_ms','source_to_publish_ros_ms')}
        maxima={key:max(t[key] for t in timings) if timings else None for key in metrics}
        source_sequence=[(i['sequence'],i['source_ns']) for i in self.inputs]
        summary={'id':identifier,'variant':variant,'scenario':scenario,'source_phase_ms':phase,'measurement_ros_start_ns':start,'measurement_ros_end_ns':end,
                 'measurement_s':args.seconds,'input_hz':50.,'complete_frames':len(timings),'incomplete_reasons':incomplete,
                 'output_sequence_gaps':sequence_gaps,'validation_failures':failures,'metrics':metrics,'maxima':maxima,
                 'source_sequence_sha256':hashlib.sha256(json.dumps(source_sequence).encode()).hexdigest(),
                 'clock_sequence_sha256':hashlib.sha256(json.dumps([c['clock_ns'] for c in clocks]).encode()).hexdigest(),
                 'max_clock_send_lateness_ms':max(c['wall_schedule_lateness_ms'] for c in clocks),'child_returncode_after_requested_shutdown':process.returncode}
        raw={'summary':summary,'inputs':self.inputs,'clocks':clocks,'outputs':outputs,'timings':timings,'trace_events':events,'command':command,'pid':process.pid}
        (root/(identifier+'.json')).write_text(json.dumps(raw,indent=2));print(json.dumps(summary),flush=True);return summary


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--python-root',required=True);parser.add_argument('--cpp',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--diagnostic-source',required=True);parser.add_argument('--diagnostic-build',required=True)
    parser.add_argument('--phases',default='0,5,10,15');parser.add_argument('--warmup',type=float,default=2.);parser.add_argument('--seconds',type=float,default=3.)
    args=parser.parse_args();assert os.environ.get('ROS_DOMAIN_ID')=='115';root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    files=[Path(__file__),Path(__file__).with_name('geometry_timing_python_wrapper.py'),Path(__file__).with_name('benchmark_geometry_state_ab.py'),Path(args.cpp)]
    manifest={'mode':'Diagnostic timing instrumentation, not resource benchmark','domain':115,'argv':sys.argv,
              'sha256':{str(p.resolve()):digest(p) for p in files},'source_age_phases_ms':list(map(int,args.phases.split(','))),
              'input_period_ns':20_000_000,'early_joint_delivery_wall_ms':5,
              'clock_semantics':'Identical absolute simulated nanosecond sequences for each paired variant; /clock advances 20ms every 20ms wall. Future packets arrive before the next clock update and retain original acquisition stamps.',
              'instrumentation':'Only independent diagnostic C++ copy and Python wrapper log submit/compute/publish timestamps. Production node source, periods, TTL and input stamps are unchanged.'}
    source_files=sorted(p for p in Path(args.diagnostic_source).rglob('*') if p.is_file())
    baseline_files=sorted(p for p in Path(args.python_root).rglob('*') if p.is_file() and '__pycache__' not in str(p))
    manifest['diagnostic_and_baseline_sha256']={str(p.resolve()):digest(p) for p in source_files+baseline_files}
    manifest['diagnostic_build_files']={str(p.relative_to(args.diagnostic_build)):p.read_text() for p in Path(args.diagnostic_build).rglob('*.txt') if p.name in ('flags.make','link.txt','CMakeCache.txt')}
    snapshots=root/'diagnostic_source';snapshots.mkdir(exist_ok=True)
    for p in source_files:
        target=snapshots/p.relative_to(args.diagnostic_source);target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(p.read_bytes())
    manifest['diagnostic_flags']=(Path(args.diagnostic_build)/'CMakeFiles/geometry_state_timing.dir/flags.make').read_text()
    (root/'manifest.json').write_text(json.dumps(manifest,indent=2))
    for p in files[:-1]:(root/p.name).write_bytes(p.read_bytes())
    results=[];rclpy.init();fixture=PhaseFixture()
    try:
        for scenario in ('empty','offset_payload'):
            for index,phase in enumerate(manifest['source_age_phases_ms']):
                for variant in (('python','cpp') if index%2==0 else ('cpp','python')):
                    results.append(fixture.replay(variant,scenario,phase,args));(root/'summary.json').write_text(json.dumps(results,indent=2))
    finally:fixture.destroy_node();rclpy.shutdown()
    for scenario in ('empty','offset_payload'):
        for phase in manifest['source_age_phases_ms']:
            pair=[r for r in results if r['scenario']==scenario and r['source_phase_ms']==phase]
            for key in ('source_sequence_sha256','clock_sequence_sha256'):
                assert len({r[key] for r in pair})==1,(scenario,phase,key)
    if any(r['validation_failures'] or r['incomplete_reasons'] or not r['complete_frames'] for r in results):raise SystemExit(2)


if __name__=='__main__':main()
