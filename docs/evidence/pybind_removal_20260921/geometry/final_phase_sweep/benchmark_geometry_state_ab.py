#!/usr/bin/env python3
"""Owned domain-115 protocol/performance A/B, with immutable baseline artifacts.

Python subprocess is the retained original ROS/model adapter with the original
native extension rebuilt separately; C++ is the direct Release producer. No
runtime references, defaults, shared installations or external processes change.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from collections import Counter
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene, AttachedCollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from astribot_navigation_msgs.msg import RobotGeometryState
from rosidl_runtime_py.convert import message_to_ordereddict
from astribot_s1_robot_geometry.model import RobotModel, Shape
from astribot_s1_robot_geometry.polygon import contains, geometry_hash
from astribot_s1_robot_geometry.node import polygon
from test_cpp_robot_model import URDF


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ns(stamp):
    return stamp.sec*10**9+stamp.nanosec


def percentiles(values):
    if len(values)==0:return None
    return dict(zip(('p50','p95','p99'),map(float,np.percentile(values,[50,95,99]))))


def process_sample(pid):
    fields=Path(f'/proc/{pid}/stat').read_text().split(')',1)[1].split()
    rss=int(fields[21])*os.sysconf('SC_PAGE_SIZE')
    return {'monotonic_ns':time.monotonic_ns(),'cpu_ticks':int(fields[11])+int(fields[12]),'rss_bytes':rss}


class Fixture(Node):
    def __init__(self):
        super().__init__('geometry_state_ab_fixture')
        self.payload=[];self.current_ack='';self.inputs=[];self.outputs=[];self.observed_attachments=[]
        self.coverage=dict(input_cloud_topic='/map_scan',base_frame='base',enable_outlier_filter=False)
        for i,name in enumerate(('low_obstacle','main_nav','torso_high','overhead')):
            for field,value in dict(enabled=True,z_min=-.03 if i==0 else i*.6,z_max=(i+1)*.6,min_points=1).items():self.coverage[f'slices.{name}.{field}']=value
        self.create_service(GetParameters,'/robot_state_publisher/get_parameters',self.parameters)
        self.create_service(GetParameters,'/pointcloud_slice_scan_node/get_parameters',self.parameters)
        self.create_service(GetPlanningScene,'/get_planning_scene',self.scene)
        self.ack=self.create_publisher(String,'/navigation/attachment_filter_applied',10)
        self.joints=self.create_publisher(JointState,'/joint_states',qos_profile_sensor_data)
        self.create_subscription(PlanningScene,'/navigation/attached_geometry',self.attachment,10)
        self.create_subscription(RobotGeometryState,'/navigation/geometry_state',self.output,10)
        self.create_timer(.02,self.publish)  # identical fixed 50 Hz fixture for both variants
        self.model=RobotModel(URDF,'base')

    def configure(self,scenario):
        self.payload=[];shapes=[]
        if scenario=='offset_payload':
            item=AttachedCollisionObject(link_name='arm_link')
            item.object.id='offset_payload';item.object.header.frame_id='arm_link';item.object.pose.orientation.w=1.
            item.object.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.3,.25,.2])]
            pose=Pose();pose.orientation.w=1.;pose.position.x=.8;pose.position.y=-.3;pose.position.z=.2
            item.object.primitive_poses=[pose];self.payload=[item]
            transform=np.eye(4);transform[:3,3]=[.8,-.3,.2];shapes=[Shape('arm_link','box',[.3,.25,.2],transform)]
        self.expected_revision=hashlib.sha256(json.dumps([message_to_ordereddict(x) for x in self.payload],sort_keys=True).encode()).hexdigest()
        self.expected=self.model.geometry({'arm':.2},{'arm':.003},shapes)
        self.expected_wire={name:[[p.x,p.y] for p in polygon(self.expected[name]).points] for name in ('physical','reserved')}
        self.expected_hashes={name:geometry_hash(points,'base',0.) for name,points in self.expected_wire.items()}
        self.current_ack='';self.inputs=[];self.outputs=[];self.observed_attachments=[]

    def parameters(self,request,response):
        response.values=[Parameter(name,value=URDF if name=='robot_description' else self.coverage[name]).to_parameter_msg().value for name in request.names]
        return response

    def scene(self,request,response):
        response.scene.robot_state.attached_collision_objects=self.payload;return response

    def attachment(self,message):
        self.current_ack=message.name
        self.observed_attachments.append({'receive_monotonic_ns':time.monotonic_ns(),'revision':message.name,'stamp_ns':ns(message.robot_state.joint_state.header.stamp)})
        self.ack.publish(String(data=self.current_ack))

    def publish(self):
        message=JointState();message.header.stamp=self.get_clock().now().to_msg();message.name=['arm'];message.position=[.2]
        self.inputs.append({'sequence':len(self.inputs)+1,'source_ns':ns(message.header.stamp),'send_monotonic_ns':time.monotonic_ns(),'position':.2})
        self.joints.publish(message)
        if self.current_ack:self.ack.publish(String(data=self.current_ack))

    def output(self,message):
        self.outputs.append((message,time.monotonic_ns(),time.time_ns()))

    def spin_for(self,seconds,process,samples):
        end=time.monotonic()+seconds;next_sample=0.
        while time.monotonic()<end:
            if process.poll() is not None:raise RuntimeError(f'owned producer exited {process.returncode}')
            rclpy.spin_once(self,timeout_sec=min(.01,max(0,end-time.monotonic())))
            if time.monotonic()>=next_sample:
                samples.append(process_sample(process.pid));next_sample=time.monotonic()+.1

    def run(self,variant,scenario,group,args):
        self.configure(scenario)
        root=Path(args.output);identifier=f'g{group}_{scenario}_{variant}';logpath=root/(identifier+'.log')
        env=dict(os.environ);env['PYTHONPATH']=str(Path(args.python_root).resolve())+os.pathsep+env.get('PYTHONPATH','')
        command=([sys.executable,str(Path(__file__).with_name('geometry_python_oracle.py'))] if variant=='python' else [str(Path(args.cpp).resolve())])
        command+=['--ros-args','-p','base_frame:=base']
        with logpath.open('w') as logfile:
            process=subprocess.Popen(command,env=env,stdout=logfile,stderr=subprocess.STDOUT)
            resources=[]
            try:
                self.spin_for(args.warmup,process,resources)
                environment=Path(f'/proc/{process.pid}/environ').read_bytes().split(b'\0')
                assert b'ROS_DOMAIN_ID=115' in environment
                mappings=Path(f'/proc/{process.pid}/maps').read_text();(root/(identifier+'.maps')).write_text(mappings)
                if variant=='cpp':assert 'libpython' not in mappings and '_geometry_native' not in mappings
                else:assert str(Path(args.python_root).resolve()) in mappings and '_geometry_native' in mappings
                initial=process_sample(process.pid);start=initial['monotonic_ns'];resources=[initial]
                self.spin_for(args.seconds,process,resources)
                final=process_sample(process.pid);end=final['monotonic_ns'];resources.append(final)
                # Drain in-flight frames without treating the drain as a measurement interval.
                self.spin_for(.1,process,[])
            finally:
                if process.poll() is None:process.send_signal(signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
        measured=[(m,received,wall) for m,received,wall in self.outputs if start<=received<end]
        inputs={x['source_ns']:x for x in self.inputs};measured_inputs=[x for x in self.inputs if start<=x['send_monotonic_ns']<end]
        failures=[];frames=[];reasons=Counter();matched=0;used_sources=set();publish_delay=[];receive_delay=[];complete_count=0;hashes=Counter()
        previous_sequence=None;sequence_gaps=0;exact_wire_frames=0
        for message,received,received_wall in measured:
            frame=message_to_ordereddict(message);frame['receive_monotonic_ns']=received;frame['receive_wall_ns']=received_wall
            frame['matched_input_sequence']=None;reasons[message.reason]+=1
            if previous_sequence is not None and message.sequence>previous_sequence+1:sequence_gaps+=message.sequence-previous_sequence-1
            previous_sequence=message.sequence
            if message.complete:
                complete_count+=1;source=ns(message.header.stamp);sample=inputs.get(source)
                if sample is None:failures.append(f'unknown original source stamp {source}')
                else:
                    matched+=1;used_sources.add(source);frame['matched_input_sequence']=sample['sequence']
                    publish_delay.append((ns(message.published_at)-source)/1e6)
                    receive_delay.append((received-sample['send_monotonic_ns'])/1e6)
                checks={
                    'model_revision':message.model_revision==self.model.revision,
                    'attachment_revision':message.attachment_revision==self.expected_revision,
                    'attachment_confirmed':message.attachment_state_confirmed,
                    'attachment_ids':list(message.attachment_ids)==[p.object.id for p in self.payload],
                    'source_deadline':source<=ns(message.published_at)<ns(message.valid_until)<=source+300_000_000,
                    'joint_source':len(message.joint_source_stamps)==1 and ns(message.joint_source_stamps[0])==source,
                    'joint_values':list(message.joints.name)==['arm'] and list(message.joints.position)==[.2],
                    'joint_error_bound':list(message.joint_position_error_bounds)==[.003],
                    'height':abs(message.height_m-self.expected['height'])<=1e-12,
                }
                exact=True
                for name in ('physical','reserved'):
                    points=[[p.x,p.y] for p in getattr(message,name+'_footprint').points]
                    checks[name+'_conservative_containment']=contains(points,self.expected[name],tolerance=0.)
                    frame[name+'_hash']=geometry_hash(points,'base',0.)
                    checks[name+'_hash']=frame[name+'_hash']==self.expected_hashes[name]
                    exact=exact and np.array_equal(points,self.expected_wire[name])
                exact_wire_frames+=int(exact)
                hashes[(frame['physical_hash'],frame['reserved_hash'])]+=1
                checks['height_slices_count']=len(message.height_slices)==len(self.expected['slices'])
                if checks['height_slices_count']:
                    for i,(actual,expected) in enumerate(zip(message.height_slices,self.expected['slices'])):
                        checks[f'slice_{i}_height']=actual.z_min_m==expected['z_min'] and actual.z_max_m==expected['z_max']
                        checks[f'slice_{i}_conservative_containment']=contains([[p.x,p.y] for p in actual.footprint.points],expected['footprint'],tolerance=0.)
                for name,passed in checks.items():
                    if not passed:failures.append(f'sequence={message.sequence} {name}')
                frame['checks']=checks
            frames.append(frame)
        duration=(end-start)/1e9;cpu_s=(final['cpu_ticks']-initial['cpu_ticks'])/os.sysconf('SC_CLK_TCK')
        input_stamps=[x['send_monotonic_ns'] for x in measured_inputs]
        incomplete=len(measured)-complete_count
        summary={
            'id':identifier,'group':group,'scenario':scenario,'variant':variant,'warmup_s':args.warmup,'measurement_s':duration,
            'scheduled_input_hz':50.,'published_input_count':len(measured_inputs),'published_input_hz':len(measured_inputs)/duration,
            'input_interarrival_ms':percentiles(np.diff(input_stamps)/1e6),
            'received_frames':len(measured),'complete_frames':complete_count,'incomplete_frames':incomplete,
            'complete_output_hz':complete_count/duration,'source_matched_complete_frames':matched,'unique_used_sources':len(used_sources),
            'inputs_not_selected_by_latest_state_sampling':len({x['source_ns'] for x in measured_inputs}-used_sources),
            'output_sequence_gaps':sequence_gaps,'reason_counts':dict(reasons),'validation_failures':failures,
            'exact_float32_wire_frames':exact_wire_frames,'wire_hash_counts':[{'physical':p,'reserved':r,'frames':n} for (p,r),n in hashes.items()],
            'cpu_seconds':cpu_s,'cpu_percent_one_core':100*cpu_s/duration,
            'rss_bytes':percentiles([x['rss_bytes'] for x in resources]),'rss_peak_bytes':max(x['rss_bytes'] for x in resources),
            'source_to_publish_ms':percentiles(publish_delay) if matched==complete_count and complete_count else None,
            'source_to_receive_ms':percentiles(receive_delay) if matched==complete_count and complete_count else None,
            'all_measured_complete_frames_valid':not failures and matched==complete_count and complete_count>0,
            'performance_comparison_eligible':not failures and incomplete==0 and sequence_gaps==0 and matched==complete_count and complete_count>0,
            'latency_definition':'Original JointState acquisition stamp exactly matched to published RobotGeometryState.header.stamp; publish includes scheduling+compute, receipt additionally DDS+fixture callback.',
            'sequence_gap_limit':'A gap may be an abandoned compute job or transport loss; this probe does not label it as DDS loss.',
            'unused_input_limit':'50Hz source packets are intentionally selected into approximately 10Hz geometry jobs; unused packets are not interpreted as dropped output frames.'}
        raw={'summary':summary,'command':command,'pid':process.pid,'measurement_start_monotonic_ns':start,'measurement_end_monotonic_ns':end,
             'expected_model_revision':self.model.revision,'expected_attachment_revision':self.expected_revision,'expected_hashes':self.expected_hashes,
             'expected_physical':self.expected['physical'].tolist(),'expected_reserved':self.expected['reserved'].tolist(),
             'inputs':self.inputs,'frames':frames,'resource_samples':resources,'attachment_observations':self.observed_attachments,'log_path':str(logpath)}
        (root/(identifier+'.json')).write_text(json.dumps(raw,indent=2))
        print(json.dumps(summary),flush=True);return summary


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--python-root',required=True);parser.add_argument('--cpp',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--cpp-build-directory');parser.add_argument('--baseline-build-command')
    parser.add_argument('--groups',type=int,default=3);parser.add_argument('--warmup',type=float,default=2.);parser.add_argument('--seconds',type=float,default=6.)
    args=parser.parse_args();assert os.environ.get('ROS_DOMAIN_ID')=='115';root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    import astribot_s1_robot_geometry.model as model_module
    import astribot_s1_robot_geometry._geometry_native as native_module
    assert str(Path(args.python_root).resolve()) in str(Path(model_module.__file__).resolve()), 'fixture oracle must use immutable Python baseline'
    source_root=Path(__file__).parents[1]
    files=[Path(args.cpp),Path(__file__),Path(__file__).with_name('geometry_python_oracle.py'),*Path(args.python_root).rglob('*.py'),*Path(args.python_root).rglob('*.so'),
           *source_root.glob('include/astribot_s1_robot_geometry/*.hpp'),source_root/'src/geometry_state_node.cpp',source_root/'CMakeLists.txt']
    build_details={}
    if args.baseline_build_command:
        command_file=Path(args.baseline_build_command);command=json.loads(command_file.read_text());files.append(command_file)
        build_details['python_extension_command']=command
        files.extend(Path(value) for value in command if value.endswith('.cpp') and Path(value).is_file())
        dependency_file=command_file.parent/'baseline-dependencies.txt'
        if dependency_file.is_file():
            files.append(dependency_file);build_details['baseline_dependencies']=dependency_file.read_text()
        files.extend(command_file.parent.glob('baseline_include/astribot_s1_robot_geometry/*.hpp'))
    if args.cpp_build_directory:
        build_directory=Path(args.cpp_build_directory)
        for name in ('flags.make','link.txt'):
            path=build_directory/'CMakeFiles/geometry_state.dir'/name;files.append(path);build_details['cpp_'+name]=path.read_text()
    (root/'benchmark_script.py').write_bytes(Path(__file__).read_bytes())
    sources=root/'source_snapshots';sources.mkdir(exist_ok=True)
    source_snapshots={}
    for i,path in enumerate(files):
        if path.suffix in ('.cpp','.hpp','.py'):
            snapshot=sources/f'{i:03d}_{path.name}';snapshot.write_bytes(path.read_bytes());source_snapshots[str(path.resolve())]=str(snapshot)
    manifest={'created_at_unix_ns':time.time_ns(),'argv':sys.argv,'domain':115,'python_version':sys.version,'python_executable':sys.executable,
              'loaded_oracle_model':model_module.__file__,'loaded_oracle_extension':native_module.__file__,
              'compiler':subprocess.check_output(['g++','--version'],text=True).splitlines()[0],
              'kernel':os.uname().release,'machine':os.uname().machine,'cpu_count':os.cpu_count(),'clock_ticks_per_second':os.sysconf('SC_CLK_TCK'),
              'env':{k:os.environ.get(k) for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')},
              'sha256':{str(p.resolve()):digest(p) for p in files},'input_joint_hz':50.,'posture':{'arm':.2},'geometry_hold_error':.003,
              'order_policy':'Odd group Python then C++; even group C++ then Python. Empty and offset payload each group.',
              'build_details':build_details,'source_snapshots':source_snapshots}
    (root/'manifest.json').write_text(json.dumps(manifest,indent=2))
    results=[];rclpy.init();node=Fixture()
    try:
        for group in range(1,args.groups+1):
            for scenario in ('empty','offset_payload'):
                for variant in (('python','cpp') if group%2 else ('cpp','python')):
                    results.append(node.run(variant,scenario,group,args))
                    (root/'summary.json').write_text(json.dumps(results,indent=2))
    finally:node.destroy_node();rclpy.shutdown()
    if any(not r['all_measured_complete_frames_valid'] for r in results):raise SystemExit(2)


if __name__=='__main__':main()
