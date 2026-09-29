#!/usr/bin/env python3
"""Record the existing fixed-posture corridor protocol in an owned simulation."""
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

from run_regression import stop_owned
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from run_waypoint_route import metrics,project,wrap,yaw


def wall_clearance(bounds,width):
    low,high=bounds
    if len(low)!=3 or len(high)!=3 or not all(math.isfinite(v) for v in low+high):
        raise ValueError('Invalid independent swept bounds')
    if any(a>b for a,b in zip(low,high)):raise ValueError('Reversed swept bounds')
    values=[]
    for side in (-1,1):
        center=side*(width/2+.05)
        dx=max(-.6-high[0],0.,low[0]-.6)
        dy=max(center-.05-high[1],0.,low[1]-center-.05)
        values.append(math.hypot(dx,dy))
    return min(values)


def assess_records(case,protocol,events,records,samples,returncode):
    report={}
    starts=[r for r in events if r['stage']=='CORRIDOR_EXECUTION']
    ends=[r for r in events if r.get('navigation_succeeded') and
          starts and r['wall_time']>starts[0]['wall_time']]
    arrivals=[r for r in events if 'navigation_position_error_m' in r]
    checks=dict(protocol_passed=protocol.get('passed') is True and returncode==0,
        original_cases_executed=len(protocol.get('corridor_cases',[]))>=3,
        selected_corridor_executed=len(starts)==1 and len(ends)==1,
        precise_arrivals=len(arrivals)>=2 and all(r['navigation_position_error_m']<=.002 and
            r['navigation_yaw_error_rad']<=math.radians(.1) for r in arrivals))
    if checks['selected_corridor_executed']:
        start,end=starts[0]['wall_time'],ends[0]['wall_time']
        geometry_rows=[r for r in records if r['kind']=='geometry' and start<=r['wall_s']<=end]
        stamps=[r['stamp_ns']*1e-9 for r in geometry_rows]
        checks['continuous_physical_geometry']=len(stamps)>10 and all(0<b-a<=.15 for a,b in zip(stamps,stamps[1:])) and all(
            r['geometry_valid'] and r['robot_collision_count']>0 and 'robot_swept_bounds' in r for r in geometry_rows)
        if checks['continuous_physical_geometry']:
            clearances=[wall_clearance(r['robot_swept_bounds'],case['width_m']) for r in geometry_rows]
            report['minimum_wall_clearance_lower_bound_m']=min(clearances)
            checks['full_body_clearance']=min(clearances)>=.08
        else:checks['full_body_clearance']=False
        selected=[r for r in samples if start<=r['wall_s']<=end]
        report['tracking_metrics']=metrics(selected)
        checks['follow_measurement_present']=report['tracking_metrics']['follow_samples']>=10
    report.update(protocol=protocol,arrivals=arrivals,checks=checks)
    report['failed_checks']=[k for k,v in checks.items() if not v]
    report['scenario_passed']=all(checks.values())
    report['verdict']='PASS' if report['scenario_passed'] else 'FAIL'
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',type=Path,required=True)
    p.add_argument('--session',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--policy',choices=['off'],default='off')
    p.add_argument('--capture-barrier',type=Path,
        help='Optional camera probe report; wait for its hold window after support startup')
    p.add_argument('--external-support',type=Path,
        help='Owned persistent transport support manifest; never stopped by this probe')
    p.add_argument('--no-view-capture',action='store_true',
        help='Skip optional GUI capture during measured compute workloads')
    a=p.parse_args();case=json.loads(a.case.read_text());session=json.loads(a.session.read_text())
    identity=session.get('isolation',{})
    if (session.get('state')!='ready' or not identity.get('ASTRIBOT_SIM_INSTANCE') or
        identity.get('ROS_DOMAIN_ID') in (None,'25') or
        any(os.environ.get(k)!=identity.get(k) for k in ('ROS_DOMAIN_ID','IGN_PARTITION','GZ_PARTITION'))):
        raise RuntimeError('Matching isolated simulation required')
    owner=Path('/proc')/str(session['supervisor_pid'])
    if not owner.exists() or b'sim_stack_supervisor.py' not in (owner/'cmdline').read_bytes():
        raise RuntimeError('Simulation owner unavailable')
    root=Path(__file__).resolve().parents[2]
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'case.json').write_text(json.dumps(case,indent=2)+'\n')
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from nav_msgs.msg import Odometry,Path as RosPath
    from std_msgs.msg import String,Float64MultiArray
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Twist
    from moveit_msgs.msg import PlanningScene
    from action_msgs.msg import GoalStatusArray
    from tf2_ros import Buffer,TransformListener
    from astribot_navigation_msgs.msg import (NavigationExecutionStatus,RobotGeometryState,
                                             NavigationEnvelopeV2,EnvelopeApplyStatus,MotionConstraint,
                                             CorridorAlignment)
    from rosidl_runtime_py.convert import message_to_ordereddict
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('corridor_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node)
    state={};records=[];samples=[];support=worker=capture=None;logs=[]
    report=dict(case=case['id'],scenario_passed=False,verdict='INFRA_FAILURE',failed_checks=[])
    raw=(a.output/'observations.jsonl').open('w')
    def record(kind,value):
        row=dict(kind=kind,wall_s=time.time(),ros_s=node.get_clock().now().nanoseconds*1e-9,**value)
        records.append(row);raw.write(json.dumps(row,allow_nan=False)+'\n')
    def geometry(m):
        value=json.loads(m.data);record('geometry',value)
        try:
            tf=buffer.lookup_transform('map','astribot_torso_base',rclpy.time.Time())
            stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
            now=node.get_clock().now().nanoseconds*1e-9
            od=state['odom'];osrc=od.header.stamp.sec+od.header.stamp.nanosec*1e-9
            if not 0<=now-stamp<=.3 or not 0<=now-osrc<=.3:return
            if samples and stamp<=samples[-1]['sim_s']:return
            x,y=tf.transform.translation.x,tf.transform.translation.y
            heading=yaw(tf.transform.rotation);v=od.twist.twist
            row=dict(sim_s=stamp,pose_stamp_s=stamp,velocity_stamp_s=osrc,wall_s=time.time(),
                x=x,y=y,yaw=heading,speed=math.hypot(v.linear.x,v.linear.y),wz=v.angular.z,
                phase=state.get('phase','UNKNOWN') if 0<=now-state.get('phase_s',-1e9)<=.3 else 'UNKNOWN',
                revision=state.get('revision',0),cross_track_m=None,heading_error_deg=None,
                progress_m=None,reference_curvature=None)
            pr=project(state.get('path',[]),x,y)
            if pr:
                _,ct,tangent,arc,k=pr
                row.update(cross_track_m=ct,heading_error_deg=math.degrees(wrap(heading-tangent)),
                    progress_m=arc,reference_curvature=k)
            samples.append(row)
        except Exception as error:
            state['sample_error']=str(error)
    def plan(m):
        state['path']=[(p.pose.position.x,p.pose.position.y) for p in m.poses] if m.header.frame_id=='map' else []
        state['revision']=state.get('revision',0)+1
        record('plan',dict(frame=m.header.frame_id,points=state['path'],revision=state['revision']))
    def phase(m):state.update(phase=m.data,phase_s=node.get_clock().now().nanoseconds*1e-9)
    node.create_subscription(String,'/social_sim/state',geometry,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/odom',lambda m:state.update(odom=m),qos_profile_sensor_data)
    node.create_subscription(Twist,'/cmd_vel',lambda m:record('command',dict(
        v=math.hypot(m.linear.x,m.linear.y),vx=m.linear.x,vy=m.linear.y,w=m.angular.z)),10)
    wheel_names=tuple('wheel_'+name+'_Joint' for name in ('RF','LF','RR','LR'))
    def joints(m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if stamp<=state.get('wheel_stamp',-1)+.045:return
        state['wheel_stamp']=stamp
        selected=[(i,name) for i,name in enumerate(m.name) if name in wheel_names]
        record('wheels',dict(stamp_s=stamp,velocity={name:m.velocity[i] for i,name in selected
            if i<len(m.velocity)},effort={name:m.effort[i] for i,name in selected if i<len(m.effort)}))
    def efforts(m):
        now=node.get_clock().now().nanoseconds*1e-9
        if now<state.get('effort_s',-1)+.045:return
        state['effort_s']=now
        record('wheel_command',dict(order=list(wheel_names),effort_nm=list(m.data)))
    node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data)
    node.create_subscription(Float64MultiArray,'/wheel_effort_controller/commands',efforts,10)
    node.create_subscription(RosPath,'/plan',plan,10)
    node.create_subscription(String,'/path_tracking/phase',phase,qos_profile_sensor_data)
    def body(m):
        state['body']=m
        def seconds(t):return t.sec+t.nanosec*1e-9
        record('body_geometry',dict(complete=m.complete,reason=m.reason,
            revision=m.attachment_revision,source_s=seconds(m.header.stamp),
            published_s=seconds(m.published_at),valid_until_s=seconds(m.valid_until)))
    node.create_subscription(RobotGeometryState,'/navigation/geometry_state',body,10)
    node.create_subscription(String,'/navigation/attachment_filter_applied',
        lambda m:record('attachment_filter_ack',dict(revision=m.data)),10)
    node.create_subscription(PlanningScene,'/navigation/attached_geometry',
        lambda m:record('attachment_snapshot',dict(revision=m.name,
            source_s=m.robot_state.joint_state.header.stamp.sec+
                m.robot_state.joint_state.header.stamp.nanosec*1e-9)),10)
    def source_seconds(stamp):return stamp.sec+stamp.nanosec*1e-9
    node.create_subscription(NavigationEnvelopeV2,'/navigation/envelope_v2',lambda m:
        record('envelope',dict(epoch=m.epoch,session=m.coordinator_session_id,
            allowed=m.navigation_allowed,reason=m.reason,hash=m.installed_geometry_hash,
            source_s=source_seconds(m.header.stamp),valid_until_s=source_seconds(m.valid_until))),20)
    node.create_subscription(EnvelopeApplyStatus,'/navigation/envelope_applied',lambda m:
        record('envelope_ack',dict(consumer=m.consumer_id,epoch=m.envelope_epoch,
            session=m.coordinator_session_id,hash=m.installed_geometry_hash,
            applied=m.applied,reason=m.reason,source_s=source_seconds(m.header.stamp))),50)
    node.create_subscription(String,'/navigation_policy/observation',lambda m:
        record('policy_observation',json.loads(m.data)),10)
    node.create_subscription(String,'/navigation_policy/state',lambda m:
        record('policy_decision',json.loads(m.data)),10)
    node.create_subscription(String,'/navigation_policy/protection_state',lambda m:
        record('protection_state',json.loads(m.data)),10)
    node.create_subscription(MotionConstraint,'/navigation_policy/proposed_constraint',lambda m:
        record('proposed_constraint',dict(source_s=source_seconds(m.stamp),epoch=m.epoch,
            sequence=m.sequence,lease_s=m.lease_s,hold=m.hold,reason=m.reason,
            max_linear_speed=m.max_linear_speed,max_angular_speed=m.max_angular_speed)),10)
    recorded_paths=set()
    def reference_key(path):
        value=message_to_ordereddict(path)
        digest=hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()
        if digest not in recorded_paths:
            recorded_paths.add(digest);record('reference_path',dict(sha256=digest,path=value))
        return digest
    node.create_subscription(RosPath,'/path_tracking/active_path',lambda m:
        record('active_path',dict(reference_sha256=reference_key(m))),
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(CorridorAlignment,'/navigation_policy/corridor_alignment',lambda m:
        record('corridor_alignment',dict(source_s=source_seconds(m.stamp),lease_s=m.lease_s,
            reference_sha256=reference_key(m.reference_path),tracking=m.tracking_required,
            centering=m.centering_required,anchor=message_to_ordereddict(m.anchor))),10)
    node.create_subscription(NavigationExecutionStatus,'/navigation/execution_status',lambda m:
        record('execution',dict(task_id=m.task_id,state=m.state)),
        QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    active_actions={}
    for action in ('navigate_to_pose','navigate_through_poses'):
        node.create_subscription(GoalStatusArray,'/'+action+'/_action/status',lambda m,n=action:
            active_actions.update({n:any(s.status in (1,2,3) for s in m.status_list)}),
            QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def wait_for(predicate,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return
        raise TimeoutError('Corridor preflight/data deadline expired')
    def launch(arguments,name):
        log=(a.output/(name+'.log')).open('w');logs.append(log)
        env=os.environ.copy();env['ASTRIBOT_LOG_DIR']=str(a.output/'logs')
        return subprocess.Popen(arguments,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env)
    def interrupted(signum,frame):
        report['interrupted_signal']=signum
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
    try:
        wait_for(lambda:'odom' in state and records,15)
        settle=time.monotonic()+3
        wait_for(lambda:time.monotonic()>=settle,5)
        if any(active_actions.values()):
            raise RuntimeError('Another navigation task is active')
        external=None
        if a.external_support:
            external=json.loads(a.external_support.read_text())
            process=Path('/proc')/str(external['pid'])
            live=dict(x.split('=',1) for x in (process/'environ').read_text().split('\0') if '=' in x)
            if (external['instance']!=identity['ASTRIBOT_SIM_INSTANCE'] or
                any(live.get(k)!=identity.get(k) for k in ('ROS_DOMAIN_ID','ASTRIBOT_SIM_INSTANCE')) or
                (process/'stat').read_text().split(') ',1)[1].split()[19]!=external['start_ticks'] or
                b'transport_skills.launch.py' not in (process/'cmdline').read_bytes().split(b'\0')):
                raise RuntimeError('External transport support identity mismatch')
            report['external_support']=external
        else:
            support=launch([sys.executable,'-m','astribot_logging.launch_entry','launch',
                'astribot_s1_transport','transport_skills.launch.py'],'support')
        wait_for(lambda:state.get('body') is not None and state['body'].complete,60)
        (a.output/'support_ready.json').write_text(json.dumps(
            {'support_pid':support.pid if support else external['pid'],
             'ready_wall':time.time(),'navigation_goals_sent':0},indent=2))
        if a.capture_barrier:
            def capture_ready():
                try:
                    value=json.loads(a.capture_barrier.read_text())
                    return bool(value.get('hold_started_wall')) and not value.get('error') and time.time()-value['hold_started_wall']>=2.
                except (FileNotFoundError,json.JSONDecodeError):return False
            wait_for(capture_ready,60)
        command=[sys.executable,str(root/'tools/sim/verify_fixed_hold_expiry.py'),
            '--scenario',str(root/'ws_robot/src/astribot_s1_transport/config/warehouse_transfer.json'),
            '--output',str(a.output/'protocol'),'--prepare-entry','--prepare-compact',
            '--execute-corridor','--exit-through','--drive-width',str(case['width_m'])]
        if 'navigation_sim_timeout_s' in case:
            command+=['--navigation-sim-timeout',str(case['navigation_sim_timeout_s']),
                '--navigation-wall-watchdog',str(case.get('navigation_wall_watchdog_s',300))]
        worker=launch(command,'protocol')
        cancellation=None
        deadline=time.monotonic()+case.get('wall_watchdog_s',900)
        while worker.poll() is None and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            event_file=a.output/'protocol/events.jsonl'
            if (not a.no_view_capture and capture is None and not case.get('cancel_on_motion') and samples and
                    -.4<samples[-1]['x']<.4 and samples[-1]['speed']>.02 and
                    event_file.is_file() and '"stage": "CORRIDOR_EXECUTION"' in event_file.read_text()):
                (a.output/'during_passage').mkdir()
                capture=launch([sys.executable,str(root/'tools/social_navigation/capture_views.py'),
                    '--output',str(a.output/'during_passage'),
                    '--supervisor',str(session['supervisor_pid'])],'capture_during_passage')
            if (case.get('cancel_on_motion') and cancellation is None and samples and
                    samples[-1]['speed']>.04 and samples[-1]['x']>.25):
                cancellation=node.get_clock().now().nanoseconds*1e-9
                record('cancel_request',dict(signal=signal.SIGINT))
                worker.send_signal(signal.SIGINT)
        if worker.poll() is None:raise TimeoutError('Corridor execution wall watchdog')
        protocol=json.loads((a.output/'protocol/summary.json').read_text())
        events=[json.loads(s) for s in (a.output/'protocol/events.jsonl').read_text().splitlines()]
        # A rejected action legitimately produces no moving geometry samples.
        # Preserve the protocol failure instead of replacing it with a data timeout.
        if protocol.get('passed') or case.get('cancel_on_motion'):
            wait_for(lambda:len(samples)>0,5)
        if case.get('cancel_on_motion'):
            end=node.get_clock().now().nanoseconds*1e-9+2
            wait_for(lambda:node.get_clock().now().nanoseconds*1e-9>=end,15)
            tail=[r for r in samples if end-1<=r['sim_s']<=end]
            commands=[r for r in records if r['kind']=='command' and end-1<=r['ros_s']<=end]
            lease=json.loads(Path('/tmp/astribot_transport_domain_'+os.environ['ROS_DOMAIN_ID']+'.lock').read_text())
            statuses=[r for r in records if r['kind']=='execution' and cancellation is not None and r['ros_s']>=cancellation]
            canceled=next((r for r in statuses if r['state']=='CANCELED'),None)
            checks=dict(interruption_sent=cancellation is not None,
                protocol_canceled=protocol.get('interrupted_signal')==signal.SIGINT and not protocol.get('passed'),
                cleanup_confirmed='cleanup_error' not in protocol and lease.get('unconfirmed_executor') is False,
                task_canceled=canceled is not None,
                no_task_resumed=canceled is not None and not any(r['state'] in ('ACCEPTED','EXECUTING')
                    and r['ros_s']>canceled['ros_s'] for r in statuses),
                measured_stop=len(tail)>=10 and tail[-1]['sim_s']-tail[0]['sim_s']>=.85 and
                    all(0<b['sim_s']-a['sim_s']<=.15 for a,b in zip(tail,tail[1:])) and
                    all(r['speed']<.01 and abs(r['wz'])<.02 for r in tail),
                zero_command=len(commands)>=5 and all(r['v']<1e-6 and abs(r['w'])<1e-6 for r in commands))
            report.update(protocol=protocol,checks=checks,scenario_passed=all(checks.values()))
            report['failed_checks']=[k for k,v in checks.items() if not v]
            report['verdict']='PASS' if report['scenario_passed'] else 'FAIL'
        else:report.update(assess_records(case,protocol,events,records,samples,worker.returncode))
    except (Exception,KeyboardInterrupt) as error:report['error']=str(error) or type(error).__name__
    finally:
        if capture is not None:
            try:capture.wait(timeout=20)
            except subprocess.TimeoutExpired:pass
            report['during_passage_capture']='PENDING_REVIEW' if capture.poll()==0 else 'CAPTURE_FAILED'
        for process in (capture,worker,support):
            try:stop_owned(process,35)
            except Exception as error:
                report.update(cleanup_error=str(error),scenario_passed=False,verdict='INFRA_FAILURE')
        raw.close()
        for log in logs:log.close()
        (a.output/'samples.json').write_text(json.dumps(samples)+'\n')
        (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(report,indent=2),flush=True)
    return 0 if report['scenario_passed'] else 1


if __name__=='__main__':raise SystemExit(main())
