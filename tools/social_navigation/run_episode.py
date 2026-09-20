#!/usr/bin/env python3
"""Run one explicitly selected Gazebo social episode via the existing navigation runner."""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from episode_evidence import assess_evidence, SOCIAL_STATUS_MAX_AGE_S


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',required=True,help='Episode JSON, requires a matching already-running Gazebo scene')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--policy',choices=('off','h2'),default='h2',help='Expected social policy; off is only for the empty baseline')
    args=parser.parse_args();case=json.loads(Path(args.case).read_text())
    if case.get('schema_version', 1) != 1:
        raise ValueError('unsupported episode schema; rollout plans are not executable cases')
    case.setdefault('schema_version',1)
    stop_contract=case.get('pause',{}).get('stop_contract','pause_plus_1_5s_v1')
    if stop_contract not in ('pause_plus_1_5s_v1','source_hold_budget_v2'):
        raise ValueError('unsupported pause stop evidence contract')
    if case.get('cancel') and (case['cancel'].get('when') not in ('moving','waiting') or
            case.get('expected_task_outcome') != 'CANCELED'):
        raise ValueError('cancellation fixture requires moving/waiting trigger and CANCELED outcome')
    cancel_still_s = case.get('cancel', {}).get('stationary_before_s', 0.)
    if not isinstance(cancel_still_s, (int, float)) or not math.isfinite(cancel_still_s) or cancel_still_s < 0:
        raise ValueError('stationary cancellation duration must be finite and nonnegative')
    if args.policy=='off' and (case['people'] or case.get('input_fault')):
        raise ValueError('off baseline requires an empty scene without fault injection')
    root=Path(__file__).resolve().parents[2]
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'case.json').write_text(json.dumps(case,indent=2))
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rcl_interfaces.srv import SetParameters, GetParameters
    from astribot_navigation_msgs.msg import NavigationExecutionStatus, MotionConstraint
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('social_episode_recorder',parameter_overrides=[Parameter('use_sim_time',value=True)])
    state={};records=[];events=[];child=None;goal_started=None;started=None;pause_at=None;resumed=False
    cancel_at=None;motion_seen=False
    fault_client=node.create_client(SetParameters,'/hunav_truth_adapter/set_parameters') if case.get('input_fault') else None
    fault_index=0;fault_changed=None;fault_future=None
    commands=node.create_publisher(String,'/social_sim/episode',10)
    output=(args.output/'observations.jsonl').open('w')
    def record(kind,data):
        row={'kind':kind,'ros_s':node.get_clock().now().nanoseconds*1e-9,'wall_s':time.monotonic(),**data}
        records.append(row);output.write(json.dumps(row,allow_nan=False)+'\n')
    def receive(kind,message):
        data=json.loads(message.data);state[kind]=data;record(kind,data)
        if kind == 'social':
            now = node.get_clock().now().nanoseconds*1e-9
            waiting = data.get('state') == 'WAIT' and any(not c['admissible'] for c in data.get('candidates', []))
            if not waiting:
                state['cancel_wait_since'] = None
            elif state.get('cancel_wait_since') is None or not 0 <= now-state.get('cancel_wait_at', now) <= SOCIAL_STATUS_MAX_AGE_S:
                state['cancel_wait_since'] = now
            state['cancel_wait_at'] = now
    for kind,topic in [('geometry','/social_sim/state'),('social','/social_navigation/behavior_status'),
                       ('protection','/navigation_policy/protection_state')]:
        node.create_subscription(String,topic,lambda m,k=kind:receive(k,m),qos_profile_sensor_data)
    def episode_state(message):state['episode']=message.data;record('episode_state',{'state':message.data})
    node.create_subscription(String,'/social_sim/episode_state',episode_state,
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Twist,'/cmd_vel',lambda m:record('command',{'v':math.hypot(m.linear.x,m.linear.y),
        'vx':m.linear.x,'vy':m.linear.y,'w':m.angular.z}),10)
    def motion(message):
        m = message
        row = {'v':math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y),'w':m.twist.twist.angular.z,
        'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,
        'vx':m.twist.twist.linear.x,'vy':m.twist.twist.linear.y,
        'source_s':m.header.stamp.sec+m.header.stamp.nanosec*1e-9}
        previous = state.get('motion')
        valid_step = previous is not None and 0 <= row['source_s']-previous['source_s'] <= .3
        if row['v'] >= .01 or abs(row['w']) >= .02:
            state['cancel_still_since'] = None
        elif state.get('cancel_still_since') is None or not valid_step:
            state['cancel_still_since'] = row['source_s']
        state['motion'] = row
        record('motion', row)
    node.create_subscription(Odometry,'/odom',motion,qos_profile_sensor_data)
    node.create_subscription(MotionConstraint,'/navigation_policy/constraint',lambda m:
        record('constraint',{'source_s':m.stamp.sec+m.stamp.nanosec*1e-9,'epoch':m.epoch,
            'sequence':m.sequence,'lease_s':m.lease_s,'hold':m.hold,'reason':m.reason}),10)
    node.create_subscription(NavigationExecutionStatus,'/navigation/execution_status',lambda m:
        record('execution',{'task_id':m.task_id,'sequence':m.sequence,'state':m.state,
                           'reason':m.reason,'action_status':m.action_status}),
        QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def command(value):
        commands.publish(String(data=value));events.append({'command':value,'ros_s':node.get_clock().now().nanoseconds*1e-9})
    def spin():rclpy.spin_once(node,timeout_sec=.02)
    def observation_mode(mode):
        future=fault_client.call_async(SetParameters.Request(parameters=[Parameter('observation_mode',value=mode).to_parameter_msg()]))
        events.append({'observation_mode':mode,'ros_s':node.get_clock().now().nanoseconds*1e-9})
        return future
    summary={'case':case['id'],'status':'running','expected_policy':args.policy}
    def interrupted(signum,frame):
        summary['interrupted_signal']=signum
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
    try:
        deadline=time.monotonic()+20
        while ('geometry' not in state or 'episode' not in state) and time.monotonic()<deadline:spin()
        if 'geometry' not in state or state.get('episode')!='pause':raise RuntimeError('matching paused Gazebo episode not observed')
        actual_names={p['name'] for p in state['geometry']['people']}
        if actual_names!=set(case['people']):raise RuntimeError('wrong Gazebo scene: people IDs differ')
        if not state['geometry']['geometry_valid']:raise RuntimeError('independent collision geometry unavailable')
        stop_profile=None
        if case.get('pause') and stop_contract=='source_hold_budget_v2':
            from astribot_s1_navigation_policy.profile import Profile
            client=node.create_client(GetParameters,'/navigation_final_protection/get_parameters')
            if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Live stop profile unavailable')
            future=client.call_async(GetParameters.Request(names=['profile','use_sim_time']))
            deadline=time.monotonic()+10
            while not future.done() and time.monotonic()<deadline:spin()
            if not future.done():raise TimeoutError('Live stop profile query timed out')
            values=future.result().values
            if len(values)!=2 or values[0].type!=4 or values[1].type!=1 or not values[1].bool_value:
                raise RuntimeError('Simulation stop profile required')
            profile_path=Path(values[0].string_value)
            stop_profile=Profile.load(profile_path);stop_profile.require_environment(True)
            if stop_profile.environment!='simulation':raise RuntimeError('Stop profile environment is not simulation')
            dependencies={}
            def profile_inputs(path):
                path=path.resolve()
                if str(path) in dependencies:return
                dependencies[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
                data=json.loads(path.read_text())
                for key in ('base_profile','stop_reference_file'):
                    if key in data:profile_inputs(path.parent/data[key])
            profile_inputs(profile_path)
            (args.output/'stop_profile.json').write_text(json.dumps(dict(
                node='/navigation_final_protection',profile=str(profile_path),files=dependencies,
                resolved=dict(stop_profile._values)),indent=2)+'\n')
        expected_yaw=case.get('initial_yaw_rad',0.)
        actual_yaw=state['geometry']['robot'][2]
        if abs(math.atan2(math.sin(actual_yaw-expected_yaw),math.cos(actual_yaw-expected_yaw)))>.03:
            raise RuntimeError('initial robot heading differs from the fixture')
        if fault_client and not fault_client.wait_for_service(timeout_sec=5):
            raise RuntimeError('simulation observation fault interface unavailable')
        for person in state['geometry']['people']:
            expected=case['initial_people'][person['name']]
            if math.hypot(person['x']-expected[0],person['y']-expected[1])>.05:
                raise RuntimeError('wrong Gazebo scene or episode already moved')
        (args.output/'route.json').write_text(json.dumps(case['route']))
        log=(args.output/'navigation.log').open('w')
        navigation_args=[sys.executable,str(root/'tools/run_waypoint_route.py'),
            '--route',str(args.output/'route.json'),'--cycles','1','--timeout',str(case.get('goal_timeout_s',180)),
            '--timeout-clock','sim','--wall-watchdog',str(case.get('wall_watchdog_s',2400)),
            '--duration',str(case.get('duration_s',420)),'--settle','2','--output',str(args.output/'navigation')]
        if case.get('through_poses'):
            navigation_args+=['--through-poses']
            if case.get('via_tolerance_m') is not None:navigation_args+=['--via-tolerance',str(case['via_tolerance_m'])]
        child=subprocess.Popen(navigation_args,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=node.get_clock().now().nanoseconds*1e-9+case.get('duration_s',420)+30
        wall_deadline=time.monotonic()+case.get('wall_watchdog_s',2400)+30
        while (child.poll() is None and time.monotonic()<wall_deadline and
               node.get_clock().now().nanoseconds*1e-9<deadline):
            spin();now=node.get_clock().now().nanoseconds*1e-9
            if records and records[-1]['kind']=='motion' and records[-1]['v']>.02:motion_seen=True
            status_path=args.output/'navigation/status.json'
            if goal_started is None and status_path.is_file():
                navigation=json.loads(status_path.read_text())
                if navigation.get('state')=='running':goal_started=now
            if goal_started is not None and started is None:
                if state['geometry']['robot'][0]>=case.get('start_robot_x',.2) or now-goal_started>=case.get('start_after_s',20):
                    command('start');started=now
            cancel_case=case.get('cancel')
            if cancel_case and cancel_at is None and started is not None and motion_seen:
                when=cancel_case['when']
                due=(state['geometry']['robot'][0]>=cancel_case.get('robot_x',.4) if when=='moving' else
                     state.get('social',{}).get('state')=='WAIT' and
                     any(not c['admissible'] for c in state.get('social',{}).get('candidates',[])))
                if due and cancel_still_s:
                    measured = state.get('motion', {})
                    since = state.get('cancel_still_since')
                    wait_since = state.get('cancel_wait_since')
                    due = (since is not None and wait_since is not None and
                        -.05 <= now-measured['source_s'] <= .3 and
                        0 <= now-state['cancel_wait_at'] <= SOCIAL_STATUS_MAX_AGE_S and
                        measured['source_s']-max(since, wait_since) >= cancel_still_s)
                if due:
                    cancel_at=now
                    events.append({'command':'cancel_navigation','ros_s':now,'when':when})
                    os.killpg(child.pid,signal.SIGINT)
            pause=case.get('pause')
            pause_ready=bool(pause and (
                ('after_s' in pause and started is not None and now-started>=pause['after_s']) or
                ('person_y_within' in pause and any(abs(p['y'])<=pause['person_y_within'] and
                    p['x']>=pause.get('person_x_at_least',-math.inf) for p in state['geometry']['people']))))
            if started is not None and pause_ready and pause_at is None:
                command('pause');pause_at=now
            if pause_at is not None and not resumed and now-pause_at>=pause['duration_s']:
                command('start');resumed=True
            if fault_future is not None and fault_future.done():
                if not all(r.successful for r in fault_future.result().results):
                    raise RuntimeError('simulation input fault rejected')
                fault_future=None
            if fault_client and started is not None and fault_future is None:
                fault=case['input_fault'];sequence=fault['sequence']
                due=(now-started>=fault['after_s'] if fault_changed is None else
                     now-fault_changed>=sequence[fault_index-1]['duration_s'])
                if due and fault_index<len(sequence):
                    fault_future=observation_mode(sequence[fault_index]['mode'])
                    fault_index+=1;fault_changed=now
        if child.poll() is None:raise TimeoutError('episode deadline')
        finish=node.get_clock().now().nanoseconds*1e-9+(5 if case.get('cancel') else 3)
        wall_finish=time.monotonic()+60
        while node.get_clock().now().nanoseconds*1e-9<finish and time.monotonic()<wall_finish:spin()
        results_path=args.output/'navigation/results.jsonl'
        results=[json.loads(line) for line in results_path.read_text().splitlines()] if results_path.exists() else []
        social=[r for r in records if r['kind']=='social' and r.get('active')]
        geometry=[r for r in records if r['kind']=='geometry']
        reasons=Counter(r['reason'] for r in social)
        conflicts=[r for r in social if any(not c['admissible'] for c in r['candidates'])]
        summary.update(status='completed',navigation_returncode=child.returncode,events=events,
            all_goals_passed=len(results)==(1 if case.get('through_poses') else len(case['route'])) and all(r['passed'] for r in results),
            arrivals=[{'xy_m':r['xy_m'],'yaw_deg':r['yaw_deg'],'passed':r['passed']} for r in results],
            collision_geometry_valid=all(r['geometry_valid'] for r in geometry),
            overlap_steps=max(r['conservative_overlap_steps'] for r in geometry),
            clearance_lower_bound_m=min(r['clearance_lower_bound_m'] for r in geometry) if case['people'] else None,
            social_reasons=dict(reasons),conflict_samples=len(conflicts),
            recoveries_after_conflict=sum(r['state']=='CRUISE' and r['ros_s']>conflicts[0]['ros_s'] for r in social) if conflicts else 0,
            replan_events=[e for r in results for e in r['replan_events']],
            social_processing_max_s=max((r['processing_s'] for r in social),default=0),
            boundary='Simulation episode; scenario-specific assertions and paired baseline review still required')
        checks={
            'arrivals':summary['all_goals_passed'],
            'geometry':bool(geometry) and summary['collision_geometry_valid'] and summary['overlap_steps']==0,
            'episode_ack':any(r['kind']=='episode_state' and r['state']=='start' for r in records),
            'input_fresh':not any(r.get('input_error') for r in social),
            'conflict_exercised':bool(conflicts) if case.get('expect_conflict') else True,
            'recovery_exercised':summary['recoveries_after_conflict']>0 if case.get('expect_conflict') else True,
            'event_only_planning':all(e['reason'] in ('initial_goal','new_goal') for e in summary['replan_events']),
            'policy_observed':bool(social) if args.policy=='h2' else not social,
        }
        if case['people']:
            checks['human_moved']=any(math.hypot(p['x']-case['initial_people'][p['name']][0],
                p['y']-case['initial_people'][p['name']][1])>.2 for r in geometry for p in r['people'])
        if case.get('pause'):
            checks['pause_and_resume']=pause_at is not None and resumed
            paused=[r for r in geometry if pause_at is not None and
                    pause_at+.2<=r['ros_s']<pause_at+case['pause']['duration_s']-.1]
            checks['human_stopped']=bool(paused) and all(
                math.hypot(p['x']-paused[0]['people'][j]['x'],p['y']-paused[0]['people'][j]['y'])<.03
                for r in paused for j,p in enumerate(r['people']))
            checks['wait_during_person_stop']=any(r['state']=='WAIT' and
                pause_at<=r['ros_s']<pause_at+case['pause']['duration_s'] for r in social) if pause_at is not None else False
            if stop_contract=='source_hold_budget_v2':
                from pause_stop_evidence import assess_pause_stop
                resume_at=next((e['ros_s'] for e in events if e['command']=='start' and
                    pause_at is not None and e['ros_s']>pause_at),None)
                checks['pause_stop_evidence']=pause_at is not None and resume_at is not None
                if checks['pause_stop_evidence']:
                    evidence=assess_pause_stop(records,pause_at,resume_at,
                        stop_profile.stopping_distance,stop_profile.clearance_margin_m)
                    summary['pause_stop_evidence']=evidence;checks.update(evidence['checks'])
            else:
                stopped_motion=[r for r in records if r['kind']=='motion' and pause_at is not None and
                                pause_at+1.5<=r['ros_s']<pause_at+case['pause']['duration_s']]
                checks['robot_stopped_during_person_stop']=bool(stopped_motion) and all(
                    r['v']<.01 and abs(r['w'])<.02 for r in stopped_motion)
            if case.get('expect_goal_occupied'):
                gx,gy,_=case['route'][0]
                checks['goal_actually_occupied']=any(math.hypot(p['x']-gx,p['y']-gy)<.5
                    for r in paused for p in r['people'])
        if fault_client:
            checks.pop('input_fresh')
            changes=[e for e in events if 'observation_mode' in e]
            checks['fault_sequence_completed']=fault_index==len(case['input_fault']['sequence']) and fault_future is None
            checks['lease_expired']=any(r.get('input_error') in ('SAMPLE_TIME_INVALID','TRACK_EXPIRED') for r in social)
            stable=[r for r in records if r['kind'] in ('command','motion') and
                    any(a['ros_s']+1.5<=r['ros_s']<b['ros_s'] for a,b in zip(changes,changes[1:]) if a['observation_mode']!='normal')]
            checks['stopped_on_expired_input']=bool(stable) and all(r['v']<.01 and abs(r['w'])<.02 for r in stable)
            checks['fresh_recovery']=bool(changes) and any(r['state']=='CRUISE' and r['ros_s']>changes[-1]['ros_s'] for r in social)
        summary['checks']=checks
        navigation_status=json.loads(status_path.read_text()) if status_path.exists() else {}
        summary['navigation_status']=navigation_status
        summary.update(assess_evidence(case,records,summary,args.policy,navigation_status))
    except BaseException as error:
        summary.update(status='failed',verdict='INFRA_FAILURE',scenario_passed=False,
                       error=str(error) or type(error).__name__);raise
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=20)
            except subprocess.TimeoutExpired:
                summary['cancellation_error']='navigation runner did not finish cancellation within 20 seconds'
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=3)
        if fault_client and 'cancellation_error' not in summary:
            restore=fault_client.call_async(SetParameters.Request(parameters=[Parameter('observation_mode',value='normal').to_parameter_msg()]))
            until=time.monotonic()+3
            while not restore.done() and time.monotonic()<until:spin()
            if not restore.done() or not all(r.successful for r in restore.result().results):
                summary['restore_error']='simulation observation mode was not restored'
        if summary.get('cancellation_error') or summary.get('restore_error'):
            summary.update(scenario_passed=False,verdict='INFRA_FAILURE')
        output.close();(args.output/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if summary.get('scenario_passed') else 1


if __name__=='__main__':raise SystemExit(main())
