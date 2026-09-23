#!/usr/bin/env python3
"""One bounded body-axis pulse in an owned isolated Gazebo, then zero and record."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import statistics
import time
from collections import deque


def probe_corridor_offsets(case):
    distance = case.get('probe_corridor_m', .6)
    if (isinstance(distance, bool) or not isinstance(distance, (int, float)) or
            not math.isfinite(distance) or not .6 <= distance <= 1.):
        raise ValueError('probe_corridor_m must be a finite distance in [0.6, 1.0] m')
    return tuple(min(index*.02, distance) for index in range(math.ceil(distance/.02)+1))


def wait_for_transform(lookup, spin, transform_error, timeout=10., clock=time.monotonic):
    """Wait for this subscriber's TF tree before constructing the pulse corridor."""
    deadline = clock() + timeout
    last_error = None
    while clock() < deadline:
        try:
            return lookup()
        except transform_error as error:
            last_error = error
            spin()
    raise TimeoutError('pre-motion transform unavailable before wall deadline') from last_error


def analyze(rows, zero_s, direction, profile):
    motion = [r for r in rows if r['kind'] == 'motion']
    unique = []
    for row in motion:
        if not unique or row['source_s'] > unique[-1]['source_s']:
            unique.append(row)
    before = [r for r in unique if zero_s-.2 <= r['source_s'] <= zero_s]
    after = [r for r in unique if r['source_s'] >= zero_s]
    if len(before) < 3 or len(after) < 20 or after[-1]['source_s']-zero_s < 2.:
        raise ValueError('insufficient source-time coverage around zero')
    a, b = before[-1], after[0]
    if b['source_s']-a['source_s'] > .1:
        raise ValueError('zero command has no close odometry bracket')
    f = 0. if b['source_s'] == a['source_s'] else (zero_s-a['source_s'])/(b['source_s']-a['source_s'])
    origin = [a[k]+f*(b[k]-a[k]) for k in ('x','y')]
    actual = statistics.median(math.hypot(r['vx'], r['vy']) for r in before)
    heading = a['yaw']; dx, dy = direction
    ux, uy = dx*math.cos(heading)-dy*math.sin(heading), dx*math.sin(heading)+dy*math.cos(heading)
    peak = max(0., max((r['x']-origin[0])*ux+(r['y']-origin[1])*uy for r in after))
    excursion = max(math.hypot(r['x']-origin[0],r['y']-origin[1]) for r in after)
    settled = [r for r in after if r['source_s'] >= after[-1]['source_s']-1.]
    checks = {
        'actual_motion_before_zero': actual > .02,
        'source_continuity': all(0 < y['source_s']-x['source_s'] <= .1 for x,y in zip(unique,unique[1:])),
        'settled': len(settled)>=20 and all(math.hypot(r['vx'],r['vy'])<.01 and abs(r['w'])<.02 for r in settled),
        'complete_budget_covers_peak': excursion <= profile.stopping_distance(actual),
    }
    return dict(checks=checks,actual_speed_before_zero_m_s=actual,zero_ros_s=zero_s,
        interpolated_zero_xy=origin,peak_forward_m=peak,peak_euclidean_m=excursion,
        final_offset_m=math.hypot(after[-1]['x']-origin[0],after[-1]['y']-origin[1]),
        full_stopping_budget_m=profile.stopping_distance(actual),source_samples=len(unique))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--policy',choices=('off',),default='off')
    args=parser.parse_args();case=json.loads(args.case.read_text());session=json.loads(args.session.read_text())
    if case.get('schema_version')!=2 or case.get('stop_boundary')!='final_output_zero':
        raise ValueError('Stop probes require v2 with explicit final_output_zero reference boundary')
    corridor = probe_corridor_offsets(case)
    identity=session.get('isolation',{})
    if (session.get('state')!='ready' or not identity.get('ASTRIBOT_SIM_INSTANCE') or
            identity.get('ROS_DOMAIN_ID') in (None,'25') or
            any(os.environ.get(k)!=identity.get(k) for k in ('ROS_DOMAIN_ID','IGN_PARTITION','GZ_PARTITION'))):
        raise RuntimeError('Requires the matching ready isolated simulation session')
    owner=Path('/proc')/str(session['supervisor_pid'])
    if not owner.exists() or b'sim_stack_supervisor.py' not in (owner/'cmdline').read_bytes():
        raise RuntimeError('Simulation owner no longer exists')
    speed=case['speed_m_s'];direction=tuple(case['direction'])
    if direction not in ((1,0),(-1,0),(0,1),(0,-1)) or not 0<speed<=.35:
        raise ValueError('Expected a unit body axis and speed in (0,0.35]')
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'case.json').write_text(json.dumps(case,indent=2)+'\n')
    import fcntl
    lease=open('/tmp/astribot_waypoint_'+os.environ['ROS_DOMAIN_ID']+'.lock','a')
    fcntl.flock(lease,fcntl.LOCK_EX | fcntl.LOCK_NB)
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rcl_interfaces.srv import GetParameters
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from nav_msgs.msg import Path as NavPath
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.srv import IsPathValid
    from tf2_ros import Buffer,TransformListener,TransformException
    from rclpy.time import Time
    from std_msgs.msg import String
    from astribot_navigation_msgs.msg import NavigationExecutionStatus
    from action_msgs.msg import GoalStatusArray
    from ament_index_python.packages import get_package_share_directory
    from astribot_s1_navigation_policy.profile import Profile
    profile=Profile.load(Path(get_package_share_directory('astribot_s1_navigation_policy'))/'config/h2_simulation.json')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('simulation_stop_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    tf=Buffer();listener=TransformListener(tf,node)
    pub=node.create_publisher(Twist,'/cmd_vel_nav_body_raw',10)
    rows=[];state={'active_actions':set()};started=None;zero_s=None;pending_commands=deque()
    output=(args.output/'observations.jsonl').open('w')
    def now():return node.get_clock().now().nanoseconds*1e-9
    def record(kind,**data):
        row=dict(kind=kind,ros_s=now(),**data);rows.append(row);output.write(json.dumps(row,allow_nan=False)+'\n')
    def odom(m):
        q=m.pose.pose.orientation;p=m.pose.pose.position;t=m.twist.twist
        sample=dict(source_s=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
            x=p.x,y=p.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),vx=t.linear.x,vy=t.linear.y,w=t.angular.z)
        record('motion',**sample)
        if 'motion' in state and sample['source_s']<=state['motion']['source_s']:return
        state['motion']=sample;state['motion_wall']=time.monotonic()
    def geometry(m):state['geometry']=json.loads(m.data);record('geometry',**state['geometry'])
    def execution(m):state['active']=m.state in ('ACCEPTED','EXECUTING','CANCELING');record('execution',task=m.task_id,state=m.state)
    def raw_command(m):
        # Humble Python callbacks do not expose the publisher GID. Check the
        # endpoint set separately and require an ordered echo of each probe
        # command. An unsolicited/missing echo invalidates this measurement.
        value=(m.linear.x,m.linear.y,m.angular.z)
        if not pending_commands or value!=pending_commands[0]:
            state['foreign_command']=True
        else:pending_commands.popleft()
        record('raw_input',vx=m.linear.x,vy=m.linear.y,w=m.angular.z)
    node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    node.create_subscription(String,'/social_sim/state',geometry,qos_profile_sensor_data)
    node.create_subscription(Twist,'/cmd_vel',lambda m:record('final_command',vx=m.linear.x,vy=m.linear.y,w=m.angular.z),10)
    node.create_subscription(Twist,'/cmd_vel_nav_body_raw',raw_command,10)
    node.create_subscription(NavigationExecutionStatus,'/navigation/execution_status',execution,
                             QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def action_state(name,message):
        if any(s.status in (1,2,3) for s in message.status_list):state['active_actions'].add(name)
        else:state['active_actions'].discard(name)
    for name in ('spin','backup','drive_on_heading','wait','follow_path'):
        node.create_subscription(GoalStatusArray,'/'+name+'/_action/status',lambda m,n=name:action_state(n,m),
                                 QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def publish(v):
        m=Twist();m.linear.x=direction[0]*v;m.linear.y=direction[1]*v
        pending_commands.append((m.linear.x,m.linear.y,m.angular.z));pub.publish(m)
        record('probe_command',vx=m.linear.x,vy=m.linear.y,w=0.)
    def spin():rclpy.spin_once(node,timeout_sec=.01)
    def interrupted(*_):raise KeyboardInterrupt
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,interrupted)
    report=dict(schema_version=2,case=case['id'],verdict='INFRA_FAILURE',scenario_passed=False,
        probe_corridor_m=corridor[-1],
        measurement_boundary='final_output_zero',
        boundary='Historical prior checked after downstream zero; complete normal smoothed stop recorded separately and requires I2 prediction validation. No hardware certification.')
    try:
        client=node.create_client(GetParameters,'/controller_server/get_parameters')
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('controller unavailable')
        future=client.call_async(GetParameters.Request(names=['use_sim_time']))
        until=time.monotonic()+10
        while not future.done() and time.monotonic()<until:spin()
        if not future.done():raise TimeoutError('controller use_sim_time parameter query timed out before motion')
        values=future.result().values
        report['controller_use_sim_time_query']=[dict(type=v.type,bool_value=v.bool_value) for v in values]
        if len(values)!=1 or values[0].type!=1 or not values[0].bool_value:
            raise RuntimeError('controller use_sim_time is not boolean true')
        until=time.monotonic()+10
        while ('motion' not in state or 'geometry' not in state or now()<=0) and time.monotonic()<until:spin()
        wait_for_transform(
            lambda: tf.lookup_transform('map','astribot_torso_base',Time()),
            spin, TransformException)
        report['pre_motion_transform_ready']=True
        # Let transient task status arrive before granting this probe motion.
        quiet=time.monotonic()+2
        while time.monotonic()<quiet:spin()
        if state.get('active') or state['active_actions'] or state.get('foreign_command') or not state.get('geometry',{}).get('geometry_valid') or state['geometry']['people']:
            raise RuntimeError('Requires idle navigation and a valid empty social scene')
        if math.hypot(state['motion']['vx'],state['motion']['vy'])>=.01 or abs(state['motion']['w'])>=.02:
            raise RuntimeError('Robot not stationary before the pulse')
        pose=tf.lookup_transform('map','astribot_torso_base',Time())
        p=pose.transform.translation;q=pose.transform.rotation
        heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        path=NavPath();path.header.frame_id='map';path.header.stamp=node.get_clock().now().to_msg()
        for distance in corridor:
            sample=PoseStamped();sample.header=path.header;sample.pose.orientation=q
            sample.pose.position.x=p.x+distance*(direction[0]*math.cos(heading)-direction[1]*math.sin(heading))
            sample.pose.position.y=p.y+distance*(direction[0]*math.sin(heading)+direction[1]*math.cos(heading))
            path.poses.append(sample)
        validator=node.create_client(IsPathValid,'/is_path_valid')
        if not validator.wait_for_service(timeout_sec=5):raise RuntimeError('path validator unavailable')
        validation=validator.call_async(IsPathValid.Request(path=path));until=time.monotonic()+10
        while not validation.done() and time.monotonic()<until:spin()
        if not validation.done() or not validation.result().is_valid:
            raise RuntimeError(f'complete {corridor[-1]:g} m pulse/stop corridor rejected by current costmap')
        endpoints=node.get_publishers_info_by_topic('/cmd_vel_nav_body_raw')
        report['raw_input_publishers']=[dict(node=p.node_name,namespace=p.node_namespace,gid=list(p.endpoint_gid)) for p in endpoints]
        if (sum(p.node_name==node.get_name() and p.node_namespace==node.get_namespace() for p in endpoints)!=1 or
                any(p.node_name not in (node.get_name(),'controller_server','behavior_server') or p.node_namespace!='/' for p in endpoints)):
            raise RuntimeError('Unexpected raw command publisher in the isolated probe session')
        origin=state['motion'];started=now();until=time.monotonic()+180;next_send=started
        while now()-started<7.6 and time.monotonic()<until:
            spin();t=now()
            if (state.get('active') or state['active_actions'] or state.get('foreign_command') or time.monotonic()-state['motion_wall']>.5 or
                    not 0<=t-state['motion']['source_s']<=.2):raise RuntimeError('task ownership or feedback changed')
            if math.hypot(state['motion']['x']-origin['x'],state['motion']['y']-origin['y'])>corridor[-1]:
                raise RuntimeError(f'probe exceeded {corridor[-1]:g} m displacement')
            if t>=next_send:
                if t-started>=1.6:
                    if zero_s is None:zero_s=t
                    publish(0.)
                else:publish(speed)
                next_send=t+.02
        if zero_s is None or now()-started<7.6:raise RuntimeError('simulation pulse watchdog expired')
        # The last publication and its subscription callback are asynchronous.
        # Drain the reliable zero-command echo before evaluating its count.
        drain=time.monotonic()+.3
        while time.monotonic()<drain:spin()
        final_zero=next((r for r in rows if r['kind']=='final_command' and r['ros_s']>=zero_s and
                         math.hypot(r['vx'],r['vy'])<1e-5 and abs(r['w'])<1e-5),None)
        if final_zero is None:raise RuntimeError('downstream zero was not observed')
        normal_stop=analyze(rows,zero_s,direction,profile)
        normal_stop['acceptance']='Diagnostic normal-stop excursion; the historical final-zero model does not certify the upstream smoother.'
        report['normal_stop']=normal_stop
        report['raw_zero_ros_s']=zero_s
        measurements=analyze(rows,final_zero['ros_s'],direction,profile)
        measurements['checks']['ordered_command_echo']=not pending_commands and not state.get('foreign_command')
        measurements['checks']['final_zero_persistent']=all(math.hypot(r['vx'],r['vy'])<1e-5 and abs(r['w'])<1e-5
            for r in rows if r['kind']=='final_command' and r['ros_s']>=final_zero['ros_s'])
        measurements['final_command_zero_delay_s']=final_zero['ros_s']-zero_s
        measurements['zero_time_source']='Downstream unstamped Twist receipt mapped to local ROS clock; not actuator acknowledgement.'
        geometry_rows=[r for r in rows if r['kind']=='geometry' and r['ros_s']>0]
        measurements['checks']['geometry']=bool(geometry_rows) and all(r['geometry_valid'] and r['conservative_overlap_steps']==0 for r in geometry_rows)
        report.update(measurements,scenario_passed=all(measurements['checks'].values()))
        report['failed_checks']=[name for name,passed in measurements['checks'].items() if not passed]
        report['verdict']='PASS' if report['scenario_passed'] else 'FAIL'
    except (Exception,KeyboardInterrupt) as error:
        report.update(error=str(error) or 'interrupted')
    finally:
        try:
            if started is not None:
                publish(0.)
                end=time.monotonic()+1
                last_zero=time.monotonic()
                while time.monotonic()<end:
                    if time.monotonic()-last_zero>=.05:
                        publish(0.);last_zero=time.monotonic()
                    spin()
        except Exception as error:
            report.update(verdict='INFRA_FAILURE',scenario_passed=False,cleanup_error=str(error))
        finally:
            output.close();(args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2),flush=True);node.destroy_node();rclpy.shutdown()
    return 0 if report['scenario_passed'] else 1


if __name__=='__main__':raise SystemExit(main())
