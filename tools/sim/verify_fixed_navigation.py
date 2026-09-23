#!/usr/bin/env python3
"""Owned simulation validation of formal fixed envelope during Nav2 motion."""
import math
import statistics

class OwnedHoldLease:
    """Bind renewal to feedback for this Action UUID, never global status alone."""
    def __init__(self, goal_id):
        self.goal_id=goal_id
        self.accepted=False
        self.identity=None

    def observe(self, goal_id, lease_id, epoch, hold_id):
        if goal_id!=self.goal_id or not all((lease_id,epoch,hold_id)):
            return
        identity=(lease_id,epoch,hold_id)
        if self.identity is not None and self.identity!=identity:
            raise RuntimeError('OWN_HOLD_FEEDBACK_IDENTITY_CHANGED')
        self.identity=identity

    def matches(self, status):
        return bool(self.accepted and self.identity is not None and status.get('phase') in ('1','2','3')
                    and self.identity==(status.get('lease_id'),status.get('epoch'),status.get('hold_id')))


def navigation_entry_observed(active_goals, verified_cold_start):
    return verified_cold_start or set(active_goals) == {'navigate_to_pose', 'navigate_through_poses'}

def transform_goal(target, transform):
    x,y,yaw=transform;c=math.cos(yaw);s=math.sin(yaw)
    return [x+c*target[0]-s*target[1],y+s*target[0]+c*target[1],math.remainder(target[2]+yaw,2*math.pi)]

def measured_stop(rows, after_ros_s, now_wall, now_ros_s):
    tail = [r for r in rows if r['t'] >= rows[-1]['t']-.7-1e-8] if rows else []
    keys = ('t', 'wall', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'cmd_vx', 'cmd_vy', 'cmd_wz', 'cmd_wall')
    valid = (len(tail) >= 12 and all(r['frame']=='odom' and all(math.isfinite(r[k]) for k in keys) for r in tail)
             and all(0 < b['t']-a['t'] <= .12 for a,b in zip(tail, tail[1:])))
    if not valid:
        return {'passed': False, 'reason': 'INCOMPLETE_OR_INVALID_SOURCE'}
    span = tail[-1]['t']-tail[0]['t']
    yaw = [tail[0]['yaw']]
    for a,b in zip(tail, tail[1:]):
        yaw.append(yaw[-1]+math.remainder(b['yaw']-a['yaw'], 2*math.pi))
    drift = max(math.hypot(r['x']-tail[0]['x'],r['y']-tail[0]['y']) for r in tail)
    rotation = max(abs(v-yaw[0]) for v in yaw)
    tmean = statistics.mean(r['t'] for r in tail)
    variance = sum((r['t']-tmean)**2 for r in tail)
    slopes = [sum((r['t']-tmean)*r[k] for r in tail)/variance for k in ('x','y')]
    speed = max(math.hypot(r['vx'],r['vy']) for r in tail)
    wz = max(abs(r['wz']) for r in tail)
    cmd = max(max(abs(r[k]) for k in ('cmd_vx','cmd_vy','cmd_wz')) for r in tail)
    fresh = (math.isfinite(now_ros_s) and 0 <= now_ros_s-tail[-1]['t'] < .3 and
             0 <= now_wall-tail[-1]['wall'] < .3 and
             all(0 <= r['wall']-r['cmd_wall'] < .3 for r in tail))
    passed = (span >= .6 and tail[0]['t'] > after_ros_s and fresh and
              speed <= .01 and wz <= .02 and cmd <= 1e-6 and
              drift <= .005 and rotation <= .01 and math.hypot(*slopes) <= .01)
    return dict(passed=passed, samples=len(tail), span_s=span, start_ros_s=tail[0]['t'],
                end_ros_s=tail[-1]['t'], drift_m=drift, rotation_rad=rotation,
                speed_mps=speed, angular_speed_radps=wz, command_max=cmd, fresh=fresh)


def main():
    import argparse
    import fcntl
    import json
    import os
    from pathlib import Path
    import signal
    import time
    import uuid
    from prepare_empty_inventory import verify_owner
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.action.graph import get_action_client_names_and_types_by_node, get_action_server_names_and_types_by_node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs.msg import LaserScan
    from action_msgs.msg import GoalStatusArray
    from unique_identifier_msgs.msg import UUID
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import String
    from astribot_navigation_msgs.msg import ArmHoldStatus, RobotGeometryState, NavigationEnvelopeV2, EnvelopeApplyStatus, RobotEnvelope, NavigationExecutionStatus
    from astribot_navigation_msgs.srv import SetFixedEnvelope
    from astribot_s1_transport_native.action import HoldResources
    from astribot_s1_transport_native.srv import RenewHold

    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--owner',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--session',required=True)
    parser.add_argument('--source',default='gazebo_empty_v1')
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--cold-start-receipt',type=Path,required=True,help='own runner receipt for a fresh navigator with no preceding goal writers')
    parser.add_argument('--scenario',choices=('all','return_90','hold_cancel','consumer_pause'),default='all',help='run faults independently when another scenario has a separately recorded failure')
    args=parser.parse_args()
    owner=json.loads(args.owner.read_text())
    verify_owner(owner,args.session,args.source)
    verified_cold_start=False
    process_identities={}
    if args.cold_start_receipt:
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
        from sim_stack_supervisor import owned_descendants
        from capture_supervisor_owner import capture
        receipt=json.loads(args.cold_start_receipt.read_text())
        process_identities=receipt['processes']
        owned=owned_descendants([owner['pid']])
        if not (receipt['owner']==owner and receipt['session']==args.session and receipt['no_preceding_goal_writers'] is True
                and 0<=time.time()-receipt['captured_wall']<120
                and receipt['probe_output']==str(args.output.resolve())
                and {'bt_navigator','task_arbiter_cpp'}.issubset(process_identities)
                and all(i['pid'] in owned and capture(i['pid'])==i and Path(i['exe']).name==name for name,i in process_identities.items())):
            raise RuntimeError('COLD_START_RECEIPT_INVALID')
        verified_cold_start=True
    profile=json.loads(args.profile.read_text())
    if profile['environment']!='simulation': raise RuntimeError('SIMULATION_ONLY')
    args.output.mkdir(parents=True,exist_ok=False)
    lease=open('/tmp/astribot_waypoint_'+os.environ['ROS_DOMAIN_ID']+'.lock','a')
    fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('fixed_navigation_'+uuid.uuid4().hex[:8],parameter_overrides=[Parameter('use_sim_time',value=True)])
    tf_buffer=Buffer();tf_listener=TransformListener(tf_buffer,node)
    def interrupted(signum,frame): raise KeyboardInterrupt('owned validation interrupted')
    signal.signal(signal.SIGINT,interrupted); signal.signal(signal.SIGTERM,interrupted)
    latest={}; receipts={}; motion=[]; commands=[]; events=[]; envelopes=[]; acks=[]; scenarios=[]; plans=[]; phases=[]; diagnostics=[]; diagnostic_at={}
    active_goals={}; execution_active=set(); execution_events=[]; seen_nav_ids=set(); own_nav_ids=set(); pending_nav=None; pending_hold=None; pending_nav_id=None; pending_hold_id=None
    nav_admission_uncertain=False;hold_admission_uncertain=False
    consumers={'global_costmap','local_costmap','planner','controller','policy','protection'}
    hold_goal=hold_result=nav_goal=nav_result=None;hold_binding=None
    renew_enabled=False; renew_sequence=0; renew_pending=None; renew_last=0.; suspended=None
    cleanup_mode=False;cleanup_health_errors=[];graph_ready=False;graph_last=0.
    report=dict(passed=False,owner=owner,layer='owned_fixed_v2_empty_navigation_simulation',scenario=args.scenario,started_wall=time.time(),scenarios=scenarios)
    hold_action=ActionClient(node,HoldResources,'/transport/hold_resources')
    navigate=ActionClient(node,NavigateToPose,'/navigate_to_pose')
    renew=node.create_client(RenewHold,'/transport/hold_executor/renew')
    fixed=node.create_client(SetFixedEnvelope,'/navigation/set_fixed_envelope')
    def stamp(v): return v.sec+v.nanosec*1e-9
    def ros(): return node.get_clock().now().nanoseconds*1e-9
    def event(kind,**data):
        value=dict(kind=kind,wall=time.monotonic(),ros_s=ros(),**data);events.append(value)
        print(json.dumps(value),flush=True)
    def receive(key,msg): latest[key]=msg; receipts[key]=time.monotonic()
    def command(msg):
        row=dict(wall=time.monotonic(),ros_s=ros(),vx=msg.linear.x,vy=msg.linear.y,wz=msg.angular.z)
        commands.append(row);receive('command',row)
    def odometry(msg):
        t=stamp(msg.header.stamp)
        if motion and t<=motion[-1]['t']:
            if t<motion[-1]['t']: latest['clock_error']='ODOMETRY_CLOCK_ROLLBACK'
            return
        q=msg.pose.pose.orientation;p=msg.pose.pose.position;v=msg.twist.twist;c=latest.get('command',{})
        row=dict(t=t,wall=time.monotonic(),x=p.x,y=p.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                 vx=v.linear.x,vy=v.linear.y,wz=v.angular.z,frame=msg.header.frame_id,
                 cmd_vx=c.get('vx',math.nan),cmd_vy=c.get('vy',math.nan),cmd_wz=c.get('wz',math.nan),cmd_wall=c.get('wall',-math.inf))
        motion.append(row);receive('odom',row)
    def envelope(msg):
        receive('envelope',msg)
        envelopes.append(dict(wall=time.monotonic(),ros_s=ros(),stamp=stamp(msg.header.stamp),until=stamp(msg.valid_until),epoch=msg.epoch,
                              session=msg.coordinator_session_id,hash=msg.installed_geometry_hash,allowed=msg.navigation_allowed,
                              reason=msg.reason,hold_id=msg.hold_id,request_id=msg.request_id))
    def ack(msg):
        acks.append(dict(wall=time.monotonic(),ros_s=ros(),stamp=stamp(msg.header.stamp),consumer=msg.consumer_id,
                         session=msg.coordinator_session_id,epoch=msg.envelope_epoch,hash=msg.installed_geometry_hash,
                         applied=msg.applied,reason=msg.reason))
    node.create_subscription(Odometry,'/odom',odometry,qos_profile_sensor_data)
    node.create_subscription(Twist,'/cmd_vel',command,qos_profile_sensor_data)
    node.create_subscription(RobotGeometryState,'/navigation/geometry_state',lambda m:receive('geometry',m),10)
    node.create_subscription(ArmHoldStatus,'/navigation/arm_hold',lambda m:receive('hold',m),10)
    node.create_subscription(NavigationEnvelopeV2,'/navigation/envelope_v2',envelope,30)
    node.create_subscription(EnvelopeApplyStatus,'/navigation/envelope_applied',ack,50)
    node.create_subscription(String,'/transport/hold_executor/status',lambda m:receive('executor',json.loads(m.data)),10)
    node.create_subscription(RosPath,'/plan',lambda m:plans.append(dict(ros_s=ros(),stamp=stamp(m.header.stamp),frame=m.header.frame_id,poses=[[p.pose.position.x,p.pose.position.y] for p in m.poses])),10)
    node.create_subscription(String,'/path_tracking/phase',lambda m:phases.append(dict(ros_s=ros(),wall=time.monotonic(),phase=m.data)),qos_profile_sensor_data)
    diagnostic_stream=(args.output/'policy_diagnostics.jsonl').open('w')
    def diagnostic(key,msg):
        wall=time.monotonic()
        if wall-diagnostic_at.get(key,0.)<.2:return
        diagnostic_at[key]=wall
        value=dict(source=key,wall=wall,ros_s=ros(),data=json.loads(msg.data))
        diagnostic_stream.write(json.dumps(value)+'\n');diagnostic_stream.flush()
    for key in ('observation','state'):
        node.create_subscription(String,'/navigation_policy/'+key,lambda m,k=key:diagnostic(k,m),10)
    def scan_diagnostic(msg):
        wall=time.monotonic()
        if wall-diagnostic_at.get('scan_source',0.)<.2:return
        diagnostic_at['scan_source']=wall
        at=Time.from_msg(msg.header.stamp)
        row=dict(source='scan_source',wall=wall,ros_s=ros(),capture_s=stamp(msg.header.stamp),frame=msg.header.frame_id,
                 tf_at_capture={frame:tf_buffer.can_transform(frame,msg.header.frame_id,at) for frame in ('map','odom','astribot_torso_base')})
        diagnostic_stream.write(json.dumps(row)+'\n');diagnostic_stream.flush()
    node.create_subscription(LaserScan,'/scan_from_cloud',scan_diagnostic,qos_profile_sensor_data)
    def action_status(name,msg):
        active_goals[name]={bytes(s.goal_info.goal_id.uuid).hex() for s in msg.status_list if s.status in (1,2,3)}
        seen_nav_ids.update(bytes(s.goal_info.goal_id.uuid).hex() for s in msg.status_list)
    for name in ('navigate_to_pose','navigate_through_poses'):
        node.create_subscription(GoalStatusArray,'/'+name+'/_action/status',lambda m,n=name:action_status(n,m),
                                 QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def execution_status(msg):
        execution_events.append(dict(ros_s=ros(),task_id=msg.task_id,source=msg.source,state=msg.state,reason=msg.reason,sequence=msg.sequence))
        if msg.state in ('ACCEPTED','EXECUTING','CANCELING'):execution_active.add(msg.task_id)
        else:execution_active.discard(msg.task_id)
    node.create_subscription(NavigationExecutionStatus,'/navigation/execution_status',execution_status,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def foreign_navigation():
        return bool(execution_active-own_nav_ids) or any(ids-own_nav_ids for ids in active_goals.values())
    def check_navigation_graph():
        foreign=[];servers={};own_client=False
        owned_clients={('bt_navigator','/navigation_executor'):'bt_navigator',('navigation_task_arbiter','/'):'task_arbiter_cpp',('operator_backend','/'):'operator_backend',
                       ('loop_route_executor','/'):'loop_route_executor',('waypoint_follower','/'):'waypoint_follower'}
        for name,namespace in node.get_node_names_and_namespaces():
            for action,_ in get_action_client_names_and_types_by_node(node,name,namespace):
                if action.endswith(('/navigate_to_pose','/navigate_through_poses')):
                    if (name,namespace)==(node.get_name(),node.get_namespace()):own_client=True
                    elif owned_clients.get((name,namespace)) not in process_identities:foreign.append((name,namespace,action))
            for action,_ in get_action_server_names_and_types_by_node(node,name,namespace):
                if action in ('/navigate_to_pose','/navigate_through_poses'):
                    servers.setdefault(action,[]).append((name,namespace))
        if foreign:raise RuntimeError('FOREIGN_NAVIGATION_CLIENT:'+str(foreign))
        for identity in process_identities.values():
            if capture(identity['pid'])!=identity:raise RuntimeError('NAVIGATION_PROCESS_IDENTITY_CHANGED')
        return own_client and servers=={'/navigate_to_pose':[('navigation_task_arbiter','/')],'/navigate_through_poses':[('navigation_task_arbiter','/')]}
    def fresh(key): return key in latest and 0<=time.monotonic()-receipts[key]<.3
    def geometry_ready():
        g=latest.get('geometry')
        return bool(fresh('geometry') and g.complete and g.attachment_state_confirmed and not g.attachment_ids and 0<stamp(g.header.stamp)<=ros()<stamp(g.valid_until))
    def positive(epoch):
        e=latest.get('envelope')
        if not (fresh('envelope') and e.epoch==epoch and e.navigation_allowed and stamp(e.header.stamp)<=ros()<stamp(e.valid_until)): return False
        matched={}
        for a in acks:
            if (a['session'],a['epoch'],a['hash'])==(e.coordinator_session_id,e.epoch,e.installed_geometry_hash): matched[a['consumer']]=a
        return {name for name,a in matched.items() if a['applied'] and 0<=ros()-a['stamp']<.5 and 0<=time.monotonic()-a['wall']<.5}==consumers
    def spin(predicate,timeout,reason,monitor_motion=False):
        nonlocal renew_pending,renew_last,renew_sequence,renew_enabled,graph_last
        deadline=time.monotonic()+timeout
        while True:
            if not cleanup_mode and latest.get('hold_ownership_error'):raise RuntimeError(latest['hold_ownership_error'])
            if graph_ready and not cleanup_mode and time.monotonic()-graph_last>1.:
                graph_last=time.monotonic()
                if not check_navigation_graph():raise RuntimeError('NAVIGATION_GRAPH_CHANGED')
            if latest.get('clock_error') and (not cleanup_mode or monitor_motion): raise RuntimeError(latest['clock_error'])
            if monitor_motion and (not fresh('odom') or not 0<=ros()-latest['odom']['t']<.3): raise RuntimeError('MOTION_FEEDBACK_STALE')
            if monitor_motion and foreign_navigation(): raise RuntimeError('FOREIGN_NAVIGATION_APPEARED')
            if predicate(): break
            if time.monotonic()>deadline: raise RuntimeError(reason+': '+str(envelopes[-1] if envelopes else latest.get('executor')))
            rclpy.spin_once(node,timeout_sec=.01)
            if renew_pending is not None and renew_pending.done():
                answer=renew_pending.result()
                if renew_enabled and not answer.accepted:
                    if not cleanup_mode:raise RuntimeError('RENEW_REJECTED:'+answer.reason)
                    cleanup_health_errors.append('RENEW_REJECTED:'+answer.reason);renew_enabled=False
                renew_pending=None
            status=latest.get('executor',{})
            if renew_enabled and hold_binding is not None and hold_binding.matches(status) and renew_pending is None and time.monotonic()-renew_last>.2:
                renew_sequence+=1
                renew_pending=renew.call_async(RenewHold.Request(lease_id=status['lease_id'],resource_epoch=status['epoch'],sequence=renew_sequence))
                renew_last=time.monotonic()
    def duration(seconds):
        deadline=time.monotonic()+seconds
        spin(lambda:time.monotonic()>=deadline,seconds+1,'OBSERVE')
    def stop(after,timeout=15):
        if latest.get('clock_error'):raise RuntimeError(latest['clock_error'])
        spin(lambda:measured_stop(motion,after,time.monotonic(),ros())['passed'],timeout,'ACTUAL_STOP_TIMEOUT',True)
        return measured_stop(motion,after,time.monotonic(),ros())
    def hold_feedback(message):
        nonlocal renew_enabled
        if hold_binding is not None:
            value=message.feedback
            try:
                hold_binding.observe(bytes(message.goal_id.uuid).hex(),value.lease_id,value.resource_epoch,value.hold_id)
            except RuntimeError as error:
                # Fail the run, but let cancellation and terminal readback keep spinning.
                renew_enabled=False
                latest['hold_ownership_error']=str(error)
                if str(error) not in cleanup_health_errors:cleanup_health_errors.append(str(error))

    def acquire():
        nonlocal hold_goal,hold_result,renew_enabled,renew_sequence,renew_pending,renew_last,pending_hold,pending_hold_id,hold_admission_uncertain,hold_binding
        spin(lambda:geometry_ready() and latest.get('executor',{}).get('phase')=='0',15,'FORMAL_HOLD_ENTRY')
        renew_sequence=0;renew_pending=None;renew_last=0.;renew_enabled=True
        pending_hold_id=uuid.uuid4();hold_binding=OwnedHoldLease(pending_hold_id.hex)
        hold_admission_uncertain=True
        pending_hold=hold_action.send_goal_async(HoldResources.Goal(task_id='I0_2_fixed_navigation',request_id='n4_hold_'+uuid.uuid4().hex),goal_uuid=UUID(uuid=list(pending_hold_id.bytes)),feedback_callback=hold_feedback)
        spin(pending_hold.done,5,'HOLD_ADMISSION');hold_goal=pending_hold.result();pending_hold=None;hold_admission_uncertain=False
        if not hold_goal.accepted: raise RuntimeError('HOLD_REJECTED')
        hold_binding.accepted=True
        hold_result=hold_goal.get_result_async()
        spin(lambda:hold_result.done() or (fresh('hold') and latest['hold'].hold_confirmed and hold_binding.matches(latest.get('executor',{})) and latest['hold'].hold_id==hold_binding.identity[2]),18,'HOLD_NOT_CONFIRMED')
        if hold_result.done(): raise RuntimeError('HOLD_TERMINATED:'+str(hold_result.result()))
        spin(geometry_ready,3,'GEOMETRY_FOR_FIXED')
        limits=RobotEnvelope(frame_id=profile['base_frame'],posture_id='n4_measured_current_pose',lease_s=.3)
        for name in ('half_length_m','half_width_m','height_m','payload_mass_kg','max_speed_m_s','max_angular_speed_rad_s','max_acceleration_m_s2','brake_deceleration_m_s2'): setattr(limits,name,float(profile[name]))
        request=SetFixedEnvelope.Request(request_id='n4_fixed_'+uuid.uuid4().hex,hold_id=latest['hold'].hold_id,geometry_sequence=latest['geometry'].sequence,limits=limits)
        future=fixed.call_async(request);spin(future.done,3,'FIXED_REQUEST');response=future.result()
        if not response.accepted: raise RuntimeError('FIXED_REJECTED:'+response.reason)
        spin(lambda:positive(response.epoch),15,'SIX_CONSUMERS_NOT_APPLIED')
        event('formal_admission',epoch=response.epoch,hold_id=request.hold_id,request_id=request.request_id)
        return response.epoch
    def start_nav(target):
        nonlocal nav_goal,nav_result,pending_nav,pending_nav_id,nav_admission_uncertain
        duration(.15)
        if not navigation_entry_observed(active_goals,verified_cold_start):raise RuntimeError('NAVIGATION_IDLE_UNOBSERVED')
        if foreign_navigation():raise RuntimeError('OTHER_NAVIGATION_ACTIVE_NO_PREEMPTION')
        if not check_navigation_graph():raise RuntimeError('NAVIGATION_GRAPH_INCOMPLETE')
        if nav_goal is not None or pending_nav is not None:raise RuntimeError('OWN_NAVIGATION_NOT_TERMINAL')
        spin(lambda:tf_buffer.can_transform('map','odom',Time()),5,'GOAL_FRAME_TF_UNAVAILABLE')
        transform=tf_buffer.lookup_transform('map','odom',Time())
        tf_stamp=stamp(transform.header.stamp)
        if tf_stamp and not 0<=ros()-tf_stamp<.3:raise RuntimeError('GOAL_FRAME_TF_STALE')
        q=transform.transform.rotation;t=transform.transform.translation
        if abs(q.x)>.001 or abs(q.y)>.001:raise RuntimeError('NONPLANAR_GOAL_FRAME')
        mapped=transform_goal(target,[t.x,t.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)])
        pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=node.get_clock().now().to_msg()
        pose.pose.position.x=mapped[0];pose.pose.position.y=mapped[1];pose.pose.orientation.z=math.sin(mapped[2]/2);pose.pose.orientation.w=math.cos(mapped[2]/2)
        pending_nav_id=uuid.uuid4();own_nav_ids.add(pending_nav_id.hex)
        nav_admission_uncertain=True
        pending_nav=navigate.send_goal_async(NavigateToPose.Goal(pose=pose),goal_uuid=UUID(uuid=list(pending_nav_id.bytes)))
        spin(pending_nav.done,5,'NAV_ADMISSION');nav_goal=pending_nav.result();pending_nav=None;nav_admission_uncertain=False
        if not nav_goal.accepted: raise RuntimeError('NAV_REJECTED')
        nav_result=nav_goal.get_result_async()
        spin(lambda:pending_nav_id.hex in seen_nav_ids,3,'OWN_GOAL_STATUS_UNOBSERVED',True)
        event('nav_started',target_odom=target,target_map=mapped)
    def cancel_nav():
        nonlocal nav_goal,nav_result,pending_nav,nav_admission_uncertain
        if pending_nav is not None:
            spin(pending_nav.done,5,'PENDING_NAV_ADMISSION_UNRESOLVED')
            nav_goal=pending_nav.result();pending_nav=None;nav_admission_uncertain=False
            if nav_goal.accepted:nav_result=nav_goal.get_result_async()
        if nav_goal is not None:nav_admission_uncertain=False
        if nav_admission_uncertain:raise RuntimeError('NAV_ADMISSION_OUTCOME_UNKNOWN')
        if nav_goal is not None and nav_goal.accepted and nav_result is None:nav_result=nav_goal.get_result_async()
        if nav_goal is not None and nav_goal.accepted and nav_result is not None and not nav_result.done():
            future=nav_goal.cancel_goal_async();spin(future.done,5,'NAV_CANCEL_ACK');spin(nav_result.done,10,'NAV_CANCEL_TERMINAL')
        if nav_result is not None and nav_result.done():event('nav_terminal',status=nav_result.result().status)
        nav_goal=nav_result=None
    def finish_nav(target):
        spin(nav_result.done,180,'NAV_RESULT_TIMEOUT',True)
        if nav_result.result().status!=4: raise RuntimeError('NAV_NOT_SUCCEEDED:'+str(nav_result.result().status))
        observed=stop(ros())
        p=latest['odom'];position=math.hypot(p['x']-target[0],p['y']-target[1]);angle=abs(math.remainder(p['yaw']-target[2],2*math.pi))
        if position>.002 or angle>math.radians(.1): raise RuntimeError('ARRIVAL_OUT_OF_TOLERANCE:'+str((position,angle)))
        event('arrival',target=target,error_m=position,error_deg=math.degrees(angle),stop=observed)
        cancel_nav();return dict(position_m=position,angle_deg=math.degrees(angle),stop=observed)
    def release_hold():
        nonlocal hold_goal,hold_result,renew_enabled,pending_hold,hold_admission_uncertain
        renew_enabled=False
        if pending_hold is not None:
            spin(pending_hold.done,5,'PENDING_HOLD_ADMISSION_UNRESOLVED')
            hold_goal=pending_hold.result();pending_hold=None;hold_admission_uncertain=False
            if hold_goal.accepted:hold_result=hold_goal.get_result_async()
        if hold_goal is not None:hold_admission_uncertain=False
        if hold_admission_uncertain:raise RuntimeError('HOLD_ADMISSION_OUTCOME_UNKNOWN')
        if hold_goal is not None and hold_goal.accepted:
            if hold_result is None:hold_result=hold_goal.get_result_async()
            if not hold_result.done():
                future=hold_goal.cancel_goal_async();spin(future.done,3,'HOLD_CANCEL_ACK')
            spin(hold_result.done,15,'HOLD_RELEASE_TIMEOUT')
            value=hold_result.result()
            if not value.result.resources_released: raise RuntimeError('HOLD_RESOURCE_RELEASE_UNCONFIRMED')
            event('hold_released',status=value.status,reason=value.result.reason)
        hold_goal=hold_result=None
    def pause_planner():
        nonlocal suspended
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
        from sim_stack_supervisor import owned_descendants,process_identity
        candidates=[]
        for pid,started in owned_descendants([owner['pid']]).items():
            try:
                if Path(f'/proc/{pid}/exe').resolve().name=='planner_server': candidates.append((pid,started))
            except OSError: pass
        if len(candidates)!=1: raise RuntimeError('OWNED_PLANNER_AMBIGUOUS')
        pid,started=candidates[0];fd=os.pidfd_open(pid)
        try:
            verify_owner(owner,args.session,args.source)
            if process_identity(pid)[1]!=started: raise RuntimeError('PLANNER_IDENTITY_CHANGED')
            suspended=dict(pid=pid,start_ticks=started,fd=fd)
            signal.pidfd_send_signal(fd,signal.SIGSTOP);event('planner_suspended',pid=pid,start_ticks=started)
        except BaseException:
            if suspended is None: os.close(fd)
            raise
    def resume_planner():
        nonlocal suspended
        if suspended is not None:
            signal.pidfd_send_signal(suspended['fd'],signal.SIGCONT);os.close(suspended['fd']);suspended=None;event('planner_resumed')
    try:
        spin(lambda:hold_action.server_is_ready() and navigate.server_is_ready() and renew.service_is_ready() and fixed.service_is_ready() and fresh('odom') and fresh('command') and geometry_ready(),30,'ENTRY_UNAVAILABLE')
        duration(2.)
        spin(lambda:navigation_entry_observed(active_goals,verified_cold_start) and check_navigation_graph(),5,'NAVIGATION_IDLE_UNOBSERVED')
        if foreign_navigation():raise RuntimeError('OTHER_NAVIGATION_ACTIVE_AT_ENTRY')
        graph_ready=True
        report['cold_start_receipt']=receipt if verified_cold_start else None
        initial_stop=stop(ros());origin=latest['odom'].copy();yaw=origin['yaw']
        outward=[origin['x']+.8*math.cos(yaw),origin['y']+.8*math.sin(yaw),yaw]
        home=[origin['x'],origin['y'],yaw+math.pi/2]
        report.update(origin=origin,targets=[outward,home],initial_stop=initial_stop)
        epoch=acquire()
        for name,target in ((('positive_outward',outward),('positive_return_90deg',home)) if args.scenario in ('all','return_90') else ()):
            start_nav(target);arrival=finish_nav(target);scenarios.append(dict(name=name,passed=True,epoch=epoch,arrival=arrival))
        for mode in (('hold_cancel','consumer_pause') if args.scenario=='all' else (() if args.scenario=='return_90' else (args.scenario,))):
            if not positive(epoch):raise RuntimeError('LOST_BEFORE_MOTION')
            start=latest['odom'].copy();start_nav(outward)
            spin(lambda:nav_result.done() or (math.hypot(latest['odom']['vx'],latest['odom']['vy'])>.04 and math.hypot(latest['odom']['x']-start['x'],latest['odom']['y']-start['y'])>.025),60,'MOTION_NOT_OBSERVED',True)
            if nav_result.done():raise RuntimeError('NAV_TERMINATED_BEFORE_INJECTION')
            trigger_ros=ros();trigger_pose=latest['odom'].copy();begin=len(envelopes)
            event('fault_trigger',mode=mode,epoch=epoch,pose=trigger_pose)
            if mode=='hold_cancel':release_hold()
            else:pause_planner()
            spin(lambda:any(e['epoch']==epoch and not e['allowed'] for e in envelopes[begin:]),5,'AUTHORITY_NOT_REVOKED',True)
            observed=stop(trigger_ros);stopped=latest['odom'].copy()
            bad=[e for e in envelopes[begin:] if e['epoch']==epoch and not e['allowed']]
            if mode=='consumer_pause':
                if not any(e['reason'].startswith('WAITING_FOR:') and 'planner' in e['reason'] for e in bad):raise RuntimeError('WRONG_REVOCATION_SOURCE')
                if not fresh('hold') or not latest['hold'].hold_confirmed or not geometry_ready():raise RuntimeError('SOURCE_FAILED_IN_ACK_TEST')
            cancel_nav()
            previous=epoch
            if mode=='hold_cancel':
                duration(.7)
                if any(e['allowed'] for e in envelopes[begin:] if e['epoch']==previous):raise RuntimeError('REVOKED_HOLD_REAUTHORIZED')
                epoch=acquire()
                if epoch<=previous:raise RuntimeError('NEW_HOLD_WITHOUT_NEW_EPOCH')
            else:
                resume_planner();spin(lambda:positive(epoch),10,'ACK_RECOVERY_FAILED')
            event('recovered',mode=mode,old_epoch=previous,epoch=epoch)
            start_nav(outward);arrival=finish_nav(outward)
            scenarios.append(dict(name=mode,passed=True,old_epoch=previous,recovery_epoch=epoch,trigger_ros=trigger_ros,revocation=bad[0],stop=observed,
                                  stop_displacement_m=math.hypot(stopped['x']-trigger_pose['x'],stopped['y']-trigger_pose['y']),arrival=arrival))
            if args.scenario=='all':start_nav(home);finish_nav(home)
        report['passed']=True
    except BaseException as error:
        report['error']=repr(error)
    finally:
        cleanup_mode=True
        cleanup_errors=[];navigation_terminal=False;stopped=False
        for name,operation in (('resume_planner',resume_planner),('cancel_navigation',cancel_nav)):
            try:
                operation()
                if name=='cancel_navigation':navigation_terminal=True
            except BaseException as error:cleanup_errors.append(name+': '+repr(error))
        try:
            report['cleanup_stop']=stop(ros());stopped=True
        except BaseException as error:cleanup_errors.append('actual_stop: '+repr(error))
        if stopped and navigation_terminal:
            try:release_hold()
            except BaseException as error:cleanup_errors.append('release_hold: '+repr(error))
        else:cleanup_errors.append('HOLD_RETAINED_UNTIL_OWNED_STACK_SHUTDOWN_OR_LEASE_EXPIRY')
        cleanup_errors.extend(cleanup_health_errors)
        report['cleanup_complete']=not cleanup_errors
        if cleanup_errors:report.update(passed=False,cleanup_errors=cleanup_errors)
        report.update(finished_wall=time.time(),events=events,envelopes=envelopes,acks=acks,motion=motion,commands=commands,plans=plans,phases=phases,
                      execution_events=execution_events,pending_nav_uuid=pending_nav_id.hex if nav_admission_uncertain else None,pending_hold_uuid=pending_hold_id.hex if hold_admission_uncertain else None)
        (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('events','envelopes','acks','motion','commands')},indent=2),flush=True)
        diagnostic_stream.close();node.destroy_node();rclpy.shutdown();lease.close()
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
