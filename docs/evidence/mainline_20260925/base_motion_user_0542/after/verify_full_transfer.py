#!/usr/bin/env python3
"""Submit one C++ FixedStationTransfer; collect evidence and independently verify terminal EMPTY.

This client never sends navigation, controller, fixed-envelope, PICK or PLACE
subgoals. Runtime execution requires the owner's separately authorized runner.
"""
import copy
import math
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from verify_full_pick import ledger_summary, full_scene_matches, source_identity, verify_parameter_values


def goal_from_dict(values):
    from astribot_s1_transport_native.action import FixedStationTransfer
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.set_message import set_message_fields
    if set(values)!=set(FixedStationTransfer.Goal.get_fields_and_field_types()):
        raise RuntimeError('COMPLETE_EXACT_GOAL_FIELDS_REQUIRED')
    if FixedStationTransfer.Result.get_fields_and_field_types().get('final_attachment_revision')!='string':
        raise RuntimeError('FINAL_ATTACHMENT_REVISION_STRING_INTERFACE_REQUIRED')
    typed=copy.deepcopy(values)
    # Humble JSON uses one character for CollisionObject.byte; decoding needs
    # the corresponding byte value, not a string passed to bytes().
    for station in typed['stations']:
        station['operation']=station['operation'].encode('latin1')
    goal=FixedStationTransfer.Goal();set_message_fields(goal,typed)
    if message_to_ordereddict(goal)!=values:raise RuntimeError('GOAL_SERIALIZATION_CHANGED')
    return goal


def main():
    import argparse
    import gc
    import fcntl
    import hashlib
    import json
    import os
    import signal
    import time
    import uuid
    import yaml
    sys.path.insert(0,'/home/yjh/WorkSpace/astribot_sdk_ros2/tools/sim')
    from prepare_empty_inventory import verify_owner, CaptureReceipts, ReadbackBarrier, read_model_parameters
    from verify_fixed_navigation import measured_stop
    from capture_supervisor_owner import capture
    sys.path.insert(0,'/home/yjh/WorkSpace/astribot_sdk_ros2/tools')
    from sim_stack_supervisor import owned_descendants
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.action.graph import get_action_client_names_and_types_by_node, get_action_server_names_and_types_by_node
    from rclpy.serialization import serialize_message,deserialize_message
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict as message_dict
    from action_msgs.msg import GoalStatusArray
    from unique_identifier_msgs.msg import UUID
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rcl_interfaces.srv import GetParameters
    from moveit_msgs.srv import GetPlanningScene
    from astribot_payload_msgs.msg import AttachmentState, AttachmentObservation
    from astribot_navigation_msgs.msg import ArmHoldStatus, RobotGeometryState, NavigationEnvelopeV2, EnvelopeApplyStatus, NavigationExecutionStatus,MotionConstraint
    from astribot_s1_transport_native.action import FixedStationTransfer
    from astribot_s1_transport_native.srv import RenewHold
    from control_msgs.msg import JointTrajectoryControllerState

    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('owner','output','cold-start-receipt','goal-json'):
        parser.add_argument('--'+name,type=Path,required=True)
    for name in ('session','source','expected-binary-sha256','expected-executor-sha256','action-endpoint'):
        parser.add_argument('--'+name,required=True)
    parser.add_argument('--scenario',type=Path,default=Path(__file__).resolve().parent/'scenario.json')
    parser.add_argument('--executor-identity',type=Path)
    parser.add_argument('--executor-parameters',type=Path)
    parser.add_argument('--relax-base-motion',action='store_true')
    args=parser.parse_args()
    non_navigation_angular_speed_limit=.10 if args.relax_base_motion else .03
    if not args.action_endpoint.startswith('/') or args.action_endpoint.endswith('/'):
        raise RuntimeError('EXPLICIT_ABSOLUTE_TRANSFER_ENDPOINT_REQUIRED')
    if args.executor_identity is None:args.executor_identity=args.output.parent/'executor_owner.json'
    if args.executor_parameters is None:args.executor_parameters=args.output.parent/'executor_registration/executor.yaml'
    owner=json.loads(args.owner.read_text());verify_owner(owner,args.session,args.source)
    scenario=json.loads(args.scenario.read_text())
    if scenario['environment']!='simulation':raise RuntimeError('SIMULATION_ONLY')
    goal_dict=json.loads(args.goal_json.read_text())
    goal=goal_from_dict(goal_dict)
    if (not all((goal.task_id,goal.request_id,goal.context_id)) or goal.object_id!=scenario['object_id']
            or not math.isfinite(goal.timeout_s) or not 0<goal.timeout_s<=540):raise RuntimeError('TRANSFER_GOAL_SCOPE_INVALID')
    task_id=goal.task_id;context_id=goal.context_id;parent_request=goal.request_id
    cold=json.loads(args.cold_start_receipt.read_text());identities=cold['processes'];owned=owned_descendants([owner['pid']])
    executor_identity=json.loads(args.executor_identity.read_text())
    if not (cold['owner']==owner and cold['session']==args.session and cold['no_preceding_goal_writers'] is True
            and 0<=time.time()-cold['captured_wall']<120 and cold['probe_output']==str(args.output.resolve())
            and {'bt_navigator','task_arbiter_cpp'}.issubset(identities)
            and all(v['pid'] in owned and capture(v['pid'])==v for v in identities.values())):raise RuntimeError('COLD_START_RECEIPT_INVALID')
    if cold.get('fresh_executor')!=executor_identity or capture(executor_identity['pid'])!=executor_identity:
        raise RuntimeError('COLD_EXECUTOR_INSTANCE_MISMATCH')
    args.output.mkdir(parents=True,exist_ok=False)
    lease=open('/tmp/astribot_waypoint_'+os.environ['ROS_DOMAIN_ID']+'.lock','a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    journal=Path.home()/'.local/state/astribot/transport'/('domain_'+os.environ['ROS_DOMAIN_ID']+'.jsonl')
    journal_start=journal.stat().st_size if journal.exists() else 0
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('verify_fixed_station_transfer_'+uuid.uuid4().hex[:8],parameter_overrides=[Parameter('use_sim_time',value=True)])
    def interrupted(sig,frame):raise KeyboardInterrupt('owned transfer validation interrupted')
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    started=time.monotonic();deadline=started+540.
    latest={};receipts={};captures=CaptureReceipts();motion=[];commands=[];events=[];envelopes=[];acks=[];constraints=[]
    feedback_records=[];typed_hold_records=[];geometry_records=[];payload_records=[];jtc_records=[];navigation_records=[];nav_status_records=[];renew_events=[]
    active_nav={};unbound_nav_since={};navigation_uuids=set();readback=None;monitor=False;cleanup=False
    parent_goal=parent_result=pending_parent=None;parent_uuid=uuid.uuid4();binding=None;admission_uncertain=False
    renew_enabled=False;renew_sequence=0;renew_last=0.;renew_pending=None;renew_rejected=None
    report=dict(passed=False,started_wall=time.time(),started_steady=started,steady_budget_s=540,owner=owner,
                task_id=task_id,context_id=context_id,parent_request=parent_request,action_endpoint=args.action_endpoint,
                evidence_layer='owned_cpp_fixed_station_transfer_simulation',python_parent_goals_sent=0,
                python_navigation_goals_sent=0,python_controller_goals_sent=0,python_fixed_requests_sent=0,
                goal_json=str(args.goal_json),goal_sha256=hashlib.sha256(args.goal_json.read_bytes()).hexdigest(),goal=goal_dict,
                source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__).resolve(),Path(__file__).resolve().parent/'verify_full_pick.py')})
    report['base_motion_conditions']=dict(relaxed=args.relax_base_motion,
        acceptance_scope='relaxed_simulation_not_original_precision_acceptance' if args.relax_base_motion else 'original_simulation_precision_conditions',
        original_limits=dict(rotation_rad=.02,angular_speed_radps=.03),
        active_limits=dict(rotation_rad=.05 if args.relax_base_motion else .02,angular_speed_radps=non_navigation_angular_speed_limit),
        non_navigation_linear_speed_limit_mps=.02,nonzero_command_threshold=1e-6,final_stop_thresholds_unchanged=True)
    gc_events=[];gc_started={};slow_spins=[]
    def gc_timing(phase,info):
        generation=info['generation']
        if generation==0:return
        at=time.monotonic()
        if phase=='start':gc_started[generation]=at
        elif generation in gc_started:
            first=gc_started.pop(generation)
            gc_events.append(dict(generation=generation,start=first,end=at,duration_s=at-first,collected=info['collected']))
    gc.callbacks.append(gc_timing)
    action=ActionClient(node,FixedStationTransfer,args.action_endpoint)
    renew=node.create_client(RenewHold,'/transport/hold_executor/renew')
    params=node.create_client(GetParameters,'/robot_envelope_coordinator/get_parameters')
    scene=node.create_client(GetPlanningScene,'/get_planning_scene')
    controllers={'arm_left_controller','arm_right_controller','gripper_left_controller','gripper_right_controller','torso_controller','head_controller'}
    def ros():return node.get_clock().now().nanoseconds*1e-9
    def stamp(s):return s.sec+s.nanosec*1e-9
    def receive(key,value):latest[key]=value;receipts[key]=time.monotonic()
    def event(kind,**data):
        row=dict(kind=kind,wall=time.monotonic(),ros_s=ros(),**data);events.append(row);print(json.dumps(row),flush=True)
    def fresh(key):return key in latest and 0<=time.monotonic()-receipts[key]<.3
    def ledger(msg):
        value=message_dict(msg);wall=time.monotonic();captures.observe('ledger',value,wall)
        payload_records.append(dict(kind='ledger',wall=wall,ros_s=ros(),message=value));receive('ledger',value);observe_readback()
    def geometry(msg):
        geometry_records.append((time.monotonic(),ros(),serialize_message(msg)));receive('geometry',msg);observe_readback()
    def typed_hold(msg):
        row=dict(wall=time.monotonic(),ros_s=ros(),message=message_dict(msg));typed_hold_records.append(row);receive('hold',row)
    def physical_input(key,msg):
        value=json.loads(msg.data) if key=='diagnostic' else message_dict(msg);wall=time.monotonic();captures.observe(key,value,wall)
        row=dict(kind=key,wall=wall,ros_s=ros(),message=value);payload_records.append(row);receive(key,value)
    def jtc(controller,msg):
        jtc_records.append((controller,time.monotonic(),ros(),serialize_message(msg)))
    def command(msg):
        value=dict(wall=time.monotonic(),vx=msg.linear.x,vy=msg.linear.y,wz=msg.angular.z);commands.append(value);receive('command',value)
    def odom(msg):
        t=stamp(msg.header.stamp)
        if motion and t<=motion[-1]['t']:
            if t<motion[-1]['t']:latest['clock_error']='ODOMETRY_CLOCK_ROLLBACK'
            return
        q=msg.pose.pose.orientation;p=msg.pose.pose.position;v=msg.twist.twist;c=latest.get('command',{})
        value=dict(t=t,wall=time.monotonic(),x=p.x,y=p.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),vx=v.linear.x,vy=v.linear.y,wz=v.angular.z,frame=msg.header.frame_id,command_observed=bool(c),cmd_vx=c.get('vx',math.nan),cmd_vy=c.get('vy',math.nan),cmd_wz=c.get('wz',math.nan),cmd_wall=c.get('wall',-math.inf))
        motion.append(value);receive('odom',value)
    def constraint(msg):
        constraints.append(dict(wall=time.monotonic(),ros_s=ros(),message=message_dict(msg)));receive('constraint',msg)
    def envelope(msg):
        value=dict(wall=time.monotonic(),ros_s=ros(),stamp=stamp(msg.header.stamp),until=stamp(msg.valid_until),session=msg.coordinator_session_id,epoch=msg.epoch,request=msg.request_id,hash=msg.installed_geometry_hash,mode=msg.mode,mass=msg.limits.payload_mass_kg,allowed=msg.navigation_allowed,reason=msg.reason,hold_id=msg.hold_id,message=message_dict(msg))
        envelopes.append(value);receive('envelope',value)
    def ack(msg):
        acks.append(dict(wall=time.monotonic(),ros_s=ros(),message=message_dict(msg),stamp=stamp(msg.header.stamp),session=msg.coordinator_session_id,epoch=msg.envelope_epoch,hash=msg.installed_geometry_hash,consumer=msg.consumer_id,applied=msg.applied,reason=msg.reason))
    def nav_status(key,msg):
        nav_status_records.append(dict(action=key,wall=time.monotonic(),ros_s=ros(),message=message_dict(msg)))
        active_nav[key]={bytes(s.goal_info.goal_id.uuid).hex() for s in msg.status_list if s.status in (1,2,3)}
        for identity in active_nav[key]:
            if identity not in navigation_uuids and (key,identity) not in unbound_nav_since:
                unbound_nav_since[(key,identity)]=time.monotonic()
                event('navigation_binding_pending',action=key,navigation_goal_uuid=identity)
    def execution_status(msg):navigation_records.append(dict(wall=time.monotonic(),ros_s=ros(),message=message_dict(msg)))
    node.create_subscription(AttachmentState,'/payload/attachment_state',ledger,10)
    node.create_subscription(AttachmentObservation,'/payload/attachment_observation',lambda m:physical_input('source',m),32)
    node.create_subscription(String,'/payload/simulation_inventory_diagnostics',lambda m:physical_input('diagnostic',m),32)
    node.create_subscription(RobotGeometryState,'/navigation/geometry_state',geometry,10)
    node.create_subscription(ArmHoldStatus,'/navigation/arm_hold',typed_hold,10)
    node.create_subscription(NavigationEnvelopeV2,'/navigation/envelope_v2',envelope,30)
    node.create_subscription(EnvelopeApplyStatus,'/navigation/envelope_applied',ack,50)
    node.create_subscription(String,'/transport/hold_executor/status',lambda m:receive('executor',json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Twist,'/cmd_vel',command,qos_profile_sensor_data)
    node.create_subscription(MotionConstraint,'/navigation_policy/constraint',constraint,10)
    node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    node.create_subscription(NavigationExecutionStatus,'/navigation/execution_status',execution_status,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for controller in controllers:node.create_subscription(JointTrajectoryControllerState,'/'+controller+'/controller_state',lambda m,c=controller:jtc(c,m),qos_profile_sensor_data)
    for name in ('navigate_to_pose','navigate_through_poses'):
        node.create_subscription(GoalStatusArray,'/'+name+'/_action/status',lambda m,k=name:nav_status(k,m),QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def summary():
        value=ledger_summary(latest['ledger'],args.session,args.source,node.get_clock().now().nanoseconds)
        if not 0<=time.monotonic()-captures.received.get('ledger',-math.inf)<.3:raise ValueError('STALE_LEDGER_CAPTURE')
        g=latest['geometry']
        if not (fresh('geometry') and g.complete and g.attachment_state_confirmed and g.attachment_revision==value['attachment_revision'] and sorted(g.attachment_ids)==value['ids'] and 0<stamp(g.header.stamp)<=ros()<stamp(g.valid_until)):raise ValueError('LEDGER_GEOMETRY_MISMATCH')
        value.update(model_revision=g.model_revision,geometry_source=g.source_id,geometry_clock_epoch=g.clock_epoch,geometry_frame=g.header.frame_id);return value
    def ready():
        try:summary();return True
        except (KeyError,ValueError):return False
    def observe_readback():
        if readback is None:return
        try:value=summary();valid=True;key=source_identity(value)
        except (KeyError,ValueError):valid=False;key=None
        readback.observe(valid,key,time.monotonic(),node.get_clock().now().nanoseconds)
    def check_graph(expected_native_count=0):
        known={('bt_navigator','/navigation_executor'):'bt_navigator',('navigation_task_arbiter','/'):'task_arbiter_cpp',('operator_backend','/'):'operator_backend',('loop_route_executor','/'):'loop_route_executor',('waypoint_follower','/'):'waypoint_follower'}
        native=[]
        nodes=node.get_node_names_and_namespaces()
        if nodes.count(('task_trajectory_executor','/'))!=1:raise RuntimeError('NATIVE_EXECUTOR_NODE_NOT_UNIQUE')
        if capture(executor_identity['pid'])!=executor_identity:raise RuntimeError('COLD_EXECUTOR_INSTANCE_CHANGED')
        for name,namespace in nodes:
            for path,types in get_action_client_names_and_types_by_node(node,name,namespace):
                if path.endswith(('/navigate_to_pose','/navigate_through_poses')):
                    if (name,namespace)==('task_trajectory_executor','/'):
                        if path!='/navigate_to_pose' or 'nav2_msgs/action/NavigateToPose' not in types:raise RuntimeError('UNEXPECTED_NATIVE_NAVIGATION_CLIENT')
                        native.append((name,namespace,path))
                    elif known.get((name,namespace)) not in identities:raise RuntimeError('FOREIGN_NAVIGATION_CLIENT:'+name)
        if len(native)!=expected_native_count:
            if expected_native_count==1 and not native:return False
            raise RuntimeError('NATIVE_NAVIGATION_CLIENT_LIFECYCLE_MISMATCH')
        for identity in identities.values():
            if capture(identity['pid'])!=identity:raise RuntimeError('COLD_START_PROCESS_CHANGED')
        report['native_navigation_client']=native
        return True
    def controller_graph(discovery=False):
        expected={'/'+c+'/follow_joint_trajectory':[] for c in controllers}
        for name,namespace in node.get_node_names_and_namespaces():
            for path,types in get_action_client_names_and_types_by_node(node,name,namespace):
                if path in expected:expected[path].append(dict(node=name,namespace=namespace,types=types))
        report['controller_action_graph']=expected
        if any(len(rows)>1 or rows and (rows[0]['node']!='task_trajectory_executor' or rows[0]['namespace']!='/') for rows in expected.values()):raise RuntimeError('FOREIGN_FJT_CLIENT')
        if any(not rows for rows in expected.values()):
            if discovery:return False
            raise RuntimeError('MISSING_FJT_CLIENT')
        return True
    def feedback(message):
        nonlocal binding
        if bytes(message.goal_id.uuid).hex()!=parent_uuid.hex:return
        v=message.feedback;row=dict(wall=time.monotonic(),ros_s=ros(),goal_id=parent_uuid.hex,feedback=message_dict(v));feedback_records.append(row);receive('parent_feedback',row)
        if v.context_id!=context_id or v.object_id!=goal.object_id:latest['ownership_error']='PARENT_FEEDBACK_CONTEXT_MISMATCH';return
        if v.phase not in ('PICK','NAVIGATE','PLACE','FINAL_VERIFY','COMPLETE'):latest['ownership_error']='PARENT_FEEDBACK_PHASE_INVALID';return
        if v.lease_id or v.resource_epoch:
            identity=(v.lease_id,v.resource_epoch)
            if not all(identity) or binding is not None and binding!=identity:latest['ownership_error']='PARENT_FEEDBACK_LEASE_CHANGED';return
            binding=identity
        if v.navigation_goal_uuid:
            try:valid=uuid.UUID(hex=v.navigation_goal_uuid).hex==v.navigation_goal_uuid
            except ValueError:valid=False
            if not valid:latest['ownership_error']='NAVIGATION_UUID_INVALID';return
            navigation_uuids.add(v.navigation_goal_uuid)
            if len(navigation_uuids)>1:latest['ownership_error']='MULTIPLE_NAVIGATION_UUIDS';return
            for (action_name,pending_uuid),first_seen in list(unbound_nav_since.items()):
                if action_name!='navigate_to_pose' or pending_uuid!=v.navigation_goal_uuid:
                    latest['ownership_error']='FOREIGN_NAVIGATION_UUID_DURING_BINDING';return
                if v.phase!='NAVIGATE' or row['wall']-first_seen>=.3:
                    latest['ownership_error']='NAVIGATION_UUID_BINDING_DEADLINE';return
                del unbound_nav_since[(action_name,pending_uuid)]
                event('navigation_binding_confirmed',navigation_goal_uuid=pending_uuid,delay_s=row['wall']-first_seen)
    def health():
        observe_readback()
        if latest.get('ownership_error'):raise RuntimeError(latest['ownership_error'])
        if latest.get('clock_error'):raise RuntimeError(latest['clock_error'])
        now=time.monotonic();phase=latest.get('parent_feedback',{}).get('feedback',{}).get('phase')
        # Public Nav status and parent feedback arrive on different DDS topics.
        # Only one new candidate belonging to this live parent may await its
        # exact feedback UUID; disappearance does not erase the obligation.
        pending_binding=bool(unbound_nav_since)
        for (key,identity),first_seen in unbound_nav_since.items():
            status=latest.get('executor',{})
            own_parent=(parent_goal is not None and parent_goal.accepted and parent_result is not None
                        and not parent_result.done() and binding is not None and fresh('parent_feedback')
                        and fresh('executor') and (status.get('lease_id'),status.get('epoch'))==binding)
            if (key!='navigate_to_pose' or navigation_uuids or len(unbound_nav_since)!=1
                    or not own_parent or phase not in ('PICK','NAVIGATE') or now-first_seen>=.3):
                raise RuntimeError('FOREIGN_OR_UNBOUND_NAVIGATION_ACTIVE:'+identity)
        for key,ids in active_nav.items():
            if key!='navigate_to_pose' and ids:raise RuntimeError('FOREIGN_NAVIGATION_ACTION_ACTIVE:'+key)
            if any(identity not in navigation_uuids and (key,identity) not in unbound_nav_since for identity in ids):
                raise RuntimeError('UNATTRIBUTED_NAVIGATION_ACTIVE')
        c=latest.get('command')
        if c and any(not math.isfinite(c[k]) for k in ('vx','vy','wz')):raise RuntimeError('INVALID_CHASSIS_COMMAND')
        if monitor and not cleanup:
            if not fresh('odom') or not 0<=ros()-latest['odom']['t']<.3:
                report.setdefault('first_odom_stale',dict(wall=time.monotonic(),ros_s=ros(),odom=copy.deepcopy(latest.get('odom')),last_receipt=receipts.get('odom'),gc_tail=gc_events[-3:],slow_spin_tail=slow_spins[-3:]))
                raise RuntimeError('ODOM_STALE')
            if phase!='NAVIGATE' and not pending_binding:
                if c and any(abs(c[k])>1e-6 for k in ('vx','vy','wz')):raise RuntimeError('NONZERO_CHASSIS_COMMAND_OUTSIDE_NAVIGATION')
                v=latest['odom']
                if math.hypot(v['vx'],v['vy'])>.02 or abs(v['wz'])>non_navigation_angular_speed_limit:raise RuntimeError('UNEXPECTED_MOTION_OUTSIDE_NAVIGATION')
    def spin(predicate,timeout,reason):
        nonlocal renew_pending,renew_last,renew_sequence,renew_enabled,renew_rejected
        end=time.monotonic()+timeout
        while True:
            if not cleanup:
                health()
                if time.monotonic()>deadline:raise RuntimeError('TRANSFER_CLIENT_540S_DEADLINE')
            if predicate():return
            now=time.monotonic()
            if now>end or not cleanup and now>deadline:raise RuntimeError(reason if now<=deadline or cleanup else 'TRANSFER_CLIENT_540S_DEADLINE')
            spin_started=time.monotonic()
            rclpy.spin_once(node,timeout_sec=.01)
            spin_finished=time.monotonic()
            if spin_finished-spin_started>.05:slow_spins.append(dict(start=spin_started,end=spin_finished,duration_s=spin_finished-spin_started))
            if parent_result is not None and parent_result.done():renew_enabled=False
            if renew_pending is not None and renew_pending.done():
                completed=renew_pending;renew_pending=None;answer=completed.result()
                renew_events.append(dict(wall=time.monotonic(),ros_s=ros(),kind='response',accepted=answer.accepted,reason=answer.reason))
                if renew_enabled and not answer.accepted:renew_enabled=False;renew_rejected=(time.monotonic(),answer.reason)
            if renew_rejected and not cleanup and parent_result is not None and not parent_result.done() and time.monotonic()-renew_rejected[0]>=.3:raise RuntimeError('RENEW_REJECTED:'+renew_rejected[1])
            status=latest.get('executor',{})
            if renew_enabled and binding is not None and parent_goal is not None and parent_goal.accepted and fresh('executor') and (status.get('lease_id'),status.get('epoch'))==binding and renew_pending is None and time.monotonic()-renew_last>.2:
                renew_sequence+=1;renew_last=time.monotonic();renew_pending=renew.call_async(RenewHold.Request(lease_id=binding[0],resource_epoch=binding[1],sequence=renew_sequence))
                renew_events.append(dict(wall=renew_last,ros_s=ros(),kind='request',sequence=renew_sequence,lease=binding[0],resource_epoch=binding[1]))
    def idle_stop_context():
        c=latest.get('constraint');status=latest.get('executor',{})
        return (fresh('constraint') and fresh('executor') and status.get('phase')=='0' and not any(active_nav.values())
                and c is not None and c.hold and c.max_linear_speed==0. and c.max_angular_speed==0.
                and 0<c.lease_s<=.5 and 0<stamp(c.stamp)<=ros()<stamp(c.stamp)+c.lease_s)
    def stop():
        after=ros()
        accepted=parent_goal is not None and parent_goal.accepted
        submission=next((row['wall'] for row in events if row.get('kind')=='parent_submission'),None)
        def stopped():
            if accepted and (parent_result is None or not parent_result.done()):return False
            current=latest.get('command')
            if current and (any(not math.isfinite(current[k]) or abs(current[k])>1e-6 for k in ('vx','vy','wz'))
                    or accepted and (submission is None or current['wall']<submission)):
                return False
            idle_silence=not accepted and not commands and idle_stop_context()
            if not accepted and not idle_stop_context():return False
            return measured_stop(motion,after,time.monotonic(),ros(),event_driven_command=True,
                idle_command_silence=idle_silence)['passed']
        spin(stopped,15,'ACTUAL_STOP_TIMEOUT')
        return measured_stop(motion,after,time.monotonic(),ros(),event_driven_command=True,
            idle_command_silence=not accepted and not commands and idle_stop_context())
    def release():
        nonlocal renew_enabled,pending_parent,parent_goal,parent_result,admission_uncertain
        renew_enabled=False
        if pending_parent is not None:
            spin(pending_parent.done,5,'PENDING_PARENT_ADMISSION');parent_goal=pending_parent.result();pending_parent=None;admission_uncertain=False
        if admission_uncertain:raise RuntimeError('PARENT_ADMISSION_UNKNOWN')
        if parent_goal is None or not parent_goal.accepted:return
        if parent_result is None:parent_result=parent_goal.get_result_async()
        if not parent_result.done():
            event('cancel_requested',goal_uuid=parent_uuid.hex,identity=binding)
            future=parent_goal.cancel_goal_async();spin(future.done,3,'CANCEL_PARENT');event('cancel_response',response=message_dict(future.result()))
        spin(parent_result.done,20,'PARENT_RESOURCE_RELEASE')
        result=parent_result.result();report['parent_terminal']=dict(status=result.status,result=message_dict(result.result))
        event('parent_terminal',**report['parent_terminal'])
        if not result.result.resources_released:raise RuntimeError('RESOURCE_RELEASE_UNCONFIRMED')
    try:
        candidates=[]
        for pid in owned_descendants([owner['pid']]):
            try:
                identity=capture(pid)
                if Path(identity['exe']).name=='fixed_envelope_cpp': candidates.append(identity)
            except (OSError, ProcessLookupError): pass
        if len(candidates)!=1: raise RuntimeError('FIXED_BINARY_AMBIGUOUS')
        identity=candidates[0]; binary=Path(identity['exe']); digest=hashlib.sha256(binary.read_bytes()).hexdigest()
        report['binary']=dict(identity=identity,sha256=digest,maps=Path(f"/proc/{identity['pid']}/maps").read_text())
        if digest != args.expected_binary_sha256: raise RuntimeError('FIXED_BINARY_SHA_MISMATCH')
        response=read_model_parameters(params,GetParameters.Request(names=['payload_environment','payload_session_id','payload_source_id']),lambda:rclpy.spin_once(node,timeout_sec=.01),events=events)
        values=[v.string_value for v in response.values]; report['identity_parameters']=values
        if values != ['simulation',args.session,args.source]: raise RuntimeError('PAYLOAD_IDENTITY_PARAMETERS_MISMATCH')
        observer_identity=json.loads((args.output.parent/'observer_owner.json').read_text())
        if capture(observer_identity['pid'])!=observer_identity:raise RuntimeError('M5_OBSERVER_OWNER_CHANGED')
        observation=args.output.parent/'m5_observer'
        observer_manifest=json.loads((observation/'manifest.json').read_text())
        for key in ('ROS_DOMAIN_ID','IGN_PARTITION','GZ_PARTITION'):
            if observer_manifest['environment'].get(key)!=os.environ.get(key):raise RuntimeError('M5_OBSERVER_ENVIRONMENT_MISMATCH:'+key)
        observer_deadline=time.monotonic()+5.
        while True:
            observer_nodes=[v for v in node.get_node_names_and_namespaces() if v[0]=='m5_passive_observer']
            report.setdefault('observer_discovery_first_nodes',observer_nodes)
            report['observer_discovery_nodes']=observer_nodes
            if observer_nodes:
                if observer_nodes!=[('m5_passive_observer','/')]:raise RuntimeError('M5_OBSERVER_NODE_AMBIGUOUS_OR_WRONG_NAMESPACE')
                break
            if time.monotonic()>=observer_deadline:raise RuntimeError('M5_OBSERVER_NODE_NOT_DISCOVERED')
            rclpy.spin_once(node,timeout_sec=.01)
        observer_config=Path(__file__).resolve().parent/'observer_topics_transfer.json'
        observer_config_sha=hashlib.sha256(observer_config.read_bytes()).hexdigest()
        expected_specs=json.loads(observer_config.read_text())['topics']
        expected_specs.append(dict(topic='/transport/hold_executor/status',type='std_msgs/msg/String',kind='status',required=False,gap_budget_sec=None,qos='reliable'))
        if (len(expected_specs)!=43 or observer_manifest['topics']!=expected_specs
                or observer_manifest['source_sha256'].get(str(observer_config))!=observer_config_sha
                or observer_manifest.get('error')
                or observer_manifest['phase_source']['topic']!='/transport/hold_executor/status'):
            raise RuntimeError('M5_OBSERVER_CONFIGURATION_MISMATCH')
        expected_subscriptions={v['topic']:v['type'] for v in expected_specs}
        subscription_deadline=time.monotonic()+5.
        while True:
            actual_subscriptions=dict(node.get_subscriber_names_and_types_by_node('m5_passive_observer','/'))
            report['observer_discovery_subscriptions']=actual_subscriptions
            if any(topic in actual_subscriptions and typ not in actual_subscriptions[topic] for topic,typ in expected_subscriptions.items()):raise RuntimeError('M5_OBSERVER_SUBSCRIPTION_TYPE_MISMATCH')
            if all(topic in actual_subscriptions for topic in expected_subscriptions):break
            if time.monotonic()>=subscription_deadline:raise RuntimeError('M5_OBSERVER_SUBSCRIPTIONS_INCOMPLETE')
            rclpy.spin_once(node,timeout_sec=.01)
        observed_topics=set()
        with (observation/'events.jsonl').open() as stream:
            for line in stream:
                if line.endswith('\n'):observed_topics.add(json.loads(line)['topic'])
        if not {'/clock','/transport/execution_guard/status','/transport/hold_executor/status'}.issubset(observed_topics):raise RuntimeError('M5_OBSERVER_REQUIRED_OUTPUT_MISSING')
        publishers={topic:[dict(node=x.node_name,namespace=x.node_namespace,type=x.topic_type,gid=list(x.endpoint_gid),qos=dict(reliability=int(x.qos_profile.reliability),durability=int(x.qos_profile.durability),depth=x.qos_profile.depth)) for x in node.get_publishers_info_by_topic(topic)] for topic in expected_subscriptions}
        report['observer_pre_action']=dict(identity=observer_identity,manifest_sha256=hashlib.sha256((observation/'manifest.json').read_bytes()).hexdigest(),subscriptions=actual_subscriptions,publishers=publishers,topics_recorded=sorted(observed_topics),ros_s=ros(),wall=time.monotonic())
        (args.output.parent/'observer_pre_action.json').write_text(json.dumps(report['observer_pre_action'],indent=2)+'\n')
        registered=yaml.safe_load(args.executor_parameters.read_text())['/**']['ros__parameters']
        registration=json.loads((args.executor_parameters.parent/'result.json').read_text())
        if not (registration['passed'] and registration['owner']==owner and registration['session']==args.session and registration['source']==args.source and registration['scenario_sha256']==hashlib.sha256(args.scenario.read_bytes()).hexdigest()):raise RuntimeError('EXECUTOR_REGISTRATION_CONTEXT_MISMATCH')
        if {k:v for k,v in registered.items() if k!='robot_description'}!=registration['registered_parameters']:raise RuntimeError('EXECUTOR_REGISTRATION_PARAMETERS_CHANGED')
        executor_params=node.create_client(GetParameters,'/task_trajectory_executor/get_parameters')
        names=list(registered)
        actual_parameters=read_model_parameters(executor_params,GetParameters.Request(names=names),lambda:rclpy.spin_once(node,timeout_sec=.01),events=events)
        decoded=verify_parameter_values(registered,[message_dict(v) for v in actual_parameters.values])
        xml_sha=hashlib.sha256(decoded['robot_description'].encode()).hexdigest()
        if xml_sha!=registration['xml_sha256']['robot_state_publisher']:raise RuntimeError('EXECUTOR_REGISTERED_XML_CHANGED')
        report['executor_registered_parameters']=dict(path=str(args.executor_parameters),yaml_sha256=hashlib.sha256(args.executor_parameters.read_bytes()).hexdigest(),actual={k:v for k,v in decoded.items() if k!='robot_description'},xml_sha256=xml_sha,wall=time.monotonic(),ros_s=ros())
        if capture(executor_identity['pid'])!=executor_identity:raise RuntimeError('M1_EXECUTOR_CAPTURE_MISMATCH')
        if Path(executor_identity['exe']).name!='trajectory_executor':raise RuntimeError('M1_EXECUTOR_WRONG_EXECUTABLE')
        executor_environment=dict(item.split('=',1) for item in Path('/proc/%s/environ'%executor_identity['pid']).read_bytes().decode().split('\0') if '=' in item)
        query_environment=owner.get('query_environment',{})
        for key in ('ROS_DOMAIN_ID','IGN_PARTITION','GZ_PARTITION'):
            if not os.environ.get(key) or executor_environment.get(key)!=os.environ[key] or query_environment.get(key)!=os.environ[key]:raise RuntimeError('M1_EXECUTOR_QUERY_ENVIRONMENT_MISMATCH:'+key)
        executor_digest=hashlib.sha256(Path(executor_identity['exe']).read_bytes()).hexdigest()
        if executor_digest!=args.expected_executor_sha256:raise RuntimeError('M1_EXECUTOR_SHA_MISMATCH')
        server_deadline=time.monotonic()+5.
        while True:
            servers=[]
            for name,namespace in node.get_node_names_and_namespaces():
                for path,types in get_action_server_names_and_types_by_node(node,name,namespace):
                    if path==args.action_endpoint:servers.append((name,namespace,types))
            report['action_discovery_servers']=servers
            if servers:
                if len(servers)!=1 or tuple(servers[0][:2])!=('task_trajectory_executor','/') or 'astribot_s1_transport_native/action/FixedStationTransfer' not in servers[0][2]:raise RuntimeError('M1_ACTION_SERVER_NOT_UNIQUE')
                break
            if time.monotonic()>=server_deadline:raise RuntimeError('M1_ACTION_SERVER_NOT_DISCOVERED')
            rclpy.spin_once(node,timeout_sec=.01)
        report['executor_binary']=dict(identity=executor_identity,sha256=executor_digest,query_environment={k:executor_environment[k] for k in ('ROS_DOMAIN_ID','IGN_PARTITION','GZ_PARTITION')},action_servers=servers)
        spin(lambda:action.server_is_ready() and renew.service_is_ready() and ready() and fresh('odom') and idle_stop_context(),30,'ENTRY_UNAVAILABLE')
        monitor=True; check_graph(); report['initial_stop']=stop(); initial=summary(); report['initial_ledger']=initial
        if not scene.wait_for_service(timeout_sec=3): raise RuntimeError('SCENE_UNAVAILABLE')
        if initial['ids'] or initial['mass_kg']!=0.:raise RuntimeError('M1_REQUIRES_EMPTY')
        if latest.get('envelope',{}).get('allowed'):raise RuntimeError('M1_POSITIVE_NAVIGATION_AT_ENTRY')
        mtc_parameters=node.create_client(GetParameters,'/transport_mtc_planner/get_parameters')
        mtc_names=['trajectory_time_scaling','max_velocity_scaling','max_acceleration_scaling','joint_limit_margin_rad']
        mtc_response=read_model_parameters(mtc_parameters,GetParameters.Request(names=mtc_names),lambda:rclpy.spin_once(node,timeout_sec=.01),events=events)
        mtc_values=[v.double_value for v in mtc_response.values]
        report['mtc_parameters']=dict(zip(mtc_names,mtc_values))
        (args.output.parent/'mtc_parameters_pre_action.json').write_text(json.dumps(report['mtc_parameters'],indent=2)+'\n')
        if mtc_values!=[2.5,.1,.1,.1]:raise RuntimeError('MTC_TIME_SCALING_PARAMETERS_MISMATCH')
        spin(lambda:controller_graph(discovery=True),5,'FJT_CLIENT_DISCOVERY_TIMEOUT')
        req=GetPlanningScene.Request();req.components.components=1023
        future=scene.call_async(req);spin(future.done,5,'INITIAL_FULL_SCENE_READBACK')
        initial_scene=message_dict(future.result().scene);report['initial_full_scene']=initial_scene
        if not full_scene_matches([],initial_scene):raise RuntimeError('INITIAL_FULL_EMPTY_SCENE_REQUIRED')
        target_objects=[o for o in initial_scene['world']['collision_objects'] if o['id']==goal.object_id]
        if len(target_objects)!=1:raise RuntimeError('INITIAL_SCENE_TARGET_NOT_UNIQUE')
        report['initial_ledger_raw']=copy.deepcopy(latest['ledger'])
        report['initial_geometry_raw']=message_dict(latest['geometry'])
        if any(active_nav.values()):raise RuntimeError('NAVIGATION_ACTIVE_AT_TRANSFER_ENTRY')
        check_graph();controller_graph()
        if capture(executor_identity['pid'])!=executor_identity:raise RuntimeError('EXECUTOR_CHANGED_BEFORE_SUBMISSION')
        # Bind this acceptance condition to both live simulation consumers before any Goal.
        report['base_motion_conditions']['readback']={}
        for target,client in (('task_trajectory_executor',executor_params),('manipulation_execution_guard',node.create_client(GetParameters,'/manipulation_execution_guard/get_parameters'))):
            response=read_model_parameters(client,GetParameters.Request(names=['simulation_relaxed_base_motion']),lambda:rclpy.spin_once(node,timeout_sec=.01),events=events)
            report['base_motion_conditions']['readback'][target]=[message_dict(v) for v in response.values]
            if len(response.values)!=1 or response.values[0].type!=1 or response.values[0].bool_value is not args.relax_base_motion:raise RuntimeError('SIMULATION_BASE_MOTION_CONDITION_MISMATCH:'+target)
        event('parent_submission',goal=goal_dict,goal_uuid=parent_uuid.hex,remaining_steady_s=deadline-time.monotonic())
        admission_uncertain=True;renew_enabled=True
        pending_parent=action.send_goal_async(goal,goal_uuid=UUID(uuid=list(parent_uuid.bytes)),feedback_callback=feedback)
        report['python_parent_goals_sent']=1
        spin(pending_parent.done,5,'PARENT_ADMISSION');parent_goal=pending_parent.result();pending_parent=None;admission_uncertain=False
        if not parent_goal.accepted:raise RuntimeError('PARENT_REJECTED')
        parent_result=parent_goal.get_result_async()
        spin(lambda:parent_result.done() or check_graph(1),5,'NATIVE_NAVIGATION_CLIENT_DISCOVERY_TIMEOUT')
        spin(parent_result.done,540,'TRANSFER_PARENT_RESULT_TIMEOUT');renew_enabled=False
        wrapped=parent_result.result();terminal=message_dict(wrapped.result)
        report['parent_terminal']=dict(status=wrapped.status,result=terminal);event('parent_result',**report['parent_terminal'])
        if wrapped.status!=4 or not wrapped.result.success or not wrapped.result.resources_released or wrapped.result.reason!='TRANSFER_COMPLETE':raise RuntimeError('TRANSFER_NOT_SUCCESSFULLY_RELEASED')
        if binding is None or wrapped.result.lease_id!=binding[0]:raise RuntimeError('TERMINAL_LEASE_MISMATCH')
        if navigation_uuids!={wrapped.result.navigation_goal_uuid} or not wrapped.result.navigation_goal_uuid:raise RuntimeError('TERMINAL_NAVIGATION_UUID_MISMATCH')
        submission_wall=next(row['wall'] for row in events if row.get('kind')=='parent_submission')
        for row in nav_status_records:
            if row['wall']<submission_wall:continue
            for state in row['message']['status_list']:
                identity=bytes(state['goal_info']['goal_id']['uuid']).hex()
                if state['status'] in (1,2,3) and (row['action']!='navigate_to_pose' or identity not in navigation_uuids):raise RuntimeError('UNATTRIBUTED_NAVIGATION_GOAL_IN_RECORD')
        phases={row['feedback']['phase'] for row in feedback_records}
        if not {'PICK','NAVIGATE','PLACE'}.issubset(phases):raise RuntimeError('TRANSFER_PHASE_EVIDENCE_INCOMPLETE')
        spin(lambda:ready() and fresh('envelope') and 0<=ros()-latest['envelope']['stamp']<.3 and not latest['envelope']['allowed'] and not any(active_nav.values()),5,'FINAL_EMPTY_DENIAL_UNAVAILABLE')
        final=summary()
        if final['ids'] or final['mass_kg']!=0.:raise RuntimeError('FINAL_AUTHORITATIVE_EMPTY_REQUIRED')
        if not isinstance(wrapped.result.final_attachment_revision,str) or wrapped.result.final_attachment_revision!=final['attachment_revision']:raise RuntimeError('FINAL_ATTACHMENT_REVISION_MISMATCH')
        for key in ('ledger_epoch','source_epoch','clock_epoch','model_revision','geometry_source','geometry_clock_epoch','geometry_frame'):
            if final[key]!=initial[key]:raise RuntimeError('FINAL_SOURCE_CONTEXT_CHANGED:'+key)
        if final['source_revision']<=initial['source_revision'] or final['attachment_revision']==initial['attachment_revision']:raise RuntimeError('FINAL_PAYLOAD_TRANSITION_UNPROVEN')
        readback=ReadbackBarrier(source_identity(final),time.monotonic(),node.get_clock().now().nanoseconds)
        future=scene.call_async(req);spin(future.done,5,'FINAL_INDEPENDENT_FULL_SCENE')
        final_scene=message_dict(future.result().scene);observe_readback()
        if not readback.valid or source_identity(summary())!=source_identity(final):raise RuntimeError('FINAL_SCENE_SOURCE_CHANGED_OR_EXPIRED')
        readback=None
        if not full_scene_matches([],final_scene):raise RuntimeError('FINAL_SCENE_ATTACHMENTS_REMAIN')
        objects=[o for o in final_scene['world']['collision_objects'] if o['id']==goal.object_id]
        if len(objects)!=1:raise RuntimeError('FINAL_WORLD_TARGET_NOT_UNIQUE')
        obj=objects[0]
        # Bind retained collision geometry to this parent's completed physical transactions.
        with journal.open() as stream:
            stream.seek(journal_start);world_journal=[json.loads(line) for line in stream if line.strip()]
        world_journal=[r for r in world_journal if r.get('lease_id')==binding[0] and r.get('epoch')==binding[1] and r.get('owner')==task_id]
        world_transactions=[]
        for stage in ('ATTACH_CONFIRM','DETACH_CONFIRM'):
            submitted=[(i,r) for i,r in enumerate(world_journal) if r.get('event')=='physical_submission' and r['details']['stage'].split(':')[0]==stage]
            if len(submitted)!=1:raise RuntimeError('FINAL_WORLD_PHYSICAL_SUBMISSION_REQUIRED:'+stage)
            si,submission=submitted[0];transaction=submission['details']['transaction']
            applied=[(i,r) for i,r in enumerate(world_journal) if r.get('event')=='physical_applied_scene_submission' and r['details']['transaction']==transaction]
            confirmed=[(i,r) for i,r in enumerate(world_journal) if r.get('event')=='payload_transaction_confirmed' and r['details']['transaction']==transaction]
            if len(applied)!=1 or len(confirmed)!=1:raise RuntimeError('FINAL_WORLD_CONFIRMED_TRANSACTION_REQUIRED:'+stage)
            ai,application=applied[0];ci,confirmation=confirmed[0]
            if not (si<ai<ci and confirmation['details']['stage'].split(':')[0]==stage and application['details']['command_id']==confirmation['details']['command_id'] and submission['details']['source_epoch']==final['source_epoch'] and submission['details']['clock_epoch']==final['clock_epoch']):raise RuntimeError('FINAL_WORLD_TRANSACTION_BINDING_MISMATCH:'+stage)
            world_transactions.append(dict(submission_index=si,application_index=ai,confirmation_index=ci,submission=submission['details'],application=application['details'],confirmation=confirmation['details']))
        attached,detached=world_transactions
        if not (attached['confirmation_index']<detached['submission_index'] and detached['application']['command_id']==attached['application']['command_id']+1 and attached['application']['source_revision']<detached['application']['source_revision']==final['source_revision'] and detached['confirmation']['attachment_revision']==final['attachment_revision']):raise RuntimeError('FINAL_WORLD_PAYLOAD_TRANSITION_MISMATCH')
        sources=[(i,r) for i,r in enumerate(payload_records) if r['kind']=='source' and r['message']['environment']=='simulation' and r['message']['session_id']==args.session and r['message']['source_id']==args.source and r['message']['source_epoch']==final['source_epoch'] and r['message']['clock_epoch']==final['clock_epoch'] and r['message']['revision']==attached['application']['source_revision'] and r['message']['sequence']==attached['application']['source_sequence']]
        if len(sources)!=1:raise RuntimeError('FINAL_WORLD_BOUND_SOURCE_REQUIRED')
        source_index,source_record=sources[0];source=source_record['message']
        if not (source['full_inventory'] and source['status']==2 and len(source['objects'])==1 and source['objects'][0]['object']['id']==goal.object_id):raise RuntimeError('FINAL_WORLD_BOUND_INVENTORY_MISMATCH')
        ledgers=[(i,r) for i,r in enumerate(payload_records) if r['kind']=='ledger' and r['message']['confirmed'] and r['message']['ledger_epoch']==final['ledger_epoch'] and r['message']['attachment_revision']==attached['confirmation']['attachment_revision'] and all(r['message']['observation'][k]==source[k] for k in ('environment','session_id','source_id','source_epoch','clock_epoch','revision','full_inventory','status','objects'))]
        if not ledgers:raise RuntimeError('FINAL_WORLD_BOUND_LEDGER_REQUIRED')
        ledger_index,ledger_record=ledgers[-1];expected=source['objects'][0]['object']
        report['final_world_geometry_source']=dict(transactions=world_transactions,source_record_index=source_index,ledger_record_index=ledger_index,source=source,ledger=ledger_record['message'],expected_primitives=expected['primitives'],actual_primitives=obj['primitives'],independent_pre_detach_scene=False)
        if not obj['header']['frame_id'] or len(obj['primitives'])!=1 or obj['primitives'][0]['type']!=1 or obj['primitives']!=expected['primitives'] or len(obj['primitive_poses'])!=1 or obj['primitive_poses']!=expected['primitive_poses'] or obj['meshes'] or obj['planes']:raise RuntimeError('FINAL_WORLD_TARGET_GEOMETRY_MISMATCH')
        report['final_full_scene']=final_scene;report['final_ledger']=copy.deepcopy(latest['ledger']);report['final_geometry']=message_dict(latest['geometry']);report['final_source_identity']=final
        # Prove returned/adopted/sent suffix identity only after the parent succeeds.
        report['trajectory_handoff']=dict(journal_scope='owner+lease_id+epoch since journal_start',operations=[])
        accepted_children=set()
        for operation,physical,names in (('PICK',attached,('LIFT','TRANSPORT_POSTURE')),('PLACE',detached,('RETREAT','STOW'))):
            context=physical['submission']['context'];transaction=physical['submission']['transaction']
            adoptions=[(i,r['details']) for i,r in enumerate(world_journal) if r.get('event')=='payload_suffix_adopted' and r['details']['transaction']==transaction]
            if len(adoptions)!=1:raise RuntimeError('TRAJECTORY_SUFFIX_ADOPTION_REQUIRED:'+operation)
            adoption_index,adoption=adoptions[0]
            if not (physical['application_index']<adoption_index<physical['confirmation_index'] and adoption['context']==context and adoption['start_index']==4 and len(adoption['stages'])==2 and type(adoption['transport_replanned']) is bool and (operation=='PICK' or not adoption['transport_replanned'])):raise RuntimeError('TRAJECTORY_SUFFIX_BINDING_MISMATCH:'+operation)
            proof=dict(operation=operation,context=context,transaction=transaction,adoption_index=adoption_index,adoption=adoption,stages=[])
            report['trajectory_handoff']['operations'].append(proof)
            previous_index=physical['confirmation_index'];previous_generation=-1
            for index,(name,stage) in enumerate(zip(names,adoption['stages']),4):
                stage_id=stage['stage_id'];digest=stage['returned_digest']
                if not (stage['stage_index']==index and stage_id.split(':')[0]==name and isinstance(digest,str) and len(digest)==64 and all(c in '0123456789abcdef' for c in digest) and digest==stage['adopted_digest'] and stage['point_count']>0 and stage['joint_names']):raise RuntimeError('TRAJECTORY_SUFFIX_CONTENT_MISMATCH:'+operation+':'+name)
                sends=[(i,r['details']) for i,r in enumerate(world_journal) if r.get('event')=='child_trajectory_submission' and r['details']['context']==context and r['details']['transaction']==transaction and r['details']['stage_id']==stage_id and r['details']['stage_index']==index and r['details']['controller']=='arm_left_controller']
                if len(sends)!=1:raise RuntimeError('TRAJECTORY_ACTIVE_SUBMISSION_REQUIRED:'+operation+':'+name)
                send_index,sent=sends[0]
                if not (previous_index<send_index and sent['stage_generation']>previous_generation and sent['sent_digest']==digest and sent['point_count']==stage['point_count'] and sent['joint_names']==stage['joint_names']):raise RuntimeError('TRAJECTORY_SENT_CONTENT_MISMATCH:'+operation+':'+name)
                responses=[(i,r['details']) for i,r in enumerate(world_journal) if r.get('event')=='child_response' and r['details'].get('context')==context and r['details'].get('transaction')==transaction and r['details'].get('stage_id')==stage_id and r['details']['controller']==sent['controller']]
                if len(responses)!=1:raise RuntimeError('TRAJECTORY_ACTIVE_RESPONSE_REQUIRED:'+operation+':'+name)
                response_index,response=responses[0];child_uuid=response['uuid'];child=(sent['controller'],child_uuid)
                if not (send_index<response_index and all(response.get(k)==v for k,v in sent.items()) and isinstance(child_uuid,str) and len(child_uuid)==32 and all(c in '0123456789abcdef' for c in child_uuid) and child not in accepted_children):raise RuntimeError('TRAJECTORY_RESPONSE_BINDING_MISMATCH:'+operation+':'+name)
                accepted_children.add(child)
                terminals=[(i,r['details']) for i,r in enumerate(world_journal) if r.get('event')=='child_terminal' and (r['details']['controller'],r['details']['uuid'])==child]
                confirmations=[(i,r['details']) for i,r in enumerate(world_journal) if r.get('event')=='stage_confirmed' and r['details']['context']==context and r['details']['stage_id']==stage_id and r['details']['index']==index]
                if len(terminals)!=1 or len(confirmations)!=1:raise RuntimeError('TRAJECTORY_COMPLETION_REQUIRED:'+operation+':'+name)
                terminal_index,terminal=terminals[0];confirmed_index,confirmed=confirmations[0]
                if not (response_index<terminal_index<confirmed_index and terminal['success'] is True and terminal['result_code']==4 and confirmed['children'].split(';').count(sent['controller']+':'+child_uuid)==1):raise RuntimeError('TRAJECTORY_COMPLETION_BINDING_MISMATCH:'+operation+':'+name)
                proof['stages'].append(dict(stage_id=stage_id,stage_index=index,stage_generation=sent['stage_generation'],digest=digest,point_count=sent['point_count'],joint_names=sent['joint_names'],controller=sent['controller'],uuid=child_uuid,send_index=send_index,response_index=response_index,terminal_index=terminal_index,confirmed_index=confirmed_index))
                previous_index=confirmed_index;previous_generation=sent['stage_generation']
            if operation=='PICK' and previous_index>=detached['submission_index']:raise RuntimeError('TRAJECTORY_PICK_PLACE_ORDER_MISMATCH')
        report['final_stop']=stop()
        if not ready() or source_identity(summary())!=source_identity(final) or not fresh('envelope') or not 0<=ros()-latest['envelope']['stamp']<.3 or latest['envelope']['allowed'] or any(active_nav.values()):raise RuntimeError('FINAL_EMPTY_STOP_DENIAL_LOST')
        report['final_denied_envelope']=copy.deepcopy(latest['envelope'])
        report['passed']=True
    except BaseException as error:
        report['error']=repr(error)
    finally:
        cleanup=True;readback=None;renew_enabled=False;cleanup_errors=[]
        # Cancel only this parent, then let C++ terminate its own children before
        # independently checking physical stop; never send Python child cancels.
        try:release();report['resource_disposition']='RELEASE_CONFIRMED' if parent_goal is not None and parent_goal.accepted else 'NO_ACCEPTED_PARENT'
        except BaseException as error:cleanup_errors.append('parent_release:'+repr(error));report['resource_disposition']='UNRESOLVED'
        try:report['cleanup_stop']=stop()
        except BaseException as error:cleanup_errors.append('actual_stop:'+repr(error))
        if binding is not None:
            try:
                with journal.open() as stream:
                    stream.seek(journal_start);rows=[json.loads(line) for line in stream if line.strip()]
                report['executor_journal']=[row for row in rows if row.get('lease_id')==binding[0]]
            except (OSError,ValueError) as error:cleanup_errors.append('journal_capture:'+repr(error))
        report['cleanup_complete']=not cleanup_errors;report['cleanup_errors']=cleanup_errors
        if cleanup_errors:report['passed']=False
        gc.callbacks.remove(gc_timing)
        geometry_records=[dict(wall=at,ros_s=source,message=message_dict(deserialize_message(raw,RobotGeometryState))) for at,source,raw in geometry_records]
        jtc_records=[dict(controller=controller,wall=at,ros_s=source,message=message_dict(deserialize_message(raw,JointTrajectoryControllerState))) for controller,at,source,raw in jtc_records]
        report.update(finished_wall=time.time(),finished_steady=time.monotonic(),goal_uuid=parent_uuid.hex,lease_epoch=binding,
                      admission_uncertain=admission_uncertain,feedback_records=feedback_records,typed_hold_records=typed_hold_records,
                      geometry_records=geometry_records,payload_records=payload_records,jtc_records=jtc_records,
                      navigation_execution_records=navigation_records,nav_status_records=nav_status_records,navigation_goal_uuids=sorted(navigation_uuids),
                      events=events,envelopes=envelopes,acks=acks,commands=commands,motion=motion,renew_events=renew_events,constraints=constraints,
                      gc_events=gc_events,slow_spins=slow_spins)
        (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:report[k] for k in ('passed','task_id','context_id','parent_terminal','error','cleanup_complete','cleanup_errors','resource_disposition') if k in report},indent=2),flush=True)
        node.destroy_node();rclpy.shutdown();lease.close()
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
