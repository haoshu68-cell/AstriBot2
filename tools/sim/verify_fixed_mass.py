#!/usr/bin/env python3
"""Stationary simulation mass admission and clock-freeze evidence; no chassis goals.

Requires an owned cold start and an already reconciled real inventory. This
probe neither attaches objects nor writes PlanningScene/hold/consumer ACK topics.
"""
import copy
import math

CONSUMERS = {'global_costmap', 'local_costmap', 'planner', 'controller', 'policy'}


def ns(stamp):
    return stamp['sec'] * 1_000_000_000 + stamp['nanosec']


def ledger_summary(state, session, source, now_ns):
    o = state['observation']
    if not (state['confirmed'] and state['ledger_epoch'] and state['ledger_revision'] > 0
            and state['attachment_revision'] and o['environment'] == 'simulation'
            and o['session_id'] == session and o['source_id'] == source and o['source_epoch']
            and o['sequence'] > 0 and o['revision'] > 0 and o['full_inventory']
            and 0 < ns(o['observed_at']) <= now_ns < ns(o['valid_until'])
            and 0 < ns(state['published_at']) <= now_ns < ns(state['valid_until'])):
        raise ValueError('INCOMPLETE_STALE_OR_FOREIGN_LEDGER')
    objects = o['objects']
    if not ((o['status'] == 1 and not objects) or (o['status'] == 2 and objects)):
        raise ValueError('INVENTORY_STATUS_MISMATCH')
    ids = [v['object']['id'] for v in objects]
    weights = [v['weight'] for v in objects]
    if len(set(ids)) != len(ids) or any(not i for i in ids) or any(not math.isfinite(w) or w <= 0 for w in weights):
        raise ValueError('INVALID_OBJECT_IDS_OR_MASS')
    try:
        mass = math.fsum(weights)
    except OverflowError as error:
        raise ValueError('MASS_SUM_OVERFLOW') from error
    if not math.isfinite(mass):
        raise ValueError('MASS_SUM_OVERFLOW')
    return dict(ids=sorted(ids), mass_kg=mass, attachment_revision=state['attachment_revision'],
                ledger_epoch=state['ledger_epoch'], ledger_revision=state['ledger_revision'],
                source_epoch=o['source_epoch'], clock_epoch=o['clock_epoch'],
                source_revision=o['revision'], capture_ns=ns(o['observed_at']),
                valid_until_ns=min(ns(o['valid_until']), ns(state['valid_until'])))


def positive_match(envelope, acks, request, epoch, mass, now_wall, now_ros, hold_id=None):
    e = envelope
    if not (e and e['request'] == request and e['epoch'] == epoch and e['mass'] == mass
            and (hold_id is None or e.get('hold_id')==hold_id)
            and e['allowed'] and 0 <= now_wall-e['wall'] < .3 and e['stamp'] <= now_ros < e['until']):
        return False
    latest = {}
    for a in acks:
        if (a['session'], a['epoch'], a['hash']) == (e['session'], epoch, e['hash']):
            latest[a['consumer']] = a
    return CONSUMERS.issubset({key for key, a in latest.items() if a['applied']
                              and 0 <= now_wall-a['wall'] < .5 and 0 <= now_ros-a['stamp'] < .5})


def freeze_baseline(envelope, acks, request, epoch, mass, source, wall, ros, hold_id=None):
    if not positive_match(envelope,acks,request,epoch,mass,wall,ros,hold_id):
        raise ValueError('FREEZE_REQUIRES_CURRENT_POSITIVE_AUTHORITY')
    remaining=source['valid_until_ns']*1e-9-ros
    if not math.isfinite(remaining) or remaining<=0:
        raise ValueError('FREEZE_SOURCE_ALREADY_EXPIRED')
    return dict(envelope=copy.deepcopy(envelope),source=copy.deepcopy(source),wall=wall,ros=ros,
                remaining_source_lease_s=remaining)


def revoked_after_freeze(envelope, baseline, frozen_ros):
    before=baseline['envelope']
    return bool(before['allowed'] and not envelope['allowed']
                and all(envelope[k]==before[k] for k in ('session','epoch','request'))
                and envelope['wall']>=baseline['wall'] and envelope['stamp']>=baseline['ros']
                and abs(envelope['ros_s']-frozen_ros)<1e-8)


def full_scene_matches(objects, scene):
    """Conservative geometry readback; mass is proven by the physical ledger.

    Scene serialization need not preserve weight or source timestamps. Compare
    every collision shape/pose and its attachment frame; unsupported or missing
    fields fail closed instead of reducing the check to object IDs.
    """
    if scene.get('is_diff') is not False or scene.get('robot_state',{}).get('is_diff') is not False:return False
    actual=scene['robot_state'].get('attached_collision_objects')
    if not isinstance(actual,list) or len(actual)!=len(objects):return False
    keys=('id','pose','primitives','primitive_poses','meshes','mesh_poses','planes','plane_poses','subframe_names','subframe_poses')
    def geometry(v):
        o=v['object']
        # Humble message_to_ordereddict serializes the ROS char as one character.
        if not v['link_name'] or not o['id'] or not o['header']['frame_id'] or o['operation'] not in (0,'\x00'):raise ValueError('INVALID_SCENE_OBJECT')
        return dict(link=v['link_name'],frame=o['header']['frame_id'],**{k:o[k] for k in keys})
    def same(a,b):
        if isinstance(a,dict):return isinstance(b,dict) and a.keys()==b.keys() and all(same(a[k],b[k]) for k in a)
        if isinstance(a,list):return isinstance(b,list) and len(a)==len(b) and all(same(x,y) for x,y in zip(a,b))
        if isinstance(a,(float,int)) and not isinstance(a,bool):
            return isinstance(b,(float,int)) and not isinstance(b,bool) and math.isfinite(a) and math.isfinite(b) and math.isclose(a,b,rel_tol=0.,abs_tol=1e-8)
        return type(a)==type(b) and a==b
    try:
        expected={v['object']['id']:geometry(v) for v in objects};readback={v['object']['id']:geometry(v) for v in actual}
        weights={v['object']['id']:v['weight'] for v in objects}
        mass_ok=all(math.isfinite(v['weight']) and v['weight']>=0 and
                    (v['weight']==0 or v['weight']==weights.get(v['object']['id'])) for v in actual)
        return mass_ok and len(expected)==len(objects) and len(readback)==len(actual) and same(expected,readback)
    except (KeyError,TypeError,ValueError):return False


def source_identity(summary):
    return tuple(tuple(summary[k]) if isinstance(summary[k],list) else summary[k] for k in (
        'ids','mass_kg','attachment_revision','ledger_epoch','ledger_revision','source_epoch','clock_epoch','source_revision',
        'model_revision','geometry_source','geometry_clock_epoch','geometry_frame'))


def cleanup_owned_resources(paused, resume, stop, release, reconcile):
    result=dict(cleanup_complete=False,cleanup_errors=[],resource_disposition='UNRESOLVED')
    if paused:
        try:resume()
        except BaseException as error:result['cleanup_errors'].append('resume_world:'+repr(error))
    try:result['cleanup_stop']=stop()
    except BaseException as error:result['cleanup_errors'].append('stop:'+repr(error))
    if result.get('cleanup_stop',{}).get('passed'):
        try:release();result['resource_disposition']='RELEASE_CONFIRMED'
        except BaseException as error:result['cleanup_errors'].append('release:'+repr(error))
    else:
        try:reconcile()
        except BaseException as error:result['cleanup_errors'].append('reconcile_admission:'+repr(error))
        result['resource_disposition']='STOP_UNPROVEN_HOLD_NOT_CANCELLED_OWNING_SIMULATION_TEARDOWN_REQUIRED'
        if not result['cleanup_errors']:result['cleanup_errors'].append('STOP_UNPROVEN')
    result['cleanup_complete']=not result['cleanup_errors']
    return result


def freeze_summary(start_wall, revoked_wall, remaining_lease, frozen_ros, revoked_ros, reason, renew_attempts):
    return dict(revoked_with_clock_frozen=abs(revoked_ros-frozen_ros) < 1e-8 and revoked_wall >= start_wall,
                revocation_delay_s=revoked_wall-start_wall, remaining_source_lease_s=remaining_lease,
                watchdog_period_s=.05, nominal_deadline_wall=start_wall+remaining_lease+.05,
                scheduling_excess_s=max(0., revoked_wall-start_wall-remaining_lease-.05),
                renew_attempts_during_freeze=renew_attempts, reason=reason,
                isolated_payload_expiry_proven=False,
                attribution='Whole-chain revoke observed; geometry and hold can expire before the payload consumer.')


def main():
    import argparse
    import fcntl
    import hashlib
    import json
    import os
    from pathlib import Path
    import signal
    import subprocess
    import sys
    import time
    import uuid
    from prepare_empty_inventory import verify_owner, CaptureReceipts, ReadbackBarrier, read_model_parameters
    from verify_fixed_navigation import slam_pose_sample, measured_stop, OwnedHoldLease
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from sim_stack_supervisor import owned_descendants
    from capture_supervisor_owner import capture
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.action.graph import get_action_client_names_and_types_by_node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict as message_dict
    from action_msgs.msg import GoalStatusArray
    from unique_identifier_msgs.msg import UUID
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import String
    from rcl_interfaces.srv import GetParameters
    from moveit_msgs.srv import GetPlanningScene
    from astribot_payload_msgs.msg import AttachmentState
    from astribot_navigation_msgs.msg import ArmHoldStatus, RobotGeometryState, NavigationEnvelopeV2, EnvelopeApplyStatus, RobotEnvelope
    from astribot_navigation_msgs.srv import SetFixedEnvelope
    from astribot_s1_transport_native.action import HoldResources
    from astribot_s1_transport_native.srv import RenewHold

    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('owner', 'output', 'profile', 'cold-start-receipt'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--session', required=True)
    parser.add_argument('--source', required=True)
    parser.add_argument('--expected-binary-sha256', required=True)
    parser.add_argument('--mode', choices=('empty-freeze', 'loaded'), default='empty-freeze')
    args = parser.parse_args()
    owner = json.loads(args.owner.read_text()); verify_owner(owner, args.session, args.source)
    profile = json.loads(args.profile.read_text())
    if profile['environment'] != 'simulation': raise RuntimeError('SIMULATION_ONLY')
    cold = json.loads(args.cold_start_receipt.read_text())
    identities = cold['processes']; owned = owned_descendants([owner['pid']])
    if not (cold['owner'] == owner and cold['session'] == args.session and cold['no_preceding_goal_writers'] is True
            and 0 <= time.time()-cold['captured_wall'] < 120 and cold['probe_output'] == str(args.output.resolve())
            and {'bt_navigator', 'task_arbiter_cpp'}.issubset(identities)
            and all(v['pid'] in owned and capture(v['pid']) == v for v in identities.values())):
        raise RuntimeError('COLD_START_RECEIPT_INVALID')
    args.output.mkdir(parents=True, exist_ok=False)
    lease = open('/tmp/astribot_waypoint_'+os.environ['ROS_DOMAIN_ID']+'.lock', 'a')
    fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('fixed_mass_'+uuid.uuid4().hex[:8], parameter_overrides=[Parameter('use_sim_time', value=True)])
    def interrupted(sig, frame): raise KeyboardInterrupt('stationary mass validation interrupted')
    signal.signal(signal.SIGINT, interrupted); signal.signal(signal.SIGTERM, interrupted)
    latest = {}; receipts = {}; captures = CaptureReceipts(); motion = []; commands = []; events = []; envelopes = []; acks = []; scenarios = []
    active_nav = {}; paused = False; cleanup = False; freeze_active = False
    hold_goal = hold_result = pending_hold = None; hold_uuid = None; hold_binding = None; admission_uncertain = False
    readback = None; cleanup_health_errors = []; mass_request_active = False
    renew_enabled = False; renew_sequence = 0; renew_last = 0.; renew_pending = None; renew_events = []
    report = dict(passed=False, started_wall=time.time(), mode=args.mode, owner=owner,
                  evidence_layer='owned_fixed_v2_stationary_simulation', motion_goals_sent=0, scenarios=scenarios)
    action = ActionClient(node, HoldResources, '/transport/hold_resources')
    renew = node.create_client(RenewHold, '/transport/hold_executor/renew')
    fixed = node.create_client(SetFixedEnvelope, '/navigation/set_fixed_envelope')
    params = node.create_client(GetParameters, '/robot_envelope_coordinator/get_parameters')
    scene = node.create_client(GetPlanningScene, '/get_planning_scene')
    def ros(): return node.get_clock().now().nanoseconds*1e-9
    def stamp(s): return s.sec+s.nanosec*1e-9
    def receive(key, value): latest[key]=value; receipts[key]=time.monotonic()
    def event(kind, **data):
        row=dict(kind=kind, wall=time.monotonic(), ros_s=ros(), **data); events.append(row)
        print(json.dumps(row), flush=True)
    def ledger(msg):
        value=message_dict(msg); captures.observe('ledger', value, time.monotonic()); receive('ledger', value)
        observe_readback()
    def geometry(msg):
        receive('geometry',msg);observe_readback()
    def command(msg):
        value=dict(wall=time.monotonic(), vx=msg.linear.x, vy=msg.linear.y, wz=msg.angular.z)
        commands.append(value); receive('command', value)
    def slam_pose(msg):
        t=stamp(msg.header.stamp)
        if motion and t<=motion[-1]['t']:return
        value=slam_pose_sample(msg,time.monotonic(),latest.get('command'))
        motion.append(value);receive('slam_pose',value)

    def envelope(msg):
        value=dict(wall=time.monotonic(), ros_s=ros(), stamp=stamp(msg.header.stamp), until=stamp(msg.valid_until),
                   session=msg.coordinator_session_id, epoch=msg.epoch, request=msg.request_id, hash=msg.installed_geometry_hash,
                   mass=msg.limits.payload_mass_kg, allowed=msg.navigation_allowed, reason=msg.reason, hold_id=msg.hold_id)
        envelopes.append(value); receive('envelope', value)
    def ack(msg):
        acks.append(dict(wall=time.monotonic(), stamp=stamp(msg.header.stamp), session=msg.coordinator_session_id,
                         epoch=msg.envelope_epoch, hash=msg.installed_geometry_hash, consumer=msg.consumer_id,
                         applied=msg.applied, reason=msg.reason))
    node.create_subscription(AttachmentState, '/payload/attachment_state', ledger, 10)
    node.create_subscription(RobotGeometryState, '/navigation/geometry_state', geometry, 10)
    node.create_subscription(ArmHoldStatus, '/navigation/arm_hold', lambda m:receive('hold', m), 10)
    node.create_subscription(NavigationEnvelopeV2, '/navigation/envelope_v2', envelope, 30)
    node.create_subscription(EnvelopeApplyStatus, '/navigation/envelope_applied', ack, 50)
    node.create_subscription(String, '/transport/hold_executor/status', lambda m:receive('executor', json.loads(m.data)), 10)
    node.create_subscription(Twist, '/cmd_vel', command, qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped, '/slam/pose', slam_pose, qos_profile_sensor_data)
    def nav_status(key, msg): active_nav[key]={bytes(s.goal_info.goal_id.uuid).hex() for s in msg.status_list if s.status in (1,2,3)}
    for name in ('navigate_to_pose', 'navigate_through_poses'):
        node.create_subscription(GoalStatusArray, '/'+name+'/_action/status', lambda m,k=name:nav_status(k,m), QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def check_graph():
        known={('bt_navigator','/navigation_executor'):'bt_navigator', ('navigation_task_arbiter','/'):'task_arbiter_cpp',
               ('operator_backend','/'):'operator_backend', ('loop_route_executor','/'):'loop_route_executor', ('waypoint_follower','/'):'waypoint_follower'}
        for name, namespace in node.get_node_names_and_namespaces():
            for path,_ in get_action_client_names_and_types_by_node(node, name, namespace):
                if path.endswith(('/navigate_to_pose','/navigate_through_poses')) and known.get((name,namespace)) not in identities:
                    raise RuntimeError('FOREIGN_NAVIGATION_CLIENT:'+name)
        for value in identities.values():
            if capture(value['pid']) != value: raise RuntimeError('COLD_START_PROCESS_CHANGED')
    def fresh(key): return key in latest and 0 <= time.monotonic()-receipts[key] < .3
    def summary():
        value=ledger_summary(latest['ledger'], args.session, args.source, node.get_clock().now().nanoseconds)
        if not 0 <= time.monotonic()-captures.received.get('ledger', -math.inf) < .3: raise ValueError('STALE_LEDGER_CAPTURE')
        g=latest['geometry']
        if not (fresh('geometry') and g.complete and g.attachment_state_confirmed and g.attachment_revision == value['attachment_revision']
                and sorted(g.attachment_ids) == value['ids'] and 0 < stamp(g.header.stamp) <= ros() < stamp(g.valid_until)):
            raise ValueError('LEDGER_GEOMETRY_MISMATCH')
        value.update(model_revision=g.model_revision,geometry_source=g.source_id,
                     geometry_clock_epoch=g.clock_epoch,geometry_frame=g.header.frame_id)
        return value
    def ready():
        try: summary(); return True
        except (KeyError, ValueError): return False
    def observe_readback():
        if readback is None:return
        try:value=summary();valid=True;key=source_identity(value)
        except (KeyError,ValueError):valid=False;key=None
        readback.observe(valid,key,time.monotonic(),node.get_clock().now().nanoseconds)
    def own_hold_ready():
        return bool(hold_binding is not None and hold_goal is not None and hold_goal.accepted
                    and hold_result is not None and not hold_result.done()
                    and fresh('hold') and fresh('executor') and latest['hold'].hold_confirmed
                    and hold_binding.matches(latest['executor'])
                    and latest['hold'].hold_id==hold_binding.identity[2])
    def health():
        observe_readback()
        if latest.get('hold_ownership_error'):raise RuntimeError(latest['hold_ownership_error'])
        if mass_request_active and not own_hold_ready():raise RuntimeError('OWN_HOLD_LOST_DURING_MASS_ADMISSION')
        if any(active_nav.values()): raise RuntimeError('FOREIGN_NAVIGATION_ACTIVE')
        if latest.get('clock_error'): raise RuntimeError(latest['clock_error'])
        c=latest.get('command')
        if c and any(not math.isfinite(c[k]) or abs(c[k]) > 1e-6 for k in ('vx','vy','wz')): raise RuntimeError('NONZERO_CHASSIS_COMMAND')
    def spin(predicate, timeout, reason):
        nonlocal renew_pending, renew_last, renew_sequence, renew_enabled
        end=time.monotonic()+timeout
        while True:
            try:health()
            except RuntimeError as error:
                if not cleanup:raise
                if str(error) not in cleanup_health_errors:cleanup_health_errors.append(str(error))
            if predicate(): return
            if time.monotonic() > end: raise RuntimeError(reason+': '+str(latest.get('executor'))+' '+str(latest.get('envelope')))
            rclpy.spin_once(node, timeout_sec=.01)
            if renew_pending is not None and renew_pending.done():
                completed=renew_pending;renew_pending=None
                try:answer=completed.result()
                except Exception as error:
                    renew_events.append(dict(wall=time.monotonic(),kind='response_error',error=repr(error)))
                    if not cleanup:raise
                    renew_enabled=False;cleanup_health_errors.append('RENEW_RESPONSE_ERROR:'+repr(error));continue
                renew_events.append(dict(wall=time.monotonic(), kind='response', accepted=answer.accepted, reason=answer.reason))
                if renew_enabled and not answer.accepted:
                    renew_enabled=False
                    if not freeze_active and not cleanup:raise RuntimeError('RENEW_REJECTED:'+answer.reason)
            status=latest.get('executor', {})
            if renew_enabled and hold_binding is not None and hold_binding.matches(status) and renew_pending is None and time.monotonic()-renew_last > .2:
                renew_sequence+=1; renew_last=time.monotonic()
                renew_pending=renew.call_async(RenewHold.Request(lease_id=status['lease_id'], resource_epoch=status['epoch'], sequence=renew_sequence))
                renew_events.append(dict(wall=renew_last, kind='request', sequence=renew_sequence, lease=status['lease_id']))
    def duration(seconds):
        end=time.monotonic()+seconds; spin(lambda:time.monotonic()>=end, seconds+1, 'OBSERVE')
    def stop():
        def settled():
            if latest.get('clock_error'):raise RuntimeError(latest['clock_error'])
            return measured_stop(motion,after,time.monotonic())['passed']
        if latest.get('clock_error'):raise RuntimeError(latest['clock_error'])
        after=time.monotonic();spin(settled,15,'ACTUAL_STOP_TIMEOUT')
        return measured_stop(motion, after, time.monotonic())
    def hold_feedback(message):
        nonlocal renew_enabled
        if hold_binding is None:return
        value=message.feedback
        try:hold_binding.observe(bytes(message.goal_id.uuid).hex(),value.lease_id,value.resource_epoch,value.hold_id)
        except RuntimeError as error:
            renew_enabled=False;latest['hold_ownership_error']=str(error)
            if str(error) not in cleanup_health_errors:cleanup_health_errors.append(str(error))
    def acquire():
        nonlocal pending_hold, hold_goal, hold_result, hold_uuid, hold_binding, admission_uncertain, renew_enabled, renew_sequence, renew_pending, renew_last
        spin(lambda:ready() and latest.get('executor', {}).get('phase')=='0',15,'HOLD_ENTRY')
        report.setdefault('stops', []).append(stop()); check_graph()
        renew_sequence=0; renew_pending=None; renew_last=0.; renew_enabled=True
        hold_uuid=uuid.uuid4(); hold_binding=OwnedHoldLease(hold_uuid.hex);admission_uncertain=True
        pending_hold=action.send_goal_async(HoldResources.Goal(task_id='I0_2_mass_stationary',request_id='mass_hold_'+uuid.uuid4().hex), goal_uuid=UUID(uuid=list(hold_uuid.bytes)),feedback_callback=hold_feedback)
        spin(pending_hold.done,5,'HOLD_ADMISSION'); hold_goal=pending_hold.result(); pending_hold=None; admission_uncertain=False
        if not hold_goal.accepted: raise RuntimeError('HOLD_REJECTED')
        hold_binding.accepted=True
        hold_result=hold_goal.get_result_async()
        spin(lambda:hold_result.done() or (fresh('hold') and latest['hold'].hold_confirmed and hold_binding.matches(latest.get('executor',{})) and latest['hold'].hold_id==hold_binding.identity[2]),18,'HOLD_CONFIRM')
        if hold_result.done(): raise RuntimeError('HOLD_TERMINATED:'+str(hold_result.result()))
    def release():
        nonlocal renew_enabled, pending_hold, hold_goal, hold_result, admission_uncertain
        renew_enabled=False
        if pending_hold is not None:
            spin(pending_hold.done,5,'PENDING_HOLD_ADMISSION'); hold_goal=pending_hold.result(); pending_hold=None; admission_uncertain=False
        if hold_goal is not None: admission_uncertain=False
        if admission_uncertain: raise RuntimeError('HOLD_ADMISSION_UNKNOWN')
        if hold_goal is not None and hold_goal.accepted:
            if hold_result is None: hold_result=hold_goal.get_result_async()
            if not hold_result.done():
                future=hold_goal.cancel_goal_async(); spin(future.done,3,'CANCEL_HOLD')
            spin(hold_result.done,15,'RELEASE_HOLD')
            result=hold_result.result()
            if not result.result.resources_released: raise RuntimeError('RESOURCE_RELEASE_UNCONFIRMED')
            event('hold_released', reason=result.result.reason, status=result.status)
        hold_goal=hold_result=None
    def reconcile_without_release():
        nonlocal pending_hold, hold_goal, hold_result, admission_uncertain
        if pending_hold is not None:
            spin(pending_hold.done,5,'CLEANUP_HOLD_ADMISSION')
            hold_goal=pending_hold.result();pending_hold=None;admission_uncertain=False
        if admission_uncertain:raise RuntimeError('HOLD_ADMISSION_UNKNOWN')
        if hold_goal is not None and hold_goal.accepted:
            if hold_result is None:hold_result=hold_goal.get_result_async()
            if hold_result.done():report['unreleased_hold_terminal']=message_dict(hold_result.result().result)
    def request_mass(mass, expect):
        nonlocal mass_request_active
        if not own_hold_ready():raise RuntimeError('OWN_HOLD_REQUIRED_FOR_MASS_ADMISSION')
        mass_request_active=True
        try:return submit_mass(mass,expect)
        finally:mass_request_active=False
    def submit_mass(mass, expect):
        spin(ready,3,'REQUEST_SOURCE_UNAVAILABLE'); before=summary()
        if args.mode=='loaded' and source_identity(before)!=source_identity(initial):
            raise RuntimeError('PAYLOAD_CHANGED_SINCE_INDEPENDENT_SCENE_READBACK')
        limits=RobotEnvelope(frame_id=profile['base_frame'],posture_id='mass_measured_current_pose',lease_s=.3)
        for key in ('half_length_m','half_width_m','height_m','max_speed_m_s','max_angular_speed_rad_s','max_acceleration_m_s2','brake_deceleration_m_s2'):
            setattr(limits,key,float(profile[key]))
        limits.payload_mass_kg=mass
        if not own_hold_ready():raise RuntimeError('OWN_HOLD_REQUIRED_FOR_MASS_ADMISSION')
        owned_hold_id=hold_binding.identity[2]
        request=SetFixedEnvelope.Request(request_id='mass_fixed_'+uuid.uuid4().hex,hold_id=owned_hold_id,geometry_sequence=latest['geometry'].sequence,limits=limits)
        future=fixed.call_async(request); spin(future.done,3,'FIXED_REQUEST'); response=future.result()
        row=dict(request=request.request_id,epoch=response.epoch,mass_kg=mass,accepted=response.accepted,reason=response.reason,ledger=before)
        if response.accepted != expect: raise RuntimeError('UNEXPECTED_MASS_ADMISSION:'+str(row))
        if expect:
            spin(lambda:positive_match(latest.get('envelope'),acks,request.request_id,response.epoch,mass,time.monotonic(),ros(),owned_hold_id),15,'FIVE_MATCHED_GEOMETRY_CONSUMERS')
        elif not response.reason.startswith('PAYLOAD_MASS_UNDERREPORTED'):
            raise RuntimeError('WRONG_LOW_MASS_REJECTION:'+response.reason)
        after=summary()
        if source_identity(after)!=source_identity(before):
            raise RuntimeError('PAYLOAD_CHANGED_DURING_REQUEST')
        event('mass_admission', **row); scenarios.append(row); return row
    def pause(value):
        nonlocal paused
        verify_owner(owner,args.session,args.source)
        if value: paused=True
        process=subprocess.Popen(['ign','service','-s','/world/default/control','--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean','--timeout','3000','--req','pause: '+str(value).lower()],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            spin(lambda:process.poll() is not None,4,'WORLD_PAUSE_REPLY')
            stdout,stderr=process.communicate(timeout=.1)
            if process.returncode or 'data: true' not in stdout: raise RuntimeError('WORLD_PAUSE_UNCERTAIN:'+stderr)
        finally:
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=1)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=1)
        paused=value
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
        spin(lambda:action.server_is_ready() and renew.service_is_ready() and fixed.service_is_ready() and ready() and ('slam_pose' in latest) and fresh('command'),30,'ENTRY_UNAVAILABLE')
        check_graph(); report['initial_stop']=stop(); initial=summary(); report['initial_ledger']=initial
        if not scene.wait_for_service(timeout_sec=3): raise RuntimeError('SCENE_UNAVAILABLE')
        readback_deadline=time.monotonic()+5.
        while True:
            spin(ready,max(.01,readback_deadline-time.monotonic()),'SCENE_SOURCE_UNAVAILABLE')
            initial=summary();objects=copy.deepcopy(latest['ledger']['observation']['objects'])
            readback=ReadbackBarrier(source_identity(initial),time.monotonic(),node.get_clock().now().nanoseconds)
            req=GetPlanningScene.Request();req.components.components=4
            future=scene.call_async(req)
            end=time.monotonic()+.25
            spin(lambda:future.done() or time.monotonic()>=end,.3,'INDEPENDENT_SCENE_READBACK')
            observe_readback();valid=readback.valid;readback=None
            completed=future.done()
            report.setdefault('scene_attempts',[]).append(dict(source=initial,completed=completed,continuous_identity=valid))
            if completed and valid:
                full_scene=message_dict(future.result().scene)
                report['independent_full_attachment_scene']=full_scene
                if not full_scene_matches(objects,full_scene):raise RuntimeError('FULL_SCENE_GEOMETRY_OR_LINK_MISMATCH')
                break
            if not completed:scene.remove_pending_request(future)
            if time.monotonic()>=readback_deadline:raise RuntimeError('SCENE_READBACK_CONTINUITY_UNPROVEN')
        if args.mode=='loaded':
            if not initial['ids'] or initial['mass_kg']<=0: raise RuntimeError('LOADED_LEDGER_REQUIRED')
            previous=0
            for mass,expected in ((initial['mass_kg']*.5,False),(initial['mass_kg'],True),(initial['mass_kg']*1.25,True)):
                acquire(); row=request_mass(mass,expected)
                if expected and row['epoch']<=previous: raise RuntimeError('OLD_EPOCH_REUSED')
                if expected: previous=row['epoch']
                report['stops'].append(stop()); release()
        else:
            if initial['ids']: raise RuntimeError('EMPTY_LEDGER_REQUIRED')
            acquire(); first=request_mass(0.,True); report['stops'].append(stop())
            spin(lambda:own_hold_ready() and positive_match(latest.get('envelope'),acks,first['request'],first['epoch'],0.,time.monotonic(),ros(),hold_binding.identity[2]),3,'PRE_FREEZE_AUTHORITY_LOST')
            baseline=freeze_baseline(latest['envelope'],acks,first['request'],first['epoch'],0.,summary(),time.monotonic(),ros(),hold_binding.identity[2])
            report['freeze_baseline']=baseline
            freeze_active=True; begin=len(envelopes); pause_request_wall=baseline['wall']; pause(True)
            # Spin through transport replies while renewal continues, then verify a stable clock.
            duration(.15); frozen_ros=ros(); frozen_wall=time.monotonic(); duration(.15)
            if abs(ros()-frozen_ros)>1e-8: raise RuntimeError('WORLD_NOT_FROZEN')
            spin(lambda:any(revoked_after_freeze(e,baseline,frozen_ros) for e in envelopes[begin:]),3,'FROZEN_AUTHORITY_NOT_REVOKED')
            revoked=next(e for e in envelopes[begin:] if revoked_after_freeze(e,baseline,frozen_ros))
            remaining=baseline['remaining_source_lease_s']
            freeze=freeze_summary(pause_request_wall,revoked['wall'],remaining,frozen_ros,revoked['ros_s'],revoked['reason'],sum(e['kind']=='request' and e['wall']>=pause_request_wall for e in renew_events))
            freeze.update(frozen_verified_wall=frozen_wall,revocation=revoked,renew_enabled=renew_enabled)
            if not freeze['revoked_with_clock_frozen']: raise RuntimeError('REVOCATION_NOT_OBSERVED_AT_FROZEN_CLOCK')
            report['freeze']=freeze; event('frozen_revoke',**freeze)
            pause(False)
            spin(lambda:('slam_pose' in latest) and ros()>frozen_ros and ready(),20,'FRESH_SOURCE_NOT_RECOVERED')
            # A source fault may already have terminated the hold; release owns the final reconciliation.
            report['stops'].append(stop()); release(); freeze_active=False
            acquire(); recovered=request_mass(0.,True)
            if recovered['epoch']<=first['epoch']: raise RuntimeError('RECOVERY_WITHOUT_NEW_EPOCH')
            report['stops'].append(stop()); release()
        report['passed']=True
    except BaseException as error:
        report['error']=repr(error)
    finally:
        cleanup=True; readback=None;mass_request_active=False
        disposition=cleanup_owned_resources(paused,lambda:pause(False),stop,release,reconcile_without_release)
        report.update(disposition)
        if cleanup_health_errors:
            report['cleanup_health_errors']=cleanup_health_errors;report['cleanup_complete']=False
        if not report['cleanup_complete']:report['passed']=False
        report.update(finished_wall=time.time(),events=events,envelopes=envelopes,acks=acks,commands=commands,motion=motion,
                      renew_events=renew_events,pending_hold_uuid=hold_uuid.hex if admission_uncertain else None,
                      owned_hold_identity=hold_binding.identity if hold_binding else None,
                      hold_handle_unresolved=bool(hold_goal is not None and hold_goal.accepted))
        (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('events','envelopes','acks','commands','motion','binary','renew_events')},indent=2),flush=True)
        node.destroy_node(); rclpy.shutdown(); lease.close()
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
