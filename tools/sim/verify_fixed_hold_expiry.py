#!/usr/bin/env python3
"""Simulation handshake, corridor and hold-expiry validation.

Run only in the owned canonical simulation, after the transport task terminates.
Acquires its normal resource lease, confirms an empty attachment scene and
fresh stationary geometry, then stops renewing its OWN hold token. The final
transaction always revokes navigation. By default sends no motion goals.
--execute-corridor additionally navigates the selected aligned fixture through
the normal task arbiter, after its complete-footprint plan passes. This is not
a braking-distance or physical contact/force acceptance test.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose,ComputePathThroughPoses
from astribot_s1_transport.corridor_route import CorridorRoute
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import GetCostmap
from rosidl_runtime_py.convert import message_to_ordereddict
from astribot_navigation_msgs.msg import ArmHoldStatus
from astribot_s1_transport.core import Ledger, ResourceLease, TaskFailure, validate_scenario
from astribot_s1_transport.ros_backend import RosBackend, seconds


class ValidationBackend(RosBackend):
    """Use physical simulation time only for the optional navigation deadline."""
    validation_navigation = False
    navigation_sim_timeout = None
    navigation_wall_watchdog = 300.

    def future(self, future, timeout=20., checked=True):
        if not (checked and self.validation_navigation and self.navigation_sim_timeout is not None
                and self.active is not None and future is self.active[1]):
            return super().future(future, timeout, checked)
        start_ros = previous_ros = self.get_clock().now().nanoseconds * 1e-9
        start_wall = time.monotonic()
        try:
            while not future.done():
                self.check()
                now_ros = self.get_clock().now().nanoseconds * 1e-9
                if now_ros < previous_ros:
                    raise TaskFailure('NAVIGATION_CLOCK_REVERSED')
                previous_ros = now_ros
                if now_ros - start_ros >= self.navigation_sim_timeout:
                    raise TaskFailure('NAVIGATION_SIM_TIMEOUT')
                if time.monotonic() - start_wall >= self.navigation_wall_watchdog:
                    raise TaskFailure('NAVIGATION_WALL_WATCHDOG')
                time.sleep(.02)
            return future.result()
        finally:
            self.ledger.emit(self.ledger.stage, validation_event='NAVIGATION_WAIT_FINISHED',time_domain='simulation',
                simulation_budget_s=self.navigation_sim_timeout,
                wall_watchdog_s=self.navigation_wall_watchdog,
                elapsed_sim_s=self.get_clock().now().nanoseconds * 1e-9 - start_ros,
                elapsed_wall_s=time.monotonic() - start_wall)


def remove_owned(node,names):
    """Reconcile an uncertain remove response before retrying our own entity."""
    errors=[]
    for name in names:
        try:
            for attempt in range(3):
                try:node.ign('remove','Entity','name: '+json.dumps(name)+' type: MODEL')
                except TaskFailure:pass
                scene=subprocess.run(['ign','service','-s','/world/'+node.c['world']+'/scene/info',
                    '--reqtype','ignition.msgs.Empty','--reptype','ignition.msgs.Scene',
                    '--timeout','2000','--req',''],capture_output=True,text=True,timeout=3.)
                if scene.returncode or not scene.stdout.strip():raise TaskFailure('FIXTURE_CLEANUP_QUERY_FAILED')
                if not re.search(r'name:\s*"'+re.escape(name)+'"',scene.stdout):break
                time.sleep(.1)
            else:raise TaskFailure('FIXTURE_STILL_EXISTS:'+name)
        except Exception as error:errors.append(str(error))
    if errors:raise TaskFailure(';'.join(errors))


def navigate_checked(node,destination,corridor=None):
    node.carry_joints=dict(zip(node.hold_reference.joints.name,node.hold_reference.joints.position))
    node.admitted=True;node.c['navigation_timeout_s']=90.
    node.ledger.emit('TRANSPORT',loaded=False,validation_target=destination)
    node.validation_motion_goals+=1
    node.validation_navigation=True
    try:
        if corridor is None:node.navigate_to(destination)
        else:node.navigate_corridor(corridor)
    finally:node.validation_navigation=False
    node.ledger.emit('STATIONARY_CORRIDOR_PLAN',navigation_succeeded=True)
    node.admitted=False


def path_uses_fixture(path,width,through=False):
    """Require the selected entry/exit and inspect every segment in the wall strip.

    Goal arrival alone cannot prove traversal: an inflation-cost planner may
    legitimately prefer a detour around the outside of both walls.
    """
    points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
    if len(points)<2 or not all(math.isfinite(v) for p in points for v in p):return False
    if points[0][0]<=.6 or points[-1][0]>(-.6 if through else .05):return False
    crossed=False
    for (x0,y0),(x1,y1) in zip(points,points[1:]):
        if abs(x1-x0)<1e-12:
            interval=(0.,1.) if -.6<=x0<=.6 else None
        else:
            a,b=sorted(((-.6-x0)/(x1-x0),(.6-x0)/(x1-x0)))
            interval=(max(0.,a),min(1.,b))
        if interval is None or interval[0]>interval[1]:continue
        crossed=True
        if any(abs(y0+t*(y1-y0))>=width/2 for t in interval):return False
    return crossed


def corridor_plans(node,execute=False,drive_width=1.1,exit_through=False):
    """Observe owned walls and validate plans; optionally execute the selected case."""
    base=node.transform(node.c['map_frame'],node.c['base_frame'])
    if math.hypot(base[0,3]-(1.4 if execute else 1.1),base[1,3])>.03:
        raise TaskFailure('CORRIDOR_FIXTURE_REQUIRES_COMPLETED_WAREHOUSE_TASK')
    query=node.create_client(GetCostmap,'/global_costmap/get_costmap')
    planner=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
    through_planner=ActionClient(node,ComputePathThroughPoses,'/compute_path_through_poses')
    if not planner.wait_for_server(timeout_sec=5.):raise TaskFailure('PLANNER_UNAVAILABLE')
    def costs(width):
        grid=node.call(query,GetCostmap.Request()).map;m=grid.metadata
        if grid.header.frame_id!=node.c['map_frame'] or abs(m.origin.orientation.z)>1e-6:
            raise TaskFailure('FIXTURE_COSTMAP_FRAME_UNSUPPORTED')
        values=[]
        # Sample the wall away from the existing pick station at (0.1, 0.7).
        for x,y in ((-.3,width/2+.025),(-.3,-width/2-.025),(0.,0.)):
            ix=math.floor((x-m.origin.position.x)/m.resolution)
            iy=math.floor((y-m.origin.position.y)/m.resolution)
            if not 0<=ix<m.size_x or not 0<=iy<m.size_y:raise TaskFailure('FIXTURE_OUTSIDE_MAP')
            if y:
                # Rays mark the visible wall face, not every interior voxel.
                # Check a one-cell neighborhood around the known entity.
                values.append(max(grid.data[j*m.size_x+i] for j in range(max(0,iy-1),min(m.size_y,iy+2))
                    for i in range(max(0,ix-1),min(m.size_x,ix+2))))
            else:values.append(grid.data[iy*m.size_x+ix])
        return values
    rows=[]
    cases=[(.85,math.pi),(1.1,0.),(1.1,math.pi)]
    if drive_width!=1.1:cases.append((drive_width,math.pi))
    for width,goal_yaw in cases:
        owned=[]
        try:
            if any(v>=254 for v in costs(width)[:2]):
                raise TaskFailure('FIXTURE_OBSERVATION_REGION_ALREADY_OCCUPIED')
            for side in (-1,1):
                name='nonhome_corridor_'+str(os.getpid())+'_'+str(round(width*100))+'_'+str(side+1)
                node.spawn(name,[0.,side*(width/2+.05),.9],[1.2,.1,1.8],True)
                owned.append(name)
                node.ledger.emit('FIXTURE_CREATED',owned_fixture=name)
            deadline=time.monotonic()+10.
            while True:
                observed=costs(width)
                if all(v==254 for v in observed[:2]) and observed[2]<254:break
                if time.monotonic()>deadline:raise TaskFailure('FIXTURE_NOT_OBSERVED:'+str(observed))
                time.sleep(.1)
            e=node.navigation_envelope
            if not e.navigation_allowed:raise TaskFailure('ENVELOPE_LOST_BEFORE_PLANNING:'+e.reason)
            # Freeze the observed grid and installed polygon together. Candidate
            # planners consume this file offline, without publishing control or
            # acknowledgements into the live navigation graph.
            grid=node.call(query,GetCostmap.Request()).map
            base=node.transform(node.c['map_frame'],node.c['base_frame'])
            snapshot=dict(evidence='observed_simulation_costmap_planning_only',
                costmap=message_to_ordereddict(grid),envelope=message_to_ordereddict(e),
                start=[float(base[0,3]),float(base[1,3]),math.atan2(base[1,0],base[0,0])],
                goal=[-1.4 if execute and exit_through and width==drive_width and goal_yaw!=0. else 0.,0.,goal_yaw],
                fixture_width_m=width)
            (node.ledger.directory/f'planner_input_{round(width*100)}_{round(goal_yaw*100)}.json').write_text(
                json.dumps(snapshot,separators=(',',':'))+'\n')
            goal=ComputePathToPose.Goal();goal.goal.header.frame_id=node.c['map_frame']
            goal.goal.header.stamp=node.get_clock().now().to_msg()
            if execute and exit_through and width==drive_width and goal_yaw!=0.:
                goal.goal.pose.position.x=-1.4
            goal.goal.pose.orientation.z=math.sin(goal_yaw/2)
            goal.goal.pose.orientation.w=math.cos(goal_yaw/2)
            selected=execute and exit_through and width==drive_width and goal_yaw!=0.
            corridor=CorridorRoute('fixture_'+str(width),node.c['map_frame'],(.6,0.),(-.6,0.),width) if selected else None
            request,client=goal,planner
            if corridor is not None:
                if not through_planner.wait_for_server(timeout_sec=5.):raise TaskFailure('PLANNER_UNAVAILABLE')
                request=ComputePathThroughPoses.Goal()
                for x,y,yaw in corridor.via_poses():
                    p=PoseStamped();p.header=goal.goal.header
                    p.pose.position.x=x;p.pose.position.y=y
                    p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
                    request.goals.append(p)
                client=through_planner
            node.pending_goal=client.send_goal_async(request)
            handle=node.future(node.pending_goal,10.);node.pending_goal=None
            if not handle.accepted:raise TaskFailure('PLANNING_REQUEST_REJECTED')
            future=handle.get_result_async();node.active=(handle,future)
            result=node.future(future,20.);node.active=None
            snapshot['baseline_status']=result.status
            snapshot['baseline_path']=message_to_ordereddict(result.result.path)
            (node.ledger.directory/f'planner_input_{round(width*100)}_{round(goal_yaw*100)}.json').write_text(
                json.dumps(snapshot,separators=(',',':'))+'\n')
            should_plan=width>=1. and goal_yaw!=0.
            row=dict(width_m=width,goal_x_m=goal.goal.pose.position.x,goal_yaw_rad=goal_yaw,observed_wall_and_center_costs=observed,
                action_status=result.status,path_poses=len(result.result.path.poses),
                expected='PLAN' if should_plan else 'REJECT',envelope_epoch=e.epoch)
            rows.append(row)
            node.ledger.emit('STATIONARY_CORRIDOR_PLAN',**row)
            if (not should_plan and result.status!=6) or (should_plan and (result.status!=4 or not result.result.path.poses)):
                raise TaskFailure('UNEXPECTED_CORRIDOR_PLAN_RESULT:'+json.dumps(row))
            if should_plan and execute and width==drive_width:
                row['selected_corridor_route']=path_uses_fixture(result.result.path,width,exit_through)
                node.ledger.emit('CORRIDOR_ROUTE_CHECK',**row)
                if not row['selected_corridor_route']:
                    raise TaskFailure('PLANNED_ROUTE_BYPASSES_CORRIDOR')
                node.ledger.emit('CORRIDOR_EXECUTION',fixture_width_m=width,loaded=False)
                navigate_checked(node,[goal.goal.pose.position.x,0.,goal_yaw],corridor)
                row['navigation_succeeded']=True
        finally:
            node.cancel_active()
            remove_owned(node,owned)
        deadline=time.monotonic()+10.
        while any(v==254 for v in costs(width)[:2]):
            if time.monotonic()>deadline:raise TaskFailure('REMOVED_FIXTURE_OCCUPANCY_NOT_CLEARED')
            time.sleep(.1)
    planner.destroy();through_planner.destroy();node.destroy_client(query)
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--check-corridor-plans',action='store_true',
        help='Add/remove two temporary walls after a completed warehouse task; plan without moving')
    parser.add_argument('--execute-corridor',action='store_true',
        help='Execute the aligned case from prepared entry (1.4, 0, pi), using normal navigation safety gates')
    parser.add_argument('--drive-width',type=float,choices=[1.1,1.3],default=1.1)
    parser.add_argument('--exit-through',action='store_true',
        help='For the executed case, target (-1.4, 0, pi) beyond the exit; other negative cases stay inside')
    parser.add_argument('--prepare-entry',action='store_true',
        help='Navigate to (1.4, 0, pi) before creating any fixtures; existing map and collision gates remain active')
    parser.add_argument('--prepare-compact',action='store_true',
        help='While stopped and attachment-free, plan and execute the existing compact poses before installing V2')
    parser.add_argument('--navigation-sim-timeout',type=float,
        help='Optional physical-time navigation deadline for an isolated simulation; default keeps the original 90 s wall deadline')
    parser.add_argument('--navigation-wall-watchdog',type=float,default=300.,
        help='Independent wall watchdog when the simulation deadline is selected')
    args=parser.parse_args()
    if args.exit_through and not args.execute_corridor:parser.error('--exit-through requires --execute-corridor')
    if args.navigation_sim_timeout is not None:
        if not all(math.isfinite(v) and v>0 for v in (args.navigation_sim_timeout,args.navigation_wall_watchdog)):
            parser.error('Navigation deadlines must be finite and positive')
        if not os.environ.get('ASTRIBOT_SIM_INSTANCE') or os.environ.get('ROS_DOMAIN_ID') in (None,'25'):
            parser.error('Simulation deadline requires an explicitly isolated simulation')
    config=json.loads(Path(args.scenario).read_text())
    validate_scenario(config)
    config['navigation_geometry_mode']='fixed_v2'
    output=Path(args.output)
    if output.exists():raise SystemExit('Use a new evidence directory')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    with ResourceLease('/tmp/astribot_transport_domain_'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock') as lease:
        lease.file.seek(0);previous=lease.file.read()
        if previous and json.loads(previous).get('unconfirmed_executor',False):
            raise SystemExit('Previous executor termination unconfirmed')
        ledger=Ledger(output,config['object_id'])
        node=ValidationBackend(config,ledger)
        node.navigation_sim_timeout=args.navigation_sim_timeout
        node.navigation_wall_watchdog=args.navigation_wall_watchdog
        if args.navigation_sim_timeout is not None and not node.get_parameter('use_sim_time').value:
            raise RuntimeError('Simulation deadline requires use_sim_time')
        interrupted=[]
        def request_stop(signum,frame):
            interrupted.append(signum)
            node.cancel_requested=True
        signal.signal(signal.SIGINT,request_stop)
        signal.signal(signal.SIGTERM,request_stop)
        node.validation_motion_goals=0
        executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
        last_hold=[]
        node.create_subscription(ArmHoldStatus,'/navigation/arm_hold',
            lambda m:last_hold.append(copy.deepcopy(m)) if m.owner_id==node.hold_owner else None,10)
        spinner=threading.Thread(target=executor.spin,daemon=True);spinner.start()
        lease.checkpoint(dict(pid=os.getpid(),ledger=str(output/'state.json'),unconfirmed_executor=True))
        report=dict(evidence='stationary_runtime_protocol_only',motion_goals_sent=0)
        success=False
        try:
            node.wait(lambda:node.envelope is not None and node.stopped() and
                node.fresh(node.joints) and node.fresh(node.scan),20.)
            if node.scene().robot_state.attached_collision_objects:
                raise TaskFailure('EXISTING_ATTACHMENT_RECOVERY_REQUIRED')
            ledger.emit('STATIONARY_HOLD_EXPIRY')
            node.change_envelope(False)
            if args.prepare_compact:
                for group in ('arm_right','arm_left'):
                    ledger.emit('PREPARE_COMPACT',group=group)
                    node.skill('named',group,'transport_compact')
            node.change_envelope(True)
            if args.prepare_entry:navigate_checked(node,[1.4,0.,math.pi])
            if args.check_corridor_plans or args.execute_corridor:
                report['corridor_cases']=corridor_plans(node,args.execute_corridor,args.drive_width,args.exit_through)
            velocities=[];end=time.monotonic()+2.
            while time.monotonic()<end:
                e=node.navigation_envelope
                if not e.navigation_allowed or seconds(e.valid_until)<=node.get_clock().now().nanoseconds/1e9:
                    raise TaskFailure('PRE_INJECTION_ENVELOPE_LOST:'+e.reason)
                v=node.odom.twist.twist;velocities.append((math.hypot(v.linear.x,v.linear.y),abs(v.angular.z)))
                time.sleep(.02)
            injected=node.get_clock().now().nanoseconds/1e9
            token=node.hold_id;node.hold_id=''
            node.wait(lambda:not node.navigation_envelope.navigation_allowed,2.)
            revoked=node.get_clock().now().nanoseconds/1e9
            e=node.navigation_envelope
            own=[m for m in last_hold if m.hold_id==token]
            if not own or e.reason!='ARM_HOLD_EXPIRED':raise TaskFailure('UNEXPECTED_REVOCATION:'+e.reason)
            last=own[-1];deadline=seconds(last.header.stamp)+last.lease_s
            report.update(epoch=e.epoch,last_hold_ros_s=seconds(last.header.stamp),
                last_hold_deadline_ros_s=deadline,injected_ros_s=injected,revoked_ros_s=revoked,
                revocation_after_deadline_s=revoked-deadline,reason=e.reason,
                maximum_pre_injection_speed_m_s=max(v[0] for v in velocities),
                maximum_pre_injection_yaw_rate_rad_s=max(v[1] for v in velocities))
            if not 0<=revoked-deadline<=.2 or not node.stopped():
                raise TaskFailure('HOLD_EXPIRY_LATENCY_OR_STATIONARITY_FAILED')
            success=True
        except Exception as error:
            report['error']=str(error)
        finally:
            try:node.stop_and_hold()
            except Exception as error:report['cleanup_error']=str(error);success=False
            if interrupted:
                report['interrupted_signal']=interrupted[0];success=False
            report['passed']=success
            report['motion_goals_sent']=node.validation_motion_goals
            if args.execute_corridor:report['evidence']='canonical_warehouse_corridor_execution_and_hold_expiry'
            elif args.prepare_entry:report['evidence']='canonical_warehouse_entry_preparation_and_hold_expiry'
            elif args.check_corridor_plans:report['evidence']='canonical_warehouse_stationary_corridor_planning_and_hold_expiry'
            ledger.emit('PASS' if success else 'FAULT',**report)
            lease.checkpoint(dict(pid=os.getpid(),ledger=str(output/'state.json'),
                unconfirmed_executor=node.active is not None or node.pending_goal is not None))
            node.sync_stop.set();node.sync_thread.join(timeout=4.)
            executor.shutdown();spinner.join(timeout=3.)
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
        (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report),flush=True)
    raise SystemExit(0 if success else 1)


if __name__=='__main__':main()
