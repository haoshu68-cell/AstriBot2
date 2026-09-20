#!/usr/bin/env python3
"""Inject a bounded fault into an OWNED isolated simulation while navigating.

Uses the existing task resource lease, arm hold and navigation arbiter. After
injection, observes the independent final output before requesting cancellation.
No raw velocity command, synthetic ACK, or collision threshold override.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from astribot_s1_transport.core import Ledger,ResourceLease,TaskFailure,validate_scenario
from astribot_s1_transport.ros_backend import RosBackend,pose_at,seconds
from verify_fixed_hold_expiry import remove_owned


def owned_process(pid,token):
    fd=os.pidfd_open(pid)
    try:
        root=Path(f'/proc/{pid}')
        env=dict(v.split('=',1) for v in (root/'environ').read_bytes().decode().split('\0') if '=' in v)
        args=(root/'cmdline').read_bytes().decode().split('\0')
        for key in ('ROS_DOMAIN_ID','IGN_PARTITION','ASTRIBOT_SIM_INSTANCE'):
            if not os.environ.get(key) or env.get(key)!=os.environ[key]:
                raise ValueError('FAULT_PROCESS_ISOLATION_MISMATCH:'+key)
        if not any(Path(a).name==token for a in args[:2]):raise ValueError('FAULT_PROCESS_ROLE_MISMATCH')
        return fd
    except Exception:
        os.close(fd);raise


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario',required=True);p.add_argument('--output',required=True)
    p.add_argument('--fault',choices=['hold','geometry','policy','obstacle'],required=True)
    p.add_argument('--pid',type=int);p.add_argument('--distance',type=float,default=1.)
    p.add_argument('--injection-speed',type=float,default=.04)
    p.add_argument('--heading',type=float,help='Optional map-frame goal heading (radians)')
    a=p.parse_args()
    if not os.environ.get('ASTRIBOT_SIM_INSTANCE') or os.environ.get('ROS_DOMAIN_ID') in (None,'0','25'):
        p.error('an explicitly isolated simulation environment is required')
    if not .7<=a.distance<=1.5:p.error('distance must be 0.7-1.5 m')
    if not .03<=a.injection_speed<=.15:p.error('injection speed must be 0.03-0.15 m/s')
    if a.heading is not None and not math.isfinite(a.heading):p.error('finite heading required')
    if a.fault in ('geometry','policy') and not a.pid:p.error('process fault requires an exact owned PID')
    output=Path(a.output)
    if output.exists():p.error('use a new evidence directory')
    fd=owned_process(a.pid,'geometry_state' if a.fault=='geometry' else 'policy_controller') if a.pid else None
    config=json.loads(Path(a.scenario).read_text());validate_scenario(config)
    config['navigation_geometry_mode']='fixed_v2'
    rclpy.init()
    with ResourceLease('/tmp/astribot_transport_domain_'+os.environ['ROS_DOMAIN_ID']+'.lock') as lease:
        lease.file.seek(0);old=lease.file.read()
        if old and json.loads(old).get('unconfirmed_executor'):raise TaskFailure('PREVIOUS_EXECUTOR_UNCONFIRMED')
        node=RosBackend(config,Ledger(output,config['object_id']))
        ex=MultiThreadedExecutor(num_threads=3);ex.add_node(node)
        rows=[];commands=[]
        def odom(m):
            v=m.twist.twist;p=m.pose.pose.position
            rows.append(dict(source=seconds(m.header.stamp),ros=node.get_clock().now().nanoseconds/1e9,
                wall=time.monotonic(),x=p.x,y=p.y,speed=math.hypot(v.linear.x,v.linear.y),w=abs(v.angular.z)))
        def command(m):
            commands.append(dict(ros=node.get_clock().now().nanoseconds/1e9,wall=time.monotonic(),
                speed=math.hypot(m.linear.x,m.linear.y),w=abs(m.angular.z)))
        node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
        node.create_subscription(Twist,'/cmd_vel',command,20)
        spinner=threading.Thread(target=ex.spin,daemon=True);spinner.start()
        lease.checkpoint(dict(pid=os.getpid(),ledger=str(output/'state.json'),unconfirmed_executor=True))
        report=dict(fault=a.fault,evidence='empty_fixed_posture_simulation_independent_protection',passed=False)
        suspended=False;owned=[]
        try:
            node.wait(lambda:node.stopped() and node.fresh(node.joints) and node.fresh(node.scan) and node.envelope is not None,20.)
            if node.scene().robot_state.attached_collision_objects:raise TaskFailure('ATTACHMENT_PRESENT')
            node.change_envelope(False);node.change_envelope(True)
            base=node.transform(config['map_frame'],config['base_frame'])
            yaw=math.atan2(base[1,0],base[0,0]) if a.heading is None else a.heading
            target=[base[0,3]+a.distance*math.cos(yaw),base[1,3]+a.distance*math.sin(yaw),yaw]
            report['target']=list(map(float,target))
            goal=NavigateToPose.Goal();goal.pose.header.frame_id=config['map_frame']
            goal.pose.pose=pose_at([target[0],target[1],0.],[0.,0.,math.sin(yaw/2),math.cos(yaw/2)])
            if not node.nav.wait_for_server(timeout_sec=5.):raise TaskFailure('ARBITER_UNAVAILABLE')
            node.pending_goal=node.nav.send_goal_async(goal)
            handle=node.future(node.pending_goal,15.,checked=False);node.pending_goal=None
            if not handle.accepted:raise TaskFailure('NAVIGATION_REJECTED')
            result=handle.get_result_async();node.active=(handle,result)
            deadline=time.monotonic()+60.
            while not rows or rows[-1]['speed']<a.injection_speed:
                if result.done() or time.monotonic()>deadline:raise TaskFailure('NO_MEASURED_MOTION_BEFORE_INJECTION')
                node.check();time.sleep(.01)
            injected=node.get_clock().now().nanoseconds/1e9;injected_wall=time.monotonic();initial=rows[-1]
            envelope=node.navigation_envelope
            report.update(injected_ros_s=injected,injected_wall_monotonic=injected_wall,
                pre_injection_speed_mps=initial['speed'],epoch=envelope.epoch,geometry_hash=envelope.installed_geometry_hash)
            if a.fault=='obstacle':
                # Axis-aligned wall fixture only; reject arbitrary orientation.
                if abs(math.sin(yaw))>.05:raise TaskFailure('OBSTACLE_FIXTURE_REQUIRES_X_ALIGNED_TRAVEL')
                name=f'nonhome_dynamic_{os.getpid()}'
                xyz=[initial['x']+.75*math.cos(yaw),initial['y'],.7]
                node.spawn(name,xyz,[.18,3.,1.4],True);owned.append(name)
                report['obstacle_center_world']=xyz
                report['obstacle_spawn_confirmed_ros_s']=node.get_clock().now().nanoseconds/1e9
            elif a.fault=='hold':node.hold_id=''
            else:signal.pidfd_send_signal(fd,signal.SIGSTOP);suspended=True
            revoked=None;stopped_at=None;window=None
            deadline=time.monotonic()+12.
            while time.monotonic()<deadline:
                now=node.get_clock().now().nanoseconds/1e9
                if not node.navigation_envelope.navigation_allowed and revoked is None:
                    revoked=now;report['revocation_reason']=node.navigation_envelope.reason
                if rows and commands and time.monotonic()-rows[-1]['wall']<.3 and time.monotonic()-commands[-1]['wall']<.3:
                    v,c=rows[-1],commands[-1]
                    if v['speed']<.005 and v['w']<.01 and c['speed']<1e-8 and c['w']<1e-8:
                        if window is None:window=now
                        if now-window>=.6:stopped_at=window;break
                    else:window=None
                time.sleep(.01)
            after=[c for c in commands if c['wall']>=injected_wall]
            zero=next((c for c in after if c['speed']<1e-8 and c['w']<1e-8),None)
            rebound=zero is not None and any(c['speed']>1e-8 or c['w']>1e-8 for c in after if c['wall']>zero['wall'])
            traveled=[v for v in rows if v['wall']>=injected_wall]
            distance=max([math.hypot(v['x']-initial['x'],v['y']-initial['y']) for v in traveled],default=0.)
            report.update(revocation_latency_ros_s=None if revoked is None else revoked-injected,
                first_zero_latency_ros_s=None if zero is None else zero['ros']-injected,
                stopped_latency_ros_s=None if stopped_at is None else stopped_at-injected,
                maximum_displacement_after_fault_m=distance,command_rebound=rebound,
                cancellation_requested_after_measurement=True)
            # Lease/ACK bound plus timer allowance. Distance is measured, not
            # promoted to a calibrated braking-distance guarantee by this test.
            if a.fault=='obstacle':
                report['policy_at_stop']=node.navigation_policy
                if traveled:
                    wall_x=report['obstacle_center_world'][0]
                    # Conservative circumradius lower bound, independent of
                    # small heading drift during the stop.
                    radius=max(math.hypot(p.x,p.y) for p in envelope.reserved_footprint.points)
                    report['obstacle_clearance_lower_bound_m']=min(abs(wall_x-v['x'])-.09-radius for v in traveled)
            fault_confirmed=(revoked is not None if a.fault!='obstacle' else
                report.get('obstacle_clearance_lower_bound_m',-1)>=envelope.clearance_m)
            report['passed']=bool(fault_confirmed and zero and zero['ros']-injected<=.7 and stopped_at is not None and not rebound)
            if not report['passed']:report['error']='INDEPENDENT_STOP_ACCEPTANCE_FAILED'
        except Exception as error:
            report['error']=str(error);report['policy_at_failure']=node.navigation_policy
        finally:
            # Revoke our source hold before restoring a paused consumer.
            node.hold_id=''
            try:node.cancel_active()
            except Exception as error:report['cancel_error']=str(error);report['passed']=False
            if suspended:
                signal.pidfd_send_signal(fd,signal.SIGCONT);suspended=False
            if fd is not None:os.close(fd)
            try:node.change_envelope(False,checked=False)
            except Exception as error:report['cleanup_error']=str(error);report['passed']=False
            try:remove_owned(node,owned)
            except Exception as error:report['fixture_cleanup_error']=str(error);report['passed']=False
            lease.checkpoint(dict(pid=os.getpid(),ledger=str(output/'state.json'),
                unconfirmed_executor=node.active is not None or node.pending_goal is not None))
            node.sync_stop.set();node.sync_thread.join(timeout=4.)
            ex.shutdown();spinner.join(timeout=3.);node.destroy_node();rclpy.shutdown()
        (output/'motion.json').write_text(json.dumps(dict(odometry=rows,commands=commands)))
        (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
    raise SystemExit(0 if report['passed'] else 1)

if __name__=='__main__':main()
