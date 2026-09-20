#!/usr/bin/env python3
"""Validate full Nav2 goals around one fixed origin; dry-run unless --execute.

STOP in the result directory or Ctrl-C cancels the owned goal and disables the
hardware bridge. Measurements use fresh map TF; SDK odometry is diagnostic only.
"""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from run_waypoint_route import wrap, yaw, project, distribution


def route(distance,angles,directions):
    points=[]
    for a in angles:
        points.extend([(f'yaw_{a:+g}',[0.,0.,math.radians(a)]),(f'return_yaw_{a:+g}',[0.,0.,0.])])
    for d in directions:
        axis=0 if d[0]=='x' else 1
        p=[0.,0.,0.];p[axis]=(1 if d[1]=='+' else -1)*distance
        points.extend([(d,p),('return_'+d,[0.,0.,0.])])
    return points


class Check:
    def __init__(self,args):
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import qos_profile_sensor_data
        from tf2_ros import Buffer, TransformListener
        from nav_msgs.msg import Odometry,Path as RosPath
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String
        from action_msgs.msg import GoalStatusArray
        self.args=args;self.ros=rclpy;self.stop=False;self.enabled=False;self.handle=None;self.future=None
        self.active=False;self.other=False;self.phase='UNKNOWN';self.phase_at=0.;self.path=[];self.anchor=None
        self.origin=None;self.rows=[];self.last_stamp=None;self.odom=None;self.sdk=None;self.goal=None
        self.post_action=False;self.target_pose=None
        self.command={};self.phase_events=[];self.raw_events=[];self.last_pose_at=0.;self.latest_pose=None
        self.output=Path(args.output);self.output.mkdir(parents=True,exist_ok=False)
        self.samples=open(self.output/'samples.jsonl','w');self.commands=open(self.output/'commands.jsonl','w')
        source=Path(__file__).read_bytes();(self.output/'runner_snapshot.py').write_bytes(source)
        self.status={'state':'preflight','results':[],'arguments':vars(args),'started':time.time(),
                     'script_sha256':hashlib.sha256(source).hexdigest(),
                     'command_frames':{'/cmd_vel_nav_body_raw':'body','/cmd_vel_nav_body':'body'}}
        self.lock=open('/tmp/astribot_precision_'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock','a')
        fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.n=rclpy.create_node('navigation_precision_check',parameter_overrides=[Parameter('use_sim_time',value=args.sim)])
        self.buf=Buffer();self.listener=TransformListener(self.buf,self.n)
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda *_:setattr(self,'stop',True))
        self.n.create_subscription(Odometry,'/odom',lambda m:setattr(self,'odom',m),qos_profile_sensor_data)
        self.n.create_subscription(Odometry,'/astribot/chassis/odom_from_sdk',lambda m:setattr(self,'sdk',m),qos_profile_sensor_data)
        self.n.create_subscription(String,'/path_tracking/phase',self.phase_cb,qos_profile_sensor_data)
        self.n.create_subscription(RosPath,'/plan',self.path_cb,10)
        self.n.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',lambda m:setattr(self,'other',any(s.status in (1,2,3) for s in m.status_list)),10)
        for topic in ('/cmd_vel_nav_body_raw','/cmd_vel_nav_body','/cmd_vel'):
            self.n.create_subscription(Twist,topic,lambda m,t=topic:self.cmd_cb(t,m),10)
        self.save()

    def save(self):
        self.status['updated']=time.time()
        (self.output/'status.tmp').write_text(json.dumps(self.status,indent=2))
        (self.output/'status.tmp').replace(self.output/'status.json')

    def phase_cb(self,m):
        if m.data!=self.phase and self.active:self.phase_events.append({'wall':time.time(),'phase':m.data})
        self.phase=m.data;self.phase_at=time.monotonic()

    def path_cb(self,m):
        if self.active and m.header.frame_id=='map':
            self.path=[(p.pose.position.x,p.pose.position.y) for p in m.poses];self.anchor=None
            with open(self.output/'paths.jsonl','a') as f:f.write(json.dumps({'wall':time.time(),'goal':self.goal,'points':self.path})+'\n')

    def cmd_cb(self,topic,m):
        v=[m.linear.x,m.linear.y,m.angular.z];self.command[topic]=v
        if self.active:
            ns=self.n.get_clock().now().nanoseconds
            row={'wall':time.time(),'wall_ns':time.time_ns(),'ros':ns*1e-9,'ros_ns':ns,'topic':topic,'v':v,'goal':self.goal}
            self.commands.write(json.dumps(row)+'\n')

    def pose(self):
        tf=self.buf.lookup_transform('map','astribot_torso_base',self.ros.time.Time())
        self.pose_stamp_ns=tf.header.stamp.sec*10**9+tf.header.stamp.nanosec
        stamp=self.pose_stamp_ns*1e-9
        age=self.n.get_clock().now().nanoseconds*1e-9-stamp
        if not -.05<=age<=.35:raise RuntimeError('map TF stale')
        t=tf.transform.translation
        p=[t.x,t.y,yaw(tf.transform.rotation)]
        if not all(math.isfinite(v) for v in p):raise RuntimeError('invalid map TF')
        return p,stamp

    def spin(self,guard=True):
        self.ros.spin_once(self.n,timeout_sec=.01)
        if guard and (self.stop or (self.output/'STOP').exists()):raise RuntimeError('operator stop')
        try:p,stamp=self.pose()
        except Exception:
            if guard and self.active and time.monotonic()-self.last_pose_at>.4:raise RuntimeError('localization lost')
            return
        if self.pose_stamp_ns!=self.last_stamp:
            self.last_pose_at=time.monotonic();self.latest_pose=p;self.last_stamp=self.pose_stamp_ns
            if self.active:
                row={'wall':time.time(),'wall_ns':time.time_ns(),'stamp':stamp,'stamp_ns':self.pose_stamp_ns,'pose':p,'goal':self.goal,'phase':self.phase,'commands':dict(self.command),
                     'post_action':self.post_action,'goal_distance_m':math.hypot(p[0]-self.target_pose[0],p[1]-self.target_pose[1])}
                if self.sdk is not None:
                    m=self.sdk;row['sdk']={'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'pose':[m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)]}
                pr=project(self.path,p[0],p[1],self.anchor)
                if pr:
                    _,ct,tangent,arc,k=pr;self.anchor=arc
                    row.update(cross_track_m=ct,heading_error_deg=math.degrees(wrap(p[2]-tangent)),curvature=k)
                self.rows.append(row);self.samples.write(json.dumps(row)+'\n')
        if guard and self.active:
            if math.hypot(p[0]-self.origin[0],p[1]-self.origin[1])>self.args.envelope:raise RuntimeError('fixed origin envelope exceeded')
            if time.monotonic()-self.last_pose_at>.4:raise RuntimeError('localization timestamp stopped')

    def wait(self,f,seconds=10,guard=True):
        end=time.monotonic()+seconds
        while not f.done() and time.monotonic()<end:self.spin(guard)
        if not f.done():raise TimeoutError('action/service timeout')
        return f.result()

    def pause(self,seconds,guard=True):
        end=time.monotonic()+seconds
        while time.monotonic()<end:self.spin(guard)

    def service(self,path,kind='trigger',value=None):
        from std_srvs.srv import Trigger,SetBool
        cls=Trigger if kind=='trigger' else SetBool;c=self.n.create_client(cls,path)
        try:
            if not c.wait_for_service(timeout_sec=5):raise RuntimeError('service unavailable: '+path)
            req=cls.Request()
            if value is not None:req.data=value
            res=self.wait(c.call_async(req),5,guard=False)
            if not res.success:raise RuntimeError(path+': '+res.message)
            return res.message
        finally:self.n.destroy_client(c)

    def target(self,point):
        c,s=math.cos(self.origin[2]),math.sin(self.origin[2])
        return [self.origin[0]+c*point[0]-s*point[1],self.origin[1]+s*point[0]+c*point[1],wrap(self.origin[2]+point[2])]

    def msg(self,p):
        from geometry_msgs.msg import PoseStamped
        m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.n.get_clock().now().to_msg()
        m.pose.position.x,m.pose.position.y=p[:2];m.pose.orientation.z=math.sin(p[2]/2);m.pose.orientation.w=math.cos(p[2]/2)
        return m

    def prepare(self):
        from lifecycle_msgs.srv import GetState
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose,ComputePathToPose
        for name in ('controller_server','smoother_server','planner_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother'):
            c=self.n.create_client(GetState,'/'+name+'/get_state')
            if not c.wait_for_service(timeout_sec=15) or self.wait(c.call_async(GetState.Request())).current_state.label!='active':raise RuntimeError(name+' not active')
            self.n.destroy_client(c)
        c=self.n.create_client(GetParameters,'/controller_server/get_parameters')
        if not c.wait_for_service(timeout_sec=5):raise RuntimeError('controller parameters unavailable')
        keys=['use_sim_time','precise_goal_checker.xy_goal_tolerance','precise_goal_checker.yaw_goal_tolerance']
        vals=self.wait(c.call_async(GetParameters.Request(names=keys))).values
        self.status['parameters']=dict(zip(keys,[parameter_value_to_python(v) for v in vals]));self.n.destroy_client(c)
        self.xy=vals[1].double_value;self.angle=vals[2].double_value
        if vals[0].bool_value!=self.args.sim or not 0<self.xy<=.03 or not 0<self.angle<=math.radians(1.5)+1e-8:raise RuntimeError('unexpected precision profile')
        c=self.n.create_client(GetParameters,'/cmd_vel_body_to_world_node/get_parameters')
        try:
            if c.wait_for_service(timeout_sec=2):
                value=self.wait(c.call_async(GetParameters.Request(names=['enable_body_to_world']))).values[0]
                if value.type==1:
                    self.status['command_frames']['/cmd_vel']='map' if value.bool_value else 'body'
        except Exception as exc:
            self.status['command_frame_query_error']=str(exc)
        finally:self.n.destroy_client(c)
        self.pause(3);p,stamp=self.pose()
        if self.other:raise RuntimeError('another navigation goal active')
        self.origin=p
        origin={'pose':p,'stamp':stamp,'source':'map TF','boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
        if self.args.origin:
            old=json.loads(Path(self.args.origin).read_text())
            if old['boot_id']!=origin['boot_id']:raise RuntimeError('fixed origin belongs to another boot')
            self.origin=old['pose']
            if math.hypot(p[0]-self.origin[0],p[1]-self.origin[1])>self.args.envelope:raise RuntimeError('outside saved origin envelope')
            origin=old
        (self.output/'origin.json').write_text(json.dumps(origin,indent=2))
        self.points=[(label,self.target(point)) for label,point in route(self.args.distance,self.args.angles,self.args.directions)]
        self.status['origin']=origin;self.status['goals']=self.points;self.save()
        planner=ActionClient(self.n,ComputePathToPose,'/compute_path_to_pose')
        if not planner.wait_for_server(timeout_sec=20):raise RuntimeError('planner unavailable')
        preflight=[];start=p
        for label,point in self.points:
            req=ComputePathToPose.Goal();req.start=self.msg(start);req.goal=self.msg(point);req.use_start=True;req.planner_id='GridBased'
            h=self.wait(planner.send_goal_async(req))
            if not h.accepted:raise RuntimeError('planner rejected '+label)
            res=self.wait(h.get_result_async(),30)
            safe=res.status==4 and bool(res.result.path.poses)
            extent=max((math.hypot(q.pose.position.x-self.origin[0],q.pose.position.y-self.origin[1]) for q in res.result.path.poses),default=math.inf)
            preflight.append({'label':label,'action_status':res.status,'extent_m':extent,'poses':len(res.result.path.poses)})
            (self.output/'preflight.json').write_text(json.dumps(preflight,indent=2))
            if not safe or extent>self.args.envelope-.05:raise RuntimeError('route not plannable inside envelope: '+label)
            start=point
        planner.destroy();self.nav=ActionClient(self.n,NavigateToPose,'/navigate_to_pose')
        if not self.nav.wait_for_server(timeout_sec=20):raise RuntimeError('navigation unavailable')
        print(json.dumps({'event':'prepared','origin':origin,'goals':self.points}),flush=True)

    def run(self):
        from nav2_msgs.action import NavigateToPose
        self.prepare()
        if not self.args.execute:self.status['state']='preflight_passed';return
        if not self.args.sim:
            self.enabled=True
            self.service('/chassis_cmd_bridge/enable','bool',True)
            self.pause(1.)
        self.status['state']='running'
        for label,point in self.points:
            self.pause(.5)
            if self.other:raise RuntimeError('navigation is occupied')
            self.goal=label;self.rows=[];self.phase_events=[];self.path=[];self.anchor=None;self.active=True
            self.target_pose=point;self.post_action=False;self.phase='UNKNOWN'
            self.last_pose_at=time.monotonic();self.status['current_goal']=label;self.save()
            req=NavigateToPose.Goal();req.pose=self.msg(point)
            begin=time.monotonic();self.handle=self.wait(self.nav.send_goal_async(req))
            if not self.handle.accepted:raise RuntimeError('navigation rejected '+label)
            self.future=self.handle.get_result_async();res=self.wait(self.future,self.args.timeout)
            success_at=time.monotonic();success_pose,_=self.pose();self.post_action=True;self.pause(self.args.settle)
            p,_=self.pose();xy=math.hypot(p[0]-point[0],p[1]-point[1]);angle=abs(wrap(p[2]-point[2]))
            tail=[r for r in self.rows if r['stamp']>=self.rows[-1]['stamp']-.6]
            drift=max((math.hypot(r['pose'][0]-p[0],r['pose'][1]-p[1]) for r in tail),default=math.inf)
            yaw_drift=max((abs(wrap(r['pose'][2]-p[2])) for r in tail),default=math.inf)
            follow=[r for r in self.rows if not r['post_action'] and r['phase']=='FOLLOW' and 'cross_track_m' in r]
            travel=[r for r in follow if r['goal_distance_m']>.5]
            passed=res.status==4 and xy<=self.xy and angle<=self.angle and drift<=max(.001,self.xy*.2) and yaw_drift<=max(.001,self.angle*.2)
            result={'label':label,'goal':point,'actual':p,'action_status':res.status,'passed':passed,'xy_m':xy,'yaw_deg':math.degrees(angle),'settle_drift_m':drift,'settle_drift_deg':math.degrees(yaw_drift),'action_success_pose':success_pose,'duration_s':time.monotonic()-begin,'post_action_s':time.monotonic()-success_at,'phases':self.phase_events,'follow_samples':len(follow),'cross_track_m':distribution([r['cross_track_m'] for r in follow]),'heading_error_deg':distribution([r['heading_error_deg'] for r in follow])}
            result.update(travel_samples=len(travel),terminal_exclusion_radius_m=.5,
                          travel_cross_track_m=distribution([r['cross_track_m'] for r in travel]),
                          travel_heading_error_deg=distribution([r['heading_error_deg'] for r in travel]))
            self.status['results'].append(result);self.active=False;self.save();self.samples.flush();self.commands.flush()
            print(json.dumps(result),flush=True)
            if not passed:raise RuntimeError('action/precision/stability failed: '+label)
        self.status['state']='completed'

    def close(self):
        self.active=False
        if self.handle is not None and self.future is not None and not self.future.done():
            try:self.wait(self.handle.cancel_goal_async(),10,guard=False);self.wait(self.future,15,guard=False)
            except Exception as e:self.status['cancel_error']=str(e)
        if self.enabled:
            try:self.status['disable_result']=self.service('/chassis_cmd_bridge/disable')
            except Exception as e:self.status['disable_error']=str(e)
        self.status['ended']=time.time();self.save();self.samples.close();self.commands.close()
        try:
            from navigation_speed_report import analyze
            report=analyze(self.output)
            self.status['speed_report']={'path':'speed_report.json','quality':report['quality']}
        except Exception as exc:
            self.status['speed_report_error']=str(exc)
        self.save();self.n.destroy_node();self.ros.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True);p.add_argument('--sim',action='store_true');p.add_argument('--execute',action='store_true')
    p.add_argument('--origin',help='reuse this origin.json on a retry; never silently redefine origin')
    p.add_argument('--distance',type=float,default=.25);p.add_argument('--envelope',type=float,default=.65)
    p.add_argument('--angles',type=float,nargs='*',default=[60.,-60.]);p.add_argument('--directions',nargs='*',choices=['x+','x-','y+','y-'],default=['x+','x-','y+','y-'])
    p.add_argument('--timeout',type=float,default=150.);p.add_argument('--settle',type=float,default=2.)
    a=p.parse_args()
    if not (.05<=a.distance<=.5 and a.distance+.1<a.envelope<=.7 and a.timeout>0 and a.settle>=1 and all(math.isfinite(v) and abs(v)<=90 for v in a.angles)):p.error('invalid bounded test plan')
    c=Check(a)
    try:c.run()
    except Exception as e:c.status.update(state='failed',error=str(e));print(json.dumps({'error':str(e)}),flush=True);raise
    finally:c.close()

if __name__=='__main__':main()
