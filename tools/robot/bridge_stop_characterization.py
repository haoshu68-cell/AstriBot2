#!/usr/bin/env python3
"""Measure bridge zero-command stopping, returning along axes to one fixed SLAM origin.

This is a ROS client, not an SDK client. The isolated bridge must subscribe to
/bridge_stop_test/cmd_vel. --execute is required for movement. STOP, SIGINT,
SIGTERM and SIGHUP stop the experiment without automatic return motion.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import statistics
import time


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def local_pose(sample, origin):
    dx, dy = sample['x']-origin['x'], sample['y']-origin['y']
    c, s = math.cos(origin['yaw']), math.sin(origin['yaw'])
    return c*dx+s*dy, -s*dx+c*dy, wrap(sample['yaw']-origin['yaw'])


def make_case(spec):
    direction, value = spec.split(':'); speed = float(value)
    if direction not in ('x+', 'x-', 'y+', 'y-', 'yaw+', 'yaw-'):
        raise ValueError('invalid direction: '+direction)
    angular = direction.startswith('yaw')
    if not math.isfinite(speed) or not 0 < speed <= (.6 if angular else .35):
        raise ValueError('speed outside the measured navigation range')
    ramp = speed/(.75 if angular else .5)
    # Large rotation has a shorter plateau to keep every trial inside 90 degrees.
    plateau = min(2., max(1., 1.45/speed-.5*ramp)) if angular else 1.0
    planned_travel = speed*(.5*ramp+plateau)
    return dict(direction=direction, axis=2 if angular else (0 if direction[0]=='x' else 1),
                sign=1 if direction[-1]=='+' else -1, speed=speed,
                ramp_sec=ramp, plateau_sec=plateau, planned_integral=planned_travel)


class Experiment:
    def __init__(self, args):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from astribot_bridge_msgs.msg import BridgeStatus
        self.ros=rclpy;self.Twist=Twist;self.Status=BridgeStatus;self.args=args
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node=rclpy.create_node('bridge_stop_characterization')
        self.pub=self.node.create_publisher(Twist,'/bridge_stop_test/cmd_vel',10)
        self.data={'odom':[],'sdk':[],'cmd':[],'status':[]};self.phase='discovery';self.scan=None
        self.stop=False;self.enabled=False;self.origin=None;self.output=Path(args.output)
        self.output.mkdir(parents=True,exist_ok=False)
        self.boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
            signal.signal(sig,lambda *_: setattr(self,'stop',True))
        for key,topic in [('odom','/odom'),('sdk','/astribot/chassis/odom_from_sdk')]:
            self.node.create_subscription(Odometry,topic,self.odom_callback(key),qos_profile_sensor_data)
        self.node.create_subscription(BridgeStatus,'/astribot/bridge/status',self.status_callback,qos_profile_sensor_data)
        self.node.create_subscription(LaserScan,'/scan',lambda m:setattr(self,'scan',m),qos_profile_sensor_data)

    def odom_callback(self,key):
        def receive(m):
            stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            if self.data[key] and stamp<=self.data[key][-1]['stamp']:return
            q=m.pose.pose.orientation
            values=[m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
            if not all(math.isfinite(v) for v in values):return
            self.data[key].append(dict(t=time.monotonic(),wall=time.time(),stamp=stamp,
                                      x=values[0],y=values[1],yaw=values[2],phase=self.phase))
        return receive

    def status_callback(self,m):
        if m.node_name=='chassis_cmd_bridge':self.data['status'].append(dict(t=time.monotonic(),state=m.state,detail=m.detail))

    def publish(self,v=(0.,0.,0.)):
        m=self.Twist();m.linear.x=float(v[0]);m.linear.y=float(v[1]);m.angular.z=float(v[2])
        event=dict(t=time.monotonic(),wall=time.time(),v=list(v),phase=self.phase)
        self.pub.publish(m);self.data['cmd'].append(event);return event

    def spin(self,seconds,zero=False):
        end=time.monotonic()+seconds;next_zero=0.
        while time.monotonic()<end:
            if zero and time.monotonic()>=next_zero:self.publish();next_zero=time.monotonic()+.05
            self.ros.spin_once(self.node,timeout_sec=.02)

    def service(self,kind,path,value=None):
        from std_srvs.srv import SetBool,Trigger
        typ=SetBool if kind=='bool' else Trigger;client=self.node.create_client(typ,path)
        try:
            if not client.wait_for_service(timeout_sec=5):raise RuntimeError('service unavailable: '+path)
            req=typ.Request()
            if value is not None:req.data=value
            f=client.call_async(req);self.ros.spin_until_future_complete(self.node,f,timeout_sec=5)
            if not f.done() or f.exception() or not f.result().success:raise RuntimeError('service failed: '+path)
            return f.result().message
        finally:self.node.destroy_client(client)

    def fresh(self,sdk=True):
        for key in (('odom','sdk') if sdk else ('odom',)):
            if not self.data[key] or not 0<=time.time()-self.data[key][-1]['stamp']<=.35:
                raise RuntimeError('feedback stale: '+key)
        if not self.args.without_scan:
            if self.scan is None:raise RuntimeError('scan unavailable')
            stamp=self.scan.header.stamp.sec+self.scan.header.stamp.nanosec*1e-9
            if not 0<=time.time()-stamp<=.4:raise RuntimeError('scan stale')
            if min((v for v in self.scan.ranges if math.isfinite(v)),default=0)<.5:raise RuntimeError('scan near obstacle')

    def pose(self):
        rows=self.data['odom'][-3:]
        return dict(x=statistics.median(r['x'] for r in rows),y=statistics.median(r['y'] for r in rows),
                    yaw=rows[-1]['yaw']+statistics.median(wrap(r['yaw']-rows[-1]['yaw']) for r in rows),stamp=rows[-1]['stamp'])

    def motion(self,key='odom',span=.4):
        rows=self.data[key];end=rows[-1];start=next((r for r in rows if r['stamp']>=end['stamp']-span),rows[0])
        dt=end['stamp']-start['stamp']
        if dt<=0:return (0.,0.)
        return math.hypot(end['x']-start['x'],end['y']-start['y'])/dt,abs(wrap(end['yaw']-start['yaw']))/dt

    def check(self):
        if self.stop or (self.output/'STOP').exists() or (self.output.parent/'STOP').exists():raise RuntimeError('operator stop')
        self.fresh()
        p=local_pose(self.pose(),self.origin)
        if math.hypot(*p[:2])>.70:raise RuntimeError('origin travel envelope exceeded 0.70 m')
        bad=(self.Status.SDK_NOT_ALIVE,self.Status.SDK_CALL_FAILED,self.Status.LEASH_TRIPPED,self.Status.SLAM_LOST_STOPPED,self.Status.SCAN_LOST_STOPPED,self.Status.SLAM_RELOCALIZED)
        if any(r['state'] in bad for r in self.data['status'] if r['t']>=self.active_since):raise RuntimeError('bridge fault')
        return p

    def prepare(self):
        self.spin(8);self.fresh(sdk=False)
        pubs=self.node.get_publishers_info_by_topic('/bridge_stop_test/cmd_vel')
        if len(pubs)!=1 or pubs[0].node_name!=self.node.get_name():raise RuntimeError('multiple test command owners')
        if len(self.node.get_publishers_info_by_topic('/odom'))!=1:raise RuntimeError('ambiguous localization odometry')
        path=Path(self.args.origin)
        if path.exists():
            obj=json.loads(path.read_text())
            if obj['boot_id']!=self.boot:raise RuntimeError('origin belongs to another boot')
            self.origin=obj['pose']
        else:
            self.origin=self.pose();path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(json.dumps(dict(boot_id=self.boot,wall=time.time(),pose=self.origin,clearance_m=1.,source='SLAM /odom'),indent=2))
        p=local_pose(self.pose(),self.origin)
        if math.hypot(*p[:2])>(.7 if self.args.return_only else .03) or abs(p[2])>(1.65 if self.args.return_only else math.radians(1.5)):
            raise RuntimeError('not at the fixed origin; refusing to redefine it')
        print(json.dumps({'event':'origin','pose':self.origin,'offset':p}),flush=True)

    def settle(self,seconds=3.):
        self.phase='coast';self.publish();self.spin(seconds,zero=True);self.check()
        a=self.motion('sdk');b=self.motion('odom')
        if a[0]>.005 or a[1]>.01 or b[0]>.015 or b[1]>.02:raise RuntimeError('not stationary after zero hold')

    def return_axis(self,axis):
        if self.args.return_mode=='staged':
            return self.return_axis_staged(axis)
        self.phase='return_'+str(axis);deadline=time.monotonic()+65;held=False;settled_at=None
        tolerance=.008 if axis<2 else math.radians(.6)
        resume=.012 if axis<2 else math.radians(.8)
        while time.monotonic()<deadline:
            p=self.check();error=-p[axis]
            held=abs(error)<(resume if held else tolerance)
            if held:
                self.publish()
                if settled_at is None:settled_at=time.monotonic()
                if time.monotonic()-settled_at>.8:
                    v,w=self.motion('sdk')
                    if v<.005 and w<.01:return
            else:
                settled_at=None
                vel=math.copysign(min(.07 if axis<2 else .12,max(.012 if axis<2 else .035,.65*abs(error))),error)
                command=[0.,0.,0.]
                if axis==2:command[2]=vel
                else:
                    # Fixed origin-axis direction expressed in the current body frame.
                    a=p[2];command[axis]=vel*math.cos(a);command[1-axis]=(-1 if axis==0 else 1)*vel*math.sin(a)
                self.publish(command)
            self.spin(.05)
        raise RuntimeError('return to origin timed out')

    def return_axis_staged(self,axis):
        """Pure body-axis recovery in short, stationary-separated segments."""
        deadline=time.monotonic()+65
        tolerance=.008 if axis<2 else math.radians(.6)
        start=self.check()
        while time.monotonic()<deadline:
            p=self.check();error=-p[axis]
            if abs(error)<tolerance:return
            velocity=math.copysign(min(.07 if axis<2 else .12,
                                      max(.012 if axis<2 else .035,.65*abs(error))),error)
            duration=min(1.2 if axis<2 else 2.,
                         max(.25,(abs(error)-.5*tolerance)/(.8*abs(velocity))))
            self.phase='return_'+str(axis)+'_pulse';until=time.monotonic()+duration
            while time.monotonic()<until:
                q=self.check()
                if axis<2 and (abs(q[1-axis]-start[1-axis])>.04 or abs(q[2])>math.radians(3)):
                    raise RuntimeError('single-axis return deviated from its corridor')
                if abs(q[axis])<tolerance or q[axis]*error>0:break
                v=[0.,0.,0.];v[axis]=velocity;self.publish(v);self.spin(.05)
            self.phase='return_'+str(axis)+'_stop';self.publish();self.spin(.8,zero=True)
            stop_deadline=time.monotonic()+2.
            while True:
                self.check();v,w=self.motion('sdk');sv,sw=self.motion('odom')
                if v<.005 and w<.01 and sv<.015 and sw<.02:break
                if time.monotonic()>stop_deadline:raise RuntimeError('staged return did not stop')
                self.spin(.1,zero=True)
        raise RuntimeError('staged return to origin timed out: axis '+str(axis))

    def run(self,case,index):
        directory=self.output/f"{index:03d}_{case['direction'].replace('+','pos').replace('-','neg')}_{case['speed']:.3f}"
        directory.mkdir();self.data={k:[] for k in self.data};self.phase='baseline';self.spin(1.)
        result=dict(case=case,origin=self.origin,success=False,return_mode=self.args.return_mode,
                    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        attempted=False
        try:
            self.fresh(sdk=False);self.active_since=time.monotonic();attempted=True
            self.service('bool','/chassis_cmd_bridge/enable',True);self.enabled=True
            self.spin(.7,zero=True);p=self.check()
            if math.hypot(*p[:2])>.03 or abs(p[2])>math.radians(1.5):raise RuntimeError('case did not start at fixed origin')
            result['start_pose']=self.pose();result['baseline_end']=time.monotonic()
            self.phase='outbound';started=time.monotonic();end=started+case['ramp_sec']+case['plateau_sec'];next_t=started
            while time.monotonic()<end:
                p=self.check();now=time.monotonic()
                if case['axis']<2:
                    if abs(p[1-case['axis']])>.08 or abs(p[2])>math.radians(4):raise RuntimeError('straight-line heading/lateral deviation')
                    if abs(p[case['axis']])>.57:raise RuntimeError('outbound travel budget exceeded')
                elif math.hypot(*p[:2])>.06 or abs(p[2])>1.65:raise RuntimeError('in-place rotation envelope exceeded')
                command=[0.,0.,0.];command[case['axis']]=case['sign']*case['speed']*min(1.,(now-started)/max(.01,case['ramp_sec']))
                self.publish(command);next_t+=.05;self.spin(max(0.,next_t-time.monotonic()))
            self.phase='coast';result['zero']=self.publish();self.settle(3.)
            result['coast_end']=time.monotonic();result['pose_after_stop']=self.pose()
            print(json.dumps({'event':'stopped','case':case,'offset':local_pose(self.pose(),self.origin)}),flush=True)
            # Restore heading and return on origin-aligned axes; no diagonal excursion.
            self.return_axis(2)
            for axis in sorted((0,1),key=lambda i:-abs(local_pose(self.pose(),self.origin)[i])):self.return_axis(axis)
            self.return_axis(2);self.phase='return_settle';self.publish();self.spin(1.,zero=True)
            q=self.check();result['return_error']={'xy_m':math.hypot(*q[:2]),'yaw_deg':math.degrees(q[2])}
            if result['return_error']['xy_m']>.03 or abs(result['return_error']['yaw_deg'])>1.5:raise RuntimeError('return accuracy outside acceptance')
            result['success']=True
        except Exception as exc:result['failure']=str(exc)
        finally:
            if attempted:
                self.phase='final_stop';self.publish();self.spin(2.,zero=True)
                try:result['disable']=self.service('trigger','/chassis_cmd_bridge/disable')
                except Exception as exc:result['disable_error']=str(exc)
                self.enabled=False
            result['end_wall']=time.time()
            (directory/'samples.json').write_text(json.dumps(self.data))
            (directory/'result.json').write_text(json.dumps(result,indent=2))
            print(json.dumps({'event':'result','path':str(directory),'direction':case['direction'],'speed':case['speed'],'success':result['success'],'return_error':result.get('return_error'),'failure':result.get('failure'),'disable':result.get('disable')}),flush=True)
        if not result['success'] or 'disable_error' in result:raise RuntimeError('case failed; suite stopped')

    def recover(self):
        result={'origin':self.origin,'success':False};attempted=False
        try:
            self.active_since=time.monotonic();attempted=True
            self.service('bool','/chassis_cmd_bridge/enable',True);self.spin(.7,zero=True)
            self.return_axis(2)
            for _ in range(2):
                for axis in (0,1):self.return_axis(axis)
            self.return_axis(2);self.publish();self.spin(1.,zero=True)
            q=self.check();result['return_error']={'xy_m':math.hypot(*q[:2]),'yaw_deg':math.degrees(q[2])}
            result['success']=result['return_error']['xy_m']<=.03 and abs(result['return_error']['yaw_deg'])<=1.5
        except Exception as exc:result['failure']=str(exc)
        finally:
            if attempted:
                self.publish();self.spin(2.,zero=True)
                try:result['disable']=self.service('trigger','/chassis_cmd_bridge/disable')
                except Exception as exc:result['disable_error']=str(exc)
            (self.output/'recovery.json').write_text(json.dumps(result,indent=2))
            (self.output/'samples.json').write_text(json.dumps(self.data))
            print(json.dumps(result),flush=True)
        if not result['success'] or 'disable_error' in result:raise RuntimeError('return-only failed')

    def close(self):
        self.node.destroy_node();self.ros.try_shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cases',default='',help='Comma-separated x+:0.03,y-:0.1,yaw+:0.3')
    p.add_argument('--output',required=True);p.add_argument('--origin',required=True)
    p.add_argument('--return-only',action='store_true',help='Return within the already recorded origin envelope; never redefine the origin')
    p.add_argument('--return-mode',choices=('continuous','staged'),default='continuous',
                   help='staged uses pure body-axis segments separated by a stationary hold')
    p.add_argument('--execute',action='store_true');p.add_argument('--without-scan',action='store_true')
    args=p.parse_args();cases=[make_case(s) for s in args.cases.split(',') if s]
    if not cases and not args.return_only:p.error('provide --cases or --return-only')
    if not args.execute:print(json.dumps(cases,indent=2));return
    exp=Experiment(args)
    try:
        exp.prepare()
        if args.return_only:exp.recover()
        else:
            for i,c in enumerate(cases):exp.run(c,i)
    finally:exp.close()

if __name__=='__main__':main()
