#!/usr/bin/env python3
"""Repeat two fixed Nav2 goals; stop on failed precision or an operator STOP file."""
import argparse
import json
import math
from pathlib import Path
import time

import navigation_precision_check as base


class TwoGoalCheck(base.Check):
    def __init__(self, args):
        self.corridor_error = None
        self.vendor_last = {}
        self.vendor_stats = {}
        super().__init__(args)
        from astribot_msgs.msg import RobotJointState
        from rclpy.qos import qos_profile_sensor_data
        self.vendor_file = (self.output/'vendor.jsonl').open('w')
        for topic in ('/astribot_chassis/joint_space_states', '/astribot_chassis/joint_space_command_recv'):
            self.n.create_subscription(RobotJointState, topic,
                lambda m,t=topic:self.vendor_cb(t,m), qos_profile_sensor_data)
        self.status['corridor'] = {'x_min':-1.35, 'x_max':1.35, 'y_abs_max':.30}
        self.status['metrics'] = 'unique SLAM stamps; FOLLOW travel excludes post_action and goal distance <= 0.5m'

    def vendor_cb(self, topic, msg):
        if not self.active or len(msg.position) != 3:
            return
        now = time.time()
        p = list(msg.position)
        previous = self.vendor_last.get(topic)
        step = p[2]-previous['position'][2] if previous else 0.
        row = {'wall':now, 'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
               'topic':topic, 'position':p, 'velocity':list(msg.velocity), 'goal':self.goal,
               'raw_theta_step':step}
        self.vendor_last[topic] = row
        st = self.vendor_stats.setdefault(topic, {'frames':0,'max_raw_theta_step':0.,'last_saved':0.})
        st['frames'] += 1
        st['max_raw_theta_step'] = max(st['max_raw_theta_step'], abs(step))
        if abs(step)>math.pi:
            self.corridor_error = 'SDK continuous theta jumped by more than pi: '+topic
        if now-st['last_saved'] >= .04 or abs(step)>.05:
            self.vendor_file.write(json.dumps(row)+'\n')
            st['last_saved'] = now

    def local(self, p):
        x,y = p[0]-self.origin[0],p[1]-self.origin[1]
        c,s = math.cos(self.origin[2]),math.sin(self.origin[2])
        return c*x+s*y,-s*x+c*y

    def inside(self, p):
        x,y = self.local(p)
        return abs(x)<=1.35 and abs(y)<=.30

    def path_cb(self, msg):
        super().path_cb(msg)
        if self.active and self.origin and msg.header.frame_id=='map':
            if any(not self.inside([p.pose.position.x,p.pose.position.y]) for p in msg.poses):
                self.corridor_error = 'planned path outside the fixed test corridor'

    def spin(self, guard=True):
        super().spin(guard)
        if guard and self.active:
            if self.corridor_error:
                raise RuntimeError(self.corridor_error)
            if self.latest_pose and not self.inside(self.latest_pose):
                raise RuntimeError('measured pose outside the fixed test corridor')
        if guard and self.args.stop_file and Path(self.args.stop_file).exists():
            raise RuntimeError('operator stop')

    def prepare(self):
        origin = json.loads(Path(self.args.origin).read_text())
        if origin.get('coordinate_transfer_validated') is False:
            raise RuntimeError('fixed goal coordinates are unverified after SLAM restart')
        for row in origin.get('slam_processes', []):
            raw = Path(f"/proc/{row['pid']}/stat").read_text().rsplit(') ',1)[1].split()
            if int(raw[19]) != row['start']:
                raise RuntimeError('SLAM session changed; fixed map origin is invalid')
        super().prepare()
        if any(not self.inside(p) for _,p in self.points):
            raise RuntimeError('target outside fixed corridor')
        from rclpy.action import ActionClient
        from nav2_msgs.action import ComputePathToPose
        planner = ActionClient(self.n, ComputePathToPose, '/compute_path_to_pose')
        start,_ = self.pose()
        plans = []
        try:
            if not planner.wait_for_server(timeout_sec=10):
                raise RuntimeError('planner unavailable')
            for label,point in self.points:
                req=ComputePathToPose.Goal();req.start=self.msg(start);req.goal=self.msg(point)
                req.use_start=True;req.planner_id='GridBased'
                handle=self.wait(planner.send_goal_async(req))
                if not handle.accepted:
                    raise RuntimeError('planner rejected '+label)
                result=self.wait(handle.get_result_async(),30)
                points=[[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses]
                plans.append({'label':label,'points':points})
                (self.output/'planned_paths.json').write_text(json.dumps(plans))
                if result.status!=4 or not points or any(not self.inside(p) for p in points):
                    raise RuntimeError('planned route outside fixed corridor: '+label)
                start=point
        finally:
            planner.destroy()

    def close(self):
        self.status['vendor_stats'] = self.vendor_stats
        try:
            super().close()
        finally:
            self.vendor_file.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    p.add_argument('--origin', required=True)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--cycles', type=int, default=3)
    p.add_argument('--stop-file')
    args = p.parse_args()
    if not 1<=args.cycles<=20:
        p.error('cycles must be in [1,20]; review results between batches')
    args.sim=False;args.distance=1.;args.envelope=1.45;args.angles=[];args.directions=[]
    args.timeout=180.;args.settle=3.
    base.route=lambda *_:[(f'{i+1:03d}_{name}',point) for i in range(args.cycles)
                          for name,point in [('A_front_1m_yaw90',[1.,0.,math.pi/2]),
                                             ('B_back_1m_yaw-90',[-1.,0.,-math.pi/2])]]
    c = TwoGoalCheck(args)
    try:
        c.run()
    except Exception as exc:
        c.status.update(state='failed',error=str(exc))
        print(json.dumps({'error':str(exc)}),flush=True)
        raise
    finally:
        c.close()


if __name__=='__main__':
    main()
