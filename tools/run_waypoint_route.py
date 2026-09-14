#!/usr/bin/env python3
"""Sequential simulation endurance runner with arrival and in-motion measurements."""
import argparse
import csv
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import time

DEFAULT_ROUTE = [(0., -8.3, -math.pi/2), (0., 0., -math.pi/2),
                 (-5.31, 9.25, -math.pi/2), (5.45, 1.9, math.pi),
                 (-2.8, -8., math.pi), (0., 0., 0.)]
SHORT_ROUTE = [(1., 0., 0.), (1., -1., 0.), (0., -2., -math.pi/2),
               (0., -2., math.pi/2), (0., 0., 0.)]


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def wrap(a):
    return math.remainder(a, 2*math.pi)


def distribution(values):
    values = sorted(abs(v) for v in values)
    if not values:
        return {'n': 0, 'rms': None, 'p95': None, 'max': None}
    return {'n': len(values), 'rms': math.sqrt(statistics.mean(v*v for v in values)),
            'p95': values[min(len(values)-1, math.ceil(.95*len(values))-1)], 'max': values[-1]}


def project(points, x, y, anchor=None):
    best = None; arc = 0.
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]; length = math.hypot(dx, dy)
        if length < 1e-8:
            continue
        if anchor is not None and not anchor-1. <= arc <= anchor+3.:
            arc += length; continue
        f = max(0., min(1., ((x-a[0])*dx+(y-a[1])*dy)/length**2))
        px, py = a[0]+f*dx, a[1]+f*dy
        d = math.hypot(x-px, y-py); tangent = math.atan2(dy, dx)
        sign = 1 if dx*(y-py)-dy*(x-px) >= 0 else -1
        # Direction change across a 0.5m neighbourhood classifies straight segments.
        lo, hi = i, i+1
        while lo>0 and math.hypot(points[lo][0]-a[0], points[lo][1]-a[1])<.25:
            lo -= 1
        while hi+1<len(points) and math.hypot(points[hi][0]-b[0], points[hi][1]-b[1])<.25:
            hi += 1
        left = math.atan2(a[1]-points[lo][1], a[0]-points[lo][0]) if lo<i else tangent
        right = math.atan2(points[hi][1]-b[1], points[hi][0]-b[0]) if hi>i+1 else tangent
        curvature = wrap(right-left)/max(.1, math.hypot(points[hi][0]-points[lo][0], points[hi][1]-points[lo][1]))
        row = (d, sign*d, tangent, arc+f*length, curvature)
        if best is None or d<best[0]:
            best = row
        arc += length
    return best


def heading_convergence(rows):
    first = previous = stable_since = None
    converged = None
    bad_duration = longest_bad = 0.
    for row in rows:
        travelling = (row['phase'] == 'FOLLOW' and row['heading_error_deg'] is not None and
                      row.get('goal_distance_m', math.inf) > row.get('terminal_heading_radius', 0))
        if not travelling:
            previous = stable_since = None
            bad_duration = 0.
            continue
        if first is None:
            first = row['sim_s']
        contiguous = (previous is not None and previous['revision'] == row['revision'] and
                      0 < row['sim_s'] - previous['sim_s'] < .2)
        if not contiguous:
            stable_since = None
            bad_duration = 0.
        if abs(row['heading_error_deg']) <= 5:
            if stable_since is None:
                stable_since = row['sim_s']
            if converged is None and row['sim_s'] - stable_since >= .5:
                converged = stable_since - first
        else:
            stable_since = None
        if contiguous and abs(previous['heading_error_deg']) > 15 and abs(row['heading_error_deg']) > 15:
            bad_duration += row['sim_s'] - previous['sim_s']
            longest_bad = max(longest_bad, bad_duration)
        else:
            bad_duration = 0.
        previous = row
    return {'first_stable_5deg_s': converged, 'longest_over_15deg_s': longest_bad if first is not None else None}


def measured_motion(rows):
    """Differentiate velocities at their acquisition stamps, not at poll times."""
    samples=[];segment=0;context=None
    for row in rows:
        stamp=row.get('velocity_stamp_s')
        if stamp is None:continue
        current=(row['phase'],row['revision'])
        if current!=context or (samples and stamp<samples[-1]['velocity_stamp_s']):segment+=1
        context=current
        if samples and stamp==samples[-1]['velocity_stamp_s']:continue
        samples.append(dict(row,_motion_segment=segment))
    acceleration=[];angular=[];jerk=[]
    def contiguous(a,b):
        return (a['phase']==b['phase']=='FOLLOW' and a['revision']==b['revision']
                and a['_motion_segment']==b['_motion_segment']
                and 0<b['velocity_stamp_s']-a['velocity_stamp_s']<.2)
    for a,b in zip(samples,samples[1:]):
        if contiguous(a,b):
            dt=b['velocity_stamp_s']-a['velocity_stamp_s']
            acceleration.append((b['speed']-a['speed'])/dt)
            angular.append((b['wz']-a['wz'])/dt)
    for a,b,c in zip(samples,samples[1:],samples[2:]):
        if contiguous(a,b) and contiguous(b,c):
            dt1=b['velocity_stamp_s']-a['velocity_stamp_s'];dt2=c['velocity_stamp_s']-b['velocity_stamp_s']
            jerk.append(((c['speed']-b['speed'])/dt2-(b['speed']-a['speed'])/dt1)/((dt1+dt2)/2))
    return {'time_basis':'odometry_header','unique_samples':len(samples),
            'acceleration_m_s2':distribution(acceleration),
            'angular_acceleration_rad_s2':distribution(angular),'jerk_m_s3':distribution(jerk)}


def metrics(rows):
    follow = [r for r in rows if r['phase']=='FOLLOW' and r['cross_track_m'] is not None]
    straight = [r for r in follow if abs(r['reference_curvature']) < .1]
    pairs = [(a,b) for a,b in zip(follow,follow[1:]) if 0 < b['sim_s']-a['sim_s'] < .2 and a['revision']==b['revision']]
    acceleration, angular_acceleration, jerk = [], [], []
    spin = heading_bad = travel_heading_bad = backwards = rotation = stalled = longest_stall = 0.; changes = 0; previous_sign = 0
    for a,b in pairs:
        dt = b['sim_s']-a['sim_s']
        acceleration.append((b['speed']-a['speed'])/dt)
        angular_acceleration.append((b['wz']-a['wz'])/dt)
        if b['speed']<.03 and abs(b['wz'])>.2:
            spin += dt
        if abs(b['heading_error_deg'])>30:
            heading_bad += dt
            if b.get('goal_distance_m',math.inf)>b.get('terminal_heading_radius',0):travel_heading_bad += dt
        backwards += max(0., a['progress_m']-b['progress_m'])
        rotation += abs(b['wz'])*dt
        stalled = stalled+dt if b['speed']<.02 else 0.
        longest_stall = max(longest_stall,stalled)
        if abs(b['cross_track_m'])>.02:
            sign = 1 if b['cross_track_m']>0 else -1
            changes += int(previous_sign != 0 and previous_sign != sign); previous_sign = sign
    # Only consecutive samples may contribute jerk; never differentiate over gaps or phase changes.
    for a,b,c in zip(rows,rows[1:],rows[2:]):
        dt1,dt2 = b['sim_s']-a['sim_s'],c['sim_s']-b['sim_s']
        if a['phase']==b['phase']==c['phase']=='FOLLOW' and a['revision']==b['revision']==c['revision'] and 0<dt1<.2 and 0<dt2<.2:
            jerk.append(((c['speed']-b['speed'])/dt2-(b['speed']-a['speed'])/dt1)/((dt1+dt2)/2))
    trajectory=[]
    for r in follow:
        if not trajectory or math.hypot(r['x']-trajectory[-1]['x'],r['y']-trajectory[-1]['y'])>=.1:
            trajectory.append(r)
    curvature=[]; curvature_rate=[]; previous_k=None
    for a,b,c in zip(trajectory,trajectory[1:],trajectory[2:]):
        ab=math.hypot(b['x']-a['x'],b['y']-a['y']); bc=math.hypot(c['x']-b['x'],c['y']-b['y'])
        if a['revision']==b['revision']==c['revision'] and c['sim_s']-a['sim_s']<3 and ab>0 and bc>0:
            k=wrap(math.atan2(c['y']-b['y'],c['x']-b['x'])-math.atan2(b['y']-a['y'],b['x']-a['x']))/((ab+bc)/2)
            curvature.append(k)
            if previous_k is not None:curvature_rate.append((k-previous_k)/ab)
            previous_k=k
        else:previous_k=None
    hold_time={};hold_events=0;held=False
    for i,row in enumerate(rows):
        current=row.get('policy_hold') is True
        hold_events+=int(current and not held);held=current
        if current and i+1<len(rows):
            dt=rows[i+1]['sim_s']-row['sim_s']
            if 0<dt<.2:
                reason=row.get('policy_reason','UNKNOWN')
                hold_time[reason]=hold_time.get(reason,0.)+dt
    return {'samples': len(rows), 'follow_samples': len(follow),
            'measured_motion':measured_motion(rows),
            'policy_hold_events':hold_events,'policy_hold_s_by_reason':hold_time,
            'heading_convergence': heading_convergence(rows),
            'cross_track_m': distribution([r['cross_track_m'] for r in follow]),
            'straight_cross_track_m': distribution([r['cross_track_m'] for r in straight]),
            'front_heading_error_deg': distribution([r['heading_error_deg'] for r in follow]),
            'straight_heading_error_deg': distribution([r['heading_error_deg'] for r in straight]),
            'acceleration_m_s2': distribution(acceleration), 'angular_acceleration_rad_s2': distribution(angular_acceleration),
            'jerk_m_s3': distribution(jerk), 'follow_spin_s': spin, 'heading_over_30deg_s': heading_bad, 'travel_heading_over_30deg_s':travel_heading_bad,
            'travel_heading_error_deg':distribution([r['heading_error_deg'] for r in follow if r.get('goal_distance_m',math.inf)>r.get('terminal_heading_radius',0)]),
            'backtracking_m': backwards, 'cross_track_sign_changes_2cm': changes,
            'follow_rotation_rad':rotation, 'longest_follow_stall_s':longest_stall,
            'trajectory_curvature_1_m':distribution(curvature), 'trajectory_curvature_rate_1_m2':distribution(curvature_rate),
            'phase_coverage':sum(r['phase']!='UNKNOWN' for r in rows)/max(1,len(rows))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', help='JSON array of [x, y, yaw_radians] in map frame')
    parser.add_argument('--short-route', action='store_true')
    parser.add_argument('--duration', type=float, default=7200, help='wall-clock seconds; cancels active goal at deadline')
    parser.add_argument('--cycles', type=int, default=0, help='0: limited only by duration')
    parser.add_argument('--timeout', type=float, default=300)
    parser.add_argument('--settle', type=float, default=2)
    parser.add_argument('--output', default=f'/tmp/astribot_route_{time.strftime("%Y%m%d_%H%M%S")}')
    parser.add_argument('--dry-run', action='store_true', help='check all route legs using the planner; no movement')
    args = parser.parse_args()
    if not all(math.isfinite(v) and v>0 for v in (args.duration,args.timeout,args.settle)) or args.cycles<0:
        parser.error('duration, timeout, settle must be positive; cycles must be nonnegative')
    route = json.loads(Path(args.route).read_text()) if args.route else SHORT_ROUTE if args.short_route else DEFAULT_ROUTE
    if not route or any(len(p)!=3 or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in p) for p in route):
        parser.error('route must contain finite [x,y,yaw_radians] points')
    route = [[float(v) for v in p] for p in route]
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter, parameter_value_to_python
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.srv import GetParameters
    from nav_msgs.msg import Path as RosPath, Odometry
    from nav2_msgs.action import NavigateToPose, ComputePathToPose
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import PoseStamped, Twist
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    lock = open('/tmp/astribot_waypoint_'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock','a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    output = Path(args.output).resolve(); output.mkdir(parents=True, exist_ok=False)
    # Keep the ROS context alive until the owned action cancellation has finished.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('waypoint_endurance', parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer = Buffer(); listener = TransformListener(buffer,node)
    state = {'odom': None, 'odom_at': 0., 'phase': 'UNKNOWN', 'phase_at':0., 'path': [], 'revision': 0,
             'cmd': Twist(), 'final_cmd':Twist(), 'protection':{}, 'protection_at':0.,
             'events': [], 'quality': [], 'active_other': False, 'recording': False, 'rows': [],
             'anchor': None, 'invalid_samples':0, 'last_valid':time.monotonic()}
    status = {'state':'preflight', 'pid':os.getpid(), 'started':time.time(), 'route':route, 'completed_goals':0,
              'deadline':time.time()+args.duration, 'output':str(output)}
    def save_status():
        status['updated'] = time.time()
        tmp = output/'status.tmp';tmp.write_text(json.dumps(status,indent=2));tmp.replace(output/'status.json')
    save_status()
    (output/'runner_snapshot.py').write_bytes(Path(__file__).read_bytes())
    terminal_heading_radius=.3
    def odom(msg):
        state.update(odom=msg,odom_at=time.monotonic())
    def plan(msg):
        if not state['recording']:
            return
        if msg.header.frame_id != 'map':
            state['path'] = []; return
        state.update(path=[(p.pose.position.x,p.pose.position.y) for p in msg.poses], anchor=None,
                     revision=state['revision']+1)
        with open(output/'plans.jsonl','a') as f:
            f.write(json.dumps({'wall_s':time.time(),'cycle':status.get('cycle'),'goal_index':status.get('goal_index'),'revision':state['revision'],'frame':msg.header.frame_id,'poses':[[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in msg.poses]})+'\n')
    def phase(msg):
        state.update(phase=msg.data,phase_at=time.monotonic())
    def event(msg, key):
        if state['recording']:
            try:
                state[key].append(json.loads(msg.data))
            except ValueError:
                state[key].append({'invalid':msg.data})
    def protection(msg):
        try:state.update(protection=json.loads(msg.data),protection_at=time.monotonic())
        except (ValueError,TypeError):pass
    node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    node.create_subscription(RosPath,'/plan',plan,10)
    node.create_subscription(String,'/path_tracking/phase',phase,qos_profile_sensor_data)
    node.create_subscription(String,'/path_tracking/replan_event',lambda m:event(m,'events'),10)
    node.create_subscription(String,'/path_tracking/path_quality',lambda m:event(m,'quality'),10)
    node.create_subscription(Twist,'/cmd_vel_nav_body_raw',lambda m:state.update(cmd=m),10)
    node.create_subscription(Twist,'/cmd_vel',lambda m:state.update(final_cmd=m),10)
    node.create_subscription(String,'/navigation_policy/protection_state',protection,10)
    node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',
        lambda m:state.update(active_other=any(s.status in (1,2,3) for s in m.status_list)),10)
    def pose():
        tf = buffer.lookup_transform('map','astribot_torso_base',rclpy.time.Time())
        age = node.get_clock().now().nanoseconds/1e9-(tf.header.stamp.sec+tf.header.stamp.nanosec/1e9)
        if not -.05<=age<=.3 or time.monotonic()-state['odom_at']>.3:
            raise RuntimeError('stale TF or odometry')
        t = tf.transform.translation
        return t.x,t.y,yaw(tf.transform.rotation)
    csv_file = open(output/'samples.csv','w',newline=''); writer = None
    last_sample = 0.;last_status = 0.
    def spin():
        nonlocal writer,last_sample,last_status
        rclpy.spin_once(node,timeout_sec=.02)
        now = time.monotonic()
        if state['recording'] and now-last_sample>=.05:
            last_sample=now
            try:
                x,y,theta=pose()
                sim=node.get_clock().now().nanoseconds/1e9
                od=state['odom'].twist.twist; speed=math.hypot(od.linear.x,od.linear.y)
                row={'cycle':status.get('cycle',0),'goal_index':status.get('goal_index',0),
                     'goal_distance_m':math.hypot(x-status['goal'][0],y-status['goal'][1]),'terminal_heading_radius':terminal_heading_radius,
                     'wall_s':time.time(),'sim_s':sim,
                     'velocity_stamp_s':state['odom'].header.stamp.sec+state['odom'].header.stamp.nanosec/1e9,'x':x,'y':y,'yaw':theta,'speed':speed,'wz':od.angular.z,
                     'cmd_vx':state['cmd'].linear.x,'cmd_vy':state['cmd'].linear.y,'cmd_wz':state['cmd'].angular.z,
                     'final_cmd_vx':state['final_cmd'].linear.x,'final_cmd_vy':state['final_cmd'].linear.y,
                     'final_cmd_wz':state['final_cmd'].angular.z,
                     'policy_hold':state['protection'].get('hold') if now-state['protection_at']<.3 else None,
                     'policy_reason':state['protection'].get('reason','UNKNOWN') if now-state['protection_at']<.3 else 'UNKNOWN',
                     'phase':state['phase'] if now-state['phase_at']<.3 else 'UNKNOWN','revision':state['revision'],
                     'cross_track_m':None,'heading_error_deg':None,'progress_m':None,'reference_curvature':None}
                pr=project(state['path'],x,y,state['anchor'])
                if pr:
                    _,ct,tangent,arc,k=pr;state['anchor']=arc
                    row.update(cross_track_m=ct,heading_error_deg=math.degrees(wrap(theta-tangent)),progress_m=arc,reference_curvature=k)
                if writer is None:
                    writer=csv.DictWriter(csv_file,fieldnames=list(row));writer.writeheader()
                writer.writerow(row);state['rows'].append(row);state['last_valid']=now
            except Exception:
                state['invalid_samples']+=1
            if now-state['last_valid']>2:
                raise RuntimeError('measurement unavailable for 2s; cancel run')
        if now-last_status>5:
            last_status=now;save_status();csv_file.flush()
    def wait(future,timeout):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:
            spin()
        if not future.done():
            raise TimeoutError('ROS action/service response timeout')
        return future.result()
    def wait_action_server(client, timeout, name):
        end = time.monotonic() + timeout
        ready_since = None
        while time.monotonic() < end:
            spin()
            if client.server_is_ready():
                if ready_since is None:
                    ready_since = time.monotonic()
                if time.monotonic() - ready_since >= 2.0:
                    return
            else:
                ready_since = None
        raise RuntimeError(f'{name} unavailable or discovery did not settle')

    def fresh_pose(timeout=15):
        end = time.monotonic() + timeout
        while True:
            spin()
            try:
                return pose()
            except Exception:
                if time.monotonic() >= end:
                    raise

    def make_pose(point):
        p=PoseStamped();p.header.frame_id='map';p.header.stamp=node.get_clock().now().to_msg()
        p.pose.position.x,p.pose.position.y=point[:2];p.pose.orientation.z=math.sin(point[2]/2);p.pose.orientation.w=math.cos(point[2]/2)
        return p
    handle = result_future = None
    def cancel():
        if handle is not None and result_future is not None and not result_future.done():
            # Stop recording so missing measurement cannot interrupt cancellation.
            state['recording']=False
            wait(handle.cancel_goal_async(),10)
            status['cancel_action_status'] = wait(result_future,15).status
    try:
        params=node.create_client(GetParameters,'/controller_server/get_parameters')
        if not params.wait_for_service(timeout_sec=20):
            raise RuntimeError('controller parameters unavailable')
        keys=['use_sim_time','precise_goal_checker.xy_goal_tolerance','precise_goal_checker.yaw_goal_tolerance','FollowPath.inner.GoalAngleCritic.threshold_to_consider','FollowPath.arrival.capture_radius','FollowPath.inner.vx_max',
              'FollowPath.inner.vy_max','FollowPath.inner.CurvatureSpeedLimitCritic.v_min_turn',
              'navigation_policy_enabled']
        values=wait(params.call_async(GetParameters.Request(names=keys)),10).values
        if not values[0].bool_value:
            raise RuntimeError('this runner requires a simulation stack (use_sim_time=true)')
        xy_limit,yaw_limit=values[1].double_value,values[2].double_value
        terminal_heading_radius=max(values[3].double_value,values[4].double_value,.3)
        (output/'metadata.json').write_text(json.dumps({'arguments':vars(args),'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'xy_limit':xy_limit,'yaw_limit':yaw_limit,'terminal_heading_radius':terminal_heading_radius,'controller_parameters':dict(zip(keys,[parameter_value_to_python(v) for v in values])),'ros_domain':os.environ.get('ROS_DOMAIN_ID')},indent=2))
        if not 0<xy_limit<=.03 or not 0<yaw_limit<=math.radians(1.5)+1e-9:
            raise RuntimeError(f'arrival limits too loose/missing: {xy_limit}, {yaw_limit}')
        end=time.monotonic()+2
        while time.monotonic()<end:spin()
        fresh_pose()
        if state['active_other']:
            raise RuntimeError('another navigation goal is active; runner will not preempt it')
        planner=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        wait_action_server(planner,15,'planner')
        preflight=[];start=fresh_pose()
        for index,point in enumerate(route):
            request=ComputePathToPose.Goal();request.start=make_pose(start);request.goal=make_pose(point)
            request.use_start=True;request.planner_id='GridBased'
            handle=wait(planner.send_goal_async(request),10)
            if not handle.accepted:raise RuntimeError('preflight goal rejected')
            result_future=handle.get_result_async();res=wait(result_future,30)
            ok=res.status==4 and bool(res.result.path.poses)
            preflight.append({'index':index,'goal':point,'status':res.status,'poses':len(res.result.path.poses)})
            (output/'preflight.json').write_text(json.dumps(preflight,indent=2))
            if not ok:raise RuntimeError(f'route leg {index} is not safely plannable')
            start=point
        # Include the cycle seam, including non-origin route files.
        if args.cycles != 1:
            request.start=make_pose(route[-1]);request.goal=make_pose(route[0])
            handle=wait(planner.send_goal_async(request),10);result_future=handle.get_result_async();res=wait(result_future,30)
            if res.status!=4 or not res.result.path.poses:raise RuntimeError('cycle seam is not safely plannable')
        if args.dry_run:
            status['state']='preflight_passed';return
        drain=time.monotonic()+.5
        while time.monotonic()<drain:spin()
        nav=ActionClient(node,NavigateToPose,'/navigate_to_pose')
        wait_action_server(nav,15,'navigation')
        deadline=time.monotonic()+args.duration;status['deadline']=time.time()+args.duration;status['state']='running'
        cycle=0
        while time.monotonic()<deadline and (not args.cycles or cycle<args.cycles):
            for index,point in enumerate(route):
                if time.monotonic()>=deadline:break
                status.update(cycle=cycle+1,goal_index=index,goal=point);save_status()
                state.update(recording=True,rows=[],events=[],quality=[],invalid_samples=0,last_valid=time.monotonic(),phase='UNKNOWN',path=[],anchor=None)
                begin=time.monotonic();began_at=time.time();revision=state['revision']
                if state['active_other']:raise RuntimeError('another navigation goal became active; refusing preemption')
                request=NavigateToPose.Goal();request.pose=make_pose(point)
                handle=wait(nav.send_goal_async(request),10)
                if not handle.accepted:raise RuntimeError('navigation goal rejected')
                result_future=handle.get_result_async()
                while not result_future.done() and time.monotonic()<min(deadline,begin+args.timeout):spin()
                if not result_future.done():
                    cancel()
                    if time.monotonic()>=deadline:
                        status['state']='duration_complete';return
                    raise TimeoutError(f'goal {index} exceeded {args.timeout}s')
                result=result_future.result()
                settle_end=time.monotonic()+args.settle
                while time.monotonic()<settle_end:spin()
                actual=pose();odom=state['odom'].twist.twist
                xy=math.hypot(actual[0]-point[0],actual[1]-point[1]);angle=abs(wrap(actual[2]-point[2]))
                quality=metrics(state['rows'])
                passed=result.status==4 and xy<=xy_limit and angle<=yaw_limit and math.hypot(odom.linear.x,odom.linear.y)<=.01 and abs(odom.angular.z)<=.01
                warnings=[]
                if quality['cross_track_m']['p95'] is not None and quality['cross_track_m']['p95']>.10:warnings.append('cross_track_p95_over_10cm')
                if quality['travel_heading_over_30deg_s']>1:warnings.append('front_heading_over_30deg')
                if quality['follow_spin_s']>.5:warnings.append('unexpected_follow_spin')
                if state['invalid_samples']:warnings.append('measurement_gaps')
                if quality['longest_follow_stall_s']>2:warnings.append('follow_stall_over_2s')
                row={'started_at':began_at,'finished_at':time.time(),'cycle':cycle+1,'index':index,'goal':point,'actual':actual,'action_status':result.status,'passed':passed,
                     'xy_m':xy,'yaw_deg':math.degrees(angle),'elapsed_s':time.monotonic()-begin,'metrics':quality,
                     'path_revisions':state['revision']-revision,'replan_events':state['events'],'path_quality':state['quality'],
                     'invalid_samples':state['invalid_samples'],'warnings':warnings}
                with open(output/'results.jsonl','a') as f:f.write(json.dumps(row)+'\n')
                print(json.dumps(row),flush=True)
                status.update(completed_goals=status['completed_goals']+1,last_result=row);state['recording']=False;save_status()
                if not passed:raise RuntimeError(f'goal {index} failed action/arrival/settled-speed check')
            cycle+=1
        status['state']='completed'
    except KeyboardInterrupt:
        status['state']='stopped'
    except Exception as exc:
        status.update(state='failed',error=str(exc));raise
    finally:
        try:cancel()
        except Exception as exc:status['cancel_error']=str(exc)
        status['ended']=time.time();save_status();csv_file.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    main()
