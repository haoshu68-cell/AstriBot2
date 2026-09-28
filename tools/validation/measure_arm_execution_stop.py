#!/usr/bin/env python3
"""Owned Gazebo commissioning experiment; never a production motion entry."""
import argparse,copy,json,math,os,signal,subprocess,sys,time,uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatusArray
from rclpy.qos import QoSProfile,DurabilityPolicy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from astribot_transport_msgs.msg import ExecutionHeartbeat
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.srv import GetStateValidity
from rcl_interfaces.srv import GetParameters

p=argparse.ArgumentParser();p.add_argument('--session',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
p.add_argument('--case',choices=['cancel','lease_loss','revoke','process_loss'],required=True);p.add_argument('--joint',type=int,default=7);p.add_argument('--delta',type=float,default=.2)
p.add_argument('--crash-owner',action='store_true')
a=p.parse_args();manifest=json.loads((a.session/'session.json').read_text())
assert manifest['state']=='ready' and manifest['runtime_preflight']['passed']
assert os.environ['ROS_DOMAIN_ID']==manifest['isolation']['ROS_DOMAIN_ID'] and os.environ['ROS_DOMAIN_ID'] not in ['0','25']
assert 1<=a.joint<=7 and 0<abs(a.delta)<=.2 and not a.output.exists()
assert not a.crash_owner or a.case=='process_loss'
child=None;checkpoint=a.output.with_suffix('.checkpoint.json');statuses={}
if a.case=='process_loss' and not a.crash_owner:
 assert not checkpoint.exists()
 child_log=a.output.with_suffix('.owner.log').open('w')
 child=subprocess.Popen([sys.executable,__file__,'--session',str(a.session),'--output',str(a.output),'--case','process_loss','--delta',str(a.delta),'--joint',str(a.joint),'--crash-owner'],stdout=child_log,stderr=subprocess.STDOUT)
rclpy.init();node=Node('arm_stop_commissioning',parameter_overrides=[Parameter('use_sim_time',value=True)])
controllers=['torso_controller','head_controller','arm_left_controller','arm_right_controller','gripper_left_controller','gripper_right_controller']
lease='commissioning_'+uuid.uuid4().hex;sequence=0;enabled=False;last_sent=None
rows=[];poses=[];joints=None;acks={};events=[];goal=None;result_future=None
report={'case':a.case,'lease':lease,'joint':a.joint,'delta':a.delta,'session':str(a.session),'domain':os.environ['ROS_DOMAIN_ID'],'scope':'Single empty-arm joint, Gazebo position-to-velocity backend; no hardware, payload or automatic recovery claim.','passed':False,'physical_braking_envelope_accepted':False}

def event(name,**extra):
 row={'event':name,'steady':time.monotonic(),'ros':node.get_clock().now().nanoseconds*1e-9,**extra};events.append(row);return row

def state(msg):
 stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
 rows.append({'steady':time.monotonic(),'stamp':stamp,'names':list(msg.joint_names),'actual':list(msg.actual.positions),'velocity':list(msg.actual.velocities),'desired':list(msg.desired.positions),'desired_velocity':list(msg.desired.velocities)})

def pose(msg):
 q=msg.pose.pose.orientation;x=msg.pose.pose.position
 poses.append({'steady':time.monotonic(),'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'frame':msg.header.frame_id,'x':x.x,'y':x.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))})

def joint(msg):
 global joints
 joints=msg

subscriptions=[node.create_subscription(JointTrajectoryControllerState,'/arm_left_controller/state',state,20),node.create_subscription(PoseWithCovarianceStamped,'/slam/pose',pose,qos_profile_sensor_data),node.create_subscription(JointState,'/joint_states',joint,qos_profile_sensor_data)]
for controller in controllers:
 subscriptions.append(node.create_subscription(ExecutionHeartbeat,'/'+controller+'/execution_heartbeat_ack',lambda msg,c=controller:acks.update({c:(time.monotonic(),msg)}),10))
publisher=None if child else node.create_publisher(ExecutionHeartbeat,'/transport/execution_heartbeat',10)
if child:
 subscriptions.append(node.create_subscription(GoalStatusArray,'/arm_left_controller/follow_joint_trajectory/_action/status',lambda msg:statuses.update({bytes(x.goal_info.goal_id.uuid).hex():x.status for x in msg.status_list}),QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)))

def heartbeat(active=True):
 global sequence,last_sent
 sequence+=1;msg=ExecutionHeartbeat();msg.stamp=node.get_clock().now().to_msg();msg.lease_id=lease;msg.sequence=sequence;msg.active=active
 publisher.publish(msg);last_sent=time.monotonic()

def timer_callback():
 if enabled:heartbeat()

timer=node.create_timer(.05,timer_callback,clock=Clock(clock_type=ClockType.STEADY_TIME))

def spin_until(check,timeout,reason):
 end=time.monotonic()+timeout
 while not check():
  if time.monotonic()>=end:raise RuntimeError(reason)
  rclpy.spin_once(node,timeout_sec=.01)

def request(name,kind,req):
 client=node.create_client(kind,name)
 if not client.wait_for_service(timeout_sec=3):raise RuntimeError('SERVICE_UNAVAILABLE '+name)
 future=client.call_async(req);spin_until(future.done,5,'SERVICE_TIMEOUT '+name);response=future.result();node.destroy_client(client);return response

def stationary(since):
 now=time.monotonic();r=[x for x in rows if x['steady']>=since];s=[x for x in poses if x['steady']>=since]
 if len(r)<3 or len(s)<3 or r[-1]['steady']-r[0]['steady']<.6 or s[-1]['steady']-s[0]['steady']<.6:return False
 # Latest continuous 0.6 s windows; all samples must be new and fresh.
 r=[x for x in r if x['steady']>=r[-1]['steady']-.65];s=[x for x in s if x['steady']>=s[-1]['steady']-.7]
 if r[-1]['steady']-r[0]['steady']<.6 or s[-1]['steady']-s[0]['steady']<.6:return False
 if now-r[-1]['steady']>=.3 or now-s[-1]['steady']>=.3:return False
 ros=node.get_clock().now().nanoseconds*1e-9
 if not(0<=ros-r[-1]['stamp']<.3 and 0<=ros-s[-1]['stamp']<.3):return False
 for series in [r,s]:
  if any(x['stamp']<=before['stamp'] or x['steady']-before['steady']>=.3 for before,x in zip(series,series[1:])):return False
 for x in r:
  if len(x['velocity'])!=7 or any(not math.isfinite(v) or abs(v)>.005 for v in x['velocity']):return False
  if any(abs(v-start)>.001 for v,start in zip(x['actual'],r[0]['actual'])):return False
 angle=0
 for previous,x in zip(s,s[1:]):
  angle+=math.remainder(x['yaw']-previous['yaw'],2*math.pi)
  if x['frame']!='map' or math.hypot(x['x']-s[0]['x'],x['y']-s[0]['y'])>.005 or abs(angle)>.01:return False
 return True

try:
 if child:
  event('observer_started')
  spin_until(lambda:checkpoint.exists() or child.poll() is not None,15,'OWNER_DID_NOT_REACH_CRASH_POINT')
  assert checkpoint.exists(),'OWNER_FAILED_BEFORE_CRASH_POINT'
  captured=json.loads(checkpoint.read_text());trigger=captured['trigger'];target=captured['target'];lease=captured['lease']
  report['lease']=lease;report['owner_pid']=captured['pid'];report['owner_events']=captured['events']
  goal_id=next(x['goal_id'] for x in captured['events'] if x['event']=='goal_accepted')
  spin_until(lambda:child.poll() is not None,2,'OWNER_STILL_ALIVE')
  assert child.returncode==-signal.SIGKILL,'OWNER_NOT_KILLED'
  event('owner_process_exit',returncode=child.returncode,pid=child.pid)
  spin_until(lambda:statuses.get(goal_id)==6,3,'INDEPENDENT_ABORTED_STATUS_NOT_OBSERVED')
  event('goal_terminal_independent_status',goal_id=goal_id,status=statuses[goal_id])
  spin_until(lambda:stationary(trigger['steady']),4,'MEASURED_STOP_NOT_OBSERVED')
  confirmed=event('measured_stop_confirmed')
 else:
  started=event('observer_started')['steady'];spin_until(lambda:joints is not None and stationary(started),8,'INITIAL_STOP_NOT_OBSERVED')
  parameters=request('/move_group/get_parameters',GetParameters,GetParameters.Request(names=['allow_trajectory_execution','use_sim_time']))
  assert [x.bool_value for x in parameters.values]==[False,True], 'MOVE_GROUP_EXECUTION_ENABLED_OR_WRONG_CLOCK'
  cs=request('/controller_manager/list_controllers',ListControllers,ListControllers.Request()).controller
  assert all(any(c.name==name and c.type=='astribot_s1_manipulation/OwnedTrajectoryController' and c.state=='active' for c in cs) for name in controllers)
  assert node.count_publishers('/transport/execution_heartbeat')==1,'ANOTHER_EXECUTION_OWNER_PRESENT'
  assert all(node.count_subscribers('/'+c+'/joint_trajectory')==0 for c in controllers),'UNLEASED_TOPIC_COMMAND_ENTRY'
  start=rows[-1]['actual'];names=rows[-1]['names'];target=list(start);target[a.joint-1]+=a.delta
  # Independent MoveGroup collision preflight. This is a sampled commissioning
  # path check, not a continuous dynamic-obstacle or braking certificate.
  for sample in range(41):
   req=GetStateValidity.Request();req.robot_state.joint_state=copy.deepcopy(joints);req.robot_state.is_diff=False
   req.robot_state.joint_state.position=list(joints.position)
   index=list(joints.name).index(names[a.joint-1]);req.robot_state.joint_state.position[index]=start[a.joint-1]+a.delta*sample/40
   response=request('/check_state_validity',GetStateValidity,req)
   if not response.valid:
    raise RuntimeError('COLLISION_PREFLIGHT sample='+str(sample)+' contacts='+str([(c.contact_body_1,c.contact_body_2) for c in response.contacts]))
  event('sampled_collision_preflight_passed',samples=41)
  assert max(abs(x-y) for x,y in zip(start,rows[-1]['actual']))<.001,'START_CHANGED_DURING_PREFLIGHT'
  enabled=True
  spin_until(lambda:all(c in acks and acks[c][1].lease_id==lease and acks[c][1].active and time.monotonic()-acks[c][0]<.3 for c in controllers),3,'CONTROLLER_LEASE_NOT_ACKNOWLEDGED')
  client=ActionClient(node,FollowJointTrajectory,'/arm_left_controller/follow_joint_trajectory')
  assert client.wait_for_server(timeout_sec=3)
  command=FollowJointTrajectory.Goal();command.trajectory.joint_names=names
  for seconds,positions in [(0,start),(4,target)]:
   point=JointTrajectoryPoint();point.time_from_start.sec=seconds;point.positions=positions;point.velocities=[0.]*7;command.trajectory.points.append(point)
  future=client.send_goal_async(command);event('goal_sent');spin_until(future.done,3,'GOAL_ACCEPT_TIMEOUT');goal=future.result();assert goal.accepted,'GOAL_REJECTED'
  event('goal_accepted',goal_id=bytes(goal.goal_id.uuid).hex());result_future=goal.get_result_async()
  spin_until(lambda:abs(rows[-1]['actual'][a.joint-1]-start[a.joint-1])>=.025,3,'MOTION_NOT_OBSERVED')
  trigger=event('stop_trigger',position=rows[-1]['actual'][a.joint-1],last_heartbeat_steady=last_sent)
  if a.crash_owner:
   temporary=checkpoint.with_suffix('.tmp');temporary.write_text(json.dumps({'lease':lease,'pid':os.getpid(),'events':events,'start':start,'target':target,'trigger':trigger},indent=2)+'\n');temporary.replace(checkpoint)
   os.kill(os.getpid(),signal.SIGKILL)
  if a.case=='cancel':
   cancel=goal.cancel_goal_async();spin_until(cancel.done,3,'CANCEL_ACK_TIMEOUT');event('cancel_ack',goals_canceling=len(cancel.result().goals_canceling))
  elif a.case=='lease_loss':enabled=False
  else:enabled=False;heartbeat(False)
  spin_until(result_future.done,3,'GOAL_TERMINAL_TIMEOUT');result=result_future.result();event('goal_terminal',status=result.status,error_code=result.result.error_code,error=result.result.error_string)
  spin_until(lambda:stationary(trigger['steady']),4,'MEASURED_STOP_NOT_OBSERVED');confirmed=event('measured_stop_confirmed')
 after=[x for x in rows if x['steady']>=trigger['steady']]
 hold=next((x for x in after if len(x['desired_velocity'])==7 and max(map(abs,x['desired_velocity']))<1e-5),None)
 assert hold is not None,'HOLD_COMMAND_NOT_OBSERVED'
 if not child:
  assert result.status==(5 if a.case=='cancel' else 6),'WRONG_GOAL_TERMINAL'
  if a.case!='cancel':assert 'EXECUTION_OWNER_EXPIRED' in result.result.error_string
 assert abs(rows[-1]['actual'][a.joint-1]-target[a.joint-1])>.02,'OLD_TRAJECTORY_CONTINUED_TO_END'
 report.update(passed=True,stop_to_desired_hold_seconds=hold['steady']-trigger['steady'],stop_to_confirmation_seconds=confirmed['steady']-trigger['steady'],maximum_post_trigger_displacement_rad=max(abs(x['actual'][a.joint-1]-trigger['position']) for x in after),last_heartbeat_to_hold_seconds=hold['steady']-trigger['last_heartbeat_steady'],final_position=rows[-1]['actual'][a.joint-1])
except Exception as error:
 report['error']=repr(error)
 raise
finally:
 enabled=False
 if child:
  if child.poll() is None:
   child.kill();child.wait(timeout=3)
  child_log.close()
 if last_sent is not None:
  heartbeat(False)
  if goal is not None and (result_future is None or not result_future.done()):goal.cancel_goal_async()
  end=time.monotonic()+1
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
 report.update(events=events,joint_samples=rows,slam_samples=poses,ended_wall=time.time());destination=a.output.with_suffix('.owner.failure.json') if a.crash_owner else a.output;destination.write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps({k:v for k,v in report.items() if k not in ['events','joint_samples','slam_samples']},indent=2))
 node.destroy_node();rclpy.shutdown()
