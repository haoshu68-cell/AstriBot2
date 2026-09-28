"""Read-only closeout observer for the owned domain94 commissioning session."""
import json,math,os,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
from action_msgs.msg import GoalStatusArray
root=Path('/tmp/astribot_arm_dynamic_recovery/simulation');assert os.environ['ROS_DOMAIN_ID']=='94'
assert json.loads((root/'session02/session.json').read_text())['state']=='ready'
rclpy.init();node=Node('arm_stop_closeout_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
rows=[];poses=[];statuses={};subscriptions=[]
def joints(msg):
 rows.append({'steady':time.monotonic(),'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'names':list(msg.name),'position':list(msg.position),'velocity':list(msg.velocity)})
def pose(msg):
 p=msg.pose.pose.position;q=msg.pose.pose.orientation
 poses.append({'steady':time.monotonic(),'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'frame':msg.header.frame_id,'x':p.x,'y':p.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))})
subscriptions+=[node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data),node.create_subscription(PoseWithCovarianceStamped,'/slam/pose',pose,qos_profile_sensor_data)]
controllers=['torso_controller','head_controller','arm_left_controller','arm_right_controller','gripper_left_controller','gripper_right_controller']
for controller in controllers:
 subscriptions.append(node.create_subscription(GoalStatusArray,'/'+controller+'/follow_joint_trajectory/_action/status',lambda msg,c=controller:statuses.update({c:[{'uuid':bytes(s.goal_info.goal_id.uuid).hex(),'status':s.status} for s in msg.status_list]}),QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
started=time.monotonic();end=started+2
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
report={'scope':'Read-only goal terminal and measured stop before owned simulation shutdown','passed':False,'joint_samples':rows,'slam_samples':poses,'goal_statuses':statuses,'heartbeat_publishers':node.count_publishers('/transport/execution_heartbeat')}
try:
 assert report['heartbeat_publishers']==0,'EXECUTION_OWNER_STILL_PRESENT'
 assert statuses.get('arm_left_controller') and all(s['status'] in [4,5,6] for messages in statuses.values() for s in messages),'NONTERMINAL_GOAL'
 now=time.monotonic();ros=node.get_clock().now().nanoseconds*1e-9
 for samples in [rows,poses]:
  assert len(samples)>=3 and samples[-1]['steady']-samples[0]['steady']>=.6,'INSUFFICIENT_STOP_WINDOW'
  assert now-samples[-1]['steady']<.3 and 0<=ros-samples[-1]['stamp']<.3,'STALE_STOP_EVIDENCE'
  assert all(x['stamp']>a['stamp'] and x['steady']-a['steady']<.3 for a,x in zip(samples,samples[1:])),'OLD_OR_GAPPED_STOP_EVIDENCE'
 for x in rows:
  assert x['names']==rows[0]['names'] and len(x['velocity'])==len(x['position'])==len(x['names'])
  assert all(math.isfinite(v) and abs(v)<=.005 for v in x['velocity']),'JOINT_VELOCITY'
  assert all(abs(v-a)<=.001 for v,a in zip(x['position'],rows[0]['position'])),'JOINT_DISPLACEMENT'
 angle=0
 for previous,x in zip(poses,poses[1:]):
  angle+=math.remainder(x['yaw']-previous['yaw'],2*math.pi)
  assert x['frame']=='map' and math.hypot(x['x']-poses[0]['x'],x['y']-poses[0]['y'])<=.005 and abs(angle)<=.01,'BASE_SLAM_DISPLACEMENT'
 report['passed']=True
except Exception as error:
 report['error']=repr(error)
 raise
finally:
 report['ended_wall']=time.time();(root/'closeout_stop.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ['joint_samples','slam_samples']},indent=2));node.destroy_node();rclpy.shutdown()
