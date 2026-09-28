import json,time,hashlib
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState,PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from astribot_transport_msgs.msg import ObservedOctomap
from control_msgs.msg import JointTrajectoryControllerState
from rosgraph_msgs.msg import Clock
rclpy.init();node=Node('arm_recovery_preflight',parameter_overrides=[Parameter('use_sim_time',value=True)])
result={'scope':'Read-only live owned simulation preflight','started_wall':time.time(),'counts':{},'last':{}}
subs=[]
for topic,kind in [('/clock',Clock),('/joint_states',JointState),('/slam/pose',PoseWithCovarianceStamped),('/arm_left_controller/state',JointTrajectoryControllerState),('/camera/head_rgbd/points',PointCloud2),('/camera/torso_rgbd/points',PointCloud2),('/moveit/observed_octomap',ObservedOctomap)]:
 def record(msg,topic=topic):
  result['counts'][topic]=result['counts'].get(topic,0)+1
  out={'receive_monotonic':time.monotonic()}
  if hasattr(msg,'header'):out.update(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,frame=msg.header.frame_id)
  if isinstance(msg,JointState):out.update(names=list(msg.name),positions=list(msg.position),velocities=list(msg.velocity))
  elif isinstance(msg,PoseWithCovarianceStamped):
   p=msg.pose.pose.position;q=msg.pose.pose.orientation;out.update(position=[p.x,p.y,p.z],orientation=[q.x,q.y,q.z,q.w])
  elif isinstance(msg,JointTrajectoryControllerState):out.update(names=list(msg.joint_names),actual=list(msg.actual.positions),desired=list(msg.desired.positions),velocities=list(msg.actual.velocities))
  elif isinstance(msg,PointCloud2):out.update(width=msg.width,height=msg.height,point_step=msg.point_step)
  elif isinstance(msg,ObservedOctomap):out.update(source=msg.source_id,epoch=msg.map_epoch,revision=msg.map_revision,bytes=len(msg.octomap.data),free_keys=len(msg.ray_free_keys)//3,processing_seconds=msg.processing_seconds,sensor_frame=msg.sensor_frame,sensor_origin=[msg.sensor_to_map.translation.x,msg.sensor_to_map.translation.y,msg.sensor_to_map.translation.z],cloud_origin=[msg.cloud_to_map.translation.x,msg.cloud_to_map.translation.y,msg.cloud_to_map.translation.z])
  elif isinstance(msg,Clock):out['clock']=msg.clock.sec+msg.clock.nanosec*1e-9
  result['last'][topic]=out
 subs.append(node.create_subscription(kind,topic,record,10 if kind in (ObservedOctomap,JointTrajectoryControllerState) else qos_profile_sensor_data))
requests=[('/controller_manager/list_controllers',ListControllers,ListControllers.Request()),('/omni_effort_drive_node/get_parameters',GetParameters,GetParameters.Request(names=['idle_position_hold','idle_position_kp'])),('/move_group/get_parameters',GetParameters,GetParameters.Request(names=['allow_trajectory_execution','use_sim_time','robot_description','robot_description_semantic'])),('/robot_state_publisher/get_parameters',GetParameters,GetParameters.Request(names=['robot_description']))]
clients=[];futures=[]
for name,kind,request in requests:
 client=node.create_client(kind,name);clients.append(client)
 if client.wait_for_service(timeout_sec=2):futures.append((name,client.call_async(request)))
 else:result[name]={'error':'SERVICE_UNAVAILABLE'}
end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
root=Path('/tmp/astribot_arm_dynamic_recovery/simulation')
for name,future in futures:
 if not future.done():result[name]={'error':'RESPONSE_TIMEOUT'};continue
 response=future.result()
 if hasattr(response,'controller'):result[name]=[{'name':c.name,'state':c.state,'type':c.type,'claimed_interfaces':list(c.claimed_interfaces)} for c in response.controller]
 else:
  values=[]
  for index,value in enumerate(response.values):
   if len(value.string_value)>200:
    output=root/(name.strip('/').replace('/','_')+'_'+str(index)+'.xml');output.write_text(value.string_value)
    values.append({'file':str(output),'sha256':hashlib.sha256(value.string_value.encode()).hexdigest()})
   else:values.append({'type':value.type,'bool':value.bool_value,'double':value.double_value,'string':value.string_value})
  result[name]=values
result['ended_wall']=time.time();(root/'preflight_live02.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'counts':result['counts'],'controllers':result.get('/controller_manager/list_controllers'),'chassis_parameters':result.get('/omni_effort_drive_node/get_parameters'),'move_group_parameters':result.get('/move_group/get_parameters'),'last':{k:v for k,v in result['last'].items() if k in ['/slam/pose','/moveit/observed_octomap']}},indent=2))
node.destroy_node();rclpy.shutdown()
