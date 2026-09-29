#!/usr/bin/env python3
"""Read live state and request motion plans only; never sends an execution action."""
import argparse,json,time,copy
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan,GetPlanningScene,GetStateValidity
from moveit_msgs.msg import Constraints,JointConstraint,PlanningSceneComponents
from rcl_interfaces.srv import GetParameters
from rosidl_runtime_py.convert import message_to_ordereddict

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 rclpy.init();node=Node('ready_right_planning_only_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
 joints=[];sub=node.create_subscription(JointState,'/joint_states',lambda m:joints.append(m) if len(joints)<10 else joints.__setitem__(-1,m),qos_profile_sensor_data)
 def call(client,req,seconds):
  if not client.wait_for_service(timeout_sec=10):raise RuntimeError('service unavailable '+client.srv_name)
  f=client.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=seconds)
  if not f.done():raise RuntimeError('service timeout '+client.srv_name)
  return f.result()
 scene_client=node.create_client(GetPlanningScene,'/get_planning_scene');plan_client=node.create_client(GetMotionPlan,'/plan_kinematic_path');valid_client=node.create_client(GetStateValidity,'/check_state_validity')
 try:
  warm_until=time.monotonic()+3
  while time.monotonic()<warm_until:rclpy.spin_once(node,timeout_sec=.02)
  parameters=node.create_client(GetParameters,'/move_group/get_parameters')
  check=call(parameters,GetParameters.Request(names=['allow_trajectory_execution']),10)
  if len(check.values)!=1 or check.values[0].type!=1 or check.values[0].bool_value:
   raise RuntimeError('planning diagnostic requires allow_trajectory_execution=false')
  req=GetPlanningScene.Request();req.components.components=1023
  scene=call(scene_client,req,15).scene
  (a.output/'scene.json').write_text(json.dumps(message_to_ordereddict(scene),indent=2))
  def measured_state():
   rclpy.spin_once(node,timeout_sec=.03)
   if not joints:raise RuntimeError('missing live joint states')
   latest=joints[-1];ns=latest.header.stamp.sec*10**9+latest.header.stamp.nanosec
   age=(node.get_clock().now().nanoseconds-ns)*1e-9
   if ns<=0 or not 0<=age<=.25:raise RuntimeError('joint state is not fresh: '+str(age))
   result=copy.deepcopy(scene.robot_state);result.joint_state=copy.deepcopy(latest)
   return result,age
  state,start_age=measured_state()
  (a.output/'measured_start.json').write_text(json.dumps(message_to_ordereddict(state),indent=2))
  rows=[]
  targets={f'astribot_arm_right_joint_{i+1}':v for i,v in enumerate([0.,0.,.6,1.,0.,0.,0.])}
  goal=copy.deepcopy(state)
  for i,name in enumerate(goal.joint_state.name):
   if name in targets:goal.joint_state.position[i]=targets[name]
  checks={}
  for name,s in [('start',state),('goal',goal)]:
   q=GetStateValidity.Request();q.robot_state=s;q.group_name='arm_right';res=call(valid_client,q,10);checks[name]=message_to_ordereddict(res)
  for index in range(3):
   current,age=measured_state()
   q=GetMotionPlan.Request();r=q.motion_plan_request;r.group_name='arm_right';r.planner_id='RRTConnectConfig';r.num_planning_attempts=1;r.allowed_planning_time=5.;r.max_velocity_scaling_factor=.3;r.max_acceleration_scaling_factor=.3;r.start_state=current
   c=Constraints();c.name='transport_compact_right'
   for name,value in targets.items():
    j=JointConstraint();j.joint_name=name;j.position=value;j.tolerance_above=.001;j.tolerance_below=.001;j.weight=1.;c.joint_constraints.append(j)
   r.goal_constraints=[c]
   started=time.monotonic();res=call(plan_client,q,15).motion_plan_response
   row=dict(repetition=index,input_joint_age_sec=age,error_code=res.error_code.val,wall_seconds=time.monotonic()-started,planning_seconds=res.planning_time,points=len(res.trajectory.joint_trajectory.points))
   rows.append(row);(a.output/f'plan_{index}.json').write_text(json.dumps(message_to_ordereddict(res),indent=2))
  report=dict(scope='live measured start and MoveIt snapshot; plan-only service, no execution; not MTC/transport acceptance',target_name='transport_compact',target=targets,state_checks=checks,world_object_count=len(scene.world.collision_objects),octomap_bytes=len(scene.world.octomap.octomap.data),scene_state_stamp=message_to_ordereddict(state.joint_state.header),results=rows)
  (a.output/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
 finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
