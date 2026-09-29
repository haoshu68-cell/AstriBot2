#!/usr/bin/env python3
"""Negative admission check: uses real geometry; publishes no hold or motion."""
import argparse,json,os,time,uuid
from pathlib import Path

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--session',required=True);a=p.parse_args()
 assert os.environ.get('ASTRIBOT_SIM_INSTANCE')==a.session and os.environ.get('ROS_DOMAIN_ID')!='25'
 assert os.environ.get('IGN_PARTITION')=='astribot_'+a.session
 a.output.mkdir(parents=True,exist_ok=False)
 import rclpy
 from rclpy.parameter import Parameter
 from astribot_navigation_msgs.msg import RobotGeometryState,RobotEnvelope
 from astribot_navigation_msgs.srv import SetFixedEnvelope,SetRobotEnvelope
 from geometry_msgs.msg import Twist
 from rosidl_runtime_py.convert import message_to_ordereddict
 rclpy.init();n=rclpy.create_node('verify_missing_hold',parameter_overrides=[Parameter('use_sim_time',value=True)])
 latest={};envelopes=[];commands=[];rows=[]
 subs=[n.create_subscription(RobotGeometryState,'/navigation/geometry_state',lambda m:latest.update(geometry=m),10),
       n.create_subscription(RobotEnvelope,'/navigation/robot_envelope',lambda m:envelopes.append(m),10),
       n.create_subscription(Twist,'/cmd_vel',lambda m:commands.append(m),10)]
 def wait(predicate,timeout=6.):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   rclpy.spin_once(n,timeout_sec=.01)
   if predicate():return
  raise AssertionError('missing required current evidence')
 def call(service,request):
  assert service.wait_for_service(timeout_sec=3)
  f=service.call_async(request);wait(f.done);value=f.result()
  rows.append(dict(request=message_to_ordereddict(request),response=message_to_ordereddict(value),received_ros=n.get_clock().now().nanoseconds))
  return value
 try:
  wait(lambda:latest.get('geometry') and latest['geometry'].complete and envelopes and commands)
  assert not n.get_publishers_info_by_topic('/navigation/arm_hold'),'hold already has an owner; do not test missing authority'
  f=n.create_client(SetFixedEnvelope,'/navigation/set_fixed_envelope')
  req=SetFixedEnvelope.Request();result=call(f,req)
  assert not result.accepted and result.reason=='REQUEST_ID_REQUIRED',result
  req.request_id='negative_'+uuid.uuid4().hex;req.hold_id='absent_'+uuid.uuid4().hex
  req.geometry_sequence=latest['geometry'].sequence;result=call(f,req)
  assert not result.accepted and result.reason=='ARM_HOLD_UNCONFIRMED',result
  legacy=n.create_client(SetRobotEnvelope,'/navigation/set_robot_envelope')
  request=SetRobotEnvelope.Request();request.envelope.transport_ready=True;result=call(legacy,request)
  assert not result.accepted and result.reason=='FIXED_V2_REQUIRES_GEOMETRY_AND_HOLD',result
  end=time.monotonic()+1
  while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
  assert all(not v.transport_ready for v in envelopes)
  maximum=max(abs(getattr(getattr(v,f),axis)) for v in commands for f in ('linear','angular') for axis in ('x','y','z'))
  assert maximum<1e-6
  report=dict(passed=True,checks=rows,geometry=message_to_ordereddict(latest['geometry']),
              envelope_samples=len(envelopes),command_samples=len(commands),max_command=maximum)
  (a.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='geometry'}))
 finally:
  (a.output/'responses.json').write_text(json.dumps(rows,indent=2));n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
