#!/usr/bin/env python3
"""Check live planner model identity and request READY_RIGHT planning only."""
import argparse,hashlib,json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from astribot_transport_msgs.srv import PlanSkill
from rosidl_runtime_py.convert import message_to_ordereddict
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
rclpy.init();node=Node('readonly_transport_skill_probe')
def call(topic,typ,req,timeout=30):
 c=node.create_client(typ,topic)
 if not c.wait_for_service(timeout_sec=20):raise RuntimeError('missing '+topic)
 f=c.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=timeout)
 if not f.done():raise RuntimeError('timeout '+topic)
 return f.result()
try:
 check=call('/move_group/get_parameters',GetParameters,GetParameters.Request(names=['allow_trajectory_execution']))
 assert check.values[0].type==1 and not check.values[0].bool_value
 models={}
 for name in ('move_group','transport_skill_planner','transport_mtc_planner'):
  print('reading model:',name,flush=True)
  values=call('/'+name+'/get_parameters',GetParameters,GetParameters.Request(names=['robot_description','robot_description_semantic'])).values
  assert len(values)==2 and all(v.type==4 and v.string_value for v in values)
  models[name]=[hashlib.sha256(v.string_value.encode()).hexdigest() for v in values]
  (a.output/'model_hashes.json').write_text(json.dumps(models,indent=2))
  print('model received:',name,[len(v.string_value) for v in values],flush=True)
 assert len({tuple(v) for v in models.values()})==1,models
 q=PlanSkill.Request();q.operation='named';q.group='arm_right';q.named_target='transport_compact'
 rows=[]
 for i in range(3):
  print('planning attempt:',i,flush=True)
  r=call('/transport/plan_skill',PlanSkill,q,60)
  (a.output/f'plan_{i}.json').write_text(json.dumps(message_to_ordereddict(r),indent=2))
  rows.append(dict(repetition=i,success=r.success,reason=r.reason,points=len(r.trajectory.joint_trajectory.points)))
 report=dict(scope='live shared-model service planning; no execution, object attachment or MTC task run',model_hashes=models,results=rows)
 (a.output/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
 assert all(row['success'] and row['points']>0 for row in rows),report
finally:node.destroy_node();rclpy.shutdown()
