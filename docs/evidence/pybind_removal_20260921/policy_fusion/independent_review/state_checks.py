import importlib.util,json,random,math,sys
from pathlib import Path
file=Path('/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation_policy_native/test/test_policy_fusion.py')
spec=importlib.util.spec_from_file_location('fusion_review_state',file);t=importlib.util.module_from_spec(spec);spec.loader.exec_module(t)
r=random.Random(543821);total=0
for scenario in range(100):
 ns=r.choice([0,2_147_483_547_000_000_000]);epoch=0;ops=[]
 for step in range(80):
  ns=max(0,ns+r.choice([-100000000,0,1,10000000,100000000,600000000]))
  if step%19==0:epoch=r.randrange(4)
  now=t.stamp(ns,epoch=epoch)
  action=r.randrange(7)
  if action<3:
   obs=[]
   for index in range(r.randrange(5)):
    capture=max(0,ns-r.choice([-1,0,1,500000000,500000001]))
    item=t.observation(capture,str(r.randrange(12)),sensor=r.choice(['scan','camera']),source=r.choice([None,'a','b']),
      center=(r.uniform(-2,2),r.uniform(-2,2),.5),capture_stamp=t.stamp(capture,epoch=epoch),valid_until=t.stamp(capture+500000001,epoch=epoch),
      geometry_quality=r.choice([0.,1.]),provenance=[r.choice(['p','q','r'])+str(r.randrange(5))])
    if r.random()<.12:item['frame_id']='map'
    if r.random()<.2:item['geometry']=dict(kind='BearingCone',direction=[1.,0.,0.],half_angle_rad=.2)
    obs.append(item)
   ops.append(dict(action=r.choice(['ingest','update']),now=now,observations=obs))
  elif action==3:ops.append(dict(action='clear',now=now,flags=[r.choice([True,False]) for _ in range(r.randrange(4))]))
  elif action==4:ops.append(dict(action='resolve',now=now,capture=t.stamp(max(0,ns-r.choice([0,1,500000000,500000001])),epoch=epoch),sensor_id=r.choice(['scan','camera']),measurement_ids=[str(r.randrange(12)) for _ in range(3)]))
  elif action==5:ops.append(dict(action='set_envelope_bounds',bounds=[r.uniform(.31,2.),r.uniform(.31,2.)]))
  else:ops.append(dict(action='snapshot',now=now,region=[r.uniform(-3,3),r.uniform(-3,3),r.uniform(0,3)]))
  ops.append(dict(action='snapshot',now=now))
 case=dict(operations=ops)
 try:t.compare(t.reference(case),t.native(case))
 except Exception:
  Path('/tmp/review_policy_fusion/state_failure.json').write_text(json.dumps(case,indent=2)+'\n');raise
 total+=len(ops)
maximum=2_147_483_647_999_999_999;capture=maximum-500000001
case=dict(operations=[t.update(maximum-1,[t.observation(capture,valid_until=t.stamp(maximum))]),t.snapshot(maximum),t.snapshot(0),t.update(0,[t.observation(0,'rollback')]),t.snapshot(0)])
t.compare(t.reference(case),t.native(case));total+=len(case['operations'])
Path('/tmp/review_policy_fusion/state_review_result.json').write_text(json.dumps(dict(status='passed',scenarios=101,operations=total,seed=543821,ros_maximum_ns=maximum),indent=2)+'\n')
print('PASS',total,'operations across101stateful scenarios incl legal ROS time max')
