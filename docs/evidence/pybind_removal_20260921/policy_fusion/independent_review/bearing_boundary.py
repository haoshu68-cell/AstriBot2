import importlib.util,json,math,random,sys
from pathlib import Path
file=Path('/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation_policy_native/test/test_policy_fusion.py')
spec=importlib.util.spec_from_file_location('review_fusion_tests',file);t=importlib.util.module_from_spec(spec);spec.loader.exec_module(t)
r=random.Random(1946);ops=[]
for i in range(8000):
 v=[r.uniform(-1,1) for _ in range(3)];length=math.hypot(*v)
 scale=r.choice([1.-1e-6,1.+1e-6]);v=[x/length*scale for x in v]
 if i%3:v[r.randrange(3)]=math.nextafter(v[r.randrange(3)],r.choice([-math.inf,math.inf]))
 ops.append(dict(action='check_contract',type='BearingCone',value=dict(direction=v,half_angle_rad=.3)))
case=dict(operations=ops);a=t.reference(case);b=t.native(case)
for i,(left,right) in enumerate(zip(a,b)):
 if left!=right:
  Path('/tmp/review_policy_fusion/bearing_boundary_failure.json').write_text(json.dumps(dict(operations=[ops[i]]),indent=2)+'\n')
  print(i,ops[i],left,right);break
else:print('all',len(ops),'boundary contracts match')
