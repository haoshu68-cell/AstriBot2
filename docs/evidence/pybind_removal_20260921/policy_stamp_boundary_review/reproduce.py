#!/usr/bin/env python3
"""Offline authority/candidate phase audit. No ROS node, graph or hardware."""
import argparse,hashlib,importlib.util,json,os,subprocess,sys,types
from pathlib import Path
from rclpy.time import Time
parser=argparse.ArgumentParser();parser.add_argument('--health-probe',required=True);parser.add_argument('--adapter-probe',required=True);args=parser.parse_args()
R=Path(__file__).resolve().parents[4];T=R/'ws_robot/src/astribot_s1_navigation_policy_native/test'
sys.path.insert(0,str(T));os.environ['POLICY_OBSERVATION_ADAPTERS_PROBE']=args.adapter_probe
import test_policy_health as H
import test_policy_observation_adapters as A
name='_stamp_profile_authority';pkg=types.ModuleType(name);pkg.__path__=[str(T/'reference/policy_observer')];sys.modules[name]=pkg
P=__import__(name+'.profile',fromlist=['Profile']).Profile
base=json.loads((T/'reference/policy_observer/simulation.json').read_text());result={'scope':'offline constructors/core operations/installed Time only; no live executor','timeouts':[],'adapters':[]}
for timeout in (.5,1e10,1e100,1e300):
 row={'timeout_s':timeout}
 try:P(dict(base,sensor_timeout_s=timeout));row['python_profile']='accepted'
 except Exception as e:row['python_profile']={'error':type(e).__name__,'message':str(e)}
 try:
  health=H.oracle.SensorHealthRegistry(timeout);row['python_registry_constructor']='accepted'
  item=health.health(H.contracts.Stamp(1_000_000_000,'ros',0))[0];row['python_expiry_ns']=item.valid_until.ns
  try:row['python_time_output']=repr(Time(nanoseconds=item.valid_until.ns).to_msg())
  except Exception as e:row['python_time_output']={'error':type(e).__name__,'message':str(e)}
 except Exception as e:row['python_registry_constructor']={'error':type(e).__name__,'message':str(e)}
 case={'timeout':timeout,'ops':[H.query(1_000_000_000)]};p=subprocess.run([args.health_probe],input=json.dumps(case)+'\n',text=True,capture_output=True)
 row['native']={'exit_code':p.returncode,'stdout':p.stdout,'stderr':p.stderr};result['timeouts'].append(row)
fixture=A.authority.__wrapped__();authority=next(fixture)
try:
 image=dict(kind='image_box',measurement_id='image',image_size_px=[100,100],box_xyxy_px=[0.,0.,10.,10.],geometry_quality=1.,provenance=['image'])
 for label,capture,timeout in [('expiry_crosses_int64',2**63-500_000_000,.5),('image_capture_above_int64',2**63,.5),('negative_capture_below_int64',-(2**63)-1,.5),('ordinary_capture_large_timeout',1_000_000_000,1e10)]:
  packet=A.vision(stamp_ns=capture,observations=[image]);case={'timeout':timeout,'operations':[{'data':json.dumps(packet)}]}
  result['adapters'].append({'case':label,'input':case,'python':A.reference(case,authority),'native':A.native(case)})
finally:
 try:next(fixture)
 except StopIteration:pass
result['binary_sha256']={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in (args.health_probe,args.adapter_probe)}
print(json.dumps(result,indent=2))
