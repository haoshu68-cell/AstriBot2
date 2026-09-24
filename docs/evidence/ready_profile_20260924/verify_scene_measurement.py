"""Replay the real scene03 serialization boundary; no live state is asserted."""
import ast,copy,json,math,hashlib
from pathlib import Path
import yaml
base=Path(__file__).resolve().parent
old=Path('/home/yjh/WorkSpace/astribot_validation/M1_scene_prepare_20260924_1024/first_scene03')
def require(value,reason):
    if not value:raise RuntimeError(reason)
source=base/'prepare_scene.py'
node=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='check_initial_measurement')
ns={'math':math,'require':require};exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),ns)
check=ns['check_initial_measurement']
result_path=old/'first_stage/result.json';scene_path=old/'fixtures/scene_readback.json'
record=json.loads(result_path.read_text())['jtc_records'][0]['message']
actual=dict(header=record['header'],name=record['joint_names'],position=record['feedback']['positions'],velocity=record['feedback']['velocities'])
scene=json.loads(scene_path.read_text())['robot_state']['joint_state']
profile=Path('/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_description/config/sim_initial_transport_ready.yaml')
names=actual['name']
values=dict(zip(scene['name'],scene['position']));expected={n:values[n] for n in names}
at=actual['header']['stamp'];now=at['sec']*10**9+at['nanosec']+10_000_000
assert scene['header']['stamp']=={'sec':0,'nanosec':0} and scene['velocity']==[]
check(actual,10.,10.01,now,expected,scene)
rows=[dict(name='actual_joints_accept_zero_stamp_empty_velocity_scene',passed=True)]
for name,sample,clock,why in [('missing_actual_velocity',dict(actual,velocity=[]),now,'INCOMPLETE'),('stale_actual_stamp',actual,now+300_000_000,'STALE')]:
    try:check(sample,10.,10.01,clock,expected,scene)
    except RuntimeError as e: assert why in str(e);rows.append(dict(name=name,passed=True,reason=str(e)))
    else:raise AssertionError(name+' incorrectly passed')
report=dict(passed=True,evidence='offline replay of seven real scene03 left JTC feedback joints and full scene serialization; synthetic check time/receipt only, not a 22-joint live acceptance',cases=rows,scene_header=scene['header'],scene_velocity=scene['velocity'],inputs={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (source,result_path,scene_path,profile)})
(base/'scene_measurement_result.json').write_text(json.dumps(report,indent=2)+'\n')
print('3/3 PASS; retained scene stamp=0 and velocity=[]; actual missing velocity/stale source rejected')
