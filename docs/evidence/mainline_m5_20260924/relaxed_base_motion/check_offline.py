#!/usr/bin/env python3
"""Targeted execution of private script policy blocks without ROS."""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import textwrap
from types import SimpleNamespace as NS

here=Path(__file__).resolve().parent
base=Path('/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring')
source=(base/'verify_full_transfer.py').read_text()
reader=(base/'read_full_executor_parameters.py').read_text()
old=(here/'verify_full_transfer.py.before').read_text()
ast.parse(source);ast.parse(reader)
for begin,end in [('    def stop():','    def release():'),('        # Prove returned/adopted/sent suffix identity',"        report['final_stop']=stop()")]:
    assert source[source.index(begin):source.index(end,source.index(begin))]==old[old.index(begin):old.index(end,old.index(begin))]
parser=argparse.ArgumentParser()
exec(next(line.strip() for line in source.splitlines() if "parser.add_argument('--relax-base-motion'" in line))
assert parser.parse_args([]).relax_base_motion is False
assert parser.parse_args(['--relax-base-motion']).relax_base_motion is True
policy=compile(textwrap.dedent(source[source.index("            if phase!='NAVIGATE' and not pending_binding:"):source.index('    def spin(')]),'production_health','exec')
assignment=next(line.strip() for line in source.splitlines() if line.strip().startswith('non_navigation_angular_speed_limit='))
results=[]
for name,relaxed,vx,wz,command,phase,want in [
    ('strict_below',False,0.,.029,0.,'PICK',None),
    ('strict_reject',False,0.,.04,0.,'PICK','UNEXPECTED_MOTION_OUTSIDE_NAVIGATION'),
    ('relaxed_accept',True,0.,.09,0.,'PLACE',None),
    ('relaxed_reject',True,0.,.101,0.,'PICK','UNEXPECTED_MOTION_OUTSIDE_NAVIGATION'),
    ('linear_unchanged',True,.021,0.,0.,'PICK','UNEXPECTED_MOTION_OUTSIDE_NAVIGATION'),
    ('command_unchanged',True,0.,0.,.001,'PICK','NONZERO_CHASSIS_COMMAND_OUTSIDE_NAVIGATION'),
    ('navigation_unchanged',False,.1,.2,0.,'NAVIGATE',None),
]:
    env=dict(args=NS(relax_base_motion=relaxed),math=math,phase=phase,pending_binding=False,c=dict(vx=command,vy=0.,wz=0.),latest=dict(odom=dict(vx=vx,vy=0.,wz=wz)))
    exec(assignment,env);actual=None
    try:exec(policy,env)
    except RuntimeError as error:actual=str(error)
    assert actual==want,(name,actual,want)
    results.append(dict(name=name,passed=True,observed_error=actual))
begin=source.index('        # Bind this acceptance condition')
end=source.index("        event('parent_submission'",begin)
readback=compile(textwrap.dedent(source[begin:end]),'production_pre_goal_readback','exec')
for name,flag,ev,gv,want in [
    ('strict_readback',False,NS(type=1,bool_value=False),NS(type=1,bool_value=False),None),
    ('relaxed_readback',True,NS(type=1,bool_value=True),NS(type=1,bool_value=True),None),
    ('executor_mismatch',True,NS(type=1,bool_value=False),NS(type=1,bool_value=True),'SIMULATION_BASE_MOTION_CONDITION_MISMATCH:task_trajectory_executor'),
    ('guard_mismatch',True,NS(type=1,bool_value=True),NS(type=1,bool_value=False),'SIMULATION_BASE_MOTION_CONDITION_MISMATCH:manipulation_execution_guard'),
    ('undeclared_not_false',False,NS(type=0,bool_value=False),NS(type=1,bool_value=False),'SIMULATION_BASE_MOTION_CONDITION_MISMATCH:task_trajectory_executor'),
]:
    values={'/task_trajectory_executor/get_parameters':ev,'/manipulation_execution_guard/get_parameters':gv}
    env=dict(args=NS(relax_base_motion=flag),report=dict(base_motion_conditions={}),executor_params='/task_trajectory_executor/get_parameters',node=NS(create_client=lambda service,path:path),GetParameters=NS(Request=lambda **kw:kw),read_model_parameters=lambda client,request,spin,events:NS(values=[values[client]]),message_dict=vars,events=[])
    actual=None
    try:exec(readback,env)
    except RuntimeError as error:actual=str(error)
    assert actual==want,(name,actual,want)
    results.append(dict(name=name,passed=True,observed_error=actual))
begin=reader.index("        idle_hold = report['chassis_parameters']")
end=reader.index('        filter_names =',begin)
idle=compile(textwrap.dedent(reader[begin:end]),'production_idle_hold','exec')
for name,value,want in [('idle_enabled',dict(type=1,bool_value=True),None),('idle_disabled',dict(type=1,bool_value=False),'CHASSIS_IDLE_POSITION_HOLD_REQUIRED'),('idle_wrong_type',dict(type=0,bool_value=True),'CHASSIS_IDLE_POSITION_HOLD_REQUIRED')]:
    actual=None
    try:exec(idle,dict(report=dict(chassis_parameters=dict(idle_position_hold=value))))
    except RuntimeError as error:actual=str(error)
    assert actual==want,(name,actual,want)
    results.append(dict(name=name,passed=True,observed_error=actual))
report_begin=source.index("    report['base_motion_conditions']=dict")
report_end=source.index('    gc_events=',report_begin)
for flag in (False,True):
    env=dict(report={},args=NS(relax_base_motion=flag));exec(assignment,env)
    exec(textwrap.dedent(source[report_begin:report_end]),env)
    condition=env['report']['base_motion_conditions']
    assert condition['original_limits']==dict(rotation_rad=.02,angular_speed_radps=.03)
    assert condition['active_limits']==dict(rotation_rad=.05 if flag else .02,angular_speed_radps=.10 if flag else .03)
    assert condition['final_stop_thresholds_unchanged'] is True
    if flag:assert condition['acceptance_scope']=='relaxed_simulation_not_original_precision_acceptance'
out=dict(scope='Offline private script blocks with mocked parameter service; no ROS/build or simulation acceptance',cases=results,syntax=True,cli_flag_checked=True,condition_report_checked=True,stop_and_handoff_blocks_unchanged=True,hashes={name:hashlib.sha256((base/name).read_bytes()).hexdigest() for name in ('verify_full_transfer.py','read_full_executor_parameters.py')})
(here/'offline_results.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(dict(passed=len(results),failed=0,hashes=out['hashes'])))
