import importlib.util,json,sys
from pathlib import Path
p=Path('/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation_policy_native/test/reference/policy_observer/contracts.py')
s=importlib.util.spec_from_file_location('_pixel_contract_review',p);c=importlib.util.module_from_spec(s);sys.modules[s.name]=c;s.loader.exec_module(c)
rows=[]
for name,width,xmax in [('uint64_max_float_edge',2**64-1,float(2**64)),('uint64_plus_float_edge',2**64,float(2**64)),('uint64_max_integer_edge',2**64-1,2**64-1),('huge_dimension_small_box',10**399,1.),('huge_coordinate',10**399,10**398)]:
    try:
        box=c.ImageBox('cam',width,480,0.,0.,xmax,1.)
        outcome='accepted'
    except Exception as e:outcome=type(e).__name__+': '+str(e)
    rows.append(dict(case=name,width=str(width),xmax=str(xmax),xmax_type=type(xmax).__name__,outcome=outcome))
print(json.dumps({'int_max_str_digits':sys.get_int_max_str_digits(),'cases':rows},indent=2))
