import importlib.util,json
from pathlib import Path
file=Path('/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation_policy_native/test/test_policy_fusion.py')
spec=importlib.util.spec_from_file_location('review_fusion_tests',file);t=importlib.util.module_from_spec(spec);spec.loader.exec_module(t)
for name,x,y,shared in [('association',.3653736529718966,.34132403037871817,False),('provenance',-.035354137271507,.03535654080629834,True)]:
 first=t.observation(0,'first',sensor='a',source=None)
 second=t.observation(0,'second',sensor='b',source=None,center=(x,y,.5),**({'provenance':['a:first']} if shared else {}))
 case=dict(operations=[t.update(0,[first,second])])
 a=t.reference(case);b=t.native(case)
 Path('/tmp/review_policy_fusion/'+name+'_boundary_case.json').write_text(json.dumps(case,indent=2)+'\n')
 Path('/tmp/review_policy_fusion/'+name+'_boundary_result.json').write_text(json.dumps(dict(reference=a,native=b),indent=2)+'\n')
 print(name,'reference track count',len(a[0]['result']['tracks']),'native track count',len(b[0]['result']['tracks']))
 for label,r in [('python',a),('native',b)]:
  print(label,[(track['fused_track_id'],track['geometry']['center_m'],track['geometry']['size_m'],track['provenance']) for track in r[0]['result']['tracks']])
