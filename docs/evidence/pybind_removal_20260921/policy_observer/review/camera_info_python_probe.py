"""Actual generated CameraInfo plus frozen contracts; no ROS init/node/graph."""
import importlib.util
from pathlib import Path
import sys
from sensor_msgs.msg import CameraInfo

reference=Path('/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation_policy_native/test/reference/policy_observer/contracts.py')
spec=importlib.util.spec_from_file_location('_camera_wire_contracts',reference)
c=importlib.util.module_from_spec(spec);sys.modules[spec.name]=c;spec.loader.exec_module(c)
m=CameraInfo(width=640,height=480,k=[500.,0.,320.,0.,500.,240.,0.,0.,1.],distortion_model='plumb_bob',d=[0.,0.,0.,0.,0.])
m.header.frame_id='camera_optical'
print('K container:',type(m.k).__module__,type(m.k).__name__)
print('K element types:',[(type(v).__module__,type(v).__name__) for v in tuple(m.k)])
print('D element types:',[(type(v).__module__,type(v).__name__) for v in tuple(m.d)])
for desc,values in [('actual CameraInfo fields',tuple(m.k)),('explicit Python float conversion',tuple(float(v) for v in m.k))]:
    try:
        candidate=c.CameraCalibration('cam',m.header.frame_id,0,m.width,m.height,values,m.distortion_model,tuple(m.d),None)
        print(desc,'ACCEPTED',candidate.calibration_epoch)
    except Exception as e:print(desc,type(e).__name__,str(e))
