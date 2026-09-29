"""Installed executor exception check + literal frozen main; no ROS init/DDS."""
import ast,json,types
from pathlib import Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
p=Path('/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation_policy_native/test/reference/policy_observer/observer_node.py')
main=next(n for n in ast.parse(p.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='main')
events=[]
class OfflineExecutor:
    def __init__(self,**kw):
        self.future=Future();self.future.set_exception(AssertionError("The 'calibration_epoch' field must be an unsigned integer in [0, 18446744073709551615]"))
        self._executor=types.SimpleNamespace(submit=lambda handler:events.append('submit completed callback'))
        self._futures=[]
    def add_node(self,node):events.append('add stub node')
    def wait_for_ready_callbacks(self,*args):return self.future,None,None
    def spin(self):MultiThreadedExecutor._spin_once_impl(self)
    def shutdown(self):events.append('executor shutdown')
ns={'rclpy':types.SimpleNamespace(init=lambda:events.append('stub init'),shutdown=lambda:events.append('stub shutdown')),
    'PolicyObserver':lambda:types.SimpleNamespace(destroy_node=lambda:events.append('destroy stub node')),
    'MultiThreadedExecutor':OfflineExecutor}
exec(compile(ast.fix_missing_locations(ast.Module(body=[main],type_ignores=[])),str(p),'exec'),ns)
try:ns['main']();outcome='returned'
except Exception as e:outcome=type(e).__name__+': '+str(e)
print(json.dumps(dict(outcome=outcome,events=events),indent=2))
