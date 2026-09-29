#!/usr/bin/env python3
"""Own an isolated canonical warehouse sensor validation session (no arm tasks)."""
import fcntl,json,os,shlex,signal,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from tools.sim_isolation import SimulationIsolation,is_stack_process
RUN=ROOT/'runs/grasp_pose_sim_20260921/session'

def start():
 RUN.mkdir(parents=True,exist_ok=True)
 overlay=ROOT/'runs/grasp_pose_sim_20260921/ros_ws/install/local_setup.bash'
 if not overlay.is_file():
  raise RuntimeError('Build the perception overlay first: tools/vision/build_manipulation_perception.sh '+str(overlay.parents[1]))
 iso=SimulationIsolation('grasp_pose_20260921',87)
 locks=[]
 for name in iso.lock_paths():
  f=open(name,'a+');fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB);locks.append(f)
 for proc in Path('/proc').iterdir():
  if not proc.name.isdigit():continue
  try:
   args=proc.joinpath('cmdline').read_bytes().decode().rstrip('\0').split('\0')
   if not is_stack_process(args):continue
   env=dict(x.split('=',1) for x in proc.joinpath('environ').read_bytes().decode().split('\0') if '=' in x)
   conflict=iso.conflict(env)
   if conflict:raise RuntimeError(f'{conflict}: pid {proc.name}')
  except (FileNotFoundError,PermissionError,ProcessLookupError):pass
 env=os.environ.copy();env.update(iso.environment())
 env.update(ASTRIBOT_LOG_DIR=str(RUN),ROS_LOG_DIR=str(RUN/'ros'),ASTRIBOT_LOG_CAPTURE='1')
 # ABI 0.3 perception consumers and transport config must come from the same
 # durable overlay. Description remains the source-linked robot asset package.
 cmd=f'''source /opt/ros/humble/setup.bash
source {ROOT}/ws_robot/install/setup.bash
source {ROOT}/install/setup.bash
source {shlex.quote(str(overlay))}
export AMENT_PREFIX_PATH={ROOT}/ws_robot/install/astribot_s1_description:$AMENT_PREFIX_PATH
exec ros2 launch {ROOT}/ws_robot/src/astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py ros_domain_id:=87 headless:=true use_rviz:=false use_lidar:=false use_camera:=true use_camera_postprocess:=false use_camera_pointcloud:=false enable_effort_drive:=false camera_profile:={ROOT}/ws_robot/src/astribot_s1_description/config/camera_head_rgbd_nav_sim.yaml torso_camera_profile:={ROOT}/ws_robot/src/astribot_s1_description/config/camera_torso_rgbd_nav_sim.yaml'''
 with open(RUN/'session.log','w') as log:
  p=subprocess.Popen(['bash','-c',cmd],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  identity={'supervisor_pid':os.getpid(),'launch_pid':p.pid,'launch_start_ticks':Path(f'/proc/{p.pid}/stat').read_text().split()[21],'started_wall':time.time(),'env':iso.environment(),'session_log':str(RUN/'session.log'),'command':cmd,'scope':'canonical warehouse sensor/physics only; no navigation or arm task'}
  (RUN/'session.json').write_text(json.dumps(identity,indent=2))
  (RUN/'query_env.sh').write_text('source /opt/ros/humble/setup.bash\nsource '+str(ROOT)+'/ws_robot/install/setup.bash\nsource '+str(ROOT)+'/install/setup.bash\nsource '+shlex.quote(str(overlay))+'\n'+''.join('export '+k+'='+shlex.quote(v)+'\n' for k,v in iso.environment().items()))
  def stop(signum,_):
   if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
  signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
  rc=p.wait();identity.update(exit_code=rc,stopped_wall=time.time());(RUN/'session.json').write_text(json.dumps(identity,indent=2))
 return rc
if __name__=='__main__':sys.exit(start())
