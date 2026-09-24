"""Owned M1 actual PREGRASP and measured Hold validation, no navigation."""
import argparse,hashlib,json,os,shlex,signal,subprocess,sys,time
from pathlib import Path
R=Path('/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_navigation_20260923_155154');E=Path(__file__).resolve().parent
I=R.parent/'I0_2_20260923_065115';ROOT=I/'source_repo';OVERLAY=E/'overlay.bash'
W=Path('/home/yjh/WorkSpace/astribot_sdk_ros2')
sys.path.insert(0,str(ROOT/'tools/social_navigation'))
from run_regression import runtime_environment,read_json,save,stop_owned,descendants
from sim_isolation import SimulationIsolation
from sim_camera_preset import DEFAULT_RELATIVE_PRESET
from owned_command import run_owned_capture
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--case',required=True)
parser.add_argument('--executor',type=Path,default=W/'runs/m1_transport_20260924/install/lib/astribot_s1_transport_native/trajectory_executor')
parser.add_argument('--executor-sha256',default='e3331a0d6f4b43fe6991774cdc67d56ab1bd860105c504e45e70ccfadf11284b')

options=parser.parse_args()
if not options.case.replace('_','').isalnum():raise RuntimeError('invalid case identifier')
b=E/options.case;b.mkdir(exist_ok=False)
for name in ('run_first_stage.py','prepare_scene.py','verify_first_stage.py','overlay.bash'):
 (b/('used_'+name)).write_bytes((E/name).read_bytes())
isolation=SimulationIsolation('ready_first_stage_20260924',90)
env=os.environ.copy();env.update(ASTRIBOT_SIM_INITIAL_JOINT_PROFILE='transport_ready',ASTRIBOT_SCAN_TIMING_DIAGNOSTICS='1',ASTRIBOT_POLICY_NATIVE_TF='1',DISPLAY=':1',ASTRIBOT_OVERLAY_SETUP=str(OVERLAY),EGL_PLATFORM='surfaceless',LIBGL_ALWAYS_SOFTWARE='0',__GLX_VENDOR_LIBRARY_NAME='nvidia',__EGL_VENDOR_LIBRARY_FILENAMES='/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
launch=moveit=hold=runner=bridge=recorder=observer=truth=None;prefix='';files=[];handles={};report=dict(passed=False,started_wall=time.time(),initial_joint_profile='transport_ready',zero_start_motion_validated=False)
def interrupted(sig,frame):raise KeyboardInterrupt('owned fixed batch interrupted')
signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
def spawn(name,command):
 f=(b/(name+'.log')).open('w');files.append(f)
 p=subprocess.Popen(command,cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
 time.sleep(.05)
 if p.poll() is None:
  proc=Path('/proc')/str(p.pid)
  save(b/(name+'_identity.json'),dict(pid=p.pid,start_ticks=proc.joinpath('stat').read_text().rsplit(') ',1)[1].split()[19],exe=str(proc.joinpath('exe').resolve()),cmdline=proc.joinpath('cmdline').read_bytes().decode().rstrip('\0').split('\0'),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()))
  handles[p.pid]=os.pidfd_open(p.pid)
 return p
def child(name,arguments):return spawn(name,['bash','-c',prefix+'exec '+shlex.join([str(x) for x in arguments])])
def run(name,args,timeout):
 global runner
 runner=child(name,args);runner.wait(timeout=timeout)
 if runner.returncode:raise RuntimeError(name+' exit '+str(runner.returncode))
 runner=None
try:
 command=['bash',str(ROOT/'tools/launch_sim_stack.sh'),'--mode','baseline','--instance',isolation.instance,'--ros-domain-id','90','--navigation-geometry-mode','fixed_v2','--navigation-policy','p4','--payload-source-id','gazebo_kinematic_v1','--corridor-file',str(R/'N4_empty_corridors.json'),'--social-policy','off','--nav-transport','udp','--camera-preset',str(ROOT/DEFAULT_RELATIVE_PRESET),'--headless','--no-rviz','--exclusive-performance','--ready-timeout','180','--log-dir',str(b/'stack')]
 launch=spawn('launcher',command)
 deadline=time.monotonic()+200
 while launch.poll() is None and time.monotonic()<deadline:
  session=read_json(b/'stack/session.json')
  if session.get('state')=='ready':break
  time.sleep(.5)
 else:raise RuntimeError('fixed supervisor not ready')
 if session['supervisor_pid']!=launch.pid:raise RuntimeError('owner mismatch')
 prefix=runtime_environment(launch.pid,b,OVERLAY,isolation)
 # Read actual mappings before any navigation goal; selecting an overlay alone is insufficient.
 expected_library=Path('/home/yjh/WorkSpace/astribot_sdk_ros2/runs/task_chain_20260922/gz_control_fix/install/gz_ros2_control/lib/libgz_ros2_control-system.so').resolve()
 expected_sha='bea2defb55a928490f68ace9900302c7560af6ba85270d04f3c14b40e8d5bfca'
 loaded=[]
 for pid in descendants(launch.pid):
  try:
   paths={Path(line.split()[-1]) for line in Path(f'/proc/{pid}/maps').read_text().splitlines() if 'libgz_ros2_control-system.so' in line and line.split()[-1].startswith('/')}
   for path in paths:loaded.append(dict(pid=pid,path=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),start_ticks=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]))
  except (OSError,ValueError):continue
 save(b/'gz_control_runtime_manifest.json',dict(expected=str(expected_library),expected_sha256=expected_sha,loaded=loaded))
 if len(loaded)!=1 or loaded[0]['path']!=str(expected_library) or loaded[0]['sha256']!=expected_sha:raise RuntimeError('BOUNDED_URDF_RUNTIME_IDENTITY_MISMATCH')
 libraries=read_json(b/'runtime_manifest.json')['libraries']
 if libraries.get(str(I/'install/astribot_s1_path_tracking/lib/libastribot_s1_path_tracking.so'))!='e9e483fc68194089e09ea34397e4553d46c19893629befde33ace5f64254ab78':raise RuntimeError('ARRIVAL_DIAGNOSTIC_RUNTIME_IDENTITY_MISMATCH')
 startup_log=(b/'stack/session.log').read_text(errors='replace')
 bounded_lines=[line for line in startup_log.splitlines() if 'bounded request(s)' in line or 'bounded retry' in line]
 save(b/'bounded_urdf_log.json',dict(lines=bounded_lines))
 if not any('Received URDF after' in line for line in bounded_lines):raise RuntimeError('BOUNDED_URDF_LOG_UNOBSERVED')
 if session['camera_baseline']['sha256']!='8942e6b0952a982ad3edb3b9fb8aa293f0c528a06ea6a720f3e145056cdd625f':raise RuntimeError('SIX_CAMERA_BASELINE_CHANGED')
 run('capture_owner' ,['python3',W/'tools/sim/capture_supervisor_owner.py','--pid',launch.pid,'--session',isolation.instance,'--source','gazebo_kinematic_v1','--output',b/'owner.json'],10)
 from ament_index_python.packages import get_package_share_directory,get_package_prefix
 desc=Path(get_package_share_directory('astribot_s1_description'));config=desc/'config';camera=config/'simulation_navigation_full'
 moveit=child('transport_skills',['ros2','launch',str(W/'ws_robot/src/astribot_s1_transport/launch/transport_skills.launch.py'),'allow_trajectory_execution:=false','use_camera:=true','use_wrist_cameras:=true','use_stereo_cameras:=true','use_lidar:=true','camera_profile:='+str(camera/'camera_head_rgbd.yaml'),'torso_camera_profile:='+str(camera/'camera_torso_rgbd.yaml'),'camera_calibration_dir:='+str(camera),'camera_mounts_profile:='+str(config/'camera_mounts_reference_sim.yaml')])
 executor=options.executor.resolve()
 executor_sha=options.executor_sha256
 if hashlib.sha256(executor.read_bytes()).hexdigest()!=executor_sha:raise RuntimeError('EXECUTOR_HASH_CHANGED')
 hold=child('trajectory_executor',[str(executor),'--ros-args','-p','use_sim_time:=true','-p','simulation_commissioning:=true'])
 worlds=[]
 for pid in descendants(launch.pid):
  try:
   args=Path(f'/proc/{pid}/cmdline').read_bytes().decode().rstrip('\0').split('\0')
   if len(args)==1 and args[0].startswith('ign gazebo '): args=shlex.split(args[0])
   if args[:2]==['ign','gazebo'] and '-s' in args:
    for a in args:
     p=Path(a)
     if a.endswith(('.sdf','.world')) and p.is_file():worlds.append(str(p.resolve()))
  except OSError:pass
 worlds=sorted(set(worlds))
 if len(worlds)!=1:raise RuntimeError('actual Gazebo world ambiguous: '+str(worlds))
 save(b/'actual_world.json',dict(path=worlds[0],sha256=hashlib.sha256(Path(worlds[0]).read_bytes()).hexdigest()))
 plugin=Path(get_package_prefix('astribot_s1_gazebo_bringup'))/'lib'
 deadline=time.monotonic()+300
 while not (E/'prepare_scene.py').exists() and time.monotonic()<deadline:time.sleep(.25)
 run('prepare_scene',['python3',E/'prepare_scene.py','--owner',b/'owner.json','--session',isolation.instance,'--source','gazebo_kinematic_v1','--plugin-directory',plugin,'--world-reference',worlds[0],'--output',b/'fixtures'],120)
 report['fixture_directory']=str(b/'fixtures')
 bridge=child('overview_bridge',[str(Path(get_package_prefix('ros_gz_bridge'))/'lib/ros_gz_bridge/parameter_bridge'),'/transport/overview@sensor_msgs/msg/Image[ignition.msgs.Image','--ros-args','-p','use_sim_time:=true'])
 recorder=child('recorder',['python3',W/'tools/sim/record_transport_demo.py','--output',b/'first_stage.mp4','--head-topic','/camera/raw/head_rgbd/image','--status-topic','/transport/hold_executor/status','--status-format','native_hold','--title','ASTRIBOT | M1 PREGRASP to measured Hold','--timeout','240'])
 time.sleep(3)
 truth=child('world_poses',['ign','topic','-e','-t','/world/default/pose/info','-d','24'])
 try:
  run('head_snapshot',['python3',W/'tools/vision/sim_pose_capture.py','--camera','head_rgbd','--seconds','6','--truth-stream',b/'world_poses.log','--output',b/'head_snapshot'],25)
  report['head_snapshot']='captured; ROI and coverage not evaluated'
 except RuntimeError as error:
  report['head_snapshot_error']=str(error)
  runner=None
 sys.path.insert(0,str(W/'tools/sim'))
 from capture_supervisor_owner import capture
 current_executor=capture(hold.pid)
 if Path(current_executor['exe']).resolve()!=executor.resolve():raise RuntimeError('EXECUTOR_NOT_STARTED')
 save(b/'executor_owner.json',current_executor)
 component_images=[]
 for pid in [hold.pid,moveit.pid,*descendants(moveit.pid)]:
  try:
   identity=capture(pid);paths={line.split()[-1] for line in Path(f'/proc/{pid}/maps').read_text().splitlines() if line.split()[-1].startswith('/home/yjh/') and any(key in line for key in ('astribot_s1_manipulation','astribot_s1_transport_mtc','astribot_s1_transport_native'))}
   component_images.append(dict(identity=identity,executable_sha256=hashlib.sha256(Path(identity['exe']).read_bytes()).hexdigest(),libraries={path:hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in sorted(paths)}))
  except (FileNotFoundError,ProcessLookupError):continue
 save(b/'manipulation_runtime_manifest.json',component_images)
 identities={}
 for pid in descendants(launch.pid):
  try:
   identity=capture(pid);name=Path(identity['exe']).name
   if name in ('bt_navigator','task_arbiter_cpp','operator_backend','loop_route_executor','waypoint_follower'):
    if name in identities:raise RuntimeError('ambiguous navigation process '+name)
    identities[name]=identity
  except (FileNotFoundError,ProcessLookupError):pass
 save(b/'cold_start.json',dict(owner=read_json(b/'owner.json'),session=isolation.instance,captured_wall=time.time(),no_preceding_goal_writers=True,processes=identities,probe_output=str((b/'first_stage').resolve())))
 run('first_stage',['python3',E/'verify_first_stage.py','--owner',b/'owner.json','--session',isolation.instance,'--source','gazebo_kinematic_v1','--cold-start-receipt',b/'cold_start.json','--expected-binary-sha256','7848ef4eef5bf46512e6296e53db9b4416df8f076f49335a3c128f756be68794','--expected-executor-sha256',executor_sha,'--profile',ROOT/'ws_robot/src/astribot_s1_navigation_policy/config/simulation.json','--output',b/'first_stage'],220)
 report['passed']=read_json(b/'first_stage/result.json').get('passed',False)
 save(b/'stage_finished.json',dict(owner=read_json(b/'owner.json'),prefix=prefix,report=report))
 time.sleep(20)

except BaseException as error:report['error']=repr(error);report['passed']=False
finally:
 if runner is not None:
  try:stop_owned(runner,40)
  except Exception as error:report['runner_cleanup_error']=str(error)
 # This focused return regression needs no additional visual matrix.
 errors=[]
 for name,process,timeout in [('recorder',recorder,15),('truth',truth,5),('observer',observer,10),('overview_bridge',bridge,10),('trajectory_executor',hold,15),('transport_skills',moveit,20),('supervisor',launch,45)]:
  try:stop_owned(process,timeout)
  except BaseException as error:errors.append(name+': '+repr(error))
 session=read_json(b/'stack/session.json')
 report['cleanup_complete']=bool(launch is not None and launch.poll() is not None and session.get('state') in ('stopped','failed') and session.get('ended') and session.get('remaining_owned_pids')==[] and not session.get('log_capture_errors') and not errors)
 if errors:report['cleanup_errors']=errors
 if not report['cleanup_complete']:report['passed']=False
 for fd in handles.values():os.close(fd)
 for f in files:f.close()
 report['finished_wall']=time.time();save(b/'result.json',report);print(json.dumps(report),flush=True)
raise SystemExit(0 if report['passed'] else 1)
