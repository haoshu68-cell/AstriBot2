"""Private one-Goal C++ PICK, navigation, PLACE; run only after owner dispatch."""
import argparse,hashlib,json,os,shlex,signal,subprocess,sys,time,yaml
from pathlib import Path
R=Path('/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_navigation_20260923_155154');E=Path(__file__).resolve().parent
I=R.parent/'I0_2_20260923_065115';ROOT=I/'source_repo';OVERLAY=None
W=Path('/home/yjh/WorkSpace/astribot_sdk_ros2')
sys.path.insert(0,str(ROOT/'tools/social_navigation'))
from run_regression import runtime_environment,read_json,save,stop_owned,descendants
from sim_isolation import SimulationIsolation
from sim_camera_preset import DEFAULT_RELATIVE_PRESET
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--case',required=True)
parser.add_argument('--executor',type=Path,required=True)
parser.add_argument('--executor-sha256',required=True)
parser.add_argument('--native-library-sha256',required=True)
parser.add_argument('--navigation-helper-sha256',required=True)
parser.add_argument('--payload-client-sha256',required=True)
parser.add_argument('--payload-scene-sha256',required=True)
parser.add_argument('--payload-geometry-library',type=Path,required=True)
parser.add_argument('--payload-geometry-sha256',required=True)
parser.add_argument('--payload-execution-sha256',required=True)
parser.add_argument('--path-tracking-sha256',required=True)
parser.add_argument('--fixed-envelope-sha256',required=True)
parser.add_argument('--mtc-prefix',type=Path,required=True)
parser.add_argument('--mtc-sha256',required=True)
parser.add_argument('--mtc-scene-library-sha256',required=True)
parser.add_argument('--overlay',type=Path,required=True)
parser.add_argument('--show-gui',action='store_true',help='Open the current owned Gazebo and RViz views')
parser.add_argument('--relax-base-motion',action='store_true',help='User-authorized simulation angular-motion limits')
parser.add_argument('--session',required=True)
parser.add_argument('--domain',type=int,required=True)

options=parser.parse_args();OVERLAY=options.overlay.resolve()
scenario=read_json(E/'scenario.json')
spawn_world=scenario['spawn_world_xyyaw']
if not options.case.replace('_','').isalnum():raise RuntimeError('invalid case identifier')
b=E/options.case;b.mkdir(exist_ok=False)
for name in ('run_full_transfer.py','prepare_scene.py','verify_full_transfer.py','verify_full_pick.py','prepare_transfer_goal.py','read_full_executor_parameters.py','scenario.json','sim_initial_transport_ready.yaml','observer_topics_transfer.json','head_raw_qos.yaml','transport_skills_candidate.launch.py','record_transport_demo.py'):
 (b/('used_'+name)).write_bytes((E/name).read_bytes())
(b/'used_overlay.bash').write_bytes(OVERLAY.read_bytes())
isolation=SimulationIsolation(options.session,options.domain)
env=os.environ.copy();env.update(ASTRIBOT_SIM_INITIAL_JOINT_PROFILE='transport_ready',ASTRIBOT_SCAN_TIMING_DIAGNOSTICS='1',ASTRIBOT_POLICY_NATIVE_TF='1',DISPLAY=':1',ASTRIBOT_OVERLAY_SETUP=str(OVERLAY),EGL_PLATFORM='surfaceless',LIBGL_ALWAYS_SOFTWARE='0',__GLX_VENDOR_LIBRARY_NAME='nvidia',__EGL_VENDOR_LIBRARY_FILENAMES='/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
launch=moveit=hold=runner=bridge=recorder=observer=truth=payload_truth=raw=rviz=None;prefix='';files=[];handles={};report=dict(passed=False,started_wall=time.time(),initial_joint_profile='transport_ready',zero_start_motion_validated=False)
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
 command=['bash',str(ROOT/'tools/launch_sim_stack.sh'),'--mode','baseline','--instance',isolation.instance,'--ros-domain-id',str(options.domain),'--navigation-geometry-mode','fixed_v2','--navigation-policy','p4','--payload-source-id','gazebo_kinematic_v1','--corridor-file',str(R/'N4_empty_corridors.json'),'--social-policy','off','--nav-transport','udp','--camera-preset',str(ROOT/DEFAULT_RELATIVE_PRESET),'--headless','--no-rviz','--exclusive-performance','--ready-timeout','180','--log-dir',str(b/'stack')]
 command += ['--spawn-x',str(spawn_world[0]),'--spawn-y',str(spawn_world[1]),'--spawn-yaw',str(spawn_world[2])]
 if options.show_gui:
  command.remove('--headless')
 report['show_gui']=options.show_gui
 launch=spawn('launcher',command)
 deadline=time.monotonic()+200
 while launch.poll() is None and time.monotonic()<deadline:
  session=read_json(b/'stack/session.json')
  if session.get('state')=='ready':break
  time.sleep(.5)
 else:raise RuntimeError('fixed supervisor not ready')
 if session['supervisor_pid']!=launch.pid:raise RuntimeError('owner mismatch')
 prefix=runtime_environment(launch.pid,b,OVERLAY,isolation)
 if options.show_gui:
  view=yaml.safe_load((W/'ws_robot/src/astribot_s1_navigation/rviz/nav2_view.rviz').read_text())
  view['Panels']=[panel for panel in view['Panels'] if not panel['Class'].startswith('nav2_')]
  view['Visualization Manager']['Tools']=[{'Class':'rviz_default_plugins/MoveCamera'},{'Class':'rviz_default_plugins/Select'}]
  view['Visualization Manager']['Global Options']['Frame Rate']=15
  view['Visualization Manager']['Views']['Current']['Distance']=4
  view_file=b/'mainline_readonly.rviz';view_file.write_text(yaml.safe_dump(view,sort_keys=False))
  rviz=child('rviz_view',['rviz2','-d',view_file,'--ros-args','-p','use_sim_time:=true','-r','__node:=mainline_rviz_view'])
 # Read actual mappings before any navigation goal; selecting an overlay alone is insufficient.
 expected_library=Path('/home/yjh/WorkSpace/astribot_sdk_ros2/runs/task_chain_20260922/gz_control_fix/install/gz_ros2_control/lib/libgz_ros2_control-system.so').resolve()
 expected_sha='bea2defb55a928490f68ace9900302c7560af6ba85270d04f3c14b40e8d5bfca'
 loaded=[];hardware_loaded=[]
 for pid in descendants(launch.pid):
  try:
   map_lines=Path(f'/proc/{pid}/maps').read_text().splitlines()
   paths={Path(line.split()[-1]) for line in map_lines if 'libgz_ros2_control-system.so' in line and line.split()[-1].startswith('/')}
   hardware_paths={Path(line.split()[-1]) for line in map_lines if 'libgz_hardware_plugins.so' in line and line.split()[-1].startswith('/')}
   for path in hardware_paths:hardware_loaded.append(dict(pid=pid,path=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),start_ticks=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]))
   for path in paths:loaded.append(dict(pid=pid,path=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),start_ticks=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]))
  except (OSError,ValueError):continue
 save(b/'gz_control_runtime_manifest.json',dict(expected=str(expected_library),expected_sha256=expected_sha,loaded=loaded,hardware_plugins=hardware_loaded))
 if len(loaded)!=1 or loaded[0]['path']!=str(expected_library) or loaded[0]['sha256']!=expected_sha:raise RuntimeError('BOUNDED_URDF_RUNTIME_IDENTITY_MISMATCH')
 libraries=read_json(b/'runtime_manifest.json')['libraries']
 tracking_libraries={path:digest for path,digest in libraries.items() if path.endswith('/libastribot_s1_path_tracking.so')}
 if len(tracking_libraries)!=1 or next(iter(tracking_libraries.values()))!=options.path_tracking_sha256:raise RuntimeError('PATH_TRACKING_RUNTIME_IDENTITY_MISMATCH')
 startup_log=(b/'stack/session.log').read_text(errors='replace')
 bounded_lines=[line for line in startup_log.splitlines() if 'bounded request(s)' in line or 'bounded retry' in line]
 save(b/'bounded_urdf_log.json',dict(lines=bounded_lines))
 if not any('Received URDF after' in line for line in bounded_lines):raise RuntimeError('BOUNDED_URDF_LOG_UNOBSERVED')
 if session['camera_baseline']['sha256']!='8942e6b0952a982ad3edb3b9fb8aa293f0c528a06ea6a720f3e145056cdd625f':raise RuntimeError('SIX_CAMERA_BASELINE_CHANGED')
 run('capture_owner' ,['python3',W/'tools/sim/capture_supervisor_owner.py','--pid',launch.pid,'--session',isolation.instance,'--source','gazebo_kinematic_v1','--output',b/'owner.json'],10)
 from ament_index_python.packages import get_package_share_directory,get_package_prefix
 desc=Path(get_package_share_directory('astribot_s1_description'));config=desc/'config';camera=config/'simulation_navigation_full'
 moveit=child('transport_skills',['ros2','launch',str(E/'transport_skills_candidate.launch.py'),'allow_trajectory_execution:=false','use_camera:=true','use_wrist_cameras:=true','use_stereo_cameras:=true','use_lidar:=true','simulation_relaxed_base_motion:='+str(options.relax_base_motion).lower(),'camera_profile:='+str(camera/'camera_head_rgbd.yaml'),'torso_camera_profile:='+str(camera/'camera_torso_rgbd.yaml'),'camera_calibration_dir:='+str(camera),'camera_mounts_profile:='+str(config/'camera_mounts_reference_sim.yaml')])
 executor=options.executor.resolve()
 executor_sha=options.executor_sha256
 if hashlib.sha256(executor.read_bytes()).hexdigest()!=executor_sha:raise RuntimeError('EXECUTOR_HASH_CHANGED')
 run('read_full_executor_parameters',['python3',E/'read_full_executor_parameters.py','--owner',b/'owner.json','--session',isolation.instance,'--source','gazebo_kinematic_v1','--scenario',E/'scenario.json','--output',b/'executor_registration'],50)
 hold=child('trajectory_executor',[str(executor),'--ros-args','--params-file',str(b/'executor_registration/executor.yaml'),'-p','simulation_relaxed_base_motion:='+str(options.relax_base_motion).lower()])
 worlds=[]
 for pid in descendants(launch.pid):
  try:
   args=Path(f'/proc/{pid}/cmdline').read_bytes().decode().rstrip('\0').split('\0')
   if len(args)==1 and args[0].startswith('ign gazebo '): args=shlex.split(args[0])
   if args[:2]==['ign','gazebo'] and '-g' not in args:
    for a in args:
     p=Path(a)
     if a.endswith(('.sdf','.world')) and p.is_file():worlds.append(str(p.resolve()))
  except OSError:pass
 worlds=sorted(set(worlds))
 if len(worlds)!=1:raise RuntimeError('actual Gazebo world ambiguous: '+str(worlds))
 save(b/'actual_world.json',dict(path=worlds[0],sha256=hashlib.sha256(Path(worlds[0]).read_bytes()).hexdigest()))
 sys.path.insert(0,str(W/'tools/sim'))
 from capture_supervisor_owner import capture
 chassis=[]
 for pid in descendants(launch.pid):
  try:identity=capture(pid)
  except (FileNotFoundError,ProcessLookupError):continue
  if Path(identity['exe']).name=='omni_effort_drive_cpp':
   identity['sha256']=hashlib.sha256(Path(identity['exe']).read_bytes()).hexdigest()
   chassis.append(identity)
 save(b/'chassis_runtime_identity.json',chassis)
 if len(chassis)!=1:raise RuntimeError('CHASSIS_RUNTIME_NOT_UNIQUE')
 run('gazebo_topics',['ign','topic','-l'],10)
 spawned_executor=read_json(b/'trajectory_executor_identity.json')
 executor_deadline=time.monotonic()+10.
 while True:
  if hold.poll() is not None:raise RuntimeError('EXECUTOR_EXITED_DURING_STARTUP: '+str(hold.returncode))
  current_executor=capture(hold.pid)
  if any(current_executor[key]!=spawned_executor[key] for key in ('pid','start_ticks','boot_id')):raise RuntimeError('EXECUTOR_STARTUP_IDENTITY_CHANGED')
  if Path(current_executor['exe']).resolve()==executor:break
  if time.monotonic()>=executor_deadline:raise RuntimeError('EXECUTOR_EXEC_TIMEOUT: '+current_executor['exe'])
  time.sleep(.05)
 save(b/'executor_started_identity.json',current_executor)
 identities={}
 for pid in descendants(launch.pid):
  try:
   identity=capture(pid);name=Path(identity['exe']).name
   if name in ('bt_navigator','task_arbiter_cpp','operator_backend','loop_route_executor','waypoint_follower'):
    if name in identities:raise RuntimeError('ambiguous navigation process '+name)
    identities[name]=identity
  except (FileNotFoundError,ProcessLookupError):pass
 save(b/'preparation_cold_start.json',dict(owner=read_json(b/'owner.json'),session=isolation.instance,captured_wall=time.time(),no_preceding_goal_writers=True,fresh_executor=current_executor,processes=identities,probe_output=str((b/'full_transfer').resolve())))
 plugin=Path(get_package_prefix('astribot_s1_gazebo_bringup'))/'lib'
 run('prepare_scene',['python3',E/'prepare_scene.py','--owner',b/'owner.json','--session',isolation.instance,'--source','gazebo_kinematic_v1','--plugin-directory',plugin,'--world-reference',worlds[0],'--output',b/'fixtures','--cold-start-receipt',b/'preparation_cold_start.json'],120)
 report['fixture_directory']=str(b/'fixtures')
 geometry_expected=options.payload_geometry_library.resolve()
 geometry_loaded=[];execution_loaded=[]
 gazebo_pid=loaded[0]['pid'];gazebo_identity=capture(gazebo_pid)
 for line in Path(f'/proc/{gazebo_pid}/maps').read_text().splitlines():
  fields=line.split()
  if not fields[-1].startswith('/'):continue
  path=Path(fields[-1])
  if path.name not in ('libastribot_payload_geometry.so','libastribot_payload_execution.so'):continue
  row=dict(path=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
  target=geometry_loaded if path.name=='libastribot_payload_geometry.so' else execution_loaded
  if row not in target:target.append(row)
 save(b/'payload_geometry_runtime_manifest.json',dict(identity=gazebo_identity,expected_path=str(geometry_expected),expected_sha256=options.payload_geometry_sha256,geometry=geometry_loaded,execution=execution_loaded))
 if geometry_loaded!=[dict(path=str(geometry_expected),sha256=options.payload_geometry_sha256)]:raise RuntimeError('PAYLOAD_GEOMETRY_RUNTIME_IDENTITY_MISMATCH')
 if len(execution_loaded)!=1 or execution_loaded[0]['sha256']!=options.payload_execution_sha256:raise RuntimeError('PAYLOAD_EXECUTION_RUNTIME_IDENTITY_MISMATCH')

 registration=read_json(b/'executor_registration/result.json')
 fixture=read_json(b/'fixtures/result.json')
 if {k:v['sha256'] for k,v in fixture['model_readback'].items()}!=registration['xml_sha256']:raise RuntimeError('ACTUAL_PREPARED_MODEL_CHANGED')
 import xml.etree.ElementTree as ET
 actual_box=ET.parse(b/'fixtures/transport_box_01.sdf').getroot().find('model')
 registered=registration['registered_parameters']
 actual_size=[float(v) for v in actual_box.find('link/collision/geometry/box/size').text.split()]
 if actual_box.get('name')!=registered['payload_model'] or actual_size!=registered['payload_size_xyz']:raise RuntimeError('ACTUAL_PREPARED_BOX_CHANGED')
 actual_registry=read_json(b/'fixtures/registry.json')
 if actual_registry!=[dict(model=registered['payload_model'],object_id=registered['payload_object_id'],physical_parent_link='astribot_arm_left_link_7',attachment_link=registered['payload_tcp_frame'])]:raise RuntimeError('ACTUAL_PREPARED_REGISTRY_CHANGED')
 save(b/'prepared_registration_comparison.json',dict(passed=True,registration=registration,actual_registry=actual_registry,actual_size=actual_size,scope='registration matches this actual prepared fixture; not an attachment proof'))
 run('prepare_transfer_goal',['python3',E/'prepare_transfer_goal.py','--fixture',b/'fixtures','--scenario',E/'scenario.json','--owner',b/'owner.json','--session',isolation.instance,'--output',b/'transfer_goal.json'],45)
 observer=child('m5_observer',['python3',W/'tools/vision/capture_m5_observer.py','--topic-config',E/'observer_topics_transfer.json','--session-json',b/'stack/session.json','--output',b/'m5_observer','--seconds','600','--warmup-sec','2','--phase-topic','/transport/hold_executor/status','--reference',W/'ws_robot/src/astribot_s1_description/config/simulation_navigation_full/launch_preset.yaml'])
 observer_started=time.monotonic()
 bridge=child('overview_bridge',[str(Path(get_package_prefix('ros_gz_bridge'))/'lib/ros_gz_bridge/parameter_bridge'),'/transport/overview@sensor_msgs/msg/Image[ignition.msgs.Image','--ros-args','-p','use_sim_time:=true'])
 recorder=child('recorder',['python3',E/'record_transport_demo.py','--output',b/'full_transfer.mp4','--head-topic','/camera/raw/head_rgbd/image','--status-topic','/transport/hold_executor/status','--status-format','native_hold','--title','ASTRIBOT | C++ PICK - NAVIGATE - PLACE','--timeout','600'])
 time.sleep(3)
 truth=child('world_poses',['ign','topic','-e','-t','/world/default/pose/info','-d','600'])
 payload_truth=child('payload_physical_state',['ign','topic','-e','-t','/model/transport_box_01/kinematic_attachment/state','--json-output','-d','600'])
 try:
  run('head_snapshot',['python3',W/'tools/vision/sim_pose_capture.py','--camera','head_rgbd','--seconds','6','--truth-stream',b/'world_poses.log','--output',b/'head_snapshot'],25)
  report['head_snapshot']='captured; ROI and coverage not evaluated'
 except RuntimeError as error:
  report['head_snapshot_error']=str(error)
  runner=None
 raw_topics=list(yaml.safe_load((E/'head_raw_qos.yaml').read_text()))
 raw_start=time.monotonic()
 raw=child('head_raw_bag',['timeout','--signal=INT','--kill-after=10s','3s','ros2','bag','record','--storage','sqlite3','--max-cache-size','104857600','--qos-profile-overrides-path',E/'head_raw_qos.yaml','--output',b/'head_raw_bag',*raw_topics])
 time.sleep(.5)
 raw_identities=[]
 for pid in [raw.pid,*descendants(raw.pid)]:
  try:raw_identities.append(capture(pid))
  except (FileNotFoundError,ProcessLookupError):continue
 save(b/'head_raw_owners.json',raw_identities)
 raw.wait(timeout=15)
 report['head_raw']=dict(returncode=raw.returncode,started_steady=raw_start,ended_steady=time.monotonic(),requested_process_window_s=3,topics=raw_topics,not_activated=['/manipulation/single_box/head_rgbd/points','/perception/projection_health/single_box/head_rgbd'],scope='stationary baseline head evidence; not M3 complete input; timeout 124 is not a bag pass')
 try:
  raw_metadata=yaml.safe_load((b/'head_raw_bag/metadata.yaml').read_text())
  report['head_raw']['metadata']=raw_metadata
  raw_info=raw_metadata['rosbag2_bagfile_information']
  counts={row['topic_metadata']['name']:row['message_count'] for row in raw_info['topics_with_message_count']}
  report['head_raw']['missing_topics']=[topic for topic in raw_topics if counts.get(topic,0)==0]
 except (OSError,ValueError,KeyError,TypeError,yaml.YAMLError) as error:
  report['head_raw']['metadata_error']=repr(error)
 save(b/'head_raw_summary.json',report['head_raw'])
 current_executor=capture(hold.pid)
 if Path(current_executor['exe']).resolve()!=executor.resolve():raise RuntimeError('EXECUTOR_NOT_STARTED')
 save(b/'executor_owner.json',current_executor)
 if observer.poll() is not None:raise RuntimeError('M5_OBSERVER_EXITED_BEFORE_ACTION')
 save(b/'observer_owner.json',capture(observer.pid))
 time.sleep(max(0.,observer_started+20.-time.monotonic()))
 if observer.poll() is not None:raise RuntimeError('M5_OBSERVER_EXITED_BEFORE_ACTION')
 component_images=[]
 for pid in [hold.pid,moveit.pid,*descendants(moveit.pid)]:
  try:
   identity=capture(pid);paths={line.split()[-1] for line in Path(f'/proc/{pid}/maps').read_text().splitlines() if line.split()[-1].startswith('/home/yjh/') and any(key in line for key in ('astribot_s1_manipulation','astribot_s1_transport_mtc','astribot_s1_transport_native','libarm_hold_core.so','libfixed_station_navigation.so','libpayload_client.so','libpayload_scene.so','libtransport_scene_signature.so'))}
   component_images.append(dict(identity=identity,executable_sha256=hashlib.sha256(Path(identity['exe']).read_bytes()).hexdigest(),libraries={path:hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in sorted(paths)}))
  except (FileNotFoundError,ProcessLookupError):continue
 save(b/'manipulation_runtime_manifest.json',component_images)
 candidate_images={row['identity']['exe']:row['executable_sha256'] for row in component_images}
 candidate_libraries={path:digest for row in component_images for path,digest in row['libraries'].items()}
 expected_mtc=options.mtc_prefix.resolve()
 expected_native=executor.parents[2]
 if candidate_images.get(str(expected_mtc/'lib/astribot_s1_transport_mtc/mtc_planner'))!=options.mtc_sha256:raise RuntimeError('MTC_RUNTIME_IDENTITY_MISMATCH')
 if candidate_libraries.get(str(expected_mtc/'lib/libtransport_scene_signature.so'))!=options.mtc_scene_library_sha256:raise RuntimeError('MTC_SCENE_LIBRARY_RUNTIME_IDENTITY_MISMATCH')
 if candidate_libraries.get(str(expected_native/'lib/libarm_hold_core.so'))!=options.native_library_sha256:raise RuntimeError('NATIVE_FULL_LIBRARY_RUNTIME_IDENTITY_MISMATCH')
 if candidate_libraries.get(str(expected_native/'lib/libpayload_client.so'))!=options.payload_client_sha256:raise RuntimeError('PAYLOAD_CLIENT_RUNTIME_IDENTITY_MISMATCH')
 if candidate_libraries.get(str(expected_native/'lib/libpayload_scene.so'))!=options.payload_scene_sha256:raise RuntimeError('PAYLOAD_SCENE_RUNTIME_IDENTITY_MISMATCH')
 if candidate_libraries.get(str(expected_native/'lib/libfixed_station_navigation.so'))!=options.navigation_helper_sha256:raise RuntimeError('NATIVE_NAVIGATION_HELPER_RUNTIME_IDENTITY_MISMATCH')
 save(b/'cold_start.json',dict(owner=read_json(b/'owner.json'),session=isolation.instance,captured_wall=time.time(),no_preceding_goal_writers=True,fresh_executor=current_executor,processes=identities,probe_output=str((b/'full_transfer').resolve())))
 run('full_transfer',['python3',E/'verify_full_transfer.py','--owner',b/'owner.json','--session',isolation.instance,'--source','gazebo_kinematic_v1','--cold-start-receipt',b/'cold_start.json','--expected-binary-sha256',options.fixed_envelope_sha256,'--expected-executor-sha256',executor_sha,'--scenario',E/'scenario.json','--goal-json',b/'transfer_goal.json','--action-endpoint','/transport/fixed_station_transfer','--output',b/'full_transfer']+(['--relax-base-motion'] if options.relax_base_motion else []),600)
 report['passed']=read_json(b/'full_transfer/result.json').get('passed',False)
 save(b/'stage_finished.json',dict(owner=read_json(b/'owner.json'),prefix=prefix,report=report))

except BaseException as error:report['error']=repr(error);report['passed']=False
finally:
 if runner is not None:
  try:stop_owned(runner,40)
  except Exception as error:report['runner_cleanup_error']=str(error)
 probe_result=read_json(b/'full_transfer/result.json')
 if (observer is not None and observer.poll() is None and launch is not None and launch.poll() is None
     and probe_result.get('cleanup_complete') is True and probe_result.get('resource_disposition')=='RELEASE_CONFIRMED'
     and not report.get('runner_cleanup_error') and not report.get('error','').startswith('KeyboardInterrupt')):
  try:time.sleep(20)
  except KeyboardInterrupt as error:
   report['post_observation_interrupted']=repr(error);report['passed']=False
 # Observer ends only after the parent probe has confirmed terminal/settling/release.
 errors=[]
 for name,process,timeout in [('head_raw_bag',raw,15),('recorder',recorder,15),('truth',truth,5),('payload_physical_state',payload_truth,5),('observer',observer,20),('overview_bridge',bridge,10),('trajectory_executor',hold,15),('transport_skills',moveit,20),('rviz_view',rviz,10),('supervisor',launch,45)]:
  try:stop_owned(process,timeout)
  except BaseException as error:errors.append(name+': '+repr(error))
 if observer is not None:
  report['observer_returncode']=observer.poll()
  observer_manifest=read_json(b/'m5_observer/manifest.json')
  report['observer_capture']=dict(manifest=observer_manifest,scope='bounded action observation; an owned early interrupt is not a complete 600-second sampling pass')
  if observer.poll() not in (0,130) or observer_manifest.get('error') or observer_manifest.get('writer_error') or observer_manifest.get('writer_dropped')!=0:
   report['observer_error']='OBSERVATION_FAILED_SEPARATE_FROM_ACTION_RESULT';report['passed']=False
 if recorder is not None and recorder.poll() is not None:
  report['recorder_returncode']=recorder.poll()
  try:
   import cv2
   video=cv2.VideoCapture(str(b/'full_transfer.mp4'));decoded=0
   while True:
    ok,frame=video.read()
    if not ok:break
    decoded+=1
   video.release()
   video_metadata=read_json(b/'full_transfer.json')
   timing=[json.loads(line) for line in (b/'full_transfer.frames.jsonl').read_text().splitlines()]
   report['video_decode']=dict(decoded_frames=decoded,sidecar_rows=len(timing),metadata_frames=video_metadata.get('frames'),passed=decoded>0 and decoded==len(timing)==video_metadata.get('frames') and [v['frame_index'] for v in timing]==list(range(decoded)))
   if not report['video_decode']['passed']:report['passed']=False
  except (OSError,ValueError,KeyError) as error:
   report['video_decode']=dict(passed=False,error=repr(error));report['passed']=False
 session=read_json(b/'stack/session.json')
 report['cleanup_complete']=bool(launch is not None and launch.poll() is not None and session.get('state') in ('stopped','failed') and session.get('ended') and session.get('remaining_owned_pids')==[] and not session.get('log_capture_errors') and not errors)
 if errors:report['cleanup_errors']=errors
 if not report['cleanup_complete']:report['passed']=False
 for fd in handles.values():os.close(fd)
 for f in files:f.close()
 report['finished_wall']=time.time();save(b/'result.json',report);print(json.dumps(report),flush=True)
raise SystemExit(0 if report['passed'] else 1)
