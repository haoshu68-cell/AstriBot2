#!/usr/bin/env python3
"""Freeze selected source/interfaces and installed artifacts without claiming ABI compatibility."""
import argparse, datetime, hashlib, json, os, subprocess
from pathlib import Path
from ament_index_python.packages import get_package_prefix
ROOT=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise RuntimeError('preserve previous manifest')
packages=['astribot_s1_description','astribot_s1_moveit_config','astribot_s1_manipulation','astribot_s1_transport','astribot_s1_transport_mtc','astribot_s1_perception_components','astribot_s1_manipulation_perception','astribot_s1_navigation','astribot_navigation_msgs','astribot_perception_msgs','astribot_transport_msgs','astribot_s1_robot_geometry','moveit_ros_perception','moveit_ros_planning','moveit_ros_move_group','gz_ros2_control','astribot_s1_path_tracking','astribot_navigation_zones','astribot_s1_navigation_policy','astribot_s1_navigation_policy_native','astribot_map_manager','astribot_operator_backend']
packages += ['astribot_s1_gazebo_bringup', 'astribot_s1_social_navigation',
             'astribot_s1_exploration', 'astribot_s1_task_arbiter_native']
def digest(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
 return h.hexdigest()
files={};prefixes={};dependencies={}
for package in packages:
 prefix=Path(get_package_prefix(package));prefixes[package]=str(prefix)
 source=ROOT/'ws_robot/src'/package
 for tree in (source,prefix/'lib',prefix/'share'/package):
  if not tree.exists():continue
  for path in tree.rglob('*'):
   if not path.is_file() or '__pycache__' in path.parts or path.suffix in ('.pyc','.stl','.STL','.dae'):continue
   files[str(path)]=dict(sha256=digest(path),bytes=path.stat().st_size,resolved=str(path.resolve()))
   if tree==prefix/'lib' and path.suffix!='.a':
    with path.open('rb') as f:elf=f.read(4)==b'\x7fELF'
    if elf:
     result=subprocess.run(['ldd',str(path)],capture_output=True,text=True)
     dependencies[str(path)]=result.stdout+result.stderr
for path in (ROOT/'tools/vision/patches').glob('*.patch'):
 files[str(path)]=dict(sha256=digest(path),bytes=path.stat().st_size,resolved=str(path.resolve()))
report=dict(time=datetime.datetime.now().astimezone().isoformat(),head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),scope='selected source/interface/installed dependency snapshot; excludes mesh assets; file hashes are not a general ABI compatibility proof',prefixes=prefixes,environment={key:os.environ.get(key) for key in ('ROS_DOMAIN_ID','AMENT_PREFIX_PATH','LD_LIBRARY_PATH','RMW_IMPLEMENTATION')},files=files,dependencies=dependencies)
a.output.write_text(json.dumps(report,indent=2))
missing={path:text for path,text in dependencies.items() if 'not found' in text}
print(json.dumps(dict(files=len(files),elf_files=len(dependencies),missing_dependencies=list(missing))))
if missing:raise RuntimeError('missing ELF dependencies')
