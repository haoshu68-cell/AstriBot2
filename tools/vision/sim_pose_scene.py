#!/usr/bin/env python3
"""Create/move a known asymmetric fixture in the canonical warehouse.
This scene/truth setup is separate from the RGB-D capture and pose estimator.
"""
import argparse,json,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
RUN=ROOT/'runs/grasp_pose_sim_20260921'
BOXES=[([0,0,0],[.18,.10,.08]),([.035,.015,.065],[.070,.060,.050]),([.115,-.020,-.005],[.060,.045,.040])]

def service(name,reqtype,request,require_success=True):
 r=subprocess.run(['ign','service','-s',f'/world/default/{name}','--reqtype',reqtype,'--reptype','ignition.msgs.Boolean','--timeout','10000','--req',request],capture_output=True,text=True,timeout=15)
 if r.returncode or (require_success and 'data: true' not in r.stdout):raise RuntimeError(r.stdout+r.stderr)
 return r.stdout

def main():
 global RUN
 p=argparse.ArgumentParser();p.add_argument('scenario',choices=['near','tilted','yawed','far','occluded','close','clear']);p.add_argument('--create',action='store_true');p.add_argument('--create-occluder',action='store_true');p.add_argument('--camera',default='torso_rgbd');p.add_argument('--dataset-root',type=Path,default=RUN);a=p.parse_args()
 RUN=a.dataset_root.resolve()
 baseline=json.loads((RUN/('torso_probe' if a.camera=='torso_rgbd' else 'baseline')/'camera_info.json').read_text())['odom_from_camera']
 R=Rotation.from_quat(baseline['quaternion_xyzw']);t=np.array(baseline['translation'])
 poses={'clear':([0,-.075,1.0],[65,10,-65]),'close':([-.02,-.03,.55],[90,0,90]),'near':([-.02,.01,.70],[20,-20,35]),'tilted':([.025,.01,.78],[-25,35,70]),'yawed':([-.02,-.02,.85],[30,20,125]),'far':([.02,.02,1.15],[-15,-30,55]),'occluded':([0,0,.72],[20,-20,35])}
 xyz,angles=poses[a.scenario]
 if a.camera=='torso_rgbd':xyz[1]=-.035 if a.scenario=='close' else -.075 if a.scenario=='clear' else -.10
 world_t=t+R.apply(xyz)
 # Orientation explicitly configured relative to world; never fed to estimator.
 world_R=Rotation.from_euler('xyz',angles,degrees=True);q=world_R.as_quat()
 out=RUN/a.scenario;out.mkdir(parents=True,exist_ok=True)
 if a.create:
  parts=[]
  for i,(c,s) in enumerate(BOXES):
   shape=f'<pose>{" ".join(map(str,c))} 0 0 0</pose><geometry><box><size>{" ".join(map(str,s))}</size></box></geometry>'
   parts.append(f'<visual name="part_{i}">{shape}<material><ambient>1 0 1 1</ambient><diffuse>1 0 1 1</diffuse><specular>0 0 0 1</specular></material></visual><collision name="collision_{i}">{shape}</collision>')
  sdf='<sdf version="1.7"><model name="grasp_pose_fixture"><static>true</static><link name="body">'+''.join(parts)+'</link></model></sdf>'
  (RUN/'fixture.sdf').write_text(sdf)
  service('create','ignition.msgs.EntityFactory',f'sdf_filename: "{RUN}/fixture.sdf", name: "grasp_pose_fixture", allow_renaming: false')
 service('set_pose','ignition.msgs.Pose',f'name: "grasp_pose_fixture", position: {{x:{world_t[0]} y:{world_t[1]} z:{world_t[2]}}}, orientation: {{x:{q[0]} y:{q[1]} z:{q[2]} w:{q[3]}}}')
 if (RUN/'occluder.sdf').exists() and a.scenario!='occluded':
  service('set_pose','ignition.msgs.Pose','name: "grasp_pose_occluder", position: {x:20 y:20 z:20}, orientation: {w:1}',require_success=False)
 if a.create_occluder:
  sdf='<sdf version="1.7"><model name="grasp_pose_occluder"><static>true</static><link name="body"><visual name="green_panel"><geometry><box><size>0.035 0.16 0.02</size></box></geometry><material><ambient>0 0.8 0.1 1</ambient><diffuse>0 0.8 0.1 1</diffuse></material></visual></link></model></sdf>'
  (RUN/'occluder.sdf').write_text(sdf)
  service('create','ignition.msgs.EntityFactory',f'sdf_filename: "{RUN}/occluder.sdf", name: "grasp_pose_occluder", allow_renaming: false')
 if a.scenario=='occluded':
  ot=t+R.apply([.03,-.10,.59]);oq=R.as_quat()
  service('set_pose','ignition.msgs.Pose',f'name: "grasp_pose_occluder", position: {{x:{ot[0]} y:{ot[1]} z:{ot[2]}}}, orientation: {{x:{oq[0]} y:{oq[1]} z:{oq[2]} w:{oq[3]}}}')
 (out/'truth_setup.json').write_text(json.dumps(dict(scenario=a.scenario,world_from_object=dict(translation=world_t.tolist(),quaternion_xyzw=q.tolist()),configured_world_rpy_deg=angles,nominal_camera_location=xyz,geometry=BOXES,static_fixture=True,truth_source='Gazebo scene setup; verify actual model pose from independent pose transport sample',estimator_must_not_read_this_file=True),indent=2))
 print(str(out))
if __name__=='__main__':main()
