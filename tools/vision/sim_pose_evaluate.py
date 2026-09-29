#!/usr/bin/env python3
"""Run a frozen estimator with no truth input, then independently score results."""
import argparse,hashlib,json,os,subprocess,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];RUN=ROOT/'runs/grasp_pose_sim_20260921';EVIDENCE=ROOT/'docs/evidence/grasp_pose_sim_20260921/simulation'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('scenes',nargs='*',default=['near','tilted','yawed','far','occluded','close','clear']);p.add_argument('--dataset-root',type=Path,default=RUN);p.add_argument('--bundle',type=Path,default=RUN/'pose_snapshot');p.add_argument('--visibility-model',type=Path);p.add_argument('--results-dir',type=Path,default=EVIDENCE/'pose_results_final');a=p.parse_args()
 cli=a.bundle.resolve()/'lib/astribot_object_pose_core/object_pose_register';lib=a.bundle.resolve()/'lib/libastribot_object_pose_registration.so';model=ROOT/'ws_robot/src/astribot_object_pose_core/models/asymmetric_union.xyz'
 # Freeze the library as well as the executable. A sourced ROS overlay must
 # not silently substitute another ABI while we hash the bundle's library.
 env=os.environ.copy();env['LD_LIBRARY_PATH']=str(lib.parent)+os.pathsep+env.get('LD_LIBRARY_PATH','')
 if cli.read_bytes()[:4] == b'\x7fELF':
  linked=subprocess.run(['ldd',str(cli)],env=env,capture_output=True,text=True,check=True).stdout
  matches=[line.split('=>',1)[1].strip().split()[0] for line in linked.splitlines() if 'libastribot_object_pose_registration.so =>' in line]
  if len(matches)!=1 or Path(matches[0]).resolve()!=lib.resolve():raise RuntimeError('frozen estimator library not selected')
 out=a.results_dir.resolve();out.mkdir(parents=True,exist_ok=True)
 # A failed new evaluation must not leave a previous successful summary.
 (out/'summary.json').unlink(missing_ok=True)
 result=[]
 for name in a.scenes:
  scene=a.dataset_root/name/'scene.xyz';target=out/(name+'.json');started=time.monotonic()
  target.unlink(missing_ok=True)
  command=[str(cli),'--model',str(model),'--scene',str(scene),'--output',str(target)]
  if a.visibility_model:command += ['--visibility-model',str(a.visibility_model.resolve())]
  with (out/(name+'.log')).open('w') as log:r=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=90,env=env)
  if r.returncode not in (0,2):
   target.unlink(missing_ok=True)
   raise RuntimeError(f'{name}: estimator exited {r.returncode}; no result accepted')
  row=dict(scene=name,elapsed_wall_sec=time.monotonic()-started,exit_code=r.returncode,scene_sha256=sha(scene),model_sha256=sha(model),translation_error_m=None,rotation_error_deg=None,accepted=False)
  raw=json.loads(target.read_text())
  if raw.get('success') is not (r.returncode == 0):
   target.unlink(missing_ok=True)
   raise RuntimeError(f'{name}: estimator success disagrees with exit code {r.returncode}')
  row.update(raw);row['accepted']=False
  if raw['success']:
   estimate=np.array(raw['camera_from_object']).reshape(4,4)
   truth=np.array(json.loads((a.dataset_root/name/'truth.json').read_text())['camera_from_object'])
   row['translation_error_m']=float(np.linalg.norm(estimate[:3,3]-truth[:3,3]));row['rotation_error_deg']=float(np.degrees(Rotation.from_matrix(estimate[:3,:3].T@truth[:3,:3]).magnitude()));row['accepted']=row['translation_error_m']<=.020 and row['rotation_error_deg']<=10
  result.append(row);print(json.dumps(row),flush=True)
 summary=dict(visibility_model_sha256=sha(a.visibility_model) if a.visibility_model else None,schema_version=1,estimator_cli_sha256=sha(cli),estimator_library_sha256=sha(lib),fixed_thresholds=dict(translation_m=.020,rotation_deg=10),object_symmetry='none declared; asymmetric CAD',truth_passed_to_estimator=False,scene_source='actual raw Gazebo RGB-D, HSV validation mask; not YOLO',results=result,accepted_count=sum(x['accepted'] for x in result),total_count=len(result))
 (out/'summary.json').write_text(json.dumps(summary,indent=2))
if __name__=='__main__':main()
