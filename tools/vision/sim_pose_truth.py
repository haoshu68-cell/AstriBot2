#!/usr/bin/env python3
"""Offline truth recorder/scorer metadata; never imported by the estimator."""
import argparse,json,re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def matrix(p):
 T=np.eye(4);T[:3,:3]=Rotation.from_quat(p['quaternion_xyzw']).as_matrix();T[:3,3]=p['translation'];return T

def named_pose(text,name):
 for start in [m.start() for m in re.finditer(r'(?m)^pose \{',text)]:
  level=0;end=start
  for end in range(start,len(text)):
   level+=(text[end]=='{')-(text[end]=='}')
   if end>start+5 and level==0:break
  block=text[start:end+1]
  if f'name: "{name}"' not in block:continue
  def values(key,components,defaults):
   m=re.search(key+r' \{([^}]*)\}',block,re.S);b=m.group(1) if m else ''
   return [float(re.search(r'\b'+c+r': ([^\s]+)',b).group(1)) if re.search(r'\b'+c+r': ([^\s]+)',b) else d for c,d in zip(components,defaults)]
  return dict(translation=values('position','xyz',[0]*3),quaternion_xyzw=values('orientation','xyzw',[0,0,0,1]))
 raise ValueError('missing Gazebo model '+name)

def main():
 p=argparse.ArgumentParser();p.add_argument('directory');a=p.parse_args();d=Path(a.directory)
 m=json.loads((d/'camera_info.json').read_text());raw=(d/'world_poses.pbtxt').read_text()
 obj=named_pose(raw,'grasp_pose_fixture');robot=named_pose(raw,'astribot_s1')
 world_robot=matrix(robot);base_camera=matrix(m['base_from_camera']);odom_camera=matrix(m['odom_from_camera']);odom_robot=odom_camera@np.linalg.inv(base_camera)
 discrepancy=np.linalg.norm(world_robot[:3,3]-odom_robot[:3,3]);angle=np.degrees(Rotation.from_matrix(world_robot[:3,:3].T@odom_robot[:3,:3]).magnitude())
 # Gazebo's 3D odometry publisher is world anchored. Verify this from its
 # independently observed model pose before using the capture-time TF.
 if discrepancy>.002 or angle>.2:raise RuntimeError(f'world/odom anchor discrepancy {discrepancy} m {angle} deg')
 camera_object=np.linalg.inv(odom_camera)@matrix(obj)
 h=raw[:raw.index('\npose {')];sec=re.search(r'\bsec: (\d+)',h);ns=re.search(r'\bnsec: (\d+)',h);ts=(int(sec.group(1)) if sec else 0)*10**9+(int(ns.group(1)) if ns else 0)
 truth=dict(schema_version=1,source='independent Gazebo pose transport; target static; capture-time ROS TF',world_from_object=obj,world_from_robot_at_truth_sample=robot,camera_from_object=camera_object.tolist(),capture_stamp_ns=m['capture_stamp_ns'],truth_transport_stamp_ns=ts,truth_minus_capture_sec=(ts-m['capture_stamp_ns'])/1e9,world_odom_anchor_translation_discrepancy_m=discrepancy,world_odom_anchor_rotation_discrepancy_deg=angle,truth_used_for_estimator_input=False,target_static=True,notes='Pose transport timestamp differs from image. Target is static; dynamic camera transform is queried at the image stamp. World/odom anchor verified against independent robot pose within 2 mm and 0.2 deg.')
 (d/'truth.json').write_text(json.dumps(truth,indent=2));print(json.dumps({k:truth[k] for k in ('world_odom_anchor_translation_discrepancy_m','world_odom_anchor_rotation_discrepancy_deg','truth_minus_capture_sec')}))
if __name__=='__main__':main()
