#!/usr/bin/env python3
"""Read-only calibrated camera rays against the parent link's actual STL mesh."""
from pathlib import Path
import json,struct
import numpy as np,yaml
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];DESC=ROOT/'ws_robot/src/astribot_s1_description'

def stl(path):
 raw=path.read_bytes();n=struct.unpack('<I',raw[80:84])[0]
 if len(raw)!=84+50*n:raise ValueError('Expected binary STL')
 return np.frombuffer(raw[84:],dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')]))['v'].astype(float)

def audit(camera,mesh):
 cfg=yaml.safe_load((DESC/f'config/camera_{camera}.yaml').read_text());mesh_path=ROOT/'ws_robot/install/astribot_s1_description/share/astribot_s1_description'/mesh;tri=stl(mesh_path)
 T=np.array(cfg['extrinsic_matrix_row_major']).reshape(4,4);origin=T[:3,3];R=T[:3,:3]
 K=np.array(cfg['intrinsics']['color']).reshape(3,3)
 rays=[]
 for v in np.linspace(0,cfg['height']-1,9):
  for u in np.linspace(0,cfg['width']-1,17):
   d=R@np.array([(u-K[0,2])/K[0,0],(v-K[1,2])/K[1,1],1.]);e1=tri[:,1]-tri[:,0];e2=tri[:,2]-tri[:,0]
   h=np.cross(np.broadcast_to(d,e2.shape),e2);a=np.sum(e1*h,axis=1);valid=np.abs(a)>1e-12
   f=np.zeros_like(a);f[valid]=1/a[valid];s=origin-tri[:,0];uu=f*np.sum(s*h,axis=1);q=np.cross(s,e1);vv=f*(q@d);tt=f*np.sum(e2*q,axis=1)
   good=valid&(uu>=0)&(vv>=0)&(uu+vv<=1)&(tt>0)
   rays.append(float(np.min(tt[good])) if np.any(good) else None)
 finite=[d for d in rays if d is not None]
 return dict(camera=camera,parent=cfg['parent_frame'],mesh=str(mesh_path),mount_translation=origin.tolist(),mesh_bounds=[tri.min(axis=(0,1)).tolist(),tri.max(axis=(0,1)).tolist()],sampled_rays=len(rays),self_intersecting_rays=len(finite),hits_before_near_clip=sum(x<cfg['near_m'] for x in finite),min_camera_depth_m=min(finite) if finite else None,max_camera_depth_m=max(finite) if finite else None,rays_depth_m=rays,interpretation='Parent rigid mesh and calibrated camera move together; joint posture cannot remove intersections with this parent mesh. No meshes or calibration were modified.')
if __name__=='__main__':
 out=[audit('head_rgbd',Path('meshes/s1_head/astribot_head_link_2.STL')),audit('torso_rgbd',Path('meshes/s1_torso/astribot_torso_base_link.STL'))]
 p=ROOT/'docs/evidence/grasp_pose_sim_20260921/simulation/parent_mesh_camera_audit.json';p.write_text(json.dumps(out,indent=2));print(json.dumps([{k:v for k,v in x.items() if k!='rays_depth_m'} for x in out],indent=2))
