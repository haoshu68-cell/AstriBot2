import json,collections
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import cv2
root=Path('/home/yjh/WorkSpace/astribot_sdk_ros2')
base=root/'runs/grasp_pose_sim_20260921'
model=np.loadtxt(root/'ws_robot/src/astribot_object_pose_core/models/asymmetric_union.xyz')
boxes=json.loads((root/'ws_robot/src/astribot_object_pose_core/models/asymmetric_union.visibility.json').read_text())['boxes']
summary={}
for name in ('near','tilted','yawed','far','occluded','close','clear'):
    d=json.loads((base/'pose_v3/diagnosis'/f'{name}_v2_candidates.json').read_text())
    scene=np.loadtxt(base/name/'scene.xyz');scene[:,3:]/=np.linalg.norm(scene[:,3:],axis=1)[:,None]
    transform=np.array(d['best']['transform']).reshape(4,4);rotation=transform[:3,:3];translation=transform[:3,3]
    eye=-rotation.T@translation
    rays=model[:,:3]-eye
    visible=np.einsum('ij,ij->i',rays,model[:,3:]) < -1e-9
    for box in boxes:
        center=np.array(box['center_m']);size=np.array(box['size_m'])
        lo=np.zeros(len(model));hi=np.ones(len(model));hit=np.ones(len(model),dtype=bool)
        for k in range(3):
            parallel=np.abs(rays[:,k])<1e-12
            hit[parallel & ((eye[k]<center[k]-size[k]/2)|(eye[k]>center[k]+size[k]/2))]=False
            a=np.full(len(model),-np.inf);b=np.full(len(model),np.inf)
            a[~parallel]=(center[k]-size[k]/2-eye[k])/rays[~parallel,k]
            b[~parallel]=(center[k]+size[k]/2-eye[k])/rays[~parallel,k]
            lo=np.maximum(lo,np.minimum(a,b));hi=np.minimum(hi,np.maximum(a,b))
        visible &= ~(hit & (lo<=hi)&(lo<1-1e-5)&(hi>1e-8))
    p=model[:,:3]@rotation.T+translation;n=model[:,3:]@rotation.T
    distances,ids=cKDTree(scene[:,:3]).query(p)
    matched=(distances<=.008)&(np.einsum('ij,ij->i',n,scene[ids,3:])>.35)
    depth=np.load(base/name/'depth.npy');mask=cv2.imread(str(base/name/'mask.png'),cv2.IMREAD_GRAYSCALE)>0
    info=json.loads((base/name/'camera_info.json').read_text());k=np.array(info['K']).reshape(3,3)
    pixels=np.round((p@k.T)[:,:2]/p[:,2:3]).astype(int)
    counts=collections.Counter(); missing=[]
    for i in np.flatnonzero(visible):
        u,v=pixels[i];reason=''
        if matched[i]:reason='supported_by_scene'
        elif p[i,2]<=0 or u<0 or v<0 or u>=depth.shape[1] or v>=depth.shape[0]:reason='outside_fov'
        elif not np.isfinite(depth[v,u]) or depth[v,u]<=0:reason='invalid_depth'
        elif depth[v,u]<p[i,2]-.008:reason='foreground_outside_target_mask' if not mask[v,u] else 'target_surface_in_front'
        elif depth[v,u]>p[i,2]+.008:reason='predicted_surface_in_front_of_depth'
        else:reason='near_depth_outside_target_mask' if not mask[v,u] else 'near_depth_but_rejected_normal_or_sampling'
        counts[reason]+=1
        if reason!='supported_by_scene':missing.append({'sample':int(i),'pixel':[int(u),int(v)],'reason':reason})
    result={'candidate_based_not_truth':True,'visible_points':int(visible.sum()),'visible_support_fraction':float(matched[visible].mean()),'classes':dict(counts),'fractions':{key:value/int(visible.sum()) for key,value in counts.items()},'input_points':len(scene)}
    summary[name]=result
    (base/'pose_v3/diagnosis'/f'{name}_visible_missing.json').write_text(json.dumps({'summary':result,'missing':missing},indent=2)+'\n')
    print(name,result['fractions'])
(base/'pose_v3/diagnosis/visibility_diagnosis_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
