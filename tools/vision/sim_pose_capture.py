#!/usr/bin/env python3
"""Capture actual rendered RGB-D with exact image/info stamps and measured TF.

Color segmentation is a validation fixture, not YOLO instance segmentation.
Object poses never enter segmentation/projection. An optional evaluation-only
truth-stream filter selects source timestamps with recorded simulator poses.
scene.xyz uses rendered depth and measured intrinsics only.
"""
import argparse,json,time
from collections import deque
from pathlib import Path
import cv2,numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer,TransformListener
from astribot_perception_msgs.msg import CameraHealth
from rosidl_runtime_py.convert import message_to_ordereddict

def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec

def transform_dict(m):
 t=m.transform.translation;q=m.transform.rotation
 return dict(parent=m.header.frame_id,child=m.child_frame_id,stamp_ns=stamp(m),translation=[t.x,t.y,t.z],quaternion_xyzw=[q.x,q.y,q.z,q.w])

def image_data(m):
 dtype=np.dtype('>f4' if m.is_bigendian else '<f4') if m.encoding=='32FC1' else np.dtype('>u2' if m.is_bigendian else '<u2') if m.encoding=='16UC1' else np.dtype('u1')
 channels=3 if m.encoding in ('rgb8','bgr8') else 4 if m.encoding in ('rgba8','bgra8') else 1
 x=np.ndarray((m.height,m.width,channels),dtype=dtype,buffer=bytes(m.data),strides=(m.step,dtype.itemsize*channels,dtype.itemsize)).copy().squeeze()
 if m.encoding=='16UC1':x=x.astype(np.float32)/1000
 if m.encoding in ('bgr8','bgra8'):x=x[:,:,::-1]
 return x

class Capture(Node):
 def __init__(self,camera):
  super().__init__('sim_pose_validation_capture')
  self.msgs={k:{} for k in ('rgb','depth','info')};self.counts={k:0 for k in self.msgs};self.stamps={k:[] for k in self.msgs};self.clock=[]
  self.health={}
  self.subs=[]
  self.subs.append(self.create_subscription(CameraHealth,f'/perception/camera_health/{camera}',self.receive_health,10))
  for key,suffix,typ in [('rgb','image',Image),('depth','depth_image',Image),('info','camera_info',CameraInfo)]:
   self.subs.append(self.create_subscription(typ,f'/camera/raw/{camera}/{suffix}',lambda m,k=key:self.receive(k,m),qos_profile_sensor_data))
  self.subs.append(self.create_subscription(Clock,'/clock',lambda m:self.clock.append(m.clock.sec*10**9+m.clock.nanosec),qos_profile_sensor_data))
  self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
 def receive_health(self,m):
  ts=m.capture_stamp.sec*10**9+m.capture_stamp.nanosec
  self.health[ts]=message_to_ordereddict(m)
  while len(self.health)>200:del self.health[min(self.health)]
 def receive(self,k,m):
  self.counts[k]+=1;self.stamps[k].append(stamp(m));self.msgs[k][stamp(m)]=m
  while len(self.msgs[k])>25:del self.msgs[k][min(self.msgs[k])]

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--camera',default='head_rgbd');p.add_argument('--seconds',type=float,default=6);p.add_argument('--require-target',action='store_true');p.add_argument('--truth-stream',type=Path,help='Evaluation-only live Pose_V text file; require a complete matching stamp');a=p.parse_args()
 truth_file=a.truth_stream.open() if a.truth_stream else None
 truth_tail='';truth_stamps=deque(maxlen=512);last_truth_stamp=None
 if truth_file is not None:
  from prepare_box_roi_snapshot import complete_truth_messages
 out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
 rclpy.init();n=Capture(a.camera);start=time.monotonic();selected=None
 while time.monotonic()-start<a.seconds or selected is None or (n.clock and n.clock[-1]-stamp(selected[0])>250000000):
  if time.monotonic()-start>max(a.seconds+10,20):break
  rclpy.spin_once(n,timeout_sec=.1)
  if truth_file is not None:
   complete,truth_tail=complete_truth_messages(truth_tail+truth_file.read())
   for truth_stamp,_ in complete:
    if last_truth_stamp is not None and truth_stamp<=last_truth_stamp:
     raise RuntimeError('Truth clock repeated/regressed; snapshot epoch is invalid')
    truth_stamps.append(truth_stamp);last_truth_stamp=truth_stamp
  common=set(n.msgs['rgb'])&set(n.msgs['depth'])&set(n.msgs['info'])
  if truth_file is not None:common&=set(truth_stamps)
  for ts in sorted(common,reverse=True):
   if ts not in n.health or not n.health[ts]['valid']:continue
   rgb,dep,info=(n.msgs[k][ts] for k in ('rgb','depth','info'))
   if not(rgb.header.frame_id==dep.header.frame_id==info.header.frame_id==n.health[ts]['frame_id']):continue
   if not(rgb.width==dep.width==info.width and rgb.height==dep.height==info.height):continue
   try:
    tf=n.buffer.lookup_transform('astribot_torso_base',info.header.frame_id,Time.from_msg(info.header.stamp))
    odom=n.buffer.lookup_transform('odom',info.header.frame_id,Time.from_msg(info.header.stamp))
   except Exception:continue
   selected=(rgb,dep,info,tf,odom);break
 if truth_file is not None:truth_file.close()
 if selected is None:raise RuntimeError('No synchronized RGB/depth/info with TF and required truth stamp within bounded window')
 rgb,dep,info,tf,odom=selected
 color=image_data(rgb);depth=image_data(dep)
 if depth.ndim!=2:raise RuntimeError(f'depth encoding unsupported: {dep.encoding}')
 K=np.array(info.k).reshape(3,3)
 if np.any(np.abs(info.d)>1e-9):raise RuntimeError('Raw distortion present; requires undistortion before unprojection')
 hsv=cv2.cvtColor(color[:,:,:3],cv2.COLOR_RGB2HSV)
 mask=cv2.inRange(hsv,np.array([135,80,55]),np.array([175,255,255]))>0
 mask &= np.isfinite(depth)&(depth>.15)&(depth<4.5)
 mask=cv2.erode(mask.astype(np.uint8),np.ones((3,3),np.uint8),iterations=1)>0
 finite_depth=np.where(np.isfinite(depth),depth,np.nan)
 depth_for_projection=depth;depth=finite_depth
 v,u=np.indices(depth.shape);xyz=np.dstack(((u-K[0,2])*depth/K[0,0],(v-K[1,2])*depth/K[1,1],depth))
 dx=np.roll(xyz,-1,axis=1)-np.roll(xyz,1,axis=1);dy=np.roll(xyz,-1,axis=0)-np.roll(xyz,1,axis=0)
 normal=np.cross(dx,dy);length=np.linalg.norm(normal,axis=2)
 valid=mask & np.isfinite(length)&(length>1e-10)&(np.abs(dx[:,:,2])<.015)&(np.abs(dy[:,:,2])<.015)
 normal[valid]/=length[valid,None]
 flip=np.sum(normal*xyz,axis=2)>0;normal[flip]*=-1
 scene=np.column_stack((xyz[valid],normal[valid]))
 np.savetxt(out/'scene.xyz',scene,fmt='%.8f');np.save(out/'depth.npy',depth)
 cv2.imwrite(str(out/'rgb.png'),cv2.cvtColor(color[:,:,:3],cv2.COLOR_RGB2BGR));cv2.imwrite(str(out/'mask.png'),mask.astype(np.uint8)*255)
 safe=np.where(np.isfinite(depth),np.clip(depth/5,0,1),0);cv2.imwrite(str(out/'depth_visualization.png'),(safe*255).astype(np.uint8))
 health=n.health.get(stamp(rgb))
 metadata=dict(camera_id=a.camera,source_epoch=health['source_epoch'] if health else None,calibration_revision=health['calibration_revision'] if health else None,camera_health_at_capture=health,camera_health_samples=len(n.health),capture_age_at_end_sec=(n.clock[-1]-stamp(rgb))/1e9 if n.clock else None,schema_version=1,capture_stamp_ns=stamp(rgb),source='actual Gazebo rendered raw RGB-D',finite_depth_pixels=int(np.isfinite(depth_for_projection).sum()),camera=a.camera,frame=info.header.frame_id,width=info.width,height=info.height,K=list(info.k),D=list(info.d),P=list(info.p),R=list(info.r),distortion_model=info.distortion_model,encoding={'rgb':rgb.encoding,'depth':dep.encoding},exact_sync_stamps={k:stamp(m) for k,m in [('rgb',rgb),('depth',dep),('info',info)]},base_from_camera=transform_dict(tf),odom_from_camera=transform_dict(odom),target_mask_pixels=int(mask.sum()),scene_points=len(scene),segmentation='HSV magenta color + finite rendered depth; validation fixture; not YOLO',normal_method='organized depth central differences; toward camera; depth-edge rejection',truth_used_for_segmentation_or_projection=False,stream_counts=n.counts,stream_sim_hz={k:(len(ss)-1)/((ss[-1]-ss[0])/1e9) if len(ss)>1 else 0 for k,ss in n.stamps.items()},clock={'count':len(n.clock),'first_ns':n.clock[0] if n.clock else None,'last_ns':n.clock[-1] if n.clock else None},wall_observation_seconds=time.monotonic()-start)
 metadata['truth_stamp_filter']={'enabled':truth_file is not None,'file':str(a.truth_stream.resolve()) if a.truth_stream else None,'matched_stamp_ns':stamp(rgb) if truth_file is not None else None,'object_pose_used_for_projection':False}
 (out/'camera_info.json').write_text(json.dumps(metadata,indent=2));print(json.dumps(metadata,indent=2))
 n.destroy_node();rclpy.shutdown()
 if metadata['capture_age_at_end_sec'] is None or metadata['capture_age_at_end_sec']>.25:raise RuntimeError('Captured snapshot failed 250 ms freshness bound')
 if a.require_target and len(scene)<100:raise RuntimeError(f'Insufficient rendered target pixels: {len(scene)}')
if __name__=='__main__':main()
