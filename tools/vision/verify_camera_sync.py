#!/usr/bin/env python3
"""Read-only raw RGB-D/health/clock evidence; never declares geometry validity."""
import argparse,collections,json,os,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from rosgraph_msgs.msg import Clock
from astribot_perception_msgs.msg import CameraHealth

parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[2]/'docs/evidence/grasp_pose_sim_20260921/health_sync')
parser.add_argument('--seconds',type=float,default=10.)
args=parser.parse_args()
if not 0 < args.seconds <= 300:raise ValueError('seconds must be in (0,300]')
out=args.output
out.mkdir(parents=True,exist_ok=True)
phase=os.environ.get('HEALTH_SYNC_PHASE','before')
if not phase.replace('_','').isalnum(): raise ValueError('invalid phase')
rclpy.init(); node=Node('readonly_camera_health_sync_probe')
start=time.monotonic(); rows=[]; clocks=[]; latest={}; streams=collections.defaultdict(list); health=collections.defaultdict(list); subs=[]
def ns(stamp):return stamp.sec*1000000000+stamp.nanosec

def sensor(camera,kind):
    def callback(msg):
        wall=time.monotonic()-start
        row={'kind':kind,'camera':camera,'wall_elapsed':wall,'stamp_ns':ns(msg.header.stamp),'frame':msg.header.frame_id,'width':msg.width,'height':msg.height}
        if isinstance(msg,Image):row.update(encoding=msg.encoding,step=msg.step,data_bytes=len(msg.data))
        latest[(camera,kind)]=row['stamp_ns']; streams[(camera,kind)].append(row);rows.append(row)
    return callback

def state(camera):
    def callback(msg):
        row={'kind':'health','camera':camera,'wall_elapsed':time.monotonic()-start,'stamp_ns':ns(msg.header.stamp),'capture_ns':ns(msg.capture_stamp),'valid':msg.valid,'state':msg.state,'reason':msg.reason_code,'age_sec':msg.age_sec,'skew_sec':msg.sync_skew_sec,'hz':msg.frequency_hz,'max_interval_sec':msg.max_interval_sec,'epoch':msg.source_epoch,'latest_observed':{k:latest.get((camera,k)) for k in ('rgb','depth','info')}}
        health[camera].append(row);rows.append(row)
    return callback

def clock(msg):clocks.append((time.monotonic()-start,ns(msg.clock)))
subs.append(node.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data))
for camera in ('head_rgbd','torso_rgbd'):
    for kind,topic,typ in [('rgb','image',Image),('depth','depth_image',Image),('info','camera_info',CameraInfo)]:
        subs.append(node.create_subscription(typ,f'/camera/raw/{camera}/{topic}',sensor(camera,kind),qos_profile_sensor_data))
    subs.append(node.create_subscription(CameraHealth,f'/perception/camera_health/{camera}',state(camera),10))
while time.monotonic()-start<args.seconds:rclpy.spin_once(node,timeout_sec=.02)
summary={'duration_wall_s':time.monotonic()-start,'ROS_DOMAIN_ID':os.getenv('ROS_DOMAIN_ID'),'ROS_LOCALHOST_ONLY':os.getenv('ROS_LOCALHOST_ONLY'),'clock_messages':len(clocks),'clock_advanced_sec':(clocks[-1][1]-clocks[0][1])/1e9 if len(clocks)>1 else 0,'cameras':{}}
for camera in ('head_rgbd','torso_rgbd'):
    data={'health_count':len(health[camera]),'states':dict(collections.Counter(r['state'] for r in health[camera])),'streams':{}}
    for kind in ('rgb','depth','info'):
        observations=streams[(camera,kind)]
        intervals=[(b['stamp_ns']-a['stamp_ns'])/1e9 for a,b in zip(observations,observations[1:])]
        wall=[b['wall_elapsed']-a['wall_elapsed'] for a,b in zip(observations,observations[1:])]
        data['streams'][kind]={'messages':len(observations),'header_interval_min_s':min(intervals) if intervals else None,'header_interval_max_s':max(intervals) if intervals else None,'wall_interval_max_s':max(wall) if wall else None}
    sets={k:{r['stamp_ns'] for r in streams[(camera,k)]} for k in ('rgb','depth','info')}
    data['exact_stamp_triples']=len(sets['rgb']&sets['depth']&sets['info'])
    summary['cameras'][camera]=data
(out/f'{phase}_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(out/f'{phase}_events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
(out/f'{phase}_clock.json').write_text(json.dumps(clocks)+'\n')
print(json.dumps(summary,indent=2))
node.destroy_node();rclpy.shutdown()
