#!/usr/bin/env python3
"""Run isolated projection candidates on live raw camera data; no robot commands."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, PointCloud2


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=30)
    a=p.parse_args()
    if not 1<=a.seconds<=1800:raise ValueError('bounded probe: 1..1800 seconds')
    a.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=Node('readonly_cloud_pair_probe',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    cameras=('head_rgbd','torso_rgbd','left_wrist_rgbd','right_wrist_rgbd')
    processes=[];logs=[];subs=[];observed=defaultdict(set);clouds=defaultdict(list)
    def capture(camera,kind):
        def cb(msg):
            key=(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,msg.header.frame_id)
            observed[camera,kind].add(key)
            if kind=='cloud':clouds[camera].append((key,(node.get_clock().now().nanoseconds-key[0])*1e-9))
        return cb
    try:
        for camera in cameras:
            raw=f'/camera/raw/{camera}'
            topic=f'/validation/paired_cloud/{camera}'
            for kind,name,typ in [('depth',raw+'/depth_image',Image),('info',raw+'/camera_info',CameraInfo),('cloud',topic,PointCloud2)]:
                subs.append(node.create_subscription(typ,name,capture(camera,kind),qos_profile_sensor_data))
            log=(a.output/(camera+'.log')).open('w');logs.append(log)
            cmd=[str(a.binary.resolve()),'--ros-args','-r',f'__node:={camera}_projection_candidate','-p','use_sim_time:=true',
                 '-p',f'depth_topic:={raw}/depth_image','-p',f'camera_info_topic:={raw}/camera_info',
                 '-p',f'output_topic:={topic}','-p','decimation:=4']
            processes.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT))
        start=time.monotonic()
        while time.monotonic()-start<a.seconds:rclpy.spin_once(node,timeout_sec=.01)
        rows=[]
        for camera in cameras:
            entries=clouds[camera];keys=observed[camera,'cloud'];paired=observed[camera,'depth']&observed[camera,'info']
            ages=[v[1] for v in entries]
            rows.append(dict(camera=camera,clouds=len(entries),distinct=len(keys),unpaired=len(keys-paired),
                             max_receive_age_sec=max(ages) if ages else None,
                             pass_pairing=bool(entries) and len(keys)==len(entries) and not keys-paired))
        result=dict(scope='live static Gazebo, four C++ projection candidates; no MPPI/SLAM/inference load',
                    duration=time.monotonic()-start,rows=rows,passed=all(r['pass_pairing'] for r in rows))
        (a.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
        return 0 if result['passed'] else 1
    finally:
        for proc in processes:
            if proc.poll() is None:proc.terminate()
        for proc in processes:
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
