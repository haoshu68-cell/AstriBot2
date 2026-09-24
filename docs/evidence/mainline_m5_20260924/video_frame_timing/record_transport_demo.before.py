#!/usr/bin/env python3
"""Record actual Gazebo overview and head RGB frames with live task stage labels."""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=600.)
    parser.add_argument('--head-topic', default='/camera/color/image_raw')
    parser.add_argument('--status-topic', default='/transport/status')
    parser.add_argument('--status-format', choices=('legacy', 'native_hold'), default='legacy')
    parser.add_argument('--title', default='ASTRIBOT | Gazebo pick - carry - place')
    parser.add_argument('--task-id', default='')
    args = parser.parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('transport_demo_recorder')
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*'mp4v'), 10., (1280, 720))
    if not writer.isOpened():
        raise RuntimeError('Cannot open video writer')
    head, latest = [None], [None]
    status = {'stage': 'WAITING', 'object_state': 'WORLD'}
    terminal, count = [None], [0]
    stage_frames = []
    def frame(message):
        if message.encoding != 'rgb8':
            raise RuntimeError('Expected rgb8')
        return np.ndarray((message.height, message.width, 3), np.uint8,
                          buffer=bytes(message.data), strides=(message.step,3,1))[:,:,::-1].copy()
    def status_cb(message):
        raw = json.loads(message.data)
        incoming = (dict(stage=raw['reason'], object_state='phase='+raw['phase']+
                         ' | hold_confirmed='+str(raw['hold_confirmed']))
                    if args.status_format == 'native_hold' else raw)
        if incoming['stage'] != status['stage']:
            stage_frames.append(dict(frame=count[0], stage=incoming['stage'], wall_time=time.time(), raw_status=raw))
        status.update(incoming)
        if args.status_format == 'legacy' and status['stage'] in ('SUCCEEDED', 'FAULT', 'CANCELED'):
            terminal[0] = terminal[0] or time.monotonic()
    def overview_cb(message):
        picture = frame(message)
        if head[0] is not None:
            picture[460:700,940:1260] = cv2.resize(head[0], (320,240))
            cv2.putText(picture, 'HEAD RGB - sensor input', (942,450), cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
        cv2.rectangle(picture,(0,0),(1280,77),(24,24,24),-1)
        cv2.putText(picture, args.title, (24,30), cv2.FONT_HERSHEY_SIMPLEX,.8,(240,240,240),2,cv2.LINE_AA)
        cv2.putText(picture, status['stage']+'  |  '+status.get('object_state',''), (24,62), cv2.FONT_HERSHEY_SIMPLEX,.7,(120,220,255),2,cv2.LINE_AA)
        cv2.putText(picture, 'Kinematic attachment simulation | NOT force/contact validation', (24,700),cv2.FONT_HERSHEY_SIMPLEX,.5,(240,240,240),1,cv2.LINE_AA)
        writer.write(picture);count[0] += 1;latest[0]=picture
        if count[0]%20 == 0:
            cv2.imwrite(str(output.with_suffix('.jpg')), picture)
    subscriptions = [node.create_subscription(Image,args.head_topic,lambda m:head.__setitem__(0,frame(m)),qos_profile_sensor_data),
                     node.create_subscription(Image,'/transport/overview',overview_cb,qos_profile_sensor_data),
                     node.create_subscription(String,args.status_topic,status_cb,10)]
    start=time.monotonic()
    try:
        while time.monotonic()-start < args.timeout and (terminal[0] is None or time.monotonic()-terminal[0]<2):
            rclpy.spin_once(node,timeout_sec=.1)
    finally:
        writer.release()
        if latest[0] is not None:cv2.imwrite(str(output.with_suffix('.jpg')),latest[0])
        output.with_suffix('.json').write_text(json.dumps(dict(frames=count[0],encoding_fps=10,wall_duration_s=time.monotonic()-start,last_status=status,stage_frames=stage_frames,source='actual_gazebo_images',status_topic=args.status_topic,status_format=args.status_format,task_id=args.task_id,timing='10 fps received frames; not a wall-time performance measurement'),indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__ == '__main__':main()
