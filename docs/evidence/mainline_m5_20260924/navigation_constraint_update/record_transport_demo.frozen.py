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
from rosgraph_msgs.msg import Clock


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
    frame_sidecar = output.with_suffix('.frames.jsonl').open('w', buffering=1)
    head, latest = [None], [None]
    head_metadata, observed_clock = [None], [None]
    status = {'stage': 'WAITING', 'object_state': 'WORLD'}
    status_metadata = [dict(receive_seq=0, raw_status=None,
                            receive_monotonic_ns=None, receive_wall_ns=None, observed_clock=None)]
    status_events = []
    terminal, count = [None], [0]
    stage_frames = []
    def receive_time():
        return dict(receive_monotonic_ns=time.monotonic_ns(), receive_wall_ns=time.time_ns(),
                    observed_clock=observed_clock[0])
    def clock_cb(message):
        observed_clock[0] = dict(source_ns=message.clock.sec*10**9+message.clock.nanosec,
                                 receive_monotonic_ns=time.monotonic_ns(), receive_wall_ns=time.time_ns())
    def frame(message):
        if message.encoding != 'rgb8':
            raise RuntimeError('Expected rgb8')
        return np.ndarray((message.height, message.width, 3), np.uint8,
                          buffer=bytes(message.data), strides=(message.step,3,1))[:,:,::-1].copy()
    def head_cb(message):
        metadata = receive_time()
        metadata.update(source_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                        frame_id=message.header.frame_id, topic=args.head_topic)
        head[0] = frame(message)
        head_metadata[0] = metadata
    def status_cb(message):
        metadata = receive_time()
        raw = json.loads(message.data)
        metadata.update(receive_seq=status_metadata[0]['receive_seq']+1, raw_status=raw)
        status_metadata[0] = metadata
        status_events.append(metadata)
        incoming = (dict(stage=raw['reason'], object_state='phase='+raw['phase']+
                         ' | hold_confirmed='+str(raw['hold_confirmed']))
                    if args.status_format == 'native_hold' else raw)
        if incoming['stage'] != status['stage']:
            stage_frames.append(dict(frame=count[0], stage=incoming['stage'],
                                     wall_time=metadata['receive_wall_ns']/1e9, raw_status=raw,
                                     status_receive_seq=metadata['receive_seq']))
        status.update(incoming)
        if args.status_format == 'legacy' and status['stage'] in ('SUCCEEDED', 'FAULT', 'CANCELED'):
            terminal[0] = terminal[0] or time.monotonic()
    def overview_cb(message):
        metadata = receive_time()
        metadata.update(source_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                        frame_id=message.header.frame_id, topic='/transport/overview')
        picture = frame(message)
        if head[0] is not None:
            picture[460:700,940:1260] = cv2.resize(head[0], (320,240))
            cv2.putText(picture, 'HEAD RGB - sensor input', (942,450), cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
        cv2.rectangle(picture,(0,0),(1280,77),(24,24,24),-1)
        cv2.putText(picture, args.title, (24,30), cv2.FONT_HERSHEY_SIMPLEX,.8,(240,240,240),2,cv2.LINE_AA)
        cv2.putText(picture, status['stage']+'  |  '+status.get('object_state',''), (24,62), cv2.FONT_HERSHEY_SIMPLEX,.7,(120,220,255),2,cv2.LINE_AA)
        cv2.putText(picture, 'Kinematic attachment simulation | NOT force/contact validation', (24,700),cv2.FONT_HERSHEY_SIMPLEX,.5,(240,240,240),1,cv2.LINE_AA)
        encode_started = time.monotonic_ns()
        writer.write(picture)
        encode_finished = time.monotonic_ns()
        # OpenCV reports no successful frame count; decode and compare after capture.
        frame_sidecar.write(json.dumps(dict(frame_index=count[0], overview=metadata,
            head=head_metadata[0], status=status_metadata[0],
            encode_started_monotonic_ns=encode_started,
            encode_finished_monotonic_ns=encode_finished), separators=(',', ':'))+'\n')
        count[0] += 1;latest[0]=picture
        if count[0]%20 == 0:
            cv2.imwrite(str(output.with_suffix('.jpg')), picture)
    subscriptions = [node.create_subscription(Image,args.head_topic,head_cb,qos_profile_sensor_data),
                     node.create_subscription(Image,'/transport/overview',overview_cb,qos_profile_sensor_data),
                     node.create_subscription(String,args.status_topic,status_cb,10),
                     node.create_subscription(Clock,'/clock',clock_cb,qos_profile_sensor_data)]
    start=time.monotonic()
    try:
        while time.monotonic()-start < args.timeout and (terminal[0] is None or time.monotonic()-terminal[0]<2):
            rclpy.spin_once(node,timeout_sec=.1)
    finally:
        writer.release()
        frame_sidecar.close()
        if latest[0] is not None:cv2.imwrite(str(output.with_suffix('.jpg')),latest[0])
        output.with_suffix('.json').write_text(json.dumps(dict(frames=count[0],encoding_fps=10,wall_duration_s=time.monotonic()-start,last_status=status,stage_frames=stage_frames,status_events=status_events,source='actual_gazebo_images',status_topic=args.status_topic,status_format=args.status_format,task_id=args.task_id,timing='10 fps received frames; not a wall-time performance measurement',frame_timing_schema='astribot.video_frame_timing/1',frame_sidecar=str(output.with_suffix('.frames.jsonl')),frame_count_semantics='completed writer.write calls; decode verification required',clock_semantics='observed_clock is latest received /clock and its receive times; null before first clock'),indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__ == '__main__':main()
