#!/usr/bin/env python3
"""Read-only, bounded map/scan/cloud snapshot for paired simulator view review."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def matrix(transform):
    q, p = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]), np.array([p.x,p.y,p.z])


def grid_xy(message, mask):
    y, x = np.where(mask)
    points = np.c_[(x+.5)*message.info.resolution, (y+.5)*message.info.resolution, np.zeros(len(x))]
    # OccupancyGrid origin is a pose, including rotation.
    origin = message.info.origin
    class Transform:
        rotation = origin.orientation
        translation = origin.position
    rotation, translation = matrix(Transform)
    return points@rotation.T+translation


def cloud_xyz(message):
    fields = {f.name: f for f in message.fields}
    if any(k not in fields or fields[k].datatype != 7 or fields[k].count != 1 for k in 'xyz'):
        raise ValueError('Expected scalar FLOAT32 x/y/z fields')
    dtype = np.dtype(dict(names=list('xyz'), formats=[('>' if message.is_bigendian else '<')+'f4']*3,
                          offsets=[fields[k].offset for k in 'xyz'], itemsize=message.point_step))
    points = np.ndarray((message.height,message.width),dtype,buffer=message.data,
                        strides=(message.row_step,message.point_step))
    xyz = np.c_[tuple(points[k].ravel() for k in 'xyz')]
    return xyz[np.isfinite(xyz).all(axis=1)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--wall-seconds',type=float,default=20.)
    parser.add_argument('--scan-topic',default='/scan_from_cloud')
    args = parser.parse_args()
    if not 3 <= args.wall_seconds <= 60:
        parser.error('Snapshot wall duration must be 3..60 seconds')
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node = rclpy.create_node('social_map_snapshot')
    buffer = Buffer(); listener = TransformListener(buffer,node)
    latest, counts = {}, {}
    def receive(topic,message):
        latest[topic]=(message,node.get_clock().now().nanoseconds*1e-9)
        counts[topic]=counts.get(topic,0)+1
    durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    topics = {'/map':OccupancyGrid, '/global_costmap/costmap':OccupancyGrid,
              '/local_costmap/costmap':OccupancyGrid, args.scan_topic:LaserScan,
              '/navigation_policy/costmap_scan':LaserScan,
              '/livox/lidar_left':PointCloud2, '/livox/lidar_right':PointCloud2,
              '/livox/cloud_self_filtered':PointCloud2, '/map_scan_filtered':PointCloud2,
              '/map_cmap':PointCloud2, '/social_sim/state':String}
    subscriptions=[node.create_subscription(kind,topic,lambda m,t=topic:receive(t,m),
                   durable if kind is OccupancyGrid else qos_profile_sensor_data) for topic,kind in topics.items()]
    started=time.monotonic()
    while time.monotonic()-started<args.wall_seconds:
        rclpy.spin_once(node,timeout_sec=.02)
    now=node.get_clock().now().nanoseconds*1e-9
    report={'boundary':'Diagnostic snapshot only; counts outside static obstacles do not prove stale human residue.',
            'environment':{k:os.environ.get(k) for k in ('ROS_DOMAIN_ID','IGN_PARTITION')},
            'ros_s':now,'counts':counts,'missing_topics':sorted(set(topics)-set(latest)),'topics':{}}
    arrays={}
    for topic,(message,received) in latest.items():
        key=topic.strip('/').replace('/','_')
        raw=serialize_message(message); (args.output/(key+'.cdr')).write_bytes(raw)
        record={'ros_type':type(message).__module__+'.'+type(message).__name__,
                'received_ros_s':received,'sha256':hashlib.sha256(raw).hexdigest()}
        report['topics'][topic]=record
        if isinstance(message,String):
            record['state']=json.loads(message.data); continue
        source=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        record.update(source_s=source,age_s=now-source,frame=message.header.frame_id)
        try:
            frame=message.header.frame_id
            if frame=='map':rotation,translation=np.eye(3),np.zeros(3)
            else:
                transform=buffer.lookup_transform('map',frame,Time.from_msg(message.header.stamp))
                rotation,translation=matrix(transform.transform)
            record['map_transform']={'rotation':rotation.tolist(),'translation':translation.tolist()}
            if isinstance(message,OccupancyGrid):
                values=np.asarray(message.data,dtype=np.int16).reshape(message.info.height,message.info.width)
                arrays[key+'_occupied']=grid_xy(message,values>=100)@rotation.T+translation
                record['value_counts']={str(int(v)):int(n) for v,n in zip(*np.unique(values,return_counts=True))}
                if topic=='/map':arrays[key+'_free']=grid_xy(message,values==0)@rotation.T+translation
            else:
                if isinstance(message,LaserScan):
                    distance=np.asarray(message.ranges)
                    angles=message.angle_min+np.arange(len(distance))*message.angle_increment
                    valid=np.isfinite(distance)&(distance>=message.range_min)&(distance<=message.range_max)
                    points=np.c_[distance[valid]*np.cos(angles[valid]),distance[valid]*np.sin(angles[valid]),np.zeros(valid.sum())]
                else:points=cloud_xyz(message)
                arrays[key]=points@rotation.T+translation
                record['finite_points']=len(points)
        except Exception as error:
            record['transform_or_decode_error']=str(error)
    np.savez_compressed(args.output/'map_coordinates.npz',**arrays)
    (args.output/'snapshot.json').write_text(json.dumps(report,indent=2)+'\n')
    node.destroy_node();rclpy.shutdown()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(18,6),constrained_layout=True)
    static=arrays.get('map_occupied',np.empty((0,3)))
    for ax,key in zip(axes,('global_costmap_costmap_occupied','local_costmap_costmap_occupied',
                            args.scan_topic.strip('/').replace('/','_'))):
        ax.scatter(static[:,0],static[:,1],s=2,c='black',label='static occupied')
        points=arrays.get(key,np.empty((0,3)))
        ax.scatter(points[:,0],points[:,1],s=3,c='#d65a31',label=key)
        ax.set_title(key);ax.set_aspect('equal');ax.set_xlim(-3,5);ax.set_ylim(-4,4)
        ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)');ax.legend(loc='upper left',fontsize=7)
    fig.savefig(args.output/'map_comparison.png',dpi=150);plt.close(fig)
    print(json.dumps({'snapshot':str(args.output),'missing_topics':report['missing_topics']}))


if __name__=='__main__':main()
