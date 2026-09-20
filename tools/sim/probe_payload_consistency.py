#!/usr/bin/env python3
"""Read-only comparison of native Gazebo payload and capture-time MoveIt/TF poses.

The canonical warehouse uses the Gazebo world coordinates for its map. Keep
the native frame and stamp in the evidence; never substitute receipt time.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from control_msgs.msg import JointTrajectoryControllerState
from action_msgs.msg import GoalStatusArray
from astribot_transport_msgs.msg import ExecutionGuardStatus
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
from astribot_s1_transport.ros_backend import matrix, pose_at, seconds
from astribot_s1_transport.geometry import collision_primitive_matrix
from rosidl_runtime_py.convert import message_to_ordereddict
from scipy.spatial.transform import Rotation


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--duration',type=float,default=240.)
    parser.add_argument('--object-id',default='transport_box_01')
    parser.add_argument('--tf-wait',type=float,default=2.,help='Bounded wall wait for capture-time TF; never extrapolate')
    args=parser.parse_args()
    if any(not math.isfinite(v) or v<=0 for v in (args.duration,args.tf_wait)):parser.error('invalid duration/wait')
    output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    rclpy.init()
    node=rclpy.create_node('payload_consistency_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node)
    pending=deque();scene=None;latest_scan=0.;samples=[];received=0;failures={};physics_samples=[]
    stream=(output/'poses.jsonl').open('w')
    sources=(output/'sources.jsonl').open('w')
    def record_source(kind,msg):
        sources.write(json.dumps(dict(kind=kind,received_wall_s=time.time(),received_ros_s=node.get_clock().now().nanoseconds*1e-9,
            message=message_to_ordereddict(msg)),separators=(',',':'))+'\n')
    def receive_scene(msg):
        nonlocal scene
        scene=msg
    def receive_scan(msg):
        nonlocal latest_scan
        latest_scan=seconds(msg.header.stamp)
    def receive_pose(msg):
        nonlocal received
        received+=1
        pending.append((time.monotonic(),msg,scene,latest_scan,None))
    def receive_physics(msg):
        state=json.loads(msg.data)
        record_source('physics',msg)
        if not state.get('attached') or 'expected_world_xyzw' not in state:return
        p=PoseStamped();p.header.frame_id='default'
        ns=int(state['stamp_ns']);p.header.stamp.sec=ns//1000000000;p.header.stamp.nanosec=ns%1000000000
        v=state['actual_world_xyzw'];p.pose=pose_at(v[:3],v[3:])
        pending.append((time.monotonic(),p,scene,latest_scan,state))
    node.create_subscription(PlanningScene,'/navigation/attached_geometry',receive_scene,10)
    node.create_subscription(PoseStamped,'/simulation/transport_payload_pose',receive_pose,100)
    node.create_subscription(LaserScan,'/scan_from_cloud',receive_scan,qos_profile_sensor_data)
    node.create_subscription(String,'/model/'+args.object_id+'/kinematic_attachment/state',receive_physics,100)
    node.create_subscription(TFMessage,'/tf',lambda m:record_source('tf',m),qos_profile_sensor_data)
    node.create_subscription(TFMessage,'/tf_static',lambda m:record_source('tf_static',m),
        QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(JointState,'/joint_states',lambda m:record_source('joints',m),qos_profile_sensor_data)
    node.create_subscription(JointTrajectoryControllerState,'/arm_left_controller/state',
        lambda m:record_source('arm_controller',m),qos_profile_sensor_data)
    node.create_subscription(ExecutionGuardStatus,'/transport/execution_guard/status',
        lambda m:record_source('execution_guard',m),10)
    node.create_subscription(GoalStatusArray,'/execute_trajectory/_action/status',
        lambda m:record_source('execution_action',m),QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    start=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start<args.duration:
            rclpy.spin_once(node,timeout_sec=.01)
            while pending:
                received_at,msg,snapshot,scan_stamp,physics=pending[0]
                wait=time.monotonic()-received_at
                objects=[] if snapshot is None else [o for o in snapshot.robot_state.attached_collision_objects if o.object.id==args.object_id]
                if len(objects)==1 and wait<args.tf_wait and not buffer.can_transform(
                        'map',objects[0].link_name,Time.from_msg(msg.header.stamp)):
                    break
                pending.popleft()
                row={'native':message_to_ordereddict(msg),'latest_scan_stamp':scan_stamp,
                    'source':'physics_post_update' if physics else 'pose_publisher','tf_wait_wall_s':wait}
                try:
                    stamp=seconds(msg.header.stamp)
                    if stamp<=0 or msg.header.frame_id not in ('default','world/default'):
                        raise ValueError('invalid native stamp/frame')
                    if snapshot is None:raise ValueError('no attached snapshot')
                    if len(objects)!=1:raise ValueError('not attached')
                    obj=objects[0]
                    row['attachment_revision']=snapshot.name
                    row['scene_stamp']=seconds(snapshot.robot_state.joint_state.header.stamp)
                    if len(obj.object.primitives)!=1:raise ValueError('expected one payload primitive')
                    tf=buffer.lookup_transform('map',obj.link_name,Time.from_msg(msg.header.stamp))
                    t,q=tf.transform.translation,tf.transform.rotation
                    expected=matrix(pose_at([t.x,t.y,t.z],[q.x,q.y,q.z,q.w]))@collision_primitive_matrix(obj.object)
                    actual=matrix(msg.pose)
                    delta=np.linalg.inv(expected)@actual
                    row.update(expected_matrix=expected.tolist(),delta_local_m=delta[:3,3].tolist(),
                        translation_error_m=float(np.linalg.norm(delta[:3,3])),
                        rotation_error_rad=float(Rotation.from_matrix(delta[:3,:3]).magnitude()))
                    if physics:
                        v=physics['expected_world_xyzw'];native_expected=matrix(pose_at(v[:3],v[3:]))
                        parent_delta=np.linalg.inv(expected)@native_expected
                        row.update(physics=physics,
                            tf_target_vs_physics_target_m=float(np.linalg.norm(parent_delta[:3,3])),
                            tf_target_vs_physics_target_rad=float(Rotation.from_matrix(parent_delta[:3,:3]).magnitude()))
                        physics_samples.append(row['tf_target_vs_physics_target_m'])
                    else:samples.append(row['translation_error_m'])
                except Exception as error:
                    row['unavailable']=str(error)
                    key=str(error).split('\n')[0];failures[key]=failures.get(key,0)+1
                stream.write(json.dumps(row,separators=(',',':'))+'\n')
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        stream.close();sources.close()
        result=dict(received=received,compared=len(samples),failures=failures,
            pending_at_shutdown=len(pending),
            translation_error_m={str(q):float(np.percentile(samples,q)) for q in (50,95,99,100)} if samples else {},
            physics_target_compared=len(physics_samples),
            tf_target_vs_physics_target_m={str(q):float(np.percentile(physics_samples,q)) for q in (50,95,99,100)} if physics_samples else {})
        (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
