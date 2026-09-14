#!/usr/bin/env python3
"""Bounded ROS readiness probe; parent supervisor enforces an independent wall timeout."""
import argparse
import json
import time
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer, TransformListener


def main():
    p=argparse.ArgumentParser();p.add_argument('--phase',choices=['data','navigation'],required=True)
    p.add_argument('--timeout',type=float,default=120);p.add_argument('--scan',default='/scan_from_cloud')
    p.add_argument('--costmap-scan');a=p.parse_args();costmap_scan=a.costmap_scan or a.scan
    rclpy.init();n=rclpy.create_node('sim_startup_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    counts={'clock':0,'odom':0,'scan':0};received={};ticks=[]
    def data(key,msg):
        counts[key]+=1;received[key]=time.monotonic()
        if key=='clock':ticks.append(msg.clock.sec+msg.clock.nanosec*1e-9)
    for cls,topic,key in [(Clock,'/clock','clock'),(Odometry,'/odom','odom'),(LaserScan,a.scan,'scan')]:
        n.create_subscription(cls,topic,lambda m,k=key:data(k,m),qos_profile_sensor_data)
    if a.phase=='navigation' and costmap_scan!=a.scan:
        counts['costmap_scan']=0
        n.create_subscription(LaserScan,costmap_scan,lambda m:data('costmap_scan',m),qos_profile_sensor_data)
    buffer=Buffer();listener=TransformListener(buffer,n)
    names=['controller_server','smoother_server','planner_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother'] if a.phase=='navigation' else []
    clients={name:n.create_client(GetState,'/'+name+'/get_state') for name in names};pending={};states={};next_call={}
    scan_clients={name:n.create_client(GetParameters,'/'+name+'/'+name+'/get_parameters') for name in ['global_costmap','local_costmap']} if names else {}
    scan_pending={};scan_values={}
    start=time.monotonic();ready=False;age=None;owners={};execution_nodes={};bt_clock_nodes={}
    while time.monotonic()-start<a.timeout:
        rclpy.spin_once(n,timeout_sec=.05);now=time.monotonic()
        for name,c in clients.items():
            item=pending.get(name)
            if item:
                f,created=item
                if f.done():
                    states[name]=f.result().current_state.label;del pending[name];next_call[name]=now+1
                elif now-created>2:
                    c.remove_pending_request(f);del pending[name];next_call[name]=now+1
            if name not in pending and now>=next_call.get(name,0) and c.service_is_ready():
                pending[name]=(c.call_async(GetState.Request()),now)
        if all(states.get(name)=='active' for name in names):
            for name,c in scan_clients.items():
                item=scan_pending.get(name)
                if item:
                    f,created=item
                    if f.done():
                        scan_values[name]=f.result().values[0].string_value;del scan_pending[name]
                    elif now-created>2:
                        c.remove_pending_request(f);del scan_pending[name]
                if name not in scan_pending and scan_values.get(name)!=costmap_scan and c.service_is_ready():
                    scan_pending[name]=(c.call_async(GetParameters.Request(names=['obstacle_layer.scan.topic'])),now)
        try:
            tf=buffer.lookup_transform('map','astribot_torso_base',rclpy.time.Time())
            age=n.get_clock().now().nanoseconds*1e-9-(tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9)
            ready=(now-start>=3 and counts['scan']>=10 and counts['odom']>=30 and len(ticks)>10 and ticks[-1]>ticks[0]
                   and -.05<=age<.5 and len(received)==len(counts) and all(now-v<.5 for v in received.values())
                   and counts.get('costmap_scan',10)>=10
                   and all(states.get(name)=='active' for name in names)
                   and all(scan_values.get(name)==costmap_scan for name in scan_clients))
        except Exception:
            ready=False
        if ready and names:
            owners={}
            nodes=n.get_node_names_and_namespaces()
            execution_nodes={name:(name,'/') in nodes for name in
                             ('cmd_vel_body_to_world_node','arm_speed_limiter_node','arm_chassis_speed_coupling_node')}
            clock_subscribers={item.node_name for item in n.get_subscriptions_info_by_topic('/clock')}
            bt_clock_nodes={name:name in clock_subscribers for name in
                            ('bt_navigator_navigate_to_pose_rclcpp_node','bt_navigator_navigate_through_poses_rclcpp_node')}
            for node_name,node_ns in nodes:
                for service,types in n.get_service_names_and_types_by_node(node_name,node_ns):
                    if service.endswith('/_action/send_goal'):
                        owners.setdefault(service,[]).append(node_ns.rstrip('/')+'/'+node_name)
            ready=all(execution_nodes.values()) and all(bt_clock_nodes.values()) and all(owners.get('/'+action+'/_action/send_goal')==['/navigation_task_arbiter'] and
                      owners.get('/navigation_executor/'+action+'/_action/send_goal')==['/navigation_executor/bt_navigator']
                      for action in ('navigate_to_pose','navigate_through_poses'))
        if ready:break
    print(json.dumps({'ready':ready,'phase':a.phase,'sample_seconds':time.monotonic()-start,'counts':counts,
                      'execution_nodes':execution_nodes,'bt_clock_nodes':bt_clock_nodes,'task_endpoint_owners':owners,'tf_age':age,'scan_topics':scan_values,'clock_range':[ticks[0],ticks[-1]] if ticks else [],'lifecycle':states}),flush=True)
    n.destroy_node();rclpy.shutdown()
    return 0 if ready else 1

if __name__=='__main__':
    raise SystemExit(main())
