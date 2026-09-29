#!/usr/bin/env python3
"""Stationary owned-world fixtures: observe real cloud -> slice -> costmap.

This measures finite test targets, not volumetric free-space visibility. In
particular, a configured z_max or VoxelLayer cannot certify an occluded volume.
"""
import argparse
import json
import math
import os
from pathlib import Path
import threading
import time
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan,PointCloud2
from sensor_msgs_py import point_cloud2
from nav2_msgs.srv import GetCostmap
from astribot_s1_transport.core import Ledger,ResourceLease,TaskFailure,validate_scenario
from astribot_s1_transport.ros_backend import RosBackend,seconds
from verify_fixed_hold_expiry import remove_owned

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario',required=True);p.add_argument('--output',required=True)
    # fixed_v2 deliberately consumes the full-height /map_scan stream and
    # performs navigation slicing afterwards.  The filtered stream is kept as
    # an explicit option for regression comparison; using it as the default
    # would silently re-introduce Voxel-SLAM's legacy 1.534 m cap.
    p.add_argument('--input-cloud',default='/map_scan')
    p.add_argument('--projection-z-min',type=float,default=-0.03)
    p.add_argument('--projection-z-max',type=float,default=2.2,
                   help='fixed_v2 navigation slice upper bound in metres')
    a=p.parse_args()
    if not os.environ.get('ASTRIBOT_SIM_INSTANCE'):p.error('owned isolated instance required')
    out=Path(a.output)
    if out.exists():p.error('new evidence directory required')
    config=json.loads(Path(a.scenario).read_text());validate_scenario(config)
    config['navigation_geometry_mode']='fixed_v2';rclpy.init()
    with ResourceLease('/tmp/astribot_transport_domain_'+os.environ['ROS_DOMAIN_ID']+'.lock') as lease:
        lease.file.seek(0);old=lease.file.read()
        if old and json.loads(old).get('unconfirmed_executor'):raise TaskFailure('PREVIOUS_EXECUTOR_UNCONFIRMED')
        node=RosBackend(config,Ledger(out,config['object_id']))
        clouds={}
        scans={}
        for name,topic in [('input',a.input_cloud),('filtered','/livox/cloud_self_filtered')]:
            node.create_subscription(PointCloud2,topic,lambda m,n=name:clouds.update({n:m}),qos_profile_sensor_data)
        node.create_subscription(LaserScan,'/scan_from_cloud',lambda m:scans.update({'scan':m}),qos_profile_sensor_data)
        ex=MultiThreadedExecutor(num_threads=3);ex.add_node(node)
        spinner=threading.Thread(target=ex.spin,daemon=True);spinner.start()
        rows=[];owned=[];report=dict(evidence='stationary_finite_height_targets',input_cloud=a.input_cloud,
            projection_z_min_m=a.projection_z_min,projection_z_max_m=a.projection_z_max,cases=rows)
        try:
            # Cloud delivery and the static map may become available before
            # the dynamic map->base TF chain.  Do not sample or project a
            # fixture until that chain is connected; otherwise a transient
            # TF tree split is reported as a geometry failure.
            node.wait(lambda:node.stopped() and node.envelope is not None and
                      all(k in clouds for k in ('input','filtered')) and
                      node.tf.can_transform(config['map_frame'], config['base_frame'],
                                            rclpy.time.Time()),20.)
            node.change_envelope(False)
            if node.scene().robot_state.attached_collision_objects:raise TaskFailure('ATTACHMENT_PRESENT')
            base=node.transform(config['map_frame'],config['base_frame'])
            yaw=math.atan2(base[1,0],base[0,0]);x=base[0,3]+2.*math.cos(yaw);y=base[1,3]+2.*math.sin(yaw)
            target_base=np.linalg.inv(base)@np.array([x,y,0.,1.])
            target_range=float(math.hypot(target_base[0],target_base[1]))
            target_angle=float(math.atan2(target_base[1],target_base[0]))
            query=node.create_client(GetCostmap,'/global_costmap/get_costmap')
            def cost():
                g=node.call(query,GetCostmap.Request()).map;m=g.metadata
                ix=math.floor((x-m.origin.position.x)/m.resolution);iy=math.floor((y-m.origin.position.y)/m.resolution)
                if ix<4 or iy<4 or ix>=m.size_x-4 or iy>=m.size_y-4:raise TaskFailure('TARGET_OUTSIDE_MAP')
                return max(g.data[j*m.size_x+i] for j in range(iy-4,iy+5) for i in range(ix-4,ix+5))
            for height in (.10,.60,1.20,1.90,2.50):
                # Capture the free-space baseline before inserting the
                # fixture.  Point-cloud and costmap updates are asynchronous,
                # so sampling immediately after create() can observe the new
                # obstacle and incorrectly label the baseline as occupied.
                pre_spawn_cost=cost()
                if pre_spawn_cost>=254:raise TaskFailure('TARGET_REGION_ALREADY_OCCUPIED')
                size=[.3,.3,.2];base_name=f'nonhome_height_{os.getpid()}_{round(height*100)}'
                # Gazebo can transiently reject a create request while the
                # previous fixture's remove event is still being reconciled.
                # Retry with a fresh owned name and reconcile any partial
                # entity before the next attempt; a persistent failure still
                # remains evidence that this height was not accepted.
                spawn_error=None
                name=None
                for attempt in range(3):
                    candidate=f'{base_name}_{attempt}'
                    try:
                        node.spawn(candidate,[float(x),float(y),height],size,True)
                        owned.append(candidate);name=candidate;spawn_error=None;break
                    except Exception as error:
                        spawn_error=error
                        try:remove_owned(node,[candidate])
                        except Exception:pass
                        time.sleep(.2*(attempt+1))
                if spawn_error is not None or name is None:
                    raise TaskFailure(f'GAZEBO_CREATE_RETRY_EXHAUSTED:{spawn_error}')
                created=node.get_clock().now().nanoseconds/1e9
                row=dict(center_world_z_m=height,target_xy_m=[float(x),float(y)],size_xyz_m=size,
                    input_points=0,filtered_points=0,target_xy_points=0,target_z_min_m=None,
                    target_z_max_m=None,baseline_costs=[],observed_costs=[],fresh_frames=0)
                row['target_range_m']=target_range;row['scan_target_range_m']=None;row['scan_target_angle_rad']=target_angle
                row['baseline_costs'].append(int(pre_spawn_cost))
                deadline=time.monotonic()+5.;seen=set()
                while time.monotonic()<deadline:
                    if not node.stopped():raise TaskFailure('ROBOT_MOVED_DURING_STATIONARY_PROBE')
                    for key,msg in list(clouds.items()):
                        stamp=seconds(msg.header.stamp)
                        if stamp<=created or (key,stamp) in seen:continue
                        seen.add((key,stamp));row['fresh_frames']+=1
                        points=np.array(list(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)))
                        if points.dtype.names:points=np.column_stack([points[n] for n in ('x','y','z')])
                        if points.size==0:continue
                        tf=node.transform(config['map_frame'],msg.header.frame_id)
                        points=points@tf[:3,:3].T+tf[:3,3]
                        xy_matches=np.all(np.abs(points[:,:2]-np.array([x,y]))<=np.array(size[:2])/2+.04,axis=1)
                        matches=xy_matches & (np.abs(points[:,2]-height)<=size[2]/2+.04)
                        row[key+'_points']=max(row[key+'_points'],int(matches.sum()))
                        if np.any(xy_matches):
                            z=points[xy_matches,2]
                            row['target_xy_points']=max(row['target_xy_points'],int(z.size))
                            row['target_z_min_m']=float(z.min()) if row['target_z_min_m'] is None else min(row['target_z_min_m'],float(z.min()))
                            row['target_z_max_m']=float(z.max()) if row['target_z_max_m'] is None else max(row['target_z_max_m'],float(z.max()))
                    scan=scans.get('scan')
                    if scan is not None and seconds(scan.header.stamp)>created:
                        idx=round((target_angle-scan.angle_min)/scan.angle_increment)
                        if 0<=idx<len(scan.ranges):
                            beam=float(scan.ranges[idx])
                            if math.isfinite(beam) and scan.range_min<=beam<=scan.range_max:
                                row['scan_target_range_m']=beam if row['scan_target_range_m'] is None else min(row['scan_target_range_m'],beam)
                    row['observed_costs'].append(int(cost()));time.sleep(.15)
                row['fixture_observed']=row['target_xy_points']>0
                # The slice limits are expressed in the base frame, while
                # Gazebo fixture heights are specified in the world frame.
                # Compare bounds in the same frame and account for the
                # projected box extent under any base roll/pitch/yaw.
                world_to_base=np.linalg.inv(base)
                center_base=world_to_base@np.array([x,y,height,1.])
                rot=world_to_base[:3,:3]
                half_base_z=.5*sum(float(size[i])*abs(float(rot[2,i])) for i in range(3))
                fixture_world_min=height-size[2]/2.;fixture_world_max=height+size[2]/2.
                fixture_base_min=float(center_base[2]-half_base_z)
                fixture_base_max=float(center_base[2]+half_base_z)
                row['fixture_world_z_bounds_m']=[fixture_world_min,fixture_world_max]
                row['fixture_base_z_bounds_m']=[fixture_base_min,fixture_base_max]
                row['expected_in_projection']=row['fixture_observed'] and \
                    fixture_base_max>=a.projection_z_min and fixture_base_min<=a.projection_z_max
                row['obstacle_marked']=max(row['observed_costs'],default=0)>=254 and max(row['baseline_costs'],default=0)<254
                rows.append(row);remove_owned(node,owned);owned.clear()
                deadline=time.monotonic()+10.
                while cost()>=254:
                    if time.monotonic()>deadline:raise TaskFailure('REMOVED_TARGET_OCCUPANCY_NOT_CLEARED')
                    time.sleep(.2)
            report['passed']=all(r['fresh_frames']>0 and r['fixture_observed'] and
                r['obstacle_marked']==r['expected_in_projection'] for r in rows)
        except Exception as error:report.update(passed=False,error=str(error))
        finally:
            try:remove_owned(node,owned);node.stop_and_hold()
            except Exception as error:report.update(passed=False,cleanup_error=str(error))
            lease.checkpoint(dict(pid=os.getpid(),ledger=str(out/'state.json'),unconfirmed_executor=False))
            node.sync_stop.set();node.sync_thread.join(timeout=4.)
            ex.shutdown();spinner.join(timeout=3.);node.destroy_node();rclpy.shutdown()
        (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
    raise SystemExit(0 if report['passed'] else 1)

if __name__=='__main__':main()
