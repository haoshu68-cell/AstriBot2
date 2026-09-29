#!/usr/bin/env python3
"""Read this scene and prepare one formal transfer request; no motion commands."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from scipy.spatial.transform import Rotation


def matrix(pose):
    result=np.eye(4)
    result[:3,3]=[float(pose['position'].get(k,0.)) for k in ('x','y','z')]
    q=[float(pose['orientation'].get(k,0.)) for k in ('x','y','z','w')]
    if not np.isfinite(result).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1.)>1e-6:
        raise RuntimeError('INVALID_PHYSICAL_POSE')
    result[:3,:3]=Rotation.from_quat(q).as_matrix()
    return result


def build_fields(fixture_dir, fixture, config, map_from_world):
    from geometry_msgs.msg import Pose,PoseStamped
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from astribot_navigation_msgs.msg import RobotEnvelope
    from astribot_s1_transport_native.action import FixedStationTransfer
    from rosidl_runtime_py.convert import message_to_ordereddict as md
    truth=fixture['world_truth'][-1]
    def observed(name):
        values=[p for p in truth['pose'] if p.get('name')==name]
        if len(values)!=1:raise RuntimeError('FIXTURE_MODEL_NOT_UNIQUE:'+name)
        return matrix(values[0])
    def target(transform,frame):
        p=PoseStamped();p.header.frame_id=frame
        p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,transform[:3,3])
        q=Rotation.from_matrix(transform[:3,:3]).as_quat()
        p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=map(float,q)
        return p
    goal=FixedStationTransfer.Goal(task_id='fixed_transfer_'+uuid.uuid4().hex,request_id=uuid.uuid4().hex,
        context_id=uuid.uuid4().hex,object_id=config['object_id'],touch_links=config['touch_links'],
        grasp_width_m=float(config['size_xyz'][0]),timeout_s=540.)
    sizes={};poses={}
    for suffix in ('pick_station','place_station'):
        name=config['object_id']+'_'+suffix
        path=fixture_dir/(name+'.sdf')
        if fixture['references'].get(str(path.resolve()))!=hashlib.sha256(path.read_bytes()).hexdigest():
            raise RuntimeError('FIXTURE_SDF_CHANGED:'+name)
        model=ET.parse(path).getroot().find('model');link=model.find('link');collision=link.find('collision')
        if model.get('name')!=name or len(model.findall('link'))!=1 or len(link.findall('collision'))!=1:
            raise RuntimeError('REGISTERED_STATION_BOX_REQUIRED:'+name)
        for part in (link,collision):
            p=part.find('pose')
            if p is not None and any(float(v)!=0 for v in p.text.split()):
                raise RuntimeError('STATION_MODEL_ORIGIN_UNREGISTERED:'+name)
        size=list(map(float,collision.find('geometry/box/size').text.split()))
        if len(size)!=3 or not all(math.isfinite(v) and v>0 for v in size):raise RuntimeError('STATION_SIZE_INVALID')
        transform=observed(name);sizes[suffix]=size;poses[suffix]=transform
        obj=CollisionObject(id=name);obj.header.frame_id='gazebo_world';obj.pose=target(transform,'gazebo_world').pose
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=size)]
        identity=Pose();identity.orientation.w=1.;obj.primitive_poses=[identity];obj.operation=CollisionObject.ADD
        goal.stations.append(obj)
    pick=observed(config['object_id']);pick[:3,:3]=Rotation.from_quat(config['orientation_xyzw']).as_matrix()
    place=poses['place_station'].copy()
    place[:3,3]+=(place[:3,:3]@np.array([0.,0.,(sizes['place_station'][2]+config['size_xyz'][2])/2.]))
    place[:3,:3]=Rotation.from_quat(config['place_orientation_xyzw']).as_matrix()
    pick[2,3]+=config['grasp_offset_m'];place[2,3]+=config['grasp_offset_m']
    pre_pick=pick.copy();pre_pick[2,3]+=config['approach_m']
    pre_place=place.copy();pre_place[2,3]+=config['approach_m']
    goal.pick_target=target(pick,'gazebo_world');goal.pick_pre_target=target(pre_pick,'gazebo_world')
    pick_exit=pick.copy();pick_lift=float(config['pick_lift_m'])
    if not math.isfinite(pick_lift) or pick_lift<=0.:
        raise RuntimeError('INVALID_PICK_LIFT')
    # Keep the original XY and approach pose; only the PICK exit lift changes.
    pick_exit[2,3]+=pick_lift
    goal.pick_exit_targets=[target(pick_exit,'gazebo_world')]
    goal.place_target=target(place,'gazebo_world');goal.place_pre_target=target(pre_place,'gazebo_world')
    # This fixture defines the dock in Gazebo world; use the measured frame
    # registration for the map goal, including a nonzero spawn heading/origin.
    world_nav=np.eye(4);world_nav[0,3],world_nav[1,3],yaw=config['nav_goal_world']
    world_nav[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()
    nav=map_from_world@world_nav
    goal.navigation_target=target(nav,config['map_frame'])
    inward=world_nav[:2,3]-place[:2,3];distance=np.linalg.norm(inward)
    if distance<=.10:raise RuntimeError('INVALID_RETREAT_DIRECTION')
    for length in (.08,.10,.06):
        retreat=pre_place.copy();retreat[:2,3]+=length*inward/distance
        goal.place_exit_targets.append(target(retreat,'gazebo_world'))
    goal.navigation_limits=RobotEnvelope(lease_s=.3,frame_id=config['base_frame'],posture_id='transport_compact',
        transport_ready=True,half_length_m=.31,half_width_m=.31,height_m=1.63,payload_mass_kg=config['mass_kg'],
        max_speed_m_s=.35,max_angular_speed_rad_s=1.5,max_acceleration_m_s2=.5,brake_deceleration_m_s2=.5,
        reason='fixed_station_transfer')
    return md(goal)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('fixture','scenario','owner','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--session',required=True);args=p.parse_args()
    sys.path.insert(0,'/home/yjh/WorkSpace/astribot_sdk_ros2/tools/sim')
    from prepare_empty_inventory import verify_owner,read_model_parameters
    owner=json.loads(args.owner.read_text());verify_owner(owner,args.session,'gazebo_kinematic_v1')
    for k,v in owner['query_environment'].items():
        if os.environ.get(k)!=v:raise RuntimeError('OWNER_ENVIRONMENT_MISMATCH:'+k)
    fixture=json.loads((args.fixture/'result.json').read_text());config=json.loads(args.scenario.read_text())
    if not fixture['passed'] or fixture['session']!=args.session or fixture['owner']!=owner:
        raise RuntimeError('THIS_SCENE_FIXTURE_REQUIRED')
    import rclpy
    from rclpy.time import Time
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from rcl_interfaces.srv import GetParameters
    from rosidl_runtime_py.convert import message_to_ordereddict as md
    from tf2_ros import Buffer,TransformListener,TransformException
    rclpy.init();node=rclpy.create_node('prepare_fixed_transfer_goal',parameter_overrides=[Parameter('use_sim_time',value=True)])
    tf=Buffer();listener=TransformListener(tf,node);odom=[];events=[]
    sub=node.create_subscription(Odometry,'/odom',lambda m:odom.append((time.monotonic(),m)),qos_profile_sensor_data)
    report=dict(passed=False,owner=owner,fixture=str(args.fixture.resolve()),events=events,scope='read-only frame/parameter check and static goal registration; not live motion acceptance')
    try:
        requests={
            '/navigation_executor/bt_navigator':['navigation_geometry_mode','robot_base_frame'],
            '/navigation_task_arbiter':['navigation_geometry_mode','robot_base_frame'],
            '/controller_server':['navigation_geometry_mode','precise_goal_checker.xy_goal_tolerance','precise_goal_checker.yaw_goal_tolerance'],
            '/global_costmap/global_costmap':['robot_base_frame'],
            '/local_costmap/local_costmap':['robot_base_frame'],
            '/task_trajectory_executor':['payload_base_frame','fixed_station_position_tolerance_m','fixed_station_yaw_tolerance_rad']}
        report['parameters']={}
        for target,names in requests.items():
            client=node.create_client(GetParameters,target+'/get_parameters')
            response=read_model_parameters(client,GetParameters.Request(names=names),lambda:rclpy.spin_once(node,timeout_sec=.01),events=events)
            values={k:md(v) for k,v in zip(names,response.values)};report['parameters'][target]=values
            if len(values)!=len(names):raise RuntimeError('NAVIGATION_PARAMETER_MISSING:'+target)
            for key,value in values.items():
                expected=('fixed_v2' if key=='navigation_geometry_mode' else config['base_frame'] if key in ('robot_base_frame','payload_base_frame')
                          else .002 if 'position' in key or 'xy_goal' in key else math.radians(.1))
                actual=value['string_value'] if isinstance(expected,str) else value['double_value']
                expected_type=4 if isinstance(expected,str) else 3
                if value['type']!=expected_type or (actual!=expected if isinstance(expected,str) else abs(actual-expected)>1e-12):
                    raise RuntimeError('NAVIGATION_PARAMETER_MISMATCH:'+target+':'+key)
        graph={topic:[dict(node=v.node_name,namespace=v.node_namespace,type=v.topic_type)
                      for v in node.get_publishers_info_by_topic(topic)]
               for topic in ('/cmd_vel','/navigation_policy/constraint_state')}
        report['navigation_writers']=graph
        if graph['/cmd_vel']!=[dict(node='cmd_vel_body_to_world_node',namespace='/',type='geometry_msgs/msg/Twist')]:
            raise RuntimeError('NAVIGATION_OUTPUT_WRITER_MISMATCH')
        if graph['/navigation_policy/constraint_state']!=[dict(node='navigation_constraint',namespace='/',type='std_msgs/msg/String')]:
            raise RuntimeError('NAVIGATION_CONSTRAINT_SOURCE_MISMATCH')
        with ThreadPoolExecutor(max_workers=1) as pool:
            future=pool.submit(subprocess.run,['ign','topic','-e','-t','/world/default/pose/info','-n','1','--json-output'],capture_output=True,text=True,timeout=5,check=True)
            while not future.done():rclpy.spin_once(node,timeout_sec=.01)
            truth=json.loads(future.result().stdout)
        raw=truth['header']['stamp'];at=int(raw.get('sec',0))*10**9+int(raw.get('nsec',0))
        end=time.monotonic()+3.
        while True:
            try:transform=tf.lookup_transform(config['map_frame'],config['base_frame'],Time(nanoseconds=at));break
            except TransformException:
                if time.monotonic()>=end:raise
                rclpy.spin_once(node,timeout_sec=.01)
        if not odom or time.monotonic()-odom[-1][0]>=.3 or odom[-1][1].child_frame_id!=config['base_frame'] or odom[-1][1].header.frame_id!='odom':
            raise RuntimeError('ACTUAL_ODOMETRY_BASE_FRAME_MISMATCH_OR_STALE')
        robots=[v for v in truth['pose'] if v.get('name')=='astribot_s1']
        if len(robots)!=1:raise RuntimeError('ACTUAL_ROBOT_MODEL_NOT_UNIQUE')
        robot_world=matrix(robots[0]);expected=config['spawn_world_xyyaw']
        actual_yaw=math.atan2(robot_world[1,0],robot_world[0,0])
        position_error=math.hypot(robot_world[0,3]-expected[0],robot_world[1,3]-expected[1])
        yaw_error=abs(math.remainder(actual_yaw-expected[2],2*math.pi))
        report['front_facing_start']=dict(expected_world_xyyaw=expected,actual_world_xyyaw=[float(robot_world[0,3]),float(robot_world[1,3]),actual_yaw],position_error_m=position_error,yaw_error_rad=yaw_error)
        if position_error>.002 or yaw_error>math.radians(.1):
            raise RuntimeError('FRONT_FACING_START_POSE_MISMATCH')
        t=md(transform.transform);map_base=matrix(dict(position=t['translation'],orientation=t['rotation']))
        map_world=map_base@np.linalg.inv(matrix(robots[0]))
        fields=build_fields(args.fixture.resolve(),fixture,config,map_world)
        args.output.write_text(json.dumps(fields,indent=2,allow_nan=False)+'\n')
        report.update(passed=True,goal_path=str(args.output),goal_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
            raw_odom=md(odom[-1][1]),same_stamp_map_base=md(transform),world_truth=truth,map_from_world=map_world.tolist())
    except BaseException as error:report['error']=repr(error);raise
    finally:
        args.output.with_suffix('.preflight.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        listener.unregister();node.destroy_subscription(sub);node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
