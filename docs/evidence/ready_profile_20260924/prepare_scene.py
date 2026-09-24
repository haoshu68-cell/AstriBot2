#!/usr/bin/env python3
"""Owned stationary M1 scene preparation. No motion/action/controller commands.

Creates only the two registered stations, physical detached box, and observer.
Loads one real inventory source, applies three world CollisionObjects, and reads
back independently. Failure leaves created assets in place and records progress;
never retries a mutating command or removes models from an uncertain scene.
"""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

REPO=Path('/home/yjh/WorkSpace/astribot_sdk_ros2')
sys.path.insert(0,str(REPO/'tools/sim'))
from prepare_empty_inventory import (verify_owner,require,CaptureReceipts,empty_ready,
    proof_key,ReadbackBarrier,camera_structure,read_model_parameters)
from verify_kinematic_inventory import setup_sample,PreparationWatchdog,fixture_executions


def check_initial_measurement(measured, receipt, now_wall, now_ns, expected, scene_joints):
    stamp=measured['header']['stamp'];at=stamp['sec']*10**9+stamp['nanosec']
    require(0<at<=now_ns<at+300_000_000 and 0<=now_wall-receipt<.3,'INITIAL_READY_JOINT_SOURCE_STALE')
    names=measured['name']
    require(len(names)==len(set(names)) and len(names)==len(measured['position'])==len(measured['velocity']),'INITIAL_READY_JOINTS_INCOMPLETE')
    values=dict(zip(names,measured['position']));velocities=dict(zip(names,measured['velocity']))
    require(set(expected).issubset(values),'INITIAL_READY_JOINTS_INCOMPLETE')
    errors={name:abs(values[name]-target) for name,target in expected.items()}
    require(all(math.isfinite(values[n]) and errors[n]<=.01 for n in expected),'INITIAL_READY_POSE_MISMATCH')
    require(all(math.isfinite(velocities[n]) and abs(velocities[n])<=.01 for n in expected),'INITIAL_READY_NOT_STOPPED')
    require(len(scene_joints['name'])==len(set(scene_joints['name']))==len(scene_joints['position']),'SCENE_JOINT_POSITION_INCOMPLETE')
    scene_values=dict(zip(scene_joints['name'],scene_joints['position']))
    require(set(expected).issubset(scene_values),'SCENE_JOINT_POSITION_INCOMPLETE')
    scene_errors={name:abs(values[name]-scene_values[name]) for name in expected}
    require(all(math.isfinite(scene_values[n]) and scene_errors[n]<=.01 for n in expected),'SCENE_ACTUAL_JOINT_MISMATCH')
    return dict(source='/joint_states',joints=measured,first_stamp_receipt=receipt,checked_wall=now_wall,checked_ros_ns=now_ns,position_errors_rad=errors,scene_position_errors_rad=scene_errors,scene_joint_state=scene_joints)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('owner','plugin-directory','world-reference','output'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--session',required=True);p.add_argument('--source',required=True)
    a=p.parse_args();owner=json.loads(a.owner.read_text())
    verify_owner(owner,a.session,a.source)
    require(a.source=='gazebo_kinematic_v1','KINEMATIC_SOURCE_REQUIRED')
    for key,value in owner.get('query_environment',{}).items():
        require(os.environ.get(key)==value,'OWNER_ENVIRONMENT_MISMATCH:'+key)
    lock=open('/tmp/astribot_waypoint_'+os.environ['ROS_DOMAIN_ID']+'.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    a.output.mkdir(parents=True,exist_ok=False)
    report=dict(passed=False,owner=owner,session=a.session,source=a.source,
        operations=[],created_models=[],started_wall=time.time(),motion_commands_sent=0,
        scope='stationary scene preparation only; not M1 nonhome/Hold or grasp acceptance')
    import numpy as np
    from scipy.spatial.transform import Rotation
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist,Pose
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from shape_msgs.msg import SolidPrimitive
    from moveit_msgs.msg import CollisionObject
    from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene,GetStateValidity
    from rcl_interfaces.srv import GetParameters
    from astribot_payload_msgs.msg import AttachmentState,AttachmentObservation
    from astribot_navigation_msgs.msg import RobotGeometryState
    from rosidl_runtime_py.convert import message_to_ordereddict as md
    from tf2_ros import Buffer,TransformListener
    import yaml
    from ament_index_python.packages import get_package_share_directory
    rclpy.init();node=rclpy.create_node('prepare_m1_scene',parameter_overrides=[Parameter('use_sim_time',value=True)])
    tf_buffer=Buffer();tf_listener=TransformListener(tf_buffer,node)
    latest={};received={};samples={};receipts=CaptureReceipts();guard=None;barrier=None
    def check(raising=True):
        if guard is None:return True
        sample=setup_sample(latest.get('motion'),latest.get('command'),latest.get('hold_executor'),received,
            node.get_clock().now().nanoseconds*1e-9,time.monotonic())
        valid=guard.observe(sample)
        if raising:require(valid,'PREPARATION_STOP_WINDOW_REVOKED')
        return valid
    def ready():
        return check(False) and empty_ready(samples,receipts.received,time.monotonic(),
            node.get_clock().now().nanoseconds,a.session,a.source,'kinematic_inventory_v1')
    def receive(key,msg):
        value=json.loads(msg.data) if key in ('diagnostic','hold_executor') else msg
        duplicate_joint_stamp=(key=='joints' and key in latest and latest[key].header.stamp==msg.header.stamp)
        latest[key]=value
        if not duplicate_joint_stamp:received[key]=time.monotonic()
        check(False)
        if key in ('source','ledger','geometry','diagnostic'):
            samples[key]=value if key=='diagnostic' else md(value)
            receipts.observe(key,samples[key],time.monotonic())
            if barrier is not None:
                good=ready();barrier.observe(good,proof_key(samples) if good else None,
                    time.monotonic(),node.get_clock().now().nanoseconds)
    subs=[]
    for typ,topic,key in ((Twist,'/cmd_vel','command'),(Odometry,'/odom','motion'),(JointState,'/joint_states','joints'),
            (String,'/transport/hold_executor/status','hold_executor'),
            (AttachmentState,'/payload/attachment_state','ledger'),
            (AttachmentObservation,'/payload/attachment_observation','source'),
            (RobotGeometryState,'/navigation/geometry_state','geometry'),
            (String,'/payload/simulation_inventory_diagnostics','diagnostic')):
        subs.append(node.create_subscription(typ,topic,lambda msg,k=key:receive(k,msg),qos_profile_sensor_data if key=='joints' else 10))
    def spin():rclpy.spin_once(node,timeout_sec=.01);check()
    def wait(predicate,timeout=8.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            spin()
            if predicate():return
        raise RuntimeError('WAIT_TIMEOUT:'+str({k: v.get('reason') for k,v in samples.items() if isinstance(v,dict)}))
    def run(argv,timeout=8.,inventory_load=False):
        check();verify_owner(owner,a.session,a.source)
        row=dict(argv=argv,started_wall=time.time());report['operations'].append(row)
        with ThreadPoolExecutor(max_workers=1) as pool:
            future=pool.submit(subprocess.run,argv,capture_output=True,text=True,timeout=timeout)
            while not future.done():
                rclpy.spin_once(node,timeout_sec=.01);check(False)
            result=future.result()
        row.update(returncode=result.returncode,stdout=result.stdout,stderr=result.stderr)
        row['stop_window_valid_at_terminal']=check(False)
        if not inventory_load:check()
        require(result.returncode==0,'COMMAND_FAILED:'+str(argv))
        return result.stdout
    def call(client,request):
        wait(client.service_is_ready,5.)
        pending=client.call_async(request);wait(pending.done,5.)
        return pending.result()
    get_scene=node.create_client(GetPlanningScene,'/get_planning_scene')
    apply=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
    def full_scene():
        request=GetPlanningScene.Request();request.components.components=1023
        return call(get_scene,request).scene
    def pose_matrix(data):
        out=np.eye(4);q=data.get('orientation',{});v=[float(q.get(k,0)) for k in 'xyzw']
        require(abs(sum(x*x for x in v)-1)<1e-5,'INVALID_TRUTH_QUATERNION')
        out[:3,:3]=Rotation.from_quat(v).as_matrix()
        out[:3,3]=[float(data.get('position',{}).get(k,0)) for k in 'xyz']
        require(np.isfinite(out).all(),'NONFINITE_TRUTH')
        return out
    def truth(names):
        raw=run(['ign','topic','-e','-t','/world/default/pose/info','-n','1','--json-output'],5.)
        data=json.loads(raw);stamp=data['header']['stamp']
        at=int(stamp.get('sec',0))*10**9+int(stamp.get('nsec',0))
        now=node.get_clock().now().nanoseconds
        require(0<at<=now<at+300_000_000,'WORLD_TRUTH_STALE_OR_FUTURE')
        poses={}
        for name in names:
            found=[x for x in data['pose'] if x.get('name')==name]
            require(len(found)==1,'WORLD_TRUTH_IDENTITY:'+name);poses[name]=pose_matrix(found[0])
        report.setdefault('world_truth',[]).append(data)
        return poses
    def acquire_stopped(label):
        nonlocal guard
        guard=None
        steady=[];end=time.monotonic()+15
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            sample=setup_sample(latest.get('motion'),latest.get('command'),latest.get('hold_executor'),received,
                node.get_clock().now().nanoseconds*1e-9,time.monotonic())
            if sample is None:steady=[];continue
            if steady and sample['stamp']==steady[-1]['stamp']:continue
            if steady and (not 0<sample['stamp']-steady[-1]['stamp']<=.3 or
                math.hypot(sample['x']-steady[0]['x'],sample['y']-steady[0]['y'])>.005 or
                abs(math.remainder(sample['yaw']-steady[0]['yaw'],2*math.pi))>.01):steady=[]
            steady.append(sample)
            if len(steady)>=3 and sample['stamp']-steady[0]['stamp']>=.6 and sample['wall']-steady[0]['wall']>=.6:
                guard=PreparationWatchdog(sample);break
        require(guard is not None,'IDLE_HOLD_AND_MEASURED_STOP_REQUIRED');report[label]=steady
    try:
        acquire_stopped('stopped_samples')
        require(node.count_publishers('/payload/attachment_observation')==0 and 'diagnostic' not in latest,'INVENTORY_SOURCE_ALREADY_EXISTS')
        models={}
        for target in ('robot_state_publisher','move_group'):
            client=node.create_client(GetParameters,'/'+target+'/get_parameters')
            names=['robot_description','use_sim_time']
            if target=='move_group':names.append('robot_description_semantic')
            req=GetParameters.Request(names=names)
            response=read_model_parameters(client,req,spin,events=report.setdefault('parameter_reads',[]))
            require(len(response.values)==len(names),'MODEL_PARAMETER_RESPONSE_LENGTH:'+target+':'+str(len(response.values)))
            require(response.values[0].type==4 and response.values[0].string_value,'ROBOT_DESCRIPTION_UNAVAILABLE:'+target)
            require(response.values[1].type==1 and response.values[1].bool_value,'SIM_TIME_REQUIRED:'+target)
            models[target]=response.values[0].string_value
            (a.output/(target+'.urdf')).write_text(models[target])
            if target=='move_group':
                require(response.values[2].type==4 and response.values[2].string_value,'MOVEIT_SEMANTIC_UNAVAILABLE')
                semantic=response.values[2].string_value
        require(camera_structure(models['robot_state_publisher'])==camera_structure(models['move_group']),'SIX_CAMERA_MODEL_MISMATCH')
        def geometry_model(xml):
            root=ET.fromstring(xml)
            return [ET.canonicalize(ET.tostring(e,encoding='unicode'),strip_text=True) for e in root if e.tag in ('link','joint')]
        require(geometry_model(models['robot_state_publisher'])==geometry_model(models['move_group']),'FULL_KINEMATIC_COLLISION_MODEL_MISMATCH')
        report['model_readback']={name:dict(sha256=hashlib.sha256(xml.encode()).hexdigest(),initial_values={j.get('name'):j.find("state_interface[@name='position']/param[@name='initial_value']").text for j in ET.fromstring(xml).findall('ros2_control/joint') if j.find("state_interface[@name='position']/param[@name='initial_value']") is not None}) for name,xml in models.items()}
        report['full_geometry_models_equal']=True
        urdf=ET.fromstring(models['robot_state_publisher']);children={j.find('child').get('link') for j in urdf.findall('joint')}
        roots={l.get('name') for l in urdf.findall('link')}-children
        require(roots=={'astribot_torso_base'},'GAZEBO_MODEL_ROOT_UNSUPPORTED')
        virtual=ET.fromstring(semantic).find('virtual_joint')
        require(virtual is not None and virtual.get('type')=='fixed' and virtual.get('child_link')=='astribot_torso_base','PLANNING_ROOT_UNSUPPORTED')
        planning_frame=virtual.get('parent_frame');report['planning_frame']=planning_frame
        robot_sdf=run(['ign','sdf','-p',str(a.output/'robot_state_publisher.urdf')],15.)
        root=ET.fromstring(robot_sdf).find('model');base=root.find("link[@name='astribot_torso_base']")
        require(base is not None and root.get('name')=='astribot_s1','ROOT_MODEL_BINDING_MISMATCH')
        for element in (root.find('pose'),base.find('pose')):
            require(element is None or all(float(v)==0 for v in element.text.split()),'MODEL_ROOT_OFFSET_UNSUPPORTED')
        (a.output/'robot_reference.sdf').write_text(robot_sdf)
        # Prove a usable independent world->model anchor before any mutations.
        truth(['astribot_s1'])
        before=full_scene();require(not before.robot_state.attached_collision_objects,'PLANNING_SCENE_NOT_EMPTY')
        profile_path=Path(get_package_share_directory('astribot_s1_description'))/'config/sim_initial_transport_ready.yaml'
        expected=yaml.safe_load(profile_path.read_text())['initial_positions']
        wait(lambda:'joints' in latest,5.)
        measured=latest['joints']
        initial_measurement=check_initial_measurement(md(measured),received['joints'],time.monotonic(),node.get_clock().now().nanoseconds,expected,md(before.robot_state.joint_state))
        transforms={}
        for link in ('astribot_arm_left_link_7','astribot_arm_right_link_7','astribot_head_link_2'):
            wait(lambda: tf_buffer.can_transform('astribot_torso_base',link,rclpy.time.Time()),5.)
            transform=tf_buffer.lookup_transform('astribot_torso_base',link,rclpy.time.Time())
            stamp=transform.header.stamp.sec*10**9+transform.header.stamp.nanosec
            require(0<stamp<=node.get_clock().now().nanoseconds<stamp+300_000_000,'INITIAL_READY_TF_STALE:'+link)
            transforms[link]=md(transform)
        report['initial_ready']=dict(profile=str(profile_path),profile_sha256=hashlib.sha256(profile_path.read_bytes()).hexdigest(),measurement=initial_measurement,transforms=transforms,zero_to_ready_motion_validated=False)

        items=[('transport_box_01_pick_station',[.1,.7,.5175],[.1,.1,1.035]),
               ('transport_box_01_place_station',[1.15,.72,.5175],[.1,.1,1.035]),
               ('transport_box_01',[.1,.7,1.095],[.06,.06,.12])]
        require(not {x[0] for x in items}&{o.id for o in before.world.collision_objects},'FIXTURE_SCENE_ID_EXISTS')
        library=str((a.plugin_directory/'libastribot_kinematic_payload.so').resolve())
        require(Path(library).is_file(),'KINEMATIC_PLUGIN_MISSING');assets=[]
        for name,xyz,size in items:
            payload=name=='transport_box_01';color='0.9 0.45 0.1 1' if payload else '0.45 0.45 0.45 1'
            geom='<geometry><box><size>'+' '.join(map(str,size))+'</size></box></geometry>'
            inertial=('<inertial><mass>.2</mass><inertia><ixx>.0003</ixx><iyy>.0003</iyy><izz>.00012</izz></inertia></inertial>' if payload else '')
            plugin=('<plugin filename="'+library+'" name="astribot::KinematicPayload"><parent_model>astribot_s1</parent_model><parent_link>astribot_arm_left_link_7</parent_link></plugin>' if payload else '')
            model='<model name="'+name+'"><static>true</static><pose>'+' '.join(map(str,xyz))+' 0 0 0</pose><link name="body">'+inertial+'<collision name="collision">'+geom+'</collision><visual name="visual">'+geom+'<material><diffuse>'+color+'</diffuse></material></visual></link>'+plugin+'</model>'
            assets.append((name,ET.fromstring(model),payload))
        observer=ET.parse(REPO/'runs/normal_grasp_video_20260923/observer.sdf').getroot().find('model')
        assets.append((observer.get('name'),observer,False))
        reference=ET.parse(a.world_reference);world=reference.getroot().find('world')
        require(world is not None and world.get('name')=='default','WORLD_REFERENCE_MISMATCH')
        names={m.get('name') for m in world.findall('model')}
        require(not names&{v[0] for v in assets} and 'astribot_s1' not in names,'REFERENCE_FIXTURE_OR_ROBOT_EXISTS')
        for name,model,payload in assets:
            if not payload:world.append(copy.deepcopy(model))
            document=ET.Element('sdf',version='1.7');document.append(copy.deepcopy(model))
            (a.output/(name+'.sdf')).write_text(ET.tostring(document,encoding='unicode'))
        augmented=a.output/'augmented_baseline.sdf';reference.write(augmented,encoding='unicode')
        registry=[dict(model='transport_box_01',object_id='transport_box_01',physical_parent_link='astribot_arm_left_link_7',attachment_link='astribot_arm_left_tcp_link')]
        (a.output/'registry.json').write_text(json.dumps(registry,indent=2))
        for name,model,payload in assets:
            text=(a.output/(name+'.sdf')).read_text()
            answer=run(['ign','service','-s','/world/default/create','--reqtype','ignition.msgs.EntityFactory','--reptype','ignition.msgs.Boolean','--timeout','5000','--req','sdf: '+json.dumps(text)+' allow_renaming: false'])
            require('data: true' in answer,'CREATE_REJECTED:'+name);report['created_models'].append(name)
        poses=truth(['astribot_s1']+[x[0] for x in items])
        local=np.linalg.inv(poses['astribot_s1']);report['torso_from_gazebo_world']=local.tolist()
        request=ApplyPlanningScene.Request();request.scene.is_diff=True;request.scene.robot_state.is_diff=True
        expected={}
        for name,xyz,size in items:
            require(np.max(np.abs(poses[name][:3,3]-xyz))<1e-5,'SPAWN_POSE_MISMATCH:'+name)
            matrix=local@poses[name];expected[name]=(matrix,size)
            obj=CollisionObject(id=name);obj.header.frame_id='astribot_torso_base';obj.pose.orientation.w=1.
            obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=size)]
            position=Pose();position.position.x,position.position.y,position.position.z=map(float,matrix[:3,3])
            position.orientation.x,position.orientation.y,position.orientation.z,position.orientation.w=map(float,Rotation.from_matrix(matrix[:3,:3]).as_quat())
            obj.primitive_poses=[position];obj.operation=CollisionObject.ADD;request.scene.world.collision_objects.append(obj)
        require(call(apply,request).success,'APPLY_SCENE_REJECTED')
        actual=full_scene();(a.output/'scene_readback.json').write_text(json.dumps(md(actual),indent=2))
        validity=node.create_client(GetStateValidity,'/check_state_validity')
        state_check=GetStateValidity.Request();state_check.robot_state=actual.robot_state
        valid=call(validity,state_check)
        report['initial_ready']['full_scene_state_validity']=md(valid)
        require(valid.valid,'INITIAL_READY_COLLISION_INVALID')

        require(not actual.robot_state.attached_collision_objects,'UNEXPECTED_SCENE_ATTACHMENT')
        require({o.id for o in actual.world.collision_objects}=={o.id for o in before.world.collision_objects}|set(expected),'SCENE_WORLD_ID_SET_MISMATCH')
        for obj in actual.world.collision_objects:
            if obj.id not in expected:continue
            matrix,size=expected[obj.id]
            require(obj.header.frame_id in ('astribot_torso_base',planning_frame),'SCENE_READBACK_FRAME_UNSUPPORTED')
            require(len(obj.primitives)==len(obj.primitive_poses)==1 and not obj.meshes and not obj.planes,'SCENE_SHAPE_MISMATCH')
            require(obj.primitives[0].type==SolidPrimitive.BOX and np.allclose(obj.primitives[0].dimensions,size,rtol=0,atol=1e-9),'SCENE_DIMENSION_MISMATCH')
            measured=pose_matrix(md(obj.pose))@pose_matrix(md(obj.primitive_poses[0]))
            require(np.allclose(measured,matrix,rtol=0,atol=1e-6),'SCENE_POSE_MISMATCH:'+obj.id)
        loader=a.plugin_directory/'astribot_s1_gazebo_bringup/load_empty_inventory'
        report['observer_load_attempted']=True
        run([str(loader),'--world','default','--robot','astribot_s1','--session',a.session,'--source',a.source,
             '--world-reference',str(augmented.resolve()),'--robot-reference',str((a.output/'robot_reference.sdf').resolve()),
             '--plugin',str(a.plugin_directory/'libastribot_empty_inventory.so'),'--payload-registry',str(a.output/'registry.json'),
             '--robot-urdf',str(a.output/'robot_state_publisher.urdf')],15.,inventory_load=True)
        report['old_stop_window_valid_after_inventory_load']=check(False)
        # Dynamic plugin loading can pause physics. The original .3s watchdog
        # remains latched in evidence; establish a new measured window, never
        # reinterpret the old one or repeat the mutating loader request.
        acquire_stopped('post_inventory_load_stopped_samples')
        stable=None;last=None;end=time.monotonic()+20
        while time.monotonic()<end:
            spin();good=ready();key=proof_key(samples) if good else None
            if not good or key!=last:stable=None
            elif stable is None:stable=time.monotonic()
            elif time.monotonic()-stable>=2:break
            last=key
        else:raise RuntimeError('CONTINUOUS_CONFIRMED_EMPTY_UNAVAILABLE')
        report['detached_execution']=fixture_executions(registry,latest['diagnostic'])
        barrier=ReadbackBarrier(proof_key(samples),time.monotonic(),node.get_clock().now().nanoseconds)
        final=full_scene();require(barrier.valid and ready(),'EMPTY_CHANGED_DURING_READBACK')
        require(not final.robot_state.attached_collision_objects,'INDEPENDENT_FULL_SCENE_NOT_EMPTY')
        report['final_inventory']=copy.deepcopy(samples);report['final_scene']=md(final)
        report['references']={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in a.output.glob('*.sdf')}
        report['passed']=True
    except BaseException as error:
        report['error']=repr(error);raise
    finally:
        report['finished_wall']=time.time();report['stop_window_valid']=check(False)
        (a.output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        node.destroy_node();rclpy.shutdown();lock.close()
    print(json.dumps(dict(passed=True,output=str(a.output)),indent=2))

if __name__=='__main__':main()
