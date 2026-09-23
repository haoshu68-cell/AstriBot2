#!/usr/bin/env python3
"""Owned simulation kinematic-inventory fault checks. Never sends motion goals.

Mutations: four explicitly registered kinematic fixtures, a temporary unknown
model and PlanningScene attachment, and a bounded world pause. On success the
registered fixtures remain detached for subsequent inventory checks. On failure
detach is attempted, with cleanup failures preserved separately. Hardware and
other sessions are rejected by exact supervisor identity + environment checks.
"""
import argparse
from collections import Counter
import copy
import math
import xml.etree.ElementTree as ET
import json
import os
from pathlib import Path
import subprocess
import time
import uuid


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--owner',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--session',required=True)
    p.add_argument('--source',default='gazebo_kinematic_v1')
    p.add_argument('--plugin-directory',type=Path,required=True)
    p.add_argument('--world-reference',type=Path,required=True)
    p.add_argument('--resume-registry',type=Path,help='Reuse this owned session registry after a failed matrix; no second source is loaded')
    a=p.parse_args()
    identity=json.loads(a.owner.read_text());process=Path('/proc')/str(identity['pid'])
    assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()==identity['boot_id']
    assert process.joinpath('stat').read_text().rsplit(') ',1)[1].split()[19]==identity['start_ticks']
    assert str(process.joinpath('exe').resolve())==identity['exe']
    argv=[v.decode() for v in process.joinpath('cmdline').read_bytes().split(b'\0')[:-1]]
    assert argv==identity['cmdline']
    assert argv[argv.index('--instance')+1]==a.session
    assert argv[argv.index('--ros-domain-id')+1]==os.environ['ROS_DOMAIN_ID']!='25'
    assert os.environ['IGN_PARTITION']=='astribot_'+a.session
    a.output.mkdir(parents=True,exist_ok=False)

    import rclpy
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Twist,Pose
    from shape_msgs.msg import SolidPrimitive
    from moveit_msgs.msg import AttachedCollisionObject
    from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String
    from astribot_payload_msgs.msg import AttachmentState,AttachmentObservation
    from astribot_navigation_msgs.msg import RobotGeometryState
    from rosidl_runtime_py.convert import message_to_ordereddict
    rclpy.init();node=rclpy.create_node('verify_empty_inventory',parameter_overrides=[Parameter('use_sim_time',value=True)])
    latest={};received={};trace=[];phases=[];paused=False;created=False;phantom=False
    reuse=json.loads(a.resume_registry.read_text()) if a.resume_registry else None
    probe=reuse[0]['model'].removesuffix('_box') if reuse else 'inventory_probe_'+uuid.uuid4().hex[:12]
    def record(key,value):
        data=message_to_ordereddict(value)
        if key=='diagnostic':data=json.loads(value.data)
        latest[key]=value if key!='diagnostic' else data
        received[key]=time.monotonic()
        trace.append(dict(kind=key,wall=time.monotonic(),ros_ns=node.get_clock().now().nanoseconds,message=data))
    subs=[node.create_subscription(AttachmentState,'/payload/attachment_state',lambda v:record('ledger',v),10),
          node.create_subscription(AttachmentObservation,'/payload/attachment_observation',lambda v:record('source',v),10),
          node.create_subscription(RobotGeometryState,'/navigation/geometry_state',lambda v:record('geometry',v),10),
          node.create_subscription(String,'/payload/simulation_inventory_diagnostics',lambda v:record('diagnostic',v),10),
          node.create_subscription(Twist,'/cmd_vel',lambda v:record('command',v),10)]
    apply=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
    scene=node.create_client(GetPlanningScene,'/get_planning_scene')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    def wait(predicate,timeout=8.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return
        raise AssertionError({k:(v.reason if hasattr(v,'reason') else v.get('reason') if isinstance(v,dict) else 'received') for k,v in latest.items()})
    def ready():
        s,g=latest.get('ledger'),latest.get('geometry')
        fresh=all(time.monotonic()-received.get(k,0)<.3 for k in ('ledger','geometry','diagnostic'))
        return fresh and s and g and s.confirmed and g.complete and g.attachment_revision==s.attachment_revision and s.observation.status in (AttachmentObservation.EMPTY,AttachmentObservation.ATTACHED)
    def stable_ready(seconds=2.,timeout=12.):
        # Receiving one healthy sample does not prove the subsequent interval.
        # Reset on every observed invalid state, including startup reconciliation.
        start=None;end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            now=time.monotonic()
            if not ready():start=None
            elif start is None:start=now
            elif now-start>=seconds:return
        raise AssertionError('EMPTY readiness was not continuously observed for the required interval')
    def rejected():
        s,g=latest.get('ledger'),latest.get('geometry')
        return s and g and not s.confirmed and not g.complete
    def stationary():
        command=latest.get('command')
        assert command is not None and time.monotonic()-received['command']<.3,'missing fresh stationary command'
        assert max(abs(getattr(getattr(command,field),axis)) for field in ('linear','angular') for axis in ('x','y','z'))<1e-6
    def mark(name,**extra):
        s=latest.get('ledger');g=latest.get('geometry');d=latest.get('diagnostic',{})
        phases.append(dict(name=name,wall=time.monotonic(),ros_ns=node.get_clock().now().nanoseconds,
            ledger_reason=s.reason if s else None,geometry_reason=g.reason if g else None,
            revision=s.attachment_revision if s else None,source_revision=d.get('revision'),**extra))
    def ign(service,kind,request):
        result=subprocess.run(['ign','service','-s','/world/default/'+service,'--reqtype','ignition.msgs.'+kind,
            '--reptype','ignition.msgs.Boolean','--timeout','5000','--req',request],capture_output=True,text=True,timeout=8)
        assert result.returncode==0 and 'data: true' in result.stdout,(service,result.stdout,result.stderr)
    def call(client,request):
        assert client.wait_for_service(timeout_sec=4),client.srv_name
        future=client.call_async(request);wait(future.done,4)
        return future.result()
    def scene_attachment(remove=False):
        obj=AttachedCollisionObject(link_name='astribot_arm_left_tcp_link');obj.object.id=probe+'_phantom'
        obj.object.header.frame_id=obj.link_name;obj.object.pose.orientation.w=1.
        obj.object.operation=obj.object.REMOVE if remove else obj.object.ADD
        if not remove:
            obj.object.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.02,.02,.02])]
            pose=Pose();pose.orientation.w=1.;obj.object.primitive_poses=[pose]
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True
        req.scene.robot_state.attached_collision_objects=[obj]
        assert call(apply,req).success
    success=False
    created_models=[];commands_applied={};fixtures=[]
    def checkpoint(name,**extra):
        mark(name,**extra);print(name,flush=True)
    def apply_objects(objects=(),remove=()):
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True
        req.scene.robot_state.attached_collision_objects=list(objects)
        for identity in remove:
            obj=AttachedCollisionObject();obj.object.id=identity;obj.object.operation=obj.object.REMOVE
            req.scene.robot_state.attached_collision_objects.append(obj)
        assert call(apply,req).success
        request=GetPlanningScene.Request();request.components.components=4
        actual=call(scene,request).scene
        (a.output/f'scene_readback_{len(list(a.output.glob("scene_readback_*.json"))):02d}.json').write_text(json.dumps(message_to_ordereddict(actual),indent=2))
    def command(item,attached):
        count=commands_applied.get(item['model'],0)+1;commands_applied[item['model']]=count
        if attached:
            parent='astribot_s1::'+item['physical_parent_link'];xyz=(0,-.35,.03)
        else:parent='';xyz=(-15.,-15.+fixtures.index(item)*.3,1.)
        yaw=.7 if attached and item['kind']=='sphere' else .35 if attached and item['kind']=='mesh' else 0.
        req=f'id: {count} name: '+json.dumps(parent)+f' position {{ x: {xyz[0]} y: {xyz[1]} z: {xyz[2]} }} orientation {{ z: {math.sin(yaw/2)} w: {math.cos(yaw/2)} }}'
        result=subprocess.run(['ign','service','-s','/model/'+item['model']+'/kinematic_attachment/command',
            '--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','5000','--req',req],capture_output=True,text=True,timeout=8)
        assert result.returncode==0 and 'data: true' in result.stdout,result.stderr
    def source_ids(expected):
        src=latest.get('source')
        return src and src.full_inventory and {v.object.id for v in src.objects}==set(expected)
    def validate_physical_object(obj,item):
        assert obj.link_name==item['attachment_link'] and abs(obj.weight-item['mass'])<1e-9
        assert len(obj.object.primitives)==len(obj.object.primitive_poses)==1
        # Independent fixture oracle: fixed wrist->TCP translation is [0,-.15,0].
        position=obj.object.pose.position
        assert max(abs(position.x),abs(position.y+.20),abs(position.z-.03))<1e-9
        q=obj.object.pose.orientation
        yaw=.7 if item['kind']=='sphere' else .35 if item['kind']=='mesh' else 0.
        assert max(abs(q.x),abs(q.y),abs(q.z-math.sin(yaw/2)),abs(q.w-math.cos(yaw/2)))<1e-9
        shape=obj.object.primitives[0];kind=item['kind']
        raw={'box':(.08,.06,.04),'cylinder':(.08,.03),'sphere':(.03,), 'mesh':(.06,.04,.05)}[kind]
        radius=(math.sqrt(sum((v/2)**2 for v in raw)) if kind in ('box','mesh') else math.hypot(raw[1],raw[0]/2) if kind=='cylinder' else raw[0])
        margin=.005+.01*(.015+radius)
        want=([v+2*margin for v in raw] if kind in ('box','mesh') else [raw[0]+2*margin,raw[1]+margin] if kind=='cylinder' else [raw[0]+margin])
        assert shape.type=={'box':1,'mesh':1,'sphere':2,'cylinder':3}[kind]
        assert len(shape.dimensions)==len(want) and max(abs(x-y)for x,y in zip(shape.dimensions,want))<1e-6,(shape.dimensions,want)
        local=obj.object.primitive_poses[0].position
        assert max(abs(local.x-.015),abs(local.y),abs(local.z))<1e-6
    try:
        # Actual published URDF is independent of ECM inventory and is checked
        # against the planning model camera structure before deriving fixed TCPs.
        urdfs={}
        for target in ('robot_state_publisher','move_group'):
            client=node.create_client(GetParameters,'/'+target+'/get_parameters')
            req=GetParameters.Request();req.names=['robot_description']
            urdfs[target]=call(client,req).values[0].string_value
        def camera_model(xml):
            root=ET.fromstring(xml)
            def normalize(element):
                return (element.tag,sorted(element.attrib.items()),(element.text or '').strip(),[normalize(c)for c in element])
            return {e.get('name'):normalize(e) for e in root if e.tag in ('link','joint') and ('rgbd' in e.get('name','') or 'stereo' in e.get('name',''))}
        assert camera_model(urdfs['robot_state_publisher'])==camera_model(urdfs['move_group'])
        (a.output/'moveit_reference.urdf').write_text(urdfs['move_group'])
        (a.output/'camera_models.json').write_text(json.dumps(dict(equal=True,physical_camera_links=[k for k in camera_model(urdfs['robot_state_publisher']) if k.endswith('_camera_link')]),indent=2))
        (a.output/'robot_reference.urdf').write_text(urdfs['robot_state_publisher'])
        converted=subprocess.run(['ign','sdf','-p',str(a.output/'robot_reference.urdf')],capture_output=True,text=True,timeout=15)
        assert converted.returncode==0,converted.stderr;ET.fromstring(converted.stdout)
        (a.output/'robot_reference.sdf').write_text(converted.stdout)
        mesh=a.output/'payload_cube.obj'
        vertices=[(-.03,-.02,-.025),(.03,-.02,-.025),(.03,.02,-.025),(-.03,.02,-.025),(-.03,-.02,.025),(.03,-.02,.025),(.03,.02,.025),(-.03,.02,.025)]
        # DART requires one normal per imported vertex; OBJ faces must explicitly
        # reference normals. Missing normals can crash the upstream ODE backend.
        faces=[(1,3,2),(1,4,3),(5,6,7),(5,7,8),(1,2,6),(1,6,5),
               (2,3,7),(2,7,6),(3,4,8),(3,8,7),(4,1,5),(4,5,8)]
        normals=[(0,0,-1),(0,0,1),(0,-1,0),(1,0,0),(0,1,0),(-1,0,0)]
        mesh.write_text(''.join('v %.8f %.8f %.8f\n'%v for v in vertices)+
            ''.join('vn %d %d %d\n'%v for v in normals)+
            ''.join('f '+' '.join(f'{v}//{index//2+1}'for v in face)+'\n'for index,face in enumerate(faces)))
        library=str((a.plugin_directory/'libastribot_kinematic_payload.so').resolve())
        for index,kind in enumerate(('box','cylinder','sphere','mesh')):
            side='left' if index%2==0 else 'right'
            item=dict(model=probe+'_'+kind,object_id=probe+'_'+kind,kind=kind,mass=.5+index*.25,
                physical_parent_link='astribot_arm_'+side+'_link_7',attachment_link='astribot_arm_'+side+'_tcp_link')
            fixtures.append(item)
            geometry={'box':'<box><size>.08 .06 .04</size></box>','cylinder':'<cylinder><radius>.03</radius><length>.08</length></cylinder>',
                'sphere':'<sphere><radius>.03</radius></sphere>','mesh':'<mesh><uri>'+str(mesh.resolve())+'</uri><scale>1 1 1</scale></mesh>'}[kind]
            plugin=('<plugin filename="'+library+'" name="astribot::KinematicPayload"><parent_model>astribot_s1</parent_model><parent_link>'+item['physical_parent_link']+'</parent_link></plugin>') if index else ''
            sdf='<sdf version="1.7"><model name="'+item['model']+'"><static>true</static><pose>-15 '+str(-15+index*.3)+' 1 0 0 0</pose><link name="body"><inertial><mass>'+str(item['mass'])+'</mass><inertia><ixx>.01</ixx><iyy>.01</iyy><izz>.01</izz></inertia></inertial><collision name="shape"><pose>.015 0 0 0 0 0</pose><geometry>'+geometry+'</geometry></collision><visual name="shape"><pose>.015 0 0 0 0 0</pose><geometry>'+geometry+'</geometry></visual></link>'+plugin+'</model></sdf>'
            (a.output/(kind+'.sdf')).write_text(sdf)
            if not reuse:ign('create','EntityFactory','sdf: '+json.dumps(sdf)+' allow_renaming: false')
            created_models.append(item['model'])
        registry=[{k:v for k,v in item.items()if k in ('model','object_id','physical_parent_link','attachment_link')}for item in fixtures]
        (a.output/'registry.json').write_text(json.dumps(registry,indent=2))
        loader=[str(a.plugin_directory.parent/'lib/astribot_s1_gazebo_bringup/load_empty_inventory'),'--world','default','--robot','astribot_s1',
            '--session',a.session,'--source',a.source,'--world-reference',str(a.world_reference.resolve()),
            '--robot-reference',str((a.output/'robot_reference.sdf').resolve()),'--plugin',str((a.plugin_directory/'libastribot_empty_inventory.so').resolve()),
            '--payload-registry',str((a.output/'registry.json').resolve()),'--robot-urdf',str((a.output/'robot_reference.urdf').resolve())]
        if not reuse:
            loaded=subprocess.run(loader,capture_output=True,text=True,timeout=15)
            (a.output/'loader.json').write_text(json.dumps(dict(command=loader,returncode=loaded.returncode,stdout=loaded.stdout,stderr=loaded.stderr),indent=2))
            assert loaded.returncode==0,loaded.stderr
            wait(lambda:rejected() and latest.get('diagnostic',{}).get('reason')=='PAYLOAD_REGISTRY_INCOMPLETE',12)
            checkpoint('missing_executor_cannot_claim_empty')
            first=fixtures[0];entity=next(row[0]for row in latest['diagnostic']['models']if row[1]==first['model'])
            inner='<parent_model>astribot_s1</parent_model><parent_link>'+first['physical_parent_link']+'</parent_link>'
            ign('entity/system/add','EntityPlugin_V','entity { id: '+str(entity)+' type: MODEL } plugins { name: "astribot::KinematicPayload" filename: '+json.dumps(library)+' innerxml: '+json.dumps(inner)+' }')
        else:
            assert registry==reuse,'resume registry differs from the expected explicit fixture identities'
            wait(lambda: latest.get('diagnostic',{}).get('reason')=='EMPTY_INVENTORY_OBSERVED',12)
            diagnostic=latest['diagnostic'];models={row[0]:row[1]for row in diagnostic['models']}
            for execution in diagnostic['execution']:
                assert not execution['attached'] and execution['accepted']==execution['applied']
                commands_applied[models[execution['entity']]]=execution['accepted']
            checkpoint('reuse_owned_explicit_detached_registry')
        stable_ready();assert source_ids(set());checkpoint('all_registered_explicit_detached_empty')
        current=set()
        for item in fixtures:
            stationary();command(item,True);current.add(item['object_id'])
            wait(lambda:source_ids(current));wait(rejected)
            obj=next(copy.deepcopy(v)for v in latest['source'].objects if v.object.id==item['object_id'])
            validate_physical_object(obj,item)
            checkpoint(item['kind']+'_physical_before_scene_rejected')
            apply_objects([obj]);stable_ready(seconds=.6)
            checkpoint(item['kind']+'_independently_read_back_confirmed',object_count=len(current))
        original=latest['ledger'].attachment_revision
        sdf='<sdf version="1.7"><model name="'+probe+'"><static>true</static><link name="body"/></model></sdf>'
        stationary();ign('create','EntityFactory','sdf: '+json.dumps(sdf)+' allow_renaming: false');created=True
        wait(rejected);checkpoint('unknown_world_model_revoked')
        ign('remove','Entity','name: '+json.dumps(probe)+' type: MODEL');created=False
        stable_ready(seconds=.6);assert latest['ledger'].attachment_revision!=original
        checkpoint('unknown_model_removed_new_version')
        scene_attachment();phantom=True;wait(lambda:rejected() and latest['ledger'].reason=='PHYSICAL_SCENE_MISMATCH');checkpoint('independent_scene_mismatch_revoked')
        scene_attachment(True);phantom=False;stable_ready(seconds=.6);checkpoint('scene_reconciled_recovery')
        stationary();ign('control','WorldControl','pause: true');paused=True
        spin(1.);assert rejected();checkpoint('paused_clock_wall_expiry')
        ign('control','WorldControl','pause: false');paused=False;stable_ready(seconds=.6)
        checkpoint('fresh_after_resume_confirmed')
        for item in fixtures:
            command(item,False);current.remove(item['object_id']);wait(lambda:source_ids(current));wait(rejected)
            apply_objects(remove=[item['object_id']]);stable_ready(seconds=.6)
            checkpoint(item['kind']+'_released_confirmed',object_count=len(current))
        stable_ready(seconds=2.);assert latest['ledger'].observation.status==AttachmentObservation.EMPTY
        request=GetPlanningScene.Request();request.components.components=4
        actual=call(scene,request).scene
        assert not actual.is_diff and not actual.robot_state.is_diff and not actual.robot_state.attached_collision_objects
        commands=[v['message']for v in trace if v['kind']=='command'];assert commands
        maximum=max(abs(x)for cmd in commands for field in ('linear','angular')for x in cmd[field].values());assert maximum<1e-6
        checkpoint('final_empty_stationary',max_command=maximum)
        success=True
    finally:
        cleanup=[]
        if not success:
            for item in fixtures:
                if item['model'] not in created_models:continue
                try:command(item,False);apply_objects(remove=[item['object_id']]);cleanup.append(dict(action='detach_'+item['model'],ok=True))
                except Exception as error:cleanup.append(dict(action='detach_'+item['model'],ok=False,error=str(error)))
        for needed,fn,label in [(paused,lambda:ign('control','WorldControl','pause: false'),'resume_world'),
                                 (phantom,lambda:scene_attachment(True),'remove_phantom'),
                                 (created,lambda:ign('remove','Entity','name: '+json.dumps(probe)+' type: MODEL'),'remove_probe')]:
            if needed:
                try:fn();cleanup.append(dict(action=label,ok=True))
                except Exception as error:cleanup.append(dict(action=label,ok=False,error=str(error)))
        (a.output/'trace.json').write_text(json.dumps(trace,indent=2))
        report=dict(passed=success,phases=phases,cleanup=cleanup,owner=str(a.owner.resolve()),
                    evidence='actual Gazebo kinematic attachments + physical collision/mass oracle + independent MoveIt readback; not force/contact grasp',
                    retained_registered_models=created_models,
                    reasons=dict(Counter(v['message'].get('reason') for v in trace if v['kind']=='ledger')))
        (a.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
        node.destroy_node();rclpy.shutdown()
        if success:assert all(v['ok'] for v in cleanup),cleanup

if __name__=='__main__':main()
