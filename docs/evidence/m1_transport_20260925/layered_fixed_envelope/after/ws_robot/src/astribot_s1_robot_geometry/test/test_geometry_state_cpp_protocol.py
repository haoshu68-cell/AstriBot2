"""Owned C++ producer plus read-only fake inputs; no Gazebo or robot commands."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import pytest


def test_cpp_producer_preserves_attachment_identity_and_source_deadline(tmp_path):
    executable=Path(os.environ.get('GEOMETRY_STATE_CPP','/tmp/codex_geometry_cpp_20260921/install/astribot_s1_robot_geometry/lib/astribot_s1_robot_geometry/geometry_state'))
    assert executable.is_file(), 'C++ geometry_state producer has not been built'
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.srv import GetParameters
    from moveit_msgs.srv import GetPlanningScene
    from moveit_msgs.msg import PlanningScene, AttachedCollisionObject
    from geometry_msgs.msg import Pose
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
    from astribot_navigation_msgs.msg import RobotGeometryState
    from rosidl_runtime_py.convert import message_to_ordereddict
    from test_cpp_robot_model import URDF
    profile=Path(__file__).parents[2]/'astribot_s1_mapping/config/height_slices.yaml'
    expected_profile=hashlib.sha256(profile.read_bytes()).hexdigest()
    assert os.environ.get('ROS_DOMAIN_ID')=='115', 'isolation domain 115 required'
    rclpy.init();node=Node('geometry_cpp_protocol_fixture')
    states=[];acks=[];last_source=0
    attached=AttachedCollisionObject(link_name='arm_link')
    attached.object.id='offset_payload';attached.object.header.frame_id='arm_link';attached.object.pose.orientation.w=1.
    primitive=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.2,.15,.1]);attached.object.primitives=[primitive]
    pose=Pose();pose.orientation.w=1.;pose.position.x=.3;attached.object.primitive_poses=[pose]
    expected_revision=hashlib.sha256(json.dumps([message_to_ordereddict(attached)],sort_keys=True).encode()).hexdigest()
    coverage=dict(input_cloud_topic='/map_scan',base_frame='base',enable_outlier_filter=False)
    for i,name in enumerate(('low_obstacle','main_nav','torso_high','overhead')):
        for field,value in dict(enabled=True,z_min=-.03 if i==0 else i*.6,z_max=(i+1)*.6,min_points=1).items():coverage[f'slices.{name}.{field}']=value
    def parameters(request,response):
        response.values=[Parameter(name,value=URDF if name=='robot_description' else coverage[name]).to_parameter_msg().value for name in request.names]
        return response
    def scene(request,response):
        response.scene.robot_state.attached_collision_objects=[attached];return response
    node.create_service(GetParameters,'/robot_state_publisher/get_parameters',parameters)
    node.create_service(GetParameters,'/pointcloud_slice_scan_node/get_parameters',parameters)
    node.create_service(GetPlanningScene,'/get_planning_scene',scene)
    ack=node.create_publisher(String,'/navigation/attachment_filter_applied',10)
    def attachment(message):
        acks.append(message.name);ack.publish(String(data=message.name))
    node.create_subscription(PlanningScene,'/navigation/attached_geometry',attachment,10)
    node.create_subscription(RobotGeometryState,'/navigation/geometry_state',states.append,10)
    joints=node.create_publisher(JointState,'/joint_states',qos_profile_sensor_data)
    log=(tmp_path/'geometry_state.log').open('w')
    process=subprocess.Popen([str(executable),'--ros-args','-p','base_frame:=base', '-p', 'ground_in_base_m:=-0.095', '-p', f'height_profile_path:={profile}', '-p', 'attachment_source_mode:=planning_scene_legacy'],stdout=log,stderr=subprocess.STDOUT)
    try:
        def spin(duration,publish=True):
            nonlocal last_source
            end=time.monotonic()+duration
            while time.monotonic()<end:
                assert process.poll() is None, (tmp_path/'geometry_state.log').read_text()
                if publish:
                    m=JointState();m.header.stamp=node.get_clock().now().to_msg();last_source=m.header.stamp.sec*10**9+m.header.stamp.nanosec
                    m.name=['arm'];m.position=[.2];joints.publish(m)
                if acks:ack.publish(String(data=acks[-1]))
                rclpy.spin_once(node,timeout_sec=.02)
        spin(.2)
        environment=Path(f'/proc/{process.pid}/environ').read_bytes().split(b'\0')
        assert b'ROS_DOMAIN_ID=115' in environment
        mappings=Path(f'/proc/{process.pid}/maps').read_text()
        assert '_geometry_native' not in mappings and 'libpython' not in mappings
        spin(2.8)
        complete=[s for s in states if s.complete]
        assert complete, [(s.reason,s.complete) for s in states[-5:]]
        for s in complete:
            ns=lambda t:t.sec*10**9+t.nanosec
            assert len(s.height_slices)==5
            assert s.height_profile_revision==expected_profile and s.ground_in_base_m==-.095
            edges=[-.045,.155,.585,1.085,1.535,2.205]
            for i,layer in enumerate(s.height_slices):
                assert abs(layer.z_min_m-edges[i])<1e-12 and abs(layer.z_max_m-edges[i+1])<1e-12
            assert not s.height_slices[-1].footprint.points, 'empty upper layer must retain its slot'
            assert s.attachment_revision==expected_revision
            assert s.attachment_state_confirmed and s.attachment_ids==['offset_payload']
            assert ns(s.header.stamp)<=ns(s.published_at)<ns(s.valid_until)<=ns(s.header.stamp)+300_000_000
            assert ns(s.joint_source_stamps[0])==ns(s.header.stamp)
        states.clear();spin(.7,publish=False)
        assert states and not states[-1].complete and 'STALE' in states[-1].reason
    finally:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()
