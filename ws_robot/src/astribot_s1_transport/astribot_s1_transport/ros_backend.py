"""ROS/MoveIt implementation. Gazebo payload is a kinematic surrogate, not a force grasp."""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
from xml.sax.saxutils import escape
from ament_index_python.packages import get_package_prefix

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene, PlanningSceneWorld
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.action import ExecuteTrajectory
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener, TransformException
from astribot_transport_msgs.srv import PlanSkill
from astribot_transport_msgs.srv import RevalidateManipulation
from astribot_transport_msgs.action import PlanManipulation
from rosidl_runtime_py.convert import message_to_ordereddict
from astribot_navigation_msgs.msg import RobotEnvelope,RobotGeometryState,NavigationEnvelopeV2,ArmHoldStatus
from astribot_navigation_msgs.srv import SetRobotEnvelope,SetFixedEnvelope
from astribot_logging import get_logger
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
from .core import (TransportTask, Ledger, ResourceLease, TaskFailure, Canceled,
                   validate_scenario, canonical_resource_domain, check_native_resource_release)
from .geometry import collision_bounds, collision_primitive_matrix
from .plan_guard import PlanGuard
from .payload_sync import physical_parent,confirmed_state
from astribot_transport_msgs.srv import SetExecutionGuard
from astribot_transport_msgs.msg import ExecutionGuardStatus


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def matrix(pose):
    p, q = pose.position, pose.orientation
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    result[:3, 3] = [p.x, p.y, p.z]
    return result


def pose_from_matrix(transform):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, transform[:3, 3])
    q = Rotation.from_matrix(transform[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    return pose


def pose_at(xyz, quat=(0., 0., 0., 1.)):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, quat)
    return pose


class RosBackend(Node):
    evidence_level = 'gazebo_kinematic_attachment_with_moveit_and_ros_control'

    def __init__(self, config, ledger):
        if config.get('navigation_geometry_mode', 'legacy') == 'fixed_v2':
            raise TaskFailure('CPP_TASK_HOLD_EXECUTOR_REQUIRED: legacy measured-only hold is not an execution ownership proof')
        super().__init__('transport_task', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.c, self.ledger = config, ledger
        self.fixed_v2=config.get("navigation_geometry_mode","legacy")=="fixed_v2"
        self.step_attachment=self.fixed_v2
        self.payload_parent=None;self.payload_state=None;self.payload_command=0
        self.payload_radius=float(np.linalg.norm(config['size_xyz']))/2
        self.create_subscription(String,'/model/'+config['object_id']+'/kinematic_attachment/state',
                                 self.receive_payload_state,10)
        self.geometry_state=None;self.navigation_envelope=None;self.hold_id=""
        self.hold_owner=str(uuid.uuid4());self.hold_reference=None
        self.cancel_requested = False
        self.odom = self.joints = self.scan = self.envelope = None
        self.observation = None
        self.navigation_policy = None
        self.observed_pick = None
        self.active = None
        self.mtc_bundle = None
        self.mtc_feedback = {}
        self.vla = None
        self.recovery = False
        self.execution_guard = None
        self.execution_guard_status = None
        self.execution_guard_client=self.create_client(SetExecutionGuard,'/transport/execution_guard/set')
        self.create_subscription(ExecutionGuardStatus,'/transport/execution_guard/status',
            lambda message:setattr(self,'execution_guard_status',(message,time.monotonic())),10)
        self.pending_goal = None
        self.admitted = False
        self.attached = False
        self.scene_created = False
        self.offset = None
        self.sync_error = ''
        self.last_sync = 0.
        self.last_clock = 0.
        self.carry_joints = {}
        self.clock_changed = time.monotonic()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        from .source_inbox import SourceInbox
        self.source_inboxes={name:SourceInbox(config['observation_max_age_s']) for name in ('odom','joints','scan')}
        self.create_subscription(Odometry, '/odom', lambda m: self.receive_source('odom',m), qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', lambda m: self.receive_source('joints',m), qos_profile_sensor_data)
        self.create_subscription(LaserScan, config.get('scan_topic', '/scan_from_cloud'), lambda m: self.receive_source('scan',m), qos_profile_sensor_data)
        self.create_subscription(RobotEnvelope, '/navigation/robot_envelope', self.legacy_envelope, 10)
        if self.fixed_v2:
            self.create_subscription(RobotGeometryState,'/navigation/geometry_state',lambda m:setattr(self,'geometry_state',m),10)
            self.create_subscription(NavigationEnvelopeV2,'/navigation/envelope_v2',self.receive_fixed_envelope,10)
            self.hold_pub=self.create_publisher(ArmHoldStatus,'/navigation/arm_hold',10)
            self.set_fixed=self.create_client(SetFixedEnvelope,'/navigation/set_fixed_envelope')
            self.controller_query=self.create_client(ListControllers,'/controller_manager/list_controllers')
            self.controller_future=None;self.hold_claims=set();self.claims_at=-math.inf
            self.create_timer(.1,self.publish_arm_hold)
        self.create_subscription(String, '/transport/object_observation', self.receive_observation, 10)
        self.create_subscription(String, '/navigation_policy/state', self.receive_navigation_policy, 10)
        self.status = self.create_publisher(String, '/transport/status', 10)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.plan = self.create_client(PlanSkill, '/transport/plan_skill')
        self.revalidate = self.create_client(RevalidateManipulation, '/transport/revalidate_manipulation')
        self.gz_pose = self.create_client(SetEntityPose, '/world/' + config['world'] + '/set_pose')
        self.apply = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.set_envelope = self.create_client(SetRobotEnvelope, '/navigation/set_robot_envelope')
        self.robot_params = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        self.create_service(Trigger, '/transport/cancel', self.request_cancel)
        self.arm = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.gripper = ActionClient(self, FollowJointTrajectory, '/gripper_left_controller/follow_joint_trajectory')
        self.head = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.nav_through = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self.corridor_witness = None
        self.corridor_samples = None
        self.mtc = ActionClient(self, PlanManipulation, '/transport/plan_manipulation')
        if config.get('vla'):
            from .vla_ros import VlaBridge
            self.vla = VlaBridge(self, config['vla'])
        self.timer = self.create_timer(.2, self.publish_status, clock=rclpy.clock.Clock())
        self.sync_stop = threading.Event()
        self.sync_lock = threading.Lock()
        self.sync_thread = threading.Thread(target=self.sync_loop, daemon=True)
        self.sync_thread.start()

    def legacy_envelope(self,msg):
        if not self.fixed_v2 or self.navigation_envelope is None:self.envelope=msg

    def receive_payload_state(self,msg):
        try:
            state=json.loads(msg.data)
            # Independent /clock and state transport can deliver a positive
            # sample slightly early. Keep the previous deadline; never replace
            # it with future evidence. Explicit failures remain immediate.
            if (not state.get('error') and
                state['stamp_ns']>self.get_clock().now().nanoseconds):return
            self.payload_state=state
        except (ValueError,TypeError,KeyError,AttributeError):pass

    def payload_confirmed(self,attached):
        return confirmed_state(self.payload_state,self.payload_command,attached,
            self.get_clock().now().nanoseconds*1e-9,self.payload_radius)

    def receive_fixed_envelope(self,msg):
        self.navigation_envelope=msg;self.envelope=msg.limits

    def publish_arm_hold(self):
        if not self.hold_id or self.hold_reference is None:return
        s=self.geometry_state;r=self.hold_reference
        now=self.get_clock().now()
        current=now.nanoseconds*1e-9
        if self.controller_future is not None and self.controller_future.done():
            try:
                controllers=self.controller_future.result().controller
                self.hold_claims={interface for c in controllers if c.state=='active' and
                    c.type=='joint_trajectory_controller/JointTrajectoryController' for interface in c.claimed_interfaces}
                self.claims_at=current
            except Exception:self.hold_claims=set();self.claims_at=-math.inf
            self.controller_future=None
        if self.controller_future is None and self.controller_query.service_is_ready():
            self.controller_future=self.controller_query.call_async(ListControllers.Request())
        okay=s is not None and s.complete and seconds(s.valid_until)>now.nanoseconds*1e-9
        okay=okay and 0<=current-self.claims_at<=.5 and all(n+'/position' in self.hold_claims for n in r.joints.name)
        if okay:
            q=dict(zip(s.joints.name,s.joints.position))
            okay=(s.source_id,s.model_revision,s.attachment_revision,s.clock_epoch)==(r.source_id,r.model_revision,r.attachment_revision,r.clock_epoch)
            okay=okay and all(n in q and abs(q[n]-v)<=e for n,v,e in zip(r.joints.name,r.joints.position,r.joint_position_error_bounds))
        msg=ArmHoldStatus(owner_id=self.hold_owner,hold_id=self.hold_id,lease_s=.3,
            hold_confirmed=bool(okay),attachment_revision=r.attachment_revision)
        msg.header.stamp=now.to_msg();self.hold_pub.publish(msg)

    def request_cancel(self, request, response):
        self.cancel_requested = True
        response.success = True
        response.message = 'Cancellation requested; wait for terminal /transport/status and ledger.'
        return response

    def receive_observation(self, message):
        try:
            packet = json.loads(message.data)
            if packet.get('object_id') == self.c['object_id'] and packet.get('frame_id') == self.c['map_frame']:
                self.observation = packet
        except (ValueError, TypeError):
            pass

    def receive_navigation_policy(self, message):
        try:
            packet = json.loads(message.data)
            if isinstance(packet, dict) and isinstance(packet.get('stamp_ns'), int):
                self.navigation_policy = packet
        except (ValueError, TypeError):
            pass

    def locate_object(self):
        samples = []
        def stable_observation():
            observation = self.observation
            if observation is None:
                return False
            age = (self.get_clock().now().nanoseconds - observation['stamp_ns']) * 1e-9
            if not 0 <= age <= self.c['observation_max_age_s']:
                return False
            if samples and observation['stamp_ns'] <= samples[-1]['stamp_ns']:
                return False
            point = np.asarray(observation['center_m'])
            if not np.isfinite(point).all() or np.linalg.norm(point - self.c['pick_xyz']) > .08:
                raise TaskFailure('CAMERA_OBJECT_OUTSIDE_SEARCH_REGION')
            samples.append(copy.deepcopy(observation))
            return len(samples) >= 3 and np.max(np.std([s['center_m'] for s in samples[-3:]], axis=0)) < .01
        self.wait(stable_observation, 20.)
        self.observed_pick = np.mean([s['center_m'] for s in samples[-3:]], axis=0).tolist()
        self.apply_scene(PlanningScene(world=PlanningSceneWorld(collision_objects=[
            self.box(self.c['object_id'], self.observed_pick, self.c['size_xyz'])])))
        self.ledger.object.observation_source = 'rgbd_color_template'
        self.ledger.object.version += 1
        self.ledger.emit('PERCEPTION_LOCATE', observed_center_m=self.observed_pick,
                         observation=samples[-1])

    def publish_status(self):
        self.status.publish(String(data=json.dumps(dict(stage=self.ledger.stage,
            object_state=self.ledger.object.state, version=self.ledger.object.version,
            planning=self.mtc_feedback, vla=(dict(request_id=self.vla.last_request,
                decision=self.vla.last_decision) if self.vla else None)))))

    def stage_feedback(self, stage, status, reason=''):
        if self.vla:
            try:
                self.vla.feedback(stage, status, reason)
            except Exception as error:
                # Telemetry must never replace the original task failure or stop
                # the existing cancellation/hold transaction.
                self.get_logger().warning('VLA feedback unavailable: ' + str(error))

    def receive_source(self, name, message):
        self.source_inboxes[name].receive(message)
        setattr(self,name,self.source_inboxes[name].select(self.get_clock().now().nanoseconds))

    def check(self, ignore_cancel=False):
        if self.cancel_requested and not ignore_cancel:
            raise Canceled('USER_CANCEL')
        now = self.get_clock().now().nanoseconds * 1e-9
        if now != self.last_clock:
            if now < self.last_clock:
                raise TaskFailure('CLOCK_ROLLBACK')
            self.last_clock, self.clock_changed = now, time.monotonic()
        if time.monotonic() - self.clock_changed > 4.:
            raise TaskFailure('SIM_CLOCK_STALLED')
        if not ignore_cancel and self.corridor_witness is not None:
            self.observe_corridor()
        if self.sync_error:
            raise TaskFailure(self.sync_error)
        if self.execution_guard is not None and not ignore_cancel:
            context=self.execution_guard
            packet=self.execution_guard_status
            if packet is None:raise TaskFailure('EXECUTION_GUARD_MISSING')
            state,received=packet
            if (state.context_id!=context or not state.active or not -.01<=now-seconds(state.stamp)<=.3 or
                    time.monotonic()-received>.3):raise TaskFailure('EXECUTION_GUARD_STALE_OR_MISMATCHED')
            if not state.healthy:raise TaskFailure(state.reason)
        if self.attached:
            if self.step_attachment:
                if not self.payload_confirmed(True):raise TaskFailure('PAYLOAD_STEP_SYNC_UNCONFIRMED')
            elif time.monotonic() - self.last_sync > 3.:
                raise TaskFailure('PAYLOAD_SYNC_STALE')
        if self.admitted and not ignore_cancel:
            # Promote queued acquisitions only when /clock has caught up.
            # A far-future stream cannot keep the prior sample alive past its
            # original deadline; rollback is rejected above before selection.
            observed_ns=self.get_clock().now().nanoseconds
            for name,inbox in self.source_inboxes.items():setattr(self,name,inbox.select(observed_ns))
            messages = {'odom': self.odom, 'joint_states': self.joints, self.c.get('scan_topic', '/scan_from_cloud'): self.scan}
            if not all(self.fresh(m) for m in messages.values()):
                observed_now = self.get_clock().now().nanoseconds * 1e-9
                ages = {topic: (observed_now - seconds(message.header.stamp) if message is not None else None)
                        for topic, message in messages.items()}
                raise TaskFailure('OBSERVATIONS_STALE:' + json.dumps(ages, sort_keys=True))
            if self.ledger.stage == 'TRANSPORT':
                e = self.envelope
                if self.fixed_v2:
                    if self.navigation_envelope is None:
                        raise TaskFailure('FIXED_V2_ENVELOPE_MISSING')
                    if not self.navigation_envelope.navigation_allowed:
                        raise TaskFailure('FIXED_V2_REVOKED:'+self.navigation_envelope.reason)
                    if seconds(self.navigation_envelope.valid_until)<=now:
                        raise TaskFailure('FIXED_V2_SOURCE_EVIDENCE_EXPIRED')
                if e is None or not e.transport_ready or not 0 <= now - seconds(e.stamp) <= e.lease_s:
                    raise TaskFailure('TRANSPORT_ENVELOPE_LOST')
                positions = dict(zip(self.joints.name, self.joints.position))
                if any(abs(positions.get(name, math.inf) - value) > .025
                       for name, value in self.carry_joints.items()):
                    raise TaskFailure('TRANSPORT_POSTURE_CHANGED')

    def fresh(self, message):
        if message is None:
            return False
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age_ns = self.get_clock().now().nanoseconds - stamp_ns
        # DDS delivery may put joint states one physics step ahead of /clock.
        # Integer comparison preserves the exact 10 ms boundary without float cancellation.
        return -10_000_000 <= age_ns <= round(self.c['observation_max_age_s'] * 1e9)

    def wait(self, predicate, timeout=20., checked=True):
        deadline = time.monotonic() + timeout
        while not predicate():
            if checked:
                self.check()
            if time.monotonic() >= deadline:
                raise TaskFailure('WAIT_TIMEOUT')
            time.sleep(.02)

    def future(self, future, timeout=20., checked=True):
        self.wait(future.done, timeout, checked)
        return future.result()

    def call(self, client, request, checked=True):
        if not client.wait_for_service(timeout_sec=5.):
            raise TaskFailure('SERVICE_UNAVAILABLE:' + client.srv_name)
        if client in (self.get_scene,self.robot_params):
            deadline=time.monotonic()+20.
            while True:
                future=client.call_async(request)
                try:
                    return self.future(future,min(2.,max(.01,deadline-time.monotonic())),checked)
                except TaskFailure as error:
                    if str(error)!='WAIT_TIMEOUT':raise
                    client.remove_pending_request(future);future.cancel()
                    if time.monotonic()>=deadline:raise
                    self.get_logger().warning('Retrying timed-out read-only query: '+client.srv_name)
        return self.future(client.call_async(request), checked=checked)

    def transform(self, target, source):
        stamped = self.tf.lookup_transform(target, source, Time())
        stamp_ns = stamped.header.stamp.sec * 1_000_000_000 + stamped.header.stamp.nanosec
        age_ns = self.get_clock().now().nanoseconds - stamp_ns
        if stamp_ns > 0 and not -10_000_000 <= age_ns <= round(self.c['observation_max_age_s'] * 1e9):
            raise TaskFailure('STALE_TRANSFORM:' + source + ':age_ns=' + str(age_ns))
        t = stamped.transform
        return matrix(pose_at([t.translation.x, t.translation.y, t.translation.z],
                             [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]))

    def admit(self):
        self.wait(lambda: self.fresh(self.odom) and self.fresh(self.joints) and self.fresh(self.scan), 45.)
        self.wait(lambda: self.envelope is not None)
        # Availability alone does not establish that the first TF sample is
        # current. Wait for an admissible sample before allowing any motion.
        def fresh_tcp_transform():
            try:
                self.transform(self.c['map_frame'], self.c['tcp'])
                return True
            except TransformException:
                return False
            except TaskFailure as error:
                if str(error).startswith('STALE_TRANSFORM:'):
                    return False
                raise
        self.wait(fresh_tcp_transform, 15.)
        # This scenario's world coordinates are configured Gazebo ground truth.
        # Reject use on an arbitrary SLAM map with a different origin.
        base = self.transform(self.c['map_frame'], self.c['base_frame'])
        if np.linalg.norm(base[:2, 3]) > .08 or abs(Rotation.from_matrix(base[:3, :3]).as_euler('xyz')[2]) > .05:
            raise TaskFailure('SCENARIO_REQUIRES_BASE_AT_WORLD_ORIGIN')
        if not self.gz_pose.wait_for_service(timeout_sec=5.):
            raise TaskFailure('GAZEBO_POSE_SERVICE_UNAVAILABLE')
        for client in (self.arm, self.gripper, self.head, self.nav):
            if not client.wait_for_server(timeout_sec=10.):
                raise TaskFailure('ACTION_SERVER_UNAVAILABLE')
        if not self.mtc.wait_for_server(timeout_sec=10.):
            raise TaskFailure('MTC_SERVER_UNAVAILABLE')
        if self.fixed_v2:
            # A process/action endpoint can exist while the policy emits no
            # data. Reject before creating/grasping a payload in that case.
            def policy_live():
                packet=self.navigation_policy
                return packet is not None and 0<=self.get_clock().now().nanoseconds-packet['stamp_ns']<=500_000_000
            try:self.wait(policy_live,15.)
            except TaskFailure as error:
                if str(error)!='WAIT_TIMEOUT':raise
                raise TaskFailure('NAVIGATION_POLICY_HEARTBEAT_UNAVAILABLE') from error
        if self.scene().robot_state.attached_collision_objects:
            raise TaskFailure('EXISTING_ATTACHMENT_RECOVERY_REQUIRED')
        self.ledger.emit('ADMISSION', localization='baseline_ground_truth',
                         calibration='URDF_TCP_simulation', physical_grasp=False,
                         scenario_sha256=hashlib.sha256(json.dumps(self.c, sort_keys=True).encode()).hexdigest())
        self.admitted = True
        if self.vla:
            self.vla.start()

    def stopped(self):
        if not self.fresh(self.odom):
            return False
        v = self.odom.twist.twist
        return math.hypot(v.linear.x, v.linear.y) < .02 and abs(v.angular.z) < .03

    def admit_recovery(self):
        self.recovery = True
        self.wait(lambda: all(self.fresh(m) for m in (self.odom, self.joints, self.scan)), 20.)
        self.wait(lambda: self.envelope is not None and self.tf.can_transform(
            self.c['map_frame'], self.c['base_frame'], Time()))
        if self.envelope.transport_ready or not self.stopped():
            raise TaskFailure('RECOVERY_REQUIRES_EXISTING_HOLD')
        base = self.transform(self.c['map_frame'], self.c['base_frame'])
        yaw = Rotation.from_matrix(base[:3, :3]).as_euler('xyz')[2]
        expected = self.c['nav_goal']
        if (np.linalg.norm(base[:2, 3]-expected[:2]) > .03 or
                abs(math.atan2(math.sin(yaw-expected[2]), math.cos(yaw-expected[2]))) > .03):
            raise TaskFailure('RECOVERY_BASE_POSE_MISMATCH')
        scene = self.scene()
        if scene.robot_state.attached_collision_objects:
            raise TaskFailure('RECOVERY_ATTACHMENT_STILL_PRESENT')
        objects = {o.id: o for o in scene.world.collision_objects}
        for name, xyz, size in self.stations():
            obj = objects.get(name)
            if obj is None or len(obj.primitives) != 1 or len(obj.primitive_poses) != 1:
                raise TaskFailure('RECOVERY_STATION_MISSING')
            pose = self.transform(self.c['map_frame'], obj.header.frame_id) @ collision_primitive_matrix(obj)
            if (np.linalg.norm(pose[:3, 3]-xyz) > self.c['placement_tolerance_m'] or
                    np.linalg.norm(self.observed_payload_position(name)-xyz) > .005 or
                    not np.allclose(obj.primitives[0].dimensions, size, atol=1e-6)):
                raise TaskFailure('RECOVERY_STATION_MISMATCH:' + json.dumps(dict(name=name, actual=pose[:3, 3].tolist(), expected=xyz, dimensions=list(obj.primitives[0].dimensions), expected_size=size)))
        # Planning an open trajectory is read-only; measure it without sending it.
        opened = self.call(self.plan, PlanSkill.Request(operation='open', group='arm_left'))
        if not opened.success or not opened.trajectory.joint_trajectory.points:
            raise TaskFailure('RECOVERY_GRIPPER_REFERENCE_UNAVAILABLE')
        self.settle(opened.trajectory.joint_trajectory)
        if not self.arm.wait_for_server(timeout_sec=5.):
            raise TaskFailure('ACTION_SERVER_UNAVAILABLE')
        self.verify_place()
        self.scene_created = self.admitted = True
        self.ledger.emit('RECOVERY_ADMISSION', gripper='measured_open', attachment='confirmed_absent',
                         scenario_sha256=hashlib.sha256(json.dumps(self.c, sort_keys=True).encode()).hexdigest())

    def change_envelope(self, ready, checked=True):
        if self.fixed_v2 and not ready:
            self.hold_id="";self.hold_reference=None
        self.wait(self.stopped, checked=checked)
        if self.envelope is None:
            raise TaskFailure('ENVELOPE_UNAVAILABLE')
        e = copy.deepcopy(self.envelope)
        e.posture_id = 'transport_box_01_carry' if ready else 'transport_manipulation_hold'
        e.transport_ready = ready
        # Uncertain attach/release acknowledgements must retain the load budget.
        if self.attached or self.ledger.object.state in ('ATTACH_PENDING', 'ATTACHED', 'RELEASE_PENDING'):
            e.payload_mass_kg = max(e.payload_mass_kg, self.c['mass_kg'])
        elif self.scene_created and self.ledger.object.state == 'PLACED':
            e.payload_mass_kg = 0.
        if ready and not self.fixed_v2:
            urdf = self.call(self.robot_params, GetParameters.Request(names=['robot_description'])).values[0].string_value
            bounds = collision_bounds(urdf, lambda link: self.transform(self.c['base_frame'], link),
                self.c['size_xyz'], self.transform(self.c['base_frame'], self.c['tcp']) @ self.offset)
            e.half_length_m = max(e.half_length_m, bounds[0])
            e.half_width_m = max(e.half_width_m, bounds[1])
            e.height_m = max(e.height_m, bounds[2])
            e.max_speed_m_s = min(e.max_speed_m_s, .2)
            e.max_angular_speed_rad_s = min(e.max_angular_speed_rad_s, .3)
            e.max_acceleration_m_s2 = min(e.max_acceleration_m_s2, .2)
        if ready and self.fixed_v2:
            from .fixed_hold import StableGeometryReference
            settled=StableGeometryReference(self.get_clock().now().nanoseconds)
            self.wait(lambda:settled.observe(self.geometry_state,self.get_clock().now().nanoseconds),20.,checked)
            self.hold_reference=copy.deepcopy(self.geometry_state);self.hold_id=str(uuid.uuid4())
            self.publish_arm_hold()
            e.max_speed_m_s=min(e.max_speed_m_s,.2);e.max_angular_speed_rad_s=min(e.max_angular_speed_rad_s,.3)
            e.max_acceleration_m_s2=min(e.max_acceleration_m_s2,.2)
            request=SetFixedEnvelope.Request(request_id=str(uuid.uuid4()),hold_id=self.hold_id,
                geometry_sequence=self.hold_reference.sequence,limits=e)
            deadline=time.monotonic()+2.
            while True:
                response=self.call(self.set_fixed,request,checked)
                if response.accepted or response.reason not in ('ARM_HOLD_UNCONFIRMED','ARM_HOLD_EXPIRED','GEOMETRY_EXPIRED') or time.monotonic()>=deadline:break
                # ROS service delivery is not ordered against the hold topic.
                until=time.monotonic()+.1
                self.wait(lambda:time.monotonic()>=until,.5,checked)
                state=self.geometry_state
                if state is None or not state.complete:break
                self.hold_reference=copy.deepcopy(state);request.geometry_sequence=state.sequence
                self.publish_arm_hold()
        else:
            request = SetRobotEnvelope.Request(envelope=e)
            response = self.call(self.set_envelope, request, checked)
        if not response.accepted:
            raise TaskFailure('ENVELOPE_REJECTED:' + response.reason)
        def confirmed():
            e = self.envelope
            return (e is not None and e.epoch == response.epoch and e.transport_ready == ready and
                    0 <= self.get_clock().now().nanoseconds * 1e-9 - seconds(e.stamp) <= e.lease_s)
        self.wait(confirmed, 30., checked)
        self.ledger.emit(self.ledger.stage, envelope_epoch=response.epoch,
                         half_length_m=self.envelope.half_length_m, half_width_m=self.envelope.half_width_m,
                         payload_mass_kg=self.envelope.payload_mass_kg, transport_ready=self.envelope.transport_ready)

    def hold(self):
        self.change_envelope(False)
        if self.scene_created:
            # This robot's MoveIt model has a fixed base. Refresh world->base
            # coordinates after driving, before any new arm plan.
            self.apply_scene(PlanningScene(world=PlanningSceneWorld(
                collision_objects=[self.box(name, xyz, size) for name, xyz, size in self.stations()])))

    def transport_envelope(self):
        if not self.attached:
            raise TaskFailure('PAYLOAD_NOT_ATTACHED')
        self.change_envelope(True)
        self.carry_joints = {name: value for name, value in zip(self.joints.name, self.joints.position)
                             if any(part in name for part in ('arm_', 'gripper_', 'torso_joint', 'head_joint'))}

    def cancel_active(self):
        self.ledger.emit(self.ledger.stage, executor_event='cancel_requested',
                         pending_acceptance=self.pending_goal is not None)
        if self.pending_goal is not None:
            handle = self.future(self.pending_goal, 15., checked=False)
            self.pending_goal = None
            if handle.accepted:
                self.active = (handle, handle.get_result_async())
        if self.active is None:
            return
        handle, result_future = self.active
        response=self.future(handle.cancel_goal_async(), 10., checked=False)
        self.ledger.emit(self.ledger.stage, executor_event='cancel_response',
                         return_code=response.return_code)
        # A cancel response is not evidence of a stopped executor.
        terminal=self.future(result_future, 15., checked=False)
        if terminal.status not in (4,5,6):
            raise TaskFailure('EXECUTOR_TERMINAL_UNCONFIRMED:'+str(terminal.status))
        self.ledger.emit(self.ledger.stage, executor_event='terminal_after_cancel',
                         action_status=terminal.status)
        self.active = None

    def action(self, client, goal, timeout, feedback_callback=None):
        navigation = client is self.nav or client is getattr(self,'nav_through',None)
        if self.fixed_v2 and not navigation and self.hold_id:
            raise TaskFailure("ARM_MOTION_REQUIRES_NAVIGATION_HOLD")
        sent = client.send_goal_async(goal, feedback_callback=feedback_callback)
        self.pending_goal = sent
        phase='goal_acceptance'
        try:
            # Retain the future until its handle is owned, including when a
            # guard fault or user cancellation arrives during acceptance.
            handle = self.future(sent, 15.)
            self.pending_goal = None
            if not handle.accepted:
                raise TaskFailure('ACTION_REJECTED')
            result_future = handle.get_result_async()
            self.active = (handle, result_future)
            phase='execution'
            self.check()  # A future can already be complete on the first poll.
            wrapped = self.future(result_future, timeout)
            self.check()
        except Exception as error:
            self.ledger.emit(self.ledger.stage, executor_event='fault_observed',
                             executor_phase=phase, reason=str(error))
            try:
                self.cancel_active()
            except Exception as stop_error:
                # Keep ownership and the original safety fault; the outer
                # stop/hold transaction records unresolved executor shutdown.
                self.ledger.emit(self.ledger.stage, executor_event='cancel_unconfirmed',
                                 reason=str(stop_error), original_fault=str(error))
            raise
        self.active = None
        if wrapped.status != 4:
            detail = (getattr(wrapped.result, 'reason', '') or getattr(wrapped.result, 'error_string', '') or
                      str(getattr(wrapped.result, 'error_code', '')))
            policy = self.navigation_policy
            if navigation and not detail and policy is not None:
                age_ns = self.get_clock().now().nanoseconds - policy['stamp_ns']
                if 0 <= age_ns <= round(self.c['observation_max_age_s'] * 1e9):
                    detail = 'navigation_policy=' + str(policy.get('reason', ''))
            raise TaskFailure('ACTION_FAILED:' + str(wrapped.status) + ':' + detail)
        return wrapped.result

    def settle(self, trajectory):
        if not trajectory.points:
            return
        desired = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
        stable = 0
        last_stamp = None
        def measured():
            nonlocal stable, last_stamp
            if not self.fresh(self.joints):
                stable = 0
                return False
            stamp = seconds(self.joints.header.stamp)
            if stamp == last_stamp:
                return False
            last_stamp = stamp
            positions = dict(zip(self.joints.name, self.joints.position))
            velocities = dict(zip(self.joints.name, self.joints.velocity))
            good = all(abs(positions.get(j, math.inf) - p) <= .02 and
                       abs(velocities.get(j, math.inf)) <= .03 for j, p in desired.items())
            stable = stable + 1 if good else 0
            return stable >= 4
        self.wait(measured, 15.)

    def skill(self, operation, group='arm_left', named='', target=None):
        if self.mtc_bundle is not None and not self.mtc_bundle.complete:
            self.execute_mtc('GRIPPER' if operation in ('open', 'close') else 'ARM')
            return
        if not self.stopped():
            raise TaskFailure('ARM_REQUIRES_STOPPED_BASE')
        if not self.fresh(self.joints):
            raise TaskFailure('STALE_JOINTS')
        request = PlanSkill.Request(operation=operation, group=group, named_target=named,
                                    grasp_width_m=float(self.c['size_xyz'][0]))
        if operation == 'joints':
            request.joint_target = list(self.c.get('head_pick_joints', [0., 0.]))
        if target is not None:
            request.target = target
        response = self.call(self.plan, request)
        if not response.success:
            raise TaskFailure('SKILL_PLAN:' + response.reason)
        trajectory = response.trajectory
        if not trajectory.joint_trajectory.points:
            return
        self.check()
        if operation in ('open', 'close') or group == 'head':
            controller = self.head if group == 'head' else self.gripper
            result = self.action(controller, FollowJointTrajectory.Goal(trajectory=trajectory.joint_trajectory), 60.)
            if result.error_code != 0:
                raise TaskFailure(('HEAD:' if group == 'head' else 'GRIPPER:') + result.error_string)
        else:
            result = self.action(self.arm, ExecuteTrajectory.Goal(trajectory=trajectory), self.c['operation_timeout_s'])
            if result.error_code.val != 1:
                raise TaskFailure('EXECUTION:' + str(result.error_code.val))
        self.settle(trajectory.joint_trajectory)

    def look_pick(self):
        self.require_manipulation_hold()
        self.skill('joints', group='head')

    def arm_target(self, station, lift, inward_m=0.):
        xyz = list(self.observed_pick if station == 'pick' else self.c[station + '_xyz'])
        if inward_m:
            base = self.transform(self.c['map_frame'], self.c['base_frame'])
            inward = base[:2, 3] - xyz[:2]
            distance = np.linalg.norm(inward)
            if distance <= inward_m:
                raise TaskFailure('INVALID_RETREAT_DIRECTION')
            xyz[:2] = (np.array(xyz[:2]) + inward_m * inward / distance).tolist()
        xyz[2] += self.c['grasp_offset_m'] + lift
        target = PoseStamped()
        target.header.frame_id = self.c['base_frame']
        orientation = self.c.get('place_orientation_xyzw', self.c['orientation_xyzw']) if station == 'place' else self.c['orientation_xyzw']
        world_target = matrix(pose_at(xyz, orientation))
        target.pose = pose_from_matrix(self.transform(self.c['base_frame'], self.c['map_frame']) @ world_target)
        return target

    def arm_pose(self, station, lift, inward_m=0.):
        self.skill('pose', target=self.arm_target(station, lift, inward_m))

    def scene_context(self, scene, include_occupancy=True):
        # Own payload has explicit attach/detach checkpoints; all other geometry,
        # collision policy and occupancy remain bound to this plan snapshot.
        world = copy.deepcopy(scene.world)
        world.collision_objects = sorted([o for o in world.collision_objects if o.id != self.c['object_id']], key=lambda o:o.id)
        for obj in world.collision_objects:
            obj.header.stamp.sec = obj.header.stamp.nanosec = 0
        raw = [message_to_ordereddict(world), message_to_ordereddict(scene.allowed_collision_matrix)]
        octomap = raw[0]['octomap']['octomap']
        if not include_occupancy:
            octomap['data'] = []
        elif octomap['data']:
            # Bind the plan to collision geometry, not changing sensor log-odds.
            # The C++ decoder preserves all nodes' keys/depths/classifications;
            # frame, origin, resolution and the collision matrix stay in raw.
            try:
                from _transport_scene_native import canonical_octomap
                octomap['data'] = canonical_octomap(
                    bytes(value % 256 for value in octomap['data']),
                    octomap['binary'], octomap['resolution'], octomap['id']).hex()
            except (ImportError, RuntimeError, ValueError, TypeError) as error:
                raise TaskFailure('MTC_OCTOMAP_CANONICALIZATION_FAILED:' + str(error)) from error
        return hashlib.sha256(json.dumps(raw, sort_keys=True).encode()).hexdigest()

    def payload_context(self, scene):
        objects = copy.deepcopy([o for o in scene.world.collision_objects if o.id == self.c['object_id']])
        for obj in objects:
            obj.header.stamp.sec = obj.header.stamp.nanosec = 0
        return json.dumps([message_to_ordereddict(o) for o in objects], sort_keys=True)

    def stable_scene_snapshot(self):
        # A head turn exposes new voxels. Plan after their collision semantics
        # settle; execution still compares the live scene before every stage.
        candidate, signature, stable_since = None, None, None
        def settled():
            nonlocal candidate, signature, stable_since
            candidate = self.scene(full=True)
            current = self.scene_context(candidate)
            now = time.monotonic()
            if current != signature:
                signature, stable_since = current, now
                return False
            return now - stable_since >= 1.0
        try:
            self.wait(settled, 15.)
        except TaskFailure as error:
            if str(error) == 'WAIT_TIMEOUT':
                raise TaskFailure('MTC_SCENE_NOT_STABLE') from error
            raise
        return candidate

    def prepare_manipulation(self, operation):
        if self.mtc_bundle is not None and not self.mtc_bundle.complete:
            raise TaskFailure('MTC_PREVIOUS_PLAN_UNCONSUMED')
        self.require_manipulation_hold()
        samples = []
        def base_settled():
            pose = self.transform(self.c['map_frame'], self.c['base_frame'])
            stamp = self.tf.lookup_transform(self.c['map_frame'], self.c['base_frame'], Time()).header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if samples and stamp_ns <= samples[-1][0]:
                return False
            samples.append((stamp_ns, pose))
            while len(samples) > 1 and stamp_ns - samples[1][0] >= 500_000_000:
                samples.pop(0)
            return (stamp_ns - samples[0][0] >= 500_000_000 and
                    all(np.linalg.norm(p[:3, 3] - pose[:3, 3]) <= .001 and
                        Rotation.from_matrix(p[:3, :3].T @ pose[:3, :3]).magnitude() <= .003
                        for _, p in samples))
        self.wait(base_settled, 20.)
        self.require_manipulation_hold()
        self.mtc_hold_epoch = self.envelope.epoch
        if not self.mtc.wait_for_server(timeout_sec=10.):
            raise TaskFailure('MTC_SERVER_UNAVAILABLE')
        snapshot = self.stable_scene_snapshot()
        snapshot.robot_state.joint_state = copy.deepcopy(self.joints)
        context = str(uuid.uuid4())
        goal = PlanManipulation.Goal(operation=operation, object_id=self.c['object_id'],
            context_id=context, scene=snapshot, touch_links=self.c['touch_links'],
            grasp_width_m=float(self.c['size_xyz'][0]), timeout_s=45.)
        station = 'pick' if operation == 'PICK' else 'place'
        goal.pre_target = self.arm_target(station, self.c['approach_m'])
        goal.target = self.arm_target(station, 0.)
        goal.exit_targets = ([self.arm_target('pick', self.c['approach_m'])] if operation == 'PICK' else
            [self.arm_target('place', self.c['approach_m'], d) for d in (.08, .10, .06)])
        if self.vla:
            goal = self.vla.propose(goal, snapshot)
        self.mtc_base = self.transform(self.c['map_frame'], self.c['base_frame'])
        self.mtc_scene_context = self.scene_context(snapshot)
        self.mtc_static_scene_context = self.scene_context(snapshot, include_occupancy=False)
        self.mtc_payload_context = self.payload_context(snapshot)
        self.mtc_calibration = (self.observation or {}).get('calibration_epoch')
        (self.ledger.directory / (operation.lower() + '_mtc_request.json')).write_text(
            json.dumps(message_to_ordereddict(goal), indent=2))
        def feedback(message):
            f = message.feedback
            self.mtc_feedback = dict(stage_path=f.stage_path, attempt=f.attempt, diagnostic=f.diagnostic)
        response = self.action(self.mtc, goal, 55., feedback)
        if not response.success:
            raise TaskFailure('MTC_PLAN:' + response.reason)
        self.mtc_bundle = PlanGuard(operation, response.stages, time.monotonic(), context, response.context_id)
        self.mtc_context_id = context
        # Retain the complete plan, including expected states, for review/replay inspection.
        (self.ledger.directory / (operation.lower() + '_mtc_plan.json')).write_text(
            json.dumps(message_to_ordereddict(response), indent=2))
        self.ledger.emit(self.ledger.stage, planner='moveit_task_constructor', context_id=context,
                         planned_stages=[s.stage_id for s in response.stages])

    def check_mtc_execution_context(self):
        self.check()
        self.require_manipulation_hold()
        if self.envelope.epoch != self.mtc_hold_epoch:
            raise TaskFailure('MTC_HOLD_EPOCH_CHANGED')
        base = self.transform(self.c['map_frame'], self.c['base_frame'])
        delta = np.linalg.inv(self.mtc_base) @ base
        if np.linalg.norm(delta[:3, 3]) > .02 or Rotation.from_matrix(delta[:3, :3]).magnitude() > .02:
            raise TaskFailure('MTC_BASE_MOVED')
        if (self.observation or {}).get('calibration_epoch') != self.mtc_calibration:
            raise TaskFailure('MTC_CALIBRATION_CHANGED')

    def mtc_stage(self, kind):
        if self.mtc_bundle is None or self.mtc_bundle.complete:
            if self.recovery:
                return None
            raise TaskFailure('MTC_PLAN_REQUIRED')
        self.check_mtc_execution_context()
        scene = self.scene(full=True)
        if self.scene_context(scene) != self.mtc_scene_context:
            (self.ledger.directory / 'mtc_scene_changed.json').write_text(json.dumps(dict(
                expected_context=self.mtc_scene_context,
                current_context=self.scene_context(scene),
                stage=self.ledger.stage, scene=message_to_ordereddict(scene)), indent=2))
            # Only occupancy may be refreshed. The native planner owns the
            # exact original remaining paths and their stage-specific scenes.
            scene = self.stable_scene_snapshot()
            if self.scene_context(scene, include_occupancy=False) != self.mtc_static_scene_context:
                raise TaskFailure('MTC_SCENE_CHANGED')
            checked_context = self.scene_context(scene)
            request = RevalidateManipulation.Request(context_id=self.mtc_context_id,
                start_index=self.mtc_bundle.index, scene=scene)
            response = self.call(self.revalidate, request)
            if not response.success or response.context_id != self.mtc_context_id:
                raise TaskFailure('MTC_REVALIDATION_REJECTED:' + response.reason)
            scene = self.scene(full=True)
            if self.scene_context(scene) != checked_context:
                raise TaskFailure('MTC_SCENE_CHANGED_DURING_REVALIDATION')
            self.mtc_scene_context = checked_context
            self.ledger.emit(self.ledger.stage, executor_event='remaining_plan_revalidated',
                context_id=self.mtc_context_id, start_index=self.mtc_bundle.index,
                scene_context=checked_context, validator='native_mtc')
        # Scene/service waits process new feedback; recheck the execution
        # authority and measured base before accepting the next stage.
        self.check_mtc_execution_context()
        expected_state = self.mtc_bundle.stages[self.mtc_bundle.index].expected_start
        expected = {j:p for j,p in zip(expected_state.joint_state.name, expected_state.joint_state.position)
                    if (any(part in j for part in ('arm_', 'torso_joint', 'head_joint')) or
                        j in ('astribot_gripper_left_joint_L1', 'astribot_gripper_right_joint_L1'))}
        attached = {o.object.id for o in scene.robot_state.attached_collision_objects}
        expected_attached = {o.object.id for o in expected_state.attached_collision_objects}
        if self.c['object_id'] not in expected_attached and self.payload_context(scene) != self.mtc_payload_context:
            raise TaskFailure('MTC_PAYLOAD_CHANGED')
        if self.c['object_id'] in attached & expected_attached:
            actual_body = next(o for o in scene.robot_state.attached_collision_objects if o.object.id == self.c['object_id'])
            planned_body = next(o for o in expected_state.attached_collision_objects if o.object.id == self.c['object_id'])
            if (actual_body.link_name != planned_body.link_name or
                actual_body.object.header.frame_id != planned_body.object.header.frame_id or
                actual_body.object.primitives != planned_body.object.primitives):
                raise TaskFailure('MTC_ATTACHMENT_GEOMETRY_CHANGED')
            offset = np.linalg.inv(collision_primitive_matrix(planned_body.object)) @ collision_primitive_matrix(actual_body.object)
            if np.linalg.norm(offset[:3, 3]) > .025 or Rotation.from_matrix(offset[:3, :3]).magnitude() > .05:
                raise TaskFailure('MTC_ATTACHMENT_OFFSET_CHANGED')
        return self.mtc_bundle.check(self.ledger.stage, kind, time.monotonic(),
            dict(zip(self.joints.name, self.joints.position)), expected, attached, expected_attached)

    def require_manipulation_hold(self):
        e = self.envelope
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.stopped() or e is None or e.transport_ready or not 0 <= now - seconds(e.stamp) <= e.lease_s:
            raise TaskFailure('MTC_REQUIRES_FRESH_HOLD')

    def execute_mtc(self, kind):
        stage = self.mtc_stage(kind)
        trajectory = stage.trajectory
        context=None
        if self.fixed_v2 and kind=='ARM':
            context=str(uuid.uuid4())
            reference=PoseStamped();reference.header.frame_id=self.c['map_frame']
            reference.header.stamp=self.get_clock().now().to_msg();reference.pose=pose_from_matrix(self.mtc_base)
            request=SetExecutionGuard.Request(enable=True,context_id=context,
                joint_names=list(trajectory.joint_trajectory.joint_names),base_reference=reference)
        try:
            if context is not None:
                response=self.call(self.execution_guard_client,request)
                if not response.accepted:raise TaskFailure(response.reason)
                def guard_ready():
                    packet=self.execution_guard_status
                    if packet is None or packet[0].context_id!=context:return False
                    state=packet[0]
                    if not state.healthy and state.reason!='WAITING_FOR_EXECUTION_EVIDENCE':raise TaskFailure(state.reason)
                    return state.active and state.healthy
                self.wait(guard_ready,2.)
                self.execution_guard=context
                self.check()
            if kind == 'GRIPPER':
                result = self.action(self.gripper, FollowJointTrajectory.Goal(trajectory=trajectory.joint_trajectory), 30.)
                if result.error_code != 0:
                    raise TaskFailure('GRIPPER:' + result.error_string)
            else:
                result = self.action(self.arm, ExecuteTrajectory.Goal(trajectory=trajectory), self.c['operation_timeout_s'])
                if result.error_code.val != 1:
                    raise TaskFailure('EXECUTION:' + str(result.error_code.val))
            self.settle(trajectory.joint_trajectory)
            self.check()
        finally:
            self.execution_guard=None
            if context is not None and self.active is None and self.pending_goal is None:
                try:
                    response=self.call(self.execution_guard_client,SetExecutionGuard.Request(enable=False,context_id=context),checked=False)
                    if not response.accepted:raise TaskFailure(response.reason)
                except Exception as error:self.get_logger().error('Execution guard disarm unconfirmed: '+str(error))
        self.mtc_bundle.acknowledge()

    def navigate(self):
        for waypoint in self.c.get('nav_waypoints', []):
            self.navigate_to(waypoint)
        if self.c.get('nav_corridor'):
            from .corridor_route import CorridorRoute
            self.navigate_corridor(CorridorRoute(**self.c['nav_corridor']))
        else:
            self.navigate_to(self.c['nav_goal'])

    def observe_corridor(self):
        try:
            tf=self.tf.lookup_transform(self.c['map_frame'],self.c['base_frame'],Time())
            stamp=seconds(tf.header.stamp)
            age=self.get_clock().now().nanoseconds*1e-9-stamp
            if not 0<=age<=self.c['observation_max_age_s']:
                raise ValueError('CORRIDOR_WITNESS_STALE')
            p=tf.transform.translation
            previous=self.corridor_witness.samples
            self.corridor_witness.observe(stamp,p.x,p.y)
            if self.corridor_samples is not None and self.corridor_witness.samples!=previous:
                self.corridor_samples.write(json.dumps(dict(stamp=stamp,x=p.x,y=p.y))+'\n')
                self.corridor_samples.flush()
        except (ValueError,TransformException) as error:
            raise TaskFailure(str(error)) from error

    def navigate_corridor(self,route):
        from .corridor_route import TraversalWitness
        if not self.fixed_v2 or route.frame_id!=self.c['map_frame'] or self.c['base_frame']!='astribot_torso_base':
            raise TaskFailure('CORRIDOR_REQUIRES_FIXED_V2_CANONICAL_FRAMES')
        actual=self.transform(self.c['map_frame'],self.c['base_frame'])
        approach=route.pose(-route.approach_m)
        yaw=Rotation.from_matrix(actual[:3,:3]).as_euler('xyz')[2]
        if (math.hypot(actual[0,3]-approach[0],actual[1,3]-approach[1])>.03 or
            abs(math.remainder(yaw-route.yaw,2*math.pi))>.02 or not self.stopped()):
            raise TaskFailure('CORRIDOR_REQUIRES_ALIGNED_STATIONARY_APPROACH')
        if not self.nav_through.wait_for_server(timeout_sec=5.):raise TaskFailure('ACTION_SERVER_UNAVAILABLE')
        path=self.ledger.directory/('corridor_'+str(uuid.uuid4())+'.xml')
        path.write_text(route.behavior_tree())
        goal=NavigateThroughPoses.Goal(behavior_tree=str(path.resolve()))
        for x,y,yaw in route.via_poses():
            pose=PoseStamped();pose.header.frame_id=route.frame_id
            pose.pose=pose_at([x,y,0.],[0.,0.,math.sin(yaw/2),math.cos(yaw/2)])
            goal.poses.append(pose)
        self.ledger.emit('TRANSPORT',corridor_intent=vars(route),navigation_action='/navigate_through_poses',
            behavior_tree=str(path.resolve()),behavior_tree_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        self.corridor_witness=TraversalWitness(route)
        try:
            with path.with_suffix('.trajectory.jsonl').open('w') as self.corridor_samples:
                self.observe_corridor()
                self.action(self.nav_through,goal,self.c['navigation_timeout_s'])
                self.wait(self.stopped)
                proof=self.corridor_witness.result()
                self.confirm_navigation_pose(route.via_poses()[-1])
                self.ledger.emit('TRANSPORT',corridor_traversal=proof)
        except ValueError as error:raise TaskFailure(str(error)) from error
        finally:
            self.corridor_witness=None;self.corridor_samples=None

    def navigate_to(self, destination):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.c['map_frame']
        x, y, yaw = destination
        goal.pose.pose = pose_at([x, y, 0.], [0., 0., math.sin(yaw/2), math.cos(yaw/2)])
        self.action(self.nav, goal, self.c['navigation_timeout_s'])
        self.wait(self.stopped)
        self.confirm_navigation_pose(destination)

    def confirm_navigation_pose(self,destination):
        x,y,yaw=destination
        actual = self.transform(self.c['map_frame'], self.c['base_frame'])
        position_error = float(np.linalg.norm(actual[:2, 3] - [x, y]))
        actual_yaw = float(Rotation.from_matrix(actual[:3, :3]).as_euler('xyz')[2])
        yaw_error = abs(math.atan2(math.sin(actual_yaw-yaw), math.cos(actual_yaw-yaw)))
        if position_error > .03 or yaw_error > .03:
            raise TaskFailure('NAVIGATION_POSE_NOT_CONFIRMED')
        self.ledger.emit('TRANSPORT', navigation_position_error_m=position_error,
                         navigation_yaw_error_rad=yaw_error, base_xyz=actual[:3, 3].tolist(),
                         navigation_target=destination)

    def scene(self, full=False):
        request = GetPlanningScene.Request()
        request.components.components = 1023 if full else 16 | 4
        return self.call(self.get_scene, request).scene

    def apply_scene(self, scene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        if not self.call(self.apply, ApplyPlanningScene.Request(scene=scene)).success:
            raise TaskFailure('SCENE_APPLY_FAILED')
        self.scene_pub.publish(scene)  # local validated planner's scene monitor
        time.sleep(.15)

    def box(self, name, xyz, size):
        obj = CollisionObject(id=name)
        obj.header.frame_id = self.c['base_frame']
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, size)))]
        obj.primitive_poses = [pose_from_matrix(self.transform(self.c['base_frame'], self.c['map_frame']) @ matrix(pose_at(xyz)))]
        obj.operation = CollisionObject.ADD
        return obj

    def ign(self, service, request_type, request):
        command = ['ign', 'service', '-s', '/world/' + self.c['world'] + '/' + service,
                   '--reqtype', 'ignition.msgs.' + request_type, '--reptype', 'ignition.msgs.Boolean',
                   '--timeout', '1500', '--req', request]
        result = subprocess.run(command, capture_output=True, text=True, timeout=3.)
        if result.returncode or 'data: true' not in result.stdout:
            raise TaskFailure('GAZEBO_' + service.upper() + ':' + result.stdout[-200:])

    def spawn(self, name, xyz, size, collision):
        geom = '<geometry><box><size>' + ' '.join(map(str, size)) + '</size></box></geometry>'
        sdf = '<sdf version="1.7"><model name="' + name + '"><static>true</static><pose>'
        sdf += ' '.join(map(str, xyz)) + ' 0 0 0</pose><link name="body"><visual name="visual">'
        color = '0.45 0.45 0.45 1' if collision else '0.9 0.45 0.1 1'
        sdf += geom + '<material><diffuse>' + color + '</diffuse></material></visual>'
        if collision:
            sdf += '<collision name="collision">' + geom + '</collision>'
        sdf += '</link>'
        if not collision:
            # Per-entity PosePublisher stamps each pose at the simulation step.
            # SceneBroadcaster only stamps its Pose_V container, which the
            # generic TF bridge cannot preserve on individual transforms.
            sdf += ('<plugin filename="ignition-gazebo-pose-publisher-system" '
                    'name="ignition::gazebo::systems::PosePublisher">'
                    '<publish_model_pose>true</publish_model_pose>'
                    '<publish_link_pose>false</publish_link_pose>'
                    '<publish_nested_model_pose>true</publish_nested_model_pose>'
                    '<update_frequency>100</update_frequency></plugin>')
            if self.step_attachment:
                if self.payload_parent is None:raise TaskFailure('PAYLOAD_PARENT_UNRESOLVED')
                plugin=get_package_prefix('astribot_s1_gazebo_bringup')+'/lib/libastribot_kinematic_payload.so'
                if not Path(plugin).is_file():raise TaskFailure('PAYLOAD_STEP_PLUGIN_MISSING')
                sdf+=('<plugin filename="'+escape(plugin,{'"':'&quot;'})+'" name="astribot::KinematicPayload">'
                      '<parent_model>astribot_s1</parent_model><parent_link>'+escape(self.payload_parent)+
                      '</parent_link></plugin>')
        sdf += '</model></sdf>'
        self.ign('create', 'EntityFactory', 'sdf: ' + json.dumps(sdf) + ' allow_renaming: false')

    def stations(self):
        c = self.c
        for station in ('pick', 'place'):
            xyz = list(c[station + '_xyz'])
            height = xyz[2] - c['size_xyz'][2] / 2
            xyz[2] = height / 2
            size = c['station_size_xy'] + [height]
            name = c['object_id'] + '_' + station + '_station'
            yield name, xyz, size

    def create_scene(self):
        c = self.c
        if self.step_attachment:
            urdf=self.call(self.robot_params,GetParameters.Request(names=['robot_description'])).values[0].string_value
            self.payload_parent=physical_parent(urdf,c['tcp'])
        existing = self.scene()
        ids = [o.id for o in existing.world.collision_objects]
        ids += [o.object.id for o in existing.robot_state.attached_collision_objects]
        if c['object_id'] in ids:
            raise TaskFailure('OBJECT_ALREADY_EXISTS_RECOVERY_REQUIRED')
        objects = []
        for name, xyz, size in self.stations():
            self.spawn(name, xyz, size, True)
            objects.append(self.box(name, xyz, size))
        self.spawn(c['object_id'], c['pick_xyz'], c['size_xyz'], False)
        objects.append(self.box(c['object_id'], c['pick_xyz'], c['size_xyz']))
        self.apply_scene(PlanningScene(world=PlanningSceneWorld(collision_objects=objects)))
        self.wait(lambda: set(o.id for o in self.scene().world.collision_objects).issuperset(o.id for o in objects))
        if self.step_attachment:self.wait(lambda:self.payload_confirmed(False),5.)
        self.scene_created = True

    def attach(self):
        self.mtc_stage('ATTACH')
        c = self.c
        self.offset = self.transform(c['tcp'], c['map_frame']) @ matrix(pose_at(self.observed_pick))
        if np.linalg.norm(self.offset[:3, 3]) > c['grasp_offset_m'] + .04:
            raise TaskFailure('GRASP_TCP_OBJECT_DISTANCE')
        attached = AttachedCollisionObject(link_name=c['tcp'], touch_links=c['touch_links'])
        attached.object.id = c['object_id']
        attached.object.operation = CollisionObject.ADD
        diff = PlanningScene()
        diff.robot_state.attached_collision_objects = [attached]
        self.apply_scene(diff)
        self.wait(lambda: any(o.object.id == c['object_id'] for o in self.scene().robot_state.attached_collision_objects))
        if self.step_attachment:
            # The confirmed MoveIt body is authoritative for both the filter
            # and attachment. Do not substitute a slightly different RGB-D
            # center after MoveIt has attached the configured world object.
            body=next(o for o in self.scene().robot_state.attached_collision_objects if o.object.id==c['object_id'])
            if (body.link_name!=c['tcp'] or body.object.header.frame_id not in ('',c['tcp']) or
                len(body.object.primitives)!=1 or body.object.meshes or
                body.object.primitives[0].type!=SolidPrimitive.BOX or
                not np.allclose(body.object.primitives[0].dimensions,c['size_xyz'],atol=1e-9)):
                raise TaskFailure('PAYLOAD_ATTACHMENT_SHAPE_MISMATCH')
            self.offset=collision_primitive_matrix(body.object)
            parent_offset=self.transform(self.payload_parent,c['tcp'])@self.offset
            self.command_payload_step(pose_from_matrix(parent_offset),True)
        self.last_sync = time.monotonic()
        self.attached = True
        if not self.step_attachment:self.sync_payload()
        if self.fixed_v2:
            self.wait(lambda:self.geometry_state is not None and self.geometry_state.complete and
                      c['object_id'] in self.geometry_state.attachment_ids and
                      seconds(self.geometry_state.valid_until)>self.get_clock().now().nanoseconds*1e-9,10.)
        self.mtc_bundle.acknowledge()
        self.ledger.emit('ATTACH_CONFIRM', tcp_object_transform=self.offset.tolist(),
                         attachment_backend='gazebo_physics_step_kinematic' if self.step_attachment else 'gazebo_kinematic', force_grasp=False)

    def command_payload_step(self,pose,attached):
        self.payload_command+=1
        p,q=pose.position,pose.orientation
        name='astribot_s1::'+self.payload_parent if attached else ''
        request=(f'id: {self.payload_command} name: '+json.dumps(name)+
            f' position {{x:{p.x} y:{p.y} z:{p.z}}} orientation {{x:{q.x} y:{q.y} z:{q.z} w:{q.w}}}')
        result=subprocess.run(['ign','service','-s','/model/'+self.c['object_id']+'/kinematic_attachment/command',
            '--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','1500','--req',request],
            capture_output=True,text=True,timeout=3.)
        if result.returncode or 'data: true' not in result.stdout:
            raise TaskFailure('PAYLOAD_STEP_COMMAND_UNCONFIRMED')
        # Confirm the applied transaction at source time, not the queue ACK.
        # During detach, the old attached state deliberately cannot satisfy it.
        self.wait(lambda:self.payload_confirmed(attached),2.,checked=False)

    def set_payload_pose(self, pose):
        # A persistent ros_gz client avoids per-frame process creation and
        # Ignition discovery. Keep the existing freshness failure gate.
        if not self.gz_pose.service_is_ready():
            raise TaskFailure('GAZEBO_POSE_SERVICE_UNAVAILABLE')
        request = SetEntityPose.Request(entity=Entity(name=self.c['object_id'], type=Entity.MODEL), pose=pose)
        pending = self.gz_pose.call_async(request)
        try:
            result = self.future(pending, 1.5, checked=False)
        finally:
            if not pending.done():
                self.gz_pose.remove_pending_request(pending)
        if not result.success:
            raise TaskFailure('GAZEBO_SET_POSE_REJECTED')
        self.last_sync = time.monotonic()

    def observed_payload_position(self, name=None):
        result = subprocess.run(['ign', 'topic', '-e', '-t', '/world/' + self.c['world'] + '/pose/info',
                                 '-n', '1', '--json-output'], capture_output=True, text=True, timeout=5.)
        if result.returncode:
            raise TaskFailure('GAZEBO_POSE_OBSERVATION_FAILED')
        message = json.loads(result.stdout)
        for pose in message.get('pose', []):
            if pose.get('name') == (name or self.c['object_id']):
                return np.array([float(pose.get('position', {}).get(axis, 0.)) for axis in ('x', 'y', 'z')])
        raise TaskFailure('GAZEBO_OBJECT_NOT_OBSERVED')

    def sync_payload(self):
        with self.sync_lock:
            if self.attached:
                self.set_payload_pose(pose_from_matrix(self.transform(self.c['map_frame'], self.c['tcp']) @ self.offset))

    def sync_loop(self):
        while not self.sync_stop.wait(.1):
            if self.attached and not self.step_attachment:
                try:
                    self.sync_payload()
                except Exception as error:
                    # One transport timeout does not prove that Gazebo rejected
                    # the pose. Retry the latest pose; the freshness gate remains.
                    if time.monotonic() - self.last_sync > 2.5:
                        self.sync_error = 'PAYLOAD_SYNC:' + str(error)

    def confirm_place(self):
        c = self.c
        final = pose_from_matrix(self.transform(c['map_frame'], c['tcp']) @ self.offset)
        if not self.stopped() or np.linalg.norm(matrix(final)[:3, 3] - c['place_xyz']) > c['placement_tolerance_m']:
            raise TaskFailure('PLACE_POSE_OUT_OF_TOLERANCE')
        return final

    def detach(self):
        self.mtc_stage('DETACH')
        c = self.c
        final = self.confirm_place()
        attached = AttachedCollisionObject(link_name=c['tcp'])
        attached.object.id = c['object_id']
        attached.object.operation = CollisionObject.REMOVE
        diff = PlanningScene()
        diff.robot_state.attached_collision_objects = [attached]
        # Explicit world pose avoids a race with the monitor's last joint sample.
        obj = self.box(c['object_id'], c['place_xyz'], c['size_xyz'])
        obj.primitive_poses = [pose_from_matrix(self.transform(c['base_frame'], c['map_frame']) @ matrix(final))]
        diff.world.collision_objects = [obj]
        self.apply_scene(diff)
        self.wait(lambda: not any(o.object.id == c['object_id'] for o in self.scene().robot_state.attached_collision_objects))
        with self.sync_lock:
            if self.step_attachment:self.command_payload_step(final,False)
            else:self.set_payload_pose(final)
            self.attached = False
        self.final_pose = final
        self.mtc_payload_context = self.payload_context(self.scene())
        self.mtc_bundle.acknowledge()

    def verify_place(self):
        c = self.c
        scene = self.scene()
        objects = [o for o in scene.world.collision_objects if o.id == c['object_id']]
        if len(objects) != 1 or self.attached:
            raise TaskFailure('PLACEMENT_SCENE_MISMATCH')
        xyz = (self.transform(c['map_frame'], objects[0].header.frame_id) @ collision_primitive_matrix(objects[0]))[:3, 3]
        error = float(np.linalg.norm(xyz - c['place_xyz']))
        if error > c['placement_tolerance_m']:
            raise TaskFailure('PLACEMENT_ERROR')
        observed = self.observed_payload_position()
        time.sleep(.25)
        settled = self.observed_payload_position()
        observed_error = float(np.linalg.norm(settled - c['place_xyz']))
        if observed_error > c['placement_tolerance_m'] or np.linalg.norm(observed - settled) > .005:
            raise TaskFailure('GAZEBO_PLACEMENT_NOT_CONFIRMED')
        self.ledger.emit('PLACEMENT_VERIFY', position_error_m=error,
                         gazebo_position_error_m=observed_error, gazebo_position=settled.tolist(),
                         gazebo='two_pose_observations', force_grasp_validated=False)

    def stop_and_hold(self):
        try:
            self.cancel_active()
        except Exception as cancel_error:
            # An unknown executor outcome retains ownership, but must not skip
            # the independent attempt to withdraw navigation permission.
            try:
                self.change_envelope(False, checked=False)
            except Exception as hold_error:
                raise TaskFailure('STOP_AND_HOLD_FAILED:cancel=' + str(cancel_error) +
                                  ';hold=' + str(hold_error)) from cancel_error
            raise
        self.change_envelope(False, checked=False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--navigation-geometry-mode',choices=['legacy','fixed_v2'],default='legacy')
    parser.add_argument('--vla-config', help='Optional policy gateway config; absent preserves the MTC task.')
    parser.add_argument('--instruction', help='Language task instruction passed to the selected policy.')
    parser.add_argument('--resume-placed', action='store_true',
                        help='Reconcile a failed PLACED ledger, then only retreat/stow/verify.')
    args = parser.parse_args()
    config = json.loads(Path(args.scenario).read_text())
    config['navigation_geometry_mode']=args.navigation_geometry_mode
    if args.vla_config:
        from .vla_contract import validate_config
        config['vla'] = validate_config(json.loads(Path(args.vla_config).read_text()))
    if args.instruction:
        config['instruction'] = args.instruction
    if config.get('vla'):
        from .vla_contract import validate_config
        config['vla'] = validate_config(config['vla'])
        if args.resume_placed:
            raise SystemExit('PLACED recovery uses the existing recovery path; omit VLA configuration.')
    validate_scenario(config)
    if config.get('navigation_geometry_mode', 'legacy') == 'fixed_v2':
        raise SystemExit('CPP_TASK_HOLD_EXECUTOR_REQUIRED: use the native hold action; the legacy measured-only publisher is disabled.')
    output = Path(args.output)
    if (output / 'state.json').exists() and not args.resume_placed:
        raise SystemExit('Existing ledger: inspect recovery state; use a new output directory only after scene reconciliation.')
    rclpy.init()
    logger = get_logger('astribot.transport')
    domain = canonical_resource_domain(os.environ.get('ROS_DOMAIN_ID', '0'))
    with ResourceLease('/tmp/astribot_transport_domain_' + domain + '.lock') as lease:
        check_native_resource_release(Path.home() / '.local/state/astribot/transport' / ('domain_' + domain + '.jsonl'))
        lease.file.seek(0)
        previous = lease.file.read()
        if previous and json.loads(previous).get('unconfirmed_executor', False):
            raise SystemExit('Previous executor termination unconfirmed. Reconcile the recorded ledger before resetting the lease.')
        ledger = (Ledger.resume_placed(output, config['object_id']) if args.resume_placed
                  else Ledger(output, config['object_id']))
        lease.checkpoint(dict(pid=os.getpid(), ledger=str(output / 'state.json'), unconfirmed_executor=True))
        node = RosBackend(config, ledger)
        signal.signal(signal.SIGINT, lambda *_: setattr(node, 'cancel_requested', True))
        signal.signal(signal.SIGTERM, lambda *_: setattr(node, 'cancel_requested', True))
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()
        task = TransportTask(node, config, ledger)
        ok = task.resume_placed() if args.resume_placed else task.run()
        if node.vla:
            node.vla.close()
        node.publish_status()
        lease.checkpoint(dict(pid=os.getpid(), ledger=str(output / 'state.json'),
                              unconfirmed_executor=node.active is not None or node.pending_goal is not None))
        logger.info('Transport result=%s ledger=%s', 'PASS' if ok else 'FAIL', str(output / 'state.json'))
        # Preserve failed attachment state in MoveIt and the ledger. This process
        # stops publishing surrogate poses only after all actions have terminated.
        node.sync_stop.set()
        node.sync_thread.join(timeout=4.)
        executor.shutdown()
        spinner.join(timeout=3.)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)
