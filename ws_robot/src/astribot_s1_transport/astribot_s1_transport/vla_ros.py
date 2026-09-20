"""Observation/proposal adapter inside the existing transport task owner."""
import base64
import copy
from collections import deque
from dataclasses import asdict
import json
import math
import threading
import time
import uuid

import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException

from .core import TaskFailure
from .vla_contract import VERSION, UNITS, digest, fail
from .vla_policy import PolicySession


def stamp_ns(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def pose_dict(pose):
    p, q = pose.position, pose.orientation
    return dict(position=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w])


def transform_dict(stamped):
    t = stamped.transform
    return dict(position=[t.translation.x, t.translation.y, t.translation.z],
                quaternion=[t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])


class VlaBridge:
    def __init__(self, backend, config):
        self.io, self.config = backend, config
        self.lock = threading.Lock()
        self.buffers = {}
        self.record_lock = threading.Lock()
        self.record_index = 0
        self.last_request = None
        self.last_decision = 'not_started'
        self.directory = backend.ledger.directory / 'vla'
        self.directory.mkdir(exist_ok=True)
        self.session = PolicySession(config, str(uuid.uuid4()), self.record)
        self.subscriptions = []
        for camera in config['cameras']:
            for kind, key, message_type in [('rgb', 'rgb_topic', Image), ('depth', 'depth_topic', Image),
                                             ('info', 'info_topic', CameraInfo)]:
                if not camera.get(key):
                    continue
                buffer_key = (camera['id'], kind)
                self.buffers[buffer_key] = deque(maxlen=12)
                def receive(message, buffer_key=buffer_key):
                    with self.lock:
                        self.buffers[buffer_key].append(message)
                self.subscriptions.append(backend.create_subscription(message_type, camera[key], receive,
                                                                        qos_profile_sensor_data))

    def record(self, kind, value):
        with self.record_lock:
            path = self.directory / f'{self.record_index:04d}_{kind}.json'
            self.record_index += 1
            path.write_text(json.dumps(value, indent=2, allow_nan=False))

    def start(self):
        self.session.start(self.io.check)

    def binding(self, scene):
        io = self.io
        # Both payload geometry and attachment identity must stay unchanged while
        # inference runs. Live joint/base drift is checked separately below.
        attachments = []
        for attached in scene.robot_state.attached_collision_objects:
            from rosidl_runtime_py.convert import message_to_ordereddict
            body = copy.deepcopy(attached)
            body.object.header.stamp.sec = body.object.header.stamp.nanosec = 0
            attachments.append(message_to_ordereddict(body))
        return dict(object=asdict(io.ledger.object), scene=io.scene_context(scene),
                    payload=io.payload_context(scene), attachments=attachments,
                    hold_epoch=io.envelope.epoch, stage=io.ledger.stage,
                    calibration_epoch=(io.observation or {}).get('calibration_epoch'))

    def observation(self):
        io = self.io
        now = io.get_clock().now().nanoseconds
        cameras, stamps = {}, []
        with self.lock:
            buffers = {k: list(v) for k,v in self.buffers.items()}
        for camera in self.config['cameras']:
            key = camera['id']
            rgbs, infos = buffers[key, 'rgb'], buffers[key, 'info']
            if not rgbs or not infos:
                fail('CAMERA_NOT_READY:' + key)
            selected = None
            for rgb in reversed(rgbs):
                s = stamp_ns(rgb)
                if not -10_000_000 <= now-s <= round(io.c['observation_max_age_s']*1e9):
                    continue
                if not io.tf.can_transform(io.c['base_frame'], rgb.header.frame_id, Time(nanoseconds=s)):
                    continue
                info = min(infos, key=lambda m: abs(stamp_ns(m)-s))
                depth = None
                if camera.get('depth_topic'):
                    depths = buffers[key, 'depth']
                    if not depths:
                        continue
                    depth = min(depths, key=lambda m: abs(stamp_ns(m)-s))
                    if abs(stamp_ns(depth)-s) > 50_000_000:
                        continue
                if abs(stamp_ns(info)-s) <= 100_000_000:
                    selected = rgb, info, depth
                    break
            if selected is None:
                fail('CAMERA_UNSYNCHRONIZED:' + key)
            rgb, info, depth = selected
            s = stamp_ns(rgb)
            if (info.header.frame_id != rgb.header.frame_id or info.width != rgb.width or info.height != rgb.height or
                    info.k[0] <= 0 or info.k[4] <= 0 or not np.isfinite(list(info.k)+list(info.d)).all()):
                fail('CAMERA_INFO_INVALID:' + key)
            if rgb.encoding.lower() not in ('rgb8', 'bgr8'):
                fail('RGB_ENCODING:' + rgb.encoding)
            frames = dict(rgb=self.image_packet(rgb), camera_info=dict(stamp_ns=stamp_ns(info),
                frame_id=info.header.frame_id, width=info.width, height=info.height, k=list(info.k),
                d=list(info.d), p=list(info.p), distortion_model=info.distortion_model))
            if depth is not None:
                if (depth.encoding.upper() not in ('16UC1', '32FC1') or depth.header.frame_id != rgb.header.frame_id or
                        depth.width != rgb.width or depth.height != rgb.height):
                    fail('ALIGNED_DEPTH_REQUIRED:' + key)
                frames['depth'] = self.image_packet(depth)
                frames['depth']['scale_to_m'] = .001 if depth.encoding.upper() == '16UC1' else 1.
            frames['base_from_camera'] = transform_dict(io.tf.lookup_transform(
                io.c['base_frame'], rgb.header.frame_id, Time(nanoseconds=s)))
            frames['calibration_id'] = digest({k:v for k,v in frames['camera_info'].items() if k != 'stamp_ns'})
            cameras[key] = frames
            stamps.append(s)
        joints = io.joints
        if joints is None or not io.fresh(joints):
            fail('JOINTS_STALE')
        stamps.append(stamp_ns(joints))
        if max(stamps)-min(stamps) > 100_000_000:
            fail('MULTIMODAL_SKEW')
        positions = dict(zip(joints.name, joints.position))
        arm_names = [f'astribot_arm_left_joint_{i}' for i in range(1, 8)]
        if len(positions) != len(joints.name) or any(n not in positions for n in arm_names) or not all(
                math.isfinite(v) for v in positions.values()):
            fail('JOINT_STATE_INVALID')
        anchor = min(stamps)
        tcp = io.tf.lookup_transform(io.c['base_frame'], io.c['tcp'], Time(nanoseconds=anchor))
        return dict(stamp_ns=anchor, captured_ros_ns=now, clock_domain='ros_sim', cameras=cameras,
                    joints=dict(stamp_ns=stamp_ns(joints), names=list(joints.name), positions=list(joints.position)),
                    arm_joint_names=arm_names, tcp=transform_dict(tcp), tcp_frame=io.c['tcp'],
                    object_observation=copy.deepcopy(io.observation))

    @staticmethod
    def image_packet(image):
        pixel_bytes = 3 if image.encoding.lower() in ('rgb8', 'bgr8') else (2 if image.encoding.upper() == '16UC1' else 4)
        if (not 0 < image.width <= 4096 or not 0 < image.height <= 4096 or
                image.step < image.width*pixel_bytes or len(image.data) != image.step*image.height or
                len(image.data) > 8*1024*1024):
            fail('IMAGE_LAYOUT')
        return dict(stamp_ns=stamp_ns(image), frame_id=image.header.frame_id, width=image.width,
                    height=image.height, encoding=image.encoding, step=image.step,
                    is_bigendian=bool(image.is_bigendian), data_base64=base64.b64encode(bytes(image.data)).decode())

    def propose(self, goal, scene):
        io = self.io
        if self.session.capabilities is None:
            fail('SESSION_NOT_STARTED')
        observation = None
        def ready():
            nonlocal observation
            try:
                observation = self.observation()
                return True
            except TransformException:
                return False
            except TaskFailure as error:
                if str(error).startswith(('VLA_CAMERA_NOT_READY', 'VLA_CAMERA_UNSYNCHRONIZED', 'VLA_MULTIMODAL_SKEW')):
                    return False
                raise
        io.wait(ready, 10.)
        binding = self.binding(scene)
        base = io.transform(io.c['map_frame'], io.c['base_frame'])
        joint_start = dict(zip(io.joints.name, io.joints.position))
        request = dict(schema=VERSION, episode_id=self.session.episode_id, request_id=str(uuid.uuid4()),
                       sequence=self.session.sequence, context_id=digest(binding), operation=goal.operation,
                       instruction=io.c.get('instruction', 'Pick up the orange box and place it at the destination station.'),
                       frame_id=io.c['base_frame'], units=UNITS, observation=observation, context=binding,
                       allowed_action_types=(['mtc_targets'] if self.config['mode'] == 'mtc'
                                             else self.session.capabilities['action_types']),
                       nominal_action=dict(type='mtc_targets', frame_id=io.c['base_frame'], group='arm_left',
                           pre_target=pose_dict(goal.pre_target.pose), target=pose_dict(goal.target.pose),
                           exit_targets=[pose_dict(p.pose) for p in goal.exit_targets]))
        self.last_request = request['request_id']
        started = time.monotonic()
        try:
            action = self.session.infer(request, io.check)
            io.check()
            io.require_manipulation_hold()
            if (time.monotonic()-started > self.config['max_snapshot_age_s'] or
                    io.get_clock().now().nanoseconds-observation['stamp_ns'] > self.config['max_snapshot_age_s']*1e9):
                fail('SNAPSHOT_EXPIRED')
            if self.binding(io.scene(full=True)) != binding:
                fail('CONTEXT_CHANGED_DURING_INFERENCE')
            current = io.transform(io.c['map_frame'], io.c['base_frame'])
            if (np.linalg.norm(current[:3,3]-base[:3,3]) > .002 or
                    Rotation.from_matrix(base[:3,:3].T @ current[:3,:3]).magnitude() > .003):
                fail('BASE_MOVED_DURING_INFERENCE')
            actual = dict(zip(io.joints.name, io.joints.position))
            if any(abs(actual.get(n, math.inf)-v) > .01 for n,v in joint_start.items()):
                fail('JOINTS_MOVED_DURING_INFERENCE')
            # Capture fresh sensor evidence again, including intrinsics. Images
            # themselves naturally differ, but camera calibration cannot change.
            fresh = self.observation()
            if any(fresh['cameras'][k]['calibration_id'] != v['calibration_id']
                   for k,v in observation['cameras'].items()):
                fail('CAMERA_CALIBRATION_CHANGED')
            for key, camera in observation['cameras'].items():
                before = camera['base_from_camera']
                after = fresh['cameras'][key]['base_from_camera']
                if (math.dist(before['position'], after['position']) > .002 or
                        (Rotation.from_quat(before['quaternion']).inv() *
                         Rotation.from_quat(after['quaternion'])).magnitude() > .003):
                    fail('CAMERA_EXTRINSICS_CHANGED')
            if self.config['mode'] == 'mtc':
                goal.pre_target = self.pose(action['pre_target'], request['frame_id'])
                goal.target = self.pose(action['target'], request['frame_id'])
                goal.exit_targets = [self.pose(p, request['frame_id']) for p in action['exit_targets']]
                self.last_decision = 'accepted_for_mtc_planning'
            else:
                self.last_decision = 'shadow_only'
            self.record('decision', dict(request_id=self.last_request, decision=self.last_decision))
            self.feedback('proposal', self.last_decision)
            io.ledger.emit(io.ledger.stage, vla_request_id=self.last_request, vla_decision=self.last_decision,
                           vla_policy=self.session.capabilities['policy_id'])
            return goal
        except Exception as error:
            self.last_decision = 'rejected'
            self.record('decision', dict(request_id=self.last_request, decision='rejected', reason=str(error)))
            self.feedback('proposal', 'rejected', str(error))
            raise

    @staticmethod
    def pose(value, frame):
        result = PoseStamped()
        result.header.frame_id = frame
        result.pose.position.x, result.pose.position.y, result.pose.position.z = map(float, value['position'])
        q = result.pose.orientation
        q.x, q.y, q.z, q.w = map(float, value['quaternion'])
        return result

    def feedback(self, stage, status, reason=''):
        packet = dict(request_id=self.last_request, stage=stage, status=status, reason=reason,
                      object=asdict(self.io.ledger.object), stamp_ns=self.io.get_clock().now().nanoseconds)
        self.record('feedback', packet)
        self.session.enqueue('feedback', packet)

    def close(self):
        self.session.close(dict(status=self.io.ledger.stage, object=asdict(self.io.ledger.object)))
