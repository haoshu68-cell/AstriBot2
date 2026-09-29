"""Identical ROS inputs for the frozen Python arbiter and native executable.

Only fake Nav2 action servers are used. Backend completion is explicitly gated;
no test treats cancel acknowledgement as terminal or physical stop evidence.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import SetParametersAtomically
from rosgraph_msgs.msg import Clock
from astribot_navigation_msgs.msg import NavigationExecutionStatus, NavigationEnvelopeV2

ROOT = Path(__file__).resolve().parents[2]
ORACLE = ROOT / 'astribot_s1_navigation_policy/test/reference/task_arbiter_node.py'
REFERENCE = "import runpy; runpy.run_path(" + repr(str(ORACLE)) + ")[\"main\"]()"


def wait(predicate, timeout=5.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.002)
    raise AssertionError('timed out waiting for isolated ROS contract')


class Backend:
    def __init__(self, runtime, kind, name):
        self.runtime, self.kind = runtime, kind
        self.goals, self.handles, self.cancels = [], [], []
        self.finished = []
        self.accept = True
        self.cancel_accept = True
        self.response_gate = threading.Event()
        self.response_gate.set()
        self.done = threading.Event()
        self.result_status = GoalStatus.STATUS_SUCCEEDED
        self.server = ActionServer(runtime.node, kind, runtime.prefix + '/navigation_executor/' + name,
            execute_callback=self.execute, goal_callback=self.goal,
            cancel_callback=self.cancel, callback_group=runtime.group)

    def goal(self, request):
        self.goals.append(request)
        assert self.response_gate.wait(8.), 'test fixture goal-response gate expired'
        return GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT

    def cancel(self, handle):
        self.cancels.append(bytes(handle.goal_id.uuid).hex())
        return CancelResponse.ACCEPT if self.cancel_accept else CancelResponse.REJECT

    def execute(self, handle):
        self.handles.append(handle)
        until = time.monotonic() + 12.
        while not self.done.wait(.02):
            if self.runtime.closing or time.monotonic() > until:
                handle.abort()
                self.finished.append(bytes(handle.goal_id.uuid).hex())
                return self.kind.Result()
        if self.result_status == GoalStatus.STATUS_CANCELED and handle.is_cancel_requested:
            handle.canceled()
        elif self.result_status == GoalStatus.STATUS_SUCCEEDED:
            handle.succeed()
        else:
            handle.abort()
        self.finished.append(bytes(handle.goal_id.uuid).hex())
        return self.kind.Result()

    def feedback(self):
        message = self.kind.Feedback()
        message.current_pose.header.frame_id = 'map_test_frame'
        message.current_pose.pose.position.x = 1.25
        message.navigation_time.sec = 3
        message.estimated_time_remaining.sec = 7
        message.number_of_recoveries = 2
        message.distance_remaining = 4.5
        if self.kind is NavigateThroughPoses:
            message.number_of_poses_remaining = 3
        self.handles[-1].publish_feedback(message)
        return message

    def close(self):
        self.response_gate.set()
        self.done.set()


class Runtime:
    def __init__(self, impl, directory, timeout=1.0, backend=True, sim=False, fixed_v2=False):
        assert not fixed_v2 or impl == 'cpp', 'the frozen legacy oracle has no fixed-v2 gate'
        self.impl, self.directory = impl, directory
        self.prefix = '/arbiter_' + uuid.uuid4().hex[:12]
        self.node_name = self.prefix[1:] + '_runtime'
        self.closing = False
        self.context = Context()
        self.domain = int(os.environ.get('ARBITER_DOMAIN', '150'))
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init(context=self.context, domain_id=self.domain)
        self.node = Node('arbiter_probe_' + uuid.uuid4().hex[:8], context=self.context)
        self.group = ReentrantCallbackGroup()
        self.events, self.feedbacks = [], []
        self.status_sub = self.node.create_subscription(NavigationExecutionStatus,
            self.prefix + '/navigation/execution_status', self.events.append,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.group)
        self.backends = {}
        if backend:
            self.add_backend('pose')
            self.add_backend('poses')
        self.clients = {}
        for key, kind, name in [('pose', NavigateToPose, 'navigate_to_pose'),
                                ('poses', NavigateThroughPoses, 'navigate_through_poses')]:
            for source, prefix in [('operator', ''), ('route', '/route'), ('exploration', '/exploration')]:
                self.clients[key, source] = ActionClient(self.node, kind, self.prefix + prefix + '/' + name,
                    callback_group=self.group)
        self.parameters = self.node.create_client(SetParametersAtomically,
            '/' + self.node_name + '/set_parameters_atomically', callback_group=self.group)
        self.clock = self.node.create_publisher(Clock, self.prefix + '/clock', 10)
        self.envelope_lock = threading.Lock()
        self.envelope_enabled = False
        self.envelope_epoch = 1
        self.envelope_allowed = True
        self.envelope_reason = 'READY_FIXED'
        self.envelope_sim_ns = 100_000_000_000 if sim else None
        self.envelope_messages = []
        self.envelope_pub = self.envelope_timer = None
        if fixed_v2:
            self.envelope_pub = self.node.create_publisher(NavigationEnvelopeV2,
                self.prefix + '/navigation/envelope_v2', 10)
            self.envelope_timer = self.node.create_timer(.05, self.publish_envelope,
                callback_group=self.group)
        self.executor = MultiThreadedExecutor(num_threads=8, context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()
        command = ([os.environ['ARBITER_CPP']] if impl == 'cpp' else
                   [sys.executable, '-c', REFERENCE])
        command += ['--ros-args', '-r', '__node:=' + self.node_name, '-p', f'handover_timeout_s:={timeout}',
                    '-p', 'use_sim_time:=' + str(sim).lower()]
        endpoints = ['/navigation/execution_status', '/clock']
        if fixed_v2:
            command += ['-p', 'navigation_geometry_mode:=fixed_v2']
            endpoints.append('/navigation/envelope_v2')
        for prefix in ['', '/route', '/exploration', '/navigation_executor']:
            for name in ['navigate_to_pose', 'navigate_through_poses']:
                for suffix in ['send_goal', 'get_result', 'cancel_goal', 'feedback', 'status']:
                    endpoints.append(prefix + '/' + name + '/_action/' + suffix)
        for endpoint in endpoints:
            command += ['-r', endpoint + ':=' + self.prefix + endpoint]
        self.log_path = directory / ('arbiter_' + impl + '_' + uuid.uuid4().hex[:8] + '.log')
        self.log = self.log_path.open('w')
        self.proc = subprocess.Popen(command, env=dict(os.environ,
            ROS_DOMAIN_ID=str(self.domain), ROS_LOCALHOST_ONLY='1'),
            stdout=self.log, stderr=subprocess.STDOUT)
        try:
            wait(lambda: self.proc.poll() is not None or self.clients['pose', 'operator'].server_is_ready() and
                 self.clients['poses', 'exploration'].server_is_ready() and
                 self.parameters.service_is_ready(), 8.)
            assert self.proc.poll() is None, self.log_path.read_text()
            actual = (Path('/proc') / str(self.proc.pid) / 'environ').read_bytes().split(b'\0')
            assert f'ROS_DOMAIN_ID={self.domain}'.encode() in actual
            assert b'ROS_LOCALHOST_ONLY=1' in actual
        except BaseException:
            from rclpy.action import get_action_names_and_types
            print('discovery failure:', self.node.get_node_names(), get_action_names_and_types(self.node), self.node.get_service_names_and_types())
            self.close()
            raise

    def add_backend(self, key):
        kind, name = ((NavigateToPose, 'navigate_to_pose') if key == 'pose' else
                      (NavigateThroughPoses, 'navigate_through_poses'))
        self.backends[key] = Backend(self, kind, name)
        return self.backends[key]

    def publish_envelope(self):
        with self.envelope_lock:
            if self.closing or not self.envelope_enabled:
                return
            message = NavigationEnvelopeV2()
            now_ns = (self.node.get_clock().now().nanoseconds if self.envelope_sim_ns is None
                      else self.envelope_sim_ns)
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(now_ns, 1_000_000_000)
            message.valid_until.sec, message.valid_until.nanosec = divmod(now_ns + 300_000_000, 1_000_000_000)
            message.header.frame_id = 'astribot_torso_base'
            message.coordinator_session_id = self.prefix + '_coordinator'
            message.installed_geometry_hash = 'fixture_installed_geometry'
            message.hold_id = 'fixture_owned_hold'
            message.model_revision = 'fixture_model_revision'
            message.attachment_revision = 'fixture_attachment_revision'
            message.request_id = 'fixture_fixed_request'
            message.epoch = self.envelope_epoch
            message.clock_epoch = 1
            message.reference_state_sequence = 1
            message.source_state_sequence = len(self.envelope_messages) + 1
            message.mode = NavigationEnvelopeV2.FIXED_POSTURE
            message.navigation_allowed = self.envelope_allowed
            message.reason = self.envelope_reason
            message.limits.stamp = message.header.stamp
            message.limits.epoch = message.epoch
            message.limits.lease_s = .3
            message.limits.frame_id = message.header.frame_id
            message.limits.posture_id = 'fixture_measured_hold'
            message.limits.transport_ready = self.envelope_allowed
            message.limits.half_length_m = .5
            message.limits.half_width_m = .4
            message.limits.height_m = 1.5
            message.limits.max_speed_m_s = .2
            message.limits.max_angular_speed_rad_s = .3
            message.limits.max_acceleration_m_s2 = .2
            message.limits.brake_deceleration_m_s2 = .3
            from geometry_msgs.msg import Point32
            message.reserved_footprint.points = [Point32(x=x, y=y, z=0.)
                for x, y in [(-.5, -.4), (.5, -.4), (.5, .4), (-.5, .4)]]
            message.installed_footprint = message.reserved_footprint
            self.envelope_pub.publish(message)
            self.envelope_messages.append(message)

    def envelope(self, allowed=None, epoch=None, reason=None, publishing=True):
        assert self.envelope_pub is not None
        with self.envelope_lock:
            if allowed is not None: self.envelope_allowed = allowed
            if epoch is not None: self.envelope_epoch = epoch
            if reason is not None: self.envelope_reason = reason
            self.envelope_enabled = publishing
        self.publish_envelope()

    def sim_clock(self, nanoseconds):
        assert self.envelope_sim_ns is not None
        with self.envelope_lock:
            self.envelope_sim_ns = nanoseconds
        message = Clock()
        message.clock.sec, message.clock.nanosec = divmod(nanoseconds, 1_000_000_000)
        self.clock.publish(message)

    def send(self, source='operator', key='pose'):
        kind = NavigateToPose if key == 'pose' else NavigateThroughPoses
        goal = kind.Goal()
        from geometry_msgs.msg import PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'map_test_goal'
        pose.header.stamp.sec = 123
        pose.pose.position.x = 7.25
        pose.pose.orientation.w = .9
        if key == 'pose': goal.pose = pose
        else: goal.poses = [pose, pose]
        goal.behavior_tree = 'fixture_tree.xml'
        future = self.clients[key, source].send_goal_async(goal,
            feedback_callback=lambda msg: self.feedbacks.append(msg.feedback))
        wait(future.done)
        handle = future.result()
        if handle.accepted:
            handle.result_future = handle.get_result_async()
        return handle

    def states(self, handle):
        uid = bytes(handle.goal_id.uuid).hex()
        return [(e.state, e.reason, e.action_status) for e in self.events if e.task_id == uid]

    def result(self, handle):
        wait(handle.result_future.done)
        result = handle.result_future.result()
        wait(lambda: any(s[0] in ['SUCCEEDED', 'FAILED', 'PREEMPTED', 'CANCELED']
             for s in self.states(handle)))
        return result

    def cancel(self, handle):
        response = handle.cancel_goal_async()
        wait(response.done)
        assert response.result().goals_canceling

    def set_timeout(self, timeout):
        request = SetParametersAtomically.Request(parameters=[
            Parameter('handover_timeout_s', value=timeout).to_parameter_msg()])
        f = self.parameters.call_async(request)
        wait(f.done)
        assert f.result().result.successful

    def close(self):
        self.closing = True
        if self.envelope_timer is not None: self.envelope_timer.cancel()
        for b in self.backends.values(): b.close()
        if getattr(self, 'proc', None):
            self.proc.send_signal(signal.SIGINT)
            try: self.proc.wait(4.)
            except subprocess.TimeoutExpired:
                self.proc.kill(); self.proc.wait(2.)
        # Let gated fake-server callbacks return before shutting their executor.
        # This avoids abandoning a newly scheduled execute coroutine when the
        # runtime under test exits before its delayed goal response arrives.
        wait(lambda: all(not b.accept or len(b.finished) >= len(b.goals)
                         for b in self.backends.values()), 3.)
        self.executor.shutdown(timeout_sec=3.)
        self.thread.join(3.)
        for b in self.backends.values(): b.server.destroy()
        for c in self.clients.values(): c.destroy()
        self.node.destroy_node()
        self.context.try_shutdown()
        if getattr(self, 'log', None): self.log.close()
        self.directory.joinpath('trace_' + self.log_path.stem + '.json').write_text(json.dumps([
            dict(task_id=e.task_id, source=e.source, sequence=e.sequence, state=e.state,
                 reason=e.reason, action_status=e.action_status,
                 stamp=[e.stamp.sec, e.stamp.nanosec]) for e in self.events], indent=2))


@pytest.fixture(params=['python', 'cpp'])
def runtime(request, tmp_path):
    r = Runtime(request.param, tmp_path)
    try: yield r
    finally: r.close()


@pytest.mark.parametrize('source', ['operator', 'route', 'exploration'])
@pytest.mark.parametrize('key', ['pose', 'poses'])
def test_goal_feedback_and_result(runtime, source, key):
    r = runtime; b = r.backends[key]
    h = r.send(source, key)
    assert h.accepted
    wait(lambda: b.handles)
    request = b.goals[0]
    pose = request.pose if key == 'pose' else request.poses[0]
    assert request.behavior_tree == 'fixture_tree.xml'
    assert pose.header.frame_id == 'map_test_goal' and pose.header.stamp.sec == 123
    assert pose.pose.position.x == 7.25 and pose.pose.orientation.w == .9
    if key == 'poses': assert len(request.poses) == 2
    expected = b.feedback()
    wait(lambda: r.feedbacks)
    assert r.feedbacks[-1] == expected
    b.done.set()
    assert r.result(h).status == GoalStatus.STATUS_SUCCEEDED
    assert r.states(h) == [('ACCEPTED', '', 0), ('EXECUTING', '', 0),
                           ('SUCCEEDED', 'EXECUTOR_RESULT', 4)]
    events = [e for e in r.events if e.task_id == bytes(h.goal_id.uuid).hex()]
    assert all(e.source == source for e in events)
    assert all(a.sequence < z.sequence for a, z in zip(events, events[1:]))
    assert all(e.stamp.sec > 0 for e in events)


@pytest.mark.parametrize('status', [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED])
def test_backend_failure(runtime, status):
    r = runtime; b = r.backends['pose']
    h = r.send(); wait(lambda: b.handles)
    if status == GoalStatus.STATUS_CANCELED:
        # A backend may cancel for its own reason; request direct backend cancel
        # only in this fixture, never as a production bypass.
        from action_msgs.srv import CancelGoal
        service = r.node.create_client(CancelGoal,
            r.prefix + '/navigation_executor/navigate_to_pose/_action/cancel_goal', callback_group=r.group)
        wait(service.service_is_ready)
        f = service.call_async(CancelGoal.Request()); wait(f.done)
        wait(lambda: b.handles[0].is_cancel_requested)
    b.result_status = status; b.done.set()
    assert r.result(h).status == GoalStatus.STATUS_ABORTED
    assert r.states(h)[-1] == ('FAILED', 'EXECUTOR_RESULT', status)


def test_cancel_ack_is_not_terminal(runtime):
    r = runtime; b = r.backends['pose']
    h = r.send(); wait(lambda: b.handles)
    r.cancel(h); wait(lambda: b.cancels)
    time.sleep(.08)
    assert not h.result_future.done()
    assert not r.send('route', 'poses').accepted
    b.result_status = GoalStatus.STATUS_CANCELED; b.done.set()
    assert r.result(h).status == GoalStatus.STATUS_CANCELED
    assert r.states(h)[-1] == ('CANCELED', 'EXECUTOR_RESULT', 5)


@pytest.mark.parametrize('equal', [False, True])
def test_replace_busy_and_delayed_terminal(runtime, equal):
    r = runtime; old = r.backends['pose']; new = r.backends['poses']
    first = r.send('operator' if equal else 'route')
    wait(lambda: old.handles)
    replacement = r.send('operator', 'poses'); assert replacement.accepted
    wait(lambda: old.cancels)
    assert not r.send('operator').accepted
    time.sleep(.08)
    assert not new.goals and not first.result_future.done()
    old.result_status = GoalStatus.STATUS_CANCELED; old.done.set()
    assert r.result(first).status == GoalStatus.STATUS_ABORTED
    assert r.states(first)[-1] == ('PREEMPTED', 'HIGHER_OR_EQUAL_PRIORITY_TASK', 5)
    wait(lambda: new.handles)
    new.done.set(); assert r.result(replacement).status == GoalStatus.STATUS_SUCCEEDED


def test_cancel_reject_and_handover_timeout(runtime):
    r = runtime; old = r.backends['pose']; old.cancel_accept = False
    first = r.send('route'); wait(lambda: old.handles)
    replacement = r.send('operator', 'poses'); wait(lambda: old.cancels)
    assert r.result(replacement).status == GoalStatus.STATUS_ABORTED
    assert r.states(replacement)[-1] == ('FAILED', 'PREVIOUS_TASK_NOT_TERMINAL', 0)
    assert not r.backends['poses'].goals and not first.result_future.done()
    assert not r.send('exploration', 'poses').accepted
    assert len(old.cancels) == 1
    old.done.set(); r.result(first)
    assert r.states(first)[-1] == ('PREEMPTED', 'HIGHER_OR_EQUAL_PRIORITY_TASK', 4)
    recovery = r.send('exploration', 'poses'); assert recovery.accepted
    wait(lambda: r.backends['poses'].handles)
    r.backends['poses'].done.set(); r.result(recovery)


def test_pending_cancel_keeps_old_owner(runtime):
    r = runtime; old = r.backends['pose']
    first = r.send('route'); wait(lambda: old.handles)
    pending = r.send('operator', 'poses'); r.cancel(pending)
    assert r.result(pending).status == GoalStatus.STATUS_CANCELED
    assert r.states(pending)[-1] == ('CANCELED', 'USER_CANCEL', 0)
    assert not r.backends['poses'].goals
    assert not r.send('exploration').accepted
    old.done.set(); r.result(first)


def test_late_goal_response_still_canceled_before_release(runtime):
    r = runtime; old = r.backends['pose']; old.response_gate.clear()
    first = r.send('route'); wait(lambda: old.goals)
    replacement = r.send('operator', 'poses')
    time.sleep(.08)
    assert not old.cancels and not r.backends['poses'].goals
    old.response_gate.set(); wait(lambda: old.cancels)
    assert not first.result_future.done() and not r.backends['poses'].goals
    old.result_status = GoalStatus.STATUS_CANCELED; old.done.set()
    r.result(first); wait(lambda: r.backends['poses'].handles)
    r.backends['poses'].done.set(); r.result(replacement)
    assert [s[0] for s in r.states(first)] == ['ACCEPTED', 'CANCELING', 'EXECUTING', 'PREEMPTED']


def test_executor_reject(runtime):
    r = runtime; r.backends['pose'].accept = False
    h = r.send(); assert h.accepted
    assert r.result(h).status == GoalStatus.STATUS_ABORTED
    assert r.states(h) == [('ACCEPTED', '', 0), ('FAILED', 'EXECUTOR_REJECTED', 0)]


@pytest.mark.parametrize('impl', ['python', 'cpp'])
@pytest.mark.parametrize('recover', [False, True])
def test_unavailable_recovery_and_wall_timeout(impl, recover, tmp_path):
    r = Runtime(impl, tmp_path, timeout=1.2, backend=False, sim=True)
    try:
        started = time.monotonic(); h = r.send()
        if recover:
            b = r.add_backend('pose'); wait(lambda: b.handles)
            b.done.set(); assert r.result(h).status == GoalStatus.STATUS_SUCCEEDED
        else:
            assert r.result(h).status == GoalStatus.STATUS_ABORTED
            assert r.states(h)[-1] == ('FAILED', 'EXECUTOR_UNAVAILABLE', 0)
            assert 1.1 < time.monotonic() - started < 2.5
        # No /clock: stamps stay zero, but admission/server deadlines advance.
        assert all(e.stamp.sec == 0 and e.stamp.nanosec == 0 for e in r.events)
    finally: r.close()


@pytest.mark.parametrize('impl', ['python', 'cpp'])
def test_parameter_snapshot_consumption(impl, tmp_path):
    r = Runtime(impl, tmp_path, timeout=.35, backend=False)
    try:
        r.set_timeout(3.0)
        start = time.monotonic(); h = r.send(); r.result(h)
        assert .28 < time.monotonic() - start < 1.4
        assert r.states(h)[-1] == ('FAILED', 'EXECUTOR_UNAVAILABLE', 0)
    finally: r.close()


@pytest.mark.parametrize('accept', [True, False])
def test_user_cancel_before_late_goal_response(runtime, accept):
    r = runtime; b = r.backends['pose']; b.accept = accept
    b.response_gate.clear()
    h = r.send(); wait(lambda: b.goals)
    r.cancel(h)
    time.sleep(.04)
    assert not h.result_future.done() and not b.cancels
    assert not r.send('route', 'poses').accepted
    b.response_gate.set()
    if accept:
        wait(lambda: b.cancels)
        assert not h.result_future.done()
        b.result_status = GoalStatus.STATUS_CANCELED; b.done.set()
    assert r.result(h).status == GoalStatus.STATUS_CANCELED
    assert r.states(h)[-1] == ('CANCELED', 'EXECUTOR_RESULT' if accept else 'EXECUTOR_REJECTED',
                               5 if accept else 0)


def test_late_response_after_pending_timeout(runtime):
    r = runtime; old = r.backends['pose']; old.response_gate.clear()
    first = r.send('route'); wait(lambda: old.goals)
    pending = r.send('operator', 'poses')
    r.result(pending)
    assert r.states(pending)[-1] == ('FAILED', 'PREVIOUS_TASK_NOT_TERMINAL', 0)
    assert not first.result_future.done()
    assert not r.send('exploration').accepted
    old.response_gate.set(); wait(lambda: old.cancels)
    assert not first.result_future.done() and not r.backends['poses'].goals
    old.done.set(); r.result(first)
    # A completed old generation must not release this new active owner.
    newest = r.send('exploration', 'poses'); wait(lambda: r.backends['poses'].handles)
    time.sleep(.05)
    assert not newest.result_future.done()
    r.backends['poses'].done.set(); assert r.result(newest).status == 4


@pytest.mark.parametrize('impl', ['python', 'cpp'])
def test_preemption_while_server_recovers(impl, tmp_path):
    r = Runtime(impl, tmp_path, timeout=1.2, backend=False)
    try:
        old = r.send('route'); time.sleep(.06)
        pending = r.send('operator', 'poses'); assert pending.accepted
        original_backend = r.add_backend('pose')
        replacement_backend = r.add_backend('poses')
        assert r.result(old).status == GoalStatus.STATUS_ABORTED
        assert r.states(old)[-1] == ('PREEMPTED', 'CANCELED_BEFORE_DISPATCH', 0)
        assert not original_backend.goals
        wait(lambda: replacement_backend.handles)
        replacement_backend.done.set(); r.result(pending)
    finally: r.close()


@pytest.mark.parametrize('impl', ['python', 'cpp'])
def test_ros_clock_and_transient_status(impl, tmp_path):
    r = Runtime(impl, tmp_path, sim=True)
    try:
        wait(lambda: r.clock.get_subscription_count() == 1)
        clock = Clock(); clock.clock.sec = 100
        for _ in range(15):
            r.clock.publish(clock); time.sleep(.01)
        h = r.send(); wait(lambda: r.backends['pose'].handles)
        wait(lambda: len(r.states(h)) >= 2)
        assert all(e.stamp.sec == 100 for e in r.events)
        clock.clock.sec = 5
        for _ in range(15):
            r.clock.publish(clock); time.sleep(.01)
        r.backends['pose'].done.set(); r.result(h)
        assert r.events[-1].stamp.sec == 5
        replay = []
        sub = r.node.create_subscription(NavigationExecutionStatus,
            r.prefix + '/navigation/execution_status', replay.append,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=r.group)
        wait(lambda: len(replay) >= 3)
        assert [(e.state, e.sequence) for e in replay] == [(e.state, e.sequence) for e in r.events]
    finally: r.close()


@pytest.mark.parametrize('impl', ['python', 'cpp'])
@pytest.mark.parametrize('value', ['0.0', '-1.0', '60.1', '.nan', '.inf', '-.inf', '10', 'true', '"10.0"'])
def test_invalid_startup_nonzero(impl, value, tmp_path):
    command = ([os.environ['ARBITER_CPP']] if impl == 'cpp' else [sys.executable, '-c', REFERENCE])
    command += ['--ros-args', '-p', 'handover_timeout_s:=' + value]
    result = subprocess.run(command, env=dict(os.environ, ROS_DOMAIN_ID='151',
        ROS_LOCALHOST_ONLY='1'), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=8.)
    tmp_path.joinpath('invalid_startup.log').write_bytes(result.stdout)
    assert result.returncode != 0


@pytest.mark.parametrize('impl', ['python', 'cpp'])
@pytest.mark.parametrize('value', ['0.01', '10.0', '60.0'])
def test_valid_startup_numeric_conversion(impl, value, tmp_path):
    r = Runtime(impl, tmp_path, timeout=value)
    try:
        h = r.send(); wait(lambda: r.backends['pose'].handles)
        r.backends['pose'].done.set()
        assert r.result(h).status == 4
    finally: r.close()


@pytest.mark.parametrize('impl', ['python', 'cpp'])
@pytest.mark.parametrize('phase', ['goal_response', 'result'])
def test_shutdown_with_unresolved_backend_is_bounded(impl, phase, tmp_path):
    r = Runtime(impl, tmp_path)
    try:
        b = r.backends['pose']
        if phase == 'goal_response': b.response_gate.clear()
        h = r.send()
        wait(lambda: b.goals if phase == 'goal_response' else b.handles)
        assert not h.result_future.done()
        # Snapshot parameter service remains responsive with the backend held.
        r.set_timeout(2.0)
        start = time.monotonic()
        r.proc.send_signal(signal.SIGINT)
        r.proc.wait(timeout=2.)
        assert r.proc.returncode == 0
        assert time.monotonic() - start < 2.
    finally: r.close()


@pytest.fixture
def fixed_runtime(tmp_path):
    r = Runtime('cpp', tmp_path, fixed_v2=True)
    try:
        wait(lambda: r.envelope_pub.get_subscription_count() == 1)
        yield r
    finally:
        r.close()


def fresh_fixture_envelope(runtime, **changes):
    count = len(runtime.envelope_messages)
    runtime.envelope(**changes)
    # Keep refreshing while DDS discovers/delivers, without freezing a stamp.
    wait(lambda: len(runtime.envelope_messages) >= count + 3)


def envelope_failed(runtime, handle, reason):
    assert runtime.result(handle).status == GoalStatus.STATUS_ABORTED
    state, actual_reason, _ = runtime.states(handle)[-1]
    assert state == 'FAILED' and actual_reason == reason
    assert not any(state == 'PREEMPTED' for state, _, _ in runtime.states(handle))


@pytest.mark.parametrize('source', ['operator', 'route', 'exploration'])
@pytest.mark.parametrize('key', ['pose', 'poses'])
def test_fixed_v2_six_entry_admission(fixed_runtime, source, key):
    r = fixed_runtime; b = r.backends[key]
    missing = r.send(source, key)
    assert not missing.accepted  # No observation is not permission.
    wait(lambda: r.states(missing))
    assert r.states(missing) == [('REJECTED', 'ENVELOPE_MISSING', 0)]
    fresh_fixture_envelope(r, allowed=False, reason='FIXTURE_REVOKED')
    denied = r.send(source, key); assert not denied.accepted
    wait(lambda: r.states(denied))
    assert r.states(denied) == [('REJECTED', 'ENVELOPE_REVOKED: FIXTURE_REVOKED', 0)]
    assert not any(backend.goals for backend in r.backends.values())
    fresh_fixture_envelope(r, allowed=True, reason='READY_FIXED')
    h = r.send(source, key); assert h.accepted
    wait(lambda: b.handles)
    b.done.set()
    assert r.result(h).status == GoalStatus.STATUS_SUCCEEDED
    assert r.states(h)[-1] == ('SUCCEEDED', 'EXECUTOR_RESULT', GoalStatus.STATUS_SUCCEEDED)


def test_fixed_v2_revoke_then_immediate_recovery_keeps_terminal_barrier(fixed_runtime):
    r = fixed_runtime; b = r.backends['pose']
    fresh_fixture_envelope(r)
    # Publisher counts do not prove the arbiter callback has received a sample.
    # Bootstrap only with explicitly rejected, zero-child requests; never retry
    # an accepted goal or a different failure from the actual test operation.
    deadline = time.monotonic() + 5.
    while True:
        h = r.send()
        if h.accepted:
            break
        wait(lambda: r.states(h))
        assert r.states(h) == [('REJECTED', 'ENVELOPE_MISSING', 0)]
        assert not b.goals and time.monotonic() < deadline
        from rclpy.duration import Duration
        assert r.envelope_pub.wait_for_all_acked(Duration(seconds=.3))
        time.sleep(.05)
    wait(lambda: b.handles)
    # Two reliable same-topic messages, no wait for a cancel or polling tick.
    # Recovery must not erase the invalidation that was already received.
    r.envelope(allowed=False, reason='FIXTURE_REVOKED')
    r.envelope(allowed=True, reason='READY_FIXED')
    wait(lambda: b.cancels)
    time.sleep(.08)
    assert len(b.cancels) == 1 and not h.result_future.done()
    assert not r.send('route', 'poses').accepted
    assert not r.backends['poses'].goals
    b.result_status = GoalStatus.STATUS_CANCELED; b.done.set()
    envelope_failed(r, h, 'ENVELOPE_REVOKED: FIXTURE_REVOKED')
    assert len(b.cancels) == 1


def test_fixed_v2_epoch_change_cancels_current_execution(fixed_runtime):
    r = fixed_runtime; b = r.backends['pose']
    fresh_fixture_envelope(r)
    h = r.send(); wait(lambda: b.handles)
    r.envelope(epoch=2)
    wait(lambda: b.cancels)
    assert not h.result_future.done()
    b.result_status = GoalStatus.STATUS_CANCELED; b.done.set()
    envelope_failed(r, h, 'NAVIGATION_ENVELOPE_CHANGED')
    assert len(b.cancels) == 1


def test_fixed_v2_late_accept_is_canceled_without_early_release(fixed_runtime):
    r = fixed_runtime; b = r.backends['pose']; b.response_gate.clear()
    fresh_fixture_envelope(r)
    h = r.send(); wait(lambda: b.goals)
    r.envelope(allowed=False, reason='FIXTURE_REVOKED')
    time.sleep(.08)
    assert not b.cancels and not h.result_future.done()
    b.response_gate.set(); wait(lambda: b.cancels)
    fresh_fixture_envelope(r, allowed=True, reason='READY_FIXED')
    assert not h.result_future.done()
    assert not r.send('route', 'poses').accepted
    assert not r.backends['poses'].goals
    b.result_status = GoalStatus.STATUS_CANCELED; b.done.set()
    envelope_failed(r, h, 'ENVELOPE_REVOKED: FIXTURE_REVOKED')
    assert len(b.cancels) == 1


def test_fixed_v2_queued_goal_invalidated_without_backend_dispatch(fixed_runtime):
    r = fixed_runtime; old = r.backends['pose']; new = r.backends['poses']
    fresh_fixture_envelope(r)
    first = r.send('route'); wait(lambda: old.handles)
    queued = r.send('operator', 'poses'); assert queued.accepted
    wait(lambda: old.cancels)
    r.envelope(allowed=False, reason='FIXTURE_REVOKED')
    envelope_failed(r, queued, 'ENVELOPE_REVOKED: FIXTURE_REVOKED')
    assert not new.goals and not first.result_future.done()
    old.result_status = GoalStatus.STATUS_CANCELED; old.done.set()
    envelope_failed(r, first, 'ENVELOPE_REVOKED: FIXTURE_REVOKED')
    assert len(old.cancels) == 1


@pytest.mark.parametrize('clock_kind', ['steady', 'ros'])
def test_fixed_v2_expiry_cancels_without_backend_terminal(clock_kind, tmp_path):
    r = Runtime('cpp', tmp_path, sim=True, fixed_v2=True)
    try:
        wait(lambda: r.clock.get_subscription_count() == 1 and
             r.envelope_pub.get_subscription_count() == 1)
        for _ in range(10):
            r.sim_clock(100_000_000_000); time.sleep(.01)
        fresh_fixture_envelope(r)
        h = r.send(); b = r.backends['pose']; wait(lambda: b.handles)
        assert all(e.stamp.sec == 100 for e in r.events)
        r.envelope(publishing=False)
        if clock_kind == 'ros':
            r.sim_clock(100_310_000_000)
            reason = 'ENVELOPE_ROS_STALE'
        else:
            # Frozen ROS time keeps the 0.3 s ROS lease valid; steady age alone expires.
            reason = 'ENVELOPE_WALL_STALE'
        wait(lambda: b.cancels)
        assert not h.result_future.done()
        b.result_status = GoalStatus.STATUS_CANCELED; b.done.set()
        envelope_failed(r, h, reason)
        assert len(b.cancels) == 1
    finally:
        r.close()


def test_fixed_v2_ready_backend_success_cannot_win_over_expired_authority(tmp_path):
    r = Runtime('cpp', tmp_path, sim=True, fixed_v2=True)
    stopped = False
    try:
        wait(lambda: r.clock.get_subscription_count() == 1 and
             r.envelope_pub.get_subscription_count() == 1)
        for _ in range(10):
            r.sim_clock(100_000_000_000); time.sleep(.01)
        fresh_fixture_envelope(r)
        h = r.send(); b = r.backends['pose']; wait(lambda: b.handles)
        r.envelope(publishing=False)
        # Drain the last periodic publication while the arbiter still runs;
        # a late first receipt must not masquerade as an expired old receipt.
        r.set_timeout(1.0)
        time.sleep(.08)
        # Stop only this fixture-owned arbiter, not the fake backend/executor.
        # On resume both an already-ready SUCCEEDED result and an expired steady
        # envelope are pending, so either callback order must finish FAILED.
        r.proc.send_signal(signal.SIGSTOP); stopped = True
        time.sleep(.6)
        b.done.set(); wait(lambda: b.finished)
        assert not h.result_future.done()
        r.proc.send_signal(signal.SIGCONT); stopped = False
        envelope_failed(r, h, 'ENVELOPE_WALL_STALE')
    finally:
        if stopped: r.proc.send_signal(signal.SIGCONT)
        r.close()
