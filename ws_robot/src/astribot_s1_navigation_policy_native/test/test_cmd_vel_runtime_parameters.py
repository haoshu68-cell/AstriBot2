"""Exercise committed parameter updates through isolated ROS services and Twist output."""
from pathlib import Path as _ReferencePath
import sys as _reference_sys
_reference_sys.path.insert(0, str(_ReferencePath(__file__).resolve().parents[4] / "tools/migration/python_reference"))
from reference_bootstrap import REFERENCE_ROOT, enable as _enable_references
_enable_references()

import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParametersAtomically

ROOT = Path(__file__).resolve().parents[2]
REFERENCE = '''from reference_bootstrap import enable
enable()

import rclpy
from astribot_s1_navigation.cmd_vel_body_to_world_node import CmdVelBodyToWorldNode
rclpy.init()
node = CmdVelBodyToWorldNode()
try: rclpy.spin(node)
except KeyboardInterrupt: pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
'''


class Runtime:
    def __init__(self, impl, binary, directory, parameters):
        self.impl = impl
        self.name = 'cmd_parameter_' + uuid.uuid4().hex[:12]
        self.context = self.node = self.executor = self.proc = None
        self.events = []
        self.log = (directory / (self.name + '.log')).open('w')
        env = dict(os.environ, ROS_DOMAIN_ID='124', ROS_LOCALHOST_ONLY='1',
                   ASTRIBOT_NAV_NATIVE_KERNELS='0')
        env['PYTHONPATH'] = os.pathsep.join([str(REFERENCE_ROOT),
            str(ROOT / 'astribot_s1_navigation'),
            str(ROOT / 'astribot_s1_navigation_policy'), env.get('PYTHONPATH', '')])
        command = [binary] if impl == 'cpp' else [sys.executable, '-c', REFERENCE]
        params = dict(input_topic=f'/{self.name}/in', output_topic=f'/{self.name}/out',
                      odom_topic=f'/{self.name}/odom', enable_posture_monitor=False,
                      cmd_timeout_sec=1.0)
        params.update(parameters)
        command += ['--ros-args', '-r', '__node:=' + self.name]
        for name, value in params.items():
            command += ['-p', f'{name}:={str(value).lower() if isinstance(value, bool) else value}']
        try:
            self.proc = subprocess.Popen(command, env=env, stdout=self.log, stderr=subprocess.STDOUT)
            os.environ['ROS_LOCALHOST_ONLY'] = '1'
            self.context = Context()
            rclpy.init(context=self.context, domain_id=124)
            self.node = Node(self.name + '_probe', context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.output = self.node.create_subscription(Twist, params['output_topic'],
                lambda msg: self.events.append((msg.linear.x, msg.linear.y, msg.angular.z)), 10)
            self.command = self.node.create_publisher(Twist, params['input_topic'], 10)
            self.odometry = self.node.create_publisher(Odometry, params['odom_topic'], qos_profile_sensor_data)
            self.parameters = self.node.create_client(SetParametersAtomically,
                f'/{self.name}/set_parameters_atomically')
            self.until(lambda: self.parameters.service_is_ready() and
                       self.command.get_subscription_count() == 1 and
                       self.odometry.get_subscription_count() == 1)
            # Read after the child has initialized: /proc/environ can be empty
            # transiently while exec installs the new process image.
            actual_env = (Path('/proc') / str(self.proc.pid) / 'environ').read_bytes().split(b'\0')
            assert b'ROS_DOMAIN_ID=124' in actual_env and b'ROS_LOCALHOST_ONLY=1' in actual_env
            self.drive(.15)
            assert self.events, 'No wire output during discovery probe'
        except BaseException:
            self.close()
            raise

    def until(self, condition, timeout=5.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            assert self.proc.poll() is None, 'runtime exited'
            self.executor.spin_once(timeout_sec=.005)
            if condition():
                return
        raise AssertionError('ROS parameter replay timed out')

    def set(self, **values):
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name, value=value).to_parameter_msg() for name, value in values.items()])
        future = self.parameters.call_async(request)
        self.until(future.done)
        return future.result().result

    def drive(self, duration=.12, *, yaw=0., roll=0., z=.134, age=0., odom=True, cmd=True):
        self.events.clear()
        end = time.monotonic() + duration
        next_sample = 0.
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_sample:
                if odom:
                    message = Odometry()
                    stamp_ns = self.node.get_clock().now().nanoseconds - int(age * 1e9)
                    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 10**9)
                    message.pose.pose.position.z = z
                    q = message.pose.pose.orientation
                    q.x = math.sin(roll / 2.) * math.cos(yaw / 2.)
                    q.y = math.sin(roll / 2.) * math.sin(yaw / 2.)
                    q.z = math.cos(roll / 2.) * math.sin(yaw / 2.)
                    q.w = math.cos(roll / 2.) * math.cos(yaw / 2.)
                    self.odometry.publish(message)
                if cmd:
                    message = Twist()
                    message.linear.x = .2
                    message.angular.z = .1
                    self.command.publish(message)
                next_sample = now + .01
            self.executor.spin_once(timeout_sec=.002)
        return self.events

    def expect(self, expected):
        assert self.events, 'No Twist output'
        assert self.events[-1] == pytest.approx(expected, abs=1e-8)

    def close(self):
        if self.executor is not None: self.executor.shutdown()
        if self.node is not None: self.node.destroy_node()
        if self.context is not None and self.context.ok(): self.context.shutdown()
        if self.proc is not None:
            if self.proc.poll() is None: self.proc.send_signal(signal.SIGINT)
            try: self.proc.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=3.)
        self.log.close()


@pytest.fixture(params=['python', 'cpp'])
def runtime(request, tmp_path):
    binary = os.environ.get('CMD_VEL_CPP_BINARY')
    if not binary: pytest.skip('CMD_VEL_CPP_BINARY is required')
    active = []
    def start(**parameters):
        item = Runtime(request.param, binary, tmp_path, parameters)
        active.append(item)
        return item
    start.impl = request.param
    try: yield start
    finally:
        for item in active: item.close()


def test_runtime_rotation_toggle_changes_wire_output(runtime):
    node = runtime()
    node.drive(yaw=math.pi / 2.)
    node.expect((.2, 0., .1))
    assert node.set(enable_body_to_world=True).successful
    node.drive(yaw=math.pi / 2.)
    node.expect((0., .2, .1))
    assert node.set(enable_body_to_world=False).successful
    node.drive(yaw=math.pi / 2.)
    node.expect((.2, 0., .1))


def test_runtime_odom_timeout_rejects_old_stamp_then_recovers(runtime):
    node = runtime(enable_body_to_world=True)
    node.drive(age=.25)
    node.expect((.2, 0., .1))
    assert node.set(odom_timeout_sec=.1).successful
    node.drive(age=.25)
    node.expect((0., 0., 0.))
    assert node.set(odom_timeout_sec=.5).successful
    node.drive(age=.25)
    node.expect((.2, 0., .1))


def test_runtime_command_timeout_stops_an_existing_command(runtime):
    node = runtime()
    node.drive(.25, odom=False, cmd=False)
    assert not any(event == (0., 0., 0.) for event in node.events)
    assert node.set(cmd_timeout_sec=.05).successful
    node.drive(.12, odom=False, cmd=False)
    node.expect((0., 0., 0.))


@pytest.mark.parametrize('name,value,restore,pose', [
    ('normal_height', .3, .134, {}),
    ('max_height_deviation', .005, .06, {'z': .154}),
    ('max_tilt_rad', .005, .12, {'roll': .02}),
])
def test_runtime_threshold_uses_existing_samples_and_keeps_stop_latched(runtime, name, value, restore, pose):
    node = runtime(enable_posture_monitor=True)
    node.drive(.35, **pose)
    node.expect((.2, 0., .1))
    assert node.set(**{name: value}).successful
    node.drive(.1, **pose)
    node.expect((0., 0., 0.))
    assert node.set(**{name: restore}).successful
    node.drive(.1, **pose)
    node.expect((0., 0., 0.))


@pytest.mark.parametrize('name,value', [
    ('odom_timeout_sec', 0.), ('cmd_timeout_sec', -1.),
    ('max_height_deviation', -0.01), ('max_tilt_rad', -0.01),
    ('normal_height', math.nan), ('max_tilt_rad', math.inf),
    ('enable_body_to_world', 'true'), ('odom_timeout_sec', '0.1'),
])
def test_invalid_atomic_update_rejects_without_applying_valid_member(runtime, name, value):
    if runtime.impl == 'python': pytest.skip('C++ validation rejects unsafe legacy updates explicitly')
    node = runtime()
    result = node.set(**({'enable_body_to_world': True, name: value}
                        if name != 'enable_body_to_world' else {name: value}))
    assert not result.successful
    node.drive(yaw=math.pi / 2.)
    node.expect((.2, 0., .1))


@pytest.mark.parametrize('name,value', [
    ('input_topic', '/unused/in'), ('output_topic', '/unused/out'),
    ('odom_topic', '/unused/odom'), ('enable_posture_monitor', True),
])
def test_startup_only_parameters_reject_instead_of_claiming_an_update(runtime, name, value):
    if runtime.impl == 'python': pytest.skip('C++ explicitly advertises existing startup-only contract')
    node = runtime()
    assert not node.set(**{name: value}).successful
    node.drive()
    node.expect((.2, 0., .1))


@pytest.mark.parametrize('name,value', [
    ('odom_timeout_sec', '0.0'), ('cmd_timeout_sec', '-1.0'),
    ('odom_timeout_sec', '.nan'), ('cmd_timeout_sec', '.inf'),
])
def test_invalid_startup_reports_failure(name, value):
    binary = os.environ.get('CMD_VEL_CPP_BINARY')
    if not binary: pytest.skip('CMD_VEL_CPP_BINARY is required')
    result = subprocess.run([binary, '--ros-args', '-r', '__node:=invalid_cmd_parameters',
        '-p', f'{name}:={value}'], env=dict(os.environ, ROS_DOMAIN_ID='124', ROS_LOCALHOST_ONLY='1'),
        text=True, capture_output=True, timeout=5.)
    assert result.returncode != 0, result.stdout + result.stderr
    assert 'body-to-world settings' in result.stderr


def test_normal_shutdown_reports_success(runtime):
    node = runtime()
    node.proc.send_signal(signal.SIGINT)
    assert node.proc.wait(timeout=3.) == 0
