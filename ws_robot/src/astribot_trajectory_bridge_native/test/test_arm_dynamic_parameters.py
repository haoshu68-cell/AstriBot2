"""Real ROS A/B regression for parameters formerly cached by the C++ adapter.

The independent expectations below fail if any of the seven callback-time
parameters is frozen at construction. Only owned child PIDs are terminated.
Run with ASTRIBOT_ARM_LIMITER_CPP pointing to a clean-build executable.
"""
from pathlib import Path as _ReferencePath
import sys as _reference_sys
_reference_sys.path.insert(0, str(_ReferencePath(__file__).resolve().parents[4] / "tools/migration/python_reference"))
from reference_bootstrap import REFERENCE_ROOT, enable as _enable_references
_enable_references()

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import pytest


def oracle_main():
    # This is a test-only launcher for the retired Python reference class.
    source = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(source / 'astribot_s1_navigation'))
    os.environ['ASTRIBOT_BRIDGE_NATIVE_KERNELS'] = '0'
    import rclpy
    from astribot_s1_navigation.arm_speed_limiter_node import ArmSpeedLimiterNode
    rclpy.init(args=sys.argv[2:])
    node = ArmSpeedLimiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class Pair:
    def __init__(self, metric, name, output):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from nav2_msgs.msg import SpeedLimit
        from rcl_interfaces.srv import GetParameters, SetParametersAtomically
        from sensor_msgs.msg import JointState
        from tf2_ros import TransformBroadcaster

        self.rclpy = rclpy
        self.TransformStamped = TransformStamped
        self.JointState = JointState
        self.parameter_service = SetParametersAtomically
        self.prefix = 'arm_parameter_' + uuid.uuid4().hex[:10]
        self.node = rclpy.create_node(self.prefix)
        self.pub = self.node.create_publisher(JointState, '/' + self.prefix + '/joints', 10)
        self.tf = TransformBroadcaster(self.node)
        self.values = {'python': [], 'cpp': []}
        self.clients = {}
        self.readers = {}
        self.get_parameters_service = GetParameters
        self.processes = []
        self.handles = []
        self.output = output
        self.output.mkdir(parents=True, exist_ok=True)
        self.name = name
        self.position = 0.8
        self.reach = 0.7
        self.tf_enabled = metric == 'horizontal_reach'
        self.joints_enabled = True
        self.tf_offset_ns = -20_000_000
        self.steps = []
        self.cleanup = []
        self.subscriptions = []
        cpp = os.environ.get('ASTRIBOT_ARM_LIMITER_CPP', '')
        assert cpp and Path(cpp).is_file(), 'Set ASTRIBOT_ARM_LIMITER_CPP to the C++ executable'
        for implementation in ('python', 'cpp'):
            ns = '/' + self.prefix + '/' + implementation
            topic = ns + '/speed_limit'
            self.subscriptions.append(self.node.create_subscription(
                SpeedLimit, topic,
                lambda message, key=implementation: self.values[key].append({
                    'wall': time.monotonic(), 'speed_limit': message.speed_limit,
                    'percentage': message.percentage,
                    'stamp_ns': message.header.stamp.sec * 10**9 + message.header.stamp.nanosec}), 100))
            args = ['--ros-args', '-r', '__ns:=' + ns,
                    '-p', 'extension_metric:=' + metric,
                    '-p', 'joint_states_topic:=/' + self.prefix + '/joints',
                    '-p', 'speed_limit_topic:=' + topic,
                    '-p', 'check_period:=0.1', '-p', 'reach_tf_timeout_sec:=0.2',
                    '-p', 'chassis_base_frame:=' + self.frame('base'),
                    '-p', 'monitored_links:=[' + self.frame('reach') + ']']
            command = ([sys.executable, str(Path(__file__).resolve()), '--oracle']
                       if implementation == 'python' else [cpp]) + args
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = os.environ.get('ASTRIBOT_ARM_PARAMETER_DOMAIN', '118')
            env['ROS_LOCALHOST_ONLY'] = '1'
            env['ROS_LOG_DIR'] = str(output / 'ros_logs')
            env['ASTRIBOT_BRIDGE_NATIVE_KERNELS'] = '0'
            handle = (output / (implementation + '.log')).open('w')
            process = subprocess.Popen(command, env=env, stdout=handle, stderr=subprocess.STDOUT)
            self.processes.append((implementation, process, command))
            self.handles.append(handle)
            self.clients[implementation] = self.node.create_client(
                SetParametersAtomically, ns + '/arm_speed_limiter_node/set_parameters_atomically')
            self.readers[implementation] = self.node.create_client(
                GetParameters, ns + '/arm_speed_limiter_node/get_parameters')
        try:
            self.until(lambda: all(c.service_is_ready() for c in self.clients.values()), 8.0)
            self.pump(0.35)
            for implementation, process, _ in self.processes:
                environ = Path('/proc', str(process.pid), 'environ').read_bytes().split(b'\0')
                assert ('ROS_DOMAIN_ID=' + os.environ.get('ASTRIBOT_ARM_PARAMETER_DOMAIN', '118')).encode() in environ
                assert b'ROS_LOCALHOST_ONLY=1' in environ
                if implementation == 'cpp':
                    maps = Path('/proc', str(process.pid), 'maps').read_text()
                    assert 'libpython' not in maps and '_chassis_math_native' not in maps
        except BaseException:
            self.close()
            raise

    def frame(self, value):
        return self.prefix + '_' + value

    def input(self):
        if self.tf_enabled:
            transforms = []
            stamp = self.node.get_clock().now().nanoseconds + self.tf_offset_ns
            for parent, child, x in (('world', 'base', 0.0), ('world', 'offset_base', 0.5),
                                     ('base', 'reach', self.reach), ('base', 'short', 0.2)):
                message = self.TransformStamped()
                message.header.frame_id = self.frame(parent)
                message.child_frame_id = self.frame(child)
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
                message.transform.translation.x = x
                message.transform.rotation.w = 1.0
                transforms.append(message)
            self.tf.sendTransform(transforms)
        if self.joints_enabled:
            message = self.JointState()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.name = ['astribot_arm_left_joint_1']
            message.position = [self.position]
            self.pub.publish(message)

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            for name, process, _ in self.processes:
                assert process.poll() is None, '{} exited: see {}'.format(name, self.output)
            self.input()
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.005)

    def until(self, predicate, timeout):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            self.pump(0.03)
        assert predicate(), 'ROS operation timed out: ' + str(self.output)

    def set(self, **values):
        from rclpy.parameter import Parameter
        parameters = [Parameter(name, value=value).to_parameter_msg() for name, value in values.items()]
        requests = {key: client.call_async(self.parameter_service.Request(parameters=parameters))
                    for key, client in self.clients.items()}
        self.until(lambda: all(future.done() for future in requests.values()), 3.0)
        accepted = {key: future.result().result.successful for key, future in requests.items()}
        self.steps.append({'parameters': values, 'accepted': accepted})
        assert all(accepted.values()), 'Both real parameter services must accept the update'

    def expect(self, expected, label):
        # Wait for multiple *fresh* output samples after the transition; checking
        # only final values could accidentally read a sample preceding an update.
        starts = {key: len(value) for key, value in self.values.items()}
        self.pump(0.4)
        observed = {key: values[starts[key]:] for key, values in self.values.items()}
        self.steps.append({'case': label, 'expected': expected, 'observed': observed})
        for key, values in observed.items():
            assert len(values) >= 2, '{} did not republish current state'.format(key)
            assert all(v['percentage'] for v in values)
            actual = [v['speed_limit'] for v in values[-2:]]
            assert actual == [expected, expected], '{} {}: expected {}, observed {}'.format(
                key, label, expected, actual)

    def reject_cpp_update(self, **values):
        from rclpy.parameter import Parameter
        names = list(values)
        before_future = self.readers['cpp'].call_async(self.get_parameters_service.Request(names=names))
        self.until(before_future.done, 3.0)
        before = before_future.result().values
        parameters = [Parameter(name, value=value).to_parameter_msg() for name, value in values.items()]
        future = self.clients['cpp'].call_async(self.parameter_service.Request(parameters=parameters))
        self.until(future.done, 3.0)
        result = future.result().result
        # Nonfinite values are represented as strings in the evidence JSON.
        self.steps.append({'invalid_parameters_repr': {k: repr(v) for k, v in values.items()},
                           'cpp_accepted': result.successful, 'reason': result.reason})
        assert not result.successful, 'C++ accepted unsafe parameter batch: ' + repr(values)
        assert result.reason
        after_future = self.readers['cpp'].call_async(self.get_parameters_service.Request(names=names))
        self.until(after_future.done, 3.0)
        assert after_future.result().values == before, 'Rejected atomic batch partially changed stored parameters'
        self.expect(50.0, 'invalid atomic update preserves existing extended limit')

    def close(self):
        for name, process, command in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2.0)
            self.cleanup.append({'implementation': name, 'pid': process.pid,
                                 'command': command, 'exit_code': process.returncode,
                                 'owned_pid_reaped': process.poll() is not None})
        for handle in self.handles:
            handle.close()
        (self.output / 'result.json').write_text(json.dumps({
            'test': self.name, 'domain': os.environ.get('ASTRIBOT_ARM_PARAMETER_DOMAIN', '118'),
            'localhost_only': True, 'steps': self.steps, 'outputs': self.values,
            'cleanup': self.cleanup}, ensure_ascii=False, indent=2) + '\n')
        self.node.destroy_node()


@pytest.fixture
def pair(request, tmp_path):
    import rclpy
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('ASTRIBOT_ARM_PARAMETER_DOMAIN', '118')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init()
    metric = 'joint_deviation' if request.node.name.startswith('test_joint') else 'horizontal_reach'
    output = Path(os.environ.get('ASTRIBOT_ARM_PARAMETER_EVIDENCE', str(tmp_path))) / request.node.name
    owned = None
    try:
        owned = Pair(metric, request.node.name, output)
        yield owned
    finally:
        if owned is not None:
            owned.close()
        if rclpy.ok():
            rclpy.shutdown()


def test_joint_speed_percentage_changes_on_timer_without_new_joint_callback(pair):
    pair.expect(50.0, 'initial extended')
    pair.joints_enabled = False
    pair.set(extended_speed_limit_pct=27.0)
    pair.expect(27.0, 'same classification, timer must read updated percentage')


def test_joint_reference_update_changes_classification(pair):
    pair.expect(50.0, 'initial extended')
    pair.set(folded_reference_rad=[0.8] + [0.0] * 13)
    pair.expect(0.0, 'new folded reference')


def test_joint_threshold_update_changes_classification(pair):
    pair.expect(50.0, 'initial extended')
    pair.set(extended_threshold_rad=1.0)
    pair.expect(0.0, 'new joint threshold')


def test_reach_frame_update_changes_tf_origin(pair):
    pair.expect(50.0, 'initial extended at 0.7 m')
    pair.set(chassis_base_frame=pair.frame('offset_base'))
    pair.expect(0.0, 'offset chassis frame yields 0.2 m')


def test_reach_monitored_links_update_changes_tf_selection(pair):
    pair.expect(50.0, 'initial long link')
    pair.set(monitored_links=[pair.frame('short')])
    pair.expect(0.0, 'selected short link is 0.2 m')


def test_reach_threshold_update_changes_classification(pair):
    pair.expect(50.0, 'initial 0.7 m exceeds 0.64 m')
    pair.set(extended_reach_m=0.9)
    pair.expect(0.0, '0.7 m is below new release threshold')


def test_reach_hysteresis_update_preserves_previous_classification(pair):
    pair.expect(50.0, 'initial extended')
    pair.reach = 0.62
    pair.expect(50.0, '0.62 m remains inside original extended hysteresis band')
    pair.set(extended_reach_hysteresis_m=0.01)
    pair.expect(0.0, '0.62 m falls below updated 0.63 m release threshold')
    pair.set(extended_reach_hysteresis_m=0.03)
    pair.expect(0.0, 'changing hysteresis retains folded last state')


def test_reach_stale_tf_keeps_fail_closed_behavior(pair):
    pair.reach = 0.2
    pair.expect(0.0, 'fresh short reach')
    pair.tf_enabled = False
    pair.pump(0.25)
    pair.expect(50.0, 'expired transform restricts speed')


def test_reach_missing_updated_link_fails_closed(pair):
    pair.reach = 0.2
    pair.expect(0.0, 'fresh short reach')
    pair.set(monitored_links=[pair.frame('missing')])
    pair.expect(50.0, 'missing newly selected TF must restrict speed')


def test_joint_metric_and_timer_stay_construction_parameters(pair):
    pair.position = 0.0
    pair.expect(0.0, 'folded joint state')
    pair.set(extension_metric='horizontal_reach', check_period=0.001)
    pair.expect(0.0, 'startup metric remains joint_deviation')
    pair.joints_enabled = False
    starts = {key: len(values) for key, values in pair.values.items()}
    pair.pump(0.3)
    counts = {key: len(values) - starts[key] for key, values in pair.values.items()}
    assert all(1 <= count <= 6 for count in counts.values()), counts


@pytest.mark.parametrize('parameter,invalid_values', (
    ('extended_reach_m', [float('nan'), float('inf'), -0.1, 0.0]),
    ('extended_reach_hysteresis_m', [float('nan'), float('inf'), -0.1, 0.64, 0.8]),
    ('extended_threshold_rad', [float('nan'), float('inf'), -0.1]),
    ('folded_reference_rad', [[float('nan')], [float('inf')], [float('-inf')]]),
    ('extended_speed_limit_pct', [float('nan'), float('inf'), -1.0, 0.0, 101.0]),
    ('reach_tf_timeout_sec', [float('nan'), float('inf'), 0.0, -0.1]),
), ids=('reach', 'hysteresis', 'joint_threshold', 'joint_reference', 'speed_percentage', 'tf_timeout'))
def test_reach_invalid_atomic_update_preserves_extended_limit(pair, parameter, invalid_values):
    pair.expect(50.0, 'initial extended before invalid update')
    for value in invalid_values:
        changes = {'extended_speed_limit_pct': 27.0, parameter: value}
        pair.reject_cpp_update(**changes)


def test_reach_valid_atomic_threshold_pair_uses_merged_candidate(pair):
    pair.expect(50.0, 'initial extended')
    # 0.02 would be incompatible with the *old* 0.03 hysteresis. The candidate
    # batch must be validated jointly, then committed by ROS once.
    pair.set(extended_reach_m=0.02, extended_reach_hysteresis_m=0.01)
    pair.expect(50.0, 'valid merged reach/hysteresis pair')


def test_joint_valid_short_reference_and_zero_threshold_keep_zip_semantics(pair):
    pair.expect(50.0, 'initial extended')
    pair.set(folded_reference_rad=[0.8], extended_threshold_rad=0.0)
    pair.expect(0.0, 'short reference vector and zero joint threshold remain valid')
    pair.set(folded_reference_rad=[0.0], extended_speed_limit_pct=100.0)
    pair.expect(100.0, 'upper valid percentage')
    pair.set(extended_speed_limit_pct=0.5)
    pair.expect(0.5, 'positive percentage below one remains a limit')


@pytest.mark.parametrize('implementation', ('python', 'cpp'))
@pytest.mark.parametrize('parameter,value', (
    ('reach_tf_timeout_sec', '0.0'), ('check_period', '-0.1')))
def test_invalid_startup_parameter_reports_failure(implementation, parameter, value, tmp_path):
    cpp = os.environ.get('ASTRIBOT_ARM_LIMITER_CPP', '')
    assert cpp and Path(cpp).is_file(), 'Set ASTRIBOT_ARM_LIMITER_CPP'
    name = 'invalid_startup_' + parameter + '_' + implementation
    output = Path(os.environ.get('ASTRIBOT_ARM_PARAMETER_EVIDENCE', str(tmp_path))) / name
    output.mkdir(parents=True, exist_ok=True)
    command = ([sys.executable, str(Path(__file__).resolve()), '--oracle']
               if implementation == 'python' else [cpp])
    command += ['--ros-args', '-r', '__ns:=/arm_invalid_' + uuid.uuid4().hex[:10],
                '-p', parameter + ':=' + value]
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=os.environ.get('ASTRIBOT_ARM_PARAMETER_DOMAIN', '118'),
               ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(output / 'ros_logs'),
               ASTRIBOT_BRIDGE_NATIVE_KERNELS='0')
    child = subprocess.Popen(command, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        # /proc/environ can be briefly empty while exec replaces the process
        # image. Sample the owned PID until its new environment is visible.
        deadline = time.monotonic() + 1.0
        environ = []
        while not any(environ) and child.poll() is None and time.monotonic() < deadline:
            environ = Path('/proc', str(child.pid), 'environ').read_bytes().split(b'\0')
            if not any(environ):
                time.sleep(0.002)
        assert ('ROS_DOMAIN_ID=' + env['ROS_DOMAIN_ID']).encode() in environ
        assert b'ROS_LOCALHOST_ONLY=1' in environ
        log, _ = child.communicate(timeout=8.0)
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=2.0)
    (output / 'node.log').write_text(log)
    (output / 'result.json').write_text(json.dumps({
        'command': command, 'pid': child.pid, 'returncode': child.returncode,
        'domain': env['ROS_DOMAIN_ID'], 'localhost_only': True,
        'owned_pid_reaped': child.poll() is not None}, indent=2) + '\n')
    assert 'must be' in log, 'Failure must come from parameter validation, not a missing dependency'
    assert child.returncode != 0, '{} incorrectly reported successful startup for {}'.format(
        implementation, parameter)


if __name__ == '__main__' and len(sys.argv) > 1 and sys.argv[1] == '--oracle':
    oracle_main()
