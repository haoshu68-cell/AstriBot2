"""Completion may publish without another sampling tick, never submit another job."""
import os
import hashlib
from pathlib import Path
import signal
import subprocess
import time


def test_ready_geometry_publishes_without_another_sampling_tick(tmp_path):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.srv import GetParameters
    from moveit_msgs.srv import GetPlanningScene
    from moveit_msgs.msg import PlanningScene
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from astribot_navigation_msgs.msg import RobotGeometryState
    from test_cpp_robot_model import URDF
    profile=Path(__file__).parents[2]/'astribot_s1_mapping/config/height_slices.yaml'
    expected_profile=hashlib.sha256(profile.read_bytes()).hexdigest()

    assert os.environ.get('ROS_DOMAIN_ID') == '115'
    executable = Path(os.environ['GEOMETRY_STATE_CPP'])
    rclpy.init()
    node = Node('geometry_completion_schedule_fixture')
    states = []
    revision = ''
    coverage = dict(input_cloud_topic='/map_scan', base_frame='base', enable_outlier_filter=False)
    for index, name in enumerate(('low_obstacle', 'main_nav', 'torso_high', 'overhead')):
        for field, value in dict(enabled=True, z_min=-.03 if index == 0 else index * .6,
                                 z_max=(index + 1) * .6, min_points=1).items():
            coverage[f'slices.{name}.{field}'] = value

    def parameters(request, response):
        response.values = [Parameter(name, value=URDF if name == 'robot_description' else coverage[name])
                           .to_parameter_msg().value for name in request.names]
        return response

    node.create_service(GetParameters, '/robot_state_publisher/get_parameters', parameters)
    node.create_service(GetParameters, '/pointcloud_slice_scan_node/get_parameters', parameters)
    node.create_service(GetPlanningScene, '/get_planning_scene', lambda request, response: response)
    ack = node.create_publisher(String, '/navigation/attachment_filter_applied', 10)

    def attachment(message):
        nonlocal revision
        revision = message.name
        ack.publish(String(data=revision))

    node.create_subscription(PlanningScene, '/navigation/attached_geometry', attachment, 10)
    node.create_subscription(RobotGeometryState, '/navigation/geometry_state', states.append, 10)
    joints = node.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
    clocks = node.create_publisher(Clock, '/clock', 10)
    log = (tmp_path / 'session.log').open('w')
    process = subprocess.Popen([str(executable), '--ros-args', '-p', 'base_frame:=base', '-p', 'attachment_source_mode:=planning_scene_legacy',
                                '-p', 'use_sim_time:=true', '-p', f'height_profile_path:={profile}'], stdout=log, stderr=subprocess.STDOUT)
    try:
        def spin(duration):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                assert process.poll() is None, (tmp_path / 'session.log').read_text()
                rclpy.spin_once(node, timeout_sec=min(.005, max(0., deadline - time.monotonic())))

        def source(stamp_ns):
            message = JointState()
            message.header.stamp.sec = stamp_ns // 10**9
            message.header.stamp.nanosec = stamp_ns % 10**9
            message.name = ['arm']
            message.position = [.2]
            joints.publish(message)
            return message.header.stamp

        now = 2_000_000_000
        for _ in range(110):
            now += 20_000_000
            stamp = source(now)
            spin(.005)
            clocks.publish(Clock(clock=stamp))
            if revision:
                ack.publish(String(data=revision))
            spin(.015)
        assert any(message.complete for message in states), [s.reason for s in states[-5:]]
        for message in states:
            assert message.height_profile_revision==expected_profile and message.ground_in_base_m==-.095
        # At the observed slow simulation rate, every 20 ms ROS tick still gets
        # a sampling opportunity. An additional 100 ms wall throttle would lose
        # every other tick and leave short-lived geometry without a replacement.
        states.clear()
        expected_stamps = set()
        for _ in range(20):
            now += 20_000_000
            stamp = source(now)
            expected_stamps.add(now)
            spin(.005)
            ack.publish(String(data=revision))
            clocks.publish(Clock(clock=stamp))
            spin(.0575)
        sampled = {m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
                   for m in states if m.complete}
        assert len(sampled & expected_stamps) >= 16, ('Sampling opportunities lost', len(sampled & expected_stamps))
        # Do not give the producer another ROS tick after the final sample.
        spin(.15)
        now += 20_000_000
        stamp = source(now)
        spin(.02)
        ack.publish(String(data=revision))
        states.clear()
        clocks.publish(Clock(clock=stamp))
        spin(.25)
        completed = [message for message in states if message.complete and message.header.stamp == stamp]
        assert len(completed) == 1, ('A ready result must publish once without another ROS sampling tick',
                                     [(s.complete, s.reason, s.header.stamp) for s in states])
        message = completed[0]
        assert message.published_at == stamp
        assert message.joint_source_stamps == [stamp]
        deadline = message.valid_until.sec * 10**9 + message.valid_until.nanosec
        assert now < deadline <= now + 300_000_000
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
