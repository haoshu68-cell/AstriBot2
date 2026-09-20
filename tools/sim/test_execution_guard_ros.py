#!/usr/bin/env python3
"""Synthetic ROS protocol test for the C++ guard, without a simulator or actuator."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import TransformStamped,PoseStamped
from tf2_msgs.msg import TFMessage
from astribot_transport_msgs.srv import SetExecutionGuard
from astribot_transport_msgs.msg import ExecutionGuardStatus


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True,type=Path);args=p.parse_args()
    # Refuse synthetic state publication into any live simulation/controller.
    domain=os.environ.get('ROS_DOMAIN_ID')
    if domain in (None,'0','25'):raise RuntimeError('explicit isolated test domain required')
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            argv=proc.joinpath('cmdline').read_bytes().decode().split('\0')
            env=dict(v.split('=',1) for v in proc.joinpath('environ').read_bytes().decode().split('\0') if '=' in v)
        except (OSError,UnicodeError):continue
        if env.get('ROS_DOMAIN_ID')==domain and Path(argv[0]).name in ('robot_state_publisher','controller_server','execution_guard'):
            raise RuntimeError('test domain already has runtime owners')
    args.output.mkdir(exist_ok=False,parents=True)
    binary=get_package_prefix('astribot_s1_transport_mtc')+'/lib/astribot_s1_transport_mtc/execution_guard'
    with (args.output/'guard.log').open('w') as log:
        child=subprocess.Popen([binary,'--ros-args','-p','base_frame:=guard_test_base'],stdout=log,stderr=subprocess.STDOUT)
        rclpy.init();node=rclpy.create_node('guard_protocol_test');status=[];records=[]
        joint=node.create_publisher(JointTrajectoryControllerState,'/arm_left_controller/state',10)
        tf=node.create_publisher(TFMessage,'/tf',10)
        node.create_subscription(ExecutionGuardStatus,'/transport/execution_guard/status',lambda s:status.append(s),100)
        client=node.create_client(SetExecutionGuard,'/transport/execution_guard/set')
        error=0.;offset=0.;publish_joint=True
        def step():
            stamp=node.get_clock().now().to_msg()
            if publish_joint:
                m=JointTrajectoryControllerState();m.header.stamp=stamp;m.joint_names=['arm']
                m.desired.positions=[0.];m.actual.positions=[error];joint.publish(m)
            t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='guard_test_world';t.child_frame_id='guard_test_base'
            t.transform.translation.x=offset;t.transform.rotation.w=1.;tf.publish(TFMessage(transforms=[t]))
            rclpy.spin_once(node,timeout_sec=.01)
        def until(predicate,timeout=3.):
            end=time.monotonic()+timeout
            while not predicate():
                assert child.poll() is None,'C++ guard exited'
                if time.monotonic()>end:raise TimeoutError('guard evidence timed out')
                step()
        def set_guard(context,enable=True):
            pose=PoseStamped();pose.header.frame_id='guard_test_world';pose.pose.orientation.w=1.
            f=client.call_async(SetExecutionGuard.Request(enable=enable,context_id=context,joint_names=['arm'],base_reference=pose))
            until(f.done);return f.result()
        def state(context,healthy):
            return bool(status) and status[-1].context_id==context and status[-1].active and status[-1].healthy==healthy
        passed=False
        try:
            assert client.wait_for_service(timeout_sec=8.)
            until(lambda:len(status)>3)
            assert set_guard('one').accepted;until(lambda:state('one',True));records.append('fresh_evidence_ready')
            assert not set_guard('foreign').accepted;records.append('foreign_context_rejected')
            error=.051;until(lambda:state('one',False) and status[-1].reason.startswith('MANIPULATION_TRACKING_ERROR'))
            error=0.;start=time.monotonic()
            while time.monotonic()-start<.4:step()
            assert not status[-1].healthy;records.append('tracking_fault_latched_after_measurement_recovers')
            assert set_guard('one',False).accepted
            assert set_guard('two').accepted;until(lambda:state('two',True))
            offset=.020001;until(lambda:state('two',False) and status[-1].reason=='MTC_BASE_MOVED_DURING_EXECUTION')
            records.append('base_displacement_rejected');assert set_guard('two',False).accepted
            offset=0.;assert set_guard('three').accepted;until(lambda:state('three',True))
            publish_joint=False;until(lambda:state('three',False) and status[-1].reason=='MANIPULATION_TRACKING_STATE_STALE')
            records.append('source_timeout_rejected');assert set_guard('three',False).accepted
            passed=True
        finally:
            node.destroy_node();rclpy.shutdown()
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=5.)
            except subprocess.TimeoutExpired:child.kill();child.wait()
            (args.output/'result.json').write_text(json.dumps(dict(passed=passed,checks=records,
                evidence='synthetic ROS protocol only; no physical execution claim',child_returncode=child.returncode),indent=2)+'\n')
        print(json.dumps(dict(passed=passed,checks=records)))


if __name__=='__main__':main()
