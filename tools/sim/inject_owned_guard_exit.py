#!/usr/bin/env python3
"""Exit one explicitly owned simulation guard during measured MTC arm motion."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time

import rclpy
from rclpy.parameter import Parameter
from control_msgs.msg import JointTrajectoryControllerState
from astribot_transport_msgs.msg import ExecutionGuardStatus


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pid', type=int, required=True)
    p.add_argument('--instance', required=True)
    p.add_argument('--owner-log-dir', type=Path, required=True)
    p.add_argument('--ledger', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    proc = Path('/proc') / str(args.pid)
    start_ticks = proc.joinpath('stat').read_text().rsplit(')', 1)[1].split()[19]
    argv = proc.joinpath('cmdline').read_bytes().decode().split('\0')
    env = dict(x.split('=', 1) for x in proc.joinpath('environ').read_bytes().decode().split('\0') if '=' in x)
    assert Path(argv[0]).name == 'execution_guard'
    assert env.get('ASTRIBOT_SIM_INSTANCE') == args.instance == os.environ.get('ASTRIBOT_SIM_INSTANCE')
    assert env.get('ROS_DOMAIN_ID') == os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '25')
    assert Path(env['ASTRIBOT_LOG_DIR']).resolve() == args.owner_log_dir.resolve()
    assert not args.output.exists()
    # pidfd pins this process even if its numeric PID is later reused.
    fd = os.pidfd_open(args.pid)
    if proc.joinpath('stat').read_text().rsplit(')', 1)[1].split()[19] != start_ticks:
        os.close(fd)
        raise RuntimeError('guard PID identity changed')
    rclpy.init()
    node = rclpy.create_node('owned_guard_fault_injector', parameter_overrides=[Parameter('use_sim_time', value=True)])
    packets = {}
    node.create_subscription(ExecutionGuardStatus, '/transport/execution_guard/status',
                             lambda m: packets.update(guard=m), 10)
    node.create_subscription(JointTrajectoryControllerState, '/arm_left_controller/state',
                             lambda m: packets.update(joints=m), 10)
    result = dict(injected=False, guard_pid=args.pid, instance=args.instance)
    try:
        deadline = time.monotonic()+100.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            if not args.ledger.exists():
                continue
            state = json.loads(args.ledger.read_text())
            if state['stage'] in ('FAULT', 'CANCELED', 'SUCCEEDED'):
                raise RuntimeError('task ended before requested injection')
            guard, joints = packets.get('guard'), packets.get('joints')
            if guard is None or joints is None or state['stage'] != 'PREGRASP':
                continue
            now = node.get_clock().now().nanoseconds
            fresh = lambda s: -10_000_000 <= now-s.sec*10**9-s.nanosec <= 300_000_000
            velocity = joints.actual.velocities
            if (guard.active and guard.healthy and fresh(guard.stamp) and fresh(joints.header.stamp) and
                    velocity and all(math.isfinite(v) for v in velocity) and max(map(abs, velocity)) > .03):
                result.update(injected=True, wall_time=time.time(), ros_time_s=now*1e-9,
                              stage=state['stage'], context_id=guard.context_id,
                              maximum_actual_joint_velocity_rad_s=max(map(abs, velocity)))
                signal.pidfd_send_signal(fd, signal.SIGTERM)
                break
        if not result['injected']:
            raise TimeoutError('no fresh guarded MTC motion observed')
    finally:
        args.output.write_text(json.dumps(result, indent=2)+'\n')
        os.close(fd)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result))


if __name__ == '__main__':
    main()
