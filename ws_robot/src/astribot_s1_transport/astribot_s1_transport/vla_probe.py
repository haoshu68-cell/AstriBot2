"""Read-only live RGB-D / policy probe. No services, actions or robot command publishers."""
import argparse
import json
import math
from pathlib import Path
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException

from .core import Ledger, TaskFailure
from .vla_contract import VERSION, UNITS, digest, validate_config
from .vla_ros import VlaBridge, stamp_ns


class Probe(Node):
    def __init__(self, config, output):
        super().__init__('vla_readonly_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.c = dict(base_frame='astribot_torso_base', tcp='astribot_arm_left_tcp_link', observation_max_age_s=.5)
        self.ledger = Ledger(output, 'read_only_probe')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.joints = self.observation = None
        self.create_subscription(JointState, '/joint_states', lambda m:setattr(self, 'joints', m), qos_profile_sensor_data)
        self.bridge = VlaBridge(self, dict(config, mode='shadow'))

    def fresh(self, message):
        return message is not None and -10_000_000 <= self.get_clock().now().nanoseconds-stamp_ns(message) <= 500_000_000

    def check(self):
        if not rclpy.ok():
            raise TaskFailure('PROBE_SHUTDOWN')

    def run(self, duration):
        self.bridge.start()
        end = time.monotonic()+duration
        last_error = ''
        while time.monotonic() < end:
            try:
                observation = self.bridge.observation()
                break
            except (TaskFailure, TransformException) as error:
                last_error = str(error)
                time.sleep(.03)
        else:
            raise TaskFailure('READ_ONLY_OBSERVATION_TIMEOUT:' + last_error)
        pose = observation['tcp']
        request = dict(schema=VERSION, episode_id=self.bridge.session.episode_id, request_id=str(uuid.uuid4()),
            sequence=0, context_id=digest(dict(joints=observation['joints'], stamp_ns=observation['stamp_ns'])),
            operation='OBSERVE', instruction='Read-only sensor and adapter validation; no execution.',
            frame_id=self.c['base_frame'], units=UNITS, observation=observation,
            context=dict(execution_permitted=False), allowed_action_types=self.bridge.session.capabilities['action_types'],
            nominal_action=dict(type='mtc_targets', frame_id=self.c['base_frame'], group='arm_left',
                pre_target=pose, target=pose, exit_targets=[pose]))
        action = self.bridge.session.infer(request, self.check)
        return dict(evidence_level='live_observation_and_policy_only', execution_permitted=False,
            adapter=self.bridge.session.capabilities, action_type=action['type'],
            stamp_ns=observation['stamp_ns'], joint_count=len(observation['joints']['names']),
            cameras={k:dict(width=v['rgb']['width'], height=v['rgb']['height'], encoding=v['rgb']['encoding'],
                          frame=v['rgb']['frame_id'], depth='depth' in v, calibration_id=v['calibration_id'])
                     for k,v in observation['cameras'].items()})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=15.)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 60.:
        parser.error('--duration must be in (0,60] seconds')
    config = validate_config(json.loads(Path(args.config).read_text()))
    output = Path(args.output)
    if output.exists():
        raise SystemExit('Use a new evidence directory.')
    rclpy.init()
    node = Probe(config, output)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    ok = False
    try:
        result = node.run(args.duration)
        ok = True
    except Exception as error:
        result = dict(evidence_level='probe_failed', execution_permitted=False, reason=str(error))
    finally:
        node.bridge.session.close(dict(status='read_only_probe_finished'))
        executor.shutdown()
        thread.join(2.)
        node.destroy_node()
        rclpy.shutdown()
    (output/'summary.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
