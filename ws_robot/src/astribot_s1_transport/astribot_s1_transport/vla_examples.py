"""Two inference-only SDK examples. Neither is a trained VLA model."""
import copy
from .vla_policy import ReferencePolicy


class OffsetTargetPolicy(ReferencePolicy):
    """Demonstrate normalized SI target output; safe to inspect in shadow mode."""
    def call(self, endpoint, packet):
        result = super().call(endpoint, packet)
        if endpoint == 'capabilities':
            result['policy_id'] = 'example_offset_target'
        elif endpoint == 'infer':
            poses = [result['action']['pre_target'], result['action']['target']] + result['action']['exit_targets']
            for pose in poses:
                pose['position'][0] += .005
        return result


class DeltaChunkPolicy(ReferencePolicy):
    """Two timesteps using explicit translation/rotation/gripper conventions."""
    def call(self, endpoint, packet):
        result = super().call(endpoint, packet)
        if endpoint == 'capabilities':
            result['policy_id'] = 'example_delta_chunk'
            result['action_types'] = ['ee_delta_chunk']
        elif endpoint == 'infer':
            sample = dict(time_s=.1, position_delta_m=[.001,0.,0.],
                          rotation_vector_rad=[0.,0.,0.], gripper_width_m=.06)
            result['action'] = dict(type='ee_delta_chunk', frame_id=packet['frame_id'], group='arm_left',
                                    samples=[sample, dict(copy.deepcopy(sample), time_s=.2)])
        return result
