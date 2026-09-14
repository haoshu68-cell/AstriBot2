"""Explicitly scoped simulation parameters; hardware evidence is a separate input."""
import json
from pathlib import Path
from types import MappingProxyType

from .contracts import finite, require


class Profile:
    def __init__(self, values):
        data = json.loads(json.dumps(values, allow_nan=False))
        data.setdefault('scan_occupancy_resolution_m',.05)
        data.setdefault('scan_obstacle_padding_m',.04)
        finite(data['scan_obstacle_padding_m'],'scan_obstacle_padding_m',0)
        data.setdefault('narrow_angular_speed_rad_s',min(.2,data.get('max_angular_speed_rad_s',.2)))
        data.setdefault('narrow_centering_speed_m_s',.05)
        data.setdefault('narrow_centering_tolerance_m',.01)
        data.setdefault('narrow_centering_max_offset_m',.3)
        require(data.get('schema_version') == 1, 'profile.schema_version')
        require(data.get('environment') in ('simulation', 'hardware'), 'profile.environment')
        require(type(data.get('hardware_validated')) is bool, 'profile.hardware_validated')
        for name in ('half_length_m', 'half_width_m', 'height_m', 'clearance_margin_m',
                     'scan_occupancy_resolution_m',
                     'max_speed_m_s', 'max_acceleration_m_s2', 'brake_deceleration_m_s2',
                     'reaction_time_s', 'sensor_timeout_s', 'track_memory_s',
                     'prediction_horizon_s', 'prediction_step_s', 'association_distance_m',
                     'max_obstacle_speed_m_s', 'clear_hold_s', 'blocked_confirm_s',
                     'wait_budget_s', 'narrow_speed_m_s', 'narrow_heading_limit_rad', 'narrow_angular_speed_rad_s',
                     'command_timeout_s','scan_min_valid_fraction','velocity_confirmation_s',
                     'velocity_fit_window_s','velocity_fit_max_residual_m','min_tracked_speed_m_s',
                     'stationary_velocity_variance_m2_s2', 'max_angular_speed_rad_s',
                     'max_angular_acceleration_rad_s2', 'angular_brake_deceleration_rad_s2',
                     'constraint_lease_s', 'input_command_timeout_s','path_risk_timeout_s',
                     'local_rejoin_distance_m','local_max_deviation_m',
                     'narrow_centering_speed_m_s','narrow_centering_tolerance_m','narrow_centering_max_offset_m'):
            finite(data.get(name), name, 0)
            require(data[name] > 0, name)
        for name in ('payload_mass_kg', 'payload_extra_margin_m'):
            finite(data.get(name), name, 0)
        require(data['constraint_lease_s'] <= .5, 'constraint_wire_lease_limit')
        require(data['path_risk_timeout_s'] <= data['reaction_time_s'], 'path_evidence_age_budget')
        require(data['reaction_time_s'] >= data['input_command_timeout_s'], 'input_stop_budget')
        require(data['narrow_speed_m_s'] <= data['max_speed_m_s'], 'narrow_speed')
        require(data['narrow_angular_speed_rad_s'] <= data['max_angular_speed_rad_s'], 'narrow_angular_speed')
        require(data['narrow_centering_speed_m_s'] <= min(.05,data['narrow_speed_m_s']), 'narrow_centering_speed')
        require(data['narrow_centering_tolerance_m'] < data['narrow_centering_max_offset_m'] <= .3, 'narrow_centering_bounds')
        require(data['prediction_step_s'] <= data['prediction_horizon_s'], 'prediction_step')
        require(data['blocked_confirm_s'] < data['wait_budget_s'], 'wait_budget')
        require(data['reaction_time_s'] >= data['command_timeout_s'], 'watchdog_stop_budget')
        require(data['scan_min_valid_fraction'] <= 1, 'scan_min_valid_fraction')
        require(data['velocity_confirmation_s'] <= data['velocity_fit_window_s'], 'velocity_fit_window')
        for name in ('position_tolerance_m','linear_speed_m_s','angular_speed_rad_s',
                     'terminal_exclusion_m','context_timeout_s'):
            value=data.get('planning_takeover',{}).get(name)
            finite(value,'planning_takeover.'+name,0)
            require(value>0,'planning_takeover.'+name)
        self._values = MappingProxyType(data)

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text()))

    def __getattr__(self, name):
        try:
            return self._values[name]
        except KeyError:
            raise AttributeError(name) from None

    def require_environment(self, use_sim_time):
        require(type(use_sim_time) is bool, 'use_sim_time')
        if self.environment == 'simulation':
            require(use_sim_time, 'simulation_profile_requires_sim_time')
        else:
            keys=('transport_envelope','payload','braking','latency','sensor_coverage')
            require(self.hardware_validated and all(self.hardware_evidence.get(k) for k in keys),
                    'hardware_evidence_required')

    def stopping_distance(self, speed):
        finite(speed, 'speed', 0)
        return (speed*self.reaction_time_s + speed*speed/(2*self.brake_deceleration_m_s2)
                + self.clearance_margin_m)
