#!/usr/bin/env python3
"""Replay historical stopping bounds without ROS or any robot commands."""
import argparse
import hashlib
import json
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / 'ws_robot/src/astribot_s1_navigation_policy'))
    from astribot_s1_navigation_policy.profile import Profile
    from astribot_s1_navigation_policy.motion_geometry import stopping_horizon
    from astribot_s1_navigation_policy.protection import swept_point_collision
    from astribot_s1_navigation_policy.robot_envelope import EnvelopeProfile
    from astribot_s1_navigation_policy.stop_reference import describe_stop_reference
    config = root / 'ws_robot/src/astribot_s1_navigation_policy/config'
    base = Profile.load(config / 'simulation.json')
    profile = Profile.load(config / 'h2_simulation.json')
    reference = profile.stopping_reference
    assert hashlib.sha256((root / reference['source_file']).read_bytes()).hexdigest() == reference['source_sha256']
    fit = reference['polynomial']
    coverage = []
    for direction, samples in reference['samples'].items():
        for sample in samples:
            speed = sample['actual_mps']
            candidate = fit['engineering_scale'] * (fit['nominal_linear_s'] * speed +
                fit['nominal_quadratic_s2_per_m'] * speed**2) + fit['additive_margin_m']
            tail = profile.stopping_distance(speed) - speed*base.reaction_time_s - base.clearance_margin_m
            assert tail >= candidate - 1e-12 and tail >= sample['peak_m']
            coverage.append(dict(direction=direction, speed_m_s=speed,
                observed_peak_m=sample['peak_m'], tail_bound_m=tail))
    sweeps = []
    for speed in (.06, .10, .20, .35):
        for x, y in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            command = (x*speed, y*speed, 0.)
            old_end = speed*stopping_horizon(command, base) + base.half_length_m + base.clearance_margin_m
            new_end = speed*stopping_horizon(command, profile) + profile.half_length_m + profile.clearance_margin_m
            distance = (old_end + new_end) / 2
            point = ((x*distance, y*distance),)
            assert not swept_point_collision(point, command, base)
            assert swept_point_collision(point, command, profile)
            sweeps.append(dict(command=command, point=point[0], old_rejects=False, new_rejects=True))
    assert EnvelopeProfile(profile).stopping_distance(.2) == profile.stopping_distance(.2)
    for key in ('max_speed_m_s', 'max_angular_speed_rad_s', 'max_acceleration_m_s2',
                'max_angular_acceleration_rad_s2', 'angular_brake_deceleration_rad_s2'):
        assert getattr(profile, key) == getattr(base, key)
    assert stopping_horizon((0., 0., .6), profile) == stopping_horizon((0., 0., .6), base)
    assert describe_stop_reference(profile, .35)['range_status'] == 'outside_historical_range'
    budgets = [dict(speed_m_s=v, before_m=base.stopping_distance(v),
        after_m=profile.stopping_distance(v)) for v in (.055, .09, .16, .245, .35)]
    report = dict(passed=True, historical_samples_covered=len(coverage), coverage=coverage,
        four_direction_sweep_checks=sweeps, budgets=budgets,
        model=describe_stop_reference(profile, .245),
        boundary='Historical peak coverage and simulation bounds; not current hardware certification')
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('coverage','four_direction_sweep_checks')}, indent=2))


if __name__ == '__main__':
    main()
