"""Use historical chassis peak excursion as a conservative simulation prior."""
import copy
import math


def apply_stop_reference(values, reference):
    """Tighten analytical bounds; this does not certify the current hardware plant.

    B_peak = scale * (a*v + b*v*v) + margin is a spatial engineering
    envelope, not the final resting offset. Its equivalent delay/deceleration
    are bounds for the existing sweep, not identified mechanical parameters.
    """
    if values['environment'] != 'simulation' or reference['hardware_validated']:
        raise ValueError('historical stop reference is simulation-only')
    if reference['schema_version'] != 1 or reference['quantity'] != 'peak_excursion':
        raise ValueError('unsupported stop reference')
    fit = reference['polynomial']
    names = ('nominal_linear_s', 'nominal_quadratic_s2_per_m',
             'engineering_scale', 'additive_margin_m')
    if any(not math.isfinite(fit[name]) or fit[name] <= 0 for name in names):
        raise ValueError('positive finite stopping coefficients required')
    lo, hi = reference['actual_speed_range_m_s']
    if not 0 < lo < hi or not math.isfinite(hi):
        raise ValueError('invalid measured speed range')
    result = copy.deepcopy(values)
    result['linear_stop_delay_s'] = fit['engineering_scale'] * fit['nominal_linear_s']
    result['brake_deceleration_m_s2'] = min(result['brake_deceleration_m_s2'],
        1. / (2. * fit['engineering_scale'] * fit['nominal_quadratic_s2_per_m']))
    result['clearance_margin_m'] += fit['additive_margin_m']
    result['stopping_reference'] = copy.deepcopy(reference)
    result['sources']['braking'] = reference['model_id'] + ': historical peak-envelope prior; simulation engineering bound'
    return result


def describe_stop_reference(profile, speed):
    reference = getattr(profile, 'stopping_reference', None)
    if reference is None:
        return None
    lo, hi = reference['actual_speed_range_m_s']
    return dict(model_id=reference['model_id'], actual_speed_m_s=speed,
        historical_speed_range_m_s=[lo, hi],
        range_status='within_historical_range' if lo <= speed <= hi else 'outside_historical_range',
        hardware_validated=False, brake_deceleration_bound_m_s2=profile.brake_deceleration_m_s2,
        reaction_budget_s=profile.reaction_time_s,
        linear_post_zero_delay_bound_s=profile.linear_stop_delay_s,
        clearance_margin_m=profile.clearance_margin_m,
        stopping_distance_budget_m=profile.stopping_distance(speed))
