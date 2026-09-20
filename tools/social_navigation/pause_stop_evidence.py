"""Source-timed stop evidence while a person occupies the route or goal."""
import bisect
import math

from analyze_protection_stops import analyze


def assess_pause_stop(rows, pause, resume, stopping_distance, clearance_margin):
    if not all(math.isfinite(t) for t in (pause, resume)) or resume-pause < .5:
        raise ValueError('A completed pause with a continuous settling window is required')
    stops = analyze(rows, stopping_distance)
    relevant = [s for s in stops if s['hold_source_s'] < resume and
                (s['release_source_s'] is None or s['release_source_s'] > pause)]
    motion = {r['source_s']: r for r in rows if r['kind'] == 'motion'}
    times = sorted(t for t in motion if pause <= t <= resume)
    continuous = (len(times) >= 20 and times[0]-pause <= .1 and resume-times[-1] <= .1
                  and all(0 < b-a <= .1 for a, b in zip(times, times[1:])))
    def stopped(t):return motion[t]['v'] < .01 and abs(motion[t]['w']) < .02
    settled = None
    for index, t in enumerate(times):
        window=times[index:bisect.bisect_right(times,t+.5)]
        if len(window)>=20 and window[-1]-t>=.45 and all(stopped(s) for s in window):
            settled=t;break
    settled_ok = continuous and settled is not None and resume-settled >= .5
    # Starting the clock again after creeping would conceal a resumed motion.
    stayed_stopped = settled_ok and all(stopped(t) for t in times if t>=settled)
    constraints = [r for r in rows if r['kind'] == 'constraint']
    stamps = [r['source_s'] for r in constraints]
    held = False
    if settled_ok and stamps:
        first = bisect.bisect_right(stamps, settled)-1
        selected = constraints[max(0, first):bisect.bisect_right(stamps, resume)]
        held = (first >= 0 and bool(selected) and all(r['hold'] for r in selected)
                and selected[0]['source_s']+selected[0]['lease_s'] >= settled
                and selected[-1]['source_s']+selected[-1]['lease_s'] >= resume
                and all(b['source_s'] <= a['source_s']+a['lease_s']
                        for a, b in zip(selected, selected[1:])))
    commands = [r for r in rows if r['kind'] == 'command' and settled is not None
                and settled <= r['ros_s'] <= resume]
    command_zero = (len(commands) >= 2 and commands[0]['ros_s']-settled <= .1
                    and resume-commands[-1]['ros_s'] <= .1
                    and all(r['v'] < 1e-5 and abs(r['w']) < 1e-5 for r in commands)
                    and all(0 <= b['ros_s']-a['ros_s'] <= .1
                            for a, b in zip(commands, commands[1:])))
    geometry = [r for r in rows if r['kind'] == 'geometry' and pause <= r['stamp_ns']*1e-9 <= resume]
    geometry_times=[r['stamp_ns']*1e-9 for r in geometry]
    geometry_continuous=(len(geometry_times)>=3 and geometry_times[0]-pause<=.15
        and resume-geometry_times[-1]<=.15
        and all(0<b-a<=.15 for a,b in zip(geometry_times,geometry_times[1:]))
        and all(r['geometry_valid'] and r['conservative_overlap_steps']==0 for r in geometry))
    clearance = min((r['clearance_lower_bound_m'] for r in geometry), default=None)
    # Already stopped at pause is valid only with continuous motion, leased
    # HOLD and zero-command evidence; it is not counted as a braking trial.
    stationary_from_pause = continuous and all(stopped(t) for t in times)
    checks = dict(pause_motion_continuous=continuous,
        pause_geometry_continuous=geometry_continuous,
        settled_before_person_resumes=settled_ok,
        no_motion_resume_while_occupied=stayed_stopped,
        hold_maintained_while_occupied=held,
        occupied_wait_command_zero=command_zero,
        occupied_clearance=clearance is not None and clearance >= clearance_margin,
        hold_braking_budget=(bool(relevant) and all(s['verdict'] == 'PASS' for s in relevant))
            or (stationary_from_pause and not relevant))
    legacy = [motion[t] for t in times if t >= pause+1.5]
    return dict(contract='source_hold_budget_v2', checks=checks, stops=relevant,
        pause_source_s=pause, resume_source_s=resume, settled_source_s=settled,
        stationary_from_pause=stationary_from_pause, clearance_lower_bound_m=clearance,
        clearance_margin_m=clearance_margin, verdict='PASS' if all(checks.values()) else 'FAIL',
        legacy_pause_plus_1_5s_pass=bool(legacy) and all(r['v'] < .01 and abs(r['w']) < .02 for r in legacy))
