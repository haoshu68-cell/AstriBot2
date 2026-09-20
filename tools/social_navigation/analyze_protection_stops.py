#!/usr/bin/env python3
"""Measure physical stop tails at stamped final-protection HOLD transitions."""
import argparse
import bisect
import json
import math
from pathlib import Path


def analyze(rows, stopping_distance):
    samples = {}
    last_source = -math.inf
    for row in rows:
        if row['kind'] == 'motion' and all(k in row for k in ('source_s', 'x', 'y', 'v', 'w')):
            keys = ('source_s', 'x', 'y', 'v', 'w')
            if not all(math.isfinite(row[k]) for k in keys) or row['source_s'] < last_source:
                raise ValueError('invalid or rewound motion source time')
            if row['source_s'] in samples and any(row[k] != samples[row['source_s']][k] for k in keys):
                raise ValueError('different motion values at one source time')
            samples.setdefault(row['source_s'], row)
            last_source = row['source_s']
    times = sorted(samples)
    if len(times) < 20:
        raise ValueError('position-bearing source-time motion evidence missing')
    constraints = [r for r in rows if r['kind'] == 'constraint']
    episodes = []
    active = None
    last = None
    for row in constraints:
        if last and (row['epoch'] != last['epoch'] or row['sequence'] <= last['sequence'] or row['source_s'] < last['source_s']):
            raise ValueError('constraint epoch/sequence/time changed; split recordings before analysis')
        if row['hold'] and (last is None or not last['hold']):
            active = row
        elif not row['hold'] and active:
            episodes.append((active, row['source_s'], True)); active = None
        last = row
    if active:
        episodes.append((active, times[-1], False))
    result = []
    for hold, end, released in episodes:
        start = hold['source_s']
        index = bisect.bisect_right(times, start)
        if index == 0 or index == len(times):
            continue
        left, right = samples[times[index-1]], samples[times[index]]
        f = (start-left['source_s'])/(right['source_s']-left['source_s'])
        speed = left['v']+f*(right['v']-left['v'])
        if speed < .02:
            continue
        origin = [left[k]+f*(right[k]-left[k]) for k in ('x','y')]
        motion = [samples[t] for t in times[index:] if t <= end]
        if not motion:
            continue
        gaps = [right['source_s']-left['source_s']] + [b['source_s']-a['source_s'] for a,b in zip(motion,motion[1:])]
        settled = None
        for i, sample in enumerate(motion):
            window = [r for r in motion[i:] if r['source_s'] <= sample['source_s']+.5]
            if len(window) >= 20 and window[-1]['source_s']-window[0]['source_s'] >= .45 and all(r['v'] < .01 and abs(r['w']) < .02 for r in window):
                settled = sample['source_s']; break
        peak = max(math.hypot(r['x']-origin[0],r['y']-origin[1]) for r in motion)
        budget = stopping_distance(speed)
        commands = [r for r in rows if r['kind']=='command' and start+.1 <= r['ros_s'] < end-.05]
        checks = dict(source_continuity=max(gaps) <= .1 and end-motion[-1]['source_s'] <= .1,
                      stop_observed=settled is not None,
                      held_command_zero=len(commands)>=2 and all(r['v']<1e-5 and abs(r['w'])<1e-5 for r in commands),
                      complete_budget_covers_peak=peak <= budget)
        result.append(dict(hold_source_s=start, reason=hold['reason'], release_source_s=end if released else None,
            speed_at_hold_m_s=speed, peak_excursion_m=peak, complete_budget_m=budget,
            stop_delay_s=None if settled is None else settled-start, checks=checks,
            verdict='PASS' if all(checks.values()) else 'FAIL'))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from astribot_s1_navigation_policy.profile import Profile
    profile = Profile.load(args.profile)
    episode_summary = json.loads((args.episode/'summary.json').read_text())
    if episode_summary.get('status') != 'completed':
        raise ValueError('episode recording is not complete')
    rows = [json.loads(line) for line in (args.episode/'observations.jsonl').read_text().splitlines()]
    result = analyze(rows, profile.stopping_distance)
    report = dict(episode=str(args.episode.resolve()), profile=str(args.profile.resolve()),
        episode_verdict=episode_summary.get('verdict'), stops=result,
        verdict='PASS' if result and all(r['verdict']=='PASS' for r in result) else ('FAIL' if result else 'INVALID_FIXTURE'),
        boundary='Stamped final-protection HOLD, existing Gazebo feedback; not normal smoothed zero, full-stage acceptance, or hardware certification.')
    with args.output.open('x') as output:
        json.dump(report, output, indent=2); output.write('\n')
    print(json.dumps(report, indent=2))
    return 0 if report['verdict']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
