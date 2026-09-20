#!/usr/bin/env python3
"""Read-only comparison of raw zero, smoothed command and physical stop."""
import argparse
import json
import math
from pathlib import Path


def analyze(directory):
    summary = json.loads((directory / 'summary.json').read_text())
    rows = [json.loads(line) for line in (directory / 'observations.jsonl').read_text().splitlines()]
    raw_summary = summary.get('normal_stop', summary)
    zero = raw_summary['zero_ros_s']
    motion = sorted((r for r in rows if r['kind'] == 'motion'), key=lambda r: r['source_s'])
    unique = {r['source_s']: r for r in motion}
    motion = list(unique.values())
    commands = [r for r in rows if r['kind'] == 'final_command']
    raw = [r for r in rows if r['kind'] == 'probe_command']
    preceding = [r for r in commands if r['ros_s'] <= zero]
    before = [r for r in motion if r['source_s'] <= zero]
    if not preceding or not before or zero - preceding[-1]['ros_s'] > .1 or zero - before[-1]['source_s'] > .1:
        raise ValueError('no fresh command/feedback at raw zero')
    after = [r for r in motion if r['source_s'] >= zero]
    speed = lambda r: math.hypot(r['vx'], r['vy'])
    peak = max(after, key=speed)
    final_zero = next((r for r in commands if r['ros_s'] >= zero and speed(r) < 1e-5 and abs(r['w']) < 1e-5), None)
    result = dict(
        episode=str(directory.resolve()), original_verdict=summary['verdict'],
        zero_ros_s=zero, measured_at_zero_m_s=speed(before[-1]),
        measured_preceding_median_m_s=raw_summary['actual_speed_before_zero_m_s'],
        downstream_command_at_zero_m_s=speed(preceding[-1]),
        measured_peak_after_zero_m_s=speed(peak), measured_peak_delay_s=peak['source_s']-zero,
        downstream_zero_delay_s=None if final_zero is None else final_zero['ros_s']-zero,
        peak_excursion_m=raw_summary['peak_euclidean_m'],
        measured_only_budget_m=raw_summary['full_stopping_budget_m'],
        measured_only_budget_covers_peak=raw_summary['checks']['complete_budget_covers_peak'],
        boundary='Normal raw-input zero through the smoother; not a final-protection stop or a hardware stop certification.',
        acceptance='Diagnostic only; original verdict and stage gate unchanged.')
    return result, raw, commands, motion


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episodes', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    results = []
    for directory in args.episodes:
        result, raw, commands, motion = analyze(directory)
        results.append(result)
        zero = result['zero_ros_s']
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, constrained_layout=True)
        for series, label, key in ((raw, 'raw command', 'ros_s'), (commands, 'downstream command (receipt time)', 'ros_s'), (motion, 'measured velocity (source time)', 'source_s')):
            selected = [r for r in series if -2 <= r[key]-zero <= 4]
            axes[0].plot([r[key]-zero for r in selected], [math.hypot(r['vx'],r['vy']) for r in selected], label=label)
        summary = json.loads((directory/'summary.json').read_text())
        origin = summary.get('normal_stop',summary)['interpolated_zero_xy']
        selected = [r for r in motion if 0 <= r['source_s']-zero <= 4]
        axes[1].plot([r['source_s']-zero for r in selected], [math.hypot(r['x']-origin[0],r['y']-origin[1]) for r in selected], label='excursion after raw zero')
        axes[1].axhline(result['measured_only_budget_m'], color='tab:red', linestyle='--', label='measured-only budget')
        for ax in axes:
            ax.axvline(0, color='black', linestyle=':'); ax.grid(alpha=.25); ax.legend()
        axes[0].set_ylabel('speed magnitude (m/s)'); axes[1].set_ylabel('distance (m)')
        axes[1].set_xlabel('physical seconds from raw zero')
        axes[0].set_title(directory.parent.name + ' / ' + result['original_verdict'])
        fig.savefig(args.output/(directory.parent.name+'.png'), dpi=150); plt.close(fig)
    (args.output/'summary.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
