#!/usr/bin/env python3
"""Analyze passive M5 evidence; sampling checks are not robot acceptance."""
import argparse
from collections import Counter
import json
from pathlib import Path


def statistics(values):
    """Percentiles use linear interpolation at (n - 1) * percentile."""
    ordered = sorted(values)
    result = {'samples': len(ordered), 'percentile_method': 'linear'}
    if not ordered:
        return result
    result.update(min=ordered[0], max=ordered[-1])
    for name, fraction in [('p50', .5), ('p95', .95), ('p99', .99)]:
        position = (len(ordered) - 1) * fraction
        lower = int(position)
        upper = min(lower + 1, len(ordered) - 1)
        result[name] = ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)
    return result


def analyze(folder):
    folder = Path(folder)
    manifest = json.loads((folder / 'manifest.json').read_text())
    start = manifest['started_monotonic_ns']
    end = manifest['ended_monotonic_ns']
    warmup = manifest.get('warmup_sec', 2.)
    cutoff = start + round(warmup * 1e9)
    if end < start or warmup < 0:
        raise ValueError('Invalid capture interval or warmup')
    specs = {item['topic']: item for item in manifest['topics']}
    if len(specs) != len(manifest['topics']):
        raise ValueError('Duplicate topics in manifest')
    grouped = {topic: [] for topic in specs}
    phases, status_observations = [], []
    with (folder / 'events.jsonl').open() as stream:
        for line in stream:
            event = json.loads(line)
            topic = event['topic']
            if topic not in specs or event['kind'] != specs[topic]['kind']:
                raise ValueError(f'Event is not bound to manifest topic/kind: {topic}')
            received = event['receive_monotonic_ns']
            if not start <= received <= end:
                raise ValueError(f'Event outside capture interval: {topic}')
            if grouped[topic] and received < grouped[topic][-1]['receive_monotonic_ns']:
                raise ValueError(f'Receive clock regressed: {topic}')
            grouped[topic].append(event)
            if event['kind'] == 'status':
                status = event['data']
                observation = {'topic': topic, 'receive_monotonic_ns': received,
                               'source_ns': event['source_ns'], 'data': status,
                               'parse_status': 'UNPARSED', 'reason': 'NO_STAGE_OR_PHASE_FIELD'}
                if 'stage' not in status and 'phase' not in status and 'data' in status:
                    if isinstance(status['data'], str):
                        try:
                            status = json.loads(status['data'])
                        except json.JSONDecodeError:
                            observation['reason'] = 'STRING_IS_NOT_JSON'
                    else:
                        observation['reason'] = 'STRING_PAYLOAD_IS_NOT_TEXT'
                if isinstance(status, dict) and any(key in status for key in ('stage', 'phase')):
                    labels = {key: status[key] for key in ('stage', 'phase') if key in status}
                    observation.update(parse_status='PARSED', reason='EXPLICIT_STAGE_OR_PHASE_FIELD')
                    phases.append({'topic': topic, **labels, 'receive_monotonic_ns': received,
                                   'source_ns': event['source_ns']})
                elif not isinstance(status, dict):
                    observation['reason'] = 'STATUS_JSON_IS_NOT_OBJECT'
                status_observations.append(observation)
    complete = (manifest['completed'] and not manifest['interrupted']
                and manifest['writer_dropped'] == 0 and end > cutoff)
    result = {
        'scope': 'Passive observer samples; no continuous-time coverage or robot acceptance',
        'observation_status': 'COMPLETE' if complete else 'INCOMPLETE',
        'sampling_criteria_scope': 'Required topic presence and explicit receive-gap budgets only',
        'duration_wall_sec': (end - start) * 1e-9,
        'warmup_sec': warmup,
        'metrics_window': 'At or after start + warmup; full-window sample counts retained',
        'writer_dropped': manifest['writer_dropped'],
        'streams': {},
        'phase_observations': phases,
        'status_observations': status_observations,
        'phase_scope': 'Observed task stage labels; not proof of physical motion',
        'not_measured': {key: 'NOT_MEASURED' for key in [
            'actual_motion', 'control_latency', 'control_causality', 'roi_coverage',
            'three_dimensional_coverage', 'graph_ownership', 'dds_queue_occupancy']},
    }
    checks = []
    for topic, spec in specs.items():
        all_rows = grouped[topic]
        rows = [r for r in all_rows if r['receive_monotonic_ns'] >= cutoff]
        receives = [r['receive_monotonic_ns'] for r in rows]
        gaps = [(b - a) * 1e-9 for a, b in zip(receives, receives[1:])]
        sources = [r['source_ns'] for r in rows if r['source_ns'] is not None]
        source_intervals = [(b - a) * 1e-9 for a, b in zip(sources, sources[1:])]
        ages = [(r['ros_now_ns'] - r['source_ns']) * 1e-9 for r in rows
                if r['source_ns'] is not None and r['ros_now_ns'] != 0]
        initial = (receives[0] - cutoff) * 1e-9 if receives else None
        tail = (end - receives[-1]) * 1e-9 if receives else None
        budget = spec.get('gap_budget_sec')
        if budget is not None and budget <= 0:
            raise ValueError(f'Nonpositive receive-gap budget: {topic}')
        gap_pass = bool(rows) and (budget is None or
                   (initial <= budget and tail <= budget and all(g <= budget for g in gaps)))
        row = dict(kind=spec['kind'], required=spec['required'],
                   presence='PRESENT' if rows else 'MISSING', samples=len(rows),
                   samples_full_window=len(all_rows), receive_interval_wall_sec=statistics(gaps),
                   source_interval_sec=statistics(source_intervals),
                   duplicate_source_intervals=sum(d == 0 for d in source_intervals),
                   source_regressions=sum(d < 0 for d in source_intervals),
                   source_age_ros_sec=statistics(ages),
                   source_age_negative_future_samples=sum(age < 0 for age in ages),
                   source_age_unavailable_samples=len(rows) - len(ages),
                   initial_silence_wall_sec=initial, tail_silence_wall_sec=tail,
                   gap_budget_sec=budget,
                   receive_gaps_over_budget=sum(g > budget for g in gaps) if budget is not None else None,
                   gap_budget_status=('PASS' if gap_pass else 'FAIL') if budget is not None else 'NOT_EVALUATED')
        if spec['required']:
            checks.append(gap_pass)
        if spec['kind'] == 'clock':
            clock_rows = [r for r in rows if r['source_ns'] is not None]
            advance = (sources[-1] - sources[0]) * 1e-9 if sources else None
            span = ((clock_rows[-1]['receive_monotonic_ns'] - clock_rows[0]['receive_monotonic_ns'])
                    * 1e-9 if clock_rows else None)
            rtf_status = ('SOURCE_REGRESSION' if row['source_regressions'] else
                          'INSUFFICIENT_SAMPLES' if len(clock_rows) < 2 or span <= 0 else 'OBSERVED')
            row['clock'] = dict(source_advance_sec=advance, receive_steady_span_sec=span,
                                observed_rtf=advance / span if rtf_status == 'OBSERVED' else None,
                                rtf_status=rtf_status,
                                frozen_source_intervals=row['duplicate_source_intervals'],
                                source_regressions=row['source_regressions'],
                                scope='Source clock advance divided by observer steady receive span; not Gazebo execution timing')
        if spec['kind'] in ('raw_health', 'projection_health'):
            epoch_field = 'source_epoch' if spec['kind'] == 'raw_health' else 'processing_epoch'
            first_seen = {}
            steady_ages, capture_ages, capture_unavailable = [], [], 0
            last_key = None
            for event in all_rows:
                data = event['data']
                stamp = data['capture_stamp']
                capture_ns = stamp['sec'] * 10**9 + stamp['nanosec']
                key = (data[epoch_field], capture_ns)
                received = event['receive_monotonic_ns']
                first_seen.setdefault(key, received)
                last_key = key
                if received >= cutoff:
                    steady_ages.append((received - first_seen[key]) * 1e-9)
                    if event['ros_now_ns'] != 0:
                        capture_ages.append((event['ros_now_ns'] - capture_ns) * 1e-9)
                    else:
                        capture_unavailable += 1
            health = dict(
                valid_samples=sum(r['data']['valid'] for r in rows),
                invalid_samples=sum(not r['data']['valid'] for r in rows),
                state_counts=dict(Counter(r['data']['state'] for r in rows if 'state' in r['data'])),
                reason_counts=dict(Counter(r['data']['reason_code'] for r in rows if 'reason_code' in r['data'])),
                epochs=sorted({r['data'][epoch_field] for r in rows}),
                capture_age_ros_sec=statistics(capture_ages),
                capture_age_negative_future_samples=sum(age < 0 for age in capture_ages),
                capture_age_unavailable_samples=capture_unavailable,
                capture_steady_age_sec=statistics(steady_ages),
                capture_steady_tail_age_sec=(end - first_seen[last_key]) * 1e-9 if last_key else None,
                steady_age_scope='Time since observer first saw same epoch and capture stamp; not acquisition latency')
            if spec['kind'] == 'projection_health':
                health.update(pending_depth=statistics([r['data']['pending_depth'] for r in rows]),
                              pending_depth_scope='Sampled application latest-pending slot only',
                              dds_queue_occupancy='NOT_MEASURED')
            row['health'] = health
        result['streams'][topic] = row
    result['sampling_criteria_status'] = ('INCOMPLETE' if not complete else
                                          'NOT_EVALUATED' if not checks else
                                          'PASS' if all(checks) else 'FAIL')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.capture)
    with args.output.open('x') as output:
        json.dump(result, output, indent=2)
        output.write('\n')
    print(json.dumps({'observation_status': result['observation_status'],
                      'sampling_criteria_status': result['sampling_criteria_status']}))
    return 0 if result['sampling_criteria_status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
