#!/usr/bin/env python3
"""Recompute the compact acceptance result from preserved completed-run summaries."""
import json
from pathlib import Path

root = Path(__file__).resolve().parent
read = lambda path: json.loads((root / path).read_text())
before = read('wall_ab/aggregate.json')
after = read('final_wall_ab/aggregate.json')
phases_before = read('phase_sweep/summary.json')
phases_after = read('final_phase_sweep/summary.json')
faults = read('completion_faults/summary.json')

comparison = []
for scenario in ('empty', 'offset_payload'):
    select = lambda rows, variant: next(r for r in rows['groups']
                                        if r['scenario'] == scenario and r['variant'] == variant)
    old_cpp = select(before, 'cpp')
    cpp = select(after, 'cpp')
    python = select(after, 'python')
    comparison.append(dict(
        scenario=scenario,
        cpp_before_source_to_publish_ms=old_cpp['source_to_publish_ms'],
        cpp_after_source_to_publish_ms=cpp['source_to_publish_ms'],
        python_source_to_publish_ms=python['source_to_publish_ms'],
        latency_no_regression=all(cpp['source_to_publish_ms'][p] <= python['source_to_publish_ms'][p]
                                  for p in ('p50', 'p95', 'p99')),
        receipt_latency_no_regression=all(cpp['source_to_receive_ms'][p] <= python['source_to_receive_ms'][p]
                                          for p in ('p50', 'p95', 'p99')),
        cpp_before_cpu_percent_one_core=old_cpp['cpu_percent_one_core'],
        cpp_after_cpu_percent_one_core=cpp['cpu_percent_one_core'],
        python_cpu_percent_one_core=python['cpu_percent_one_core'],
        cpp_rss_median_mib=cpp['rss_mib']['p50'],
        python_rss_median_mib=python['rss_mib']['p50'],
        all_rounds_valid=cpp['performance_comparison_eligible'] and python['performance_comparison_eligible']))

phase_pairs = []
for stage, rows in (('before', phases_before), ('after', phases_after)):
    for scenario in ('empty', 'offset_payload'):
        for phase in (0, 5, 10, 15):
            pair = {r['variant']: r for r in rows
                    if r['scenario'] == scenario and r['source_phase_ms'] == phase}
            assert set(pair) == {'cpp', 'python'}
            assert all(not r['validation_failures'] and not r['incomplete_reasons']
                       and not r['output_sequence_gaps'] and r['complete_frames'] for r in pair.values())
            for field in ('source_sequence_sha256', 'clock_sequence_sha256'):
                assert pair['cpp'][field] == pair['python'][field]
            phase_pairs.append(dict(
                stage=stage, scenario=scenario, source_phase_ms=phase,
                source_and_clock_sequences_identical=True,
                variants={name: r['metrics'] for name, r in pair.items()}))

assert all(r['rejected_complete_source'] for r in faults)
assert all(r['all_rounds_valid'] for r in comparison)
result = dict(
    scope='Isolated ROS2 fixture, one articulated arm joint, empty and offset payload; no simulator or hardware',
    comparisons=comparison, controlled_phase_pairs=phase_pairs,
    completion_faults=faults,
    cpp_complete_frames=sum(r['complete_frames'] for r in after['groups'] if r['variant'] == 'cpp'),
    python_complete_frames=sum(r['complete_frames'] for r in after['groups'] if r['variant'] == 'python'),
    performance_passed=all(r['latency_no_regression'] and r['receipt_latency_no_regression']
                           and r['cpp_after_cpu_percent_one_core'] < r['python_cpu_percent_one_core']
                           and r['cpp_rss_median_mib'] < r['python_rss_median_mib'] for r in comparison),
    excluded_batch='contended_aborted: startup overlapped another test; entire partial batch excluded')
(root / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({k: v for k, v in result.items() if k != 'controlled_phase_pairs'}, indent=2))
