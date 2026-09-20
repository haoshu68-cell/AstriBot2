#!/usr/bin/env python3
"""Summarize BRIDGE_TIMING windows without treating window quantiles as global quantiles."""
import argparse
import json
from pathlib import Path


def analyze(path):
    windows = []
    decoder = json.JSONDecoder()
    with Path(path).open(errors='replace') as source:
        for line in source:
            marker = line.find('BRIDGE_TIMING ')
            if marker < 0:
                continue
            try:
                row, _ = decoder.raw_decode(line[marker + len('BRIDGE_TIMING '):])
            except ValueError:
                continue
            windows.append(row)
    metrics = {}
    for name in sorted({k for row in windows for k in row['timing']}):
        values = [row['timing'][name] for row in windows if name in row['timing']]
        count = sum(v['samples'] for v in values)
        metrics[name] = {
            'samples': count, 'dropped': sum(v['dropped'] for v in values),
            'mean_ms': sum(v['samples']*v['mean_ms'] for v in values)/count,
            'max_ms': max(v['max_ms'] for v in values),
            'window_p95_range_ms': [min(v['p95_ms'] for v in values), max(v['p95_ms'] for v in values)],
            'window_p99_range_ms': [min(v['p99_ms'] for v in values), max(v['p99_ms'] for v in values)],
        }
    return {'windows': len(windows), 'metrics': metrics, 'window_details': windows,
            'interpretation': 'Monotonic wall durations during enabled operation. SDK call duration is API execution time, not physical actuator response. Window quantiles are not pooled quantiles; dropped samples invalidate full-window tail coverage.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = analyze(args.log)
    Path(args.output).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='window_details'}, indent=2))
