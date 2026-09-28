#!/usr/bin/env python3
"""Paired real-controller update benchmark on ideal position interfaces."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rounds', type=int, default=30)
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '25', '40')
    assert args.rounds > 0
    args.output.mkdir(parents=True, exist_ok=False)
    binaries = {'baseline': args.build/'controller_stop_test', 'owned': args.build/'owned_controller_test'}
    observations = []
    for iteration in range(args.rounds):
        for kind in (('baseline', 'owned') if iteration % 2 == 0 else ('owned', 'baseline')):
            stem = args.output/f'{iteration:02}_{kind}'
            with stem.with_suffix('.log').open('w') as output:
                subprocess.run([str(binaries[kind]), '--gtest_filter=*.UpdateCycleBenchmark',
                                '--gtest_output=xml:'+str(stem.with_suffix('.xml'))],
                               check=True, stdout=output, stderr=subprocess.STDOUT, timeout=15)
            result = ET.parse(stem.with_suffix('.xml')).getroot()
            assert result.attrib['tests'] == '1' and result.attrib['failures'] == '0'
            observations.append(dict(iteration=iteration, kind=kind,
                **{p.attrib['name']: float(p.attrib['value']) for p in result.iter('property')}))
    summary = {}
    for metric in ('update_ns_mean', 'dispatch_and_update_ns_mean'):
        before = [r[metric] for r in observations if r['kind'] == 'baseline']
        after = [r[metric] for r in observations if r['kind'] == 'owned']
        baseline, owned = statistics.median(before), statistics.median(after)
        summary[metric] = dict(baseline_median=baseline, owned_median=owned,
                              difference=owned-baseline, percent=(owned/baseline-1)*100)
    report = dict(evidence='Isolated real JTC implementation, ideal position interface; not Gazebo or physical braking.',
        scope='100 Hz controller updates and synthetic heartbeat; same one-joint 0 to 1 rad trajectory over 2 seconds; each run measures first 0.5 seconds.',
        limitation='Shared host has other active simulation processes; descriptive microbenchmark, not exclusive full-stack performance acceptance.',
        binaries={name: dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for name,path in binaries.items()},
        rounds=args.rounds, domain=os.environ['ROS_DOMAIN_ID'], summary=summary, observations=observations)
    (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
