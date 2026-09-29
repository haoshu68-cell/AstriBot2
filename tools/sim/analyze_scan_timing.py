#!/usr/bin/env python3
"""Read-only R0 scan timing analysis, preserving ROS and steady clock domains."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from analyze_policy_freshness import distribution


def summarize(rows):
    records={};stages={key:[] for key in ('capture_to_receive_ros','receive_to_first_tf_checked_ready',
                                       'receive_to_select','select_to_finish')}
    observation_ages=[];constraint_ages=[];after_finish=[];diagnostics=0;epoch_mismatch=0
    failed={};waiting={};counters={};negative=Counter()
    for row in rows:
        data=row.get('data',{})
        timing=data.get('scan_timing')
        constraint=data.get('scan_timing_constraint')
        if timing is not None:
            diagnostics+=1;epoch=timing.get('clock_epoch')
            counters[str(epoch)]={key:timing.get(key) for key in ('drop_counts','defer_counts','missing_updates','evictions','resets')}
            for field in ('waiting','finished','successful'):
                rec=timing.get(field)
                if not rec:continue
                if rec.get('clock_epoch')!=epoch:
                    epoch_mismatch+=1;continue
                key=(epoch,rec['sequence'])
                if field=='waiting':waiting[key]=rec
                if field=='finished' and not rec.get('processing_success'):failed[key]=rec
                if field=='successful' and rec.get('processing_success'):records[key]=rec
            rec=timing.get('successful');published=timing.get('observation_pre_serialize')
            if rec and published and rec.get('clock_epoch')==epoch and rec.get('processing_success'):
                observation_ages.append((published['ros_ns']-rec['capture_ros_ns'])/1e6)
        if constraint is not None:
            rec=constraint.get('successful');published=constraint.get('publish_started')
            if rec and published and rec.get('processing_success'):
                # Constraint payload lacks a recorder top-level epoch; retain its
                # record epoch and compare with world version if supplied.
                epoch=constraint.get('world_version',{}).get('clock_epoch',rec['clock_epoch'])
                if rec['clock_epoch']!=epoch:
                    epoch_mismatch+=1;continue
                records[(epoch,rec['sequence'])]=rec
                constraint_ages.append((published['ros_ns']-rec['capture_ros_ns'])/1e6)
                if rec.get('finished'):
                    after_finish.append((published['steady_ns']-rec['finished']['steady_ns'])/1e6)
    for rec in records.values():
        stages['capture_to_receive_ros'].append((rec['receive']['ros_ns']-rec['capture_ros_ns'])/1e6)
        for label,first,last in [('receive_to_first_tf_checked_ready','receive','first_tf_ready'),
                                 ('receive_to_select','receive','selected'),('select_to_finish','selected','finished')]:
            if rec.get(first) and rec.get(last):
                stages[label].append((rec[last]['steady_ns']-rec[first]['steady_ns'])/1e6)
    for label,values in {**stages,'observation_publish_age':observation_ages,
                         'constraint_publish_age':constraint_ages,'finish_to_constraint':after_finish}.items():
        negative[label]=sum(v<0 for v in values)
    return dict(diagnostic_samples=diagnostics,unique_successful_scans=len(records),
                stages_ms={key:distribution(values) for key,values in stages.items()},
                observation_publish_age_ms=distribution(observation_ages),
                constraint_publish_age_ms=distribution(constraint_ages),
                finish_to_constraint_ms=distribution(after_finish),
                constraint_over_300ms_samples=sum(v>300. for v in constraint_ages),
                unique_waiting_scans=len(waiting),
                waiting_edges=dict(Counter(str((r.get('tracking_tf'),r.get('map_tf'))) for r in waiting.values())),
                processing_failure_reasons=dict(Counter(r.get('reason','') for r in failed.values())),
                epoch_mismatch_records=epoch_mismatch,negative_intervals=dict(negative),last_counters_by_epoch=counters)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();raw=args.trace.read_bytes();rows=[];incomplete=False
    lines=raw.splitlines()
    for index,line in enumerate(lines):
        try:rows.append(json.loads(line))
        except ValueError:
            if index==len(lines)-1 and not raw.endswith(b'\n'):incomplete=True
            else:raise
    result=dict(trace=str(args.trace.resolve()),sha256=hashlib.sha256(raw).hexdigest(),
                incomplete_tail_ignored=incomplete,summary=summarize(rows),
                limitations=['Sampled diagnostics, not every frame or worst-case bound.',
                             'First TF checked ready includes time before first check; it is not pure TF latency.',
                             'Per-stage distributions use unique successful epoch/sequence; publication ages use samples.',
                             'Steady compute intervals and ROS capture ages are separate; percentiles are not additive.'])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result['summary'],indent=2))


if __name__=='__main__':main()
