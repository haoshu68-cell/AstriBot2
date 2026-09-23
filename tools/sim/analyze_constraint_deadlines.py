#!/usr/bin/env python3
"""Read-only wire/source deadline audit; no runtime or motion authority."""
import argparse
from collections import Counter
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path


def lease_within_deadline(stamp_ns, lease_s, deadline_ns):
    return (isinstance(stamp_ns,int) and isinstance(deadline_ns,int) and stamp_ns>=0
            and math.isfinite(lease_s) and 0<lease_s<=.5 and stamp_ns<deadline_ns
            and Fraction.from_float(float(lease_s))*10**9 <= deadline_ns-stamp_ns)


def keyed(rows, source, get, errors):
    result={}
    for row in rows:
        if row.get('source')!=source:continue
        value=get(row.get('data',{}))
        if value is None:continue
        key=(value['epoch'],value['sequence'])
        if key in result and result[key]!=value:errors.append(dict(reason='CONFLICTING_DUPLICATE',source=source,key=key))
        result[key]=value
    return result


def sequence_coverage(wires, errors):
    epochs={}
    for epoch,sequence in wires:epochs.setdefault(epoch,[]).append(sequence)
    coverage=[]
    for epoch,sequences in epochs.items():
        values=sorted(set(sequences));gaps=[(a+1,b-1) for a,b in zip(values,values[1:]) if b>a+1]
        coverage.append(dict(epoch=epoch,first=values[0],last=values[-1],captured=len(values),internal_gaps=gaps))
        if gaps:errors.append(dict(reason='WIRE_SEQUENCE_GAPS',epoch=epoch,gaps=gaps))
    return coverage


def audit_proposed(rows):
    errors=[];positive_pairs=0;unpaired=[];hold_count=0
    wires=keyed(rows,'proposed_constraint_wire',lambda d:d,errors)
    coverage=sequence_coverage(wires,errors)
    states={}
    for row in rows:
        if row.get('source')!='state':continue
        data=row.get('data',{});gate=data.get('publication_gate')
        if gate is None:continue
        key=(gate['constraint_epoch'],gate['constraint_sequence'])
        if key in states and states[key]['gate']!=gate:errors.append(dict(reason='CONFLICTING_GATE',key=key))
        states[key]=dict(gate=gate,health=data.get('coverage_health'),publish_ns=data.get('scan_timing_constraint',{}).get('publish_started',{}).get('ros_ns'))
    for key,wire in wires.items():
        if wire['hold']:
            hold_count+=1;continue
        if key not in states:unpaired.append(key);continue
        positive_pairs+=1;state=states[key];g=state['gate'];checks={}
        checks['WIRE_GATE_MISMATCH']=(g['constraint_stamp_ns']==wire['stamp_ns']==g['publication_ns']
                                      and g['constraint_lease_s']==wire['lease_s'] and g['constraint_hold']==wire['hold'])
        checks['GATE_DID_NOT_AUTHORIZE']=g['allowed'] is True
        checks['EPOCH_OR_TIME_REVERSED']=(g['decision_epoch']==g['publication_epoch'] and 0<=g['decision_ns']<=g['publication_ns'])
        sources=g['required_sources']
        health=state['health']
        original=[dict(sensor_id=v['sensor_id'],capture_ns=v['capture_ns'],deadline_ns=v['valid_until_ns']) for v in health if v.get('required') is True] if isinstance(health,list) else []
        checks['REQUIRED_SOURCE_SNAPSHOT_MISMATCH']=(bool(original) and len({v['sensor_id'] for v in sources})==len(sources)
                                                  and sorted(sources,key=lambda v:v['sensor_id'])==sorted(original,key=lambda v:v['sensor_id']))
        checks['INVALID_SOURCE_INTERVALS']=bool(sources) and all(0<=v['capture_ns']<=wire['stamp_ns']<v['deadline_ns'] for v in sources)
        checks['DEADLINE_NOT_ORIGINAL_MINIMUM']=bool(sources) and g['deadline_ns']==min(v['deadline_ns'] for v in sources)
        checks['WIRE_LEASE_EXCEEDS_SOURCE']=lease_within_deadline(wire['stamp_ns'],wire['lease_s'],g['deadline_ns'])
        checks['PUBLICATION_AFTER_DEADLINE']=(state['publish_ns'] is not None and wire['stamp_ns']<=state['publish_ns']<g['deadline_ns'])
        for reason,passed in checks.items():
            if not passed:errors.append(dict(reason=reason,key=key,stamp_ns=wire['stamp_ns'],lease_s=wire['lease_s'],deadline_ns=g['deadline_ns'],publish_ns=state['publish_ns']))
    for key,state in states.items():
        if not state['gate']['constraint_hold'] and key not in wires:
            errors.append(dict(reason='POSITIVE_GATE_WITHOUT_WIRE',key=key))
    return dict(passed=positive_pairs>0 and not errors and not unpaired,wire_samples=len(wires),
                positive_pairs=positive_pairs,hold_samples=hold_count,unpaired_positive=unpaired,
                coverage=coverage,error_counts=dict(Counter(e['reason'] for e in errors)),errors=errors)


def audit_final(rows):
    errors=[];positive_pairs=0;unpaired=[];hold_count=0
    wires=keyed(rows,'final_constraint_wire',lambda d:d,errors)
    coverage=sequence_coverage(wires,errors)
    proposals=keyed(rows,'proposed_constraint_wire',lambda d:d,errors)
    diagnostics={}
    for row in rows:
        if row.get('source')!='protection':continue
        d=row.get('data',{})
        if 'constraint_sequence' not in d:continue
        key=(d['constraint_epoch'],d['constraint_sequence'])
        if key in diagnostics and diagnostics[key]!=d:errors.append(dict(reason='CONFLICTING_FINAL_DIAGNOSTIC',key=key))
        diagnostics[key]=d
    for key,wire in wires.items():
        if wire['hold']:hold_count+=1;continue
        d=diagnostics.get(key)
        if d is None:unpaired.append(key);continue
        p=proposals.get((d['proposal_epoch'],d['proposal_sequence']))
        if p is None:unpaired.append(key);continue
        positive_pairs+=1
        # Runtime may conservatively discard an extra nanosecond. Verify that
        # declared boundary is no later than the exact wire lifetime.
        exact_deadline=p['stamp_ns']+int(Fraction.from_float(float(p['lease_s']))*10**9)
        upstream_deadline=d['proposal_deadline_ns']
        checks={
            'FINAL_WIRE_MISMATCH':wire['stamp_ns']==d['publication_stamp_ns'] and wire['lease_s']==d['effective_lease_s'] and wire['hold']==d['hold'],
            'FINAL_INPUTS_NOT_FRESH':d['publication_inputs_fresh'] is True and d['publication_proposal_fresh'] is True,
            'UPSTREAM_IDENTITY_OR_DEADLINE':not p['hold'] and d['proposal_stamp_ns']==p['stamp_ns'] and isinstance(upstream_deadline,int) and p['stamp_ns']<upstream_deadline<=exact_deadline,
        }
        intervals=[(p['stamp_ns'],upstream_deadline),(d['scan_capture_ns'],d['scan_deadline_ns']),(d['odom_capture_ns'],d['odom_deadline_ns'])]
        checks['FINAL_INVALID_SOURCE_INTERVAL']=all(isinstance(b,int) and 0<=a<=wire['stamp_ns']<b for a,b in intervals)
        deadlines=[b for _,b in intervals if isinstance(b,int)]
        checks['FINAL_NOT_MINIMUM_DEADLINE']=len(deadlines)==3 and d['effective_deadline_ns']==min(deadlines)
        checks['FINAL_RENEWED_SOURCE_LEASE']=all(lease_within_deadline(wire['stamp_ns'],wire['lease_s'],b) for b in deadlines) and len(deadlines)==3
        checks['FINAL_PUBLISHED_AFTER_DEADLINE']=bool(deadlines) and all(wire['stamp_ns']<=d[name]<min(deadlines) for name in ('output_publish_start_ns','constraint_publish_start_ns'))
        for reason,passed in checks.items():
            if not passed:errors.append(dict(reason=reason,key=key,stamp_ns=wire['stamp_ns'],lease_s=wire['lease_s'],deadline_ns=d['effective_deadline_ns']))
    for key,d in diagnostics.items():
        if not d['hold'] and key not in wires:
            errors.append(dict(reason='POSITIVE_FINAL_DIAGNOSTIC_WITHOUT_WIRE',key=key))
    return dict(passed=positive_pairs>0 and not errors and not unpaired,wire_samples=len(wires),
                positive_pairs=positive_pairs,hold_samples=hold_count,unpaired_positive=unpaired,
                coverage=coverage,error_counts=dict(Counter(e['reason'] for e in errors)),errors=errors)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();raw=args.trace.read_bytes();rows=[json.loads(line) for line in raw.splitlines()]
    result=dict(trace=str(args.trace.resolve()),sha256=hashlib.sha256(raw).hexdigest(),proposed=audit_proposed(rows),final=audit_final(rows),
                limitations=['Checks captured wire messages and matching diagnostics; unrecorded emissions are not proven.',
                             'Lease is anchored at message stamp, never receive time.',
                             'This contract covers the listed required sensor snapshots, not every world/odom input.'])
    result['passed']=result['proposed']['passed'] and result['final']['passed']
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({stage:{k:v for k,v in result[stage].items() if k!='errors'} for stage in ('proposed','final')},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
