"""Offline derived assertions; never rewrites the original exit-1 evidence."""
from pathlib import Path
import hashlib
import json

base = Path(__file__).resolve().parent
inputs = [base / 'late_revision_green_26.json', base / 'domain_26.jsonl']
data = json.loads(inputs[0].read_text())
records = [json.loads(line) for line in inputs[1].read_text().splitlines()]
events = data['events']
injected = next(e['wall'] for e in events if e['event'] == 'late_invalid_published')
sends = [e for e in events if e['event'] == 'send' and e['controller'] != 'mtc']
assert len(sends) == 18 and all(e['wall'] < injected for e in sends)
assert not any(confirmed for _, confirmed in data['holds'])
assert not data['outcome']['resources_released']
assert data['outcome']['status'] == 6
assert any(r['event'] == 'stop_requested' and r['reason'] == 'PAYLOAD_RAW_REVISION_CHANGED' for r in records)
assert any(s['authority_reason'] == 'PAYLOAD_RAW_REVISION_CHANGED' for s in data['statuses'])
assert not any(r['event'] == 'payload_transaction_confirmed' for r in records)
assert records[-1]['event'] == 'quarantined' and records[-1]['phase'] == 5
assert records[-1]['side_effects']
assert not any(r.get('reason') == 'RESOURCE_CLOCK_RESET' for r in records)
assert all(p['returncode'] == 0 for p in events[-1]['processes'])

stages = ['PREGRASP', 'GRASP_APPROACH', 'GRASP_CONFIRM']
assert [r['details']['stage_id'] for r in records if r['event'] == 'stage_confirmed'] == stages
expected = {name: 0.0 for name in data['geometry_samples'][0]['positions']}
settling = []
for i, stage in enumerate(stages):
    for joint in expected:
        if (i < 2 and joint.startswith('astribot_arm_left_joint_')) or (i == 2 and joint == 'astribot_gripper_left_joint_L1'):
            expected[joint] += .005
    submitted = [e for e in sends if e['stage'] == stage]
    terminal = [e for e in events if e['event'] == 'terminal' and e['stage'] == stage]
    assert len(submitted) == len(terminal) == 6
    assert len({e['controller'] for e in submitted}) == 6
    assert {e['uuid'] for e in submitted} == {e['uuid'] for e in terminal}
    assert all(e['status'] == 4 for e in terminal)
    end = max(e['wall'] for e in terminal)
    end_ros = max(e['ros_ns'] for e in terminal)
    cutoff = (min(e['wall'] for e in sends if e['stage'] == stages[i + 1]) if i < 2
              else next(e['wall'] for e in events if e['event'] == 'physical_command'))
    samples = [v for v in data['geometry_samples'] if end < v['wall'] < cutoff and
               v['capture_ns'] > end_ros and v['confirmed'] and v['positions'] == expected]
    assert len(samples) >= 3
    steady_span = samples[-1]['wall'] - samples[0]['wall']
    source_span = samples[-1]['capture_ns'] - samples[0]['capture_ns']
    assert steady_span >= .5 and source_span >= 500000000
    settling.append(dict(stage=stage, samples=len(samples), steady_span_s=steady_span, source_span_ns=source_span))

result = dict(
    scope='Offline derived barrier and settling checks only; original driver exit 1 remains unchanged',
    original_driver_pass=False, derived_checks_pass=True,
    native_initial_stop='PAYLOAD_RAW_REVISION_CHANGED',
    final_action=data['outcome'], all_jtc=18, after_injection_jtc=0,
    payload_confirmations=0, hold=False, quarantine_phase=5, settling=settling,
    first_cause_return_gap='cleanup geometry_fresh overwrites reason_, so final Action reason omits initial raw failure',
    inputs={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
)
(base / 'domain26_derived_audit.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
