#!/usr/bin/env python3
"""Offline evidence and injection-timing helper; contains no ROS or motion calls."""
import argparse
import json
import math
from pathlib import Path
import re

STATES = {
    'CORNER_APPROACH': ('APPROACH', 'CORNER_APPROACH'),
    'CORNER_SETTLING': ('SETTLING_BEFORE_TURN', 'CORNER_APPROACH'),
    'ALIGN_CORNER': ('TURNING', 'ALIGN_CORNER'),
    'CORNER_RECOVERY': ('RECOVERING', 'ALIGN_CORNER'),
    'CORNER_SETTLING_AFTER': ('SETTLING_AFTER_TURN', 'ALIGN_CORNER'),
    'REANCHOR_SETTLING': ('REANCHOR_SETTLING', 'REANCHOR_SETTLING'),
}
VALID_STATES = {state for state, _ in STATES.values()} | {'IDLE'}
PATTERN = re.compile(r'CORNER_STATE state=([A-Z_]+) cursor=(\d+) revision=(\d+) pose_s=([0-9.eE+-]+)(?:\s|$)')
METADATA = re.compile(r'schema=(\d+) execution=(\d+) event=(\d+) ros_s=([0-9.eE+-]+) '
                      r'pose_valid=([01]) active=([01]) reason=([a-z_]+)')
CONTROLLER_METADATA = {'schema', 'execution', 'event_id', 'ros_s', 'pose_valid', 'active', 'reason'}


def valid_metadata(event):
    try:
        return (type(event['schema']) is int and event['schema'] == 1 and
                type(event['execution']) is int and event['execution'] >= 0 and
                type(event['event_id']) is int and event['event_id'] > 0 and
                type(event['pose_valid']) is bool and type(event['active']) is bool and
                math.isfinite(event['ros_s']) and event['ros_s'] > 0 and
                math.isfinite(event['pose_s']) and
                (event['pose_s'] > 0 if event['pose_valid'] else event['pose_s'] == 0) and
                event['pose_s'] <= event['ros_s'] + .05 and
                event['active'] == (event['state'] not in ('IDLE', 'REANCHOR_SETTLING')) and
                event['reason'] in ('transition', 'reset', 'plan_refresh') and
                (event['reason'] != 'reset' or event['state'] == 'IDLE'))
    except (KeyError, TypeError):
        return False


def parse_state(message, **context):
    match = PATTERN.search(message)
    if not match or match[1] not in VALID_STATES:
        raise ValueError('missing or invalid explicit CORNER_STATE')
    payload = dict(state=match[1], cursor=int(match[2]), revision=int(match[3]), pose_s=float(match[4]))
    tail = message[match.end():].strip()
    if tail:
        metadata = METADATA.fullmatch(tail)
        if not metadata:
            raise ValueError('incomplete or unsupported CORNER_STATE metadata')
        payload.update(schema=int(metadata[1]), execution=int(metadata[2]), event_id=int(metadata[3]),
                       ros_s=float(metadata[4]), pose_valid=metadata[5]=='1', active=metadata[6]=='1',
                       reason=metadata[7])
        if not valid_metadata(payload):
            raise ValueError('invalid CORNER_STATE metadata')
    elif not math.isfinite(payload['pose_s']) or payload['pose_s'] <= 0:
        raise ValueError('invalid state source stamp')
    if (CONTROLLER_METADATA & context.keys()) - payload.keys():
        raise ValueError('controller metadata must come from the actual log payload')
    if any(key in context and context[key] != value for key, value in payload.items()):
        raise ValueError('recording context conflicts with controller payload')
    return dict(context, **payload)


class CornerPhaseGate:
    """Confirms a fresh recorded state; READY is neither motion authorization nor acceptance.

    The recorder must bind session/clock epoch to the actual owned publisher,
    assign a monotonic receive sequence, and provide the exact plan revision.
    Missing context must not be guessed from a phase name or a test-case label.
    """
    def __init__(self, session_id, clock_epoch, revision, cursor, requested_state,
                 armed_ros_s, armed_wall_s, timeout_s=10., observation_age_s=.3, execution=None):
        if (not isinstance(session_id, str) or not session_id.strip() or requested_state not in STATES or
                type(clock_epoch) is not int or clock_epoch < 0 or
                type(revision) is not int or revision < 1 or type(cursor) is not int or cursor < 0 or
                (execution is not None and (type(execution) is not int or execution < 1)) or
                not all(math.isfinite(v) and v > 0 for v in
                        (armed_ros_s, armed_wall_s, timeout_s, observation_age_s))):
            raise ValueError('invalid injection evidence context')
        self.session_id, self.clock_epoch = session_id, clock_epoch
        self.revision, self.cursor = revision, cursor
        self.state, self.phase = STATES[requested_state]
        self.armed_ros, self.armed_wall = armed_ros_s, armed_wall_s
        self.timeout, self.max_age = timeout_s, observation_age_s
        self.last_ros, self.last_wall = armed_ros_s, armed_wall_s
        self.event, self.error, self.sequence = None, None, -1
        self.source_watermark = armed_ros_s
        self.execution, self.event_id = execution, 0
        # A newly emitted state can reference the preceding fresh sensor tick.
        # Control emission must still follow arming; check() bounds both ages.
        self.pose_watermark = armed_ros_s - observation_age_s
        self.dispatched, self.candidate, self.confirmed = False, None, None

    def observe(self, event):
        if self.error or self.dispatched:
            return
        if (event.get('session_id'), event.get('clock_epoch')) != (self.session_id, self.clock_epoch):
            return
        versioned = 'schema' in event
        if versioned:
            if not valid_metadata(event):
                return
            if self.execution is None:
                self.candidate = None; self.event = None
                self.error = 'EXECUTION_CONTEXT_REQUIRED'; return
            if event['execution'] < self.execution or event['event_id'] <= self.event_id:
                return
        elif self.execution is not None:
            return
        stamp = event.get('ros_s') if versioned else event.get('pose_s')
        try:
            valid = (event['state'] in VALID_STATES and
                     all(type(event[k]) is int and event[k] >= 0 for k in ('revision', 'cursor', 'sequence', 'clock_epoch')) and
                     all(math.isfinite(event[k]) for k in ('pose_s', 'received_ros_s', 'received_wall_s')) and
                     stamp >= self.armed_ros and event['received_wall_s'] >= self.armed_wall and
                     event['received_ros_s'] >= self.armed_ros and
                     stamp <= event['received_ros_s']+.05 and
                     event['sequence'] > self.sequence)
        except (KeyError, TypeError):
            return
        if not valid:
            return
        if versioned and event['execution'] > self.execution:
            self.candidate = None; self.event = None; self.error = 'EXECUTION_CHANGED'; return
        if event['revision'] < self.revision:
            return
        self.sequence = event['sequence']; self.candidate = None
        self.event = None
        if versioned:
            self.event_id = event['event_id']
        if stamp < self.source_watermark:
            return
        self.source_watermark = stamp
        if event['revision'] != self.revision:
            self.error = 'PLAN_CHANGED'; return
        if event['cursor'] > self.cursor:
            self.error = 'CORNER_PASSED'; return
        if event['cursor'] != self.cursor:
            return
        if versioned and event['pose_valid']:
            if event['pose_s'] < self.pose_watermark:
                return
            self.pose_watermark = event['pose_s']
        self.event = dict(event)

    def check(self, now_ros_s, now_wall_s, pose_s, phase):
        self.candidate = None
        def result(reason, status='NOT_READY'):
            return {'status': status, 'reason': reason, 'effect_validated': False}
        if not all(math.isfinite(v) for v in (now_ros_s, now_wall_s, pose_s)):
            self.error = 'INVALID_TIME'
        elif now_ros_s < self.last_ros or now_wall_s < self.last_wall:
            self.error = 'CLOCK_ROLLBACK'
        self.last_ros, self.last_wall = now_ros_s, now_wall_s
        if self.error:
            return result(self.error, 'INVALID_EVIDENCE')
        if self.dispatched:
            return result('ALREADY_DISPATCHED')
        if now_wall_s-self.armed_wall > self.timeout:
            self.error = 'INJECTION_DEADLINE'; return result(self.error, 'INVALID_EVIDENCE')
        if self.event is None:
            return result('NO_STATE_EVENT')
        e = self.event
        if e['state'] != self.state or phase != self.phase:
            return result('STATE_MISMATCH')
        if 'schema' in e and not e['pose_valid']:
            return result('POSE_UNAVAILABLE')
        if 'schema' in e and not 0 <= now_ros_s-e['ros_s'] <= self.max_age:
            return result('CONTROL_EVENT_STALE_OR_FUTURE')
        if not (0 <= now_ros_s-e['pose_s'] <= self.max_age and
                0 <= now_ros_s-e['received_ros_s'] <= self.max_age and
                0 <= now_wall_s-e['received_wall_s'] <= self.max_age):
            return result('STATE_EVENT_STALE_OR_FUTURE')
        if pose_s < e['pose_s'] or not 0 <= now_ros_s-pose_s <= self.max_age:
            return result('POSE_STALE_OR_FUTURE')
        self.candidate = dict(result('EXPLICIT_STATE_MATCH', 'READY_FOR_INJECTION'),
                              revision=self.revision, cursor=self.cursor, state=self.state,
                              sequence=e['sequence'], checked_ros_s=now_ros_s, checked_wall_s=now_wall_s)
        self.candidate.update(execution=self.execution, event_id=e.get('event_id'),
                              identity_scope='EXECUTION_BOUND' if self.execution is not None else 'LEGACY_CONTEXT_ONLY')
        return self.candidate

    def dispatch(self, verdict, now_ros_s, now_wall_s):
        if self.error or self.dispatched or verdict is not self.candidate or not verdict or verdict['status'] != 'READY_FOR_INJECTION':
            raise ValueError('dispatch needs the current matching verdict and can occur only once')
        if (not all(math.isfinite(v) for v in (now_ros_s, now_wall_s)) or
                now_ros_s < verdict['checked_ros_s'] or now_wall_s < verdict['checked_wall_s'] or
                now_ros_s-self.event['pose_s'] > self.max_age or
                now_ros_s-self.event['received_ros_s'] > self.max_age or
                ('schema' in self.event and now_ros_s-self.event['ros_s'] > self.max_age) or
                now_wall_s-self.event['received_wall_s'] > self.max_age or
                now_wall_s-self.armed_wall > self.timeout):
            self.candidate = None
            raise ValueError('dispatch no longer lies inside the recorded state evidence window')
        self.dispatched = True; self.candidate = None

    def confirm(self, acknowledged):
        if not self.dispatched or type(acknowledged) is not bool:
            raise ValueError('acknowledgement needs an actual recorded dispatch and boolean result')
        if self.confirmed is not None:
            if self.confirmed['acknowledged'] != acknowledged:
                raise ValueError('conflicting acknowledgement')
            return self.confirmed
        self.confirmed = {'status': 'INJECTION_ACKNOWLEDGED' if acknowledged else 'INJECTION_FAILED',
                          'acknowledged': acknowledged, 'effect_validated': False,
                          'scope': 'Transport acknowledgement only; physical effect and controller outcome need separate evidence.'}
        return self.confirmed


def replay(data):
    gate = CornerPhaseGate(**data['context'])
    verdict, outputs = None, []
    for record in data['records']:
        kind = record['kind']
        if kind == 'state':
            gate.observe(record['event'])
        elif kind == 'probe':
            verdict = gate.check(**record['sample']); outputs.append(dict(kind=kind, **verdict))
        elif kind == 'dispatch':
            gate.dispatch(verdict, record['now_ros_s'], record['now_wall_s']); outputs.append({'kind': kind, 'status': 'DISPATCH_RECORDED'})
        elif kind == 'ack':
            outputs.append(dict(kind=kind, **gate.confirm(record['acknowledged'])))
        else:
            raise ValueError('unknown replay record kind')
    return {'status': 'EVIDENCE_TIMING_ONLY', 'results': outputs, 'robot_acceptance': 'NOT_EVALUATED'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = replay(json.loads(args.input.read_text()))
    except (ValueError, KeyError, TypeError, OSError) as error:
        result = {'status': 'INVALID_EVIDENCE', 'reason': str(error)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2, allow_nan=False))
    return 2 if result['status']=='INVALID_EVIDENCE' else 0


if __name__ == '__main__':
    raise SystemExit(main())
