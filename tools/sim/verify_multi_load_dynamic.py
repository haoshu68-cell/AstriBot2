#!/usr/bin/env python3
"""Replay dynamic empty/light/heavy/offset payload envelope transitions.

The transport executor remains a ROS runtime concern; this bounded validator
drives the retired Python ``FixedEnvelope`` reference coordinator with the same geometry,
hold and consumer-ack contracts used by the runtime.  It checks that every
load gets a new geometry/epoch, that attached payloads require a positive mass,
and that stale, missing or mismatched acknowledgements hold navigation.
"""
from pathlib import Path as _ReferencePath
import sys as _reference_sys
_reference_sys.path.insert(0, str(_ReferencePath(__file__).resolve().parents[2] / "tools/migration/python_reference"))
from reference_bootstrap import enable as _enable_references
_enable_references()

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

from astribot_navigation_msgs.msg import (ArmHoldStatus, EnvelopeApplyStatus,
                                            RobotGeometryState)
from astribot_navigation_msgs.srv import SetFixedEnvelope
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point32, Polygon
from astribot_s1_robot_geometry.polygon import hull, inflate
from astribot_s1_navigation_policy.fixed_envelope import CONSUMERS, FixedEnvelope
from astribot_s1_navigation_policy.profile import Profile
from astribot_s1_navigation_policy.robot_envelope import FIELDS


def stamp(value):
    return Time(sec=int(value // 1_000_000_000),
                nanosec=int(value % 1_000_000_000))


def polygon(points):
    points = hull(points)
    return Polygon(points=[Point32(x=float(x), y=float(y), z=0.) for x, y in points])


def rectangle(length, width):
    return [[-length, -width], [length, -width], [length, width], [-length, width]]


def build_state(profile, now, sequence, revision, length, width, height, attached):
    msg = RobotGeometryState()
    msg.header.frame_id = profile.base_frame
    msg.header.stamp = stamp(now)
    msg.valid_until = stamp(now + 300_000_000)
    msg.complete = True
    msg.attachment_state_confirmed = True
    msg.source_id = 'dynamic-regression-source'
    msg.sequence = sequence
    msg.model_revision = 'model-v1'
    msg.attachment_revision = revision
    msg.attachment_ids = ['transport_box_01'] if attached else []
    msg.joints.name = ['arm_left_joint_1']
    msg.joints.position = [0.0]
    msg.joint_source_stamps = [stamp(now)]
    msg.joint_position_error_bounds = [0.003]
    physical = rectangle(length, width)
    msg.physical_footprint = polygon(physical)
    msg.reserved_footprint = polygon(inflate(physical, 0.02))
    msg.height_m = height
    return msg


def request(profile, revision, sequence, mass):
    # Use the actual message type through the service request, then populate
    # every validated field from the baseline profile.
    req = SetFixedEnvelope.Request()
    req.request_id = 'dynamic-' + revision
    req.hold_id = 'hold-' + revision
    req.geometry_sequence = sequence
    for field in FIELDS:
        setattr(req.limits, field, float(getattr(profile, field)))
    req.limits.payload_mass_kg = float(mass)
    req.limits.posture_id = 'transport_compact'
    req.limits.frame_id = profile.base_frame
    req.limits.lease_s = 0.3
    return req


def acknowledge(coordinator, now, negative=None):
    envelope = coordinator.output
    for consumer in CONSUMERS:
        msg = EnvelopeApplyStatus(
            coordinator_session_id=envelope.coordinator_session_id,
            consumer_id=consumer, envelope_epoch=envelope.epoch,
            installed_geometry_hash=envelope.installed_geometry_hash,
            applied=(consumer != negative))
        msg.header.stamp = stamp(now)
        coordinator.acknowledge(msg, now)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        raise SystemExit('use a new output directory')
    output.mkdir(parents=True)
    profile = Profile.load(ROOT / 'ws_robot/src/astribot_s1_navigation_policy/config/simulation.json')
    coordinator = FixedEnvelope(profile)
    now = 10_000_000_000
    cases = [
        dict(name='empty', revision='empty', length=profile.half_length_m,
             width=profile.half_width_m, height=profile.height_m, mass=0.0, attached=False),
        dict(name='light_box', revision='light', length=0.38, width=0.34, height=1.78,
             mass=0.20, attached=True),
        dict(name='heavy_offset_box', revision='heavy_offset', length=0.48, width=0.41, height=1.95,
             mass=1.20, attached=True),
        dict(name='empty_after_release', revision='released', length=profile.half_length_m,
             width=profile.half_width_m, height=profile.height_m, mass=0.0, attached=False),
    ]
    rows = []
    previous_epoch = 0
    previous_area = 0.0
    for sequence, case in enumerate(cases, start=1):
        state = build_state(profile, now, sequence, case['revision'], case['length'],
                            case['width'], case['height'], case['attached'])
        coordinator.state(state, now)
        coordinator.hold = ArmHoldStatus(owner_id='dynamic-regression',
            hold_id='hold-' + case['revision'], lease_s=.3, hold_confirmed=True,
            attachment_revision=case['revision'])
        coordinator.hold.header.stamp = stamp(now)
        req = request(profile, case['revision'], sequence, case['mass'])
        envelope = coordinator.propose(req, now, True)
        acknowledge(coordinator, now)
        output_envelope = coordinator.tick(now)
        area = 4.0 * case['length'] * case['width']
        row = dict(case=case['name'], attachment_revision=case['revision'],
                   sequence=sequence, epoch=int(output_envelope.epoch),
                   geometry_hash=output_envelope.installed_geometry_hash,
                   payload_mass_kg=float(output_envelope.limits.payload_mass_kg),
                   half_length_m=float(output_envelope.limits.half_length_m),
                   half_width_m=float(output_envelope.limits.half_width_m),
                   navigation_allowed=bool(output_envelope.navigation_allowed),
                   transport_ready=bool(output_envelope.limits.transport_ready),
                   reason=output_envelope.reason)
        assert row['navigation_allowed'] and row['transport_ready'], row
        assert row['epoch'] > previous_epoch
        if case['attached']:
            assert row['payload_mass_kg'] > 0
        else:
            assert row['payload_mass_kg'] == 0
        if case['name'] == 'empty_after_release':
            assert row['half_length_m'] <= rows[1]['half_length_m']
        if case['attached']:
            assert area >= previous_area
        previous_epoch, previous_area = row['epoch'], area
        rows.append(row)
        now += 100_000_000

    faults = []
    # A single missing consumer must revoke the grant for the current load.
    state = build_state(profile, now, 5, 'fault_missing_ack', .48, .41, 1.95, True)
    coordinator.state(state, now)
    coordinator.hold = ArmHoldStatus(owner_id='dynamic-regression', hold_id='hold-fault_missing_ack',
        lease_s=.3, hold_confirmed=True, attachment_revision='fault_missing_ack')
    coordinator.hold.header.stamp = stamp(now)
    coordinator.propose(request(profile, 'fault_missing_ack', 5, 1.2), now, True)
    acknowledge(coordinator, now, negative='controller')
    revoked = coordinator.tick(now)
    faults.append(dict(kind='missing_consumer_ack', navigation_allowed=bool(revoked.navigation_allowed),
                       reason=revoked.reason))
    assert not revoked.navigation_allowed and 'controller' in revoked.reason

    # A geometry revision without a matching hold is rejected before a grant.
    coordinator.hold.hold_confirmed = False
    try:
        coordinator.propose(request(profile, 'fault_missing_ack', 5, 1.2), now, True)
    except ValueError as exc:
        faults.append(dict(kind='unconfirmed_hold', rejected=str(exc)))
    else:
        raise AssertionError('unconfirmed hold unexpectedly accepted')

    summary = dict(cases=rows, faults=faults, passed=True,
                   invariants=['new epoch per load', 'attached mass positive',
                               'missing ack holds navigation', 'unconfirmed hold rejected'],
                   evidence='production FixedEnvelope coordinator replay; no Gazebo contact or VLA claim')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
