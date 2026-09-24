#!/usr/bin/env python3
"""Bounded, passive M5 metadata capture. No actions, services or control publishers.

This validation script observes ROS callback delivery, not driver/bridge latency.
It deliberately does not retain image/cloud payloads or claim ROI coverage.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
from queue import Full, Queue
import signal
import threading
import time


def validate_topics(topics):
    names = set()
    for item in topics:
        name = item['topic']
        if name in names:
            raise ValueError(f'Duplicate topic: {name}')
        names.add(name)
        if not name.startswith('/') or '/msg/' not in item['type']:
            raise ValueError(f'Absolute topic and ROS message type required: {name}')
        if item['qos'] not in ('sensor', 'reliable', 'latched'):
            raise ValueError(f'Unsupported QoS: {item["qos"]}')
        if item['kind'] not in ('image', 'info', 'cloud', 'raw_health',
                'projection_health', 'clock', 'status', 'odom', 'joints',
                'twist', 'tf', 'tf_static', 'constraint', 'protection', 'context'):
            raise ValueError(f'Unsupported observation kind: {item["kind"]}')
        if not isinstance(item['required'], bool):
            raise ValueError(f'required must be boolean: {name}')
        budget = item['gap_budget_sec']
        if budget is not None and (not math.isfinite(budget) or budget <= 0):
            raise ValueError(f'Positive gap budget required: {name}')
    clocks = [x for x in topics if x['kind'] == 'clock']
    if len(clocks) != 1 or clocks[0]['topic'] != '/clock' or not clocks[0]['required']:
        raise ValueError('Exactly one required /clock observation is required')


def metadata(kind, message, serialize):
    stamp = message.clock if kind == 'clock' else (
        message.header.stamp if hasattr(message, 'header') else None)
    row = {'source_ns': None if stamp is None else stamp.sec*10**9 + stamp.nanosec}
    if hasattr(message, 'header'):
        row['frame_id'] = message.header.frame_id
    if kind == 'image':
        row['data'] = {key: getattr(message, key) for key in
                       ('width', 'height', 'encoding', 'step', 'is_bigendian')}
        row['data']['bytes'] = len(message.data)
    elif kind == 'cloud':
        row['data'] = {key: getattr(message, key) for key in
                       ('width', 'height', 'is_dense', 'point_step', 'row_step')}
        row['data'].update(point_count=message.width*message.height,
                           bytes=len(message.data))
    else:
        row['data'] = serialize(message)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic-config', type=Path, required=True)
    parser.add_argument('--session-json', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--seconds', type=float, default=300)
    parser.add_argument('--warmup-sec', type=float, default=2)
    parser.add_argument('--reference', type=Path, nargs='*', default=[])
    parser.add_argument('--phase-topic', help='Explicit stage source, e.g. /transport/status; no default')
    parser.add_argument('--phase-type', default='std_msgs/msg/String')
    parser.add_argument('--phase-events', type=Path,
                        help='Owner event file to snapshot after capture; schema remains owner-defined')
    parser.add_argument('--check-config', action='store_true',
                        help='Validate the file offline; do not initialize ROS')
    args = parser.parse_args()
    topics = json.loads(args.topic_config.read_text())['topics']
    if args.phase_topic:
        topics.append({'topic': args.phase_topic, 'type': args.phase_type, 'kind': 'status',
                       'required': False, 'gap_budget_sec': None, 'qos': 'reliable'})
    validate_topics(topics)
    if args.check_config:
        print(json.dumps({'topics': len(topics), 'ros_initialized': False}))
        return
    if args.session_json is None or args.output is None:
        parser.error('--session-json and --output are required for capture')
    if not (1 <= args.seconds <= 600 and 0 <= args.warmup_sec < args.seconds):
        parser.error('duration must be 1..600 seconds and warmup shorter than duration')
    session = json.loads(args.session_json.read_text())
    expected_domain = str(session['isolation']['ROS_DOMAIN_ID'])
    if os.environ.get('ROS_DOMAIN_ID') != expected_domain:
        raise RuntimeError('Query ROS_DOMAIN_ID does not match authorized session snapshot')
    for key in ('IGN_PARTITION', 'GZ_PARTITION', 'ASTRIBOT_SIM_INSTANCE'):
        expected = session['isolation'].get(key)
        if expected and os.environ.get(key) != expected:
            raise RuntimeError(f'Query {key} does not match authorized session snapshot')
    # Imports occur only after the offline configuration and session checks.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    types = {item['type']: get_message(item['type']) for item in topics}
    references = [args.topic_config, args.session_json, Path(__file__), *args.reference]
    hashes = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in references}
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'session_snapshot.json').write_text(json.dumps(session, indent=2))
    manifest = {
        'schema': 'astribot.m5.observer/1', 'topics': topics,
        'source_sha256': hashes, 'requested_duration_sec': args.seconds,
        'warmup_sec': args.warmup_sec, 'session_json': str(args.session_json.resolve()),
        'session_log': session.get('unified_log'), 'completed': False,
        'interrupted': False, 'writer_dropped': 0, 'writer_queue_peak': 0,
        'observer_scope': 'ROS callback metadata; no internal DDS queue, ROI or causal control latency',
        'phase_source': {'topic': args.phase_topic, 'type': args.phase_type if args.phase_topic else None,
                         'owner_events_path': str(args.phase_events.resolve()) if args.phase_events else None},
        'clock_semantics': 'ros_now_ns is latest observed /clock; zero means not received',
        'environment': {k: os.environ.get(k) for k in (
            'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'FASTRTPS_DEFAULT_PROFILES_FILE',
            'ROS_LOCALHOST_ONLY', 'IGN_PARTITION', 'GZ_PARTITION', 'ASTRIBOT_SIM_INSTANCE')},
    }
    manifest['started_monotonic_ns'] = time.monotonic_ns()
    manifest['started_wall_ns'] = time.time_ns()
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2))
    (args.output/'events.jsonl').open('x').close()
    interrupted = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: interrupted.set())
    signal.signal(signal.SIGTERM, lambda *_: interrupted.set())
    pending = Queue(maxsize=8192)
    writer_error = []

    def write_events():
        try:
            with (args.output/'events.jsonl').open('a', buffering=1024*1024) as stream:
                while True:
                    row = pending.get()
                    if row is None:
                        break
                    stream.write(json.dumps(row, separators=(',', ':'), allow_nan=False)+'\n')
        except Exception as error:
            writer_error.append(error)  # Propagated below; never counted as successful capture.

    initialized = False
    node = None
    writer = None
    last_clock = 0

    def callback(item):
        def receive(message):
            nonlocal last_clock
            received = time.monotonic_ns()
            wall = time.time_ns()
            row = metadata(item['kind'], message, message_to_ordereddict)
            if item['kind'] == 'clock':
                last_clock = row['source_ns']
            row.update(kind=item['kind'], topic=item['topic'],
                       receive_monotonic_ns=received, receive_wall_ns=wall,
                       ros_now_ns=last_clock)
            if 'camera' in item:
                row['camera'] = item['camera']
            try:
                pending.put_nowait(row)
                manifest['writer_queue_peak'] = max(manifest['writer_queue_peak'], pending.qsize())
            except Full:
                manifest['writer_dropped'] += 1
                raise RuntimeError('Observer evidence queue overflow; capture is invalid')
        return receive

    subscriptions = []
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = Node('m5_passive_observer', enable_rosout=False, start_parameter_services=False)
        writer = threading.Thread(target=write_events, name='m5-evidence-writer', daemon=True)
        writer.start()
        for item in topics:
            qos = (qos_profile_sensor_data if item['qos'] == 'sensor' else
                   QoSProfile(depth=4, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if item['qos'] == 'latched' else QoSProfile(depth=10))
            subscriptions.append(node.create_subscription(
                types[item['type']], item['topic'], callback(item), qos))
        manifest['started_monotonic_ns'] = time.monotonic_ns()
        manifest['started_wall_ns'] = time.time_ns()
        (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2))
        deadline = manifest['started_monotonic_ns'] + int(args.seconds*1e9)
        while not interrupted.is_set() and time.monotonic_ns() < deadline:
            if writer_error:
                raise RuntimeError('Evidence writer failed') from writer_error[0]
            rclpy.spin_once(node, timeout_sec=.01)
        manifest['interrupted'] = interrupted.is_set()
        manifest['completed'] = not manifest['interrupted']
    except BaseException as error:
        manifest['error'] = f'{type(error).__name__}: {error}'
        manifest['completed'] = False
        raise
    finally:
        manifest['ended_monotonic_ns'] = time.monotonic_ns()
        manifest['ended_wall_ns'] = time.time_ns()
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        if writer is not None and writer.is_alive():
            drain_deadline = time.monotonic() + 10
            while writer.is_alive() and time.monotonic() < drain_deadline:
                try:
                    pending.put(None, timeout=.1)
                    break
                except Full:
                    if writer_error:
                        break
            writer.join(timeout=max(0, drain_deadline-time.monotonic()))
        writer_alive = writer is not None and writer.is_alive()
        if writer_alive or writer_error:
            manifest['completed'] = False
            manifest['writer_error'] = str(writer_error[0]) if writer_error else 'writer drain timeout'
        (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2))
        if (writer_alive or writer_error) and 'error' not in manifest:
            raise RuntimeError(manifest['writer_error'])
    if args.phase_events:
        # Preserve the owner's schema, IDs and timestamps; never infer a new phase.
        try:
            content = args.phase_events.read_bytes()
            (args.output/'phase_events_owner.jsonl').write_bytes(content)
        except OSError as error:
            manifest['completed'] = False
            manifest['error'] = f'Owner phase snapshot failed: {error}'
            (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2))
            raise
        manifest['phase_source']['snapshot_sha256'] = hashlib.sha256(content).hexdigest()
        (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(json.dumps({'output': str(args.output.resolve()), 'completed': manifest['completed']}))
    if manifest['interrupted']:
        raise SystemExit(130)


if __name__ == '__main__':
    main()
