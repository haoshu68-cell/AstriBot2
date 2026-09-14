#!/usr/bin/env python3
"""Read-only hardware evidence collection; offline commands need only Python 3."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
TEMPLATE = ROOT / 'ws_robot/src/astribot_s1_navigation_policy/config/hardware.template.json'
ITEMS = {
    '01_geometry': '测量运输姿态、双臂夹爪、载荷的完整包络、质量和误差界',
    '02_frames': '静止及人工低速前后左右/正负旋转，检查坐标方向、单位、TF及定位抖动',
    '03_braking': '每个方向/速度/载荷/地面分别测停车距离、余转和停止响应',
    '04_watchdog': '人工验证上游断流、约束断流、末级退出、驱动命令断流的独立停车',
    '05_latency': '检查采集时间、时钟偏差、推理/融合/执行延迟及高负载尾延迟',
    '06_coverage': '标靶逐格验证雷达/视觉近场、盲区、自滤、低矮/悬空障碍和遮挡',
    '07_tracking': '标靶运动真值验证身份关联、速度、协方差、遮挡和无深度退化',
    '08_navigation': '直线/转弯/接近段重复路线；独立真值验证到点、横偏和航向',
    '09_avoidance': '横穿/迎面/暂堵/久堵/绕行/全局重规划/目标不可达',
    '10_narrow': '已测宽度通道的入口对齐、通行、会车、出口堵塞及退出',
}
DEFAULT_TOPICS = [
    ('/astribot_chassis/joint_space_states', 'astribot_msgs/msg/RobotJointState', True),
    ('/astribot_chassis/joint_space_command_recv', 'astribot_msgs/msg/RobotJointState', False),
    ('/odom', 'nav_msgs/msg/Odometry', True),
    ('/scan', 'sensor_msgs/msg/LaserScan', True),
    ('/tf', 'tf2_msgs/msg/TFMessage', True),
    ('/tf_static', 'tf2_msgs/msg/TFMessage', False),
    ('/joint_states', 'sensor_msgs/msg/JointState', False),
    ('/cmd_vel', 'geometry_msgs/msg/Twist', False),
    ('/cmd_vel_nav_body', 'geometry_msgs/msg/Twist', False),
    ('/cmd_vel_policy_input', 'geometry_msgs/msg/Twist', False),
    ('/plan', 'nav_msgs/msg/Path', False),
    ('/path_tracking/phase', 'std_msgs/msg/String', False),
    ('/path_tracking/replan_event', 'std_msgs/msg/String', False),
    ('/navigation_policy/observation', 'std_msgs/msg/String', False),
    ('/navigation_policy/protection_state', 'std_msgs/msg/String', False),
    ('/navigation_policy/vision_observations', 'std_msgs/msg/String', False),
    ('/clock', 'rosgraph_msgs/msg/Clock', False),
]


def read(path):
    return json.loads(Path(path).read_text())


def write(path, data):
    with Path(path).open('x') as f:
        json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('数值必须有限')
    return value


def positive(value):
    value = finite(value)
    if value <= 0:
        raise argparse.ArgumentTypeError('必须大于零')
    return value


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def percentile(values, q):
    values = sorted(values)
    return values[min(len(values)-1, math.ceil(q * len(values))-1)] if values else None


def init(args):
    root = Path(args.directory)
    root.mkdir(parents=True, exist_ok=False)
    profile = read(args.template)
    write(root / 'hardware.candidate.json', profile)
    write(root / 'topics.json', [{'topic': t, 'type': typ, 'required': req,
          'transient_local': t == '/tf_static', 'role': ('manufacturer_feedback' if t == '/astribot_chassis/joint_space_states' else 'command_echo_not_measurement' if t == '/astribot_chassis/joint_space_command_recv' else 'slam_derived_reference' if t == '/odom' else 'auxiliary')} for t, typ, req in DEFAULT_TOPICS])
    write(root / 'conditions.json', {key: '' for key in (
        'robot_serial', 'operator', 'software_revision', 'posture_and_joint_positions',
        'payload_kg_and_shape', 'floor_and_slope', 'battery', 'localization_source',
        'command_frame_and_bridge', 'clock_sync_evidence', 'external_reference_and_uncertainty')})
    write(root / 'measurements.json', {
        'template_sha256': digest(args.template), 'hardware_validated': False,
        'items': {k: {'instruction': v, 'status': 'pending', 'evidence': [],
                     'measured_values': {}, 'reviewer': '', 'notes': ''} for k, v in ITEMS.items()},
        'parameter_review': {k: {'template_value': v, 'proposed_value': None,
                                'evidence': [], 'status': 'pending'}
                             for k, v in profile.items()
                             if k not in ('sources', 'hardware_evidence', 'hardware_validated')}
    })
    (root / 'arrival.csv').write_text(
        'trial,reference,goal_x_m,goal_y_m,goal_yaw_deg,actual_x_m,actual_y_m,actual_yaw_deg,position_uncertainty_m,yaw_uncertainty_deg\n')
    print(root.resolve())


def record(args):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    root = Path(args.directory)
    topics = read(root / 'topics.json')
    if len({t['topic'] for t in topics}) != len(topics):
        raise ValueError('topics.json 有重复话题')
    types = {t['topic']: get_message(t['type']) for t in topics}
    condition = read(root / 'conditions.json')
    out = root / args.item / time.strftime('%Y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=[])
    node = rclpy.create_node('hardware_parameter_recorder')
    start = time.monotonic()
    stop = [False]
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    counts = {t['topic']: 0 for t in topics}
    received = {t['topic']: [] for t in topics}
    ages = {t['topic']: [] for t in topics}
    stamps = {t['topic']: [] for t in topics}
    meta = {'environment_label': args.environment, 'conditions': condition,
            'started_unix_s': time.time(), 'topics': topics, 'duration_requested_s': args.duration,
            'script_sha256': digest(__file__),
            'profile_sha256': digest(root / 'hardware.candidate.json'),
            'dds_environment': {k: os.environ.get(k) for k in (
                'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION',
                'FASTRTPS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI')},
            'motion_authority': 'none; subscriptions only', 'hardware_validated': False}
    write(out / 'metadata.json', meta)
    with (out / 'messages.jsonl').open('x', buffering=1) as log:
        def receive(msg, topic):
            now = time.monotonic() - start
            stamp = None
            if hasattr(msg, 'header'):
                s = msg.header.stamp
                stamp = s.sec + s.nanosec * 1e-9
            wall = time.time()
            entry = {'t_s': now, 'receive_unix_s': wall, 'topic': topic,
                     'stamp_s': stamp, 'msg': clean(message_to_ordereddict(msg))}
            log.write(json.dumps(entry, allow_nan=False) + '\n')
            counts[topic] += 1
            received[topic].append(now)
            if stamp is not None:
                stamps[topic].append(stamp)
                ages[topic].append(wall - stamp)
        for spec in topics:
            qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=(DurabilityPolicy.TRANSIENT_LOCAL if spec.get('transient_local')
                                         else DurabilityPolicy.VOLATILE))
            node.create_subscription(types[spec['topic']], spec['topic'],
                                     lambda msg, t=spec['topic']: receive(msg, t), qos)
        print(f'只读采集中：{out.resolve()}\n另一个终端可运行 mark；Ctrl-C 正常封口。', flush=True)
        try:
            while not stop[0] and time.monotonic() - start < args.duration:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            elapsed = time.monotonic() - start
            summary = {}
            for spec in topics:
                t = spec['topic']
                seq, source = received[t], stamps[t]
                summary[t] = {
                    'frames': counts[t], 'required': spec.get('required', False),
                    'receive_hz': (len(seq)-1)/(seq[-1]-seq[0]) if len(seq)>1 else None,
                    'discovery_delay_s': seq[0] if seq else None,
                    'last_receive_age_s': elapsed-seq[-1] if seq else None,
                    'max_receive_gap_s': max((b-a for a,b in zip(seq, seq[1:])), default=None),
                    'nonincreasing_source_stamps': sum(b<=a for a,b in zip(source, source[1:])),
                    'source_age_p95_s': percentile(ages[t], .95),
                    'source_age_max_s': max(ages[t], default=None),
                    'source_age_min_s': min(ages[t], default=None)}
            # Observation of frames is not an acceptance decision about hardware health.
            write(out / 'capture_summary.json', {
                'elapsed_s': elapsed, 'interrupted': stop[0], 'topics': summary,
                'missing_required': [t for t in counts if not counts[t] and summary[t]['required']],
                'clock_observed': counts.get('/clock', 0) > 0,
                'age_interpretation': 'only valid with synchronized wall-clock source timestamps',
                'hardware_validated': False})
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    print(f'采集完成；检查 capture_summary.json，收到数据不等于参数已通过。目录：{out.resolve()}')
    return 1 if any(not counts[t['topic']] for t in topics if t.get('required')) else 0


def mark(args):
    out = Path(args.run)
    meta = read(out / 'metadata.json')
    write(out / f'mark_{time.time_ns()}.json', {
        'label': args.label, 'receive_unix_s': time.time(), 'notes': args.notes,
        'kind': 'manual_approximate_event', 'run_started_unix_s': meta['started_unix_s']})
    print('标记已保存。人工标记含操作延迟；精确制动起点请用实收命令/驱动时间。')


def load_odom(run, topic):
    rows = []
    with (Path(run) / 'messages.jsonl').open() as f:
        for line in f:
            e = json.loads(line)
            if e['topic'] != topic:
                continue
            m = e['msg']; p = m['pose']['pose']; q = p['orientation']; v = m['twist']['twist']
            rows.append({'t': finite(e['t_s']), 'stamp': finite(e['stamp_s']),
                         'frame': m['header']['frame_id'], 'child': m['child_frame_id'],
                         'x': finite(p['position']['x']), 'y': finite(p['position']['y']),
                         'yaw': math.atan2(2*(q['w']*q['z']+q['x']*q['y']),
                                           1-2*(q['y']**2+q['z']**2)),
                         'v': math.hypot(finite(v['linear']['x']), finite(v['linear']['y'])),
                         'w': abs(finite(v['angular']['z']))})
    return rows


def brake(args):
    rows = load_odom(args.run, args.odom_topic)
    before = [r for r in rows if args.event_s-.5 <= r['t'] <= args.event_s]
    after = [r for r in rows if args.event_s <= r['t'] <= args.event_s+args.window]
    if not before or len(after)<3:
        raise ValueError('制动起点前后数据不足')
    sequence = before + [r for r in after if r['t']>before[-1]['t']]
    if len({(r['frame'], r['child']) for r in sequence}) != 1:
        raise ValueError('制动窗口内坐标系发生变化')
    for a,b in zip(sequence, sequence[1:]):
        if not 0 < b['stamp']-a['stamp'] <= args.max_gap or not 0 < b['t']-a['t'] <= args.max_gap:
            raise ValueError('时间戳重复/回退/数据间断，不能计算制动证据')
    if args.event_s-before[-1]['t']>args.max_gap or after[0]['t']-args.event_s>args.max_gap:
        raise ValueError('事件附近没有足够新鲜的里程计')
    initial = before[-1]
    if initial['v'] <= args.stop_speed and initial['w'] <= args.stop_yaw_rate:
        raise ValueError('事件前机器人已低于停止阈值，不能作为制动试验')
    settle = None
    for i,r in enumerate(after):
        if r['v']>args.stop_speed or r['w']>args.stop_yaw_rate:
            continue
        j=i
        while j<len(after) and after[j]['v']<=args.stop_speed and after[j]['w']<=args.stop_yaw_rate:
            if after[j]['t']-r['t']>=args.settle:
                settle=(i,j); break
            j+=1
        if settle:
            break
    if settle is None:
        raise ValueError('窗口内未连续稳定停车；延长录制或调查失败原因')
    stop_i, confirm_i = settle
    path = [initial] + [r for r in after[:confirm_i+1] if r['t']>initial['t']]
    distance = sum(math.hypot(b['x']-a['x'], b['y']-a['y']) for a,b in zip(path,path[1:]))
    rotation = sum(abs(wrap(b['yaw']-a['yaw'])) for a,b in zip(path,path[1:]))
    report = {'status': 'measurement_only_not_release', 'event_s': args.event_s,
              'event_source': args.event_source, 'initial_speed_m_s': initial['v'],
              'initial_angular_speed_rad_s': initial['w'],
              'stop_threshold_reached_s': after[stop_i]['t']-args.event_s,
              'settle_confirmed_s': after[confirm_i]['t']-args.event_s,
              'travel_to_settle_confirmation_m': distance, 'absolute_rotation_to_confirmation_rad': rotation,
              'stop_speed_threshold_m_s': args.stop_speed, 'stop_yaw_rate_threshold_rad_s': args.stop_yaw_rate,
              'equivalent_deceleration_m_s2': initial['v']**2/(2*distance) if distance>0 else None,
              'note': '里程计估计，含事件采样边界和静止噪声；等效减速度包含响应过程，不能直接当物理减速度。用独立位姿/视频复核滑移、响应延迟及定位跳变。'}
    write(args.output, report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


def arrival(args):
    results = []
    with Path(args.csv).open(newline='') as f:
        for r in csv.DictReader(f):
            if not r['trial'].strip() or not r['reference'].strip():
                raise ValueError('每行必须填写 trial 和独立测量 reference')
            n = {k: finite(v) for k,v in r.items() if k not in ('trial','reference')}
            if n['position_uncertainty_m']<0 or n['yaw_uncertainty_deg']<0:
                raise ValueError('不确定度必须非负')
            xy = math.hypot(n['actual_x_m']-n['goal_x_m'], n['actual_y_m']-n['goal_y_m'])
            yaw = abs(math.degrees(wrap(math.radians(n['actual_yaw_deg']-n['goal_yaw_deg']))))
            results.append({'trial': r['trial'], 'reference': r['reference'], 'xy_m': xy,
                            'yaw_deg': yaw, 'within_limits_with_uncertainty':
                            xy+n['position_uncertainty_m']<=.03 and yaw+n['yaw_uncertainty_deg']<=1.5})
    if not results:
        raise ValueError('CSV 没有测量数据')
    write(args.output, {'trials': results, 'all_within_limits_with_uncertainty':
                       all(r['within_limits_with_uncertainty'] for r in results),
                       'hardware_validated': False})
    print(f'{len(results)} 次；含测量不确定度的达标次数：{sum(r["within_limits_with_uncertainty"] for r in results)}')


def inspect(args):
    rows = load_odom(args.run, args.odom_topic)
    acceleration, angular_acceleration, invalid = [], [], 0
    for a,b in zip(rows, rows[1:]):
        dt = b['stamp']-a['stamp']
        if not 0 < dt <= .2 or (a['frame'],a['child']) != (b['frame'],b['child']):
            invalid += 1
            continue
        acceleration.append(abs(b['v']-a['v'])/dt)
        angular_acceleration.append(abs(b['w']-a['w'])/dt)
    scans, zero_events, previous = {}, [], {}
    with (Path(args.run)/'messages.jsonl').open() as f:
        for line in f:
            e=json.loads(line); m=e['msg']; topic=e['topic']
            if 'ranges' in m:
                values=[float(v) for v in m['ranges']]
                count=len(values)
                if count:
                    finite_count=sum(math.isfinite(v) and m['range_min']<=v<=m['range_max'] for v in values)
                    no_return=sum(v==math.inf for v in values)
                    scans.setdefault(topic, []).append({'valid_fraction': (finite_count+no_return)/count,
                                                        'finite_return_fraction': finite_count/count})
            if 'linear' in m and 'angular' in m:
                speed=math.hypot(float(m['linear']['x']),float(m['linear']['y']))
                zero=speed<1e-6 and abs(float(m['angular']['z']))<1e-6
                if zero and previous.get(topic) is False:
                    zero_events.append({'topic':topic,'t_s':e['t_s'],
                                        'note':'subscriber receive time, not actuator time'})
                previous[topic]=zero
    report={'odom_samples':len(rows), 'invalid_derivative_intervals':invalid,
            'speed_p95_m_s':percentile([r['v'] for r in rows],.95),
            'speed_max_m_s':max((r['v'] for r in rows),default=None),
            'angular_speed_max_rad_s':max((r['w'] for r in rows),default=None),
            'speed_magnitude_derivative_abs_p95_m_s2':percentile(acceleration,.95),
            'angular_speed_magnitude_derivative_abs_p95_rad_s2':percentile(angular_acceleration,.95),
            'scan':{t:{'frames':len(v),'min_valid_fraction':min(x['valid_fraction'] for x in v),
                       'min_finite_return_fraction':min(x['finite_return_fraction'] for x in v)} for t,v in scans.items()},
            'command_nonzero_to_zero_events':zero_events,
            'manual_markers':[read(p) for p in sorted(Path(args.run).glob('mark_*.json'))],
            'limitations':['导数为速度模长变化，不代表完整矢量加速度/jerk；保留原始消息供分轴分析。',
                           '有效射线包含正无穷；比例不能证明物理覆盖或盲区合格。',
                           '里程计并非独立真值；没有自动判定硬件通过。']}
    write(args.output, report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    s=sub.add_parser('init'); s.add_argument('directory'); s.add_argument('--template', default=str(TEMPLATE)); s.set_defaults(func=init)
    s=sub.add_parser('record'); s.add_argument('directory'); s.add_argument('--item', choices=ITEMS, required=True)
    s.add_argument('--duration', type=positive, default=60); s.add_argument('--environment', choices=('hardware','simulation'), required=True); s.set_defaults(func=record)
    s=sub.add_parser('mark'); s.add_argument('run'); s.add_argument('label'); s.add_argument('--notes', default=''); s.set_defaults(func=mark)
    s=sub.add_parser('brake'); s.add_argument('run'); s.add_argument('--event-s', type=finite, required=True)
    s.add_argument('--event-source', required=True, help='例如 final cmd zero / driver last command / manual estimate')
    s.add_argument('--odom-topic', default='/odom'); s.add_argument('--window', type=positive, default=10)
    s.add_argument('--stop-speed', type=positive, default=.01); s.add_argument('--stop-yaw-rate', type=positive, default=.02)
    s.add_argument('--settle', type=positive, default=1); s.add_argument('--max-gap', type=positive, default=.2)
    s.add_argument('--output', required=True); s.set_defaults(func=brake)
    s=sub.add_parser('inspect'); s.add_argument('run'); s.add_argument('--odom-topic', default='/odom'); s.add_argument('--output', required=True); s.set_defaults(func=inspect)
    s=sub.add_parser('arrival'); s.add_argument('csv'); s.add_argument('--output', required=True); s.set_defaults(func=arrival)
    args=p.parse_args()
    try:
        return args.func(args) or 0
    except (ValueError, OSError, KeyError, ImportError) as exc:
        print(f'失败：{exc}', file=sys.stderr); return 2


if __name__ == '__main__':
    sys.exit(main())
