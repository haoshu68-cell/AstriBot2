#!/usr/bin/env python3
"""Stop navigation, then SLAM/lidar using captured PID identities; retain robot body drivers."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time

from hardware_sensors import LIDAR_ROOT, SENSOR_ROLES, role as sensor_role

NAV_EXECUTABLES = {
    'controller_server', 'planner_server', 'smoother_server', 'behavior_server',
    'bt_navigator', 'waypoint_follower', 'velocity_smoother', 'jerk_velocity_smoother', 'lifecycle_manager',
    'cmd_vel_body_to_world_node', 'arm_speed_limiter_node', 'arm_chassis_speed_coupling_node',
    'xtalpi_navigation', 'task_arbiter', 'policy_controller', 'final_protection',
    'envelope_coordinator', 'costmap_scan_adapter',
}
GENERATORS = {'loop_route_executor', 'mapping_session_node', 'exploration_coordinator_node', 'frontier_explorer_node', 'run_waypoint_route.py',
              'navigation_precision_check.py'}
BRIDGES = {'bridge_container', 'chassis_cmd_bridge_node'}
LAUNCHES = {'navigation.launch.py', 'xtalpi_navigation_bringup.launch.py',
            'exploration_coordinator.launch.py', 'frontier_explore.launch.py',
            'bridge_bringup.launch.py'}
ROOTS = ('/home/astribot/astribot_projects/', '/home/astribot/Downloads/astribot_sdk_aarch64/',
         '/home/astribot/Downloads/lw_test/astribot_sdk_ros2/')
PROTECTED = {'robot_state_publisher', 'joint_state_publisher', 'static_transform_publisher',
             'tf_to_odom_node.py', 'grid_self_clear_node.py', 'pointcloud_to_laserscan_node',
             'pointcloud_slice_scan_node', 'voxelslam', 'livox_ros_driver2_node'}


def identity(pid):
    try:
        row = Path(f'/proc/{pid}/stat').read_text().rsplit(') ', 1)[1].split()
        return {'pid': int(pid), 'ppid': int(row[1]), 'start': int(row[19]), 'state': row[0]}
    except (OSError, ValueError, IndexError):
        return None


def ancestors():
    result = set()
    pid = os.getpid()
    while pid > 1 and pid not in result:
        result.add(pid)
        item = identity(pid)
        if not item: break
        pid = item['ppid']
    return result


def table():
    result = {}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit(): continue
        try:
            if path.stat().st_uid != os.getuid(): continue
            item = identity(int(path.name))
            if not item or item['state'] == 'Z': continue
            item['argv'] = [os.fsdecode(a) for a in (path/'cmdline').read_bytes().split(b'\0') if a]
            env = dict(v.split(b'=', 1) for v in (path/'environ').read_bytes().split(b'\0') if b'=' in v)
            item['domain'] = env.get(b'ROS_DOMAIN_ID', b'').decode()
            item['prefixes'] = env.get(b'AMENT_PREFIX_PATH', b'').decode()
            item['session'] = env.get(b'ASTRIBOT_TASK_SESSION', b'').decode()
            result[item['pid']] = item
        except OSError: pass
    return result


def role(item):
    kind = sensor_role(item, ROOTS)
    if kind:
        return kind
    args = item['argv']
    if not args or any(a in ('-c', '-lc', '-s') for a in args[:3]): return None
    # Inspect executable/script tokens, never substring-match a shell's command text.
    tokens = args[:2] if Path(args[0]).name.startswith(('python', 'bash')) else args[:1]
    names = {Path(a).name for a in tokens}
    if names & PROTECTED or any(a.startswith('/opt/astribot_ros/') for a in tokens): return None
    owned_path = any(a.startswith(ROOTS) for a in tokens) or any(
        p.startswith(ROOTS) for p in item['prefixes'].split(':'))
    if not owned_path and not any(a.startswith('/home/astribot/s1_tools/') for a in tokens): return None
    if names & GENERATORS: return 'exploration'
    if names & BRIDGES: return 'bridge'
    if names & NAV_EXECUTABLES: return 'navigation'
    if 'hardware_exploration.py' in names: return 'supervisor'
    if any(Path(a).name.split(':=')[-1] in LAUNCHES for a in args[2:] if ':=' not in a):
        if any(Path(a).name in ('ros2', 'roslaunch.py') for a in tokens):
            return 'exploration' if any('explor' in a for a in args[2:4]) else 'navigation'
    if 'rviz2' in names and any('/ws_robot/' in a for a in args): return 'viewer'
    return None


def select_records(session=None, domain='25'):
    current = table()
    excluded = ancestors()
    selected = {}
    if session:
        meta = json.loads((Path(session)/'session.json').read_text())
        if meta['boot_id'] != Path('/proc/sys/kernel/random/boot_id').read_text().strip(): return {}
        for row in [meta['supervisor'], *meta.get('processes', [])]:
            now = current.get(row['pid'])
            if now and now['start'] == row['start'] and row['pid'] not in excluded:
                selected[row['pid']] = dict(now, role=row.get('role', 'navigation'))
    else:
        for pid, row in current.items():
            if (pid not in excluded and (kind := role(row))
                    and (row['domain'] == str(domain) or kind in ('slam', 'lidar'))):
                selected[pid] = dict(row, role=kind)
    while True:
        added = {}
        for pid, row in current.items():
            if pid in selected or pid in excluded or row['ppid'] not in selected: continue
            names = {Path(a).name for a in row['argv'][:2]}
            kind = sensor_role(row, ROOTS)
            if names & PROTECTED and not kind: continue
            if not kind and any(a.startswith('/opt/astribot_ros/') and not a.startswith(LIDAR_ROOT)
                                for a in row['argv'][:2]): continue
            added[pid] = dict(row, role=kind or selected[row['ppid']]['role'])
        if not added: break
        selected.update(added)
    return selected


def live(records):
    return {pid: row for pid, row in records.items()
            if (now := identity(pid)) and now['start'] == row['start'] and now['state'] != 'Z'}


def send_signal(records, sig):
    for pid, row in live(records).items():
        try:
            fd = os.pidfd_open(pid)
            try:
                if pid in live({pid: row}): signal.pidfd_send_signal(fd, sig)
            finally: os.close(fd)
        except ProcessLookupError: pass


class RosStop:
    def __init__(self):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        self.ros = rclpy
        self.owns_context = not rclpy.ok()
        if self.owns_context: rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('astribot_stop_tasks')
        self.zero = Twist()
        self.publishers = [self.node.create_publisher(Twist, t, 10) for t in
                           ('/cmd_vel', '/cmd_vel_nav_body_raw', '/cmd_vel_nav_body', '/cmd_vel_pre_arm_coupling')]
        self.samples = []
        self.last_stamp = 0
        self.node.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        self.requests = {}
        self.clients = []
        self.next_zero = 0.0

    def odom(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp <= self.last_stamp: return
        self.last_stamp = stamp
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.samples.append((time.monotonic(), stamp, math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
                             abs(msg.twist.twist.angular.z), msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))
        self.samples = self.samples[-60:]

    def tick(self, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if time.monotonic() >= self.next_zero:
                for pub in self.publishers: pub.publish(self.zero)
                self.next_zero = time.monotonic()+0.05
            self.ros.spin_once(self.node, timeout_sec=0.04)

    def request_stop(self):
        from std_srvs.srv import Trigger
        from action_msgs.srv import CancelGoal
        self.tick(0.6)
        services = dict(self.node.get_service_names_and_types())
        names = [('/exploration_coordinator_node/pause', Trigger),
                 ('/chassis_cmd_bridge/disable', Trigger), ('/astribot_bridge_container/disable', Trigger)]
        actions = ['navigate_to_pose', 'navigate_through_poses', 'follow_path', 'compute_path_to_pose',
                   'navigation_executor/navigate_to_pose', 'navigation_executor/navigate_through_poses',
                   'exploration/navigate_to_pose', 'route/navigate_to_pose', 'route/navigate_through_poses']
        names += [('/'+a+'/_action/cancel_goal', CancelGoal) for a in actions]
        pending = {}
        for name, kind in names:
            expected = 'std_srvs/srv/Trigger' if kind is Trigger else 'action_msgs/srv/CancelGoal'
            if expected not in services.get(name, []): continue
            client = self.node.create_client(kind, name); self.clients.append(client)
            pending[name] = client.call_async(kind.Request())
        self.tick(1.5)
        for name, future in pending.items():
            response = future.result() if future.done() and not future.exception() else None
            self.requests[name] = (('success='+str(response.success)) if hasattr(response, 'success')
                                    else ('return_code='+str(response.return_code))) if response else 'no response'

    def feedback_stopped(self):
        if len(self.node.get_publishers_info_by_topic('/odom')) != 1: return False
        now = time.monotonic()
        rows = [r for r in self.samples if now-r[0] <= 1.0]
        if len(rows) < 4 or rows[-1][0]-rows[0][0] < 0.5 or now-rows[-1][0] > 0.3: return False
        if not -0.1 <= self.node.get_clock().now().nanoseconds/1e9-rows[-1][1]/1e9 <= 0.5: return False
        x, y, yaw = rows[0][4:]
        return all(all(math.isfinite(v) for v in r[2:]) and r[2] <= 0.01 and r[3] <= 0.02
                   and math.hypot(r[4]-x, r[5]-y) <= 0.005
                   and abs(math.atan2(math.sin(r[6]-yaw), math.cos(r[6]-yaw))) <= 0.01 for r in rows)

    def close(self):
        self.node.destroy_node()
        if self.owns_context: self.ros.shutdown()


def terminate_records(records, tick=None, first_signal=signal.SIGINT):
    for sig, duration in ((first_signal, 5.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 0.5)):
        if sig: send_signal(records, sig)
        deadline = time.monotonic()+duration
        while live(records) and time.monotonic() < deadline:
            if tick: tick(0.1)
            else: time.sleep(0.05)


def shutdown_control(records):
    if not records:
        return {'tasks_closed': True, 'feedback_stopped': None, 'note': '没有匹配的本机运动任务；未判定机器人整体运动状态', 'remaining': []}
    report = {'selected': [{'pid': p, 'role': r['role'], 'argv': r['argv']} for p, r in records.items()]}
    stop = None
    control_tasks = any(r['role'] not in ('probe', 'viewer', 'supervisor') for r in records.values())
    if control_tasks:
        try:
            stop = RosStop()
            stop.request_stop()
            # Snapshot parking before notifying the supervisor, which may stop SLAM/odom.
            deadline = time.monotonic()+3.0
            while not stop.feedback_stopped() and time.monotonic() < deadline:
                stop.tick(0.1)
            report['feedback_stopped'] = stop.feedback_stopped()
        except Exception as exc:
            report['stop_request_error'] = str(exc)
            report['feedback_stopped'] = False
            if stop:
                stop.close()
                stop = None
    for row in records.values():
        if row.get('session'):
            path = Path(row['session'])/'STOP'
            if path.parent.is_dir(): path.touch()
    # Quiesce goal generators/executors first; keep SDK bridge alive for the stop response.
    send_signal({p:r for p,r in records.items() if r['role'] != 'bridge'}, signal.SIGINT)
    if stop: stop.tick(1.0)
    send_signal({p:r for p,r in records.items() if r['role'] == 'bridge'}, signal.SIGINT)
    terminate_records(records, stop.tick if stop else None, first_signal=None)
    if stop:
        report.update(services=stop.requests)
        stop.close()
    else: report.setdefault('feedback_stopped', False if control_tasks else None)
    report['remaining'] = sorted(live(records))
    report['tasks_closed'] = not report['remaining']
    return report


def shutdown(records):
    sensors = {p: r for p, r in records.items() if r['role'] in SENSOR_ROLES}
    control = {p: r for p, r in records.items() if p not in sensors}
    report = shutdown_control(control)
    if not report['tasks_closed']:
        report['sensors_retained'] = '控制任务尚未退出，保留定位和雷达用于停车反馈'
    else:
        # End derived consumers/SLAM first, then release the lidar's UDP ports.
        terminate_records({p: r for p, r in sensors.items() if r['role'] != 'lidar'})
        terminate_records({p: r for p, r in sensors.items() if r['role'] == 'lidar'})
    report['selected'] = [{'pid': p, 'role': r['role'], 'argv': r['argv']} for p, r in records.items()]
    report['remaining'] = sorted(live(records))
    report['tasks_closed'] = not report['remaining']
    report['sensors_closed'] = not live(sensors)
    report['parking_checked_before_sensor_shutdown'] = bool(control) and report['feedback_stopped'] is not None
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='仅显示进程清单，不发停车请求、不发信号')
    parser.add_argument('--session', help='仅操作指定 session.json 中记录的任务')
    args = parser.parse_args()
    selected = select_records(args.session, os.environ.get('ROS_DOMAIN_ID', '25'))
    if args.dry_run:
        print(json.dumps({'dry_run': True, 'targets': list(selected.values())}, ensure_ascii=False, indent=2)); return 0
    result = shutdown(selected)
    if args.session:
        (Path(args.session)/'stop_result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not result['tasks_closed']: return 1
    if result['feedback_stopped'] is False:
        print('任务进程已关停，但无法确认底盘停稳（可能缺少反馈或存在多发布端）；请现场确认，必要时使用独立急停。')
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
