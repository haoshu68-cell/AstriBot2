#!/usr/bin/env python3
"""Supervise hardware sensors, manual navigation, or autonomous exploration."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from uuid import uuid4

from robot_task_control import identity, select_records, shutdown, table
import hardware_sensors


def input_failure(report_path, returncode):
    """Describe the existing probe result without running another check."""
    try:
        report = json.loads(report_path.read_text())
        errors = report.get('errors', []) if isinstance(report, dict) else []
        if isinstance(errors, list):
            details = '; '.join(str(error) for error in errors if error)
            if details:
                return details
    except (OSError, ValueError):
        pass
    return f'输入检查程序退出码 {returncode}，未返回具体原因；见 {report_path.parent/"input_check.stderr.log"}'


def commands(root, autonomous=False):
    config = hardware_sensors.load_config(root)
    if autonomous and not config.get('previous_map') and not config.get('map_name'):
        config['map_name'] = 'explore_' + time.strftime('%Y%m%d_%H%M%S') + '_' + uuid4().hex[:8]
    return {
        **hardware_sensors.commands(root, config),
        'mapping_session': ['ros2', 'run', 'astribot_s1_exploration', 'mapping_session_node', '--ros-args',
                            '--params-file', str(root/'tools/robot/config/deployed_exploration.yaml'),
                            '-p', 'use_sim_time:=false'],
        'navigation': ['ros2', 'launch', 'astribot_s1_navigation', 'navigation.launch.py',
                       'use_sim_time:=false', 'arrival_precision_profile:=hardware',
                       'navigation_policy_stage:=off', 'controller_plugin:=mppi',
                       'map_topic:=/map', 'map_transient_local:=true', 'scan_topic:=/scan_from_cloud',
                       'enable_posture_monitor:=false', 'enable_arm_chassis_coupling:=true',
                       'max_linear_speed:=0.35', 'autostart:=true'],
        'bridge': ['ros2', 'run', 'astribot_trajectory_bridge', 'bridge_container', '--ros-args',
                   '--params-file', str(root/'tools/robot/config/deployed_chassis.yaml')],
        'exploration': ['ros2', 'run', 'astribot_s1_exploration', 'exploration_coordinator_node', '--ros-args',
                        '--params-file', str(root/'ws_robot/install/astribot_s1_exploration/share/astribot_s1_exploration/config/exploration_coordinator_params.yaml'),
                        '--params-file', str(root/'tools/robot/config/deployed_exploration.yaml')],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='只打印启动顺序，不打开 SDK 或启动 ROS 节点')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--sensors-only', action='store_true', help='重启雷达/SLAM/感知链并持续监控，不启动导航或底盘指令桥')
    modes.add_argument('--navigation-only', action='store_true', help='启动并使能导航，等待人工目标，不启动自主探索')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    sensor_config = hardware_sensors.load_config(root)
    cmds = commands(root, autonomous=not args.sensors_only and not args.navigation_only)
    if args.sensors_only:
        cmds = {key: value for key, value in cmds.items() if key in hardware_sensors.SENSOR_ROLES}
    elif args.navigation_only:
        cmds.pop('exploration')
        cmds.pop('mapping_session')
    suffix = 'sensors' if args.sensors_only else 'precision' if args.navigation_only else 'explore'
    mode = {'sensors': 'hardware_sensors', 'precision': 'manual_precision', 'explore': 'autonomous_exploration'}[suffix]
    if args.dry_run:
        print(json.dumps({'mode': mode, 'commands': cmds,
                          'order': ['stop old navigation and sensor instances', 'lidar + actual data check',
                                    'manufacturer joint states + robot model',
                                    'SLAM + perception + fresh odom/map', 'input/ownership check'] +
                                   ([] if args.sensors_only else ['navigation + disabled bridge',
                                    '7 lifecycle nodes active', 'enable chassis',
                                    'wait for manual goal' if args.navigation_only else 'exploration'])}, indent=2, ensure_ascii=False))
        return 0
    from astribot_logging.output import ProcessOutput, SessionLog
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from lifecycle_msgs.srv import GetState
    from std_srvs.srv import SetBool
    from std_msgs.msg import String
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from tf2_ros import Buffer, TransformListener

    logs = Path.home()/'.ros/log/astribot/hardware'
    logs.mkdir(parents=True, exist_ok=True)
    lock = (logs/'.hardware-start.lock').open('w')
    try: fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('另一条启动/重启命令正在执行，请等待其完成。', file=sys.stderr); return 1
    run = Path(tempfile.mkdtemp(prefix=suffix+'_'+time.strftime('%Y%m%d_%H%M%S')+'_', dir=logs))
    os.environ.update(ASTRIBOT_TASK_SESSION=str(run), ASTRIBOT_LOG_DIR=str(run),
                      ROS_LOG_DIR=str(run), ASTRIBOT_LOG_CAPTURE='1')
    link = logs.parent/('latest_hardware_'+suffix)
    temp = link.with_name('.'+link.name+'_'+str(os.getpid()))
    temp.symlink_to(run); os.replace(temp, link)
    journal = SessionLog(run/'session.log', 'supervisor')
    meta = {'mode': mode, 'project': str(root), 'state': 'checking',
            'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'supervisor': dict(identity(os.getpid()), role='supervisor'), 'processes': []}
    stop_requested = False
    map_save_state = "IDLE"
    children = {}
    outputs = []
    node = None
    last_remember = 0.0

    def save():
        path = run/'session.json.tmp'
        path.write_text(json.dumps(meta, ensure_ascii=False, indent=2)+'\n'); path.replace(run/'session.json')

    def announce(message):
        journal.write(message); print(message, flush=True)

    def request_stop(*_):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    save(); announce(f'{mode} 会话 {run}\n一键关停: bash {root}/tools/robot/stop_robot_tasks.sh')

    def check_alive():
        if stop_requested or (run/'STOP').exists(): raise InterruptedError('收到关停请求')
        failed = {role: p.returncode for role, p in children.items() if p.poll() is not None}
        if failed: raise RuntimeError('任务子进程退出: '+str(failed))
        if any(out.error for out in outputs): raise RuntimeError('日志写入失败')

    def spin(seconds=0.1):
        check_alive()
        if node: rclpy.spin_once(node, timeout_sec=seconds)
        else: time.sleep(seconds)

    def spawn(role):
        child = subprocess.Popen(cmds[role], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
        children[role] = child
        outputs.append(ProcessOutput(child.stdout, run/'session.log', role))
        record = identity(child.pid)
        if record is None: raise RuntimeError(f'{role} 启动后立即退出: {child.poll()}')
        meta['processes'].append(dict(record, role=role)); save()

    def remember(force=False):
        nonlocal last_remember
        if not force and time.monotonic()-last_remember < 1.0: return
        last_remember = time.monotonic()
        # Keep descendants' identities even if a launch parent later exits unexpectedly.
        known = {row['pid']: row for row in meta['processes']}
        current = table()
        while True:
            added = {pid: dict(row, role=known[row['ppid']]['role']) for pid, row in current.items()
                     if pid not in known and row['ppid'] in known and row['ppid'] in current
                     and current[row['ppid']]['start'] == known[row['ppid']]['start']}
            if not added: break
            known.update(added)
        meta['processes'] = list(known.values()); save()

    def check_inputs():
        deadline = time.monotonic()+sensor_config['navigation_inputs_timeout_sec']
        attempt = 0
        while True:
            check_alive()
            attempt += 1
            with (run/'input_check.json').open('w') as output, (run/'input_check.stderr.log').open('a') as errors:
                probe = subprocess.Popen([sys.executable, str(root/'tools/robot/hardware_readiness.py'),
                                          '--require-idle-navigation', '--require-idle-bridge'], stdout=output,
                                         stderr=errors, start_new_session=True)
                record = identity(probe.pid)
                if record: meta['processes'].append(dict(record, role='probe')); save()
                probe_deadline = time.monotonic()+20
                while probe.poll() is None:
                    if time.monotonic() > probe_deadline:
                        raise RuntimeError(f'输入检查进程超时；见 {run/"input_check.stderr.log"}')
                    remember(); spin()
            if probe.returncode == 0: return
            (run/f'input_check_attempt_{attempt}.json').write_text((run/'input_check.json').read_text())
            reason = input_failure(run/'input_check.json', probe.returncode)
            if time.monotonic() >= deadline:
                raise RuntimeError(f'输入就绪等待超时：{reason}；报告 {run/"input_check.json"}')
            announce(f'等待输入就绪，第 {attempt} 次检查未通过：{reason}；报告 {run/"input_check.json"}')
            until = time.monotonic()+2
            while time.monotonic() < until: remember(); spin()

    result = 1
    try:
        hardware_sensors.validate_files(root, sensor_config)
        # Stop motion owners before resetting their localization input.
        meta['state'] = 'stopping_previous'; save()
        old = select_records(domain=os.environ.get('ROS_DOMAIN_ID', '25'))
        announce(f'先关停已识别的旧任务及感知实例，共 {len(old)} 个进程。')
        restart_result = shutdown(old)
        (run/'restart_stop_result.json').write_text(json.dumps(restart_result, ensure_ascii=False, indent=2)+'\n')
        if not restart_result['tasks_closed']:
            raise RuntimeError('旧任务未完全退出，拒绝叠加启动')
        spin(0.3)
        remaining = select_records(domain=os.environ.get('ROS_DOMAIN_ID', '25'))
        if remaining:
            raise RuntimeError('发现残留或自动重启的旧实例: '+str(sorted(remaining)))
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('hardware_exploration_supervisor')
        buffer = Buffer(); listener = TransformListener(buffer, node)
        lidar = hardware_sensors.LidarReadiness(node, sensor_config)
        try:
            spawn('lidar'); meta['state'] = 'waiting_lidar'; save()
            deadline = time.monotonic()+sensor_config['lidar_ready_timeout_sec']
            next_report = 0.0
            while True:
                if time.monotonic() >= next_report:
                    report = lidar.report()
                    (run/'lidar_check.json').write_text(json.dumps(report, indent=2)+'\n')
                    if report['ready']: break
                    next_report = time.monotonic()+0.2
                if time.monotonic() > deadline: raise RuntimeError('雷达未输出新鲜且唯一的点云/IMU，见 lidar_check.json')
                remember(); spin(0.05)
        finally:
            lidar.close()
        # Reuse external model/state publishers; manage only nodes started by this session.
        names = {name for name, _ in node.get_node_names_and_namespaces()}
        use_rsp = 'robot_state_publisher' not in names
        use_bridge = not node.get_publishers_info_by_topic('/joint_states')
        if use_rsp or use_bridge:
            cmds['model'] += ['use_robot_state_publisher:='+str(use_rsp).lower(),
                             'use_state_bridge:='+str(use_bridge).lower()]
            spawn('model')
        for role in ('odom', 'slam', 'perception'):
            spawn(role)
        meta['state'] = 'waiting_inputs'; save()
        check_inputs()
        measured = json.loads((run/'input_check.json').read_text())
        if measured['topics']['map']['frame'] != 'map':
            raise RuntimeError('统一 SLAM 必须直接输出 map 坐标系栅格')
        if args.sensors_only:
            meta['state'] = 'running'; save()
            fcntl.flock(lock, fcntl.LOCK_UN)
            announce('雷达、SLAM 和感知链已就绪；未启动导航或底盘指令桥；SDK 仅用于读取状态/里程计。')
            while True:
                remember(); spin(0.2)
        spawn('navigation'); spawn('bridge')
        meta['state'] = 'waiting_ready'; save()
        servers = ['controller_server', 'planner_server', 'smoother_server', 'behavior_server',
                   'bt_navigator', 'waypoint_follower', 'velocity_smoother']
        clients = {name: node.create_client(GetState, '/'+name+'/get_state') for name in servers}
        active = set(); pending = {}; deadline = time.monotonic()+90
        while len(active) < len(servers):
            if time.monotonic()>deadline: raise RuntimeError('Nav2 未在 90 秒内全部 active')
            for name, client in clients.items():
                if name in active: continue
                if name in pending and pending[name].done():
                    response = pending.pop(name).result()
                    if response and response.current_state.id == 3: active.add(name)
                if name not in active and name not in pending and client.service_is_ready():
                    pending[name] = client.call_async(GetState.Request())
            remember(); spin(0.15)
        enable = node.create_client(SetBool, '/chassis_cmd_bridge/enable')
        deadline = time.monotonic()+60
        while not enable.service_is_ready():
            if time.monotonic()>deadline: raise RuntimeError('底盘桥接未就绪；检查 SDK 控制权和反馈')
            remember(); spin()
        request = SetBool.Request(); request.data = True
        waiting_pose = False
        while True:
            future = enable.call_async(request)
            response_deadline = min(deadline, time.monotonic()+5)
            while not future.done():
                if time.monotonic()>response_deadline: raise RuntimeError('底盘使能没有返回')
                remember(); spin()
            response = future.result()
            if response and response.success: break
            if (not response or response.message != '位姿源不可用且 require_slam_to_enable=true'
                    or time.monotonic() >= deadline):
                raise RuntimeError('底盘使能被拒绝: '+str(response))
            if not waiting_pose:
                announce('桥接服务已出现，等待其 TF 缓存收到定位后重试使能。')
                waiting_pose = True
            until = time.monotonic()+0.5
            while time.monotonic()<until: remember(); spin()
        if args.navigation_only:
            meta['state']='running';save()
            fcntl.flock(lock, fcntl.LOCK_UN)
            announce('到位精度测试已就绪并使能：等待 RViz 或单目标脚本，不自动选择探索目标。')
            while True:
                remember();spin(0.2)
        def mapping_status(msg):
            nonlocal map_save_state
            try:
                status = json.loads(msg.data)
                state = status['state']
            except (ValueError, KeyError, TypeError):
                return
            if state != map_save_state:
                announce('建图保存状态: ' + msg.data)
            map_save_state = state
            meta['mapping_session'] = status
        node.create_subscription(String, '/mapping_session/status', mapping_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        spawn('mapping_session')
        spawn('exploration'); meta['state']='running'; save()
        fcntl.flock(lock, fcntl.LOCK_UN)
        announce('自主探索已启动：自动选择前沿目标，沿用当前跟踪和到位控制。')
        while map_save_state != "SAVED":
            remember(); spin(0.2)
        announce('地图文件和 manifest 已保存，正在停车并结束本次任务。'); result=0
    except InterruptedError as exc:
        announce(str(exc)); result=0
    except Exception as exc:
        meta['error']=str(exc); announce('启动/运行失败: '+str(exc))
    finally:
        remember(force=True); meta['state']='stopping'; save()
        records=select_records(run)
        stop_result=shutdown(records)
        (run/'stop_result.json').write_text(json.dumps(stop_result, ensure_ascii=False, indent=2)+'\n')
        for child in children.values():
            if child.poll() is not None: child.wait()
        for output in outputs: output.thread.join(timeout=1)
        if node: node.destroy_node(); rclpy.shutdown()
        meta['state']='stopped' if stop_result['tasks_closed'] else 'stop_failed'
        meta['feedback_stopped']=stop_result['feedback_stopped']; save()
        if stop_result['feedback_stopped'] is False:
            announce('关停前未获得可靠停稳反馈；请现场确认，必要时使用独立急停。')
            result=2
        if not stop_result['tasks_closed']: result=1
        announce('记录: '+str(run/'session.json'))
    return result


if __name__ == '__main__':
    raise SystemExit(main())
