#!/usr/bin/env python3
"""Exercise the existing legacy P4/P5 stack without a fixed-v2 hold handshake."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from run_corridor_probe import wall_clearance
from run_regression import stop_owned


def assess(case, records, navigation):
    geometry = [r for r in records if r['kind'] == 'geometry']
    stamps = [r['stamp_ns'] * 1e-9 for r in geometry]
    policy = [r for r in records if r['kind'] == 'policy']
    continuous = (len(stamps) > 20 and all(0 < b-a <= .15 for a, b in zip(stamps, stamps[1:]))
                  and all(r.get('geometry_valid') and r.get('robot_collision_count', 0) > 0
                          and 'robot_swept_bounds' in r for r in geometry))
    clearances = [wall_clearance(r['robot_swept_bounds'], case['width_m']) for r in geometry] if continuous else []
    crossing = [r for r in geometry if -.6 <= r['robot'][0] <= .6]
    checks = dict(
        precise_arrival=len(navigation) == 1 and navigation[0]['passed'],
        continuous_full_body_geometry=continuous,
        full_body_clearance=bool(clearances) and min(clearances) >= .08,
        crossed_inside_walls=(continuous and bool(crossing) and geometry[0]['robot'][0] < -.9
            and geometry[-1]['robot_swept_bounds'][0][0] > .6
            and all(abs(r['robot'][1]) < case['width_m']/2 for r in crossing)),
        corridor_permit_observed=any(r.get('corridor_state') == 'TRANSIT' and r.get('corridor_permit') for r in policy),
        no_policy_failure=bool(policy) and not any(r.get('reason', '').startswith('CORRIDOR_BLOCKED') for r in policy),
        follow_measurement_present=bool(navigation) and navigation[0]['metrics']['follow_samples'] >= 10,
    )
    return dict(checks=checks, failed_checks=[k for k, v in checks.items() if not v],
        scenario_passed=all(checks.values()), verdict='PASS' if all(checks.values()) else 'FAIL',
        minimum_wall_clearance_lower_bound_m=min(clearances) if clearances else None,
        arrivals=navigation, corridor_states=sorted({r.get('corridor_state', '') for r in policy}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=Path, required=True)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--policy', choices=['off'], default='off')
    args = parser.parse_args()
    case = json.loads(args.case.read_text()); session = json.loads(args.session.read_text())
    identity = session.get('isolation', {})
    if (session.get('state') != 'ready' or not identity.get('ASTRIBOT_SIM_INSTANCE')
        or identity.get('ROS_DOMAIN_ID') in (None, '25') or any(os.environ.get(k) != identity.get(k)
        for k in ('ROS_DOMAIN_ID', 'IGN_PARTITION', 'GZ_PARTITION'))):
        raise RuntimeError('Matching owned isolated simulation required')
    owner = Path('/proc')/str(session['supervisor_pid'])
    if not owner.exists() or b'sim_stack_supervisor.py' not in (owner/'cmdline').read_bytes():
        raise RuntimeError('Simulation owner unavailable')
    if case['kind'] != 'legacy_corridor_probe' or not 1.1 <= case['width_m'] <= 1.8:
        raise ValueError('Unsupported legacy corridor fixture')
    root = Path(__file__).resolve().parents[2]
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'case.json').write_text(json.dumps(case, indent=2)+'\n')
    import rclpy
    from rclpy.parameter import Parameter, parameter_value_to_python
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from action_msgs.msg import GoalStatusArray
    from nav2_msgs.srv import GetCostmap
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('legacy_corridor_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
    rows = []; state = {}; active = {}; owned = []; child = capture = None; logs = []
    raw = (args.output/'observations.jsonl').open('w')
    report = dict(case=case['id'], verdict='INFRA_FAILURE', scenario_passed=False, failed_checks=[],
        boundary='Legacy simulation P4/P5, unchanged posture and navigation stack. Not fixed-v2 or hardware acceptance.')
    def record(kind, message):
        row = dict(json.loads(message.data), kind=kind, wall_s=time.time(),
                   ros_s=node.get_clock().now().nanoseconds*1e-9)
        rows.append(row); state[kind] = row; raw.write(json.dumps(row, allow_nan=False)+'\n')
    for kind, topic in [('geometry', '/social_sim/state'), ('policy', '/navigation_policy/state')]:
        node.create_subscription(String, topic, lambda m, k=kind: record(k, m), qos_profile_sensor_data)
    for action in ('navigate_to_pose', 'navigate_through_poses'):
        node.create_subscription(GoalStatusArray, '/'+action+'/_action/status', lambda m, n=action:
            active.update({n:any(s.status in (1, 2, 3) for s in m.status_list)}),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def spin(): rclpy.spin_once(node, timeout_sec=.02)
    def until(predicate, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            spin()
            if predicate(): return
        raise TimeoutError('Legacy fixture readiness deadline')
    def call(client, request):
        if not client.wait_for_service(timeout_sec=5): raise RuntimeError('Service unavailable')
        future = client.call_async(request); until(future.done, 10)
        return future.result()
    def ign(service, request_type, request):
        result = subprocess.run(['ign', 'service', '-s', '/world/default/'+service,
            '--reqtype', 'ignition.msgs.'+request_type, '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2000', '--req', request], capture_output=True, text=True, timeout=4)
        if result.returncode or 'data: true' not in result.stdout:
            raise RuntimeError('Owned fixture '+service+' failed: '+result.stdout[-200:])
    def launch(command, name):
        log = (args.output/(name+'.log')).open('w'); logs.append(log)
        return subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    def interrupted(signum, frame):
        report['interrupted_signal'] = signum
        raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, interrupted)
    grid_client = node.create_client(GetCostmap, '/global_costmap/get_costmap')
    def costs():
        grid = call(grid_client, GetCostmap.Request()).map; m = grid.metadata
        if grid.header.frame_id != 'map' or abs(m.origin.orientation.z) > 1e-6:
            raise RuntimeError('Unexpected grid frame')
        output = []
        for x, y in [(-.3, -case['width_m']/2-.025), (-.3, case['width_m']/2+.025), (0., 0.)]:
            i = math.floor((x-m.origin.position.x)/m.resolution); j = math.floor((y-m.origin.position.y)/m.resolution)
            if not 1 <= i < m.size_x-1 or not 1 <= j < m.size_y-1:
                raise RuntimeError('Fixture outside costmap')
            output.append(max(grid.data[b*m.size_x+a] for a in range(i-1, i+2)
                              for b in range(j-1, j+2)))
        return output
    try:
        until(lambda: 'geometry' in state and 'policy' in state, 15)
        settle = time.monotonic()+2; until(lambda: time.monotonic() >= settle, 5)
        geometry = state['geometry']
        initial = case['initial_xy_m']
        if (any(active.values()) or geometry['people'] or not geometry['geometry_valid']
            or math.dist(geometry['robot'][:2], initial) > .03
            or abs(math.remainder(geometry['robot'][2]-case.get('initial_yaw_rad', 0.), 2*math.pi)) > .03):
            raise RuntimeError('Expected idle empty fixture at declared initial pose')
        client = node.create_client(GetParameters, '/controller_server/get_parameters')
        keys = ['use_sim_time', 'navigation_geometry_mode', 'navigation_policy_stage']
        values = [parameter_value_to_python(v) for v in call(client, GetParameters.Request(names=keys)).values]
        report['controller_configuration'] = dict(zip(keys, values))
        if values != [True, 'legacy', case['navigation_policy']]:
            raise RuntimeError('Live controller does not match legacy fixture configuration')
        if any(v >= 254 for v in costs()): raise RuntimeError('Fixture region already occupied')
        for side in (-1, 1):
            name = f'legacy_corridor_{os.getpid()}_{side+1}'; owned.append(name)
            geometry_xml = '<geometry><box><size>1.2 .1 1.8</size></box></geometry>'
            sdf = (f'<sdf version="1.7"><model name="{name}"><static>true</static>'
                f'<pose>0 {side*(case["width_m"]/2+.05)} .9 0 0 0</pose><link name="body">'
                f'<visual name="wall">{geometry_xml}</visual><collision name="wall">{geometry_xml}</collision>'
                '</link></model></sdf>')
            ign('create', 'EntityFactory', 'sdf: '+json.dumps(sdf)+' allow_renaming: false')
        observed = []
        def wall_observed():
            observed[:] = costs()
            return all(v == 254 for v in observed[:2]) and observed[2] < 254
        until(wall_observed, 15); report['observed_wall_center_costs'] = observed[:]
        route_file = args.output/'route.json'; route_file.write_text(json.dumps(case['route']))
        start = time.time()
        command = [sys.executable, str(root/'tools/run_waypoint_route.py'), '--route', str(route_file),
            '--cycles', '1', '--timeout-clock', 'sim', '--timeout', '180', '--duration', '200',
            '--wall-watchdog', '600', '--output', str(args.output/'navigation')]
        if case.get('through_poses'): command += ['--through-poses']
        child = launch(command, 'navigation')
        deadline = time.monotonic()+620
        while child.poll() is None and time.monotonic() < deadline:
            spin()
            if capture is None and abs(state['geometry']['robot'][0]) < .3:
                (args.output/'during_passage').mkdir()
                capture = launch([sys.executable, str(root/'tools/social_navigation/capture_views.py'),
                    '--output', str(args.output/'during_passage'), '--supervisor', str(session['supervisor_pid'])], 'capture')
        if child.poll() is None: raise TimeoutError('Legacy corridor wall watchdog')
        result_file = args.output/'navigation/results.jsonl'
        navigation = [json.loads(line) for line in result_file.read_text().splitlines()] if result_file.exists() else []
        report.update(assess(case, [r for r in rows if r['wall_s'] >= start], navigation))
        if child.returncode:
            report.update(scenario_passed=False, verdict='FAIL', navigation_returncode=child.returncode)
            status_file = args.output/'navigation/status.json'
            status = json.loads(status_file.read_text()) if status_file.exists() else {}
            if (not navigation and 'goal_index' not in status
                and not (args.output/'navigation/metadata.json').exists()
                and status.get('error') == 'ROS action/service response timeout'):
                report.update(verdict='INFRA_FAILURE',
                    error='Navigation parameter query timed out before goal submission',
                    navigation_status=status)
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = str(error) or type(error).__name__
    finally:
        for process in (child, capture):
            try:
                if process is capture and process is not None:
                    try: process.wait(timeout=20)
                    except subprocess.TimeoutExpired: pass
                stop_owned(process, 35)
            except Exception as error:
                report.update(cleanup_error=str(error), scenario_passed=False, verdict='INFRA_FAILURE')
        report['during_passage_capture'] = 'PENDING_REVIEW' if capture and capture.returncode == 0 else 'CAPTURE_UNAVAILABLE'
        cleanup_errors = []
        for name in owned:
            try:
                for attempt in range(3):
                    try: ign('remove', 'Entity', 'name: '+json.dumps(name)+' type: MODEL')
                    except RuntimeError: pass
                    scene = subprocess.run(['ign', 'service', '-s', '/world/default/scene/info',
                        '--reqtype', 'ignition.msgs.Empty', '--reptype', 'ignition.msgs.Scene',
                        '--timeout', '2000', '--req', ''], capture_output=True, text=True, timeout=4)
                    if scene.returncode or not scene.stdout.strip(): raise RuntimeError('Scene query failed')
                    if not re.search(r'name:\s*"'+re.escape(name)+'"', scene.stdout): break
                else: raise RuntimeError('Own wall remains: '+name)
            except Exception as error: cleanup_errors.append(str(error))
        if cleanup_errors:
            report.update(cleanup_error='; '.join(cleanup_errors), scenario_passed=False, verdict='INFRA_FAILURE')
        report['fixtures_removed'] = not cleanup_errors
        raw.close()
        for log in logs: log.close()
        (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node(); rclpy.shutdown()
    print(json.dumps(report, indent=2))
    return 0 if report['scenario_passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
