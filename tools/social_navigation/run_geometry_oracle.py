#!/usr/bin/env python3
"""Deliberate overlap in an owned disposable simulation to check the geometry oracle."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',type=Path,required=True)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--policy',choices=('off',),default='off')
    args=parser.parse_args();session=json.loads(args.session.read_text());case=json.loads(args.case.read_text())
    identity=session.get('isolation',{})
    if (session.get('state')!='ready' or not identity.get('ASTRIBOT_SIM_INSTANCE') or
        identity.get('ROS_DOMAIN_ID') in (None,'25') or
        any(os.environ.get(k)!=identity.get(k) for k in ('ROS_DOMAIN_ID','IGN_PARTITION','GZ_PARTITION'))):
        raise RuntimeError('Only the matching disposable isolated simulation is allowed')
    owner=Path('/proc')/str(session['supervisor_pid'])
    if not owner.exists() or b'sim_stack_supervisor.py' not in (owner/'cmdline').read_bytes():
        raise RuntimeError('Simulation owner is unavailable')
    args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from std_msgs.msg import String
    from astribot_navigation_msgs.msg import NavigationExecutionStatus
    rclpy.init();node=rclpy.create_node('geometry_oracle_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    rows=[];active=[]
    node.create_subscription(String,'/social_sim/state',lambda m:rows.append(json.loads(m.data)),qos_profile_sensor_data)
    node.create_subscription(NavigationExecutionStatus,'/navigation/execution_status',lambda m:active.append(m.state),
                             QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    report=dict(case=case['id'],verdict='INFRA_FAILURE',scenario_passed=False,
        boundary='Intentional overlap validates the independent oracle, not obstacle avoidance. Cold restart mandatory afterwards.')
    try:
        spin(3)
        if not rows or not rows[-1]['geometry_valid'] or len(rows[-1]['people'])!=1:
            raise RuntimeError('Expected one valid stationary person')
        if any(s in ('ACCEPTED','EXECUTING','CANCELING') for s in active):raise RuntimeError('Navigation task present')
        before=rows[-1];person=before['people'][0]
        if before['conservative_overlap_steps']!=0 or before['clearance_lower_bound_m']<=0:
            raise RuntimeError('Initial geometry is not clear')
        target=(person['x']-.5,person['y'])
        response=subprocess.run(['ign','service','-s','/world/default/set_pose','--reqtype','ignition.msgs.Pose',
            '--reptype','ignition.msgs.Boolean','--timeout','3000','--req',
            f'name:"astribot_s1",position:{{x:{target[0]},y:{target[1]},z:.15}},orientation:{{w:1}}'],
            capture_output=True,text=True,timeout=5)
        if response.returncode or 'data: true' not in response.stdout:raise RuntimeError('Owned scene rejected test pose')
        end=time.monotonic()+30;detected=None
        while time.monotonic()<end:
            spin(.1)
            # Discard the teleport's swept bound. Require sustained overlap
            # once the robot is stationary at the deliberately overlapping pose.
            window=[r for r in rows if r['stamp_ns']>before['stamp_ns']+500_000_000][-3:]
            if (len(window)==3 and window[-1]['stamp_ns']-window[0]['stamp_ns']>=90_000_000 and
                all(r['geometry_valid'] and r['clearance_lower_bound_m']<0 and
                    math.hypot(r['robot'][0]-target[0],r['robot'][1]-target[1])<.05 for r in window) and
                window[-1]['conservative_overlap_steps']>window[0]['conservative_overlap_steps'] and
                math.hypot(window[-1]['robot'][0]-window[0]['robot'][0],
                           window[-1]['robot'][1]-window[0]['robot'][1])<.005):
                detected=window[-1];break
        checks={'clear_fixture':True,'persistent_overlap_detected':detected is not None}
        report.update(checks=checks,before=before,detected=detected,scenario_passed=all(checks.values()))
        report['verdict']='PASS' if report['scenario_passed'] else 'FAIL'
    except Exception as error:report['error']=str(error)
    finally:
        (args.output/'geometry.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
        (args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True);node.destroy_node();rclpy.shutdown()
    return 0 if report['scenario_passed'] else 1


if __name__=='__main__':raise SystemExit(main())
