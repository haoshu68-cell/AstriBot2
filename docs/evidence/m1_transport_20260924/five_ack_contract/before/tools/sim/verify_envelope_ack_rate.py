#!/usr/bin/env python3
"""Isolated ROS protocol fixture; never used as real hold/geometry evidence."""
import argparse
import json
import os
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from astribot_navigation_msgs.msg import RobotGeometryState, ArmHoldStatus, NavigationEnvelopeV2, EnvelopeApplyStatus, RobotEnvelope
from astribot_navigation_msgs.srv import SetFixedEnvelope


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--executable',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    domain=int(os.environ.get('ROS_DOMAIN_ID','0'))
    assert 180<=domain<=232, 'use a dedicated fixture domain, never the real simulator'
    rclpy.init();node=Node('envelope_ack_rate_fixture',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    assert not node.get_publishers_info_by_topic('/navigation/geometry_state')
    profile=Path('ws_robot/src/astribot_s1_navigation_policy/config/simulation.json').resolve()
    data={'pass':False,'evidence_layer':'isolated_ros_protocol_fixture','domain':domain,'samples':[]}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    log=args.output.with_suffix('.log').open('w')
    process=subprocess.Popen([str(args.executable.resolve()),'--ros-args','-p','use_sim_time:=true','-p','profile:='+str(profile)],stdout=log,stderr=subprocess.STDOUT)
    data['pid']=process.pid
    clock=node.create_publisher(Clock,'/clock',10)
    geometry=node.create_publisher(RobotGeometryState,'/navigation/geometry_state',10)
    holds=node.create_publisher(ArmHoldStatus,'/navigation/arm_hold',10)
    odom=node.create_publisher(Odometry,'/odom',10)
    ack=node.create_publisher(EnvelopeApplyStatus,'/navigation/envelope_applied',50)
    service=node.create_client(SetFixedEnvelope,'/navigation/set_fixed_envelope')
    input_enabled=True
    current=None;negative=False;last_input=0.;start=time.monotonic();sequence=0;sim=10.;g=None
    def stamp(value):
        from builtin_interfaces.msg import Time
        n=int(value*1e9);return Time(sec=n//10**9,nanosec=n%10**9)
    def receive(m):
        nonlocal current
        current=m
        data['samples'].append({'at':time.monotonic(),'allowed':m.navigation_allowed,'reason':m.reason,'epoch':m.epoch})
        for consumer in ('global_costmap','local_costmap','planner','controller','policy','protection'):
            a=EnvelopeApplyStatus();a.header.stamp=m.header.stamp;a.coordinator_session_id=m.coordinator_session_id
            a.consumer_id=consumer;a.envelope_epoch=m.epoch;a.installed_geometry_hash=m.installed_geometry_hash
            a.applied=not (negative and consumer=='controller');a.reason='ISOLATED_PROTOCOL_FIXTURE';ack.publish(a)
    node.create_subscription(NavigationEnvelopeV2,'/navigation/envelope_v2',receive,50)
    def step():
        nonlocal last_input,sequence,sim,g
        now=time.monotonic()
        if input_enabled and now-last_input>=.05:
            sim=10.+now-start;last_input=now;sequence+=1
            clock.publish(Clock(clock=stamp(sim)))
            g=RobotGeometryState();g.header.frame_id='astribot_torso_base';g.header.stamp=stamp(sim);g.valid_until=stamp(sim+.3)
            g.source_id='isolated';g.model_revision='fixture';g.attachment_revision='fixture_empty';g.sequence=sequence
            g.complete=g.attachment_state_confirmed=True;g.height_m=1.7
            g.joints.name=['arm'];g.joints.position=[0.];g.joint_source_stamps=[g.header.stamp];g.joint_position_error_bounds=[.003]
            for name,size in (('physical_footprint',.3),('reserved_footprint',.4)):
                getattr(g,name).points=[Point32(x=x,y=y,z=0.) for x,y in ((-size,-size),(size,-size),(size,size),(-size,size))]
            geometry.publish(g)
            h=ArmHoldStatus();h.header.stamp=stamp(sim);h.owner_id='fixture';h.hold_id='fixture_hold';h.attachment_revision='fixture_empty';h.hold_confirmed=True;h.lease_s=.3;holds.publish(h)
            o=Odometry();o.header.stamp=stamp(sim);o.pose.pose.orientation.w=1.;odom.publish(o)
        rclpy.spin_once(node,timeout_sec=.001)
    def wait(test,timeout):
        end=time.monotonic()+timeout
        while not test():
            if time.monotonic()>end:raise RuntimeError('fixture timeout; last='+str(data['samples'][-1:]))
            step()
    try:
        wait(lambda: service.service_is_ready() and time.monotonic()-start>1.,5.)
        base=json.loads(profile.read_text());limits=RobotEnvelope(frame_id=base['base_frame'],posture_id='fixture',lease_s=.3)
        for name in ('half_length_m','half_width_m','height_m','payload_mass_kg','max_speed_m_s','max_angular_speed_rad_s','max_acceleration_m_s2','brake_deceleration_m_s2'):setattr(limits,name,float(base[name]))
        f=service.call_async(SetFixedEnvelope.Request(request_id='fixture',hold_id='fixture_hold',geometry_sequence=g.sequence,limits=limits))
        wait(f.done,3.);assert f.result().accepted,f.result().reason
        wait(lambda:current is not None and current.navigation_allowed,3.)
        begin=time.monotonic();offset=len(data['samples']);wait(lambda:time.monotonic()-begin>=2.,3.)
        data['positive_publication_hz']=(len(data['samples'])-offset)/(time.monotonic()-begin)
        assert data['positive_publication_hz']<120., 'ACK_FEEDBACK_AMPLIFICATION'
        # Freeze ROS time and stop source callbacks: only the ACK callback can
        # publish the transition. This excludes the timer/source fallback path.
        input_enabled=False
        drain=time.monotonic()
        wait(lambda:time.monotonic()-drain>=.1,.2)
        negative=True;begin=time.monotonic()
        # Explicit negative fixture ACK; the other five consumers keep positive heartbeats.
        a=EnvelopeApplyStatus();a.header.stamp=current.header.stamp;a.coordinator_session_id=current.coordinator_session_id;a.consumer_id='controller';a.envelope_epoch=current.epoch;a.installed_geometry_hash=current.installed_geometry_hash;a.applied=False;ack.publish(a)
        wait(lambda:not current.navigation_allowed,.3)
        data['negative_transition_seconds']=time.monotonic()-begin
        assert 'controller' in current.reason
        # Same-stamp positive is ambiguous and must not restore permission.
        negative=False;a.applied=True;ack.publish(a)
        blocked=time.monotonic();wait(lambda:time.monotonic()-blocked>=.05,.2)
        assert not current.navigation_allowed,'same-stamp replay restored permission'
        # Advance clock alone (not geometry/hold); positive evidence is newer.
        sim+=.001;clock.publish(Clock(clock=stamp(sim)))
        wait(lambda:node.get_clock().now().nanoseconds>=int(sim*1e9),.2)
        a.header.stamp=stamp(sim);begin=time.monotonic();ack.publish(a)
        wait(lambda:current.navigation_allowed,.3)
        data['positive_transition_seconds']=time.monotonic()-begin
        clock.publish(Clock(clock=stamp(sim+.31)))
        wait(lambda:not current.navigation_allowed,.5)
        assert current.reason=='GEOMETRY_EXPIRED',current.reason
        data['source_expiry_reason']=current.reason
        data['pass']=True
    except Exception as error:data['error']=str(error)
    finally:
        process.send_signal(2)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=5)
        log.close();data['process_exit']=process.returncode
        args.output.write_text(json.dumps(data,indent=2)+'\n')
        print(json.dumps({k:v for k,v in data.items() if k!='samples'},indent=2))
        node.destroy_node();rclpy.shutdown()
    return 0 if data['pass'] else 1
if __name__=='__main__':raise SystemExit(main())
