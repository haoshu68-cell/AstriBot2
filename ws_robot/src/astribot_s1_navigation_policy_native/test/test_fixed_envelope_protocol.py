"""Isolated ROS wire checks for both fixed_v2 coordinator implementations (domain 116)."""
import copy
import math
import os
from pathlib import Path
import subprocess
import sys

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from astribot_navigation_msgs.msg import RobotEnvelope, RobotGeometryState, ArmHoldStatus, EnvelopeApplyStatus, NavigationEnvelopeV2
from astribot_navigation_msgs.srv import SetRobotEnvelope, SetFixedEnvelope, ReserveArmMotion
from test_envelope_protocol import REFERENCE_ROOT, Protocol, POLICY, PYTHON_MAIN, stamp
from test_fixed_envelope_direct import fixture, CONSUMERS, NOW


class FixedProtocol(Protocol):
    def start(self,impl,binary,log_path):
        self.ns=NOW;self.impl=impl;self.events=[];self.v2=[]
        self.env=dict(os.environ,ROS_DOMAIN_ID='116',ROS_LOCALHOST_ONLY='1')
        self.env['PYTHONPATH']=str(REFERENCE_ROOT)+os.pathsep+str(POLICY)+os.pathsep+self.env.get('PYTHONPATH','')
        command=[binary] if impl=='cpp' else [sys.executable,'-c',PYTHON_MAIN]
        command+=['--ros-args','-p','use_sim_time:=true','-p',f'profile:={POLICY / "config/simulation.json"}',
                  '-p','navigation_geometry_mode:=fixed_v2']
        self.log=log_path.open('w');self.proc=subprocess.Popen(command,env=self.env,stdout=self.log,stderr=subprocess.STDOUT)
        os.environ['ROS_LOCALHOST_ONLY']='1'
        self.context=Context();rclpy.init(context=self.context,domain_id=116)
        self.node=Node('fixed_envelope_protocol',context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.node.create_subscription(RobotEnvelope,'/navigation/robot_envelope',self.events.append,100)
        self.node.create_subscription(NavigationEnvelopeV2,'/navigation/envelope_v2',self.v2.append,100)
        self.clock=self.node.create_publisher(Clock,'/clock',10)
        self.odom=self.node.create_publisher(Odometry,'/odom',10)
        self.state_pub=self.node.create_publisher(RobotGeometryState,'/navigation/geometry_state',10)
        self.hold_pub=self.node.create_publisher(ArmHoldStatus,'/navigation/arm_hold',10)
        self.acks=self.node.create_publisher(EnvelopeApplyStatus,'/navigation/envelope_applied',20)
        self.client=self.node.create_client(SetFixedEnvelope,'/navigation/set_fixed_envelope')
        self.legacy_client=self.node.create_client(SetRobotEnvelope,'/navigation/set_robot_envelope')
        self.reserve_client=self.node.create_client(ReserveArmMotion,'/navigation/reserve_arm_motion')
        self.until(self.client.service_is_ready,8)
        child=Path(f'/proc/{self.proc.pid}/environ').read_bytes().split(b'\0')
        assert b'ROS_DOMAIN_ID=116' in child and b'ROS_LOCALHOST_ONLY=1' in child
        self.until(lambda:all(pub.get_subscription_count()>0 for pub in
            (self.clock,self.state_pub,self.odom,self.hold_pub,self.acks)))
        self.until(lambda:self.node.count_publishers('/navigation/robot_envelope')>0)
        self.clock.publish(Clock(clock=stamp(self.ns)));self.drain(.1)
        self.state,self.hold,self.req=fixture()
        # Discovery precedes actual DDS delivery. Repeat only the setup sample
        # until a source-stamped response proves the child's ROS clock is set.
        def initial_clock_observed():
            self.clock.publish(Clock(clock=stamp(self.ns)))
            self.state_pub.publish(self.state)
            return bool(self.events) and self.events[-1].stamp==stamp(self.ns)
        self.until(initial_clock_observed)
        self.drain(.05)

    def call(self,client,request):
        future=client.call_async(request);self.until(future.done);return future.result()

    def seed(self):
        self.state_pub.publish(self.state);self.hold_pub.publish(self.hold);self.stopped();self.drain()

    def propose(self):
        self.seed();result=self.call(self.client,self.req)
        assert result.accepted,result.reason
        self.flush();assert self.v2[-1].epoch==result.epoch
        return result

    def flush(self):
        old=len(self.events);self.state_pub.publish(self.state)
        self.until(lambda:len(self.events)>old)
        self.drain(.03)
        return self.v2[-1] if self.v2 else None

    def ack_v2(self,name,applied=True):
        e=self.v2[-1]
        a=EnvelopeApplyStatus(coordinator_session_id=e.coordinator_session_id,consumer_id=name,
            envelope_epoch=e.epoch,installed_geometry_hash=e.installed_geometry_hash,applied=applied)
        a.header.stamp=stamp(self.ns);self.acks.publish(a);self.drain(.03)

    def ready(self):
        self.propose()
        for name in sorted(CONSUMERS): self.ack_v2(name)
        assert self.flush().navigation_allowed
        assert not self.events[-1].transport_ready


@pytest.fixture(params=['python','cpp'])
def protocol(request,tmp_path):
    binary=os.environ.get('FIXED_ENVELOPE_CPP_BINARY','/tmp/astribot_fixed_v2_build/fixed_envelope_cpp')
    if request.param=='cpp':assert Path(binary).is_file(),'direct C++ node must be built'
    instance=FixedProtocol(request.param,binary,tmp_path/'session.log')
    try:yield instance
    finally:instance.close()


def test_initial_legacy_hold_confirmable_and_legacy_grant_rejected(protocol):
    p=protocol
    r=p.call(p.legacy_client,SetRobotEnvelope.Request(envelope=RobotEnvelope(transport_ready=False)))
    assert r.accepted
    p.until(lambda:p.events[-1].epoch==r.epoch)
    assert not p.events[-1].transport_ready
    r=p.call(p.legacy_client,SetRobotEnvelope.Request(envelope=RobotEnvelope(transport_ready=True)))
    assert not r.accepted and r.reason=='FIXED_V2_REQUIRES_GEOMETRY_AND_HOLD'
    r=p.call(p.reserve_client,ReserveArmMotion.Request())
    assert not r.accepted and r.reason=='RESERVED_ARM_MOTION_NOT_ENABLED'


def test_all_consumers_required_negative_ack_revokes(protocol):
    p=protocol;p.ready();p.ack_v2('controller',False)
    assert not p.flush().navigation_allowed
    assert p.v2[-1].reason=='WAITING_FOR:controller'


def test_geometry_callback_relays_new_original_source_deadline(protocol):
    p=protocol;p.ready();p.ns+=250_000_000
    p.clock.publish(Clock(clock=stamp(p.ns)));p.drain()
    p.state.sequence=2;p.state.header.stamp=stamp(p.ns);p.state.joint_source_stamps=[stamp(p.ns)]
    p.state.valid_until=stamp(p.ns+300_000_000);p.hold.header.stamp=stamp(p.ns)
    p.hold_pub.publish(p.hold);p.drain()
    latest=p.flush()
    assert latest.navigation_allowed
    assert latest.valid_until==p.state.valid_until and latest.source_state_sequence==2


def test_source_expiry_cannot_be_renewed_by_heartbeat(protocol):
    p=protocol;p.ready();p.ns+=300_000_000
    p.clock.publish(Clock(clock=stamp(p.ns)));p.drain()
    assert not p.flush().navigation_allowed
    assert p.v2[-1].reason=='GEOMETRY_EXPIRED'


def test_posture_drift_latches_until_new_proposal(protocol):
    p=protocol;p.ready();p.state.sequence=2;p.state.joints.position=[.004]
    assert not p.flush().navigation_allowed
    assert p.v2[-1].reason=='FIXED_POSTURE_LEFT_RESERVATION'
    p.state.sequence=3;p.state.joints.position=[0.]
    assert not p.flush().navigation_allowed


@pytest.mark.parametrize('source,vx',[(NOW-300_000_001,0.),(NOW+1,0.),(NOW,.0200001),(NOW,math.nan)])
def test_motion_request_needs_fresh_finite_stopped_odom(protocol,source,vx):
    p=protocol;p.seed();p.stopped(source=source,vx=vx)
    result=p.call(p.client,p.req)
    assert not result.accepted and result.reason=='ROBOT_MUST_BE_STOPPED_WITH_FRESH_ODOMETRY'
