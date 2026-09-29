"""Exercise the deployed policy adapter contract without starting a ROS graph."""
import copy
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from astribot_navigation_msgs.msg import NavigationEnvelopeV2
from astribot_s1_navigation_policy.contracts import Stamp, BearingCone, Vec3, Health
from astribot_s1_navigation_policy.sensor_health import SensorHealthRegistry
from astribot_s1_navigation_policy.observer_node import PolicyObserver
from astribot_s1_navigation_policy.profile import Profile
from astribot_s1_navigation_policy.robot_envelope import FIELDS, FixedEnvelopeProfile
from astribot_s1_navigation_policy.path_evidence import PathEvidence, assess_path
from astribot_s1_robot_geometry.polygon import inflate, geometry_hash

PROFILE=Path(__file__).resolve().parents[2]/'ws_robot/src/astribot_s1_navigation_policy/config/simulation.json'

def stamp(ns):return N(sec=ns//10**9,nanosec=ns%10**9)

class LatestPolicyTests(unittest.TestCase):
    def setUp(self):self.profile=Profile.load(PROFILE)

    def test_future_delayed_health_is_valid_and_old_sample_ignored(self):
        registry=SensorHealthRegistry(.3)
        cones=(BearingCone(Vec3(1.,0.,0.),3.14),)
        self.assertTrue(registry.record('scan',Stamp(20,'sim',0),Stamp(10,'sim',0),'base',cones,True,0))
        self.assertEqual(registry.health(Stamp(10**12,'sim',0))[0].health,Health.VALID)
        self.assertFalse(registry.record('scan',Stamp(19,'sim',0),Stamp(30,'sim',0),'base',(),False,0))
        self.assertEqual(registry.records['scan'].stamp.ns,20)
        self.assertFalse(registry.required_valid(Stamp(30,'sim',1)))

    def test_missing_coverage_remains_negative(self):
        registry=SensorHealthRegistry(.3)
        self.assertFalse(registry.required_valid(Stamp(10,'sim',0)))
        registry.record('scan',Stamp(20,'sim',0),Stamp(10,'sim',0),'base',(),True,0)
        self.assertFalse(registry.required_valid(Stamp(30,'sim',0)))

    def test_clock_arrival_order_does_not_clear_data(self):
        node=N(get_clock=lambda:N(now=lambda:N(nanoseconds=9)),last_time=10,clock_id='sim',epoch=2,path=('path',),scan_at=12)
        self.assertEqual(PolicyObserver.stamp(node),Stamp(9,'sim',2))
        self.assertEqual(node.path,('path',));self.assertEqual(node.scan_at,12)

    def test_odom_keeps_newest_source(self):
        node=N(profile=self.profile,odom_lock=threading.Lock(),odom_at=None,robot=None)
        msg=Odometry();msg.header.frame_id=self.profile.tracking_frame;msg.header.stamp.sec=20
        msg.pose.pose.orientation.w=1.;msg.pose.pose.position.x=2.
        PolicyObserver.odom(node,msg)
        msg.header.stamp.sec=19;msg.pose.pose.position.x=9.
        PolicyObserver.odom(node,msg)
        self.assertEqual(node.robot.x,2.);self.assertEqual(node.odom_at,20.)

    def test_scan_mailbox_selects_newest_not_latest_arrival(self):
        messages=[]
        for seconds in (30,29,31,28):
            m=LaserScan();m.header.stamp.sec=seconds;m.header.frame_id='laser';messages.append(m)
        calls=[];selected=[]
        node=N(scan_timing=None,map=object(),scan_lock=threading.Lock(),pending_scans=messages,
               scan_at=27.,profile=self.profile,
               tf=N(can_transform=lambda target,source,when:calls.append(when.nanoseconds) or True),
               process_scan=lambda msg:selected.append(msg.header.stamp.sec))
        PolicyObserver.process_scans(node,Stamp(1,'sim',0))
        self.assertEqual(selected,[31]);self.assertTrue(all(t==0 for t in calls))

    def envelope(self):
        p=self.profile;m=NavigationEnvelopeV2();m.header.frame_id=p.base_frame;m.header.stamp.sec=20
        m.coordinator_session_id='session';m.epoch=1;m.clock_epoch=0;m.hold_id='hold';m.mode=m.FIXED_POSTURE
        m.navigation_allowed=True;m.clearance_m=p.clearance_margin_m+p.payload_extra_margin_m
        m.limits.frame_id=p.base_frame;m.limits.posture_id='hold';m.limits.lease_s=.3;m.limits.transport_ready=True
        for field in FIELDS:setattr(m.limits,field,getattr(p,field))
        reserved=[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]]
        installed=inflate(reserved,m.clearance_m)
        m.reserved_footprint.points=[Point32(x=float(x),y=float(y),z=0.) for x,y in reserved]
        m.installed_footprint.points=[Point32(x=float(x),y=float(y),z=0.) for x,y in installed]
        m.installed_geometry_hash=geometry_hash([(v.x,v.y) for v in m.installed_footprint.points],p.base_frame,m.clearance_m)
        return m

    def test_future_envelope_and_expired_deadline_remain_available(self):
        profile=FixedEnvelopeProfile(self.profile);m=self.envelope()
        self.assertTrue(profile.accept(m,Stamp(10**9,'sim',0)))
        self.assertTrue(profile.ready(Stamp(100*10**9,'sim',0)))
        self.assertTrue(profile.confirms_applied(m,Stamp(1,'sim',0)))
        old=copy.deepcopy(m);old.header.stamp.sec=19;old.navigation_allowed=False
        self.assertFalse(profile.accept(old,Stamp(10**9,'sim',0)))
        self.assertTrue(profile.ready(Stamp(10**9,'sim',0)))
        negative=copy.deepcopy(m);negative.header.stamp.sec=21;negative.navigation_allowed=False
        self.assertTrue(profile.accept(negative,Stamp(10**9,'sim',0)))
        self.assertFalse(profile.ready(Stamp(10**9,'sim',0)))
        self.assertFalse(profile.accept(m,Stamp(10**9,'sim',0)))

    def test_path_evidence_age_does_not_create_expiry(self):
        for now in (1.,100.):
            clear=PathEvidence(('p',),20.,20.,0,True,False,2.)
            self.assertEqual(assess_path(clear,('p',),now,100.,0,False,self.profile).status,'CLEAR')
            occupied=PathEvidence(('p',),20.,20.,0,True,True,2.)
            self.assertTrue(assess_path(occupied,('p',),now,100.,0,False,self.profile).blocked)

if __name__=='__main__':unittest.main()
