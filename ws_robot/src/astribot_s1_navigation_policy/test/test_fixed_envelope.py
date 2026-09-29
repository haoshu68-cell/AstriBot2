from pathlib import Path as _ReferencePath
import sys as _reference_sys
_reference_sys.path.insert(0, str(_ReferencePath(__file__).resolve().parents[4] / "tools/migration/python_reference"))
from reference_bootstrap import enable as _enable_references
_enable_references()

import copy
import unittest
from pathlib import Path
from types import SimpleNamespace
from astribot_navigation_msgs.msg import RobotGeometryState,ArmHoldStatus,EnvelopeApplyStatus,RobotEnvelope
from astribot_navigation_msgs.srv import SetFixedEnvelope
from astribot_s1_robot_geometry.node import polygon,stamp
from astribot_s1_robot_geometry.polygon import inflate
from astribot_s1_navigation_policy.profile import Profile
from astribot_s1_navigation_policy.fixed_envelope import FixedEnvelope,CONSUMERS
from astribot_s1_navigation_policy.robot_envelope import FIELDS,FixedEnvelopeProfile
from astribot_s1_navigation_policy.contracts import Stamp

class FixedTests(unittest.TestCase):
    def setUp(self):
        self.p=Profile.load(Path(__file__).parents[1]/'config/simulation.json')
        self.now=10_000_000_000;self.c=FixedEnvelope(self.p)
        s=RobotGeometryState();s.header.frame_id=self.p.base_frame;s.header.stamp=stamp(self.now)
        s.valid_until=stamp(self.now+300_000_000);s.complete=True;s.attachment_state_confirmed=True
        s.source_id='source';s.sequence=1;s.model_revision='model';s.attachment_revision='empty'
        s.joints.name=['arm'];s.joints.position=[0.];s.joint_source_stamps=[stamp(self.now)];s.joint_position_error_bounds=[.003]
        s.physical_footprint=polygon([[-.31,-.31],[.31,-.31],[.7,.5],[-.31,.31]])
        s.reserved_footprint=polygon(inflate([(v.x,v.y) for v in s.physical_footprint.points],.02));s.height_m=1.7
        self.s=s;self.c.state(s,self.now)
        self.c.hold=ArmHoldStatus(owner_id='task',hold_id='hold',lease_s=.3,hold_confirmed=True,attachment_revision='empty');self.c.hold.header.stamp=stamp(self.now)
        e=RobotEnvelope(frame_id=self.p.base_frame,posture_id='arbitrary_non_home',lease_s=.3)
        for f in FIELDS:setattr(e,f,getattr(self.p,f))
        self.req=SetFixedEnvelope.Request(request_id='request',hold_id='hold',geometry_sequence=1,limits=e)
    def commit(self):return self.c.propose(self.req,self.now,True)
    def acknowledge(self):
        e=self.c.output
        for name in CONSUMERS:
            a=EnvelopeApplyStatus(coordinator_session_id=e.coordinator_session_id,consumer_id=name,envelope_epoch=e.epoch,installed_geometry_hash=e.installed_geometry_hash,applied=True);a.header.stamp=stamp(self.now);self.c.acknowledge(a,self.now)
    def test_initial_hold_epoch_is_confirmable(self):
        from astribot_s1_navigation_policy.fixed_envelope_node import FixedEnvelopeAdapter
        from astribot_navigation_msgs.srv import SetRobotEnvelope
        sent={}
        def publisher(kind,topic,depth):
            return SimpleNamespace(publish=lambda m:sent.update({topic:copy.deepcopy(m)}))
        node=SimpleNamespace(create_publisher=publisher,create_subscription=lambda *a:None,
            create_service=lambda *a:None,create_timer=lambda *a:None,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=self.now,to_msg=lambda:stamp(self.now))))
        adapter=FixedEnvelopeAdapter(node,self.p)
        request=SetRobotEnvelope.Request(envelope=RobotEnvelope(transport_ready=False))
        response=adapter.legacy_request(request,SetRobotEnvelope.Response())
        self.assertTrue(response.accepted)
        self.assertEqual(sent['/navigation/robot_envelope'].epoch,response.epoch)
        self.assertFalse(sent['/navigation/robot_envelope'].transport_ready)
    def test_all_consumers_required(self):
        self.assertFalse(self.commit().navigation_allowed);self.acknowledge();self.assertTrue(self.c.tick(self.now).navigation_allowed)
        self.c.acks.pop('controller');self.assertFalse(self.c.tick(self.now).navigation_allowed)

    def test_each_consumer_requires_matching_session_epoch_hash_and_fresh_ack(self):
        for consumer in CONSUMERS:
            for field,value in [('coordinator_session_id','other-session'),('envelope_epoch',0),
                                ('installed_geometry_hash','other-hull'),('applied',False),
                                ('header.stamp',self.now-500_000_001),('header.stamp',self.now+1)]:
                with self.subTest(consumer=consumer,field=field,value=value):
                    self.setUp();e=self.commit();self.acknowledge();self.c.acks.pop(consumer)
                    a=EnvelopeApplyStatus(coordinator_session_id=e.coordinator_session_id,
                        consumer_id=consumer,envelope_epoch=e.epoch,
                        installed_geometry_hash=e.installed_geometry_hash,applied=True)
                    a.header.stamp=stamp(self.now)
                    if field=='header.stamp':a.header.stamp=stamp(value)
                    else:setattr(a,field,value)
                    self.c.acknowledge(a,self.now)
                    self.assertFalse(self.c.tick(self.now).navigation_allowed)
                    self.assertIn(consumer,self.c.output.reason)

    def test_each_consumer_negative_ack_revokes_an_existing_grant(self):
        for consumer in CONSUMERS:
            with self.subTest(consumer=consumer):
                self.setUp();e=self.commit();self.acknowledge()
                self.assertTrue(self.c.tick(self.now).navigation_allowed)
                a=EnvelopeApplyStatus(coordinator_session_id=e.coordinator_session_id,
                    consumer_id=consumer,envelope_epoch=e.epoch,
                    installed_geometry_hash=e.installed_geometry_hash,applied=False)
                a.header.stamp=stamp(self.now)
                self.c.acknowledge(a,self.now)
                self.assertFalse(self.c.tick(self.now).navigation_allowed)
                self.assertIn(consumer,self.c.output.reason)
    def test_new_geometry_relays_deadline_before_next_heartbeat(self):
        from astribot_s1_navigation_policy.fixed_envelope_node import FixedEnvelopeAdapter
        self.commit();self.acknowledge();self.c.tick(self.now)
        next_state=copy.deepcopy(self.s);next_state.sequence=2
        next_state.header.stamp=stamp(self.now+100_000_000)
        next_state.joint_source_stamps=[next_state.header.stamp]
        next_state.valid_until=stamp(self.now+400_000_000)
        self.c.hold.header.stamp=stamp(self.now+200_000_000)
        now=self.now+250_000_000;sent=[]
        adapter=object.__new__(FixedEnvelopeAdapter);adapter.core=self.c
        adapter.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:
            SimpleNamespace(nanoseconds=now,to_msg=lambda:stamp(now))))
        adapter.pub=SimpleNamespace(publish=lambda m:sent.append(copy.deepcopy(m)))
        adapter.legacy=SimpleNamespace(publish=lambda m:None);adapter.footprints=[]
        adapter.geometry(next_state)
        self.assertTrue(sent[-1].navigation_allowed)
        self.assertEqual(sent[-1].valid_until,next_state.valid_until)
        self.assertEqual(sent[-1].header.stamp,stamp(now))
    def test_no_heartbeat_refresh(self):
        self.commit();self.acknowledge();self.assertFalse(self.c.tick(self.now+301_000_000).navigation_allowed)
        self.assertIn('EXPIRED',self.c.output.reason)
    def test_posture_drift_latched(self):
        self.commit();self.acknowledge();s=copy.deepcopy(self.s);s.sequence=2;s.joints.position=[.004];self.c.state(s,self.now)
        self.assertFalse(self.c.tick(self.now).navigation_allowed);s.sequence=3;s.joints.position=[0.];self.c.state(s,self.now)
        self.assertFalse(self.c.tick(self.now).navigation_allowed)
    def test_attachment_restart_and_clock_invalidate(self):
        for attr,val in [('source_id','restart'),('clock_epoch',1),('attachment_revision','changed'),('model_revision','new')]:
            self.setUp();self.commit();self.acknowledge();s=copy.deepcopy(self.s);setattr(s,attr,val);s.sequence=2;self.c.state(s,self.now)
            self.assertFalse(self.c.tick(self.now).navigation_allowed)
    def test_stop_and_owner_required(self):
        with self.assertRaises(ValueError):self.c.propose(self.req,self.now,False)
        self.c.hold.hold_confirmed=False
        with self.assertRaises(ValueError):self.commit()
    def test_wrong_ack_rejected(self):
        self.commit();self.acknowledge();self.c.acks.clear()
        a=EnvelopeApplyStatus(coordinator_session_id='old',consumer_id='planner',envelope_epoch=1,installed_geometry_hash=self.c.output.installed_geometry_hash,applied=True);a.header.stamp=stamp(self.now);self.c.acknowledge(a,self.now)
        self.assertEqual(self.c.acks,{})
    def test_future_ack_does_not_revoke_or_renew_previous_evidence(self):
        e=self.commit();self.acknowledge()
        a=EnvelopeApplyStatus(coordinator_session_id=e.coordinator_session_id,
            consumer_id='planner',envelope_epoch=e.epoch,
            installed_geometry_hash=e.installed_geometry_hash,applied=True)
        a.header.stamp=stamp(self.now+1_000_000)
        self.c.acknowledge(a,self.now)
        self.assertTrue(self.c.tick(self.now).navigation_allowed)
        self.assertEqual(self.c.acks['planner'],self.now)
        # Later fresh sources and other consumers cannot extend planner's old
        # proof after 500 ms merely because a future ACK was seen earlier.
        later=self.now+500_000_000
        s=copy.deepcopy(self.s);s.sequence=2;s.header.stamp=stamp(later)
        s.joint_source_stamps=[s.header.stamp];s.valid_until=stamp(later+300_000_000)
        self.c.state(s,later);self.c.hold.header.stamp=stamp(later)
        for name in CONSUMERS-{'planner'}:self.c.acks[name]=later
        self.assertFalse(self.c.tick(later).navigation_allowed)
        self.assertEqual(self.c.output.reason,'WAITING_FOR:planner')
        self.setUp();e=self.commit();self.acknowledge()
        a.coordinator_session_id=e.coordinator_session_id;a.envelope_epoch=e.epoch
        self.c.acks.pop('planner')
        self.c.acknowledge(a,self.now)
        self.assertFalse(self.c.tick(self.now).navigation_allowed)
        self.acknowledge();a.applied=False
        self.c.acknowledge(a,self.now)
        self.assertFalse(self.c.tick(self.now).navigation_allowed)
    def test_consumer_polygon_and_expiry(self):
        e=self.commit();self.acknowledge();e=self.c.tick(self.now);p=FixedEnvelopeProfile(self.p);t=Stamp(self.now,'ros',1)
        self.assertTrue(p.accept(e,t));self.assertTrue(p.ready(t));self.assertIsNotNone(p.footprint_xy)
        self.assertFalse(p.ready(Stamp(self.now+300_000_000,'ros',1)))
        bad=copy.deepcopy(e);bad.installed_geometry_hash='wrong'
        with self.assertRaises(ValueError):p.accept(bad,t)
        self.assertFalse(p.ready(t))
    def test_configuration_confirmation_never_refreshes_decision_profile(self):
        e=self.commit();self.acknowledge();e=self.c.tick(self.now)
        p=FixedEnvelopeProfile(self.p);t=Stamp(self.now,'sim',0)
        self.assertFalse(p.confirms_applied(e,t));p.accept(e,t)
        later=self.now+600_000_000;fresh=copy.deepcopy(e)
        fresh.header.stamp=stamp(later);fresh.valid_until=stamp(later+300_000_000)
        self.assertTrue(p.confirms_applied(fresh,Stamp(later,'sim',0)))
        self.assertEqual(p.received,t)
        self.assertEqual(p.v2.header.stamp,e.header.stamp)
        self.assertFalse(p.ready(Stamp(later,'sim',0)))
        for field in ('installed_footprint','reserved_footprint'):
            bad=copy.deepcopy(fresh);getattr(bad,field).points[0].x+=1e-7
            self.assertFalse(p.confirms_applied(bad,Stamp(later,'sim',0)))
        for field,value in [('epoch',fresh.epoch+1),('clock_epoch',1),('hold_id','other'),
                            ('model_revision','other'),('attachment_revision','other')]:
            bad=copy.deepcopy(fresh);setattr(bad,field,value)
            self.assertFalse(p.confirms_applied(bad,Stamp(later,'sim',0)))
        bad=copy.deepcopy(fresh);bad.limits.max_speed_m_s*=.9
        self.assertFalse(p.confirms_applied(bad,Stamp(later,'sim',0)))
        self.assertFalse(p.confirms_applied(fresh,Stamp(later-1,'sim',0)))
        self.assertFalse(p.confirms_applied(fresh,Stamp(later+300_000_000,'sim',0)))
        self.assertFalse(p.confirms_applied(fresh,Stamp(later,'sim',1)))
        self.assertFalse(p.confirms_applied(e,Stamp(self.now-1,'sim',0)))
        bad=copy.deepcopy(fresh);bad.installed_geometry_hash='invalid'
        with self.assertRaises(ValueError):p.accept(bad,Stamp(later,'sim',0))
        self.assertFalse(p.confirms_applied(fresh,Stamp(later,'sim',0)))

    def test_missing_joint_arrays_and_future(self):
        self.c.current.joint_source_stamps=[]
        with self.assertRaises(ValueError):self.commit()
        self.setUp();self.c.current.header.stamp=stamp(self.now+1)
        with self.assertRaises(ValueError):self.commit()

if __name__=='__main__':unittest.main()
