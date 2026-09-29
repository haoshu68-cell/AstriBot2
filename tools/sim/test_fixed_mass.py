import copy
import math
import unittest
from verify_fixed_mass import ledger_summary, positive_match, freeze_summary, freeze_baseline, revoked_after_freeze, full_scene_matches, cleanup_owned_resources


def stamp(n): return {'sec': 0, 'nanosec': n}


def ledger():
    return dict(confirmed=True, ledger_epoch='ledger', ledger_revision=1,
                attachment_revision='a', published_at=stamp(100), valid_until=stamp(300),
                observation=dict(environment='simulation', session_id='s', source_id='source',
                                 source_epoch='epoch', clock_epoch=1, sequence=1, revision=1,
                                 observed_at=stamp(100), valid_until=stamp(300), full_inventory=True,
                                 status=2, objects=[dict(object=dict(id='box'), weight=.5)]))


class MassProbeTest(unittest.TestCase):
    def baseline(self):
        env=dict(session='c',epoch=2,hash='h',request='r',mass=.5,allowed=True,wall=2.,stamp=10.,until=10.3)
        ack=[dict(session='c',epoch=2,hash='h',consumer=k,applied=True,wall=2.,stamp=10.) for k in ('global_costmap','local_costmap','planner','controller','policy')]
        source=dict(valid_until_ns=10250000000,capture_ns=10000000000)
        return env,ack,source

    def test_freeze_requires_matched_positive_immediately_before_pause(self):
        e,a,s=self.baseline()
        b=freeze_baseline(e,a,'r',2,.5,s,2.1,10.1)
        self.assertAlmostEqual(b['remaining_source_lease_s'],.15)
        e['allowed']=False
        with self.assertRaises(ValueError):freeze_baseline(e,a,'r',2,.5,s,2.1,10.1)
        e['allowed']=True
        with self.assertRaises(ValueError):freeze_baseline(e,a[:-1],'r',2,.5,s,2.1,10.1)

    def test_freeze_snapshot_and_transition_identity(self):
        e,a,s=self.baseline();b=freeze_baseline(e,a,'r',2,.5,s,2.1,10.1)
        s['valid_until_ns']=0;e['allowed']=False
        self.assertEqual(b['source']['valid_until_ns'],10250000000)
        self.assertTrue(b['envelope']['allowed'])
        revoked=dict(e,wall=2.3,ros_s=10.1,stamp=10.1)
        self.assertTrue(revoked_after_freeze(revoked,b,10.1))
        for key,value in [('request','old'),('epoch',1),('session','foreign'),('allowed',True),('wall',2.0),('stamp',10.),('ros_s',10.2)]:
            with self.subTest(key=key):self.assertFalse(revoked_after_freeze(dict(revoked,**{key:value}),b,10.1))

    def scene(self):
        obj=dict(link_name='arm',object=dict(id='box',header=dict(frame_id='arm',stamp=stamp(1)),operation=0,
            pose=dict(position=dict(x=.01,y=0.,z=0.),orientation=dict(x=0.,y=0.,z=0.,w=1.)),
            primitives=[dict(type=1,dimensions=[.1,.2,.3])],primitive_poses=[],meshes=[],mesh_poses=[],planes=[],plane_poses=[],subframe_names=[],subframe_poses=[]),weight=.5)
        return obj,dict(is_diff=False,robot_state=dict(is_diff=False,attached_collision_objects=[copy.deepcopy(obj)]))

    def test_scene_requires_full_geometry_and_link(self):
        obj,scene=self.scene();self.assertTrue(full_scene_matches([obj],scene))
        for change in ('diff','state_diff','link','pose','shape','extra','missing'):
            o,s=self.scene();a=s['robot_state']['attached_collision_objects'][0]
            if change=='diff':s['is_diff']=True
            if change=='state_diff':s['robot_state']['is_diff']=True
            if change=='link':a['link_name']='other_arm'
            if change=='pose':a['object']['pose']['position']['x']+=.01
            if change=='shape':a['object']['primitives'][0]['dimensions'][0]+=.01
            if change=='extra':s['robot_state']['attached_collision_objects'].append(copy.deepcopy(a))
            if change=='missing':s['robot_state']['attached_collision_objects']=[]
            with self.subTest(change=change):self.assertFalse(full_scene_matches([o],s))

    def test_scene_stamp_and_mass_are_not_geometry_evidence(self):
        obj,s=self.scene();a=s['robot_state']['attached_collision_objects'][0]
        a['weight']=0.;a['object']['header']['stamp']=stamp(99)
        self.assertTrue(full_scene_matches([obj],s))
        a['object']['pose']['position']['x']=math.nan
        self.assertFalse(full_scene_matches([obj],s))

    def test_scene_operation_ros_char_serialization(self):
        obj,s=self.scene()
        obj['object']['operation']='\x00'
        s['robot_state']['attached_collision_objects'][0]['object']['operation']='\x00'
        self.assertTrue(full_scene_matches([obj],s))
        for operation in (1,'\x01','0',''):
            s['robot_state']['attached_collision_objects'][0]['object']['operation']=operation
            with self.subTest(operation=operation):
                self.assertFalse(full_scene_matches([obj],s))

    def test_cleanup_resume_stop_and_release_order(self):
        calls=[]
        def resume():calls.append('resume')
        def stop():calls.append('stop');return dict(passed=True)
        def release():calls.append('release')
        result=cleanup_owned_resources(True,resume,stop,release,lambda:calls.append('reconcile'))
        self.assertEqual(calls,['resume','stop','release']);self.assertTrue(result['cleanup_complete'])

    def test_cleanup_stop_failure_keeps_hold_and_reports_quarantine(self):
        calls=[]
        def stop():calls.append('stop');raise RuntimeError('STALE')
        result=cleanup_owned_resources(False,lambda:None,stop,lambda:calls.append('release'),lambda:calls.append('reconcile'))
        self.assertEqual(calls,['stop','reconcile']);self.assertFalse(result['cleanup_complete'])
        self.assertIn('STOP_UNPROVEN',result['resource_disposition'])

    def test_cleanup_resume_failure_still_attempts_feedback_and_release(self):
        calls=[]
        def resume():calls.append('resume');raise RuntimeError('NO_REPLY')
        def stop():calls.append('stop');return dict(passed=True)
        result=cleanup_owned_resources(True,resume,stop,lambda:calls.append('release'),lambda:None)
        self.assertEqual(calls,['resume','stop','release']);self.assertFalse(result['cleanup_complete'])

    def test_cleanup_release_failure_is_not_released(self):
        def release():raise RuntimeError('UNKNOWN_TERMINAL')
        result=cleanup_owned_resources(False,lambda:None,lambda:dict(passed=True),release,lambda:None)
        self.assertFalse(result['cleanup_complete']);self.assertNotEqual(result['resource_disposition'],'RELEASE_CONFIRMED')

    def test_complete_mass(self):
        self.assertEqual(ledger_summary(ledger(), 's', 'source', 150)['mass_kg'], .5)

    def test_empty_and_status_mismatch(self):
        v=ledger();v['observation'].update(status=1, objects=[])
        self.assertEqual(ledger_summary(v, 's', 'source', 150)['mass_kg'], 0)
        v['observation']['status']=2
        with self.assertRaises(ValueError):ledger_summary(v, 's', 'source', 150)

    def test_identity_completeness_and_time(self):
        for key,value in [('session_id','foreign'),('full_inventory',False),('source_epoch',''),('revision',0),('sequence',0)]:
            v=ledger();v['observation'][key]=value
            with self.subTest(key=key), self.assertRaises(ValueError):ledger_summary(v,'s','source',150)
        for now in (99,300,301):
            with self.subTest(now=now), self.assertRaises(ValueError):ledger_summary(ledger(),'s','source',now)

    def test_weight_boundaries(self):
        for weight in (0,-1,math.nan,math.inf):
            v=ledger();v['observation']['objects'][0]['weight']=weight
            with self.subTest(weight=weight), self.assertRaises(ValueError):ledger_summary(v,'s','source',150)
        v=ledger();v['observation']['objects']*=2
        with self.assertRaises(ValueError):ledger_summary(v,'s','source',150)
        v['observation']['objects']=[dict(object=dict(id=k),weight=1.7e308) for k in ('a','b')]
        with self.assertRaises(ValueError):ledger_summary(v,'s','source',150)

    def test_matching_request_and_five_acks(self):
        env=dict(session='coordinator',epoch=2,hash='h',request='request',mass=.5,allowed=True,wall=2.,stamp=10.,until=10.3)
        ack=[dict(session='coordinator',epoch=2,hash='h',consumer=k,applied=True,wall=2.,stamp=10.) for k in ('global_costmap','local_costmap','planner','controller','policy')]
        self.assertTrue(positive_match(env,ack,'request',2,.5,2.1,10.1))
        legacy=dict(ack[-1],consumer='protection')
        self.assertFalse(positive_match(env,[a for a in ack if a['consumer']!='controller']+[legacy],'request',2,.5,2.1,10.1))
        self.assertTrue(positive_match(env,ack+[dict(legacy,applied=False)],'request',2,.5,2.1,10.1))
        self.assertFalse(positive_match(dict(env,hold_id='foreign'),ack,'request',2,.5,2.1,10.1,'ours'))
        self.assertTrue(positive_match(dict(env,hold_id='ours'),ack,'request',2,.5,2.1,10.1,'ours'))
        self.assertFalse(positive_match(env,ack,'new',2,.5,2.1,10.1))
        self.assertFalse(positive_match(env,ack,'request',3,.5,2.1,10.1))
        self.assertFalse(positive_match(env,ack,'request',2,1.,2.1,10.1))
        self.assertFalse(positive_match(env,ack[:-1],'request',2,.5,2.1,10.1))
        ack.append(dict(ack[-1],applied=False))
        self.assertFalse(positive_match(env,ack,'request',2,.5,2.1,10.1))

    def test_stale_and_future_ack(self):
        env=dict(session='c',epoch=2,hash='h',request='r',mass=0.,allowed=True,wall=2.,stamp=10.,until=10.3)
        ack=[dict(session='c',epoch=2,hash='h',consumer=k,applied=True,wall=2.,stamp=10.) for k in ('global_costmap','local_costmap','planner','controller','policy')]
        self.assertFalse(positive_match(env,ack,'r',2,0.,2.6,10.1))
        ack[-1]['stamp']=11.
        self.assertFalse(positive_match(env,ack,'r',2,0.,2.1,10.1))

    def test_freeze_does_not_claim_isolated_cause(self):
        v=freeze_summary(10.,10.2,.25,1.,1.,'GEOMETRY_STALE',2)
        self.assertTrue(v['revoked_with_clock_frozen'])
        self.assertFalse(v['isolated_payload_expiry_proven'])
        self.assertAlmostEqual(v['nominal_deadline_wall'],10.3)
        self.assertEqual(v['watchdog_period_s'],.05)
        self.assertFalse(freeze_summary(10.,10.2,.25,1.,1.01,'PAYLOAD_STALE',2)['revoked_with_clock_frozen'])


if __name__=='__main__':unittest.main()
