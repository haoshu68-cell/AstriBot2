"""Exercise the actual operations callbacks with fake transport, without ROS."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from verify_fixed_navigation import OwnedHoldLease
from verify_fixed_mass import source_identity
from prepare_empty_inventory import ReadbackBarrier


def closures(names, scope):
    tree=ast.parse(Path(__file__).with_name('verify_fixed_mass.py').read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    functions={n.name:n for n in main.body if isinstance(n,ast.FunctionDef)}
    body='def factory(scope):\n'+''.join(f'    {name}=scope[{name!r}]\n' for name in scope)
    for name in names:
        body+='\n'.join('    '+line for line in ast.unparse(functions[name]).splitlines())+'\n'
    body+='    return {'+','.join(repr(name)+':'+name for name in names)+'}\n'
    namespace={};exec(compile(body,'actual_mass_probe_callbacks','exec'),namespace)
    return namespace['factory'](scope)


class Future:
    def __init__(self,value):self.value=value
    def done(self):return True
    def result(self):
        if isinstance(self.value,Exception):raise self.value
        return self.value


class MassLifecycle(unittest.TestCase):
    def test_mass_requires_live_owned_action_and_fresh_matching_status(self):
        binding=OwnedHoldLease('our_goal');binding.observe('our_goal','lease',3,'ours');binding.accepted=True
        for failure in ('none','terminal','stale_hold','stale_executor','foreign_hold','foreign_executor','unaccepted'):
            scope=dict(hold_binding=binding,hold_goal=NS(accepted=failure!='unaccepted'),
                       hold_result=NS(done=lambda:failure=='terminal'),
                       fresh=lambda key:failure!='stale_'+key,
                       latest=dict(hold=NS(hold_confirmed=True,hold_id='foreign' if failure=='foreign_hold' else 'ours'),
                                   executor=dict(phase='2',lease_id='foreign' if failure=='foreign_executor' else 'lease',epoch=3,hold_id='ours')))
            with self.subTest(failure=failure):
                self.assertEqual(closures(['own_hold_ready'],scope)['own_hold_ready'](),failure=='none')

    def test_mass_rejects_foreign_hold_before_submission(self):
        sent=[]
        scope=dict(mass_request_active=False,own_hold_ready=lambda:False,submit_mass=lambda *a:sent.append(a))
        request=closures(['request_mass'],scope)['request_mass']
        with self.assertRaisesRegex(RuntimeError,'OWN_HOLD_REQUIRED'):request(.5,True)
        self.assertEqual(sent,[])

    def test_mass_detects_ownership_loss_during_request_and_resets_guard(self):
        state=NS(owned=True);sent=[];api={}
        def submit(*args):
            sent.append(args);state.owned=False;api['health']()
        scope=dict(mass_request_active=False,own_hold_ready=lambda:state.owned,submit_mass=submit,
                   observe_readback=lambda:None,latest={},active_nav={})
        api.update(closures(['request_mass','health'],scope))
        with self.assertRaisesRegex(RuntimeError,'OWN_HOLD_LOST_DURING_MASS_ADMISSION'):
            api['request_mass'](.5,True)
        self.assertEqual(sent,[(.5,True)])
        api['health']()  # Request guard is cleared so cleanup can reconcile its own action.

    def test_pose_drift_does_not_abort_but_nonzero_chassis_command_does(self):
        latest=dict(command=dict(vx=0.,vy=0.,wz=0.))
        scope=dict(mass_request_active=False,observe_readback=lambda:None,
                   latest=latest,active_nav={},math=math)
        health=closures(['health'],scope)['health']
        for pose in (dict(x=0.,y=0.,yaw=0.),dict(x=.5,y=-.2,yaw=1.)):
            latest['slam_pose']=pose
            health()
        latest.pop('slam_pose')
        health()  # Stop confirmation owns the required SLAM observation window.
        latest['command']['vx']=.1
        with self.assertRaisesRegex(RuntimeError,'NONZERO_CHASSIS_COMMAND'):
            health()

    def spin_environment(self, *, accepted=True, foreign=False, cleanup=False, unhealthy=False):
        clock=NS(value=1.)
        binding=OwnedHoldLease('our_goal');binding.observe('our_goal','lease',3,'hold');binding.accepted=accepted
        status=dict(phase='2',lease_id='foreign' if foreign else 'lease',epoch=3,hold_id='hold')
        sent=[]
        def send(request):sent.append(request);return Future(NS(accepted=True,reason='OK'))
        def spin_once(node,timeout_sec):clock.value+=.11
        def health():
            if unhealthy:raise RuntimeError('FOREIGN_NAVIGATION_ACTIVE')
        scope=dict(time=NS(monotonic=lambda:clock.value),health=health,cleanup=cleanup,
                   cleanup_health_errors=[],latest=dict(executor=status),rclpy=NS(spin_once=spin_once),node=None,
                   renew_pending=None,renew_last=0.,renew_sequence=0,renew_events=[],renew_enabled=True,
                   freeze_active=False,hold_binding=binding,renew=NS(call_async=send),RenewHold=NS(Request=NS))
        return scope,clock,sent

    def test_real_spin_renews_only_accepted_own_feedback_identity(self):
        for accepted,foreign in ((False,False),(True,True),(True,False)):
            scope,clock,sent=self.spin_environment(accepted=accepted,foreign=foreign)
            closures(['spin'],scope)['spin'](lambda:clock.value>=1.35,2.,'WAIT')
            self.assertEqual(bool(sent),accepted and not foreign)
            if sent:self.assertTrue(all(v.lease_id=='lease' and v.resource_epoch==3 for v in sent))

    def test_health_fault_cannot_block_cleanup_spin(self):
        scope,clock,sent=self.spin_environment(cleanup=True,unhealthy=True)
        closures(['spin'],scope)['spin'](lambda:clock.value>=1.35,2.,'WAIT')
        self.assertEqual(scope['cleanup_health_errors'],['FOREIGN_NAVIGATION_ACTIVE'])
        scope,clock,_=self.spin_environment(unhealthy=True)
        with self.assertRaisesRegex(RuntimeError,'FOREIGN_NAVIGATION_ACTIVE'):
            closures(['spin'],scope)['spin'](lambda:False,2.,'WAIT')

    def test_renew_failure_cannot_repeat_forever_or_block_cleanup(self):
        for answer in (NS(accepted=False,reason='EXPIRED'),RuntimeError('TRANSPORT_LOST')):
            scope,clock,sent=self.spin_environment(cleanup=True)
            def send(request):sent.append(request);return Future(answer)
            scope['renew']=NS(call_async=send)
            closures(['spin'],scope)['spin'](lambda:clock.value>=1.6,2.,'WAIT')
            self.assertEqual(len(sent),1)

    def test_readback_cannot_recover_after_intermediate_revision_change(self):
        value=dict(ids=['box'],mass_kg=.5,attachment_revision='a',ledger_epoch='l',ledger_revision=1,
                   source_epoch='s',clock_epoch=1,source_revision=2,model_revision='m',geometry_source='g',
                   geometry_clock_epoch=1,geometry_frame='base')
        barrier=ReadbackBarrier(source_identity(value),1.,1000000000)
        scope=dict(readback=barrier,summary=lambda:value,source_identity=source_identity,
                   time=NS(monotonic=lambda:1.01),node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=1010000000))))
        observe=closures(['observe_readback'],scope)['observe_readback']
        observe();self.assertTrue(barrier.valid)
        value['source_revision']=3;observe();self.assertFalse(barrier.valid)
        value['source_revision']=2;observe();self.assertFalse(barrier.valid)

    def test_clock_fault_before_or_during_stop_prevents_release(self):
        from verify_fixed_mass import cleanup_owned_resources
        for initially_bad in (True,False):
            latest={};released=[]
            if initially_bad:latest['clock_error']='ODOMETRY_CLOCK_ROLLBACK'
            def spin(predicate,timeout,reason):
                latest['clock_error']='ODOMETRY_CLOCK_ROLLBACK'
                predicate()
            scope=dict(latest=latest,ros=lambda:10.,time=NS(monotonic=lambda:2.),motion=[],
                       measured_stop=lambda *args:dict(passed=True),spin=spin)
            stop=closures(['stop'],scope)['stop']
            result=cleanup_owned_resources(False,lambda:None,stop,lambda:released.append(True),lambda:None)
            self.assertFalse(result['cleanup_complete']);self.assertEqual(released,[])
            self.assertIn('STOP_UNPROVEN',result['resource_disposition'])

    def test_feedback_identity_fault_latches_without_breaking_callback(self):
        binding=OwnedHoldLease('01');scope=dict(hold_binding=binding,renew_enabled=True,latest={},cleanup_health_errors=[])
        callback=closures(['hold_feedback'],scope)['hold_feedback']
        def message(goal,lease):return NS(goal_id=NS(uuid=bytes.fromhex(goal)),feedback=NS(lease_id=lease,resource_epoch=2,hold_id='hold'))
        callback(message('ff','foreign'));self.assertIsNone(binding.identity)
        callback(message('01','ours'));self.assertEqual(binding.identity,('ours',2,'hold'))
        callback(message('01','changed'));self.assertEqual(scope['latest']['hold_ownership_error'],'OWN_HOLD_FEEDBACK_IDENTITY_CHANGED')
        self.assertEqual(scope['cleanup_health_errors'],['OWN_HOLD_FEEDBACK_IDENTITY_CHANGED'])

    def test_release_requires_own_terminal_resource_confirmation(self):
        for released in (False,True):
            events=[];cancelled=[]
            result=Future(NS(result=NS(resources_released=released,reason='terminal'),status=5))
            goal=NS(accepted=True,get_result_async=lambda:result,cancel_goal_async=lambda:cancelled.append(True))
            def spin(predicate,timeout,reason):
                if not predicate():raise RuntimeError(reason)
            scope=dict(renew_enabled=True,pending_hold=None,hold_goal=goal,hold_result=result,admission_uncertain=False,
                       spin=spin,event=lambda *args,**kwargs:events.append((args,kwargs)))
            release=closures(['release'],scope)['release']
            if released:release();self.assertEqual(events[0][0],('hold_released',))
            else:
                with self.assertRaisesRegex(RuntimeError,'RESOURCE_RELEASE_UNCONFIRMED'):release()
                self.assertEqual(events,[])
            self.assertEqual(cancelled,[])


if __name__=='__main__':unittest.main()
