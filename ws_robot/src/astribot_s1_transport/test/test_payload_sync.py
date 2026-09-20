import unittest
import json
from types import SimpleNamespace as N
from astribot_s1_transport.payload_sync import physical_parent,confirmed_state
from astribot_s1_transport.ros_backend import RosBackend


class PayloadSyncTests(unittest.TestCase):
    def state(self,**changes):
        state=dict(stamp_ns=10_000_000_000,command_id=1,attached=True,error='',
                   position_error_m=.001,rotation_error_rad=.001)
        state.update(changes);return state

    def test_applied_identity_and_source_deadline_required(self):
        self.assertTrue(confirmed_state(self.state(),1,True,10.1,.08))
        for change in (dict(command_id=0),dict(attached=False),dict(error='PARENT_LOST'),dict(stamp_ns=0)):
            self.assertFalse(confirmed_state(self.state(**change),1,True,10.1,.08))
        for now in (9.999,10.301):self.assertFalse(confirmed_state(self.state(),1,True,now,.08))
        self.assertFalse(confirmed_state(self.state(),2,False,10.1,.08))

    def test_filter_budget_covers_translation_and_rotation(self):
        self.assertFalse(confirmed_state(self.state(position_error_m=.007),1,True,10.1,.08))
        self.assertFalse(confirmed_state(self.state(position_error_m=.004,rotation_error_rad=.04),1,True,10.1,.08))
        for bad in (float('nan'),-1.,float('inf')):
            self.assertFalse(confirmed_state(self.state(position_error_m=bad),1,True,10.1,.08))

    def test_fixed_chain_resolves_to_retained_articulated_link(self):
        urdf='''<robot><link name="base"/><link name="wrist"/><link name="tool"/><link name="tcp"/>
        <joint type="revolute"><parent link="base"/><child link="wrist"/></joint>
        <joint type="fixed"><parent link="wrist"/><child link="tool"/></joint>
        <joint type="fixed"><parent link="tool"/><child link="tcp"/></joint></robot>'''
        self.assertEqual(physical_parent(urdf,'tcp'),'wrist')
        with self.assertRaisesRegex(ValueError,'NOT_IN_MODEL'):physical_parent(urdf,'missing')
        with self.assertRaisesRegex(ValueError,'CYCLE'):
            physical_parent(urdf.replace('parent link="wrist"','parent link="tcp"'),'tcp')

    def test_future_positive_does_not_destroy_previous_evidence(self):
        previous=self.state(stamp_ns=9_900_000_000)
        node=N(payload_state=previous,get_clock=lambda:N(now=lambda:N(nanoseconds=10_000_000_000)))
        RosBackend.receive_payload_state(node,N(data=json.dumps(self.state(stamp_ns=10_001_000_000))))
        self.assertIs(node.payload_state,previous)
        RosBackend.receive_payload_state(node,N(data=json.dumps(self.state(stamp_ns=10_001_000_000,error='PARENT_LOST'))))
        self.assertEqual(node.payload_state['error'],'PARENT_LOST')


if __name__=='__main__':unittest.main()
