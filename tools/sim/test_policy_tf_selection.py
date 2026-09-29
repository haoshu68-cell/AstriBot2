"""Source-level lifecycle seam checks; native TF behavior has separate C++ tests."""
import ast
import os
from pathlib import Path
from types import SimpleNamespace as N
import unittest
from unittest.mock import patch

SOURCE=Path(__file__).resolve().parents[2]/'ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/observer_node.py'


class SelectionTest(unittest.TestCase):
    def load(self):
        cls=next(x for x in ast.parse(SOURCE.read_text()).body if isinstance(x,ast.ClassDef) and x.name=='PolicyObserver')
        methods=[x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in ('configure_tf','destroy_node')]
        self.assertEqual({m.name for m in methods},{'configure_tf','destroy_node'},'native lifecycle seam missing')
        self.events=[]
        class Parent:
            def destroy_node(parent):self.events.append('node_closed');return True
        def buffer():self.events.append('python_buffer');return 'old_buffer'
        def listener(tf,node):self.events.append(('python_listener',tf));return 'old_listener'
        env={'Node':Parent,'os':os,'Buffer':buffer,'TransformListener':listener}
        tree=ast.Module(body=[ast.ClassDef(name='PolicyObserver',bases=[ast.Name(id='Node',ctx=ast.Load())],keywords=[],body=methods,decorator_list=[])],type_ignores=[])
        exec(compile(ast.fix_missing_locations(tree),str(SOURCE),'exec'),env)
        node=env['PolicyObserver']();node.get_parameter=lambda name:N(value=True)
        return node

    def test_default_keeps_existing_tf_and_lifecycle(self):
        node=self.load()
        with patch.dict(os.environ,{'ASTRIBOT_POLICY_NATIVE_TF':''}):node.configure_tf()
        self.assertEqual(node.tf,'old_buffer');self.assertFalse(node.native_tf)
        self.assertTrue(node.destroy_node())
        self.assertEqual(self.events,['python_buffer',('python_listener','old_buffer'),'node_closed'])

    def test_native_uses_one_cpp_listener_and_closes_before_node(self):
        node=self.load()
        def native(use_sim_time):
            self.assertTrue(use_sim_time);self.events.append('native_buffer')
            return N(close=lambda:self.events.append('native_closed'))
        with patch.dict(os.environ,{'ASTRIBOT_POLICY_NATIVE_TF':'1'}),patch.dict('sys.modules',{
                'astribot_s1_navigation_policy_native._native_tf_buffer':N(NativeTfBuffer=native)}):node.configure_tf()
        self.assertTrue(node.native_tf);self.assertIsNone(node.listener)
        self.assertTrue(node.destroy_node())
        self.assertEqual(self.events,['native_buffer','native_closed','node_closed'])

    def test_partial_native_construction_still_closes_parent(self):
        node=self.load();node.native_tf=True
        self.assertTrue(node.destroy_node())
        self.assertEqual(self.events,['node_closed'])


if __name__=='__main__':unittest.main()
