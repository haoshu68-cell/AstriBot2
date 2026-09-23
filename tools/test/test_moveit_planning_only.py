"""Pure launch policy checks; ROS graph verification is a separate gate."""
import ast
from pathlib import Path
import unittest

SOURCE = Path(__file__).resolve().parents[2] / 'ws_robot/src/astribot_s1_moveit_config/launch/move_group.launch.py'


class PlanningOnlyTest(unittest.TestCase):
    def test_controller_clients_are_removed_only_when_execution_disabled(self):
        tree = ast.parse(SOURCE.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                        and n.name == '_controllers_for_execution')
        namespace = {}
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE), 'exec'), namespace)
        select = namespace[function.name]
        original = {'moveit_simple_controller_manager': {'controller_names': ['arm_left_controller'],
                    'arm_left_controller': {'type': 'FollowJointTrajectory'}}}
        disabled = select(original, False)
        self.assertNotIn('moveit_simple_controller_manager', disabled)
        self.assertEqual(select(original, True), original)
        self.assertEqual(original['moveit_simple_controller_manager']['controller_names'], ['arm_left_controller'])


if __name__ == '__main__':
    unittest.main()
