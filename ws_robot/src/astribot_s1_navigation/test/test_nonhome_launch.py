"""Evaluate actual Humble parameter rewriting without starting any ROS node."""
import importlib.util
from pathlib import Path
import unittest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions

class LaunchParameters(unittest.TestCase):
    def test_costmaps_and_controllers(self):
        package=Path(__file__).parents[1]
        spec=importlib.util.spec_from_file_location('navigation_launch_under_test',package/'launch/navigation.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        recorded=[];original=module.RewrittenYaml
        def capture(*args,**kwargs):
            kwargs['source_file']=str(package/'config/nav2_params_mppi.yaml')
            value=original(*args,**kwargs);recorded.append(value);return value
        nodes=[];original_node=module.Node
        def capture_node(*args,**kwargs):
            nodes.append(kwargs)
            return original_node(*args,**kwargs)
        module.Node=capture_node
        module.RewrittenYaml=capture;description=module.generate_launch_description();context=LaunchContext()
        for action in description.entities:
            if isinstance(action,DeclareLaunchArgument):
                context.launch_configurations[action.name]=perform_substitutions(context,action.default_value)
        for mode in ('legacy','fixed_v2'):
            # ParameterValue caches evaluation within one launch instance.
            recorded.clear();nodes.clear();description=module.generate_launch_description()
            context.launch_configurations['navigation_geometry_mode']=mode
            context.launch_configurations['navigation_policy_stage']='p5'
            planner=next(n for n in nodes if n.get('name')=='planner_server')
            geometry=[p['navigation_geometry_mode'] for p in planner['parameters']
                      if isinstance(p,dict) and 'navigation_geometry_mode' in p]
            self.assertEqual(len(geometry),1)
            self.assertEqual(geometry[0].evaluate(context),mode)
            filename=Path(recorded[0].perform(context));data=yaml.safe_load(filename.read_text());filename.unlink()
            for name in ('local_costmap','global_costmap'):
                params=data[name][name]['ros__parameters']
                self.assertEqual(params['footprint_padding'],0. if mode=='fixed_v2' else .01)
                self.assertEqual(params['obstacle_layer']['footprint_clearing_enabled'],mode!='fixed_v2')

if __name__=='__main__':unittest.main()
