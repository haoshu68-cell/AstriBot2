"""Evaluate actual Humble parameter rewriting without starting any ROS node."""
import importlib.util
from pathlib import Path
import unittest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions

class LaunchParameters(unittest.TestCase):
    def test_payload_identity_is_bound_to_geometry_and_ledger(self):
        from launch_ros.utilities import evaluate_parameters, normalize_parameters
        package=Path(__file__).parents[1]
        spec=importlib.util.spec_from_file_location('payload_launch_under_test',package/'launch/navigation.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        nodes=[];real_node=module.Node
        def capture(*args,**kwargs):
            nodes.append(kwargs)
            return real_node(*args,**kwargs)
        module.Node=capture
        module.generate_launch_description()
        context=LaunchContext()
        context.launch_configurations.update(use_sim_time='true', payload_environment='simulation',
            payload_session_id='owned_offline_contract',payload_source_id='physical_inventory',
            payload_journal_path='/tmp/unused_offline_payload_journal')
        geometry=next(n for n in nodes if n.get('executable')=='geometry_state')
        values=evaluate_parameters(context,normalize_parameters(geometry['parameters']))[0]
        self.assertEqual(values.get('payload_session_id'),'owned_offline_contract')
        self.assertEqual(values.get('payload_source_id'),'physical_inventory')
        ledger=next(n for n in nodes if n.get('executable')=='payload_state')
        settings=evaluate_parameters(context,normalize_parameters(ledger['parameters']))[0]
        self.assertEqual(settings['session_id'],values['payload_session_id'])
        self.assertEqual(settings['source_id'],values['payload_source_id'])
        self.assertEqual(settings['environment'],values['payload_environment'])
        context.launch_configurations.update(enable_payload_ledger='false',navigation_geometry_mode='fixed_v2')
        self.assertFalse(ledger['condition'].evaluate(context))
        context.launch_configurations['enable_payload_ledger']='true'
        self.assertTrue(ledger['condition'].evaluate(context))
        context.launch_configurations['navigation_geometry_mode']='legacy'
        self.assertFalse(ledger['condition'].evaluate(context))

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

    def test_depth_switch_is_scoped_and_lidar_only_is_explicit(self):
        package=Path(__file__).parents[1]
        spec=importlib.util.spec_from_file_location('navigation_launch_sensor_under_test',package/'launch/navigation.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        recorded=[];original=module.RewrittenYaml
        def capture(*args,**kwargs):
            kwargs['source_file']=str(package/'config/nav2_params_mppi.yaml')
            value=original(*args,**kwargs);recorded.append(value);return value
        module.RewrittenYaml=capture
        description=module.generate_launch_description();context=LaunchContext()
        for action in description.entities:
            if isinstance(action,DeclareLaunchArgument):
                context.launch_configurations[action.name]=perform_substitutions(context,action.default_value)
        context.launch_configurations['navigation_policy_stage']='p5'
        context.launch_configurations['navigation_geometry_mode']='fixed_v2'
        context.launch_configurations['enable_depth_obstacles']='false'
        module.generate_launch_description()
        filename=Path(recorded[0].perform(context));data=yaml.safe_load(filename.read_text());filename.unlink()
        local=data['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']
        global_=data['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']
        self.assertEqual(local['observation_sources'],'scan')
        self.assertEqual(global_['observation_sources'],'scan')
        self.assertEqual(local['scan']['topic'],'/navigation_policy/costmap_scan')
        self.assertEqual(local['head_depth']['topic'],'/camera/head_rgbd/points')

    def test_voxel_profile_keeps_3d_sources_and_bounds_explicit(self):
        package=Path(__file__).parents[1]
        spec=importlib.util.spec_from_file_location('navigation_launch_voxel_under_test',package/'launch/navigation.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        recorded=[];original=module.RewrittenYaml
        def capture(*args,**kwargs):
            kwargs['source_file']=str(package/'config/nav2_params_mppi.yaml')
            value=original(*args,**kwargs);recorded.append(value);return value
        module.RewrittenYaml=capture
        description=module.generate_launch_description();context=LaunchContext()
        for action in description.entities:
            if isinstance(action,DeclareLaunchArgument):
                context.launch_configurations[action.name]=perform_substitutions(context,action.default_value)
        context.launch_configurations.update({
            'navigation_policy_stage':'p5',
            'navigation_geometry_mode':'fixed_v2',
            'obstacle_layer_plugin':'nav2_costmap_2d::VoxelLayer',
            'enable_depth_obstacles':'true',
        })
        module.generate_launch_description()
        filename=Path(recorded[0].perform(context));data=yaml.safe_load(filename.read_text());filename.unlink()
        for name in ('local_costmap','global_costmap'):
            layer=data[name][name]['ros__parameters']['obstacle_layer']
            self.assertEqual(layer['plugin'],'nav2_costmap_2d::VoxelLayer')
            self.assertEqual(layer['observation_sources'],
                             'scan head_depth torso_depth' if name=='local_costmap' else 'scan')
            self.assertGreater(layer['z_voxels'],0)
            self.assertLessEqual(layer['z_voxels'],16)
            self.assertGreater(layer['origin_z']+layer['z_voxels']*layer['z_resolution'],2.0)
        local=data['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']
        self.assertEqual(local['head_depth']['data_type'],'PointCloud2')
        self.assertEqual(local['torso_depth']['data_type'],'PointCloud2')

    def test_voxel_invalid_height_contract_rejected(self):
        import copy
        package=Path(__file__).parents[1]
        spec=importlib.util.spec_from_file_location('voxel_contract',package/'launch/navigation.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        for profile in ('mppi','rpp'):
            data=yaml.safe_load((package/f'config/nav2_params_{profile}.yaml').read_text())
            module.validate_voxel_layers(data,'nav2_costmap_2d::VoxelLayer')
            for field,value in [('z_voxels',32),('z_voxels',0),('z_voxels',True),
                                ('z_resolution',float('nan')),('z_resolution',0.1),
                                ('origin_z',float('inf')),('mark_threshold',16)]:
                bad=copy.deepcopy(data)
                bad['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer'][field]=value
                with self.subTest(profile=profile,field=field,value=value):
                    with self.assertRaises(ValueError):
                        module.validate_voxel_layers(bad,'nav2_costmap_2d::VoxelLayer')
            # Other layer types do not acquire new voxel-only restrictions.
            module.validate_voxel_layers(bad,'nav2_costmap_2d::ObstacleLayer')

if __name__=='__main__':unittest.main()
