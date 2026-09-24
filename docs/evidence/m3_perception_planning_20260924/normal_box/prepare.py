#!/usr/bin/env python3
"""Offline normal-box CAD/config preparation; no ROS, simulator or inference."""
import hashlib
import importlib.util
import json
from pathlib import Path

import yaml

root = Path(__file__).resolve().parents[4]
out = Path(__file__).resolve().parent
scenario_path = root / 'ws_robot/src/astribot_s1_transport/config/warehouse_transfer.json'
scenario = json.loads(scenario_path.read_text())
size = scenario['size_xyz']
assert size == [.06, .06, .12], ('normal scenario dimensions changed', size)
assert scenario['object_id'] == 'transport_box_01'
generator = root / 'ws_robot/src/astribot_object_pose_core/tools/generate_reference_cad.py'
spec = importlib.util.spec_from_file_location('cad_generator', generator)
cad = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cad)
rows = list(cad.points([((0., 0., 0.), tuple(size))], step=.003))
cad.write(out / 'transport_box_60x60x120.xyz', rows)
visibility = {'schema': 'astribot.box_union/1', 'units': 'm',
              'boxes': [{'center_m': [0., 0., 0.], 'size_m': size}]}
(out / 'transport_box_60x60x120.visibility.json').write_text(json.dumps(visibility, indent=2) + '\n')
x, y, z = [s / 2 for s in size]
vertices = [(-x,-y,-z),(x,-y,-z),(x,y,-z),(-x,y,-z),
            (-x,-y,z),(x,-y,z),(x,y,z),(-x,y,z)]
quads = [(1,4,3,2),(5,6,7,8),(1,2,6,5),(2,3,7,6),(3,4,8,7),(4,1,5,8)]
triangles = [(a,b,c) for a,b,c,d in quads] + [(a,c,d) for a,b,c,d in quads]
mesh = ['# metres; object origin at geometric center; generated from registered BOX dimensions']
mesh += ['v ' + ' '.join(f'{v:.9f}' for v in p) for p in vertices]
mesh += ['f ' + ' '.join(map(str, t)) for t in triangles]
(out / 'transport_box_60x60x120.obj').write_text('\n'.join(mesh) + '\n')
model_id = 'transport_box_60x60x120'
registry = {model_id: {'path': str(out / (model_id + '.xyz')),
                       'visibility_model': str(out / (model_id + '.visibility.json')),
                       'symmetry': 'square_prism_z'}}
(out / 'registry.json').write_text(json.dumps(registry, indent=2) + '\n')
grasp_worker = Path.home() / '.cache/astribot/graspnet/build_cuda/graspnet_worker'
grasp_model = root / 'runs/grasp_pose_sim_20260921/models/graspnet_cuda_v2.pt'
pose_worker = root / 'runs/m3_perception_planning_20260924/install/astribot_object_pose_core/lib/astribot_object_pose_core/object_pose_register'
receipt_path = root / 'docs/evidence/joint_acceptance_20260923/camera/six_camera_raw_streams.json'
receipt = json.loads(receipt_path.read_text())
frame = receipt['topics']['/camera/raw/head_rgbd/depth_image']['frame']
params = {'use_sim_time': True, 'camera_id': 'head_rgbd', 'optical_frame': frame,
          'camera_health_topic': '/perception/camera_health/head_rgbd',
          'projection_health_topic': '/perception/projection_health/single_box/head_rgbd',
          'calibration_revision': 0, 'planning_scene_revision': 0, 'envelope_epoch': 0,
          'max_input_age_sec': .5, 'max_result_age_sec': 5., 'max_health_age_sec': .5,
          'grasp_device': 'cuda', 'grasp_worker': str(grasp_worker), 'grasp_model': str(grasp_model),
          'pose_worker': str(pose_worker), 'model_registry': str(out / 'registry.json')}
(out / 'inference.template.yaml').write_text(
    '# Preparation template; paired launch requires actual nonzero task versions.\n'
    '# Frame copied from prior raw-camera receipt; owner must verify current capture and health.\n' +
    yaml.safe_dump({'manipulation_perception_server': {'ros__parameters': params}}, sort_keys=False))
geometry = {'object_instance': scenario['object_id'], 'model_id': model_id,
            'model_frame': model_id, 'units': 'm', 'shape': 'BOX', 'dimensions': size,
            'model_from_shape': {'translation': [0.,0.,0.], 'quaternion_xyzw': [0.,0.,0.,1.]},
            'mass_kg': scenario['mass_kg'], 'symmetry': 'square_prism_z',
            'rotation_group_order_including_identity': 8,
            'unique_orientation_observable': False,
            'object_identity_revision': None,
            'note': 'Persistent instance/revision assigned by task owner, never from a per-frame detection ID.'}
(out / 'geometry.json').write_text(json.dumps(geometry, indent=2) + '\n')
input_paths = [Path(__file__).resolve(), scenario_path, generator, receipt_path,
              root / 'ws_robot/src/astribot_s1_description/config/simulation_navigation_full/camera_head_rgbd.yaml',
              grasp_worker, grasp_model]
provenance = {'schema': 'astribot.m3.normal_box_preparation/1', 'object_id': scenario['object_id'],
              'model_id': model_id, 'cad_points': len(rows), 'cad_step_m': .003,
              'mesh_vertices': len(vertices), 'mesh_triangles': len(triangles),
              'source_sha256': {str(p.relative_to(root)) if p.is_relative_to(root) else str(p):
                                hashlib.sha256(p.read_bytes()).hexdigest() for p in input_paths},
              'raw_camera': {'rgb_topic': '/camera/raw/head_rgbd/image',
                             'depth_topic': '/camera/raw/head_rgbd/depth_image',
                             'info_topic': '/camera/raw/head_rgbd/camera_info',
                             'prior_receipt_frame': frame, 'current_frame_verified': False},
              'pose_worker_after_rebuild': str(pose_worker),
              'simulated_spawn_pose_is_algorithm_input': False,
              'sensor_capture': 'NOT_RUN', 'model_inference': 'NOT_RUN',
              'model_driven_execution': 'NOT_RUN',
              'sdf_collision_inertial_receipt': 'm2_fixture_binding.json (separate M2 evidence)'}
asset_names = [model_id + '.xyz', model_id + '.visibility.json', model_id + '.obj',
               'registry.json', 'geometry.json', 'inference.template.yaml']
provenance['generated_sha256'] = {name: hashlib.sha256((out / name).read_bytes()).hexdigest()
                                  for name in asset_names}
provenance['pose_model_revision'] = (provenance['generated_sha256'][model_id + '.xyz'] + ':' +
                                     provenance['generated_sha256'][model_id + '.visibility.json'])
(out / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
print(json.dumps({'cad_points': len(rows), 'model_id': model_id,
                  'model_revision': provenance['pose_model_revision']}))
