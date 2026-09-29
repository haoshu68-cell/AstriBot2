#!/usr/bin/env python3
"""Offline, temporary photo-reference camera mount geometry audit.

Never edits calibration, robot sources or running ROS/Gazebo state. Camera
origins are optical centers; camera_link uses +x forward, +y left, +z up.
The photo establishes approximate anatomical placement, not calibrated poses.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[2]
DESC = ROOT / 'ws_robot/src/astribot_s1_description'
SHARE = ROOT / 'ws_robot/install/astribot_s1_description/share/astribot_s1_description'
OPTICAL_RPY = [-np.pi / 2, 0, -np.pi / 2]


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    t[:3, 3] = xyz
    return t


def origin(element):
    return np.eye(4) if element is None else transform(
        np.fromstring(element.get('xyz', '0 0 0'), sep=' '),
        np.fromstring(element.get('rpy', '0 0 0'), sep=' '))


def stl(path):
    raw = path.read_bytes()
    count = struct.unpack('<I', raw[80:84])[0]
    if len(raw) != 84 + 50 * count:
        raise ValueError(f'Not binary STL: {path}')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    return np.frombuffer(raw[84:], dtype=dtype)['vertices'].astype(float)


def box_mesh(size, center):
    from scipy.spatial import ConvexHull
    vertices = np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)])
    vertices = vertices * size + center
    return vertices[ConvexHull(vertices).simplices]


class Mesh:
    """Exact triangle ray tests with a triangle-AABB BVH; no mesh decimation."""
    def __init__(self, triangles):
        self.triangles = triangles
        self.e1 = triangles[:, 1] - triangles[:, 0]
        self.e2 = triangles[:, 2] - triangles[:, 0]
        centers = triangles.mean(axis=1)
        lo, hi = triangles.min(axis=1), triangles.max(axis=1)

        def build(indices):
            bounds = [lo[indices].min(axis=0), hi[indices].max(axis=0)]
            if len(indices) <= 256:
                return bounds, indices, None
            axis = np.argmax(np.ptp(centers[indices], axis=0))
            ordered = indices[np.argsort(centers[indices, axis], kind='stable')]
            middle = len(ordered) // 2
            return bounds, build(ordered[:middle]), build(ordered[middle:])
        self.tree = build(np.arange(len(triangles)))

    @staticmethod
    def aabb_hit(bounds, point, direction, maximum):
        near, far = 0., maximum
        for axis in range(3):
            if abs(direction[axis]) < 1e-12:
                if point[axis] < bounds[0][axis] or point[axis] > bounds[1][axis]:
                    return False
            else:
                a = (bounds[0][axis] - point[axis]) / direction[axis]
                b = (bounds[1][axis] - point[axis]) / direction[axis]
                near, far = max(near, min(a, b)), min(far, max(a, b))
                if far < near:
                    return False
        return True

    def ray(self, point, direction, maximum=5.):
        best = maximum
        found = False
        pending = [self.tree]
        while pending:
            bounds, left, right = pending.pop()
            if not self.aabb_hit(bounds, point, direction, best):
                continue
            if right is not None:
                pending.extend((left, right))
                continue
            ids = left
            e1, e2 = self.e1[ids], self.e2[ids]
            h = np.cross(direction, e2)
            determinant = (e1 * h).sum(axis=1)
            valid = np.abs(determinant) > 1e-12
            inverse = np.divide(1., determinant, out=np.zeros_like(determinant), where=valid)
            delta = point - self.triangles[ids, 0]
            u = inverse * (delta * h).sum(axis=1)
            q = np.cross(delta, e1)
            v = inverse * (q * direction).sum(axis=1)
            distance = inverse * (e2 * q).sum(axis=1)
            good = valid & (u >= -1e-10) & (v >= -1e-10) & (u + v <= 1 + 1e-10) & (distance > 1e-6) & (distance <= best)
            if np.any(good):
                best, found = float(distance[good].min()), True
        return best if found else None


class Cylinder(Mesh):
    """Analytic ray intersections; tessellation is only for plotting/bounds."""
    def __init__(self, radius, length, local):
        from scipy.spatial import ConvexHull
        angles = np.linspace(0, 2 * np.pi, 128, endpoint=False)
        vertices = np.array([[radius * np.cos(a), radius * np.sin(a), z]
                             for z in (-length / 2, length / 2) for a in angles])
        tri = vertices[ConvexHull(vertices).simplices]
        super().__init__(tri @ local[:3, :3].T + local[:3, 3])
        self.radius, self.length, self.local = radius, length, local

    def ray(self, point, direction, maximum=5.):
        p = self.local[:3, :3].T @ (point - self.local[:3, 3])
        d = self.local[:3, :3].T @ direction
        candidates = []
        a, b, c = d[:2] @ d[:2], 2 * (p[:2] @ d[:2]), p[:2] @ p[:2] - self.radius**2
        discriminant = b*b - 4*a*c
        if a > 1e-16 and discriminant >= 0:
            for t in ((-b - np.sqrt(discriminant)) / (2*a), (-b + np.sqrt(discriminant)) / (2*a)):
                if 1e-6 < t <= maximum and abs(p[2] + t*d[2]) <= self.length / 2:
                    candidates.append(float(t))
        if abs(d[2]) > 1e-12:
            for z in (-self.length / 2, self.length / 2):
                t = (z - p[2]) / d[2]
                if 1e-6 < t <= maximum and np.linalg.norm(p[:2] + t*d[:2]) <= self.radius:
                    candidates.append(float(t))
        return min(candidates) if candidates else None


def forward_kinematics(robot, positions=None):
    positions = dict(positions or {})
    joints = robot.findall('joint')
    roots = {link.get('name') for link in robot.findall('link')} - {j.find('child').get('link') for j in joints}
    if roots != {'astribot_torso_base'}:
        raise ValueError(f'Unexpected roots: {roots}')
    fk = {'astribot_torso_base': np.eye(4)}
    pending = list(joints)
    while pending:
        old_count = len(pending)
        for joint in pending[:]:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent not in fk:
                continue
            q = positions.get(joint.get('name'), 0.)
            mimic = joint.find('mimic')
            if mimic is not None:
                q = positions.get(mimic.get('joint'), 0.) * float(mimic.get('multiplier', 1)) + float(mimic.get('offset', 0))
            motion = np.eye(4)
            if joint.get('type') in ('revolute', 'continuous', 'prismatic'):
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                if joint.get('type') == 'prismatic':
                    motion[:3, 3] = axis * q
                else:
                    motion[:3, :3] = Rotation.from_rotvec(axis * q).as_matrix()
            fk[child] = fk[parent] @ origin(joint.find('origin')) @ motion
            pending.remove(joint)
        if len(pending) == old_count:
            raise ValueError('Disconnected FK tree')
    return fk


def load_geometry(robot):
    meshes, metadata = {}, []
    for link in robot.findall('link'):
        triangles = []
        sources = []
        cylinder_geometry = None
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                box = visual.find('geometry/box')
                if box is None:
                    cylinder = visual.find('geometry/cylinder')
                    if cylinder is not None:
                        cylinder_geometry = Cylinder(float(cylinder.get('radius')), float(cylinder.get('length')), origin(visual.find('origin')))
                        triangles.append(cylinder_geometry.triangles)
                        sources.append({'primitive': 'cylinder', 'radius': cylinder.get('radius'), 'length': cylinder.get('length'),
                                        'ray_intersection': 'analytic', 'plot_only_segments': 128})
                    continue
                local = origin(visual.find('origin'))
                tri = box_mesh(np.fromstring(box.get('size'), sep=' '), np.zeros(3))
                triangles.append(tri @ local[:3, :3].T + local[:3, 3])
                sources.append({'primitive': 'box', 'visual_name': visual.get('name'), 'size': box.get('size'), 'origin': local.tolist()})
                continue
            path = SHARE / mesh.get('filename').split('astribot_s1_description/', 1)[1]
            tri = stl(path) * np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            local = origin(visual.find('origin'))
            triangles.append(tri @ local[:3, :3].T + local[:3, 3])
            sources.append({'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
        if triangles:
            tri = np.concatenate(triangles)
            meshes[link.get('name')] = cylinder_geometry if cylinder_geometry is not None and len(triangles) == 1 else Mesh(tri)
            metadata.append({'link': link.get('name'), 'triangles': len(tri), 'sources': sources,
                             'local_mesh_bounds': [tri.min(axis=(0, 1)).tolist(), tri.max(axis=(0, 1)).tolist()]})
    return meshes, metadata


def surface_mount(mesh, y, z, width, height, head=False):
    samples = []
    for dy in np.linspace(-height / 2 if head else -width / 2, height / 2 if head else width / 2, 7):
        for dz in np.linspace(-width / 2 if head else -height / 2, width / 2 if head else height / 2, 7):
            ray = np.array([.3, y + dy, z + dz])
            hit = mesh.ray(ray, np.array([-1., 0, 0]))
            if hit is not None:
                samples.append({'y': y + dy, 'z': z + dz, 'surface_x': .3 - hit})
    if not samples:
        raise ValueError('Camera footprint does not intersect its mounting body')
    # Housing x extent is [-.031,-.001] relative to the optical center.
    # Two millimetres gap at the most forward sampled body surface.
    x = max(sample['surface_x'] for sample in samples) + .033
    return [round(x, 6), y, z], {'footprint_samples': samples, 'housing_rear_min_gap_m': .002,
                                'housing_rear_max_gap_m': x - .031 - min(sample['surface_x'] for sample in samples)}


def candidate_mounts(meshes):
    mounts = {}
    for name, y, z, size in (
            ('head_rgbd', -.22, 0, [.03, .08, .03]),
            ('head_stereo_left', -.15, .03, [.03, .025, .025]),
            ('head_stereo_right', -.15, -.03, [.03, .025, .025])):
        xyz, fit = surface_mount(meshes['astribot_head_link_2'], y, z, size[1], size[2], head=True)
        mounts[name] = {'parent': 'astribot_head_link_2', 'xyz': xyz, 'rpy': [np.pi / 2, 0, 0], 'housing_size': size, 'fit': fit}
    xyz, fit = surface_mount(meshes['astribot_torso_link_4'], 0, .12, .08, .03)
    mounts['torso_rgbd'] = {'parent': 'astribot_torso_link_4', 'xyz': xyz, 'rpy': [0, 0, 0], 'housing_size': [.03, .08, .03], 'fit': fit}
    for side in ('left', 'right'):
        parent = f'astribot_arm_{side}_link_7'
        # Both end-frame +z axes point inward at zero FK. Negative local z is
        # the external wrist side for both arms, independently of naming.
        mounts[f'{side}_wrist_rgbd'] = {'parent': parent, 'xyz': [0., -.03, -.058],
            'rpy': [np.pi, 0, -np.pi / 2], 'housing_size': [.03, .08, .03],
            'fit': {'mount_side': 'negative local z, external wrist side',
                    'body_shell_max_negative_z_m': float(meshes[parent].triangles[:, :, 2].min()),
                    'housing_inner_face_z_m': -.043, 'side_gap_to_extreme_m': .00199}}
    for mount in mounts.values():
        mount['housing_center_camera_link'] = [-.016, 0, 0]
        mount['optical_fixed_rpy'] = OPTICAL_RPY
        mount['status'] = 'temporary_sim_photo_reference_not_calibration'
    return mounts


def support_bracket(mount, mesh):
    """Small solid bridge with centerline mechanical contact, not calibration."""
    t = transform(mount['xyz'], mount['rpy'])
    wrist = 'arm_' in mount['parent']
    contact = []
    housing_x = mount['housing_center_camera_link'][0]
    for a in np.linspace(-.006, .006, 7):
        for b in np.linspace(-.01, .01, 7):
            ray = np.array([housing_x + a, b, 0.]) if wrist else np.array([0., b, a])
            direction = np.array([0., 0, -1.]) if wrist else np.array([-1., 0, 0.])
            distance = mesh.ray(t[:3, 3] + t[:3, :3] @ ray, t[:3, :3] @ direction, .2)
            if distance is not None:
                contact.append((ray + direction * distance)[2 if wrist else 0])
    if not contact:
        raise ValueError('Support bracket has no body contact')
    ray = np.array([housing_x, 0., 0.]) if wrist else np.zeros(3)
    direction = np.array([0., 0., -1.]) if wrist else np.array([-1., 0., 0.])
    depth = mesh.ray(t[:3, 3] + t[:3, :3] @ ray, t[:3, :3] @ direction, .2)
    if depth is None:
        raise ValueError('No centerline bracket anchor')
    end = -mount['housing_size'][2] / 2 + .001 if wrist else housing_x - mount['housing_size'][0] / 2 + .001
    start = -depth - .002
    center = [housing_x, 0., (start + end) / 2] if wrist else [(start + end) / 2, 0., 0.]
    size = [.012, .020, end - start] if wrist else [end - start, .020, .012]
    return {'center_camera_link': center, 'size': size, 'surface_axis_samples': contact,
            'centerline_surface_depth_m': depth,
            'contact': 'Rear enters centerline parent surface by 2 mm; front enters housing by 1 mm. Interface overlap is intentional; lens/FOV remain clear.'}


def triangle_box_intersections(triangles, center, size):
    """Separating-axis triangle versus axis-aligned box test in camera frame."""
    v = triangles - center
    half = np.array(size) / 2
    overlap = (v.min(axis=1) <= half).all(axis=1) & (v.max(axis=1) >= -half).all(axis=1)
    v = v[overlap]
    if not len(v):
        return 0
    edges = [v[:, 1] - v[:, 0], v[:, 2] - v[:, 1], v[:, 0] - v[:, 2]]
    axes = [np.cross(edges[0], edges[1])]
    for edge in edges:
        for axis in np.eye(3):
            axes.append(np.cross(edge, axis))
    valid = np.ones(len(v), dtype=bool)
    for axis in axes:
        dots = (v * axis[:, None, :]).sum(axis=2)
        radius = np.abs(axis) @ half
        valid &= (dots.min(axis=1) <= radius + 1e-10) & (dots.max(axis=1) >= -radius - 1e-10)
    return int(valid.sum())


def housing_clearance(mounts, meshes, fk):
    results = {}
    for name, mount in mounts.items():
        camera = fk[mount['parent']] @ transform(mount['xyz'], mount['rpy'])
        inverse = np.linalg.inv(camera)
        collisions = {}
        for link, mesh in meshes.items():
            if link == name + '_camera_link':
                continue
            relative = inverse @ fk[link]
            tri = mesh.triangles @ relative[:3, :3].T + relative[:3, 3]
            count = triangle_box_intersections(tri, np.array(mount['housing_center_camera_link']), mount['housing_size'])
            if count:
                collisions[link] = count
        results[name] = {'physical_mesh_triangle_box_intersections': collisions,
                         'housing_clear_of_physical_visual_surfaces': not collisions,
                         'excluded': 'Only this camera own housing and its intentionally contacting support'}
    return results


def bracket_contacts(mounts, meshes, fk):
    contacts = {}
    for name, mount in mounts.items():
        camera = fk[mount['parent']] @ transform(mount['xyz'], mount['rpy'])
        bracket = mount['support_bracket']
        relative = np.linalg.inv(camera) @ fk[bracket['contact_link']]
        body = meshes[bracket['contact_link']].triangles @ relative[:3, :3].T + relative[:3, 3]
        housing = box_mesh(np.array(mount['housing_size']), np.array(mount['housing_center_camera_link']))
        contact_body = triangle_box_intersections(body, np.array(bracket['center_camera_link']), bracket['size'])
        contact_housing = triangle_box_intersections(housing, np.array(bracket['center_camera_link']), bracket['size'])
        contacts[name] = {'fixed_body_contact_link': bracket['contact_link'],
                          'body_surface_triangles_intersecting_support': contact_body,
                          'housing_triangles_intersecting_support': contact_housing,
                          'support_connects_body_and_housing': contact_body > 0 and contact_housing > 0,
                          'intentional_interface_overlap': True}
    return contacts


def plot_geometry(output, meshes, fk, mounts):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from scipy.spatial import ConvexHull
    fig, panels = plt.subplots(1, 3, figsize=(15, 8))
    for mesh_name, mesh in meshes.items():
        stride = max(1, len(mesh.triangles) // 1400)
        vertices = mesh.triangles[::stride].reshape(-1, 3)
        world = vertices @ fk[mesh_name][:3, :3].T + fk[mesh_name][:3, 3]
        panels[0].scatter(world[:, 1], world[:, 2], s=.15, color='#8a939c', alpha=.4, rasterized=True)
        panels[1].scatter(world[:, 0], world[:, 2], s=.15, color='#8a939c', alpha=.4, rasterized=True)
        if mesh_name.startswith(('astribot_arm_left_link_7', 'astribot_gripper_left')):
            local = (world - fk['astribot_arm_left_link_7'][:3, 3]) @ fk['astribot_arm_left_link_7'][:3, :3]
            panels[2].scatter(local[:, 2], local[:, 1], s=.2, color='#8a939c', alpha=.45, rasterized=True)
    colors = ['#e53935', '#00897b', '#00acc1', '#fb8c00', '#6a1b9a', '#3949ab']
    for (name, mount), color in zip(mounts.items(), colors):
        t = fk[mount['parent']] @ transform(mount['xyz'], mount['rpy'])
        center, forward = t[:3, 3], t[:3, 0] * .11
        housing = box_mesh(np.array(mount['housing_size']), np.array(mount['housing_center_camera_link'])).reshape(-1, 3)
        bracket = mount['support_bracket']
        support = box_mesh(np.array(bracket['size']), np.array(bracket['center_camera_link'])).reshape(-1, 3)
        for vertices, shape_color in ((housing, color), (support, '#555555')):
            world = vertices @ t[:3, :3].T + t[:3, 3]
            for panel, axis in zip(panels[:2], (1, 0)):
                projected = world[:, [axis, 2]]
                panel.add_patch(Polygon(projected[ConvexHull(projected).vertices], closed=True,
                                       facecolor=shape_color, edgecolor=shape_color, alpha=.55))
            if name == 'left_wrist_rgbd':
                local = vertices @ transform(mount['xyz'], mount['rpy'])[:3, :3].T + mount['xyz']
                projected = local[:, [2, 1]]
                panels[2].add_patch(Polygon(projected[ConvexHull(projected).vertices], closed=True,
                                           facecolor=shape_color, edgecolor=shape_color, alpha=.55))
        for panel, axis in zip(panels[:2], (1, 0)):
            panel.scatter(center[axis], center[2], s=35, color=color, label=name)
            panel.arrow(center[axis], center[2], forward[axis], forward[2], color=color, width=.002, length_includes_head=True)
        if name == 'left_wrist_rgbd':
            panels[2].scatter(mount['xyz'][2], mount['xyz'][1], color='red', s=40, label='Optical center')
            panels[2].arrow(mount['xyz'][2], mount['xyz'][1], 0, -.10, color='red', width=.001)
    for panel in panels:
        panel.set_aspect('equal'); panel.grid(alpha=.2)
    panels[0].set(title='Front: viewer at +X; zero FK', xlabel='Base +Y left (m)', ylabel='Base +Z up (m)')
    panels[1].set(title='Side: +X is robot front', xlabel='Base +X forward (m)')
    panels[2].set(title='Left wrist: outer side is local -Z', xlabel='Wrist local Z (m)', ylabel='Wrist local Y (m)')
    panels[0].legend(fontsize=7, loc='lower left')
    panels[2].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(output / 'reference_mount_geometry.png', dpi=170); plt.close(fig)


def audit_camera(name, mount, meshes, fk, mounts, columns, rows):
    camera = fk[mount['parent']] @ transform(mount['xyz'], mount['rpy'])
    cfg = yaml.safe_load((DESC / f'config/camera_{name}.yaml').read_text())
    horizontal = np.tan(cfg['horizontal_fov'] / 2)
    vertical = horizontal * cfg['height'] / cfg['width']
    objects = [(link, mesh, fk[link]) for link, mesh in meshes.items()]
    for other, m in mounts.items():
        if other + '_camera_link' in meshes:
            continue
        housing = Mesh(box_mesh(np.array(m['housing_size']), np.array(m['housing_center_camera_link'])))
        objects.append((other + '_housing', housing, fk[m['parent']] @ transform(m['xyz'], m['rpy'])))
    local_rays = []
    for v in np.linspace(-1, 1, rows):
        for u in np.linspace(-1, 1, columns):
            local_rays.append(np.array([1., -u * horizontal, -v * vertical]))
    ray_results = []
    for ray in local_rays:
        direction = camera[:3, :3] @ ray
        nearest, blocker = cfg['far_m'], None
        for link, mesh, world in objects:
            point = world[:3, :3].T @ (camera[:3, 3] - world[:3, 3])
            d = world[:3, :3].T @ direction
            distance = mesh.ray(point, d, nearest)
            if distance is not None:
                nearest, blocker = distance, link
        ray_results.append({'camera_x_depth_m': nearest if blocker else None, 'blocker': blocker})
    hits = [r for r in ray_results if r['blocker']]
    counts = {}
    for hit in hits:
        counts[hit['blocker']] = counts.get(hit['blocker'], 0) + 1
    center = ray_results[len(ray_results) // 2]
    return {'world_from_camera_link': camera.tolist(), 'world_forward': camera[:3, 0].tolist(),
            'world_up': camera[:3, 2].tolist(), 'ray_grid': [columns, rows], 'total_rays': len(ray_results),
            'hit_rays': len(hits), 'hit_fraction': len(hits) / len(ray_results),
            'center_ray': center, 'hits_before_80mm': sum(h['camera_x_depth_m'] < .08 for h in hits),
            'hits_before_250mm': sum(h['camera_x_depth_m'] < .25 for h in hits),
            'nearest_hit_depth_m': min((h['camera_x_depth_m'] for h in hits), default=None),
            'blocker_counts': counts, 'rays': ray_results,
            'interpretation': 'All physical visual meshes and all six opaque camera housings retained; wrist/gripper peripheral rays may be normal close-hand occlusion.'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, default=ROOT / 'runs/camera_reference_20260921')
    p.add_argument('--urdf', type=Path)
    p.add_argument('--mount-profile', type=Path, help='Expand actual source xacro with this reference overlay')
    p.add_argument('--columns', type=int, default=25)
    p.add_argument('--rows', type=int, default=15)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    urdf = args.urdf or args.output / ('robot_reference_mounts.urdf' if args.mount_profile else 'robot_without_cameras.urdf')
    if not args.urdf:
        import xacro
        source = (DESC / 'urdf/astribot_s1.xacro').read_text().replace('$(find astribot_s1_description)', str(DESC))
        doc = xacro.parse(io.StringIO(source))
        mappings = {'robot_name': 'astribot_s1', 'use_camera': 'true' if args.mount_profile else 'false', 'use_lidar': 'true', 'use_gripper': 'true'}
        if args.mount_profile:
            mappings.update(camera_mounts_profile=str(args.mount_profile.resolve()))
        xacro.process_doc(doc, mappings=mappings)
        urdf.write_text(doc.toxml())
    robot = ET.parse(urdf).getroot()
    meshes, metadata = load_geometry(robot)
    mounts = candidate_mounts(meshes)
    if args.mount_profile:
        for name, mount in mounts.items():
            joint = robot.find(f"joint[@name='{name}_camera_joint']")
            visual = robot.find(f"link[@name='{name}_camera_link']/visual")
            mount.update(parent=joint.find('parent').get('link'),
                         xyz=origin(joint.find('origin'))[:3, 3].tolist(),
                         rpy=np.fromstring(joint.find('origin').get('rpy'), sep=' ').tolist(),
                         housing_size=np.fromstring(visual.find('geometry/box').get('size'), sep=' ').tolist(),
                         housing_center_camera_link=origin(visual.find('origin'))[:3, 3].tolist())
    zero_fk = forward_kinematics(robot)
    for name, mount in mounts.items():
        contact_link = mount['parent']
        contact_mesh = meshes[contact_link]
        if 'wrist' in name and mount['xyz'][1] < -.05:
            # Close-up reference mounts on the fixed palm, not the rotating
            # fingers or the small arm_link_7 wrist shell.
            contact_link = f"astribot_gripper_{name.split('_')[0]}_base"
            relative = np.linalg.inv(zero_fk[mount['parent']]) @ zero_fk[contact_link]
            tri = meshes[contact_link].triangles @ relative[:3, :3].T + relative[:3, 3]
            contact_mesh = Mesh(tri)
        mount['support_bracket'] = support_bracket(mount, contact_mesh)
        mount['support_bracket']['contact_link'] = contact_link
        if args.mount_profile:
            actual = robot.find(f"link[@name='{name}_camera_link']/visual[@name='{name}_mount_bracket']")
            if actual is not None:
                mount['support_bracket']['suggested_size'] = mount['support_bracket']['size']
                mount['support_bracket']['suggested_center_camera_link'] = mount['support_bracket']['center_camera_link']
                mount['support_bracket']['size'] = np.fromstring(actual.find('geometry/box').get('size'), sep=' ').tolist()
                mount['support_bracket']['center_camera_link'] = origin(actual.find('origin'))[:3, 3].tolist()
                mount['support_bracket']['geometry_source'] = 'actual expanded xacro visual and collision'
    wrist_extrinsics = {}
    for side in ('left', 'right'):
        mount = mounts[f'{side}_wrist_rgbd']
        optical = zero_fk[mount['parent']] @ transform(mount['xyz'], mount['rpy']) @ transform(rpy=OPTICAL_RPY)
        arm = np.linalg.inv(zero_fk[f'astribot_arm_{side}_link_7']) @ optical
        grip = np.linalg.inv(zero_fk[f'astribot_gripper_{side}_base']) @ optical
        nominal = transform([0., -.06, .04])
        wrist_extrinsics[side] = {
            'arm_link_7_from_optical': arm.tolist(), 'gripper_base_from_optical': grip.tolist(),
            'nominal_gripper_base_from_optical': nominal.tolist(),
            'nominal_difference_translation_m': float(np.linalg.norm(grip[:3, 3] - nominal[:3, 3])),
            'nominal_difference_rotation_rad': float(Rotation.from_matrix(grip[:3, :3]).magnitude()),
            'interpretation': 'Both sides use negative gripper Y provisionally; photos do not identify side or establish calibration. Exact FK retains vendor fixed angle 1.5708.'}
    initial = {'source_photo': '/home/yjh/Pictures/20260921-165049.jpg',
               'photo_sha256': hashlib.sha256(Path('/home/yjh/Pictures/20260921-165049.jpg').read_bytes()).hexdigest(),
               'urdf_sha256': hashlib.sha256(urdf.read_bytes()).hexdigest(),
               'mount_profile': str(args.mount_profile) if args.mount_profile else None,
               'mount_profile_sha256': hashlib.sha256(args.mount_profile.read_bytes()).hexdigest() if args.mount_profile else None,
               'calibration_unchanged_sha256': hashlib.sha256((DESC / 'config/camera_calibration_robot.json').read_bytes()).hexdigest(),
               'mounts': mounts, 'zero_fk': {key: value.tolist() for key, value in zero_fk.items()},
               'wrist_reference_extrinsics': wrist_extrinsics,
               'primitive_visual_count': sum('primitive' in source for item in metadata for source in item['sources']),
               'mount_bracket_visual_count': sum('mount_bracket' in (source.get('visual_name') or '') for item in metadata for source in item['sources']),
               'meshes': metadata, 'physical_visual_mesh_count': len(meshes),
               'physical_triangle_count': sum(m['triangles'] for m in metadata),
               'front_proof': 'Zero FK maps torso_link_4 +x and head_link_2 +x to torso_base +x. Left wrist local -z maps outward +y; right wrist local -z maps outward -y.'}
    (args.output / 'candidate_mounts.json').write_text(json.dumps(initial, indent=2))
    clearance = housing_clearance(mounts, meshes, zero_fk)
    (args.output / 'housing_mesh_clearance.json').write_text(json.dumps(clearance, indent=2))
    (args.output / 'bracket_contacts.json').write_text(json.dumps(bracket_contacts(mounts, meshes, zero_fk), indent=2))
    plot_geometry(args.output, meshes, zero_fk, mounts)
    print(json.dumps({key: {field: value[field] for field in ('parent', 'xyz', 'rpy', 'housing_size')} for key, value in mounts.items()}, indent=2), flush=True)
    audits = {}
    for label, angle in (('zero_open', 0.), ('zero_half_closed', .465), ('zero_closed', .93)):
        fk = forward_kinematics(robot, {f'astribot_gripper_{side}_joint_L1': angle for side in ('left', 'right')})
        audits[label] = {}
        for name, mount in mounts.items():
            audits[label][name] = audit_camera(name, mount, meshes, fk, mounts, args.columns, args.rows)
            summary = {key: audits[label][name][key] for key in ('hit_rays', 'total_rays', 'center_ray', 'blocker_counts')}
            print(json.dumps({'pose': label, 'camera': name, **summary}), flush=True)
        (args.output / 'self_occlusion_audit.json').write_text(json.dumps(audits, indent=2))
    (args.output / 'README.md').write_text(
        '# Temporary photo-reference mount geometry\n\n'
        'Offline geometry only; no live robot/simulation changes and no measured calibration claim.\n'
        'camera_link is +x forward, +y left, +z up; the fixed optical rotation is [-pi/2,0,-pi/2].\n'
        'The housing offsets and sizes are recorded explicitly in candidate_mounts.json.\n'
        'All 42 robot visual meshes, including real articulated wrists and gripper fingers, are retained.\n'
        'Candidate front body mounts use sampled real STL surfaces over the full housing footprint, not only AABB maxima.\n'
        'The wrist side is selected from zero-pose FK, and the lens points along the gripper approach direction.\n'
        'The grid audit is evidence at the listed joint poses, not an all-pose occlusion guarantee.\n')


if __name__ == '__main__':
    main()
