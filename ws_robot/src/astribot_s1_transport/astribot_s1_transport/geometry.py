"""Conservative carried footprint from the live URDF collision primitives and FK."""
import itertools
import struct
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .core import TaskFailure


def box_corners(size, transform):
    half = np.asarray(size, dtype=float) / 2
    corners = np.array([list(np.array(sign) * half) + [1.]
                        for sign in itertools.product((-1., 1.), repeat=3)])
    return (transform @ corners.T).T[:, :3]


def stl_bounds(filename):
    if filename.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        package, relative = filename[len('package://'):].split('/', 1)
        filename = str(Path(get_package_share_directory(package)) / relative)
    data = Path(filename).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    if Path(filename).suffix.lower() == '.obj':
        vertices = np.array([list(map(float, line.split()[1:4]))
                             for line in data.decode('utf-8').splitlines()
                             if line.startswith('v ')])
    elif count and len(data) == 84 + 50 * count:
        triangles = np.frombuffer(data, dtype=np.dtype([
            ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]), offset=84)
        vertices = triangles['vertices'].reshape(-1, 3)
    else:
        vertices = np.array([list(map(float, line.split()[1:]))
                             for line in data.decode('ascii').splitlines()
                             if line.strip().startswith('vertex ')])
    if vertices.size == 0 or not np.isfinite(vertices).all():
        raise TaskFailure('INVALID_COLLISION_MESH:' + filename)
    return vertices.min(axis=0), vertices.max(axis=0)


def collision_bounds(urdf, lookup, payload_size, payload_transform, padding=.01):
    points = []
    for link in ET.fromstring(urdf).findall('link'):
        collisions = link.findall('collision')
        if not collisions:
            continue
        link_transform = lookup(link.attrib['name'])
        for collision in collisions:
            origin = collision.find('origin')
            transform = np.eye(4)
            if origin is not None:
                transform[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
                transform[:3, :3] = Rotation.from_euler('xyz', list(map(float, origin.get('rpy', '0 0 0').split()))).as_matrix()
            geometry = list(collision.find('geometry'))[0]
            shape_transform = link_transform @ transform
            if geometry.tag == 'box':
                size = list(map(float, geometry.attrib['size'].split()))
            elif geometry.tag == 'cylinder':
                radius = float(geometry.attrib['radius'])
                axis = shape_transform[:3, 2]
                extent = radius * np.sqrt(np.maximum(0., 1.-axis*axis))
                extent += abs(axis) * float(geometry.attrib['length']) / 2
                points.extend((shape_transform[:3, 3]-extent, shape_transform[:3, 3]+extent))
                continue
            elif geometry.tag == 'sphere':
                # A sphere's occupied space must not grow as a wheel rotates.
                radius = float(geometry.attrib['radius'])
                points.extend((shape_transform[:3, 3]-radius, shape_transform[:3, 3]+radius))
                continue
            elif geometry.tag == 'mesh':
                lower, upper = stl_bounds(geometry.attrib['filename'])
                scale = np.array(list(map(float, geometry.get('scale', '1 1 1').split())))
                center = (lower + upper) / 2 * scale
                size = (upper - lower) * np.abs(scale)
                transform[:3, 3] += transform[:3, :3] @ center
            else:
                raise TaskFailure('UNSUPPORTED_COLLISION_GEOMETRY:' + geometry.tag)
            points.extend(box_corners(size, link_transform @ transform))
    points.extend(box_corners(payload_size, payload_transform))
    points = np.array(points)
    return (float(np.max(np.abs(points[:, 0]))) + padding,
            float(np.max(np.abs(points[:, 1]))) + padding,
            float(np.max(points[:, 2])) + padding)


def collision_primitive_matrix(obj):
    """MoveIt serializes world shape transforms as object pose * local shape pose."""
    def pose_matrix(pose):
        p, q = pose.position, pose.orientation
        value = np.eye(4)
        value[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        value[:3, 3] = [p.x, p.y, p.z]
        return value
    return pose_matrix(obj.pose) @ pose_matrix(obj.primitive_poses[0])
