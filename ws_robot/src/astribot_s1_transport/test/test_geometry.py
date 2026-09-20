from types import SimpleNamespace as NS
import tempfile
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from astribot_s1_transport.geometry import collision_bounds, stl_bounds, collision_primitive_matrix


class GeometryTest(unittest.TestCase):
    def test_moveit_object_and_local_shape_poses_compose(self):
        def pose(xyz, quat):
            return NS(position=NS(**dict(zip(('x','y','z'), xyz))),
                      orientation=NS(**dict(zip(('x','y','z','w'), quat))))
        obj = NS(pose=pose([1., 2., 3.], Rotation.from_euler('z', np.pi/2).as_quat()),
                 primitive_poses=[pose([.4, 0., 0.], [0.,0.,0.,1.])])
        np.testing.assert_allclose(collision_primitive_matrix(obj)[:3,3], [1.,2.4,3.])

    def test_spherical_wheel_bounds_do_not_depend_on_rotation(self):
        urdf = '<robot><link name="wheel"><collision><geometry><sphere radius="0.08"/></geometry></collision></link></robot>'
        for angle in np.linspace(0., 2*np.pi, 13):
            transform = np.eye(4)
            transform[:3, :3] = Rotation.from_euler('xyz', [angle, .3, .7]).as_matrix()
            transform[:3, 3] = [.24, -.24, .08]
            bounds = collision_bounds(urdf, lambda _: transform, [.01]*3, np.eye(4), padding=0.)
            np.testing.assert_allclose(bounds, [.32, .32, .16], atol=1e-12)

    def test_tilted_cylinder_and_axial_spin(self):
        urdf = '<robot><link name="arm"><collision><geometry><cylinder radius="0.1" length="0.4"/></geometry></collision></link></robot>'
        for spin in (0., .7, 1.8):
            transform = np.eye(4)
            transform[:3, :3] = (Rotation.from_euler('y', np.pi/4)*Rotation.from_euler('z', spin)).as_matrix()
            bounds = collision_bounds(urdf, lambda _: transform, [.01]*3, np.eye(4), padding=0.)
            np.testing.assert_allclose(bounds, [.3/np.sqrt(2), .1, .3/np.sqrt(2)], atol=1e-12)

    def test_offset_and_rotation_are_included(self):
        urdf = '''<robot name="test"><link name="arm"><collision>
          <origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/>
          <geometry><box size="0.2 0.4 0.6"/></geometry>
          </collision></link></robot>'''
        x, y, z = collision_bounds(urdf, lambda _: np.eye(4), [.1]*3, np.eye(4), padding=0.)
        self.assertAlmostEqual(x, 1.2)
        self.assertAlmostEqual(y, .1)
        self.assertAlmostEqual(z, .3)

    def test_payload_expands_footprint(self):
        transform = np.eye(4); transform[1, 3] = .8
        _, y, _ = collision_bounds('<robot/>', lambda _: np.eye(4), [.2]*3, transform, padding=0.)
        self.assertAlmostEqual(y, .9)

    def test_obj_bounds(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mesh.obj'
            path.write_text('v -1 -2 -3\nv 4 5 6\nf 1 2 1\n')
            lower, upper = stl_bounds(str(path))
            np.testing.assert_equal(lower, [-1, -2, -3])
            np.testing.assert_equal(upper, [4, 5, 6])


if __name__ == '__main__': unittest.main()
