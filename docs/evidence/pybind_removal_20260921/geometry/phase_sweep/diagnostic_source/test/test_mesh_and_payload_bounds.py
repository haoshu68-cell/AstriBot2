"""Sample actual surfaces across rotations, offsets and joint uncertainty."""
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from astribot_s1_robot_geometry.model import RobotModel,Shape,mesh_bounds
from astribot_s1_robot_geometry.polygon import contains

URDF='''<robot name="fixture"><link name="base"><collision><geometry><box size=".62 .62 .2"/></geometry></collision></link>
<joint name="lift" type="prismatic"><parent link="base"/><child link="tower"/><axis xyz="0 0 1"/><limit lower="0" upper="1"/></joint>
<link name="tower"/><joint name="arm" type="revolute"><parent link="tower"/><child link="wrist"/><origin xyz=".1 .2 .7"/><axis xyz="0 1 0"/><limit lower="-2" upper="2"/></joint>
<link name="wrist"><collision><origin xyz=".4 0 0"/><geometry><box size=".8 .06 .06"/></geometry></collision></link></robot>'''


class GeometryRobustness(unittest.TestCase):
    def test_obj_whitespace_vertices_cannot_be_silently_omitted(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'payload.obj'
            path.write_text('v 0 0 0\nv 1 1 1\n  v -2 .2 .4\n\tv .3 3 -.5 # extent\n')
            low,high,_=mesh_bounds(str(path))
            np.testing.assert_allclose(low,[-2,0,-.5])
            np.testing.assert_allclose(high,[1,3,1])

    def test_rotated_scaled_mesh_bounds_include_every_vertex(self):
        rng=np.random.default_rng(270)
        vertices=rng.uniform([-2,-.3,-.1],[.4,.5,.9],size=(100,3))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'fixture.obj';path.write_text('\n'.join('v '+' '.join(map(str,p)) for p in vertices))
            urdf=URDF.replace('<box size=".8 .06 .06"/>',f'<mesh filename="{path}" scale="-.4 .7 1.1"/>')
            model=RobotModel(urdf,'base')
            for angle in np.linspace(-1.8,1.8,11):
                q={'arm':angle,'lift':.2};g=model.geometry(q,{'arm':.003,'lift':.003})
                transform=model.fk(q)('wrist');points=vertices*[-.4,.7,1.1]+[.4,0,0]
                actual=points@transform[:3,:3].T+transform[:3,3]
                self.assertTrue(contains(g['physical'],actual[:,:2]))
                self.assertGreaterEqual(g['height'],actual[:,2].max())

    def test_offset_payloads_and_joint_errors_remain_inside_reserved_hull(self):
        rng=np.random.default_rng(991);model=RobotModel(URDF,'base')
        for kind,dims in [('box',[.5,.2,.3]),('sphere',[.18]),('cylinder',[.14,.55])]:
            for i in range(10):
                pose=np.eye(4);pose[:3,:3]=Rotation.random(random_state=rng).as_matrix()
                pose[:3,3]=rng.uniform([.2,-.5,-.1],[.9,.5,.2])
                shape=Shape('wrist',kind,dims,pose);q={'arm':float(rng.uniform(-1.7,1.7)),'lift':.2}
                reserved=model.geometry(q,{'arm':.01,'lift':.006},[shape])
                for da,dl in [(-.01,-.006),(-.01,.006),(.01,-.006),(.01,.006),(0.,0.)]:
                    actual=model.geometry({'arm':q['arm']+da,'lift':q['lift']+dl},{'arm':0.,'lift':0.},[shape],model_padding=0.)
                    self.assertTrue(contains(reserved['reserved'],actual['physical'],1e-9))
                    self.assertGreaterEqual(reserved['height']+1e-10,actual['height'])

    def test_missing_joint_and_unbounded_hold_error_refuse(self):
        model=RobotModel(URDF,'base')
        for q,error in [({'arm':0.},{'arm':.003,'lift':.003}),
                        ({'arm':0.,'lift':.2},{'arm':float('nan'),'lift':.003}),
                        ({'arm':0.,'lift':.2},{'arm':.101,'lift':.003})]:
            with self.assertRaises(ValueError):model.geometry(q,error)


if __name__=='__main__':unittest.main()
