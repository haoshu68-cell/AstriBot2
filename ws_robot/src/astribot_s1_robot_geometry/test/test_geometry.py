from pathlib import Path as _ReferencePath
import sys as _reference_sys
_reference_sys.path.insert(0, str(_ReferencePath(__file__).resolve().parents[4] / "tools/migration/python_reference"))
from reference_bootstrap import enable as _enable_references
_enable_references()

import math
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from astribot_s1_robot_geometry.model import RobotModel, Shape
from astribot_s1_robot_geometry.polygon import hull,inflate,contains,box_distance_many,validate
from astribot_s1_robot_geometry.state import JointSnapshot


URDF='''<robot name="fixture"><link name="base"><collision><geometry><box size="0.62 0.62 0.2"/></geometry></collision></link>
<joint name="arm" type="revolute"><parent link="base"/><child link="arm_link"/><origin xyz="0 0.3 0.5"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
<link name="arm_link"><collision><origin xyz="0.3 0 0"/><geometry><box size="0.6 0.06 0.06"/></geometry></collision></link></robot>'''


class PolygonTest(unittest.TestCase):
    def test_filled_polygon_contains_box_and_scan_point(self):
        p=[[-1,-1],[1,-1],[1,1],[-1,1]]
        self.assertLessEqual(float(box_distance_many(p,0,0,0,[-.1,-.1],[.1,.1])),0)
        self.assertLessEqual(float(box_distance_many(p,0,0,0,[.2,.3],[.2,.3])),0)
        self.assertAlmostEqual(float(box_distance_many(p,0,0,0,[2,2],[3,3])),math.sqrt(2))

    def test_asymmetric_hull_does_not_mirror_left_extension(self):
        p=hull([[-.31,-.31],[.31,-.31],[.31,.31],[0,.8],[-.31,.31]])
        self.assertGreater(float(box_distance_many(p,0,0,0,[0,-.6],[0,-.6])),.28)
        self.assertLessEqual(float(box_distance_many(p,0,0,0,[0,.6],[0,.6])),0)

    def test_inflation_is_outer_disk_not_inscribed_circle(self):
        p=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])
        inflated=inflate(p,.08)
        angles=np.linspace(0,2*math.pi,1000)
        for point in p:
            self.assertTrue(contains(inflated,point+.08*np.column_stack((np.cos(angles),np.sin(angles)))))

    def test_bad_polygon_rejected(self):
        for points in [[[0,0],[1,1],[0,1],[1,0]],[[0,0],[1,0],[.5,.1],[1,1],[0,1]],[[0,0],[1,0],[float('nan'),1]]]:
            with self.assertRaises(ValueError):validate(points)

    def test_fast_inflation_matches_exhaustive_minkowski_sum(self):
        rng=np.random.default_rng(71)
        for size in (4,8,32,128):
            for radius in (1e-6,.01,.08,.3):
                p=hull(rng.normal(size=(size,2)))
                angles=np.arange(32)*2*math.pi/32
                disk=radius/math.cos(math.pi/32)*np.column_stack((np.cos(angles),np.sin(angles)))
                reference=hull((p[:,None,:]+disk[None,:,:]).reshape(-1,2))
                actual=inflate(p,radius)
                self.assertTrue(contains(actual,reference,1e-9))
                self.assertTrue(contains(reference,actual,1e-9))

    def test_broadcast_matches_scalar(self):
        p=[[-.3,-.3],[.3,-.3],[.8,.2],[.3,.6],[-.3,.3]]
        rng=np.random.default_rng(6);x=rng.normal(size=(4,1));y=rng.normal(size=(4,1));theta=rng.normal(size=(4,1))
        low=rng.normal(size=(1,50,2));high=low+.1
        batch=box_distance_many(p,x,y,theta,low,high)
        for i in range(4):
            for j in range(50):self.assertAlmostEqual(batch[i,j],float(box_distance_many(p,x[i,0],y[i,0],theta[i,0],low[0,j],high[0,j])))


class ModelTest(unittest.TestCase):
    def test_nominal_shape_and_uncertainty_containment(self):
        model=RobotModel(URDF,'base')
        a=model.geometry({'arm':0.},{'arm':.02})
        self.assertTrue(contains(a['reserved'],a['physical']))
        for angle in np.linspace(-.02,.02,21):
            b=model.geometry({'arm':angle},{'arm':0.},model_padding=0.)
            self.assertTrue(contains(a['reserved'],b['physical']))
        self.assertTrue(all(s['z_max']>s['z_min'] for s in a['slices']))

    def test_sphere_and_tilted_cylinder_sample_containment(self):
        rng=np.random.default_rng(3)
        for kind,dims in [('sphere',[.12]),('cylinder',[.12,.5])]:
            transform=np.eye(4);transform[:3,:3]=Rotation.from_euler('xyz',[.5,.8,.3]).as_matrix()
            shape=Shape('base',kind,dims,np.eye(4));poly=shape.polygon(transform)
            if kind=='sphere':
                p=rng.normal(size=(2000,3));p*=.12/np.linalg.norm(p,axis=1)[:,None]
            else:
                theta=rng.uniform(0,2*math.pi,2000)
                p=np.column_stack((.12*np.cos(theta),.12*np.sin(theta),rng.uniform(-.25,.25,2000)))
            self.assertTrue(contains(poly,(p@transform[:3,:3].T)[:,:2]))

    def test_missing_joint_and_unsupported_shape_refuse(self):
        model=RobotModel(URDF,'base')
        with self.assertRaises(ValueError):model.geometry({}, {'arm':.003})
        with self.assertRaises(ValueError):RobotModel(URDF.replace('box size="0.62 0.62 0.2"','cone radius=".1"'),'base')

    def test_mimic_uses_source_and_inherits_error(self):
        urdf=URDF.replace('</robot>','''<joint name="finger" type="revolute"><parent link="arm_link"/><child link="tip"/><origin xyz="0.6 0 0"/><mimic joint="arm" multiplier="-2" offset="0.1"/></joint>
        <link name="tip"><collision><geometry><sphere radius=".03"/></geometry></collision></link></robot>''')
        model=RobotModel(urdf,'base')
        self.assertEqual(model.required,('arm',))
        self.assertAlmostEqual(model.joint_value('finger',{'arm':.2}),-.3)
        self.assertAlmostEqual(model.joint_value('finger',{'arm':.003},True),.006)


class StateTest(unittest.TestCase):
    def test_partial_updates_do_not_refresh_other_joints(self):
        state=JointSnapshot(['a','b'])
        state.update(['a','b'],[1.,2.],1_000_000_000,1_000_000_000)
        state.update(['a'],[1.1],1_400_000_000,1_400_000_000)
        with self.assertRaisesRegex(ValueError,'STALE'):state.snapshot(1_400_000_000)

    def test_missing_clock_rollback_and_duplicate(self):
        state=JointSnapshot(['a','b'])
        state.update(['a'],[1.],1_000_000_000,1_000_000_000)
        with self.assertRaisesRegex(ValueError,'INCOMPLETE'):state.snapshot(1_000_000_000)
        state.update(['b'],[2.],1_000_000_000,1_000_000_000)
        state.update(['a'],[9.],1_000_000_000,1_000_000_000)
        self.assertEqual(state.snapshot(1_000_000_000)[0]['a'],1.)
        with self.assertRaisesRegex(ValueError,'INCOMPLETE'):state.snapshot(500_000_000)
        self.assertEqual(state.epoch,1)

    def test_skew_and_future_rejected(self):
        state=JointSnapshot(['a','b'])
        state.update(['a'],[1.],900_000_000,1_000_000_000)
        state.update(['b'],[1.],1_050_000_000,1_050_000_000)
        with self.assertRaisesRegex(ValueError,'SKEW'):state.snapshot(1_050_000_000)
        with self.assertRaisesRegex(ValueError,'source'):state.update(['a'],[0.],2_000_000_000,1_050_000_000)


if __name__=='__main__':unittest.main()
