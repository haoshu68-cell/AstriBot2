from pathlib import Path as _ReferencePath
import sys as _reference_sys
_reference_sys.path.insert(0, str(_ReferencePath(__file__).resolve().parents[4] / "tools/migration/python_reference"))
from reference_bootstrap import enable as _enable_references
_enable_references()

import unittest
from astribot_s1_robot_geometry.projection_contract import projection_ceiling,SLICES

class ProjectionContractTests(unittest.TestCase):
    def setUp(self):
        self.values=dict(input_cloud_topic='/map_scan',base_frame='base',enable_outlier_filter=False)
        for name,low,high in zip(SLICES,(-.03,.17,.6,1.1),(.17,.6,1.1,2.2)):
            self.values.update({f'slices.{name}.enabled':True,f'slices.{name}.z_min':low,
                f'slices.{name}.z_max':high,f'slices.{name}.min_points':1})
    def test_full_height_pipeline(self):
        self.assertEqual(projection_ceiling(self.values,'base'),2.2)
    def test_old_clipped_pipeline_and_sparse_filter_cannot_claim_coverage(self):
        for key,value in [('input_cloud_topic','/map_scan_filtered'),('enable_outlier_filter',True),
                          ('base_frame','unrelated')]:
            with self.subTest(key=key),self.assertRaises(ValueError):
                projection_ceiling(dict(self.values,**{key:value}),'base')
    def test_missing_disabled_or_gapped_slice_is_rejected(self):
        for name in SLICES:
            for field,value in [('enabled',False),('z_min',None),('z_min',float('nan')),('z_min',8.),('min_points',2)]:
                with self.subTest(name=name,field=field,value=value),self.assertRaises(ValueError):
                    projection_ceiling(dict(self.values,**{f'slices.{name}.{field}':value}),'base')

if __name__=='__main__':unittest.main()
