#!/usr/bin/env python3
"""Deterministic offline equivalence/timing; does not assert ROS deadlines."""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
from astribot_s1_robot_geometry._geometry_native import scan_boxes_free
from astribot_s1_robot_geometry.polygon import hull, box_distance_many, _box_distance_many_reference
from astribot_s1_navigation_policy.scan_occupancy import angular_box_free


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True)
    args=parser.parse_args();rng=np.random.default_rng(619)
    polygon=hull(rng.normal(size=(150,2)));results=[]
    for n in (500,2117,10000):
        boxes=np.column_stack((rng.uniform(-4,4,(n,2)),np.full((n,2),.05)))
        ranges=np.full(1440,math.inf);ranges[::37]=math.nan;ranges[::29]=2.
        params=(.1,10.,-math.pi,2*math.pi/1440,.05)
        def reference_free():
            return np.asarray([angular_box_free(
                [(x+dx*sx/2,y+dy*sy/2) for dx,dy in ((-1,-1),(-1,1),(1,-1),(1,1))],
                ranges,*params) for x,y,sx,sy in boxes])
        def native_free():return scan_boxes_free(boxes,[0,0,0,0,0,0,1],ranges,*params)
        poses=rng.uniform(-2,2,(n,3));lo=boxes[:,:2]-.025;hi=boxes[:,:2]+.025
        def reference_distance():return _box_distance_many_reference(polygon,*poses.T,lo,hi)
        def native_distance():return box_distance_many(polygon,*poses.T,lo,hi)
        np.testing.assert_array_equal(reference_free(),native_free())
        np.testing.assert_allclose(reference_distance(),native_distance(),atol=2e-12,rtol=2e-12)
        timing={}
        for name,fn in [('python_free',reference_free),('cpp_free',native_free),
                        ('numpy_polygon',reference_distance),('cpp_polygon',native_distance)]:
            elapsed=[]
            for _ in range(25):
                begin=time.perf_counter();fn();elapsed.append(time.perf_counter()-begin)
            timing[name]={'p50_ms':float(np.percentile(elapsed,50)*1000),
                          'p95_ms':float(np.percentile(elapsed,95)*1000)}
        results.append(dict(boxes=n,equivalent=True,timing=timing))
    result=dict(evidence='offline same-input kernel comparison; not end-to-end deadline acceptance',results=results)
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
