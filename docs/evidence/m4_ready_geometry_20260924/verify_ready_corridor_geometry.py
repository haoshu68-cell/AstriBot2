#!/usr/bin/env python3
"""Six offline route-geometry checks using a historical measured READY polygon.

No ROS, publisher, Hold, envelope acceptance, navigation_allowed, or time refresh.
Only FixedCorridorPolicy geometric methods and its native turn kernel are called.
"""
import argparse
import datetime
import hashlib
import json
import math
from pathlib import Path
import sys

REPO=Path('/home/yjh/WorkSpace/astribot_sdk_ros2')
GEOMETRY=Path('/home/yjh/WorkSpace/astribot_validation/READY_profile_20260924_1133/ready_scene01/ready_arm_geometry.json')
NATIVE=Path('/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/narrow_arms_20260923_1902/install_v2/astribot_s1_robot_geometry/local/lib/python3.10/dist-packages')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry',type=Path,default=GEOMETRY)
    parser.add_argument('--native-python-path',type=Path,default=NATIVE)
    parser.add_argument('--output',type=Path,default=Path(__file__).with_name('result.json'))
    args=parser.parse_args()
    sys.path[:0]=[str(REPO/'ws_robot/src/astribot_s1_navigation_policy'),str(args.native_python_path)]
    import numpy as np
    from astribot_s1_navigation_policy.profile import Profile
    from astribot_s1_navigation_policy.robot_envelope import FixedEnvelopeProfile
    from astribot_s1_navigation_policy.corridor import Corridor
    from astribot_s1_navigation_policy.fixed_corridor import FixedCorridorPolicy
    from astribot_s1_robot_geometry import _geometry_native
    from astribot_s1_robot_geometry.polygon import validate
    raw=json.loads(args.geometry.read_text());state=raw['geometry']
    reference=Path(raw['source'])
    reference_digest=hashlib.sha256(reference.read_bytes()).hexdigest()
    if reference_digest!=raw['source_sha256']:raise RuntimeError('Historical source file hash changed')
    polygon=np.array([(v['x'],v['y']) for v in state['reserved_footprint']['points']],dtype=float)
    validate(polygon)
    config=REPO/'ws_robot/src/astribot_s1_navigation_policy/config/simulation.json'
    baseline=Profile.load(config);profile=FixedEnvelopeProfile(baseline)
    # This field is the pure geometry-method input only; no V2 message is
    # accepted and no permission or fresh-state evidence is created.
    profile.footprint_xy=polygon
    policy=FixedCorridorPolicy(profile,[])
    template=Corridor('offline_ready',(0.,0.),(3.,0.),2.,('historical_ready',))
    margin=policy.margin(template)
    xmin,xmax,ymin,ymax=map(float,policy.support_bounds())
    span=ymax-ymin;required=span+2*margin;target=-(ymin+ymax)/2
    radius=float(np.max(np.hypot(polygon[:,0],polygon[:,1])))
    base_required=2*baseline.half_width_m+2*margin
    if not required>base_required:raise AssertionError('Measured full-arm width does not exceed configured base-width comparator')
    if abs(target)<.01:raise AssertionError('Measured footprint does not supply a meaningful asymmetric test')
    rows=[]
    def route_case(name,width,path,expected):
        corridor=Corridor(name,(0.,0.),(3.,0.),width,('historical_ready',))
        accepted=bool(policy.route_fits(corridor,path))
        kernel=bool(_geometry_native.corridor_turns_outside(polygon,
            np.asarray([corridor.coordinates(*p) for p in path],dtype=float),
            corridor.length,corridor.width_m,margin,baseline.narrow_heading_limit_rad))
        row=dict(name=name,width_m=width,path_xy_m=path,expected_route_fits=expected,
            actual_route_fits=accepted,passed=accepted==expected,
            turn_kernel_passed=kernel,route_failure=policy.route_failure,
            lateral_interval_m=list(map(float,policy.lateral_interval(corridor))),
            target_offset_m=float(policy.target_offset(corridor)))
        rows.append(row)
        return row
    wide=max(2.,required+.3)
    route_case('wide_straight',wide,[(-1.,0.),(4.,0.)],True)
    narrow=(base_required+required)/2
    row=route_case('base_only_fits_full_arm_does_not',narrow,[(-1.,target),(4.,target)],False)
    base_profile=FixedEnvelopeProfile(baseline)
    base_profile.footprint_xy=np.array([[-baseline.half_length_m,-baseline.half_width_m],
        [baseline.half_length_m,-baseline.half_width_m],[baseline.half_length_m,baseline.half_width_m],
        [-baseline.half_length_m,baseline.half_width_m]])
    base_policy=FixedCorridorPolicy(base_profile,[])
    base_corridor=Corridor('base_comparator',(0.,0.),(3.,0.),narrow,('configured_base_comparator',))
    row['configured_base_rectangle_route_fits']=bool(base_policy.route_fits(base_corridor,[(-1.,0.),(4.,0.)]))
    row['base_comparator_scope']='Configured nominal base rectangle, not a fabricated measured whole-body footprint'
    row['passed'] &= row['configured_base_rectangle_route_fits']
    centered_width=required+.04
    route_case('asymmetric_correct_center',centered_width,[(-1.,target),(4.,target)],True)
    route_case('opposite_center_offset_rejected',centered_width,[(-1.,-target),(4.,-target)],False)
    early=template.length+(-xmin)/2
    row=route_case('ninety_degree_turn_before_full_exit',wide,[(-1.,target),(early,target),(early,target+1.)],False)
    row['rear_support_x_at_turn_m']=early+xmin
    row['required_exit_support_x_m']=template.length+margin
    row['passed'] &= not row['turn_kernel_passed'] and row['route_failure']=='CORRIDOR_ROUTE_TURN_UNREACHABLE'
    late=template.length+margin+radius+.05
    row=route_case('ninety_degree_turn_after_full_sweep_exit',wide,[(-1.,target),(late,target),(late,target+1.)],True)
    row['rear_support_x_at_turn_m']=late+xmin
    row['full_rotation_radius_lower_x_m']=late-radius
    row['required_exit_support_x_m']=template.length+margin
    row['passed'] &= row['turn_kernel_passed'] and late-radius>template.length+margin
    inputs=[args.geometry,reference,config,Path(_geometry_native.__file__),Path(__file__),
        REPO/'ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/fixed_corridor.py',
        REPO/'ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/corridor.py',
        REPO/'ws_robot/src/astribot_s1_robot_geometry/include/astribot_s1_robot_geometry/corridor_turns.hpp']
    report=dict(passed=all(row['passed'] for row in rows),recorded_at=datetime.datetime.now().astimezone().isoformat(),
        scope='Offline hypothetical straight-passage geometry evaluated with a historical real READY reserved polygon. Not current Hold, installed envelope, sensor clearance, planner/costmap, motion or closed-loop passage acceptance.',
        added_coverage='Replaces synthetic rectangular test polygons with the 60-vertex measured nonhome READY reservation; exercises true observed asymmetry and native continuous corner rejection.',
        source_geometry_identity={k:state[k] for k in ('header','valid_until','source_id','sequence','clock_epoch','model_revision','attachment_revision','attachment_ids','complete','attachment_state_confirmed')},
        source_reference_sha256_matches=True,reserved_polygon_xy_m=polygon.tolist(),
        geometry=dict(vertex_count=len(polygon),x_min_m=xmin,x_max_m=xmax,y_min_m=ymin,y_max_m=ymax,
            reserved_width_m=span,margin_m=margin,margin_components=dict(clearance=baseline.clearance_margin_m,
            payload_extra=baseline.payload_extra_margin_m,boundary=template.boundary_margin_m,tracking=template.tracking_margin_m),
            aligned_minimum_annotated_width_m=required,nominal_base_with_same_margin_width_m=base_required,
            ideal_center_offset_m=target,max_vertex_radius_m=radius),
        cases=rows,sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs})
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(passed=report['passed'],cases=len(rows),geometry=report['geometry'],
        checks=[dict(name=row['name'],passed=row['passed'],width_m=row['width_m']) for row in rows],output=str(args.output)),indent=2))
    return 0 if report['passed'] else 1

if __name__=='__main__':sys.exit(main())
