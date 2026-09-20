#!/usr/bin/env python3
"""Offline scenario matrix; does not start ROS, publish commands or certify motion."""
import argparse
import json
import math
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
for package in ('astribot_s1_navigation_policy','astribot_s1_robot_geometry'):
    sys.path.insert(0,str(ROOT/'ws_robot/src'/package))
from astribot_s1_robot_geometry.polygon import hull,projection,inflate
from types import SimpleNamespace
from astribot_s1_navigation_policy.profile import Profile
from astribot_s1_navigation_policy.robot_envelope import FixedEnvelopeProfile
from astribot_s1_navigation_policy.corridor import Corridor
from astribot_s1_navigation_policy.fixed_corridor import FixedCorridorPolicy
from astribot_s1_navigation_policy.behavior import Selection


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    shapes=dict(compact=[[-.32,-.32],[.32,-.32],[.32,.32],[-.32,.32]],
        forward=[[-.32,-.32],[1.,-.32],[1.,.32],[-.32,.32]],
        side=[[-.32,-.32],[.32,-.32],[.32,.65],[-.32,.65]],
        asymmetric=[[-.32,-.20],[.32,-.20],[.32,.44],[-.32,.44]])
    rows=[]
    for width in (.85,.95,1.,1.1,1.3):
        for name,vertices in shapes.items():
            for angle in (0.,2.5,5.,15.,90.):
                for reverse in (False,True):
                    theta=math.radians(angle)+(math.pi if reverse else 0.)
                    xmin,xmax,ymin,ymax=projection(hull(vertices),theta)
                    # Fixture values only: 8 cm clearance + 1 cm known boundary
                    # bound + 1 cm tracking bound on each side.
                    low=-width/2-ymin+.10;high=width/2-ymax-.10
                    rows.append(dict(width_m=width,posture=name,heading_deg=angle,reverse=reverse,
                        lateral_min_m=low,lateral_max_m=high,target_offset_m=(low+high)/2,
                        geometrically_fits=low<=high,rotation_displacement_at_half_5deg_m=max(math.hypot(*v) for v in vertices)*math.radians(2.5),
                        in_passage_in_place_rotation_allowed=False,
                        execution_policy='SEPARATE_REVERSE_SWEEP_REQUIRED' if reverse else ('ALIGNED_PATH_SWEEP_REQUIRED' if angle<=2.5 else 'ALIGN_OUTSIDE_REQUIRED')))
    decisions=[]
    version=SimpleNamespace(goal_id='matrix',path_revision=1,map_epoch=1,
        localization_epoch=1,envelope_epoch=1,clock_epoch=1)
    baseline=Profile.load(ROOT/'ws_robot/src/astribot_s1_navigation_policy/config/simulation.json')
    for width in (.85,.95,1.,1.1,1.3):
        for name,vertices in shapes.items():
            for degrees in (0.,2.5,5.,15.,90.):
                for reverse in (False,True):
                    for inside in (False,True):
                        for valid,clear in ((True,True),(False,True),(True,False)):
                            profile=FixedEnvelopeProfile(baseline);profile.footprint_xy=hull(vertices)
                            physical=Corridor('matrix',(0.,0.),(3.,0.),width,('any_measured_posture',),.025,.05)
                            corridor=physical.reverse() if reverse else physical
                            policy=FixedCorridorPolicy(profile,[physical])
                            offset=policy.target_offset(corridor)
                            start=corridor.point(1. if inside else -1.5,offset)
                            end=corridor.point(4.,offset)
                            robot=SimpleNamespace(x=start[0],y=start[1],yaw=corridor.heading+math.radians(degrees),vx=0.,vy=0.,wz=0.)
                            result=policy.evaluate(Selection('PROCEED',.2,'CLEAR'),robot,(start,end),version,
                                'label_must_not_change_geometry',valid,clear,1.,True,True)
                            row=dict(width_m=width,posture=name,heading_error_deg=degrees,reverse_route=reverse,
                                inside=inside,inputs_valid=valid,geometry_clear=clear,state=result.state,
                                motion=result.selection.motion,reason=result.selection.reason)
                            # Safety invariants are independent of a nominal posture label.
                            if result.state=='TRANSIT':
                                low,high=policy.lateral_interval(corridor,math.radians(degrees))
                                assert valid and clear and low<=offset<=high and math.radians(degrees)<=profile.narrow_heading_limit_rad
                            if inside:assert result.state not in ('ALIGN','CENTER')
                            if inside and not (valid and clear):assert result.selection.motion=='HOLD'
                            if valid and clear and degrees==0.:
                                low,high=policy.lateral_interval(corridor)
                                if low<=high:assert result.state=='TRANSIT'
                            decisions.append(row)
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(dict(evidence='offline_fixture_geometry_only',clearance_per_side_m=.08,
        boundary_per_side_m=.01,tracking_per_side_m=.01,rows=rows,
        policy_evidence='production FixedCorridorPolicy with synthetic shapes; no execution claim',
        policy_boundary_per_side_m=.025,policy_tracking_per_side_m=.05,policy_cases=decisions),indent=2)+'\n')
    print(json.dumps(dict(cases=len(rows),policy_cases=len(decisions),
        geometrically_fits=sum(r['geometrically_fits'] for r in rows),output=str(output))))
if __name__=='__main__':main()
