"""Explicit straight-corridor intent and independently sampled traversal proof."""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class CorridorRoute:
    route_id: str
    frame_id: str
    entry: tuple
    exit: tuple
    width_m: float
    approach_m: float = .8
    departure_m: float = .8

    def __post_init__(self):
        numbers=(*self.entry,*self.exit,self.width_m,self.approach_m,self.departure_m)
        if (not self.route_id or not self.frame_id or len(self.entry)!=2 or len(self.exit)!=2 or
            not all(math.isfinite(v) for v in numbers) or self.length<.1 or
            min(self.width_m,self.approach_m,self.departure_m)<=0):
            raise ValueError('INVALID_CORRIDOR_INTENT')

    @property
    def length(self):return math.dist(self.entry,self.exit)

    @property
    def yaw(self):return math.atan2(self.exit[1]-self.entry[1],self.exit[0]-self.entry[0])

    def pose(self,s):
        return [self.entry[0]+s*math.cos(self.yaw),self.entry[1]+s*math.sin(self.yaw),self.yaw]

    def coordinates(self,x,y):
        dx,dy=x-self.entry[0],y-self.entry[1]
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        return dx*c+dy*s,-dx*s+dy*c

    def via_poses(self):
        # Closely spaced, explicit in-corridor goals prevent a low-inflation
        # route around both walls from satisfying only an outside exit goal.
        count=max(2,math.ceil(self.length/.4))
        return [self.pose(self.length*i/count) for i in range(count+1)]+[self.pose(self.length+self.departure_m)]

    def behavior_tree(self):
        root=ET.Element('root',main_tree_to_execute='MainTree')
        tree=ET.SubElement(root,'BehaviorTree',ID='MainTree')
        execution=ET.SubElement(tree,'PolicyExecution',session='{policy_session}',goals='{goals}')
        sequence=ET.SubElement(execution,'ReactiveSequence')
        fallback=ET.SubElement(sequence,'ReactiveFallback')
        ET.SubElement(fallback,'KeepSafePath',session='{policy_session}',path='{path}',goals='{goals}',remaining_goals='{remaining_goals}')
        ET.SubElement(fallback,'ComputePathThroughPoses',goals='{remaining_goals}',path='{path}',planner_id='GridBased')
        ET.SubElement(sequence,'RequireCorridorRoute',session='{policy_session}',path='{path}',frame=self.frame_id,
            entry_x=str(self.entry[0]),entry_y=str(self.entry[1]),exit_x=str(self.exit[0]),exit_y=str(self.exit[1]),width=str(self.width_m))
        # The canonical 0.7 m radius can consume an inside goal before entry.
        ET.SubElement(sequence,'RemovePassedGoals',input_goals='{remaining_goals}',output_goals='{remaining_goals}',
            radius='0.10',global_frame=self.frame_id,robot_base_frame='astribot_torso_base')
        ET.SubElement(sequence,'FollowPath',path='{path}',controller_id='FollowPath',goal_checker_id='precise_goal_checker')
        return ET.tostring(root,encoding='unicode')+'\n'


class TraversalWitness:
    """Fail on observation gaps/teleports/side exits; arrival alone is no proof."""
    def __init__(self,route,max_speed=.2,max_gap=.3):
        self.route=route;self.max_speed=max_speed;self.max_gap=max_gap
        self.previous=None;self.entered=False;self.exited=False;self.samples=0
        self.max_abs_lateral=0.

    def observe(self,stamp,x,y):
        if not all(math.isfinite(v) for v in (stamp,x,y)) or stamp<=0:
            raise ValueError('CORRIDOR_WITNESS_INVALID')
        s,lateral=self.route.coordinates(x,y)
        if self.previous is not None:
            t0,x0,y0,s0=self.previous
            dt=stamp-t0
            if dt==0:
                if math.hypot(x-x0,y-y0)>1e-9:raise ValueError('CORRIDOR_WITNESS_CONFLICTING_SAMPLE')
                return
            if dt<0 or dt>self.max_gap:raise ValueError('CORRIDOR_WITNESS_TIME_GAP')
            if math.hypot(x-x0,y-y0)>self.max_speed*dt+.02:
                raise ValueError('CORRIDOR_WITNESS_POSE_JUMP')
            # Include boundary crossings between samples, not only samples
            # whose centers already lie inside the corridor.
            _,l0=self.route.coordinates(x0,y0)
            if s!=s0:
                for edge in (0.,self.route.length):
                    t=(edge-s0)/(s-s0)
                    if 0<=t<=1 and abs(l0+t*(lateral-l0))>=self.route.width_m/2:
                        raise ValueError('ACTUAL_ROUTE_BYPASSES_CORRIDOR')
            if s0<0<=s:self.entered=True
            if s0<self.route.length<=s and self.entered:self.exited=True
        elif s>=0:
            raise ValueError('CORRIDOR_WITNESS_MISSING_ENTRY')
        if 0<=s<=self.route.length:
            self.max_abs_lateral=max(self.max_abs_lateral,abs(lateral))
            if abs(lateral)>=self.route.width_m/2:raise ValueError('ACTUAL_ROUTE_BYPASSES_CORRIDOR')
        self.samples+=1;self.previous=(stamp,x,y,s)

    def result(self):
        if not self.entered or not self.exited:raise ValueError('CORRIDOR_TRAVERSAL_NOT_WITNESSED')
        return dict(route_id=self.route.route_id,samples=self.samples,max_abs_lateral_m=self.max_abs_lateral,
                    entered=self.entered,exited=self.exited,pose_source='capture_time_map_to_base_tf')
