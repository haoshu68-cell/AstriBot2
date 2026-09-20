"""Behavior-level social scoring; hard risk remains the existing envelope predictor."""
from dataclasses import dataclass, replace
import math
import time
import numpy as np
from .behavior import Selection
from .risk import evaluate_risk, path_position, remaining_path, box_clearance
from .contracts import Vec3


@dataclass(frozen=True)
class SocialDecision:
    selection: Selection
    state: str
    reason: str
    candidates: tuple = ()
    stationary_conflict: bool = False
    failure: str = ''
    processing_s: float = 0.
    goal_occupied: bool = False


def interaction_cost(world, robot, path, speed, profile, config):
    """SFW-inspired candidate-dependent social work, not a collision certificate.

    Integrate a short-term velocity relaxation and robot repulsion model. Human
    destination/behavior labels are deliberately absent from the input contract.
    """
    route=remaining_path(path,robot)
    dt=profile.prediction_step_s
    extent=math.hypot(profile.half_length_m,profile.half_width_m)
    total=0.
    for track in world.tracks:
        model=track.prediction_model
        px,py=track.geometry.center_m.x,track.geometry.center_m.y
        desired=np.array([model.velocity.x,model.velocity.y],dtype=float)
        velocity=desired.copy();position=np.array([px,py],dtype=float)
        radius=max(track.geometry.size_m.x,track.geometry.size_m.y)/2
        for step in range(1,math.ceil(profile.prediction_horizon_s/dt)+1):
            t=min(step*dt,profile.prediction_horizon_s)
            rx,ry,_=path_position(route,speed*t,robot)
            delta=position-np.array([rx,ry]);distance=float(np.linalg.norm(delta))
            normal=delta/max(distance,1e-6)
            gap=max(0.,distance-extent-radius)
            magnitude=config['force_strength']*math.exp(-gap/config['force_decay_m'])
            force=magnitude*normal
            # Symmetric work discourages forcing either party to change motion.
            total+=(magnitude+config['space_weight']*math.exp(-gap/config['comfort_decay_m']))*dt
            velocity+=((desired-velocity)/config['relaxation_s']+force)*dt
            norm=float(np.linalg.norm(velocity))
            if norm>profile.max_obstacle_speed_m_s:velocity*=profile.max_obstacle_speed_m_s/norm
            position+=velocity*dt
    return total


class SocialPolicy:
    def __init__(self,profile,config):
        self.profile=profile;self.config=config
        self.reset()

    def reset(self):
        self.last_time=None;self.clear_since=None;self.blocked_since=None
        self.held=False;self.episode=0;self.failure='';self.last_state='CRUISE';self.slow_since=None

    def select(self,base,world,robot,path,valid,now,invalid_reason='SOCIAL_INPUT_UNAVAILABLE',goal=None):
        start=time.perf_counter();p=self.profile;c=self.config
        if self.last_time is not None and now<self.last_time:self.reset()
        self.last_time=now
        if self.failure:
            return SocialDecision(replace(base,motion='HOLD',speed=0.,reason=self.failure),
                                  'FAILED',self.failure,failure=self.failure)
        candidates=[];stationary=False;goal_occupied=False
        if not valid or world is None or robot is None:
            selected=Selection('HOLD',0.,invalid_reason,episode=base.episode)
        else:
            if goal is not None:
                goal_occupied=any(box_clearance(*goal,track.geometry,p)<=0 for track in world.tracks)
            hard_tracks=[]
            for track in world.tracks:
                hard_tracks.append(replace(track,fused_track_id='moving:'+track.fused_track_id))
                if math.hypot(track.prediction_model.velocity.x,track.prediction_model.velocity.y)>1e-6:
                    hard_tracks.append(replace(track,fused_track_id='stopped:'+track.fused_track_id,
                        prediction_model=replace(track.prediction_model,velocity=Vec3(0.,0.,0.))))
            hard_world=replace(world,tracks=tuple(hard_tracks))
            stationary_path=((robot.x,robot.y),(robot.x,robot.y))
            risk=evaluate_risk(hard_world,robot,stationary_path,p)
            stationary=risk.blocked or risk.immediate or risk.uncertain
            if not world.tracks:
                selected=Selection('CONTINUE',p.max_speed_m_s,'SOCIAL_CLEAR',episode=base.episode)
            else:
                cap=min(p.max_speed_m_s,base.speed) if base.motion!='HOLD' else p.max_speed_m_s
                for name,speed in [('CONTINUE',cap),('SLOW',min(cap,p.narrow_speed_m_s))]:
                    risk=evaluate_risk(hard_world,robot,path,p,speed_limit=speed)
                    admissible=not (risk.blocked or risk.immediate or risk.uncertain)
                    work=interaction_cost(world,robot,path,speed,p,c) if admissible else None
                    cost=(work+c['progress_weight']*(p.max_speed_m_s-speed)/p.max_speed_m_s) if admissible else None
                    candidates.append({'motion':name,'speed':speed,'admissible':admissible,
                        'conflict_s':risk.conflict_time_s if math.isfinite(risk.conflict_time_s) else None,
                        'social_work':work,'cost':cost})
                feasible=[item for item in candidates if item['admissible']]
                if feasible:
                    best=min(feasible,key=lambda item:item['cost'])
                    slower=next((item for item in feasible if item['motion']=='SLOW'),None)
                    if best['motion']=='CONTINUE' and slower and self.slow_since is not None:
                        if (now-self.slow_since<c['minimum_slow_s'] or
                                slower['cost']-best['cost']<c['release_cost_margin']):
                            best=slower
                    selected=Selection(best['motion'],best['speed'],
                        'SOCIAL_SLOW' if best['motion']=='SLOW' else 'SOCIAL_CLEAR',episode=base.episode)
                else:selected=Selection('HOLD',0.,'STATIONARY_CONFLICT' if stationary else 'SOCIAL_YIELD',episode=base.episode)
        # Confirm social clearance in parallel with the scan policy. Applying
        # this timer after merging would count the same release twice.
        if selected.motion=='HOLD':
            self.clear_since=None;self.held=True
        elif self.held:
            if self.clear_since is None:self.clear_since=now
            if now-self.clear_since<c['clear_confirmation_s']:
                selected=replace(selected,motion='HOLD',speed=0.,reason='SOCIAL_CLEAR_CONFIRMATION')
            else:self.held=False;self.clear_since=None
        if selected.motion=='SLOW':
            if self.slow_since is None:self.slow_since=now
        else:self.slow_since=None
        # Social behavior only tightens the existing scan/path/coverage decision.
        if base.motion=='HOLD':selected=base
        elif selected.motion!='HOLD' and (base.speed<selected.speed or
                (base.speed==selected.speed and base.motion=='SLOW')):selected=base
        if selected.motion=='HOLD':
            if self.blocked_since is None:self.blocked_since=now;self.episode+=1
            if now-self.blocked_since>=c['wait_timeout_s']:
                self.failure=('SOCIAL_INPUT_TIMEOUT' if not valid else
                              'NO_SAFE_MANEUVER' if stationary else
                              'GOAL_OCCUPIED' if goal_occupied else 'SOCIAL_BLOCKED_TIMEOUT')
                selected=replace(selected,reason=self.failure)
        else:self.blocked_since=None
        state=('FAILED' if self.failure else 'WAIT' if selected.motion=='HOLD' else
               'SOCIAL_SLOW' if selected.motion=='SLOW' else 'CRUISE')
        self.last_state=state
        return SocialDecision(selected,state,selected.reason,tuple(candidates),stationary,self.failure,
                              time.perf_counter()-start,goal_occupied)
