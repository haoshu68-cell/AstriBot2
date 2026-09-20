"""Full-turn admission and bounded x-minus exit; no normal tracking control."""
import math
from dataclasses import dataclass
import numpy as np
from .continuous_sweep import motion_clearance


@dataclass(frozen=True)
class Maneuver:
    mode: str
    reason: str
    speed: float = 0.
    lateral: float = 0.
    angular: float = 0.


class ManeuverScene:
    def __init__(self, lower, upper, profile):
        self.lower=np.asarray(lower,dtype=float).reshape((-1,2))
        self.upper=np.asarray(upper,dtype=float).reshape((-1,2))
        self.profile=profile

    def clearance(self, pose, command, reserve=0.):
        if not len(self.lower):return float('inf')
        distance=math.hypot(*command[:2])
        radius=math.hypot(self.profile.half_length_m,self.profile.half_width_m)
        reach=distance+radius+self.profile.clearance_margin_m+self.profile.payload_extra_margin_m+reserve
        dx=np.maximum(np.maximum(self.lower[:,0]-pose[0],0.),pose[0]-self.upper[:,0])
        dy=np.maximum(np.maximum(self.lower[:,1]-pose[1],0.),pose[1]-self.upper[:,1])
        near=np.hypot(dx,dy)<=reach+1e-9
        if not np.any(near):return float('inf')
        gaps=motion_clearance(command,0.,1.,self.lower[near],self.upper[near],self.profile,pose)
        return float(np.min(gaps))

    def turn(self, pose, heading, reserve=0.):
        return self.clearance(pose,(0.,0.,math.remainder(heading-pose[2],2*math.pi)),reserve)


class StartManeuverPolicy:
    def __init__(self, profile):
        self.profile=profile;self.options=profile.start_maneuver
        self.reset()

    def reset(self):
        self.state='CHECK';self.anchor=None;self.target=None;self.started=None;self.origin=None
        self.progress_at=None;self.best=0.;self.clear_at=None;self.failure=''
        self.evidence={}

    def fail(self, reason):
        self.state='FAILED';self.failure='START_HEADING_UNREACHABLE: '+reason
        return Maneuver('FAILED',self.failure)

    def step(self, scene, robot, heading, now, valid, rear_covered, rotation_covered,
             turn_motion_safe=True):
        o=self.options
        if self.started is None:self.started=now;self.progress_at=now
        if now<self.started:return self.fail('CLOCK_RESET')
        if self.failure:return Maneuver('FAILED',self.failure)
        if now-self.started>o['timeout_s']:return self.fail('MANEUVER_TIMEOUT')
        if not valid or scene is None:
            self.clear_at=None
            return Maneuver('WAIT','START_MANEUVER_INPUT_UNAVAILABLE')
        pose=(robot.x,robot.y,robot.yaw)
        if self.origin is None:self.origin=pose
        stopped=math.hypot(robot.vx,robot.vy)<=o['stopped_linear_m_s'] and abs(robot.wz)<=o['stopped_angular_rad_s']
        if self.state in ('CHECK','TURN'):
            gap=scene.turn(pose,heading)
            self.evidence={'turn_clearance_m':gap if math.isfinite(gap) else None}
            if gap>0 and rotation_covered and turn_motion_safe:
                self.state='TURN'
                return Maneuver('TURN','START_FULL_TURN_CLEAR')
            if not rotation_covered:
                return Maneuver('WAIT','START_ROTATION_COVERAGE_UNAVAILABLE')
            self.evidence['turn_motion_safe']=turn_motion_safe
            self.state='PLAN_EXIT';self.clear_at=None
        if self.state=='PLAN_EXIT':
            if not stopped:return Maneuver('WAIT','START_STOP_BEFORE_REVERSE')
            if not rear_covered:return self.fail('REVERSE_COVERAGE_UNAVAILABLE')
            self.anchor=pose
            self.target=None
            budget=o['max_distance_m']-math.dist(self.origin[:2],pose[:2])
            for distance in np.arange(o['search_step_m'],budget+1e-9,o['search_step_m']):
                distance=float(distance)
                # The complete swept body, not only the center, must clear the exit.
                if scene.clearance(pose,(-distance,0.,0.))<=0:break
                target=(robot.x-distance*math.cos(robot.yaw),robot.y-distance*math.sin(robot.yaw),robot.yaw)
                early_distance=max(0.,distance-o['position_tolerance_m'])
                early=(robot.x-early_distance*math.cos(robot.yaw),robot.y-early_distance*math.sin(robot.yaw),robot.yaw)
                # The exit destination must also accommodate the already-allowed
                # position deviation. Current-pose turn admission still uses zero.
                reserve=o['lateral_tolerance_m']
                if scene.turn(target,heading,reserve)>reserve and scene.turn(early,heading,reserve)>reserve:
                    self.target=target;break
            if self.target is None:return self.fail('NO_SAFE_REVERSE_EXIT')
            self.state='REVERSE';self.best=0.;self.progress_at=now
            self.evidence.update(anchor=list(self.anchor),exit=list(self.target),distance_m=math.dist(pose[:2],self.target[:2]))
        if self.state in ('REVERSE','STOPPING'):
            ax,ay,yaw=self.anchor;c,s=math.cos(yaw),math.sin(yaw)
            along=-(robot.x-ax)*c-(robot.y-ay)*s
            side_error=-(robot.x-ax)*s+(robot.y-ay)*c
            side=abs(side_error)
            heading_error=abs(math.remainder(robot.yaw-yaw,2*math.pi))
            if side>o['lateral_tolerance_m'] or heading_error>o['heading_tolerance_rad']:
                return self.fail('REVERSE_EXIT_DRIFT')
            remaining=math.dist(self.anchor[:2],self.target[:2])-along
            if along<-o['position_tolerance_m'] or remaining<-o['position_tolerance_m']:
                return self.fail('REVERSE_EXIT_OVERSHOOT')
            if along>self.best+.005:self.best=along;self.progress_at=now
            if now-self.progress_at>o['progress_timeout_s']:return self.fail('REVERSE_EXIT_NO_PROGRESS')
            if not rear_covered:return self.fail('REVERSE_COVERAGE_UNAVAILABLE')
            if remaining<=o['position_tolerance_m']:self.state='STOPPING'
            if self.state=='STOPPING':
                if not stopped:return Maneuver('WAIT','REVERSE_EXIT_STOPPING')
                if scene.turn(pose,heading)<=0:
                    # Never rotate based only on the planned endpoint certificate.
                    self.state='PLAN_EXIT'
                    return Maneuver('WAIT','REVERSE_EXIT_RECHECK')
                if not rotation_covered:return Maneuver('WAIT','START_ROTATION_COVERAGE_UNAVAILABLE')
                if self.clear_at is None:self.clear_at=now
                if now-self.clear_at<self.profile.clear_hold_s:return Maneuver('WAIT','REVERSE_EXIT_CLEAR_CONFIRMATION')
                self.state='TURN'
                return Maneuver('TURN','REVERSE_EXIT_COMPLETE')
            # Recheck the remaining reverse segment and stopping envelope every observation.
            speed=min(o['speed_m_s'],max(0.,remaining)*o['position_gain'])
            # Correct the retreat line while translating; never request an in-place turn.
            delta=math.remainder(yaw-robot.yaw,2*math.pi)
            lateral=-o['lateral_gain']*side_error
            vx=-speed*math.cos(delta)-lateral*math.sin(delta)
            vy=-speed*math.sin(delta)+lateral*math.cos(delta)
            vy=max(-o['lateral_speed_m_s'],min(o['lateral_speed_m_s'],vy))
            ratio=min(1.,o['speed_m_s']/max(math.hypot(vx,vy),1e-12))
            vx*=ratio;vy*=ratio
            angular_cap=o['angular_speed_rad_s'] if robot.vx<-.003 else 0.
            wz=max(-angular_cap,min(angular_cap,o['heading_gain']*delta-o['angular_damping']*robot.wz))
            if vx>=0:return self.fail('REVERSE_DIRECTION_INVALID')
            if abs(wz)>.02 and not rotation_covered:
                self.clear_at=None
                return Maneuver('WAIT','REVERSE_CORRECTION_COVERAGE_UNAVAILABLE')
            horizon=self.profile.reaction_time_s+speed/self.profile.brake_deceleration_m_s2
            check=max(remaining,speed*horizon)
            if (scene.clearance(pose,(-check,0.,0.))<=0 or
                scene.clearance(pose,(vx*horizon,vy*horizon,wz*horizon))<=0):
                self.clear_at=None
                return Maneuver('WAIT','REVERSE_PATH_BLOCKED')
            if self.clear_at is None:self.clear_at=now
            if now-self.clear_at<self.profile.clear_hold_s:return Maneuver('WAIT','REVERSE_PATH_CLEAR_CONFIRMATION')
            return Maneuver('REVERSE','REVERSE_EXIT',-vx,vy,wz)
        return Maneuver('WAIT','START_MANEUVER_PENDING')
