"""Straight-corridor admission and local passage permits, independent of ROS.

Annotations describe physical boundaries. Live geometry and transport posture
must still be checked by the adapter. A permit never overrides a safety hold.
"""
from dataclasses import dataclass, replace
import math

from .behavior import Selection
from .contracts import finite, require


def angle(value):
    return math.remainder(value, 2 * math.pi)


@dataclass(frozen=True)
class Corridor:
    corridor_id: str
    entry: tuple
    exit: tuple
    width_m: float
    postures: tuple
    boundary_margin_m: float = .025
    tracking_margin_m: float = .05
    bidirectional: bool = True

    def __post_init__(self):
        require(isinstance(self.corridor_id, str) and bool(self.corridor_id), 'corridor.id')
        for point in (self.entry, self.exit):
            require(len(point) == 2, 'corridor.point')
            for value in point:finite(value, 'corridor.coordinate')
        require(self.length > .1, 'corridor.length')
        for name in ('width_m', 'boundary_margin_m', 'tracking_margin_m'):
            finite(getattr(self, name), 'corridor.' + name, 0)
        require(self.width_m > 0 and self.postures and
                all(isinstance(s, str) and s for s in self.postures), 'corridor.width/postures')
        require(type(self.bidirectional) is bool, 'corridor.bidirectional')

    @property
    def length(self):return math.dist(self.entry, self.exit)

    @property
    def heading(self):return math.atan2(self.exit[1]-self.entry[1], self.exit[0]-self.entry[0])

    def coordinates(self, x, y):
        dx, dy = x-self.entry[0], y-self.entry[1]
        c, s = math.cos(self.heading), math.sin(self.heading)
        return c*dx+s*dy, -s*dx+c*dy

    def point(self, longitudinal, lateral=0.):
        c, s = math.cos(self.heading), math.sin(self.heading)
        return (self.entry[0]+c*longitudinal-s*lateral,
                self.entry[1]+s*longitudinal+c*lateral)

    def reverse(self):
        return replace(self, entry=self.exit, exit=self.entry)


@dataclass(frozen=True)
class Passage:
    selection: Selection
    state: str = 'NORMAL'
    corridor_id: str = ''
    angular_cap: float = math.inf
    failure: str = ''
    permit: tuple = ()
    alignment_heading: float | None = None
    centering_target: tuple | None = None
    tracking_heading: float | None = None


class CorridorPolicy:
    def __init__(self, profile, corridors):
        self.profile = profile
        self.corridors = tuple(corridors)
        require(len({c.corridor_id for c in corridors}) == len(corridors), 'corridor.duplicate_id')
        self.active = None
        self.state = 'NORMAL'
        self.permit = ()
        self.clear_at = None
        self.wait_at = None
        self.last_time = None
        self.failure = ''
        self.task = None
        self.entry_stopping = False
        self.centering_target = None
        self.entry_started_at = None

    def margin(self, c):
        return (self.profile.clearance_margin_m + self.profile.payload_extra_margin_m +
                c.boundary_margin_m + c.tracking_margin_m)

    def half_projection(self, theta):
        p = self.profile
        return p.half_width_m*abs(math.cos(theta)) + p.half_length_m*abs(math.sin(theta))

    def route_fits(self, c, path):
        """Check segments, including sparse paths which cross the whole strip."""
        seen = False
        lateral_bound = c.width_m/2 - self.margin(c) - self.half_projection(self.profile.narrow_heading_limit_rad)
        for a, b in zip(path, path[1:]):
            sa, la = c.coordinates(*a);sb, lb = c.coordinates(*b)
            if max(sa, sb) < 0 or min(sa, sb) > c.length:continue
            if max(sa, sb)-min(sa, sb) < 1e-8:
                if 0 <= sa <= c.length:return False
                continue
            lo = max(0., min((0-sa)/(sb-sa), (c.length-sa)/(sb-sa)))
            hi = min(1., max((0-sa)/(sb-sa), (c.length-sa)/(sb-sa)))
            if lo > hi:continue
            seen = True
            if sb <= sa or abs(angle(math.atan2(b[1]-a[1], b[0]-a[0])-c.heading)) > self.profile.narrow_heading_limit_rad:
                return False
            if max(abs(la+(lb-la)*lo), abs(la+(lb-la)*hi)) > lateral_bound:return False
        return seen

    def choose(self, robot, path):
        choices = []
        p = self.profile
        for annotation in self.corridors:
            for c in (annotation, annotation.reverse()):
                s, lateral = c.coordinates(robot.x, robot.y)
                reach = p.stopping_distance(p.max_speed_m_s) + p.half_length_m + .8
                if not -reach <= s <= c.length+p.half_length_m+self.margin(c):continue
                if abs(lateral) > c.width_m/2+p.half_width_m:continue
                # Select a crossing in either direction, including prohibited travel.
                projected = [c.coordinates(*point) for point in path]
                crossing = any(max(a[0], b[0]) >= 0 and min(a[0], b[0]) <= c.length and
                    b[0] > a[0] and min(abs(a[1]), abs(b[1])) <= c.width_m/2
                    for a, b in zip(projected, projected[1:]))
                if crossing:choices.append((max(0., -s), c, c is annotation or annotation.bidirectional))
        return min(choices, key=lambda item: item[0])[1:] if choices else (None, True)

    def evaluate(self, selection, robot, path, version, posture, valid, geometry_clear, now, rotation_clear=False, centering_clear=False):
        finite(now, 'corridor.now', 0)
        task = (version.goal_id, version.clock_epoch)
        if self.task != task:
            self.active = None;self.state = 'NORMAL';self.permit = ()
            self.wait_at = None;self.clear_at = None;self.failure = '';self.task = task
            self.entry_stopping = False
            self.centering_target = None
            self.entry_started_at = None
        if self.last_time is not None and now < self.last_time:
            self.permit = ();self.clear_at = None;self.wait_at = now
        self.last_time = now
        if robot is None:
            return Passage(selection)
        if self.active is None:
            self.active, self.direction_allowed = self.choose(robot, path)
            if self.active is None:return Passage(selection)
            self.state = 'APPROACH';self.entry_stopping = False
            self.centering_target=None;self.entry_started_at=None
        c = self.active;p = self.profile
        s, lateral = c.coordinates(robot.x, robot.y)
        theta = abs(angle(robot.yaw-c.heading))
        margin = self.margin(c)
        rear = p.half_length_m*abs(math.cos(theta))+p.half_width_m*abs(math.sin(theta))
        binding = (c.corridor_id, c.entry, version.goal_id, version.path_revision,
                   version.map_epoch, version.localization_epoch, version.envelope_epoch, version.clock_epoch)
        if s-rear > c.length+margin:
            if self.state != 'EXIT':self.state = 'EXIT';self.clear_at = now
            if valid and now-self.clear_at >= p.clear_hold_s:
                self.active = None;self.state = 'NORMAL';self.permit = ();self.wait_at = None
                return Passage(selection)
            return self.result(selection, 'CORRIDOR_EXIT', p.narrow_speed_m_s)
        speed = math.hypot(robot.vx, robot.vy)
        stopped = speed <= p.planning_takeover['linear_speed_m_s'] and abs(robot.wz) <= p.planning_takeover['angular_speed_rad_s']
        rotation_radius = math.hypot(p.half_length_m, p.half_width_m)+margin
        # Reserve the whole rotation envelope before braking, including at zero speed.
        # The current heading projection can shrink below the space needed to align.
        prepare_distance = rotation_radius+p.stopping_distance(speed)
        body_outside = s+rear<0
        projected_side = abs(lateral)+self.half_projection(theta)+margin
        if self.permit and body_outside and s+rear+margin>=0 and projected_side>=c.width_m/2:
            self.permit=();self.entry_stopping=True
        preparing = not self.permit and (s >= -prepare_distance or self.entry_stopping)
        if preparing and self.entry_started_at is None:self.entry_started_at=now
        if (s<0 and self.entry_started_at is not None and
                now-self.entry_started_at>=p.planning_budget['episode_timeout_s']):
            self.failure='CORRIDOR_BLOCKED: ENTRANCE_PREPARATION_TIMEOUT'
        center_tolerance = min(p.narrow_centering_tolerance_m,
            max(0., (c.width_m/2-margin-self.half_projection(p.narrow_heading_limit_rad))/2))
        reason = ''
        if not valid:reason = 'CORRIDOR_INPUT_UNAVAILABLE'
        elif not self.direction_allowed:reason = 'CORRIDOR_DIRECTION_FORBIDDEN'
        elif posture not in c.postures:reason = 'CORRIDOR_POSTURE_UNCONFIRMED'
        elif not self.route_fits(c, path):reason = 'CORRIDOR_ROUTE_OUTSIDE_ENVELOPE'
        elif self.half_projection(p.narrow_heading_limit_rad)+margin >= c.width_m/2:reason = 'CORRIDOR_INSUFFICIENT_WIDTH'
        elif (s+rear+margin>=0 and not (s>c.length and rotation_clear) and
                not (body_outside and not self.permit and centering_clear) and
                projected_side>=c.width_m/2):reason = 'CORRIDOR_INSUFFICIENT_WIDTH'
        elif not geometry_clear:reason = 'CORRIDOR_GEOMETRY_UNAVAILABLE'
        elif selection.motion == 'HOLD':reason = selection.reason
        elif preparing and abs(lateral) > center_tolerance:reason = 'CORRIDOR_CENTERING_REQUIRED'
        elif preparing and theta > p.narrow_heading_limit_rad*(.5 if self.state=='ALIGN' else 1.):reason = 'CORRIDOR_ALIGNMENT_REQUIRED'
        if self.permit and self.permit != binding:
            self.permit = ();reason = 'CORRIDOR_PERMISSION_CHANGED'
        if self.failure:return self.hold(self.failure, now, selection)
        if reason=='CORRIDOR_CENTERING_REQUIRED' and not self.permit:
            self.entry_stopping=True
            if (body_outside and abs(lateral)<=p.narrow_centering_max_offset_m and
                    centering_clear and (stopped or self.centering_target is not None)):
                held=self.hold(reason,now,selection)
                if not held.failure:
                    if self.centering_target is None:self.centering_target=c.point(s)
                    self.state='CENTER';self.clear_at=None
                    return Passage(replace(selection,motion='CENTER',speed=p.narrow_centering_speed_m_s,
                        reason='CORRIDOR_CENTER'), 'CENTER',c.corridor_id,0.,
                        centering_target=self.centering_target)
        elif reason != 'CORRIDOR_CENTERING_REQUIRED':
            self.centering_target=None
        if reason=='CORRIDOR_ALIGNMENT_REQUIRED' and not self.permit:
            self.entry_stopping=True
            # A task may start closer than the preparation point. With the body still
            # outside, the adapter's full map/predicted-obstacle rotation sweep is
            # the clearance proof; crossing the entrance plane alone is not a wall.
            if speed<=p.planning_takeover['linear_speed_m_s'] and body_outside and rotation_clear:
                held=self.hold(reason,now,selection)
                if not held.failure:
                    self.state='ALIGN';self.clear_at=None
                    return Passage(replace(selection,motion='ALIGN',speed=p.narrow_speed_m_s,reason='CORRIDOR_ALIGN'),
                        'ALIGN',c.corridor_id,min(p.max_angular_speed_rad_s,p.narrow_angular_speed_rad_s),alignment_heading=c.heading)
        if reason:
            self.clear_at = None
            return self.hold(reason, now, selection)
        if self.state in ('APPROACH', 'WAIT') and s < -prepare_distance and not self.permit and not self.entry_stopping:
            self.state = 'APPROACH';self.wait_at = None
            return self.result(selection, 'CORRIDOR_APPROACH', p.narrow_speed_m_s)
        if not self.permit:
            # A restarted task inside a passage cannot acquire a new entrance permit.
            if s+rear+margin >= 0 and not (body_outside and rotation_clear):
                return self.hold('CORRIDOR_ENTRY_PERMISSION_MISSING', now, selection)
            self.state = 'PREPARE';self.entry_stopping = True
            if not stopped:self.clear_at = None
            elif self.clear_at is None:self.clear_at = now
            if self.clear_at is None or now-self.clear_at < p.clear_hold_s:
                return self.hold('CORRIDOR_PREPARE', now, selection, preserve_clear=True)
            self.permit = binding
        if self.state in ('WAIT', 'HOLD'):
            if self.clear_at is None:self.clear_at = now
            if now-self.clear_at < p.clear_hold_s:
                return self.hold('CORRIDOR_CLEAR_CONFIRMATION', now, selection, preserve_clear=True)
        self.state = 'TRANSIT';self.wait_at = None;self.clear_at = None
        return replace(self.result(selection, 'CORRIDOR_TRANSIT', p.narrow_speed_m_s),
                       tracking_heading=c.heading if s<=c.length else None)

    def result(self, selection, reason, cap):
        result = selection if selection.motion == 'HOLD' else replace(selection, motion='SLOW', speed=min(selection.speed, cap), reason=reason)
        return Passage(result, self.state, self.active.corridor_id,
                       min(self.profile.max_angular_speed_rad_s, self.profile.narrow_angular_speed_rad_s), permit=self.permit)

    def hold(self, reason, now, selection, preserve_clear=False):
        if self.wait_at is None:self.wait_at = now
        if not self.failure and now-self.wait_at >= self.profile.planning_budget['episode_timeout_s']:
            self.failure = 'CORRIDOR_BLOCKED: '+reason
        if not preserve_clear:self.state = 'HOLD' if self.permit else 'WAIT'
        return Passage(replace(selection, motion='HOLD', speed=0., reason=self.failure or reason),
                       self.state, self.active.corridor_id, 0., self.failure, self.permit)
