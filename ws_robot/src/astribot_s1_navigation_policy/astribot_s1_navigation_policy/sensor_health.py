"""Acquisition-time health and directional depth coverage, independent of ROS."""
import math
import os
from numbers import Real
from dataclasses import replace
from .contracts import Health, SensorHealth, Stamp, BearingCone, Vec3


_native = None
if os.environ.get('ASTRIBOT_NAV_NATIVE_KERNELS', '').lower() in ('1', 'true', 'yes'):
    try:
        from astribot_s1_navigation_policy_native import _navigation_math_native as _native
    except ImportError:  # pragma: no cover - Python-only overlays remain supported
        _native = None


class SensorHealthRegistry:
    def __init__(self, timeout_s, required=('scan',)):
        self.timeout_ns=int(timeout_s*1e9)
        self.required=set(required)
        self.records={}
        self.clock=None

    def record(self, sensor, capture, now, frame, coverage, depth, calibration_epoch):
        if (capture.clock,capture.epoch)!=(now.clock,now.epoch):return False
        clock=(now.clock,now.epoch)
        if clock!=self.clock:self.records.clear();self.clock=clock
        old=self.records.get(sensor)
        if old and (capture.ns<=old.stamp.ns or calibration_epoch<old.calibration_epoch):return False
        self.records[sensor]=SensorHealth(sensor,Health.VALID if coverage else Health.DEGRADED,
            capture,Stamp(capture.ns+self.timeout_ns,capture.clock,capture.epoch),frame,
            tuple(coverage),depth,calibration_epoch,'VALID' if coverage else 'NO_DECLARED_COVERAGE')
        return True

    def health(self, now):
        out=[]
        for sensor in sorted(set(self.records)|self.required):
            item=self.records.get(sensor)
            if item is None or (item.stamp.clock,item.stamp.epoch)!=(now.clock,now.epoch):
                out.append(SensorHealth(sensor,Health.UNAVAILABLE,now,
                    Stamp(now.ns+self.timeout_ns,now.clock,now.epoch),'unknown',(),False,0,'NO_CURRENT_DATA'))
            else:out.append(item)
        return tuple(out)

    def required_valid(self, now):
        return all(h.health==Health.VALID for h in self.health(now) if h.sensor_id in self.required)

    def allows_motion(self, now, vx, vy, wz):
        if not self.required_valid(now):return False
        cones=[c for h in self.health(now) if h.health==Health.VALID and h.depth_available for c in h.coverage]
        return coverage_allows_motion(cones,vx,vy,wz)

    def allows(self, now, directions):
        if not self.required_valid(now):return False
        cones=[c for h in self.health(now) if h.health==Health.VALID and h.depth_available
               for c in h.coverage]
        return all(any(abs(math.remainder(angle-math.atan2(c.direction.y,c.direction.x),2*math.pi))
                       <=c.half_angle_rad+1e-6 for c in cones) for angle in directions)


def scan_coverage(ranges, range_min, range_max, angle_min, angle_increment, body_yaw=0.):
    """Each valid beam certifies its angular cell; invalid rays leave explicit gaps."""
    if (_native is not None and isinstance(ranges, (list, tuple)) and
            all(isinstance(value, Real) for value in ranges) and
            all(isinstance(value, Real) for value in
                (range_min, range_max, angle_min, angle_increment, body_yaw))):
        values = _native.scan_coverage(
            [float(value) for value in ranges], float(range_min), float(range_max),
            float(angle_min), float(angle_increment), float(body_yaw))
        return tuple(BearingCone(Vec3(values[index], values[index + 1], values[index + 2]),
                                 values[index + 3])
                     for index in range(0, len(values), 4))
    cones=[];start=None
    valid=lambda r:(math.isfinite(r) and range_min<=r<=range_max) or r==math.inf
    for index in range(len(ranges)+1):
        good=index<len(ranges) and valid(ranges[index])
        if good and start is None:start=index
        if not good and start is not None:
            middle=angle_min+(start+index-1)*.5*angle_increment+body_yaw
            half=min(math.pi,(index-start)*abs(angle_increment)*.5)
            cones.append(BearingCone(Vec3(math.cos(middle),math.sin(middle),0.),half));start=None
    return tuple(cones)


def movement_directions(vx,vy,wz):
    if (_native is not None and all(isinstance(value, Real) for value in (vx, vy, wz))):
        return tuple(_native.movement_directions(float(vx), float(vy), float(wz)))
    if abs(wz)>.02:return tuple(i*math.pi/8 for i in range(16))
    if math.hypot(vx,vy)<.01:return ()
    angle=math.atan2(vy,vx)
    return (angle-math.pi/4,angle,angle+math.pi/4)


class CameraCalibrationRegistry:
    def __init__(self):self.records={}
    def register(self, calibration):
        old=self.records.get(calibration.camera_id)
        if old is not None and calibration.calibration_epoch<=old.calibration_epoch:
            if old!=calibration:raise ValueError('calibration epoch must increase on change')
            return
        self.records[calibration.camera_id]=calibration
    def calibration(self,camera_id,epoch):
        value=self.records[camera_id]
        if value.calibration_epoch!=epoch:raise ValueError('calibration version mismatch')
        return value


def coverage_allows_motion(cones, vx, vy, wz):
    """Cover the complete required interval, including gaps between sampled headings."""
    if (_native is not None and all(isinstance(value, Real) for value in (vx, vy, wz))):
        flat = []
        native_compatible = True
        for cone in cones:
            try:
                direction = cone.direction
                values = (direction.x, direction.y, cone.half_angle_rad)
            except AttributeError:
                native_compatible = False
                break
            if not all(isinstance(value, Real) and math.isfinite(float(value))
                       for value in values):
                native_compatible = False
                break
            flat.extend(float(value) for value in values)
        if native_compatible:
            return bool(_native.coverage_allows_motion(
                flat, float(vx), float(vy), float(wz)))
    if abs(wz)>.02:left,right=-math.pi,math.pi
    elif math.hypot(vx,vy)>=.01:
        angle=math.atan2(vy,vx);left,right=angle-math.pi/4,angle+math.pi/4
    else:return True
    intervals=[]
    for cone in cones:
        angle=math.atan2(cone.direction.y,cone.direction.x)
        for offset in (-2*math.pi,0.,2*math.pi):
            intervals.append((angle+offset-cone.half_angle_rad,angle+offset+cone.half_angle_rad))
    cursor=left
    for a,b in sorted(intervals):
        if b<cursor:continue
        if a>cursor+1e-6:return False
        cursor=max(cursor,b)
        if cursor>=right-1e-6:return True
    return False
