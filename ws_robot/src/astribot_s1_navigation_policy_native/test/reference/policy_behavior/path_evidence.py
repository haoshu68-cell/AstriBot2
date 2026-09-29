"""Path-bound collision evidence; stale data never grants distant-risk release."""
from dataclasses import dataclass
import math
import os


_native = None
if os.environ.get('ASTRIBOT_NAV_NATIVE_KERNELS', '').lower() in ('1', 'true', 'yes'):
    try:
        from astribot_s1_navigation_policy_native import _navigation_math_native as _native
    except ImportError:  # pragma: no cover - Python-only overlays remain supported
        _native = None


def path_identity(path):
    def header(h):
        return h.frame_id,h.stamp.sec,h.stamp.nanosec
    poses=[]
    for point in path.poses:
        p=point.pose.position;q=point.pose.orientation
        values=(p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('nonfinite path geometry')
        poses.append((header(point.header),values))
    # Compare semantic fields, never CDR alignment padding bytes.
    return header(path.header),tuple(poses)


@dataclass(frozen=True)
class PathEvidence:
    path_key: tuple
    stamp_s: float
    received_wall_s: float
    epoch: int
    known: bool
    blocked: bool
    distance_m: float


@dataclass(frozen=True)
class PathAssessment:
    blocked: bool
    conflict_time_s: float
    status: str
    distance_m: float = 0.


def assess_path(evidence, path_key, now_s, wall_s, epoch, legacy_blocked, profile):
    fallback=PathAssessment(bool(legacy_blocked),0. if legacy_blocked else math.inf,'LEGACY')
    if evidence is None or evidence.path_key!=path_key or evidence.epoch!=epoch:
        return fallback
    if _native is not None:
        try:
            values = (evidence.stamp_s, evidence.received_wall_s,
                      evidence.distance_m, now_s, wall_s,
                      profile.path_risk_timeout_s, profile.max_speed_m_s,
                      profile.clearance_margin_m,
                      profile.payload_extra_margin_m)
            if (all(isinstance(value, (int, float)) for value in values) and
                    math.isfinite(profile.max_speed_m_s) and
                    profile.max_speed_m_s > 0.0):
                result = _native.assess_path(
                    bool(evidence.known), bool(evidence.blocked),
                    float(evidence.stamp_s), float(evidence.received_wall_s),
                    float(evidence.distance_m), float(now_s), float(wall_s),
                    float(profile.path_risk_timeout_s),
                    float(profile.max_speed_m_s),
                    float(profile.clearance_margin_m),
                    float(profile.payload_extra_margin_m),
                    bool(legacy_blocked))
                return PathAssessment(bool(result.blocked),
                                      float(result.conflict_time_s),
                                      result.status,
                                      float(result.distance_m))
        except AttributeError:
            pass
    if not all(math.isfinite(v) for v in (now_s,wall_s,evidence.stamp_s,
                                        evidence.received_wall_s,evidence.distance_m)):
        return PathAssessment(True,0.,'INVALID')
    ros_age=now_s-evidence.stamp_s;wall_age=wall_s-evidence.received_wall_s
    if not (0<=ros_age<=profile.path_risk_timeout_s and 0<=wall_age<=profile.path_risk_timeout_s):
        return PathAssessment(bool(legacy_blocked or evidence.blocked),0.,'EXPIRED')
    if not evidence.known or evidence.distance_m<0:
        return PathAssessment(True,0.,'UNKNOWN')
    if not evidence.blocked:
        return PathAssessment(False,math.inf,'CLEAR')
    distance=max(0.,evidence.distance_m-profile.max_speed_m_s*max(ros_age,wall_age)
                 -profile.clearance_margin_m-profile.payload_extra_margin_m)
    return PathAssessment(True,distance/profile.max_speed_m_s,'OCCUPIED',distance)
