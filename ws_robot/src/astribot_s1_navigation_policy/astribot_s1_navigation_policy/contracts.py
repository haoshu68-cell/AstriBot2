"""Immutable SI-unit contracts; image-space geometry is explicitly separate."""
from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Union


class ErrorCode(str, Enum):
    INVALID_INPUT = 'INVALID_INPUT'
    CLOCK_MISMATCH = 'CLOCK_MISMATCH'
    STALE_OBSERVATION = 'STALE_OBSERVATION'
    FUTURE_OBSERVATION = 'FUTURE_OBSERVATION'
    VERSION_MISMATCH = 'VERSION_MISMATCH'
    CAPABILITY_UNAVAILABLE = 'CAPABILITY_UNAVAILABLE'
    UNSAFE_DECISION = 'UNSAFE_DECISION'


class ContractError(ValueError):
    def __init__(self, code: ErrorCode, field: str):
        self.code = code
        self.field = field
        super().__init__(f'{code.value}: {field}')


def require(condition: bool, field: str, code=ErrorCode.INVALID_INPUT):
    if not condition:
        raise ContractError(code, field)


def finite(value, field, minimum=None):
    require(type(value) in (int, float), field)
    try:
        valid = math.isfinite(value)
    except OverflowError:
        valid = False
    require(valid, field)
    require(minimum is None or value >= minimum, field)


def label(value, field):
    require(isinstance(value, str) and bool(value.strip()), field)


def immutable_tuple(value, field):
    require(type(value) is tuple, field)


@dataclass(frozen=True)
class Stamp:
    ns: int
    clock: str
    epoch: int

    def __post_init__(self):
        label(self.clock, 'clock')
        require(type(self.ns) is int and self.ns >= 0, 'stamp.ns')
        require(type(self.epoch) is int and self.epoch >= 0, 'stamp.epoch')

    def since(self, other: 'Stamp') -> int:
        require(isinstance(other, Stamp), 'stamp.type')
        require((self.clock, self.epoch) == (other.clock, other.epoch),
                'stamp.clock/epoch', ErrorCode.CLOCK_MISMATCH)
        return self.ns - other.ns


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def __post_init__(self):
        for name in ('x', 'y', 'z'):
            finite(getattr(self, name), name)


@dataclass(frozen=True)
class Covariance3:
    values: tuple[float, ...]

    def __post_init__(self):
        immutable_tuple(self.values, 'covariance')
        require(len(self.values) == 9, 'covariance.shape')
        for v in self.values:
            finite(v, 'covariance.value')
        scale = max(abs(v) for v in self.values)
        a = tuple(v / scale for v in self.values) if scale else self.values
        require(all(abs(a[i*3+j] - a[j*3+i]) <= 1e-10
                    for i in range(3) for j in range(3)), 'covariance.symmetry')
        require(all(a[i*3+i] >= 0 for i in range(3)), 'covariance.diagonal')
        require(all(a[i*3+i]*a[j*3+j] - a[i*3+j]**2 >= -1e-12
                    for i in range(3) for j in range(i+1, 3)), 'covariance.minors')
        det = (a[0]*(a[4]*a[8]-a[5]*a[7]) - a[1]*(a[3]*a[8]-a[5]*a[6])
               + a[2]*(a[3]*a[7]-a[4]*a[6]))
        require(det >= -1e-12, 'covariance.positive_semidefinite')


@dataclass(frozen=True)
class CameraCalibration:
    """Intrinsics and depth units; capture-time extrinsics remain the adapter's TF responsibility."""
    camera_id: str
    optical_frame: str
    calibration_epoch: int
    image_width_px: int
    image_height_px: int
    intrinsic_matrix: tuple[float, ...]
    distortion_model: str
    distortion_coefficients: tuple[float, ...]
    depth_unit_m: Optional[float] = None

    def __post_init__(self):
        for name in ('camera_id', 'optical_frame', 'distortion_model'):
            label(getattr(self, name), name)
        for name in ('image_width_px', 'image_height_px'):
            value = getattr(self, name)
            require(type(value) is int and value > 0, name)
        require(type(self.calibration_epoch) is int and self.calibration_epoch >= 0, 'calibration_epoch')
        immutable_tuple(self.intrinsic_matrix, 'intrinsic_matrix')
        require(len(self.intrinsic_matrix) == 9, 'intrinsic_matrix.shape')
        immutable_tuple(self.distortion_coefficients, 'distortion_coefficients')
        for value in self.intrinsic_matrix + self.distortion_coefficients:
            finite(value, 'camera.coefficient')
        k = self.intrinsic_matrix
        require(k[0] > 0 and k[4] > 0 and k[3] == 0 and k[6:] == (0, 0, 1), 'intrinsic_matrix.pinhole')
        if self.depth_unit_m is not None:
            finite(self.depth_unit_m, 'depth_unit_m', 0)
            require(self.depth_unit_m > 0, 'depth_unit_m')


@dataclass(frozen=True)
class ImageBox:
    camera_id: str
    image_width_px: int
    image_height_px: int
    xmin_px: float
    ymin_px: float
    xmax_px: float
    ymax_px: float

    def __post_init__(self):
        label(self.camera_id, 'camera_id')
        require(type(self.image_width_px) is int and self.image_width_px > 0, 'image.width')
        require(type(self.image_height_px) is int and self.image_height_px > 0, 'image.height')
        for field in ('xmin_px', 'ymin_px', 'xmax_px', 'ymax_px'):
            finite(getattr(self, field), field, 0)
        require(self.xmin_px < self.xmax_px <= self.image_width_px, 'image.box.x')
        require(self.ymin_px < self.ymax_px <= self.image_height_px, 'image.box.y')


@dataclass(frozen=True)
class BearingCone:
    direction: Vec3
    half_angle_rad: float

    def __post_init__(self):
        require(isinstance(self.direction, Vec3), 'bearing.direction')
        norm = math.hypot(self.direction.x, self.direction.y, self.direction.z)
        require(abs(norm-1.) <= 1e-6, 'bearing.unit_direction')
        finite(self.half_angle_rad, 'bearing.half_angle', 0)
        require(self.half_angle_rad <= math.pi, 'bearing.half_angle')


@dataclass(frozen=True)
class MetricBox:
    """Axis-aligned conservative box in observation.frame_id; lengths in metres."""
    center_m: Vec3
    size_m: Vec3
    position_covariance_m2: Covariance3
    velocity_m_s: Optional[Vec3] = None
    velocity_covariance_m2_s2: Optional[Covariance3] = None

    def __post_init__(self):
        require(isinstance(self.center_m, Vec3) and isinstance(self.size_m, Vec3), 'metric.geometry')
        require(all(v > 0 for v in (self.size_m.x, self.size_m.y, self.size_m.z)), 'metric.size')
        require(isinstance(self.position_covariance_m2, Covariance3), 'metric.position_covariance')
        require((self.velocity_m_s is None) == (self.velocity_covariance_m2_s2 is None),
                'metric.velocity_and_covariance')
        if self.velocity_m_s is not None:
            require(isinstance(self.velocity_m_s, Vec3), 'metric.velocity')
            require(isinstance(self.velocity_covariance_m2_s2, Covariance3), 'metric.velocity_covariance')


Geometry = Union[ImageBox, BearingCone, MetricBox]


@dataclass(frozen=True)
class Observation:
    sensor_id: str
    measurement_id: str
    source_track_id: Optional[str]
    capture_stamp: Stamp
    received_at: Stamp
    valid_until: Stamp
    frame_id: str
    calibration_epoch: int
    geometry: Geometry
    geometry_quality: float
    class_probabilities: tuple[tuple[str, float], ...]
    provenance: tuple[str, ...]
    velocity_observable: bool = True
    spatial_occupancy: bool = False

    def __post_init__(self):
        require(type(self.velocity_observable) is bool, 'velocity_observable')
        require(type(self.spatial_occupancy) is bool, 'spatial_occupancy')
        if self.spatial_occupancy:
            require(isinstance(self.geometry,MetricBox) and not self.velocity_observable and
                    self.geometry.velocity_m_s is None and self.source_track_id is not None,
                    'spatial_occupancy.requires_fixed_metric_cell')
        for field in ('sensor_id', 'measurement_id', 'frame_id'):
            label(getattr(self, field), field)
        if self.source_track_id is not None:
            label(self.source_track_id, 'source_track_id')
        require(all(isinstance(s, Stamp) for s in
                    (self.capture_stamp, self.received_at, self.valid_until)), 'observation.stamps')
        require(self.received_at.clock == 'steady', 'received_at.clock')
        require(self.valid_until.since(self.capture_stamp) > 0, 'observation.valid_until')
        require(type(self.calibration_epoch) is int and self.calibration_epoch >= 0,
                'calibration_epoch')
        require(isinstance(self.geometry, (ImageBox, BearingCone, MetricBox)), 'geometry.kind')
        finite(self.geometry_quality, 'geometry_quality', 0)
        require(self.geometry_quality <= 1, 'geometry_quality')
        immutable_tuple(self.class_probabilities, 'class_probabilities')
        labels = set()
        for pair in self.class_probabilities:
            immutable_tuple(pair, 'class_probability')
            require(len(pair) == 2, 'class_probability.shape')
            name, probability = pair
            label(name, 'class_name')
            finite(probability, 'class_probability', 0)
            require(probability <= 1 and name not in labels, 'class_probability')
            labels.add(name)
        require(sum(p for _, p in self.class_probabilities) <= 1+1e-9, 'class_probability.sum')
        immutable_tuple(self.provenance, 'provenance')
        require(bool(self.provenance), 'provenance.empty')
        for item in self.provenance:
            label(item, 'provenance.item')
        require(len(set(self.provenance)) == len(self.provenance), 'provenance.duplicate')

    def check_fresh(self, now: Stamp, max_age_ns: int, future_tolerance_ns: int = 0):
        require(type(max_age_ns) is int and max_age_ns >= 0, 'max_age_ns')
        require(type(future_tolerance_ns) is int and future_tolerance_ns >= 0, 'future_tolerance_ns')
        age = now.since(self.capture_stamp)
        require(age >= -future_tolerance_ns, 'capture_stamp', ErrorCode.FUTURE_OBSERVATION)
        require(age <= max_age_ns and now.since(self.valid_until) < 0,
                'capture_stamp/valid_until', ErrorCode.STALE_OBSERVATION)


class Health(str, Enum):
    VALID = 'VALID'
    DEGRADED = 'DEGRADED'
    UNAVAILABLE = 'UNAVAILABLE'
    STALE = 'STALE'


@dataclass(frozen=True)
class SensorHealth:
    sensor_id: str
    health: Health
    stamp: Stamp
    valid_until: Stamp
    frame_id: str
    coverage: tuple[BearingCone, ...]
    depth_available: bool
    calibration_epoch: int
    reason: str

    def __post_init__(self):
        for field in ('sensor_id', 'frame_id', 'reason'):
            label(getattr(self, field), field)
        require(isinstance(self.health, Health), 'health')
        require(isinstance(self.stamp, Stamp) and isinstance(self.valid_until, Stamp), 'health.stamps')
        require(self.valid_until.since(self.stamp) > 0, 'health.valid_until')
        immutable_tuple(self.coverage, 'coverage')
        require(all(isinstance(c, BearingCone) for c in self.coverage), 'coverage.cones')
        require(self.health != Health.VALID or bool(self.coverage), 'valid_health.coverage')
        require(type(self.depth_available) is bool, 'depth_available')
        require(type(self.calibration_epoch) is int and self.calibration_epoch >= 0, 'calibration_epoch')


@dataclass(frozen=True)
class Version:
    goal_id: str
    path_revision: int
    map_epoch: int
    envelope_epoch: int
    localization_epoch: int = 0
    clock_epoch: int = 0

    def __post_init__(self):
        label(self.goal_id, 'goal_id')
        for name in ('path_revision', 'map_epoch', 'envelope_epoch', 'localization_epoch', 'clock_epoch'):
            value = getattr(self, name)
            require(type(value) is int and value >= 0, name)


class Motion(str, Enum):
    CONTINUE = 'CONTINUE'
    SLOW = 'SLOW'
    HOLD = 'HOLD'
    STOP = 'STOP'
    FOLLOW_COMMITTED_PATH = 'FOLLOW_COMMITTED_PATH'
    RETREAT = 'RETREAT'


class Planning(str, Enum):
    NONE = 'NONE'
    LOCAL = 'LOCAL'
    GLOBAL = 'GLOBAL'


class Trigger(str, Enum):
    NONE = 'NONE'
    NEW_GOAL = 'NEW_GOAL'
    PATH_RISK = 'PATH_RISK'


@dataclass(frozen=True)
class MotionLimits:
    linear_speed_m_s: float
    angular_speed_rad_s: float
    linear_accel_m_s2: float
    angular_accel_rad_s2: float

    def __post_init__(self):
        for field in self.__dataclass_fields__:
            finite(getattr(self, field), field, 0)


@dataclass(frozen=True)
class Decision:
    decision_id: str
    episode_id: str
    version: Version
    issued_at: Stamp
    valid_until: Stamp
    motion: Motion
    planning: Planning
    trigger: Trigger
    limits: MotionLimits
    reason: str
    request_id: Optional[str] = None
    committed_path_revision: Optional[int] = None

    def __post_init__(self):
        for name in ('decision_id', 'episode_id', 'reason'):
            label(getattr(self, name), name)
        require(isinstance(self.version, Version), 'version')
        require(isinstance(self.motion, Motion) and isinstance(self.planning, Planning)
                and isinstance(self.trigger, Trigger), 'decision.enums')
        require(isinstance(self.limits, MotionLimits), 'limits')
        require(isinstance(self.issued_at, Stamp) and isinstance(self.valid_until, Stamp), 'decision.stamps')
        require(self.issued_at.clock == 'steady', 'decision.clock')
        require(self.valid_until.since(self.issued_at) > 0, 'decision.valid_until')
        if self.planning != Planning.NONE:
            require(self.trigger != Trigger.NONE, 'planning.trigger')
            label(self.request_id, 'planning.request_id')
        else:
            require(self.request_id is None, 'planning.unexpected_request')
        if self.motion in (Motion.HOLD, Motion.STOP):
            require(self.limits.linear_speed_m_s == 0 and self.limits.angular_speed_rad_s == 0,
                    'hold/stop.speed', ErrorCode.UNSAFE_DECISION)
        if self.motion in (Motion.FOLLOW_COMMITTED_PATH, Motion.RETREAT):
            require(type(self.committed_path_revision) is int
                    and self.committed_path_revision == self.version.path_revision,
                    'committed_path_revision', ErrorCode.VERSION_MISMATCH)
        else:
            require(self.committed_path_revision is None, 'unexpected_committed_revision')


@dataclass(frozen=True)
class ExecutionContext:
    version: Version
    now: Stamp
    required_inputs_valid: bool
    motion_enabled: bool
    allowed_planning: frozenset[Planning]
    retreat_enabled: bool
    baseline_limits: MotionLimits

    def __post_init__(self):
        require(isinstance(self.version, Version) and isinstance(self.now, Stamp), 'context.version/time')
        require(self.now.clock == 'steady', 'context.clock')
        for name in ('required_inputs_valid', 'motion_enabled', 'retreat_enabled'):
            require(type(getattr(self, name)) is bool, name)
        require(type(self.allowed_planning) is frozenset and
                all(isinstance(p, Planning) for p in self.allowed_planning), 'allowed_planning')
        require(isinstance(self.baseline_limits, MotionLimits), 'baseline_limits')


def check_executable(decision: Decision, context: ExecutionContext):
    """Checks authority and freshness only; does not certify collision freedom."""
    require(isinstance(decision, Decision) and isinstance(context, ExecutionContext), 'execution.types')
    require(context.motion_enabled, 'observation_only', ErrorCode.CAPABILITY_UNAVAILABLE)
    require(decision.version == context.version, 'decision.version', ErrorCode.VERSION_MISMATCH)
    require(context.now.since(decision.issued_at) >= 0 and context.now.since(decision.valid_until) < 0,
            'decision.lease', ErrorCode.STALE_OBSERVATION)
    require(decision.planning == Planning.NONE or decision.planning in context.allowed_planning,
            'planning.capability', ErrorCode.CAPABILITY_UNAVAILABLE)
    require(context.retreat_enabled or decision.motion != Motion.RETREAT,
            'retreat.capability', ErrorCode.CAPABILITY_UNAVAILABLE)
    require(context.required_inputs_valid or
            (decision.motion == Motion.STOP and decision.planning == Planning.NONE),
            'required_inputs', ErrorCode.UNSAFE_DECISION)
    for name in context.baseline_limits.__dataclass_fields__:
        require(getattr(decision.limits, name) <= getattr(context.baseline_limits, name),
                'limits.'+name, ErrorCode.UNSAFE_DECISION)
