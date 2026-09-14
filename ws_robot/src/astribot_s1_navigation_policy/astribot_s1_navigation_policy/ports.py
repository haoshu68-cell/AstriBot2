"""Dependency-inversion ports. Implementations belong to later validated stages."""
from dataclasses import dataclass
from typing import Generic, Protocol, TypeVar

from .contracts import (
    CameraCalibration, Decision, MetricBox, Observation, SensorHealth, Stamp, Version,
    immutable_tuple, label, require,
)


@dataclass(frozen=True)
class Prediction:
    offset_ns: int
    geometry: MetricBox

    def __post_init__(self):
        require(type(self.offset_ns) is int and self.offset_ns >= 0, 'prediction.offset_ns')
        require(isinstance(self.geometry, MetricBox), 'prediction.geometry')


@dataclass(frozen=True)
class TrackedObstacle:
    fused_track_id: str
    frame_id: str
    stamp: Stamp
    geometry: MetricBox
    predictions: tuple[Prediction, ...]
    provenance: tuple[str, ...]

    def __post_init__(self):
        label(self.fused_track_id, 'fused_track_id')
        label(self.frame_id, 'track.frame_id')
        require(isinstance(self.stamp, Stamp) and isinstance(self.geometry, MetricBox), 'track.geometry/time')
        immutable_tuple(self.predictions, 'predictions')
        require(all(isinstance(p, Prediction) for p in self.predictions), 'predictions.types')
        offsets = [p.offset_ns for p in self.predictions]
        require(all(a < b for a, b in zip(offsets, offsets[1:])), 'predictions.order')
        immutable_tuple(self.provenance, 'track.provenance')
        for source in self.provenance:
            label(source, 'track.provenance.item')
        require(bool(self.provenance) and len(set(self.provenance)) == len(self.provenance),
                'track.provenance')


@dataclass(frozen=True)
class WorldSnapshot:
    version: Version
    stamp: Stamp
    frame_id: str
    tracks: tuple[TrackedObstacle, ...]
    unassociated: tuple[Observation, ...]
    sensors: tuple[SensorHealth, ...]
    observation_seq: int

    def __post_init__(self):
        require(isinstance(self.version, Version) and isinstance(self.stamp, Stamp), 'world.version/time')
        label(self.frame_id, 'world.frame_id')
        require(type(self.observation_seq) is int and self.observation_seq >= 0, 'observation_seq')
        for name, cls in [('tracks', TrackedObstacle), ('unassociated', Observation), ('sensors', SensorHealth)]:
            values = getattr(self, name)
            immutable_tuple(values, name)
            require(all(isinstance(v, cls) for v in values), name+'.types')
        require(len({t.fused_track_id for t in self.tracks}) == len(self.tracks), 'world.duplicate_tracks')
        require(len({s.sensor_id for s in self.sensors}) == len(self.sensors), 'world.duplicate_sensors')
        for track in self.tracks:
            require(track.frame_id == self.frame_id, 'world.track_frame')
            require(self.stamp.since(track.stamp) >= 0, 'world.track_stamp')


Packet = TypeVar('Packet', contravariant=True)


class ObservationAdapter(Protocol, Generic[Packet]):
    def normalize(self, packet: Packet) -> tuple[Observation, ...]:
        """Use capture-time calibration/TF; retain source provenance and units."""
        ...


class SensorHealthReader(Protocol):
    def health(self, now: Stamp) -> tuple[SensorHealth, ...]: ...


class CameraCalibrationReader(Protocol):
    def calibration(self, camera_id: str, epoch: int) -> CameraCalibration: ...


class FusionEngine(Protocol):
    def update(self, observations: tuple[Observation, ...], now: Stamp) -> WorldSnapshot: ...


class WorldModelReader(Protocol):
    def snapshot(self) -> WorldSnapshot: ...


class NavigationPolicy(Protocol):
    def propose(self, world: WorldSnapshot) -> tuple[Decision, ...]: ...


class BehaviorArbiter(Protocol):
    def select(self, candidates: tuple[Decision, ...], world: WorldSnapshot) -> Decision: ...
