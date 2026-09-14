"""Conservative observation association. Missing observations are not free space."""
from dataclasses import dataclass, replace
from functools import lru_cache
import math

from .contracts import Covariance3, MetricBox, Observation, Stamp, Vec3, Version, require
from .ports import Prediction, PredictionModel, TrackedObstacle, WorldSnapshot

ZERO_VELOCITY = Vec3(0., 0., 0.)


@lru_cache(maxsize=4096)
def expanded_covariance(values, extra_variance):
    covariance = list(values)
    for i in (0, 4, 8):
        covariance[i] += extra_variance
    return Covariance3(tuple(covariance))


@lru_cache(maxsize=16384)
def translate(box, velocity, dt, extra_variance=0.):
    c = box.center_m
    center = c if velocity == ZERO_VELOCITY else Vec3(c.x+velocity.x*dt, c.y+velocity.y*dt, c.z+velocity.z*dt)
    return replace(box, center_m=center,
                   position_covariance_m2=expanded_covariance(box.position_covariance_m2.values, extra_variance))


@lru_cache(maxsize=256)
def prediction_model(velocity,variance,steps):
    return PredictionModel(velocity,variance,steps)


@dataclass
class _Track:
    identifier: str
    observation: Observation
    velocity: Vec3
    samples: tuple = ()


def fitted_velocity(samples, profile):
    if len(samples)<5 or samples[-1][0]-samples[0][0]<profile.velocity_confirmation_s-1e-8:
        return Vec3(0.,0.,0.)
    count=len(samples);mt=sum(s[0] for s in samples)/count
    mx=sum(s[1] for s in samples)/count;my=sum(s[2] for s in samples)/count
    variance=sum((s[0]-mt)**2 for s in samples)
    vx=sum((t-mt)*(x-mx) for t,x,y in samples)/variance
    vy=sum((t-mt)*(y-my) for t,x,y in samples)/variance
    residual=math.sqrt(sum((x-mx-vx*(t-mt))**2+(y-my-vy*(t-mt))**2 for t,x,y in samples)/count)
    speed=math.hypot(vx,vy)
    if residual>profile.velocity_fit_max_residual_m or speed<profile.min_tracked_speed_m_s:
        return Vec3(0.,0.,0.)
    scale=min(1.,profile.max_obstacle_speed_m_s/max(speed,1e-9))
    return Vec3(vx*scale,vy*scale,0.)


class _AssociationIndex:
    """Exact broad-phase lookup; the existing distance and identity rules decide matches."""
    def __init__(self, tracks, profile):
        self.tracks=tracks
        self.memory=profile.track_memory_s
        self.cell_size=profile.association_distance_m+.2
        self.stamp=None;self.cells={};self.track_cells={};self.evidence={};self.identities={}
        for identifier,track in tracks.items():self.add_evidence(identifier,track.observation)

    def add_evidence(self,identifier,observation):
        if observation.source_track_id is not None:
            self.identities.setdefault((observation.sensor_id,observation.source_track_id),set()).add(identifier)
        for provenance in observation.provenance:
            self.evidence.setdefault((observation.capture_stamp,provenance),set()).add(identifier)

    def correlated(self,observation):
        if not isinstance(observation.geometry,MetricBox):return False
        identifiers=set()
        for provenance in observation.provenance:
            identifiers.update(self.evidence.get((observation.capture_stamp,provenance),()))
        center=observation.geometry.center_m
        return any(math.dist((self.tracks[i].observation.geometry.center_m.x,
                              self.tracks[i].observation.geometry.center_m.y),(center.x,center.y))<.05
                   for i in identifiers if self.tracks[i].observation.spatial_occupancy==observation.spatial_occupancy)

    def add_cell(self,identifier,track):
        if self.stamp is None:return
        dt=self.stamp.since(track.observation.capture_stamp)*1e-9
        if dt<0:return
        center=track.observation.geometry.center_m;t=min(dt,self.memory)
        cell=(math.floor((center.x+track.velocity.x*t)/self.cell_size),
              math.floor((center.y+track.velocity.y*t)/self.cell_size))
        self.cells.setdefault(cell,set()).add(identifier);self.track_cells[identifier]=cell

    def nearby(self,observation):
        if self.stamp!=observation.capture_stamp:
            self.stamp=observation.capture_stamp;self.cells={};self.track_cells={}
            for identifier,track in self.tracks.items():self.add_cell(identifier,track)
        center=observation.geometry.center_m
        x,y=math.floor(center.x/self.cell_size),math.floor(center.y/self.cell_size)
        return tuple(identifier for dx in (-1,0,1) for dy in (-1,0,1)
                     for identifier in self.cells.get((x+dx,y+dy),()))

    def remove(self,identifier):
        track=self.tracks.get(identifier)
        if track is not None:
            obs=track.observation
            if obs.source_track_id is not None:
                key=(obs.sensor_id,obs.source_track_id)
                bucket=self.identities[key];bucket.discard(identifier)
                if not bucket:del self.identities[key]
            for provenance in track.observation.provenance:
                key=(track.observation.capture_stamp,provenance)
                bucket=self.evidence[key];bucket.discard(identifier)
                if not bucket:del self.evidence[key]
        cell=self.track_cells.pop(identifier,None)
        if cell is not None:
            bucket=self.cells[cell];bucket.discard(identifier)
            if not bucket:del self.cells[cell]

    def add(self,identifier,track):
        self.add_evidence(identifier,track.observation)
        self.add_cell(identifier,track)


class ConservativeFusion:
    def __init__(self, profile, frame_id='odom'):
        self.profile = profile
        self.frame_id = frame_id
        self.tracks = {}
        self.unassociated = {}
        self.seen = {}
        self.last_sensor_stamp = {}
        self.next_id = 1
        self.epoch = None
        self.sequence = 0
        self.sensors = ()
        self.version = Version('idle', 0, 0, 0)
        self.prediction_steps = tuple((int(round(t*1e9)), t) for t in self.prediction_times())

    def update(self, observations, now):
        self.ingest(observations, now)
        return self.snapshot(now)

    def ingest(self, observations, now):
        for obs in observations:
            obs.check_fresh(now, int(self.profile.sensor_timeout_s*1e9), 0)
            if isinstance(obs.geometry, MetricBox):
                require(obs.frame_id == self.frame_id, 'fusion.requires_capture_time_transform')
        if self.epoch is not None and self.epoch != (now.clock, now.epoch):
            self.tracks.clear()
            self.unassociated.clear()
            self.seen.clear()
            self.last_sensor_stamp.clear()
        self.epoch = (now.clock, now.epoch)
        assigned = set()
        index=_AssociationIndex(self.tracks,self.profile)
        for obs in sorted(observations, key=lambda o: o.capture_stamp.ns):
            obs.check_fresh(now, int(self.profile.sensor_timeout_s*1e9), 0)
            if obs.capture_stamp.ns < self.last_sensor_stamp.get(obs.sensor_id, -1):
                continue
            self.last_sensor_stamp[obs.sensor_id] = obs.capture_stamp.ns
            key = (obs.sensor_id, obs.measurement_id)
            if key in self.seen:
                continue
            # Same source frame-derived detections must not count as independent evidence.
            if index.correlated(obs):
                self.seen[key] = now.ns
                continue
            self.seen[key] = now.ns
            if not isinstance(obs.geometry, MetricBox) or obs.geometry_quality <= 0:
                self.unassociated[key] = obs
                continue
            require(obs.frame_id == self.frame_id, 'fusion.requires_capture_time_transform')
            candidates = []
            # A valid exact identity already outranks every proximity match.
            # If it fails the unchanged gates, fall back to the full search.
            for exact in (True,False):
                identifiers=(index.identities.get((obs.sensor_id,obs.source_track_id),())
                             if exact else index.nearby(obs))
                for identifier in identifiers:
                    track=self.tracks[identifier]
                    if identifier in assigned and track.observation.sensor_id == obs.sensor_id:
                        continue
                    source=track.observation
                    if source.spatial_occupancy!=obs.spatial_occupancy:continue
                    same_source=source.sensor_id==obs.sensor_id
                    if not same_source and source.capture_stamp!=obs.capture_stamp:continue
                    if (same_source and source.source_track_id is not None and obs.source_track_id is not None
                            and source.source_track_id!=obs.source_track_id):continue
                    dt = obs.capture_stamp.since(track.observation.capture_stamp)*1e-9
                    if dt < 0:
                        continue
                    center = track.observation.geometry.center_m
                    prediction_dt = min(dt, self.profile.track_memory_s)
                    distance = math.dist((center.x+track.velocity.x*prediction_dt, center.y+track.velocity.y*prediction_dt),
                                         (obs.geometry.center_m.x, obs.geometry.center_m.y))
                    gate = self.profile.association_distance_m + min(dt, 1.)*.2
                    if distance <= gate:
                        identity_match=same_source and obs.source_track_id is not None and source.source_track_id==obs.source_track_id
                        candidates.append((not identity_match,distance, identifier))
                if candidates:break
            identifier = min(candidates)[2] if candidates else f'obstacle-{self.next_id}'
            if not candidates:
                self.next_id += 1
            velocity = obs.geometry.velocity_m_s or Vec3(0., 0., 0.)
            samples=()
            if identifier in self.tracks:
                old = self.tracks[identifier]
                dt = obs.capture_stamp.since(old.observation.capture_stamp)*1e-9
                consistent_shape=all(abs(getattr(obs.geometry.size_m,k)-getattr(old.observation.geometry.size_m,k))
                                     <=self.profile.velocity_fit_max_residual_m for k in ('x','y'))
                if (dt<=self.profile.sensor_timeout_s and obs.velocity_observable
                        and old.observation.velocity_observable and consistent_shape):
                    samples=old.samples
                if dt == 0:
                    # Conservative union for simultaneous different-sensor geometry.
                    a, b = old.observation.geometry, obs.geometry
                    lo = [min(getattr(a.center_m, k)-getattr(a.size_m, k)/2,
                              getattr(b.center_m, k)-getattr(b.size_m, k)/2) for k in ('x','y','z')]
                    hi = [max(getattr(a.center_m, k)+getattr(a.size_m, k)/2,
                              getattr(b.center_m, k)+getattr(b.size_m, k)/2) for k in ('x','y','z')]
                    union = replace(b, center_m=Vec3(*[(l+h)/2 for l,h in zip(lo,hi)]),
                                    size_m=Vec3(*[h-l for l,h in zip(lo,hi)]),
                                    velocity_m_s=b.velocity_m_s or a.velocity_m_s,
                                    velocity_covariance_m2_s2=b.velocity_covariance_m2_s2 or a.velocity_covariance_m2_s2,
                                    position_covariance_m2=Covariance3(tuple(max(x,y) if i in (0,4,8) else 0.
                                        for i,(x,y) in enumerate(zip(a.position_covariance_m2.values,b.position_covariance_m2.values)))))
                    obs = replace(obs, geometry=union,
                                  velocity_observable=obs.velocity_observable or old.observation.velocity_observable,
                                  provenance=tuple(sorted(set(obs.provenance+old.observation.provenance))))
                    velocity = union.velocity_m_s or old.velocity
                elif obs.geometry.velocity_m_s is None:
                    velocity = Vec3(0.,0.,0.)
            at=obs.capture_stamp.ns*1e-9;c=obs.geometry.center_m
            samples=tuple(s for s in samples if at-self.profile.velocity_fit_window_s<=s[0]<at)+((at,c.x,c.y),)
            if obs.geometry.velocity_m_s is None:
                velocity=fitted_velocity(samples,self.profile) if obs.velocity_observable else Vec3(0.,0.,0.)
            index.remove(identifier)
            self.tracks[identifier] = _Track(identifier,obs,velocity,samples)
            index.add(identifier,self.tracks[identifier])
            assigned.add(identifier)
        # Retain source identity long enough to reject repeats within the allowed age window.
        self.seen = {k:t for k,t in self.seen.items() if now.ns-t <= int(2e9*self.profile.track_memory_s)}
        self.sequence += 1

    def resolve_unassociated(self, sensor_id, measurement_ids, capture, now):
        require(0 <= now.since(capture) <= int(self.profile.sensor_timeout_s*1e9),
                'clearance.requires_fresh_source_evidence')
        for identifier in measurement_ids:
            key = (sensor_id, identifier)
            obs = self.unassociated.get(key)
            if obs is not None and capture.since(obs.capture_stamp) > 0:
                del self.unassociated[key]

    def clear_observed_free(self, now, free_at):
        """free_at(box) must establish fresh free-space coverage of the entire box."""
        for identifier, track in list(self.tracks.items()):
            age = now.since(track.observation.capture_stamp)*1e-9
            if age > self.profile.sensor_timeout_s:
                box = track.observation.geometry if track.observation.spatial_occupancy else translate(track.observation.geometry,track.velocity,min(age,self.profile.track_memory_s))
                if free_at(box):
                    del self.tracks[identifier]

    def snapshot(self, now, region=None):
        if self.epoch is not None and self.epoch != (now.clock, now.epoch):
            return self.update((), now)
        tracks=[]
        for track in self.tracks.values():
            obs=track.observation
            age=max(0.,now.since(obs.capture_stamp)*1e-9)
            old=age>self.profile.track_memory_s
            velocity=ZERO_VELOCITY if old else track.velocity
            occupancy_only=not obs.velocity_observable and obs.geometry.velocity_m_s is None
            stationary=(len(track.samples)>=5 and track.samples[-1][0]-track.samples[0][0]>=self.profile.velocity_confirmation_s-1e-8
                        and math.hypot(track.velocity.x,track.velocity.y)<self.profile.min_tracked_speed_m_s
                        and all(math.hypot(s[1]-track.samples[-1][1],s[2]-track.samples[-1][2])<=
                                2*self.profile.velocity_fit_max_residual_m for s in track.samples))
            variance=self.profile.stationary_velocity_variance_m2_s2 if occupancy_only or stationary else .01
            if obs.geometry.velocity_covariance_m2_s2 is not None:
                variance=max(variance,*(obs.geometry.velocity_covariance_m2_s2.values[i] for i in (0,4,8)))
            # A fixed occupied cell is a statement about space, not the
            # uncertain identity or velocity of the object that illuminated it.
            # Retain it until fresh free-space evidence; tracked objects retain
            # their full age, motion and covariance forecasts.
            if obs.spatial_occupancy:variance=0.
            age_variance=variance if occupancy_only else max(variance,.04)
            current=obs.geometry if obs.spatial_occupancy else translate(obs.geometry,track.velocity,min(age,self.profile.track_memory_s),min(age,3.)**2*age_variance)
            relevant=True
            if region is not None:
                x,y,travel=region;t=self.profile.prediction_horizon_s
                radius=(travel+self.profile.half_length_m+self.profile.half_width_m+
                        self.profile.clearance_margin_m+self.profile.payload_extra_margin_m+
                        math.hypot(current.size_m.x,current.size_m.y)/2+
                        2*math.sqrt(2*max(current.position_covariance_m2.values[i]+t*t*variance for i in (0,4)))+
                        math.hypot(velocity.x,velocity.y)*t)
                relevant=math.hypot(current.center_m.x-x,current.center_m.y-y)<=radius
            model=prediction_model(velocity,variance,self.prediction_steps) if relevant else None
            tracks.append(TrackedObstacle(track.identifier,self.frame_id,now,current,(),obs.provenance,model))
        return WorldSnapshot(self.version,now,self.frame_id,tuple(tracks),tuple(self.unassociated.values()),self.sensors,self.sequence)

    def prediction_times(self):
        return [i*self.profile.prediction_step_s for i in range(1,int(round(self.profile.prediction_horizon_s/self.profile.prediction_step_s))+1)]
