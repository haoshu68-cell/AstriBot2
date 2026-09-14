"""Shared bounds for policy and independent protection; fixed simulation defaults remain unchanged."""
import math

FIELDS=('half_length_m','half_width_m','height_m','payload_mass_kg','max_speed_m_s',
        'max_angular_speed_rad_s','max_acceleration_m_s2','brake_deceleration_m_s2')


def validate_envelope(envelope, baseline):
    if not envelope.posture_id or envelope.frame_id!=baseline.base_frame:raise ValueError('invalid posture/frame')
    if not 0<envelope.lease_s<=.5:raise ValueError('invalid envelope lease')
    for name in FIELDS:
        value=getattr(envelope,name)
        if not math.isfinite(value) or value<0 or (name!='payload_mass_kg' and value==0):raise ValueError(name)
    for name in ('half_length_m','half_width_m','height_m'):
        if getattr(envelope,name)<getattr(baseline,name):raise ValueError('envelope cannot undercut baseline: '+name)
    for name in ('max_speed_m_s','max_angular_speed_rad_s','max_acceleration_m_s2','brake_deceleration_m_s2'):
        if getattr(envelope,name)>getattr(baseline,name):raise ValueError('unvalidated limit increase: '+name)


class EnvelopeProfile:
    def __init__(self, baseline):
        self.baseline=baseline
        self.envelope=None
        self.received=None
    def __getattr__(self,name):
        if name in FIELDS and self.envelope is not None:return getattr(self.envelope,name)
        return getattr(self.baseline,name)
    def accept(self,envelope,now):
        validate_envelope(envelope,self.baseline)
        if self.envelope is not None and envelope.epoch<self.envelope.epoch:return False
        if self.envelope is not None and envelope.epoch==self.envelope.epoch:
            if any(getattr(envelope,f)!=getattr(self.envelope,f) for f in FIELDS+('posture_id','frame_id')):
                raise ValueError('geometry and limits require a new envelope epoch')
        ns=envelope.stamp.sec*10**9+envelope.stamp.nanosec
        if not 0<=now.ns-ns<=int(envelope.lease_s*1e9):return False
        self.envelope=envelope;self.received=now
        return True
    def ready(self,now):
        e=self.envelope
        if e is None or not e.transport_ready:return False
        try:return 0<=now.since(self.received)<=int(e.lease_s*1e9) and 0<=now.ns-(e.stamp.sec*10**9+e.stamp.nanosec)<=int(e.lease_s*1e9)
        except ValueError:return False
    def stopping_distance(self,speed):
        return speed*self.reaction_time_s+speed*speed/(2*self.brake_deceleration_m_s2)+self.clearance_margin_m
