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
        if self.envelope is not None and envelope.epoch==self.envelope.epoch and ns<self.envelope.stamp.sec*10**9+self.envelope.stamp.nanosec:return False
        self.envelope=envelope;self.received=now
        return True
    def ready(self,now):
        e=self.envelope
        if e is None or not e.transport_ready:return False
        return True
    def stopping_distance(self,speed):
        return speed*(self.reaction_time_s+getattr(self,'linear_stop_delay_s',0.))+speed*speed/(2*self.brake_deceleration_m_s2)+self.clearance_margin_m


class FixedEnvelopeProfile(EnvelopeProfile):
    """Only V2 messages may refresh this profile; malformed updates revoke it."""
    def __init__(self,baseline):
        super().__init__(baseline);self.v2=None;self.footprint_xy=None;self.accepted_identity=None
        self.applied_configuration=None
        self.last_update=None
    @staticmethod
    def configuration_key(msg):
        # Compare exact geometry as well as its rounded identity hash.
        return (msg.coordinator_session_id,msg.epoch,msg.clock_epoch,msg.request_id,
            msg.hold_id,msg.reference_state_sequence,msg.model_revision,msg.attachment_revision,
            msg.mode,msg.header.frame_id,msg.clearance_m,msg.installed_geometry_hash,
            tuple((p.x,p.y,p.z) for p in msg.reserved_footprint.points),
            tuple((p.x,p.y,p.z) for p in msg.installed_footprint.points),
            tuple(getattr(msg.limits,f) for f in FIELDS+('epoch','posture_id','frame_id','lease_s')))
    def confirms_applied(self,msg,now):
        applied=self.applied_configuration
        return (applied is not None and (now.clock,now.epoch)==applied[1:3] and
                self.configuration_key(msg)==applied[0])
    def accept(self,msg,now):
        from astribot_s1_robot_geometry.polygon import validate, contains, inflate, geometry_hash
        import numpy as np
        def pts(poly):return np.array([(p.x,p.y) for p in poly.points])
        source=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        context=(msg.coordinator_session_id,msg.epoch,msg.clock_epoch)
        if self.last_update is not None:
            previous,stamp=self.last_update
            if context==previous and source<stamp:return False
            if context[0]==previous[0] and (context[1]<previous[1] or context[2]<previous[2]):return False
        self.last_update=(context,source)
        try:
            validate_envelope(msg.limits,self.baseline)
            if msg.header.frame_id!=self.baseline.base_frame or msg.mode!=msg.FIXED_POSTURE:raise ValueError('invalid V2 frame/mode')
            if not msg.coordinator_session_id or not msg.hold_id:raise ValueError('missing V2 identity')
            if not math.isfinite(msg.clearance_m) or msg.clearance_m<self.baseline.clearance_margin_m+self.baseline.payload_extra_margin_m:raise ValueError('V2 clearance undercut')
            reserved=validate(pts(msg.reserved_footprint));installed=validate(pts(msg.installed_footprint))
            if not contains(installed,inflate(reserved,msg.clearance_m),2e-6):raise ValueError('V2 installed footprint undercut')
            if geometry_hash(installed,msg.header.frame_id,msg.clearance_m)!=msg.installed_geometry_hash:raise ValueError('V2 hash mismatch')
            if np.max(np.abs(reserved[:,0]))>msg.limits.half_length_m+1e-6 or np.max(np.abs(reserved[:,1]))>msg.limits.half_width_m+1e-6:raise ValueError('V2 broad-phase undercut')
            identity=(msg.coordinator_session_id,msg.epoch)
            if self.v2 is not None and msg.coordinator_session_id==self.v2.coordinator_session_id:
                if msg.epoch<self.v2.epoch:return False
                if msg.epoch==self.v2.epoch and (msg.installed_geometry_hash!=self.v2.installed_geometry_hash or any(getattr(msg.limits,f)!=getattr(self.envelope,f) for f in FIELDS)):raise ValueError('V2 mutated epoch')
            source=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            self.envelope=msg.limits;self.v2=msg;self.received=now;self.footprint_xy=reserved
            self.accepted_identity=identity
            self.applied_configuration=(self.configuration_key(msg),now.clock,now.epoch,now.ns)
            return True
        except (ValueError,AttributeError):
            self.v2=None;self.envelope=None;self.received=None;self.accepted_identity=None
            self.applied_configuration=None
            raise
    def ready(self,now):
        e=self.v2
        if e is None or not e.navigation_allowed:return False
        return super().ready(now)


def configure_envelope_input(node,baseline,callback,consumer_id,depth=10):
    """Selection is explicit; no silent downgrade if a V2 publisher disappears."""
    node.declare_parameter('navigation_geometry_mode','legacy')
    mode=node.get_parameter('navigation_geometry_mode').value
    if mode not in ('legacy','fixed_v2'):raise ValueError('invalid navigation_geometry_mode')
    if mode=='legacy':
        from astribot_navigation_msgs.msg import RobotEnvelope
        node.create_subscription(RobotEnvelope,'/navigation/robot_envelope',callback,depth)
        return EnvelopeProfile(baseline),None
    from astribot_navigation_msgs.msg import NavigationEnvelopeV2,EnvelopeApplyStatus
    publisher=node.create_publisher(EnvelopeApplyStatus,'/navigation/envelope_applied',10)
    # Keep bounded reception history for the larger V2 polygon stream, even
    # when the policy uses a latest-only mailbox. Applying that mailbox still
    # selects the newest source timestamp within the coordinator context.
    node.create_subscription(NavigationEnvelopeV2,'/navigation/envelope_v2',callback,max(10,depth))
    def ack(msg):
        result=EnvelopeApplyStatus();result.header.stamp=node.get_clock().now().to_msg()
        result.header.frame_id=baseline.base_frame;result.coordinator_session_id=msg.coordinator_session_id
        result.consumer_id=consumer_id;result.envelope_epoch=msg.epoch
        result.installed_geometry_hash=msg.installed_geometry_hash;result.applied=True
        result.reason='POLYGON_APPLIED';publisher.publish(result)
    return FixedEnvelopeProfile(baseline),ack
