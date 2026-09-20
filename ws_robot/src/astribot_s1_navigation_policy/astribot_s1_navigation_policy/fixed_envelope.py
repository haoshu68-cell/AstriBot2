"""Fixed-posture handshake, independent of ROS executors and wall timers."""
import copy
import math
import uuid
import time
import numpy as np
from astribot_s1_robot_geometry.polygon import validate, contains, inflate, geometry_hash
from astribot_s1_robot_geometry.node import ns, stamp, polygon
from astribot_navigation_msgs.msg import NavigationEnvelopeV2
from .robot_envelope import validate_envelope

CONSUMERS=frozenset(('global_costmap','local_costmap','planner','controller','policy','protection'))

def points(poly):return np.array([(p.x,p.y) for p in poly.points],dtype=float)

class FixedEnvelope:
    def __init__(self,baseline):
        if baseline.environment!='simulation':raise ValueError('FIXED_V2_HARDWARE_NOT_VALIDATED')
        self.baseline=baseline;self.session=str(uuid.uuid4());self.epoch=time.monotonic_ns()
        self.current=None;self.reference=None;self.history={};self.hold=None;self.acks={}
        self.output=None;self.fault='NO_FIXED_ENVELOPE';self.last_now=0

    def state(self,msg,now):
        if self.current is not None and msg.source_id==self.current.source_id and msg.sequence<=self.current.sequence:return
        self.current=copy.deepcopy(msg)
        self.history[(msg.source_id,msg.sequence)]=self.current
        while len(self.history)>8:self.history.pop(next(iter(self.history)))
        if not msg.complete:self.revoke('GEOMETRY_INCOMPLETE: '+msg.reason)

    def revoke(self,reason):
        self.fault=reason
        if self.output is not None:self.output.navigation_allowed=False

    def validate_state(self,state,now):
        if state is None or not state.complete or not state.attachment_state_confirmed:raise ValueError('GEOMETRY_INCOMPLETE')
        if not state.source_id or not state.model_revision or not state.attachment_revision:raise ValueError('GEOMETRY_ID_MISSING')
        if state.header.frame_id!=self.baseline.base_frame:raise ValueError('GEOMETRY_FRAME_MISMATCH')
        if not 0<ns(state.header.stamp)<=now<ns(state.valid_until)<=ns(state.header.stamp)+500_000_000:raise ValueError('GEOMETRY_EXPIRED')
        n=len(state.joints.name)
        if not n or len(set(state.joints.name))!=n or any(len(x)!=n for x in (state.joints.position,state.joint_source_stamps,state.joint_position_error_bounds)):raise ValueError('JOINT_ARRAY_INVALID')
        if not all(math.isfinite(q) for q in state.joints.position):raise ValueError('JOINT_INVALID')
        if not all(math.isfinite(e) and 0<e<=.025 for e in state.joint_position_error_bounds):raise ValueError('HOLD_ERROR_INVALID')
        times=[ns(t) for t in state.joint_source_stamps]
        if min(times)!=ns(state.header.stamp) or max(times)>now or max(times)-min(times)>100_000_000:raise ValueError('JOINT_TIMING_INVALID')
        if not math.isfinite(state.height_m) or state.height_m<=0:raise ValueError('HEIGHT_INVALID')
        if not contains(validate(points(state.reserved_footprint)),validate(points(state.physical_footprint))):raise ValueError('RESERVATION_UNDERSIZED')

    def hold_until(self,hold_id,revision,now):
        h=self.hold
        if h is None or not h.hold_confirmed or not h.owner_id or h.hold_id!=hold_id or h.attachment_revision!=revision:raise ValueError('ARM_HOLD_UNCONFIRMED')
        if not math.isfinite(h.lease_s) or not 0<h.lease_s<=.5 or not 0<=now-ns(h.header.stamp)<h.lease_s*1e9:raise ValueError('ARM_HOLD_EXPIRED')
        return ns(h.header.stamp)+round(h.lease_s*1e9)

    def propose(self,request,now,stopped):
        if not stopped:raise ValueError('ROBOT_MUST_BE_STOPPED_WITH_FRESH_ODOMETRY')
        if not request.request_id or not request.hold_id:raise ValueError('REQUEST_ID_REQUIRED')
        self.validate_state(self.current,now)
        reference=self.history.get((self.current.source_id,request.geometry_sequence))
        self.validate_state(reference,now)
        self.hold_until(request.hold_id,reference.attachment_revision,now)
        e=NavigationEnvelopeV2();e.header.frame_id=self.baseline.base_frame;e.coordinator_session_id=self.session
        e.request_id=request.request_id;e.hold_id=request.hold_id;e.mode=e.FIXED_POSTURE
        e.reference_state_sequence=reference.sequence;e.clock_epoch=reference.clock_epoch
        e.model_revision=reference.model_revision;e.attachment_revision=reference.attachment_revision
        e.reserved_footprint=copy.deepcopy(reference.reserved_footprint)
        e.clearance_m=self.baseline.clearance_margin_m+self.baseline.payload_extra_margin_m
        e.installed_footprint=polygon(inflate(points(e.reserved_footprint),e.clearance_m))
        e.installed_geometry_hash=geometry_hash(points(e.installed_footprint),e.header.frame_id,e.clearance_m)
        e.limits=copy.deepcopy(request.limits)
        if reference.attachment_ids and e.limits.payload_mass_kg<=0:raise ValueError("PAYLOAD_MASS_REQUIRED")
        p=points(e.reserved_footprint)
        e.limits.half_length_m=max(self.baseline.half_length_m,float(np.max(np.abs(p[:,0]))))
        e.limits.half_width_m=max(self.baseline.half_width_m,float(np.max(np.abs(p[:,1]))))
        e.limits.height_m=max(self.baseline.height_m,reference.height_m)
        validate_envelope(e.limits,self.baseline)
        self.reference=reference;self.output=e;self.epoch+=1;e.epoch=self.epoch;e.limits.epoch=e.epoch
        self.acks={};self.fault='';self.tick(now)
        if self.fault:raise ValueError(self.fault)
        return e

    def acknowledge(self,msg,now):
        e=self.output
        if e is None or msg.consumer_id not in CONSUMERS:return
        if (msg.coordinator_session_id,msg.envelope_epoch,msg.installed_geometry_hash)!=(self.session,e.epoch,e.installed_geometry_hash):return
        if not msg.applied:
            self.acks.pop(msg.consumer_id,None)
        elif 0<=now-ns(msg.header.stamp)<=500_000_000:
            self.acks[msg.consumer_id]=max(self.acks.get(msg.consumer_id,0),ns(msg.header.stamp))
        # Independent /clock subscriptions may deliver a positive ACK before
        # its time reaches this node. It proves nothing yet, but does not revoke
        # earlier evidence. Keep that evidence's ORIGINAL expiry; never renew
        # from a future or delayed positive packet. A negative ACK revokes now.

    def tick(self,now):
        if now<self.last_now:self.revoke('CLOCK_RESET');self.history.clear()
        self.last_now=now;e=self.output
        if e is None:return None
        e.header.stamp=stamp(now);e.navigation_allowed=False;e.limits.transport_ready=False
        try:
            if self.fault:raise ValueError(self.fault)
            s=self.current;r=self.reference;self.validate_state(s,now)
            if (s.source_id,s.clock_epoch,s.model_revision,s.attachment_revision)!=(r.source_id,r.clock_epoch,r.model_revision,r.attachment_revision):raise ValueError('GEOMETRY_VERSION_CHANGED')
            actual=dict(zip(s.joints.name,s.joints.position))
            if set(actual)!=set(r.joints.name) or any(abs(actual[n]-q)>err for n,q,err in zip(r.joints.name,r.joints.position,r.joint_position_error_bounds)):raise ValueError('FIXED_POSTURE_LEFT_RESERVATION')
            if not contains(points(e.reserved_footprint),points(s.physical_footprint),1e-6):raise ValueError('PHYSICAL_GEOMETRY_LEFT_RESERVATION')
            until=min(ns(s.valid_until),self.hold_until(e.hold_id,e.attachment_revision,now))
            e.source_state_sequence=s.sequence;e.valid_until=stamp(until)
            missing=CONSUMERS-{name for name,t in self.acks.items() if 0<=now-t<500_000_000}
            e.navigation_allowed=not missing;e.reason='WAITING_FOR:'+','.join(sorted(missing)) if missing else 'READY_FIXED'
        except ValueError as error:self.revoke(str(error));e.reason=self.fault;e.valid_until=stamp(now)
        e.limits.transport_ready=e.navigation_allowed;e.limits.stamp=e.header.stamp;e.limits.reason=e.reason
        return e
