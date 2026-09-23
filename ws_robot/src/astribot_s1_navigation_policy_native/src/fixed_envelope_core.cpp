#include "astribot_s1_navigation_policy_native/fixed_envelope_core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>
#include <astribot_s1_robot_geometry/geometry_kernels.hpp>

namespace astribot::navigation {
namespace {
namespace geometry = astribot_s1_robot_geometry;
const std::set<std::string> consumers{
  "global_costmap","local_costmap","planner","controller","policy","protection"};
void require(bool condition, const std::string& error) {
  if (!condition) throw std::invalid_argument(error);
}
geometry::Polygon2 points(const geometry_msgs::msg::Polygon& message) {
  geometry::Polygon2 result;
  for (const auto& p : message.points) result.push_back({p.x,p.y});
  return result;
}
geometry_msgs::msg::Polygon polygon(const geometry::Polygon2& input) {
  geometry_msgs::msg::Polygon result;
  for (const auto& p:geometry::serializedPolygon(input)) {
    geometry_msgs::msg::Point32 vertex;vertex.x=p[0];vertex.y=p[1];vertex.z=0;
    result.points.push_back(vertex);
  }
  return result;
}
void validate_limits(const astribot_navigation_msgs::msg::RobotEnvelope& e,
                     const nlohmann::json& p) {
  require(!e.posture_id.empty() && e.frame_id==p.at("base_frame").get<std::string>(),"invalid posture/frame");
  require(0<e.lease_s && e.lease_s<=.5,"invalid envelope lease");
  const std::vector<std::pair<std::string,double>> fields{
    {"half_length_m",e.half_length_m},{"half_width_m",e.half_width_m},{"height_m",e.height_m},
    {"payload_mass_kg",e.payload_mass_kg},{"max_speed_m_s",e.max_speed_m_s},
    {"max_angular_speed_rad_s",e.max_angular_speed_rad_s},
    {"max_acceleration_m_s2",e.max_acceleration_m_s2},{"brake_deceleration_m_s2",e.brake_deceleration_m_s2}};
  for (const auto& [name,value]:fields)
    require(std::isfinite(value) && value>=0 && (name=="payload_mass_kg" || value!=0),name);
  for (std::size_t i=0;i<3;++i)
    require(fields[i].second>=p.at(fields[i].first).get<double>(),"envelope cannot undercut baseline: "+fields[i].first);
  for (std::size_t i=4;i<fields.size();++i)
    require(fields[i].second<=p.at(fields[i].first).get<double>(),"unvalidated limit increase: "+fields[i].first);
}
}

std::int64_t fixed_stamp_ns(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::int64_t>(stamp.sec)*1000000000LL+stamp.nanosec;
}
builtin_interfaces::msg::Time fixed_stamp(std::int64_t ns) {
  builtin_interfaces::msg::Time result;
  auto sec=ns/1000000000LL, sub=ns%1000000000LL;
  if(sub<0) {--sec;sub+=1000000000LL;}
  require(sec>=std::numeric_limits<std::int32_t>::min() && sec<=std::numeric_limits<std::int32_t>::max(),
    "time outside ROS int32 seconds");
  result.sec=static_cast<std::int32_t>(sec);result.nanosec=static_cast<std::uint32_t>(sub);
  return result;
}
FixedEnvelopeCore::FixedEnvelopeCore(nlohmann::json baseline,std::string session,std::uint64_t epoch)
    :baseline_(std::move(baseline)),session_(std::move(session)),epoch_(epoch) {
  require(baseline_.at("environment")=="simulation","FIXED_V2_HARDWARE_NOT_VALIDATED");
}
void FixedEnvelopeCore::state(const State& message) {
  if(current_ && message.source_id==current_->source_id && message.clock_epoch==current_->clock_epoch) {
    if(message.sequence<current_->sequence)return;
    if(message.sequence==current_->sequence) {
      if(message!=*current_)revoke("GEOMETRY_CONFLICTING_SEQUENCE");
      return;
    }
  } else if(current_) {
    // A restarted source/clock may reset its sequence. Old history cannot be
    // addressed by sequence alone across that boundary.
    history_.clear();history_order_.clear();
  }
  current_=std::make_shared<State>(message);
  const Key key{message.source_id,message.sequence};
  if(!history_.count(key))history_order_.push_back(key);
  history_[key]=current_;
  while(history_order_.size()>8) {history_.erase(history_order_.front());history_order_.pop_front();}
  if(!message.complete || !message.attachment_state_confirmed)revoke("GEOMETRY_INCOMPLETE: "+message.reason);
  if(output_ && reference_ && (message.source_id!=reference_->source_id || message.clock_epoch!=reference_->clock_epoch ||
      message.model_revision!=reference_->model_revision || message.attachment_revision!=reference_->attachment_revision))
    revoke("GEOMETRY_VERSION_CHANGED");
}
void FixedEnvelopeCore::hold(const Hold& message) {
  // Latch a revocation at receipt. A later positive heartbeat cannot erase a
  // cancel/version transition before the timer observes it.
  if(output_ && (!message.hold_confirmed || message.hold_id!=output_->hold_id ||
      message.attachment_revision!=output_->attachment_revision || message.owner_id.empty() ||
      (hold_ && message.owner_id!=hold_->owner_id)))revoke("ARM_HOLD_UNCONFIRMED");
  hold_=message;
}
void FixedEnvelopeCore::revoke(const std::string& reason) {
  fault_=reason;
  if(output_) {
    output_->navigation_allowed=false;output_->limits.transport_ready=false;
    output_->reason=reason;output_->limits.reason=reason;
  }
}
void FixedEnvelopeCore::validate_state(const std::shared_ptr<State>& ptr,std::int64_t now) const {
  require(ptr && ptr->complete && ptr->attachment_state_confirmed,"GEOMETRY_INCOMPLETE");
  const auto& s=*ptr;
  require(!s.source_id.empty() && !s.model_revision.empty() && !s.attachment_revision.empty(),"GEOMETRY_ID_MISSING");
  require(s.header.frame_id==baseline_.at("base_frame").get<std::string>(),"GEOMETRY_FRAME_MISMATCH");
  const auto source=fixed_stamp_ns(s.header.stamp),until=fixed_stamp_ns(s.valid_until);
  require(0<source && source<=now && now<until && until<=source+500000000LL,"GEOMETRY_EXPIRED");
  const auto size=s.joints.name.size();
  const std::set<std::string> names(s.joints.name.begin(),s.joints.name.end());
  require(size && names.size()==size && s.joints.position.size()==size &&
    s.joint_source_stamps.size()==size && s.joint_position_error_bounds.size()==size,"JOINT_ARRAY_INVALID");
  for(auto q:s.joints.position)require(std::isfinite(q),"JOINT_INVALID");
  for(auto e:s.joint_position_error_bounds)require(std::isfinite(e) && 0<e && e<=.025,"HOLD_ERROR_INVALID");
  auto low=fixed_stamp_ns(s.joint_source_stamps.front()),high=low;
  for(const auto& t:s.joint_source_stamps) {low=std::min(low,fixed_stamp_ns(t));high=std::max(high,fixed_stamp_ns(t));}
  require(low==source && high<=now && high-low<=100000000LL,"JOINT_TIMING_INVALID");
  require(std::isfinite(s.height_m) && s.height_m>0,"HEIGHT_INVALID");
  require(geometry::containsPolygon(geometry::validatePolygon(points(s.reserved_footprint)),
    geometry::validatePolygon(points(s.physical_footprint))),"RESERVATION_UNDERSIZED");
}
std::int64_t FixedEnvelopeCore::hold_until(const std::string& id,const std::string& revision,std::int64_t now) const {
  require(hold_ && hold_->hold_confirmed && !hold_->owner_id.empty() && hold_->hold_id==id &&
    hold_->attachment_revision==revision,"ARM_HOLD_UNCONFIRMED");
  const auto& h=*hold_;const auto source=fixed_stamp_ns(h.header.stamp);
  require(std::isfinite(h.lease_s) && 0<h.lease_s && h.lease_s<=.5 &&
    now>=source && static_cast<double>(now-source)<h.lease_s*1e9,"ARM_HOLD_EXPIRED");
  return source+static_cast<std::int64_t>(std::nearbyint(h.lease_s*1e9));
}
const FixedEnvelopeCore::Envelope& FixedEnvelopeCore::propose(const Request& request,std::int64_t now,bool stopped) {
  (void)fixed_stamp(now);  // Bound differences before signed source-time arithmetic.
  require(stopped,"ROBOT_MUST_BE_STOPPED_WITH_FRESH_ODOMETRY");
  require(!request.request_id.empty() && !request.hold_id.empty(),"REQUEST_ID_REQUIRED");
  validate_state(current_,now);
  const auto found=history_.find({current_->source_id,request.geometry_sequence});
  const auto reference=found==history_.end()?nullptr:found->second;
  validate_state(reference,now);hold_until(request.hold_id,reference->attachment_revision,now);
  Envelope e;e.header.frame_id=baseline_.at("base_frame").get<std::string>();
  e.coordinator_session_id=session_;e.request_id=request.request_id;e.hold_id=request.hold_id;
  e.mode=Envelope::FIXED_POSTURE;e.reference_state_sequence=reference->sequence;e.clock_epoch=reference->clock_epoch;
  e.model_revision=reference->model_revision;e.attachment_revision=reference->attachment_revision;
  e.reserved_footprint=reference->reserved_footprint;
  e.clearance_m=baseline_.at("clearance_margin_m").get<double>()+baseline_.at("payload_extra_margin_m").get<double>();
  const auto reserved=points(e.reserved_footprint);
  e.installed_footprint=polygon(geometry::inflatePolygon(reserved,e.clearance_m));
  e.installed_geometry_hash=geometry::geometryHash(points(e.installed_footprint),e.header.frame_id,e.clearance_m);
  e.limits=request.limits;
  require(reference->attachment_ids.empty() || !(e.limits.payload_mass_kg<=0),"PAYLOAD_MASS_REQUIRED");
  double x=baseline_.at("half_length_m").get<double>(),y=baseline_.at("half_width_m").get<double>();
  for(const auto& p:reserved) {x=std::max(x,std::abs(p[0]));y=std::max(y,std::abs(p[1]));}
  e.limits.half_length_m=x;e.limits.half_width_m=y;
  e.limits.height_m=std::max(baseline_.at("height_m").get<double>(),reference->height_m);
  validate_limits(e.limits,baseline_);
  reference_=reference;output_=e;++epoch_;output_->epoch=epoch_;output_->limits.epoch=epoch_;
  acks_.clear();ack_revocations_.clear();fault_.clear();tick(now);
  require(fault_.empty(),fault_);
  return *output_;
}
void FixedEnvelopeCore::acknowledge(const Ack& message,std::int64_t now) {
  (void)fixed_stamp(now);
  if(!output_ || !consumers.count(message.consumer_id))return;
  const auto& e=*output_;
  if(message.coordinator_session_id!=session_ || message.envelope_epoch!=e.epoch ||
     message.installed_geometry_hash!=e.installed_geometry_hash)return;
  const auto source=fixed_stamp_ns(message.header.stamp);
  if(!message.applied) {
    auto boundary=ack_revocations_.find(message.consumer_id);
    if(boundary==ack_revocations_.end())ack_revocations_[message.consumer_id]=source;
    else boundary->second=std::max(boundary->second,source);
    acks_.erase(message.consumer_id);tick(now);
  }
  else if(now>=source && now-source<=500000000LL &&
      (!ack_revocations_.count(message.consumer_id) || source>ack_revocations_.at(message.consumer_id)))
    acks_[message.consumer_id]=std::max(acks_[message.consumer_id],source);
}
const std::optional<FixedEnvelopeCore::Envelope>& FixedEnvelopeCore::tick(std::int64_t now) {
  (void)fixed_stamp(now);
  if(now<last_now_) {revoke("CLOCK_RESET");history_.clear();history_order_.clear();}
  last_now_=now;
  if(!output_)return output_;
  auto& e=*output_;e.header.stamp=fixed_stamp(now);e.navigation_allowed=false;e.limits.transport_ready=false;
  try {
    require(fault_.empty(),fault_);validate_state(current_,now);
    const auto& s=*current_;const auto& r=*reference_;
    require(s.source_id==r.source_id && s.clock_epoch==r.clock_epoch && s.model_revision==r.model_revision &&
      s.attachment_revision==r.attachment_revision,"GEOMETRY_VERSION_CHANGED");
    std::map<std::string,double> actual;
    for(std::size_t i=0;i<s.joints.name.size();++i)actual[s.joints.name[i]]=s.joints.position[i];
    require(actual.size()==r.joints.name.size(),"FIXED_POSTURE_LEFT_RESERVATION");
    for(std::size_t i=0;i<r.joints.name.size();++i) {
      const auto found=actual.find(r.joints.name[i]);
      require(found!=actual.end() && std::abs(found->second-r.joints.position[i])<=r.joint_position_error_bounds[i],
        "FIXED_POSTURE_LEFT_RESERVATION");
    }
    require(geometry::containsPolygon(points(e.reserved_footprint),points(s.physical_footprint),1e-6),
      "PHYSICAL_GEOMETRY_LEFT_RESERVATION");
    const auto until=std::min(fixed_stamp_ns(s.valid_until),hold_until(e.hold_id,e.attachment_revision,now));
    e.source_state_sequence=s.sequence;e.valid_until=fixed_stamp(until);
    std::vector<std::string> missing;
    for(const auto& consumer:consumers) {
      const auto found=acks_.find(consumer);
      if(found==acks_.end() || now<found->second || now-found->second>=500000000LL)missing.push_back(consumer);
    }
    e.navigation_allowed=missing.empty();e.reason=missing.empty()?"READY_FIXED":"WAITING_FOR:";
    for(std::size_t i=0;i<missing.size();++i)e.reason+=(i?",":"")+missing[i];
  } catch(const std::invalid_argument& error) {
    revoke(error.what());e.reason=fault_;e.valid_until=fixed_stamp(now);
  }
  e.limits.transport_ready=e.navigation_allowed;e.limits.stamp=e.header.stamp;e.limits.reason=e.reason;
  return output_;
}
}  // namespace astribot::navigation
