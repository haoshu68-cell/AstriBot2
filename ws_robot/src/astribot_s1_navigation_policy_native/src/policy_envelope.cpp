#include "astribot_s1_navigation_policy_native/policy_envelope.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace astribot::navigation::policy {
namespace {
using E = PolicyEnvelopeProfile::Envelope;
using V2 = PolicyEnvelopeProfile::EnvelopeV2;
using Polygon = PolicyEnvelopeProfile::Polygon;
using Field = std::pair<const char*, double E::*>;
const std::array<Field, 8> fields{{
  {"half_length_m", &E::half_length_m}, {"half_width_m", &E::half_width_m},
  {"height_m", &E::height_m}, {"payload_mass_kg", &E::payload_mass_kg},
  {"max_speed_m_s", &E::max_speed_m_s}, {"max_angular_speed_rad_s", &E::max_angular_speed_rad_s},
  {"max_acceleration_m_s2", &E::max_acceleration_m_s2}, {"brake_deceleration_m_s2", &E::brake_deceleration_m_s2}}};
std::int64_t ns(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::int64_t>(stamp.sec)*1000000000LL+stamp.nanosec;
}
bool equal_fields(const E& a, const E& b) {
  for (const auto& f : fields) if (a.*f.second != b.*f.second) return false;
  return true;
}
bool equal_polygon(const geometry_msgs::msg::Polygon& a, const geometry_msgs::msg::Polygon& b) {
  if (a.points.size()!=b.points.size()) return false;
  for (std::size_t i=0;i<a.points.size();++i) {
    const auto& p=a.points[i];const auto& q=b.points[i];
    if (p.x!=q.x || p.y!=q.y || p.z!=q.z) return false;
  }
  return true;
}
bool equal_configuration(const V2& a, const V2& b) {
  return a.coordinator_session_id==b.coordinator_session_id && a.epoch==b.epoch &&
    a.clock_epoch==b.clock_epoch && a.request_id==b.request_id && a.hold_id==b.hold_id &&
    a.reference_state_sequence==b.reference_state_sequence && a.model_revision==b.model_revision &&
    a.attachment_revision==b.attachment_revision && a.mode==b.mode &&
    a.header.frame_id==b.header.frame_id && a.clearance_m==b.clearance_m &&
    a.installed_geometry_hash==b.installed_geometry_hash &&
    equal_polygon(a.reserved_footprint,b.reserved_footprint) &&
    equal_polygon(a.installed_footprint,b.installed_footprint) && equal_fields(a.limits,b.limits) &&
    a.limits.epoch==b.limits.epoch && a.limits.posture_id==b.limits.posture_id &&
    a.limits.frame_id==b.limits.frame_id && a.limits.lease_s==b.limits.lease_s;
}
Polygon validate_polygon(const geometry_msgs::msg::Polygon& message) {
  if (message.points.size()<3 || message.points.size()>256)
    throw std::invalid_argument("invalid polygon vertices");
  Polygon p;p.reserve(message.points.size());
  for (const auto& point:message.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y))
      throw std::invalid_argument("invalid polygon vertices");
    p.push_back({point.x,point.y});
  }
  return astribot_s1_robot_geometry::validatePolygon(p);
}
}  // namespace

PolicyEnvelopeProfile::PolicyEnvelopeProfile(nlohmann::json validated_baseline, bool fixed)
  : baseline(std::move(validated_baseline)), fixed_(fixed) {}
const std::string& PolicyEnvelopeProfile::base_frame() const {
  return baseline.at("base_frame").get_ref<const std::string&>();
}
const std::string& PolicyEnvelopeProfile::tracking_frame() const {
  return baseline.at("tracking_frame").get_ref<const std::string&>();
}
double PolicyEnvelopeProfile::value(const std::string& key) const {
  if (envelope_) for (const auto& f:fields) if (key==f.first) return (*envelope_).*f.second;
  return baseline.at(key).get<double>();
}
void PolicyEnvelopeProfile::validate(const Envelope& message) const {
  if (message.posture_id.empty() || message.frame_id!=base_frame())
    throw std::invalid_argument("invalid posture/frame");
  if (!(message.lease_s>0. && message.lease_s<=.5))
    throw std::invalid_argument("invalid envelope lease");
  for (std::size_t i=0;i<fields.size();++i) {
    const auto& f=fields[i];const double v=message.*f.second;
    if (!std::isfinite(v) || v<0. || (i!=3 && v==0.)) throw std::invalid_argument(f.first);
  }
  for (std::size_t i=0;i<3;++i) {
    const auto& f=fields[i];
    if (message.*f.second<baseline.at(f.first).get<double>())
      throw std::invalid_argument(std::string("envelope cannot undercut baseline: ")+f.first);
  }
  for (std::size_t i=4;i<fields.size();++i) {
    const auto& f=fields[i];
    if (message.*f.second>baseline.at(f.first).get<double>())
      throw std::invalid_argument(std::string("unvalidated limit increase: ")+f.first);
  }
}
void PolicyEnvelopeProfile::revoke() {
  v2_.reset();envelope_.reset();received_.reset();
  // FixedEnvelopeProfile deliberately retains its last canonical footprint.
}
bool PolicyEnvelopeProfile::accept(const Envelope& message, const Stamp& now) {
  if (fixed_) {revoke();throw std::invalid_argument("fixed profile requires V2 envelope");}
  validate(message);
  if (envelope_ && message.epoch<envelope_->epoch) return false;
  if (envelope_ && message.epoch==envelope_->epoch &&
      (!equal_fields(message,*envelope_) || message.posture_id!=envelope_->posture_id ||
       message.frame_id!=envelope_->frame_id))
    throw std::invalid_argument("geometry and limits require a new envelope epoch");
  if(envelope_ && message.epoch==envelope_->epoch && ns(message.stamp)<ns(envelope_->stamp))return false;
  envelope_=message;received_.emplace(now);return true;
}
bool PolicyEnvelopeProfile::accept(const EnvelopeV2& message, const Stamp& now) {
  if (!fixed_) throw std::invalid_argument("legacy profile requires RobotEnvelope");
  if(v2_ && message.coordinator_session_id==v2_->coordinator_session_id &&
      message.clock_epoch==v2_->clock_epoch && message.epoch==v2_->epoch &&
      ns(message.header.stamp)<ns(v2_->header.stamp))return false;
  try {
    validate(message.limits);
    if (message.header.frame_id!=base_frame() || message.mode!=EnvelopeV2::FIXED_POSTURE)
      throw std::invalid_argument("invalid V2 frame/mode");
    if (message.coordinator_session_id.empty() || message.hold_id.empty())
      throw std::invalid_argument("missing V2 identity");
    if (!std::isfinite(message.clearance_m) ||
        message.clearance_m<baseline.at("clearance_margin_m").get<double>()+baseline.at("payload_extra_margin_m").get<double>())
      throw std::invalid_argument("V2 clearance undercut");
    auto reserved=validate_polygon(message.reserved_footprint);
    const auto installed=validate_polygon(message.installed_footprint);
    if (!astribot_s1_robot_geometry::containsPolygon(installed,
        astribot_s1_robot_geometry::inflatePolygon(reserved,message.clearance_m),2e-6))
      throw std::invalid_argument("V2 installed footprint undercut");
    if (astribot_s1_robot_geometry::geometryHash(installed,message.header.frame_id,message.clearance_m)!=message.installed_geometry_hash)
      throw std::invalid_argument("V2 hash mismatch");
    for (const auto& p:reserved) if (std::abs(p[0])>message.limits.half_length_m+1e-6 ||
                                   std::abs(p[1])>message.limits.half_width_m+1e-6)
      throw std::invalid_argument("V2 broad-phase undercut");
    if (v2_ && message.coordinator_session_id==v2_->coordinator_session_id) {
      if (message.epoch<v2_->epoch) return false;
      if (message.epoch==v2_->epoch && (message.installed_geometry_hash!=v2_->installed_geometry_hash ||
                                     !equal_fields(message.limits,*envelope_)))
        throw std::invalid_argument("V2 mutated epoch");
    }
    envelope_=message.limits;v2_=message;received_.emplace(now);polygon_=std::move(reserved);
    return true;
  } catch (const std::invalid_argument&) {
    revoke();throw;
  }
}
bool PolicyEnvelopeProfile::ready(const Stamp& now) const {
  (void)now;
  return envelope_ && envelope_->transport_ready && (!fixed_ || (v2_ && v2_->navigation_allowed));
}
bool PolicyEnvelopeProfile::confirms_applied(const EnvelopeV2& message, const Stamp& now) const {
  (void)now;
  return fixed_ && v2_ && equal_configuration(message,*v2_);
}

double PolicyEnvelopeProfile::stopping_distance(double speed) const {
  return speed*(value("reaction_time_s")+baseline.value("linear_stop_delay_s",0.))+
    speed*speed/(2*value("brake_deceleration_m_s2"))+value("clearance_margin_m");
}
}  // namespace astribot::navigation::policy
