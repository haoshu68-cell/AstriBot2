// Test transport only. Production callbacks pass generated ROS messages directly.
#include "astribot_s1_navigation_policy_native/policy_envelope.hpp"
#include "astribot_s1_navigation_policy_native/policy_profile.hpp"
#include <cmath>
#include <iostream>
#include <limits>

using nlohmann::json;
using namespace astribot::navigation::policy;
using E=PolicyEnvelopeProfile::Envelope;
using V=PolicyEnvelopeProfile::EnvelopeV2;
using Field=std::pair<const char*,double E::*>;
const std::array<Field,8> fields{{{"half_length_m",&E::half_length_m},{"half_width_m",&E::half_width_m},
  {"height_m",&E::height_m},{"payload_mass_kg",&E::payload_mass_kg},{"max_speed_m_s",&E::max_speed_m_s},
  {"max_angular_speed_rad_s",&E::max_angular_speed_rad_s},{"max_acceleration_m_s2",&E::max_acceleration_m_s2},
  {"brake_deceleration_m_s2",&E::brake_deceleration_m_s2}}};
double number(const json& d) {
  if(d.is_number()) return d.get<double>();
  const auto s=d.get<std::string>();
  if(s=="nan") return std::numeric_limits<double>::quiet_NaN();
  return s=="inf" ? std::numeric_limits<double>::infinity() : -std::numeric_limits<double>::infinity();
}
json encoded(double x) {
  if(std::isnan(x)) return "nan";
  if(std::isinf(x)) return x>0 ? "inf" : "-inf";
  return x;
}
builtin_interfaces::msg::Time wire_stamp(std::int64_t n) {
  builtin_interfaces::msg::Time t;auto sec=n/1000000000LL, rem=n%1000000000LL;
  if(rem<0) {--sec;rem+=1000000000LL;}
  t.sec=static_cast<std::int32_t>(sec);t.nanosec=static_cast<std::uint32_t>(rem);return t;
}
E envelope(const json& d) {
  E e;e.stamp=wire_stamp(d.at("stamp").get<std::int64_t>());e.epoch=d.at("epoch");
  e.lease_s=number(d.at("lease_s"));e.frame_id=d.at("frame_id");e.posture_id=d.at("posture_id");
  e.transport_ready=d.at("transport_ready");e.reason=d.at("reason");
  for(const auto& f:fields) e.*f.second=number(d.at(f.first));
  return e;
}
V envelope_v2(const json& d) {
  V e;e.header.stamp=wire_stamp(d.at("header_stamp").get<std::int64_t>());
  e.header.frame_id=d.at("frame_id");e.valid_until=wire_stamp(d.at("valid_until").get<std::int64_t>());
  e.coordinator_session_id=d.at("coordinator_session_id");e.request_id=d.at("request_id");e.hold_id=d.at("hold_id");
  e.epoch=d.at("epoch");e.clock_epoch=d.at("clock_epoch");e.reference_state_sequence=d.at("reference_state_sequence");
  e.source_state_sequence=d.at("source_state_sequence");e.model_revision=d.at("model_revision");
  e.attachment_revision=d.at("attachment_revision");e.mode=d.at("mode");e.limits=envelope(d.at("limits"));
  e.installed_geometry_hash=d.at("installed_geometry_hash");e.clearance_m=number(d.at("clearance_m"));
  e.navigation_allowed=d.at("navigation_allowed");e.reason=d.at("reason");
  for(const auto name:{"reserved_footprint","installed_footprint"}) {
    auto& out=std::string(name)=="reserved_footprint" ? e.reserved_footprint : e.installed_footprint;
    for(const auto& p:d.at(name)) {
      geometry_msgs::msg::Point32 q;q.x=static_cast<float>(number(p[0]));q.y=static_cast<float>(number(p[1]));
      q.z=static_cast<float>(number(p[2]));out.points.push_back(q);
    }
  }
  return e;
}
json encode_envelope(const E& e) {
  json d={{"stamp",static_cast<std::int64_t>(e.stamp.sec)*1000000000LL+e.stamp.nanosec},
    {"epoch",e.epoch},{"lease_s",e.lease_s},{"frame_id",e.frame_id},{"posture_id",e.posture_id},
    {"transport_ready",e.transport_ready},{"reason",e.reason}};
  for(const auto& f:fields) d[f.first]=e.*f.second;
  return d;
}
json state(const PolicyEnvelopeProfile& p,const Stamp& now) {
  json result={{"ready",p.ready(now)},{"envelope",p.envelope() ? encode_envelope(*p.envelope()) : json(nullptr)}};
  result["values"]=json::object();for(const auto& f:fields) result["values"][f.first]=p.value(f.first);
  result["polygon"]=p.polygon() ? json(*p.polygon()) : json(nullptr);
  result["stopping"]=json::array();
  for(double v:{0.,-.1,.1,1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
    result["stopping"].push_back(encoded(p.stopping_distance(v)));
  return result;
}
int main() {
  try {
    json input;std::cin>>input;
    PolicyEnvelopeProfile p(astribot::navigation::load_policy_profile(input.at("profile_path").get<std::string>(),true), input.at("fixed"));
    json output=json::array();
    for(const auto& op:input.at("ops")) {
      const auto& n=op.at("now");Stamp now(n.at("ns"),n.at("clock"),n.at("epoch"));json row;
      try {
        if(op.at("op")=="accept") row["value"]=input.at("fixed").get<bool>() ?
          p.accept(envelope_v2(op.at("message")),now) : p.accept(envelope(op.at("message")),now);
        else if(op.at("op")=="confirm") row["value"]=p.confirms_applied(envelope_v2(op.at("message")),now);
        else row["value"]=p.ready(now);
      } catch(const std::exception& error) {row["error"]=error.what();}
      row["state"]=state(p,now);output.push_back(row);
    }
    std::cout<<output.dump()<<'\n';return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
