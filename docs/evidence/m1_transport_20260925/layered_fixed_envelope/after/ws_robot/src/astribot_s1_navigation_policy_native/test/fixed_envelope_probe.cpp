#include "astribot_s1_navigation_policy_native/fixed_envelope_core.hpp"
#include "astribot_s1_navigation_policy_native/policy_profile.hpp"
#include <iostream>
#include <nlohmann/json.hpp>

using Json=nlohmann::json;
using namespace astribot::navigation;
namespace {
using Time=builtin_interfaces::msg::Time;
Time time(const Json& j) {Time t;t.sec=j.at("sec");t.nanosec=j.at("nanosec");return t;}
Json json(const Time& t) {return {{"sec",t.sec},{"nanosec",t.nanosec}};}
std_msgs::msg::Header header(const Json& j) {
  std_msgs::msg::Header h;h.stamp=time(j.at("stamp"));h.frame_id=j.at("frame_id");return h;
}
Json json(const std_msgs::msg::Header& h) {return {{"stamp",json(h.stamp)},{"frame_id",h.frame_id}};}
geometry_msgs::msg::Polygon polygon(const Json& j) {
  geometry_msgs::msg::Polygon p;
  for(const auto& v:j.at("points")) {geometry_msgs::msg::Point32 q;q.x=v.at("x");q.y=v.at("y");q.z=v.at("z");p.points.push_back(q);}
  return p;
}
Json json(const geometry_msgs::msg::Polygon& p) {
  Json a=Json::array();for(const auto& v:p.points)a.push_back({{"x",v.x},{"y",v.y},{"z",v.z}});
  return {{"points",a}};
}
std::vector<astribot_navigation_msgs::msg::EnvelopeSlice> slices(const Json& j) {
  std::vector<astribot_navigation_msgs::msg::EnvelopeSlice> result;
  for(const auto& entry:j) {
    astribot_navigation_msgs::msg::EnvelopeSlice layer;
    layer.z_min_m=entry.at("z_min_m");layer.z_max_m=entry.at("z_max_m");layer.footprint=polygon(entry.at("footprint"));
    result.push_back(layer);
  }
  return result;
}
Json json(const std::vector<astribot_navigation_msgs::msg::EnvelopeSlice>& slices) {
  Json result=Json::array();
  for(const auto& s:slices)result.push_back({{"z_min_m",s.z_min_m},{"z_max_m",s.z_max_m},{"footprint",json(s.footprint)}});
  return result;
}
#define READ(f) result.f=j.at(#f).get<decltype(result.f)>()
#define WRITE(f) result[#f]=e.f
astribot_navigation_msgs::msg::RobotEnvelope limits(const Json& j) {
  astribot_navigation_msgs::msg::RobotEnvelope result;result.stamp=time(j.at("stamp"));
  READ(epoch);READ(posture_id);READ(frame_id);READ(half_length_m);READ(half_width_m);READ(height_m);
  READ(payload_mass_kg);READ(max_speed_m_s);READ(max_angular_speed_rad_s);READ(max_acceleration_m_s2);
  READ(brake_deceleration_m_s2);READ(lease_s);READ(transport_ready);READ(reason);return result;
}
Json json(const astribot_navigation_msgs::msg::RobotEnvelope& e) {
  Json result;result["stamp"]=json(e.stamp);
  WRITE(epoch);WRITE(posture_id);WRITE(frame_id);WRITE(half_length_m);WRITE(half_width_m);WRITE(height_m);
  WRITE(payload_mass_kg);WRITE(max_speed_m_s);WRITE(max_angular_speed_rad_s);WRITE(max_acceleration_m_s2);
  WRITE(brake_deceleration_m_s2);WRITE(lease_s);WRITE(transport_ready);WRITE(reason);return result;
}
FixedEnvelopeCore::State state(const Json& j) {
  FixedEnvelopeCore::State result;result.header=header(j.at("header"));
  result.published_at=time(j.at("published_at"));result.valid_until=time(j.at("valid_until"));
  READ(source_id);READ(sequence);READ(clock_epoch);READ(model_revision);READ(attachment_revision);
  READ(joint_position_error_bounds);READ(complete);READ(attachment_state_confirmed);READ(height_m);READ(attachment_ids);READ(reason);
  READ(height_profile_revision);READ(ground_in_base_m);result.height_slices=slices(j.at("height_slices"));
  const auto& joints=j.at("joints");result.joints.header=header(joints.at("header"));
  result.joints.name=joints.at("name").get<std::vector<std::string>>();
  result.joints.position=joints.at("position").get<std::vector<double>>();
  result.joints.velocity=joints.at("velocity").get<std::vector<double>>();
  result.joints.effort=joints.at("effort").get<std::vector<double>>();
  for(const auto& v:j.at("joint_source_stamps"))result.joint_source_stamps.push_back(time(v));
  result.physical_footprint=polygon(j.at("physical_footprint"));result.reserved_footprint=polygon(j.at("reserved_footprint"));
  return result;
}
FixedEnvelopeCore::Hold hold(const Json& j) {
  FixedEnvelopeCore::Hold result;result.header=header(j.at("header"));
  READ(owner_id);READ(hold_id);READ(lease_s);READ(hold_confirmed);READ(attachment_revision);return result;
}
FixedEnvelopeCore::Request request(const Json& j) {
  FixedEnvelopeCore::Request result;READ(request_id);READ(hold_id);READ(geometry_sequence);
  result.limits=limits(j.at("limits"));return result;
}
Json json(const FixedEnvelopeCore::Envelope& e) {
  Json result;result["header"]=json(e.header);result["valid_until"]=json(e.valid_until);
  WRITE(coordinator_session_id);WRITE(request_id);WRITE(hold_id);WRITE(epoch);WRITE(clock_epoch);
  WRITE(reference_state_sequence);WRITE(source_state_sequence);WRITE(model_revision);WRITE(attachment_revision);
  WRITE(mode);WRITE(installed_geometry_hash);WRITE(clearance_m);WRITE(navigation_allowed);WRITE(reason);
  WRITE(height_profile_revision);WRITE(ground_in_base_m);WRITE(height_geometry_hash);result["height_slices"]=json(e.height_slices);
  result["limits"]=json(e.limits);result["reserved_footprint"]=json(e.reserved_footprint);
  result["installed_footprint"]=json(e.installed_footprint);return result;
}
#undef READ
#undef WRITE
}
int main(int argc,char** argv) {
  try {
    if(argc!=2)throw std::invalid_argument("profile path required");
    FixedEnvelopeCore core(load_policy_profile(argv[1],true),"test-session",100);
    const auto events=Json::parse(std::cin);Json records=Json::array();
    for(const auto& step:events) {
      const auto op=step.at("op").get<std::string>();const auto now=step.at("now").get<std::int64_t>();
      std::string error;
      try {
        if(op=="state")core.state(state(step.at("message")));
        else if(op=="hold")core.hold(hold(step.at("message")));
        else if(op=="propose")core.propose(request(step.at("message")),now,step.at("stopped"));
        else if(op=="tick")core.tick(now);
        else if(op=="revoke")core.revoke(step.at("reason"));
        else if(op=="ack") {
          FixedEnvelopeCore::Ack a;a.coordinator_session_id=core.session();a.consumer_id=step.at("consumer");
          a.envelope_epoch=core.output()?core.output()->epoch:0;
          a.installed_geometry_hash=core.output()?core.output()->installed_geometry_hash:"";
          a.applied=true;a.header.stamp=fixed_stamp(step.value("stamp",now));
          const auto v=step.value("override",Json::object());
          if(v.contains("coordinator_session_id"))a.coordinator_session_id=v.at("coordinator_session_id");
          if(v.contains("envelope_epoch"))a.envelope_epoch=v.at("envelope_epoch");
          if(v.contains("installed_geometry_hash"))a.installed_geometry_hash=v.at("installed_geometry_hash");
          if(v.contains("applied"))a.applied=v.at("applied");
          core.acknowledge(a,now);
        } else throw std::invalid_argument("unknown operation");
      } catch(const std::invalid_argument& e) {error=e.what();}
      records.push_back({{"error",error},{"epoch",core.epoch()},{"fault",core.fault()},
        {"acks",core.acknowledgements()},{"output",core.output()?json(*core.output()):Json(nullptr)}});
    }
    std::cout<<records.dump()<<'\n';return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
