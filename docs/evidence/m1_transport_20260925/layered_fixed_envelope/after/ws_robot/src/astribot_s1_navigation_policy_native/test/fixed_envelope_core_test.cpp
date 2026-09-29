#include "astribot_s1_navigation_policy_native/fixed_envelope_core.hpp"
#include "astribot_s1_navigation_policy_native/policy_profile.hpp"
#include <stdexcept>
#include <iostream>
#include <limits>
#include <astribot_s1_robot_geometry/layered_envelope.hpp>
using namespace astribot::navigation;
namespace {
void check(bool ok,const char* message) {if(!ok)throw std::runtime_error(message);}
geometry_msgs::msg::Polygon square(float size) {
  geometry_msgs::msg::Polygon p;
  for(const auto& pair:std::vector<std::pair<float,float>>{{-size,-size},{size,-size},{size,size},{-size,size}}) {
    geometry_msgs::msg::Point32 v;v.x=pair.first;v.y=pair.second;p.points.push_back(v);
  }
  return p;
}
}
int main(int argc,char** argv) {
  try {
    check(argc==2,"profile required");const auto p=load_policy_profile(argv[1],true);
    const auto high=static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())*1000000000LL+999999999LL;
    const auto low=static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::min())*1000000000LL;
    check(fixed_stamp_ns(fixed_stamp(high))==high,"maximum ROS stamp must roundtrip");
    check(fixed_stamp_ns(fixed_stamp(low))==low,"minimum ROS stamp must roundtrip");
    for(const auto bad:std::vector<std::int64_t>{high+1,low-1,std::numeric_limits<std::int64_t>::max(),std::numeric_limits<std::int64_t>::min()}) {
      bool rejected=false;try {fixed_stamp(bad);}catch(const std::invalid_argument&) {rejected=true;}
      check(rejected,"out-of-range source time must not narrow or overflow");
      FixedEnvelopeCore bounded(p,"time-boundary",1);rejected=false;
      try {bounded.tick(bad);}catch(const std::invalid_argument&) {rejected=true;}
      check(rejected,"core must reject out-of-range input time before arithmetic");
    }
    constexpr std::int64_t now=10000000000LL;
    FixedEnvelopeCore c(p,"unit-session",1);FixedEnvelopeCore::State s;
    s.header.frame_id=p.at("base_frame");s.header.stamp=fixed_stamp(now);
    s.valid_until=fixed_stamp(now+300000000LL);s.source_id="geometry";s.sequence=1;
    s.model_revision="model";s.attachment_revision="empty";s.complete=true;s.attachment_state_confirmed=true;
    s.joints.name={"arm"};s.joints.position={0.};s.joint_source_stamps={s.header.stamp};s.joint_position_error_bounds={.003};
    s.physical_footprint=square(.3F);s.reserved_footprint=square(.4F);s.height_m=1.7;
    s.height_profile_revision=astribot_s1_robot_geometry::sha256("unit-height-profile");s.ground_in_base_m=-.095;
    const std::vector<double> edges{-.045,.155,.585,1.085,1.535,2.205};
    for(std::size_t i=1;i<edges.size();++i) {
      astribot_navigation_msgs::msg::EnvelopeSlice layer;layer.z_min_m=edges[i-1];layer.z_max_m=edges[i];
      if(i!=3)layer.footprint=s.reserved_footprint;
      s.height_slices.push_back(layer);
    }
    FixedEnvelopeCore::Hold h;h.header.stamp=s.header.stamp;h.owner_id="task";h.hold_id="hold";
    h.attachment_revision="empty";h.hold_confirmed=true;h.lease_s=.3;
    FixedEnvelopeCore::Request r;r.request_id="request";r.hold_id="hold";r.geometry_sequence=1;
    r.limits.frame_id=p.at("base_frame");r.limits.posture_id="nonhome";r.limits.lease_s=.3;
#define LIMIT(field) r.limits.field=p.at(#field).get<double>()
    LIMIT(half_length_m);LIMIT(half_width_m);LIMIT(height_m);LIMIT(payload_mass_kg);
    LIMIT(max_speed_m_s);LIMIT(max_angular_speed_rad_s);LIMIT(max_acceleration_m_s2);LIMIT(brake_deceleration_m_s2);
#undef LIMIT
    auto invalid_state=s;invalid_state.attachment_ids={"payload"};
    auto invalid_request=r;invalid_request.limits.payload_mass_kg=std::numeric_limits<double>::quiet_NaN();
    FixedEnvelopeCore invalid(p,"nan-payload",1);invalid.state(invalid_state);invalid.hold(h);
    std::string diagnostic;
    try {invalid.propose(invalid_request,now,true);}catch(const std::invalid_argument& e) {diagnostic=e.what();}
    check(diagnostic=="payload_mass_kg","NaN payload must preserve validation diagnostic");
    c.state(s);c.hold(h);check(!c.propose(r,now,true).navigation_allowed,"proposal requires all consumers");
    FixedEnvelopeCore::Ack a;a.coordinator_session_id=c.session();a.envelope_epoch=c.epoch();
    a.installed_geometry_hash=c.output()->installed_geometry_hash;a.applied=true;a.header.stamp=fixed_stamp(now);
    a.consumer_id="protection";c.acknowledge(a,now);
    check(!c.tick(now)->navigation_allowed && !c.acknowledgements().count("protection"),"legacy protection ACK has no authorization role");
    for(const auto* name:{"global_costmap","local_costmap","planner","policy"}) {
      a.consumer_id=name;c.acknowledge(a,now);
    }
    check(!c.tick(now)->navigation_allowed && c.output()->reason=="WAITING_FOR:controller","missing controller must still deny");
    a.consumer_id="protection";c.acknowledge(a,now);
    check(!c.tick(now)->navigation_allowed,"legacy protection cannot replace the controller ACK");
    a.consumer_id="controller";c.acknowledge(a,now);
    check(c.tick(now)->navigation_allowed && c.acknowledgements().size()==5,"five current geometry consumers must grant");
    a.consumer_id="protection";a.applied=false;c.acknowledge(a,now);
    check(c.tick(now)->navigation_allowed && !c.acknowledgements().count("protection"),"legacy protection negative ACK has no revocation role");
    a.applied=true;
    a.consumer_id="planner";a.header.stamp=fixed_stamp(now+1);c.acknowledge(a,now);
    check(c.acknowledgements().at("planner")==now,"future ACK must not extend evidence");
    a.applied=false;c.acknowledge(a,now);
    check(!c.tick(now)->navigation_allowed,"negative future ACK revokes immediately");
    a.applied=true;a.header.stamp=fixed_stamp(now+2);c.acknowledge(a,now+2);
    check(c.tick(now+2)->navigation_allowed,"post-revocation positive ACK restores only unlatched gate");
    check(!c.tick(now+300000000LL)->navigation_allowed,"source deadline is exclusive");
    check(c.fault()=="GEOMETRY_EXPIRED","heartbeat cannot renew source lease");
    s.sequence=2;s.header.stamp=fixed_stamp(now+300000000LL);s.joint_source_stamps={s.header.stamp};
    s.valid_until=fixed_stamp(now+600000000LL);h.header.stamp=s.header.stamp;c.state(s);c.hold(h);
    check(!c.tick(now+300000000LL)->navigation_allowed,"expired grant requires a new proposal");
    c.tick(now-1);check(c.fault()=="CLOCK_RESET","clock reversal revokes geometry commitment");
    std::cout<<"fixed envelope authority and source lease invariants passed\n";return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
