#include "astribot_s1_navigation_policy_native/fixed_envelope_core.hpp"
#include "astribot_s1_navigation_policy_native/policy_profile.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <astribot_navigation_msgs/srv/reserve_arm_motion.hpp>
#include <astribot_navigation_msgs/srv/set_robot_envelope.hpp>

namespace astribot::navigation {
namespace {
std::string new_session() {
  std::random_device random;std::array<unsigned char,16> bytes{};
  for(auto& b:bytes)b=static_cast<unsigned char>(random());
  bytes[6]=(bytes[6]&0x0fU)|0x40U;bytes[8]=(bytes[8]&0x3fU)|0x80U;
  const char* digits="0123456789abcdef";std::string result;
  for(std::size_t i=0;i<bytes.size();++i) {
    if(i==4 || i==6 || i==8 || i==10)result+='-';
    result+=digits[bytes[i]>>4];result+=digits[bytes[i]&15];
  }
  return result;
}
class FixedEnvelopeCoordinatorNode final:public rclcpp::Node {
public:
  FixedEnvelopeCoordinatorNode():Node("robot_envelope_coordinator") {
    declare_parameter<std::string>("profile",ament_index_cpp::get_package_share_directory(
      "astribot_s1_navigation_policy")+"/config/simulation.json");
    declare_parameter<std::string>("navigation_geometry_mode","fixed_v2");
    if(!has_parameter("use_sim_time"))declare_parameter("use_sim_time",false);
    if(get_parameter("navigation_geometry_mode").as_string()!="fixed_v2")
      throw std::invalid_argument("fixed_envelope_cpp requires navigation_geometry_mode:=fixed_v2");
    const auto baseline=load_policy_profile(get_parameter("profile").as_string(),get_parameter("use_sim_time").as_bool());
    const auto epoch=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
    core_=std::make_unique<FixedEnvelopeCore>(baseline,new_session(),epoch);
    publisher_=create_publisher<FixedEnvelopeCore::Envelope>("/navigation/envelope_v2",10);
    legacy_=create_publisher<astribot_navigation_msgs::msg::RobotEnvelope>("/navigation/robot_envelope",10);
    for(const auto* name:{"global_costmap","local_costmap"})
      footprints_.push_back(create_publisher<geometry_msgs::msg::Polygon>(std::string("/")+name+"/footprint",1));
    state_=create_subscription<FixedEnvelopeCore::State>("/navigation/geometry_state",10,
      [this](FixedEnvelopeCore::State::ConstSharedPtr msg){core_->state(*msg);tick();});
    hold_=create_subscription<FixedEnvelopeCore::Hold>("/navigation/arm_hold",10,
      [this](FixedEnvelopeCore::Hold::ConstSharedPtr msg){core_->hold(*msg);tick();});
    applied_=create_subscription<FixedEnvelopeCore::Ack>("/navigation/envelope_applied",20,
      [this](FixedEnvelopeCore::Ack::ConstSharedPtr msg){core_->acknowledge(*msg,now().nanoseconds());tick(false);});
    odom_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){odom_message_=msg;});
    propose_=create_service<astribot_navigation_msgs::srv::SetFixedEnvelope>("/navigation/set_fixed_envelope",
      [this](const std::shared_ptr<FixedEnvelopeCore::Request> request,
             std::shared_ptr<astribot_navigation_msgs::srv::SetFixedEnvelope::Response> response) {
        try {
          const auto at=now().nanoseconds();bool stopped=false;
          (void)fixed_stamp(at);
          if(odom_message_) {
            const auto age=at-fixed_stamp_ns(odom_message_->header.stamp);const auto& v=odom_message_->twist.twist;
            stopped=age>=0 && age<=300000000LL && std::hypot(v.linear.x,v.linear.y)<=.02 && std::abs(v.angular.z)<=.03;
          }
          if(!stopped)RCLCPP_DEBUG(get_logger(),"fixed posture request stopped check: now=%ld, odom=%ld, vx=%g, vy=%g, wz=%g",
            at,odom_message_?fixed_stamp_ns(odom_message_->header.stamp):-1,
            odom_message_?odom_message_->twist.twist.linear.x:0.,
            odom_message_?odom_message_->twist.twist.linear.y:0.,
            odom_message_?odom_message_->twist.twist.angular.z:0.);
          const auto& e=core_->propose(*request,at,stopped);
          response->accepted=true;response->epoch=e.epoch;response->reason=e.reason;
        } catch(const std::invalid_argument& error) {response->reason=error.what();}
      });
    legacy_request_=create_service<astribot_navigation_msgs::srv::SetRobotEnvelope>("/navigation/set_robot_envelope",
      [this](const std::shared_ptr<astribot_navigation_msgs::srv::SetRobotEnvelope::Request> request,
             std::shared_ptr<astribot_navigation_msgs::srv::SetRobotEnvelope::Response> response) {
        if(request->envelope.transport_ready)response->reason="FIXED_V2_REQUIRES_GEOMETRY_AND_HOLD";
        else {core_->revoke("TASK_HOLD");response->accepted=true;response->epoch=core_->epoch();response->reason="TASK_HOLD";}
        tick();
      });
    reserve_=create_service<astribot_navigation_msgs::srv::ReserveArmMotion>("/navigation/reserve_arm_motion",
      [](const std::shared_ptr<astribot_navigation_msgs::srv::ReserveArmMotion::Request>,
         std::shared_ptr<astribot_navigation_msgs::srv::ReserveArmMotion::Response> response) {
        response->accepted=false;response->reason="RESERVED_ARM_MOTION_NOT_ENABLED";
      });
    timer_=rclcpp::create_timer(this,get_clock(),rclcpp::Duration::from_seconds(.1),[this]{tick();});
  }
private:
  void tick(bool source_update=true) {
    const auto& current=core_->tick(now().nanoseconds());
    // A positive installation heartbeat renews evidence, not its own input.
    // Publishing every ACK closes an unbounded envelope -> ACK -> envelope loop.
    // Always publish permission/reason transitions immediately, including negative
    // ACKs and expiry observed during this callback. Source updates and the normal
    // timer still publish the latest lease and state sequence.
    if(!source_update && (!current || (published_ &&
        published_epoch_==current->epoch && published_allowed_==current->navigation_allowed &&
        published_reason_==current->reason)))return;
    if(current) {
      published_=true;published_epoch_=current->epoch;
      published_allowed_=current->navigation_allowed;published_reason_=current->reason;
    }
    astribot_navigation_msgs::msg::RobotEnvelope old;
    if(current) {
      publisher_->publish(*current);
      if(!current->navigation_allowed)for(const auto& pub:footprints_)pub->publish(current->installed_footprint);
      old=current->limits;
    } else {
      const auto& baseline=core_->baseline();old.epoch=core_->epoch();old.frame_id=baseline.at("base_frame").get<std::string>();
      old.posture_id="fixed_v2_hold";old.lease_s=.3;
#define COPY(field) old.field=baseline.at(#field).get<double>()
      COPY(half_length_m);COPY(half_width_m);COPY(height_m);COPY(payload_mass_kg);
      COPY(max_speed_m_s);COPY(max_angular_speed_rad_s);COPY(max_acceleration_m_s2);COPY(brake_deceleration_m_s2);
#undef COPY
    }
    old.transport_ready=false;old.reason="FIXED_V2_NO_LEGACY_MOTION";old.stamp=now();legacy_->publish(old);
  }
  bool published_{false},published_allowed_{false};
  std::uint64_t published_epoch_{0};
  std::string published_reason_;
  std::unique_ptr<FixedEnvelopeCore> core_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_message_;
  rclcpp::Publisher<FixedEnvelopeCore::Envelope>::SharedPtr publisher_;
  rclcpp::Publisher<astribot_navigation_msgs::msg::RobotEnvelope>::SharedPtr legacy_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr> footprints_;
  rclcpp::Subscription<FixedEnvelopeCore::State>::SharedPtr state_;
  rclcpp::Subscription<FixedEnvelopeCore::Hold>::SharedPtr hold_;
  rclcpp::Subscription<FixedEnvelopeCore::Ack>::SharedPtr applied_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Service<astribot_navigation_msgs::srv::SetFixedEnvelope>::SharedPtr propose_;
  rclcpp::Service<astribot_navigation_msgs::srv::SetRobotEnvelope>::SharedPtr legacy_request_;
  rclcpp::Service<astribot_navigation_msgs::srv::ReserveArmMotion>::SharedPtr reserve_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);int status=0;
  try {rclcpp::spin(std::make_shared<astribot::navigation::FixedEnvelopeCoordinatorNode>());}
  catch(const std::exception& e) {RCLCPP_FATAL(rclcpp::get_logger("robot_envelope_coordinator"),"%s",e.what());status=1;}
  rclcpp::shutdown();return status;
}
