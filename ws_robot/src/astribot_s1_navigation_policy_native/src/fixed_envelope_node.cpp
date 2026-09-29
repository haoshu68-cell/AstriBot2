#include "astribot_s1_navigation_policy_native/fixed_envelope_core.hpp"
#include "astribot_s1_path_tracking/arrival_settling.hpp"
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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <astribot_navigation_msgs/srv/reserve_arm_motion.hpp>
#include <astribot_navigation_msgs/srv/set_robot_envelope.hpp>
#include <astribot_s1_payload_state/consumer.hpp>

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
    rcl_interfaces::msg::ParameterDescriptor identity;identity.read_only=true;
    astribot::payload::Config payload_config;
    payload_config.environment=declare_parameter<std::string>("payload_environment","simulation",identity);
    payload_config.session=declare_parameter<std::string>("payload_session_id","",identity);
    payload_config.source=declare_parameter<std::string>("payload_source_id","",identity);
    payload_config.allowed_links={"astribot_arm_left_tcp_link","astribot_arm_right_tcp_link"};
    if(payload_config.session.empty()!=payload_config.source.empty())throw std::invalid_argument("PAYLOAD_IDENTITY_INCOMPLETE");
    if(!payload_config.source.empty()) {
      if(payload_config.environment!="simulation")throw std::invalid_argument("FIXED_V2_PAYLOAD_SIMULATION_REQUIRED");
      payload_consumer_=std::make_unique<astribot::payload::Consumer>(payload_config);
      core_->payload_source([this](int64_t at){return payload_consumer_->current(at,steady_now());});
      payload_=create_subscription<astribot::payload::State>("/payload/attachment_state",10,
        [this](astribot::payload::State::ConstSharedPtr msg){payload_consumer_->receive(*msg,now().nanoseconds(),steady_now());tick(false);});
      // Consumer lease expiration must also be evaluated while ROS time freezes.
      payload_watchdog_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick(false);});
    }
    publisher_=create_publisher<FixedEnvelopeCore::Envelope>("/navigation/envelope_v2",10);
    legacy_=create_publisher<astribot_navigation_msgs::msg::RobotEnvelope>("/navigation/robot_envelope",10);
    for(const auto* name:{"global_costmap","local_costmap"})
      footprints_.push_back(create_publisher<geometry_msgs::msg::Polygon>(std::string("/")+name+"/footprint",1));
    state_=create_subscription<FixedEnvelopeCore::State>("/navigation/geometry_state",10,
      [this](FixedEnvelopeCore::State::ConstSharedPtr msg){core_->state(*msg);tick();});
    hold_observed_=create_publisher<FixedEnvelopeCore::Hold>("/navigation/arm_hold_observed",10);
    hold_=create_subscription<FixedEnvelopeCore::Hold>("/navigation/arm_hold",10,
      [this](FixedEnvelopeCore::Hold::ConstSharedPtr msg){
        core_->hold(*msg);tick();
        // Confirms receipt only; the existing proposal and five ACKs grant navigation.
        hold_observed_->publish(*msg);
      });
    applied_=create_subscription<FixedEnvelopeCore::Ack>("/navigation/envelope_applied",20,
      [this](FixedEnvelopeCore::Ack::ConstSharedPtr msg){core_->acknowledge(*msg,now().nanoseconds());tick(false);});
    slam_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose",rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg){
        if(msg->header.stamp.sec<0 || msg->header.stamp.nanosec>=1000000000u) {stopped_=false;xy_stop_.reset();yaw_stop_.reset();return;}
        if(slam_message_ && fixed_stamp_ns(msg->header.stamp)<=fixed_stamp_ns(slam_message_->header.stamp))return;
        slam_message_=msg;
        const auto valid=[](const auto &m) {
          const auto &p=m.pose.pose.position;const auto &q=m.pose.pose.orientation;
          const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
          return m.header.frame_id=="map" && std::isfinite(p.x) && std::isfinite(p.y) &&
            std::isfinite(p.z) && std::isfinite(norm) && std::abs(norm-1.)<.01;
        };
        stopped_=false;
        if(!valid(*msg)) {xy_stop_.reset();yaw_stop_.reset();return;}
        const auto &p=msg->pose.pose.position;const auto &q=msg->pose.pose.orientation;
        const double yaw=std::atan2(2.*(q.w*q.z+q.x*q.y),1.-2.*(q.y*q.y+q.z*q.z));
        const double received=steady_now()*1e-9,source=fixed_stamp_ns(msg->header.stamp)*1e-9;
        constexpr double stop_window=.6;
        xy_stop_.command(true,received);yaw_stop_.command(true,received);
        xy_stop_.observe(source,received,{p.x,p.y},stop_window,.02,.02*stop_window);
        yaw_stop_.observe(source,received,{yaw},stop_window,.03,.03*stop_window,true);
        stopped_=xy_stop_.evidence().stopped&&yaw_stop_.evidence().stopped;
      });
    propose_=create_service<astribot_navigation_msgs::srv::SetFixedEnvelope>("/navigation/set_fixed_envelope",
      [this](const std::shared_ptr<FixedEnvelopeCore::Request> request,
             std::shared_ptr<astribot_navigation_msgs::srv::SetFixedEnvelope::Response> response) {
        try {
          const auto at=now().nanoseconds();
          (void)fixed_stamp(at);
          const auto& e=core_->propose(*request,at,stopped_);
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
  static int64_t steady_now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
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
  std::unique_ptr<astribot::payload::Consumer> payload_consumer_;
  rclcpp::Subscription<astribot::payload::State>::SharedPtr payload_;
  rclcpp::TimerBase::SharedPtr payload_watchdog_;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr slam_message_;
  bool stopped_{false};
  astribot_s1_path_tracking::ArrivalSettling<2> xy_stop_;
  astribot_s1_path_tracking::ArrivalSettling<1> yaw_stop_;
  rclcpp::Publisher<FixedEnvelopeCore::Envelope>::SharedPtr publisher_;
  rclcpp::Publisher<astribot_navigation_msgs::msg::RobotEnvelope>::SharedPtr legacy_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr> footprints_;
  rclcpp::Subscription<FixedEnvelopeCore::State>::SharedPtr state_;
  rclcpp::Subscription<FixedEnvelopeCore::Hold>::SharedPtr hold_;
  rclcpp::Publisher<FixedEnvelopeCore::Hold>::SharedPtr hold_observed_;
  rclcpp::Subscription<FixedEnvelopeCore::Ack>::SharedPtr applied_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_;
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
