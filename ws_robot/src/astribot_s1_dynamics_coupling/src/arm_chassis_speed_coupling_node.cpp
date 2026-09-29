#include "astribot_s1_dynamics_coupling/core.hpp"
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace astribot_s1_dynamics_coupling {
class ArmChassisSpeedCouplingNode final : public rclcpp::Node {
 public:
  ArmChassisSpeedCouplingNode() : Node("arm_chassis_speed_coupling_node") {
    rcl_interfaces::msg::ParameterDescriptor immutable; immutable.read_only=true;
    const auto output=declare_parameter("output_topic", std::string("/navigation_policy/arm_speed_limit"),immutable);
    const auto input=declare_parameter("joint_states_topic", std::string("/joint_states"),immutable);
    base_=declare_parameter("chassis_base_frame", std::string("astribot_torso_base"),immutable);
    metric_=declare_parameter("extension_metric", std::string("horizontal_reach"),immutable);
    links_=declare_parameter("monitored_links", std::vector<std::string>{
      "astribot_arm_left_tcp_link", "astribot_arm_right_tcp_link",
      "astribot_gripper_left_Link_L11", "astribot_gripper_right_Link_R11"},immutable);
    folded_=declare_parameter("reach_folded_m",.42,immutable);
    full_=declare_parameter("reach_full_m",.8865,immutable);
    reach_period_=declare_parameter("reach_update_period_sec",.05,immutable);
    reference_=declare_parameter("folded_reference_rad",std::vector<double>(14,0.),immutable);
    deviation_full_=declare_parameter("extension_full_rad",1.2,immutable);
    velocity_full_=declare_parameter("velocity_full_rad_s",2.,immutable);
    minimum_=declare_parameter("min_speed_scale",.15,immutable);
    alpha_=declare_parameter("scale_smoothing_alpha",.25,immutable);
    if(metric_!="horizontal_reach" && metric_!="joint_deviation")throw std::invalid_argument("invalid extension_metric");
    if(base_.empty() || links_.empty() || reference_.size()!=14 ||
      !std::all_of(reference_.begin(),reference_.end(),[](double v){return std::isfinite(v);}) ||
      !std::isfinite(folded_) || !std::isfinite(full_) ||
      !std::isfinite(reach_period_) || reach_period_<=0. || reach_period_>.05 ||
      !std::isfinite(deviation_full_) || deviation_full_<=0. ||
      !std::isfinite(velocity_full_) || velocity_full_<=0. ||
      !std::isfinite(minimum_) || minimum_<0. || minimum_>1. ||
      !std::isfinite(alpha_) || alpha_<=0. || alpha_>1.)
      throw std::invalid_argument("invalid arm navigation limit parameters");
    validate_reach_thresholds(folded_,full_);
    for(const auto *side:{"left","right"})for(int i=1;i<=7;++i)
      joints_.push_back(std::string("astribot_arm_")+side+"_joint_"+std::to_string(i));
    if(metric_=="horizontal_reach") {
      tf_=std::make_unique<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME));
      listener_=std::make_unique<tf2_ros::TransformListener>(*tf_,this,false);
    }
    publisher_=create_publisher<nav2_msgs::msg::SpeedLimit>(output,10);
    joint_sub_=create_subscription<sensor_msgs::msg::JointState>(input,10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg){joint(*msg);});
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{publish();});
    RCLCPP_INFO(get_logger(),"Upstream arm speed percentage: %s -> %s; 0 means HOLD on this dedicated constraint topic",input.c_str(),output.c_str());
  }
 private:
  using Steady=std::chrono::steady_clock;
  static double steady_seconds() {
    return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();
  }
  void invalidate(const char *reason) {
    valid_=false;scale_=0.;
    RCLCPP_WARN_THROTTLE(get_logger(),warning_clock_,2000,"Arm navigation HOLD: %s",reason);
  }
  std::optional<double> reach() {
    const double now=steady_seconds();
    if(reach_at_>=0. && now>=reach_at_ && now-reach_at_<reach_period_)return reach_;
    reach_at_=now;reach_.reset();
    for(const auto &link:links_) {
      try {
        const auto transform=tf_->lookupTransform(base_,link,tf2::TimePointZero);
        const auto &p=transform.transform.translation;
        if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
          reach_.reset();return reach_;
        }
        const double value=horizontal_reach(p.x,p.y);
        reach_=reach_?std::max(*reach_,value):value;
      } catch(const tf2::TransformException &) {reach_.reset();return reach_;}
    }
    return reach_;
  }
  void joint(const sensor_msgs::msg::JointState &msg) {
    if(msg.header.stamp.sec<0 || msg.header.stamp.nanosec>=1000000000u) {
      invalidate("INVALID_SOURCE_STAMP");publish();return;
    }
    const int64_t source=static_cast<int64_t>(msg.header.stamp.sec)*1000000000LL+msg.header.stamp.nanosec;
    // Source ordering is independent of the local ROS clock. Duplicate/older
    // samples cannot replace the latest measured state, including an invalid one.
    if(source<=source_ns_)return;
    source_ns_=source;stamp_=msg.header.stamp;
    if(!complete_joint_sample(msg.name,msg.position,msg.velocity,joints_)) {
      invalidate("INVALID_JOINT_SAMPLE");publish();return;
    }
    try {
      const auto positions=zip_map(msg.name,msg.position),velocities=zip_map(msg.name,msg.velocity);
      bool missing_tf=false;double extension=0.;
      if(metric_=="horizontal_reach") {
        const auto value=reach();missing_tf=!value;
        extension=value?reach_activity(*value,folded_,full_):1.;
      } else extension=joint_deviation_activity(positions,joints_,reference_,deviation_full_);
      const double activity=std::max(extension,velocity_activity(velocities,joints_,velocity_full_));
      const double raw=scale_from_activity(activity,minimum_);
      // Missing reach TF keeps the old full-reach fallback, immediately at the
      // conservative minimum instead of smoothing a previously larger cap.
      scale_=missing_tf?minimum_:alpha_*raw+(1.-alpha_)*scale_;
      if(missing_tf)RCLCPP_WARN_THROTTLE(get_logger(),warning_clock_,2000,"Arm reach TF unavailable; using full-reach minimum speed percentage");
      valid_=true;
    } catch(const std::exception &error) {
      invalidate(error.what());
    }
    publish();
  }
  void publish() {
    nav2_msgs::msg::SpeedLimit message;
    // Re-publication never gives old joint evidence a new capture time.
    message.header.stamp=stamp_;message.header.frame_id=base_;
    message.percentage=true;message.speed_limit=valid_?100.*scale_:0.;
    publisher_->publish(message);
  }
  std::vector<std::string> joints_,links_;std::vector<double> reference_;
  std::string base_,metric_;double folded_{},full_{},reach_period_{};
  double deviation_full_{},velocity_full_{},minimum_{},alpha_{};
  double scale_{1.},reach_at_{-1.};bool valid_{false};
  int64_t source_ns_{-1};builtin_interfaces::msg::Time stamp_;
  std::optional<double> reach_;
  rclcpp::Clock warning_clock_{RCL_STEADY_TIME};
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace astribot_s1_dynamics_coupling
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);int result=0;
  try {rclcpp::spin(std::make_shared<astribot_s1_dynamics_coupling::ArmChassisSpeedCouplingNode>());}
  catch(const std::exception &error){RCLCPP_FATAL(rclcpp::get_logger("arm_chassis_speed_coupling_node"),"%s",error.what());result=1;}
  rclcpp::shutdown();return result;
}
