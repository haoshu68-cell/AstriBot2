#include "astribot_s1_path_tracking/slip_monitor.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace astribot_s1_path_tracking
{
void SlipMonitor::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,const std::string & name)
{
  clock_=node->get_clock();logger_=node->get_logger();
  const std::string prefix=name+".slip.";
  auto string_parameter=[&](const std::string & key,const std::string & value) {
      nav2_util::declare_parameter_if_not_declared(node,prefix+key,rclcpp::ParameterValue(value));
      return node->get_parameter(prefix+key).as_string();
    };
  mode_=string_parameter("mode","off");
  if (mode_!="off" && mode_!="monitor" && mode_!="compensate") {
    throw std::invalid_argument("slip.mode must be off, monitor or compensate");
  }
  LinearSlip::Config c;
  auto number=[&](const std::string & key,double & value) {
      nav2_util::declare_parameter_if_not_declared(node,prefix+key,rclcpp::ParameterValue(value));
      value=node->get_parameter(prefix+key).as_double();
    };
  number("window_s",c.window);number("confirm_s",c.confirm);number("max_age_s",c.max_age);
  number("min_speed",c.min_speed);number("max_speed",c.max_speed);number("max_angular",c.max_angular);
  number("max_curvature",c.max_curvature);
  number("min_command_distance",c.min_distance);number("low_ratio",c.low_ratio);
  number("recovery_ratio",c.recovery_ratio);number("max_extra_speed",c.max_extra);
  number("max_extra_ratio",c.max_extra_ratio);number("ramp_acceleration",c.ramp);
  number("attempt_s",c.duration);detector_.configure(c);
  if (!enabled()) {return;}
  const auto slam_topic=string_parameter("slam_odom_topic","/odom");
  const auto sdk_topic=string_parameter("manufacturer_odom_topic","/astribot/chassis/odom_from_sdk");
  const auto command_topic=string_parameter("executed_command_topic","/cmd_vel");
  if (slam_topic.empty() || sdk_topic.empty() || command_topic.empty() || slam_topic==sdk_topic) {
    throw std::invalid_argument("Slip inputs require distinct localization and manufacturer odometry");
  }
  slam_sub_=node->create_subscription<nav_msgs::msg::Odometry>(slam_topic,rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {pose(*msg,true);});
  sdk_sub_=node->create_subscription<nav_msgs::msg::Odometry>(sdk_topic,rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {pose(*msg,false);});
  command_sub_=node->create_subscription<geometry_msgs::msg::Twist>(command_topic,rclcpp::SensorDataQoS(),
    [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      detector_.command({clock_->now().seconds(),msg->linear.x,msg->linear.y,msg->angular.z});
    });
  RCLCPP_INFO(logger_,"SLIP_CONFIG mode=%s slam=%s manufacturer=%s command=%s window=%.2fs "
    "confirm=%.2fs extra_max=%.4fm/s ratio_max=%.2f attempt=%.2fs",
    mode_.c_str(),slam_topic.c_str(),sdk_topic.c_str(),command_topic.c_str(),
    c.window,c.confirm,c.max_extra,c.max_extra_ratio,c.duration);
  if (mode_=="compensate") {
    RCLCPP_WARN(logger_,"SLIP_COMPENSATION requires measurement-time SLAM stamps; "
      "publication-time restamping cannot detect delayed pose contents");
  }
}
void SlipMonitor::pose(const nav_msgs::msg::Odometry & msg,bool slam)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & frame=slam ? slam_frame_ : sdk_frame_;
  auto & child=slam ? slam_child_ : sdk_child_;
  const auto & p=msg.pose.pose;
  const auto & q=p.orientation;
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if (msg.header.frame_id.empty() || msg.child_frame_id.empty() || !std::isfinite(norm) ||
    std::abs(norm-1.)>0.01 || (!frame.empty() && frame!=msg.header.frame_id) ||
    (!child.empty() && child!=msg.child_frame_id)) {
    detector_.pause();frame=msg.header.frame_id;child=msg.child_frame_id;return;
  }
  frame=msg.header.frame_id;child=msg.child_frame_id;
  // Both poses must describe the same body axes; their world frames may differ.
  if (!slam_child_.empty() && !sdk_child_.empty() && slam_child_!=sdk_child_) {
    detector_.pause();return;
  }
  const LinearSlip::Pose sample{rclcpp::Time(msg.header.stamp).seconds(),clock_->now().seconds(),
    p.position.x,p.position.y,tf2::getYaw(q)};
  if (slam) {detector_.slam(sample);} else {detector_.sdk(sample);}
}
void SlipMonitor::reset() {std::lock_guard<std::mutex> lock(mutex_);detector_.reset();}
void SlipMonitor::pause() {std::lock_guard<std::mutex> lock(mutex_);detector_.pause();}
void SlipMonitor::cleanup()
{
  slam_sub_.reset();sdk_sub_.reset();command_sub_.reset();reset();
  slam_frame_.clear();sdk_frame_.clear();slam_child_.clear();sdk_child_.clear();mode_="off";
}
bool SlipMonitor::apply(geometry_msgs::msg::Twist & command,double cap,bool eligible)
{
  if (!enabled()) {return false;}
  std::lock_guard<std::mutex> lock(mutex_);
  const double base=std::hypot(command.linear.x,command.linear.y);
  const double extra=detector_.apply(clock_->now().seconds(),command.linear.x,command.linear.y,
    command.angular.z,cap,eligible,mode_=="compensate");
  const auto & e=detector_.evidence();
  const double span=std::max(1e-9,e.end-e.start);
  RCLCPP_INFO_THROTTLE(logger_,*clock_,500,
    "SLIP_METRICS mode=%s state=%s valid=%d start_s=%.6f end_s=%.6f "
    "command_m=%.6f slam_m=%.6f manufacturer_m=%.6f response_ratio=%.4f "
    "command_mean_mps=%.6f slam_mean_mps=%.6f manufacturer_mean_mps=%.6f "
    "base_mps=%.6f proposed_extra_mps=%.6f cap_mps=%.6f",
    mode_.c_str(),e.state.c_str(),e.valid,e.start,e.end,e.command_m,e.slam_m,e.sdk_m,
    e.ratio,e.command_m/span,e.slam_m/span,e.sdk_m/span,base,extra,cap);
  if (extra<=0. || base<=0.) {return false;}
  command.linear.x*=(base+extra)/base;command.linear.y*=(base+extra)/base;
  return true;
}
}  // namespace astribot_s1_path_tracking
