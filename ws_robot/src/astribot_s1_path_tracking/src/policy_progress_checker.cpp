#include "nav2_controller/plugins/pose_progress_checker.hpp"
#include "astribot_s1_path_tracking/policy_lease.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace astribot_s1_path_tracking {
class PolicyProgressChecker : public nav2_controller::PoseProgressChecker {
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                  const std::string & name) override {
    PoseProgressChecker::initialize(parent,name);
    auto node=parent.lock();
    sub_=node->create_subscription<PolicyLease::Message>("navigation_policy/constraint",10,
      [this](PolicyLease::Message::ConstSharedPtr m) {lease_.receive(*m);});
  }
  bool check(geometry_msgs::msg::PoseStamped & pose) override {
    auto now=clock_->now();
    bool held=lease_.held(now);
    if (last_>=0 && paused_ && now.seconds()>=last_ && baseline_pose_set_) {
      baseline_time_=baseline_time_+rclcpp::Duration::from_seconds(now.seconds()-last_);
    }
    last_=now.seconds();paused_=held;
    return held || PoseProgressChecker::check(pose);
  }
  void reset() override {PoseProgressChecker::reset();last_=-1;paused_=false;}
private:
  PolicyLease lease_;
  rclcpp::Subscription<PolicyLease::Message>::SharedPtr sub_;
  double last_{-1};bool paused_{false};
};
}
PLUGINLIB_EXPORT_CLASS(astribot_s1_path_tracking::PolicyProgressChecker,nav2_core::ProgressChecker)
