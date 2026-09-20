#include "nav2_controller/plugins/pose_progress_checker.hpp"
#include "astribot_s1_path_tracking/policy_lease.hpp"
#include "astribot_s1_path_tracking/arrival_progress.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace astribot_s1_path_tracking {
class PolicyProgressChecker : public nav2_controller::PoseProgressChecker {
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                  const std::string & name) override {
    PoseProgressChecker::initialize(parent,name);
    auto node=parent.lock();
    arrival_=ArrivalProgress::forNode(node.get());
    nav2_util::declare_parameter_if_not_declared(node,"navigation_policy_enabled",rclcpp::ParameterValue(false));
    if (node->get_parameter("navigation_policy_enabled").as_bool()) {
      sub_=node->create_subscription<PolicyLease::Message>("navigation_policy/constraint",10,
        [this](PolicyLease::Message::ConstSharedPtr m) {lease_.receive(*m);});
    }
  }
  bool check(geometry_msgs::msg::PoseStamped & pose) override {
    auto now=clock_->now();
    const bool settling=arrival_->settling(now.seconds());
    bool held=(sub_ && lease_.held(now)) || settling;
    // ArrivalController owns the bounded settling deadline inside goal bounds.
    // Restart pose progress here so leaving those bounds gets its usual allowance.
    if (settling) {PoseProgressChecker::reset();}
    if (last_>=0 && paused_ && now.seconds()>=last_ && baseline_pose_set_) {
      baseline_time_=baseline_time_+rclcpp::Duration::from_seconds(now.seconds()-last_);
    }
    last_=now.seconds();paused_=held;
    return held || PoseProgressChecker::check(pose);
  }
  void reset() override {
    PoseProgressChecker::reset();last_=-1;paused_=false;
    if (arrival_) {arrival_->clear();}
  }
private:
  std::shared_ptr<ArrivalProgress> arrival_;
  PolicyLease lease_;
  rclcpp::Subscription<PolicyLease::Message>::SharedPtr sub_;
  double last_{-1};bool paused_{false};
};
}
PLUGINLIB_EXPORT_CLASS(astribot_s1_path_tracking::PolicyProgressChecker,nav2_core::ProgressChecker)
