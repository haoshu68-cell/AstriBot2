#pragma once
#include <memory>
#include "nav2_core/controller.hpp"

namespace astribot_s1_path_tracking {
// Independent station-pose servo. It has no path-tracking or Arrival state.
class WorkstationAlignmentController : public nav2_core::Controller {
public:
  WorkstationAlignmentController();
  ~WorkstationAlignmentController() override;
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr&,std::string,
    std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path&) override;
  void setSpeedLimit(const double&,const bool&) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped&,
    const geometry_msgs::msg::Twist&,nav2_core::GoalChecker*) override;
private:
  struct State;std::unique_ptr<State> state_;
  friend class WorkstationAlignmentControllerTestPeer;
};
} // namespace astribot_s1_path_tracking
