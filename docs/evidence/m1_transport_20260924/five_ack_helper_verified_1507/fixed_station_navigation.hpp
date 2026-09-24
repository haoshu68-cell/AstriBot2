#pragma once
#include <astribot_navigation_msgs/msg/robot_geometry_state.hpp>
#include <astribot_navigation_msgs/srv/set_fixed_envelope.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace astribot::transport {
// One navigation leg of the owning PICK -> NAV -> PLACE task. The parent keeps
// its existing lease, validates its live grant/Hold/geometry before every tick,
// and calls cancel first on failure. This helper never acquires or renews it.
// All methods and ROS callbacks use the node's mutually exclusive callback
// group. Destroy only after the executor stops or all requests are resolved.
class FixedStationNavigation {
public:
  using Fixed=astribot_navigation_msgs::srv::SetFixedEnvelope;
  struct Request {
    std::string task_id,context_id,lease_id,resource_epoch;
    Fixed::Request fixed; // From the parent's ArmHold::request().
    astribot_navigation_msgs::msg::RobotGeometryState reference_geometry;
    geometry_msgs::msg::PoseStamped navigation_target; // map, actual station binding
    std::string robot_base_frame; // Verified Nav2 robot_base_frame == odom.child_frame_id.
    double position_tolerance_m=0,yaw_tolerance_rad=0; // Actual station/profile limits.
  };
  enum class Phase {IDLE,WAIT_FIXED,WAIT_ACK,NAVIGATING,SETTLING,SUCCEEDED,CANCELING,FAILED,UNRESOLVED};
  struct Status {
    Phase phase=Phase::IDLE;
    std::string reason="IDLE",cleanup_reason,endpoint,nav_goal_uuid,envelope_session,geometry_hash;
    uint64_t envelope_epoch=0;
    bool fixed_request_terminal=true,nav_terminal=true,measured_stopped=false,cleanup_complete=false;
    double position_error_m=0,yaw_error_rad=0;
  };
  FixedStationNavigation(rclcpp::Node&,std::string navigation_action);
  ~FixedStationNavigation();
  FixedStationNavigation(const FixedStationNavigation&)=delete;
  FixedStationNavigation& operator=(const FixedStationNavigation&)=delete;
  void start(Request);
  void tick();
  void cancel(const std::string& reason);
  const Status& status()const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace astribot::transport
