#pragma once
#include <astribot_navigation_msgs/msg/robot_geometry_state.hpp>
#include <astribot_navigation_msgs/msg/arm_hold_status.hpp>
#include <astribot_navigation_msgs/srv/set_fixed_envelope.hpp>
#include <controller_manager_msgs/msg/controller_state.hpp>
#include <deque>
#include <optional>
#include <set>
#include <string>
namespace astribot::transport {
using Geometry=astribot_navigation_msgs::msg::RobotGeometryState;
using Hold=astribot_navigation_msgs::msg::ArmHoldStatus;
struct ResourceGrant {
  std::string owner_id,lease_id,epoch;
  std::set<std::string> joints;
  int64_t issued_at=0,valid_until=0,received_steady=0,received_at=0;
};
// Produced by the owning executor after a task-owned hold/posture action terminates.
// An active controller or zero velocity alone is not this proof.
struct HoldCompletion {
  std::string owner_id,lease_id,resource_epoch,action_id;
  int64_t completed_at=0;
  bool successful=false,terminal=false;
};
class ArmHold {
public:
  void begin(const std::string &hold_id,const ResourceGrant &,const HoldCompletion &,int64_t now,int64_t steady);
  bool resource(const ResourceGrant &,int64_t now,int64_t steady);
  bool geometry(const Geometry &,int64_t now,int64_t steady);
  void controllers(const std::vector<controller_manager_msgs::msg::ControllerState> &,int64_t request_at,int64_t request_steady,int64_t now,int64_t steady);
  void cancel();
  Hold status(int64_t now,int64_t steady);
  astribot_navigation_msgs::srv::SetFixedEnvelope::Request request(const std::string &request_id,
    const astribot_navigation_msgs::msg::RobotEnvelope &limits,int64_t now,int64_t steady);
  const std::string &reason() const {return reason_;}
private:
  void revoke(const std::string &reason);
  bool clocks(int64_t now,int64_t steady);
  struct Sample {int64_t stamp,steady;std::vector<double> positions;};
  std::optional<ResourceGrant> grant_;std::optional<Geometry> reference_,latest_;
  std::set<std::string> used_ids_;
  std::deque<Sample> samples_;
  std::string id_,reason_="NO_HOLD";
  int64_t after_=0,last_ros_=-1,last_steady_=-1,geometry_deadline_=-1,claims_ros_=-1,claims_steady_=-1;
  std::set<std::string> claims_;
  bool active_=false,confirmed_once_=false;
};
}
