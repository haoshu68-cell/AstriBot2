#pragma once
#include "astribot_trajectory_bridge_native/arm_math.hpp"
#include "astribot_trajectory_bridge_native/runtime_ports.hpp"
#include <memory>
#include <optional>
namespace astribot_trajectory_bridge_native {
struct ArmSessionPort : JointSessionPort {
  virtual std::pair<JointPositions, JointPositions>
  get_joints_position_limit(const Names &) = 0;
  virtual void move_joints_waypoints(const Names &,
                                     const std::vector<JointPositions> &,
                                     const std::vector<double> &, bool use_wbc,
                                     bool add_default_torso) = 0;
};
struct ArmConfig {
  std::string part_name = "astribot_arm_left";
  std::vector<std::string> joint_names = {};
  double stream_freq = 250.0;
  std::string control_way = "direct";
  bool use_wbc = false;
  bool add_default_torso = false;
  std::string interp = "cubic";
  double max_traj_duration_sec = 60.0;
  double limit_margin_rad = 0.0;
  bool cross_check_urdf = true;
  double limit_cross_check_tol_rad = 0.01;
  bool strict_limit_check = false;
  double max_tracking_error_rad = 0.1;
  bool abort_on_tracking_error = true;
  double settle_tolerance_rad = 0.02;
  double settle_timeout_sec = 2.0;
  double hold_still_epsilon_rad = 0.001;
  int hold_still_ticks_required = 25;
  double hold_timeout_sec = 2.0;
  bool allow_recovery_from_oob = true;
  double oob_start_tolerance_rad = 0.05;
};
struct ArmEvent {
  std::string code, detail;
  double metric_1 = 0, metric_2 = 0;
};
struct ArmFeedback {
  double t;
  std::vector<double> desired, actual;
  double error;
};
struct ArmStartResult {
  bool ok;
  int code;
  std::string detail;
};
struct WaypointResult {
  bool ok;
  std::string code, detail;
  std::size_t sent, dropped;
};
// The host serializes start/step/cancel and keeps the existing action goal
// lock.
class ArmTrajExecutor {
public:
  ArmTrajExecutor(const ArmConfig &, ArmSessionPort &, ClockPort &);
  ~ArmTrajExecutor();
  std::pair<bool, std::string>
  load_limits(std::optional<std::vector<double>> lower = std::nullopt,
              std::optional<std::vector<double>> upper = std::nullopt);
  ArmStartResult
  start(const Names &, const std::vector<double> &, const JointTrajectory &,
        std::optional<JointTrajectory> velocities = std::nullopt);
  void request_cancel();
  const std::string &step();
  const std::string &phase() const;
  int error_code() const;
  const std::string &detail() const;
  std::vector<double> lower() const;
  std::vector<double> upper() const;
  std::vector<ArmEvent> events() const;
  std::vector<ArmFeedback> feedbacks() const;
  std::vector<ArmEvent> drain_events();
  std::vector<ArmFeedback> drain_feedbacks();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
class WaypointDispatcher {
public:
  WaypointDispatcher(const ArmConfig &, ArmSessionPort &, bool enabled);
  ~WaypointDispatcher();
  WaypointResult
  dispatch(const JointTrajectory &, const std::vector<double> &,
           std::optional<std::vector<double>> lower = std::nullopt,
           std::optional<std::vector<double>> upper = std::nullopt);
  std::vector<ArmEvent> events() const;
  std::vector<ArmEvent> drain_events();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace astribot_trajectory_bridge_native
