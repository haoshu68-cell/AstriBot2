#pragma once
#include "astribot_trajectory_bridge_native/chassis_math.hpp"
#include "astribot_trajectory_bridge_native/runtime_ports.hpp"
#include <memory>
#include <optional>
namespace astribot_trajectory_bridge_native {
struct StampedPose {
  std::vector<double> pose;
  double stamp;
};
struct PosePort {
  virtual ~PosePort() = default;
  virtual std::optional<StampedPose> lookup() = 0;
};
struct ChassisConfig {
  double outer_rate = 10.0;
  std::string pose_source = "slam";
  std::string part_name = "astribot_chassis";
  double freq = 250.0;
  std::string input_frame = "body";
  std::string theta_reference = "at_enable";
  double cmd_vel_timeout_sec = 0.3;
  double leash_xy_m = 0.25;
  double leash_theta_rad = 0.35;
  bool require_slam_to_enable = false;
  double slam_max_age_sec = 0.5;
  double slam_loss_grace_sec = 2.0;
  double slam_jump_threshold_m = 0.3;
  double odom_drift_window_sec = 2.0;
  double odom_drift_warn_m = 0.15;
  double max_tick_dt_sec = 0.04;
  bool require_fresh_scan = true;
  double scan_max_age_sec = 0.5;
  double scan_loss_grace_sec = 2.0;
  double pose_preview_xy_sec = 0.5;
  double pose_preview_theta_sec = 0.5;
  double pose_preview_max_xy_m = 0.2;
  double pose_preview_max_theta_rad = 0.34;
};
struct ChassisEvent {
  std::string code, detail;
  double metric_1, metric_2;
};
struct ChassisStop {
  std::string state, reason;
  double metric_1, metric_2, at;
};
struct IntegratorState {
  double stamp;
  Vec3 pose, anchor, integral, velocity;
};
struct TickStats {
  int count;
  double mean_dt, rate_hz;
  int clamp_count;
  double clamp_ratio;
  bool live;
  double max_dt;
};
struct TickWindowGap {
  double max_dt;
  std::optional<double> at;
  int over_count, ticks;
};
struct VelTrace {
  int ticks;
  double wall;
  double in_peak;
  double local_peak;
  double in_wz_peak;
  double local_wz_peak;
  int zeroed_ticks;
  double cmd_path;
  double cmd_net;
  double act_net;
  double dtheta_cmd;
  double dtheta_act;
  double corr_path;
  double dtheta_integrated;
  int pose_rebases;
  double frame_dx;
  double frame_dy;
  double frame_dtheta;
  double lead_xy_peak;
  double lead_theta_peak;
};
// The host must serialize callbacks for a core, as the existing bridge facade
// does with its control lock. No method owns a device session or control
// rights.
class ChassisBridgeCore {
public:
  ChassisBridgeCore(const ChassisConfig &, JointSessionPort &, PosePort &,
                    ClockPort &);
  ~ChassisBridgeCore();
  const std::string &state() const;
  std::optional<Vec3> pos_cmd() const;
  std::optional<ChassisStop> last_stop() const;
  std::optional<IntegratorState> integrator_state() const;
  void submit_twist(double vx, double vy, double wz);
  void submit_scan_seen(std::optional<double> stamp = std::nullopt);
  std::pair<bool, std::string> enable();
  std::pair<bool, std::string> disable();
  std::pair<bool, std::string> reset_leash();
  bool inner_tick();
  void outer_tick();
  std::vector<ChassisEvent> drain_events();
  TickStats tick_stats() const;
  TickWindowGap consume_tick_window_gap();
  void reset_tick_stats();
  VelTrace consume_vel_trace();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace astribot_trajectory_bridge_native
