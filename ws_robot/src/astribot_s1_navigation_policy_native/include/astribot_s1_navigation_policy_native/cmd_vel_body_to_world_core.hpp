#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace astribot::navigation {

struct BodyToWorldConfig {
  bool enable_body_to_world{false};
  bool enable_posture_monitor{true};
  double normal_height{0.134};
  double max_height_deviation{0.06};
  double max_tilt_rad{0.12};
  double odom_timeout_s{0.5};
  double cmd_timeout_s{0.5};
};

struct OdomStatus {
  bool publish_zero{false};
  bool tripped{false};
  std::string reason;
};

struct CommandStatus {
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
  bool publish{true};
};

class CmdVelBodyToWorldCore {
public:
  explicit CmdVelBodyToWorldCore(BodyToWorldConfig config);

  static void validate_config(const BodyToWorldConfig& config);
  // Reconfigure without discarding odometry, command age, the sample window or a stop latch.
  void update_config(const BodyToWorldConfig& config);

  OdomStatus odom(double qx, double qy, double qz, double qw, double z,
                  double stamp_s, double ros_now_s, double steady_now_s);
  CommandStatus command(double linear_x, double linear_y, double linear_z,
                        double angular_x, double angular_y, double angular_z,
                        double steady_now_s, double ros_now_s);
  bool watchdog(double steady_now_s, double ros_now_s);
  bool ready(double ros_now_s, double steady_now_s) const;
  double yaw() const { return current_yaw_; }
  bool safety_tripped() const { return safety_tripped_; }

private:
  BodyToWorldConfig config_;
  bool odom_valid_{false};
  bool has_odom_{false};
  bool has_cmd_{false};
  double last_odom_received_s_{0.0};
  double last_odom_stamp_s_{0.0};
  double last_cmd_received_s_{0.0};
  double current_yaw_{0.0};
  bool safety_tripped_{false};
  std::vector<std::array<double, 3>> attitude_samples_;
};

}  // namespace astribot::navigation
