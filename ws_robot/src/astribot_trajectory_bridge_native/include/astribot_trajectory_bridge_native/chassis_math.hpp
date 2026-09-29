#pragma once

#include <array>
#include <cstddef>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace astribot_trajectory_bridge_native {

using Vec3 = std::array<double, 3>;
using Vec2 = std::array<double, 2>;

constexpr int kX = 0;
constexpr int kY = 1;
constexpr int kTheta = 2;

double wrap_angle(double theta);
Vec2 rotate_vec2_transposed(double yaw, const Vec2 &value);
Vec3 to_local_velocity(const Vec3 &twist, const std::string &input_frame, double theta);
Vec2 local_pose_displacement(const Vec3 &previous, const Vec3 &current);
Vec3 integrate_step_dt(const Vec3 &pos_cmd, const Vec3 &local_velocity, double dt);
Vec3 integrate_step(const Vec3 &pos_cmd, const Vec3 &local_velocity, double frequency);
Vec3 pose_error(const Vec3 &pos_a, const Vec3 &pos_b);
Vec2 error_magnitude(const Vec3 &error);
Vec2 leash_error(const Vec3 &pos_cmd, const Vec3 &sdk_actual);
double pose_jump_distance(const Vec3 &pose_now, const Vec3 &pose_prev);
double odom_drift(const Vec2 &disp_sdk_xy, const Vec2 &disp_slam_xy);

struct TickDtResult {
  double dt;
  bool clamped;
  std::string reason;
  double raw;
  bool has_raw;
};

TickDtResult measure_tick_dt(double now, const std::optional<double> &previous,
                             double nominal_dt, double max_dt);

class PoseFrameIntegrator {
public:
  PoseFrameIntegrator(const Vec3 &pose, double stamp, const Vec3 &sdk_pose);
  bool observe(const Vec3 &pose, double stamp);
  void reanchor_axis(int axis, double actual);
  Vec3 target(const Vec3 &velocity, double dt) const;
  void commit(const Vec3 &velocity, double dt);
  Vec3 preview_target(const Vec3 &velocity, double dt, const Vec3 &times,
                      double xy_limit, double theta_limit) const;
  void commit_preview(const Vec3 &velocity, double dt);
  double stamp() const { return stamp_; }
  Vec3 pose() const { return pose_; }
  Vec3 anchor() const { return anchor_; }
  Vec3 integral() const { return integral_; }
  Vec3 velocity() const { return velocity_; }
  void set_integral(int axis, double value);

private:
  double stamp_;
  Vec3 pose_;
  Vec3 anchor_;
  Vec3 integral_;
  Vec3 velocity_;
};

class SdkPoseHistory {
public:
  explicit SdkPoseHistory(double duration);
  void append(double stamp, const Vec3 &pose);
  Vec3 at(double stamp) const;

private:
  double duration_;
  std::deque<std::pair<double, Vec3>> samples_;
};

struct OdomSample {
  double x;
  double y;
  double theta;
  double vx_body;
  double vy_body;
  double wz;
  double jump_m;
  bool jumped;
};

struct OdomStats {
  std::size_t samples;
  std::size_t jumps;
  double max_jump_m;
  double travelled_m;
  std::vector<double> jump_history;
};

class ChassisOdomSource {
public:
  ChassisOdomSource(double jump_threshold_m, const std::string &velocity_frame);
  OdomSample sample(const Vec3 &pos, const Vec3 &vel);
  OdomStats stats() const;
  double jump_ratio() const;

private:
  double jump_threshold_m_;
  bool velocity_body_;
  bool has_previous_pose_;
  Vec3 previous_pose_;
  std::size_t samples_;
  std::size_t jumps_;
  double max_jump_m_;
  double travelled_m_;
  std::vector<double> jump_history_;
};

}  // namespace astribot_trajectory_bridge_native
