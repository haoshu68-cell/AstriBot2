#include "astribot_trajectory_bridge_native/chassis_math.hpp"

#include <cmath>
#include <algorithm>
#include <cstdio>
#include <iterator>
#include <stdexcept>

namespace astribot_trajectory_bridge_native {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

void require_positive(const char *name, double value) {
  // Match the Python reference's ``value <= 0`` guard: NaN reaches the
  // arithmetic path and produces NaN, while zero/negative values are rejected.
  if (value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
}

}  // namespace

double wrap_angle(double theta) {
  return std::atan2(std::sin(theta), std::cos(theta));
}

Vec2 rotate_vec2_transposed(double yaw, const Vec2 &value) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * value[0] + s * value[1], -s * value[0] + c * value[1]};
}

Vec3 to_local_velocity(const Vec3 &twist, const std::string &input_frame, double theta) {
  if (input_frame == "body") {
    return twist;
  }
  if (input_frame != "world") {
    throw std::invalid_argument("input_frame must be 'body' or 'world'");
  }
  const auto local = rotate_vec2_transposed(theta, {twist[0], twist[1]});
  return {local[0], local[1], twist[2]};
}

Vec2 local_pose_displacement(const Vec3 &previous, const Vec3 &current) {
  const double turn = wrap_angle(current[kTheta] - previous[kTheta]);
  const double mid_yaw = previous[kTheta] + 0.5 * turn;
  const auto local_chord = rotate_vec2_transposed(
      mid_yaw, {current[kX] - previous[kX], current[kY] - previous[kY]});
  const double half = 0.5 * turn;
  const double scale = std::abs(half) > 1e-6 ? half / std::sin(half)
                                             : 1.0 + half * half / 6.0;
  return {local_chord[0] * scale, local_chord[1] * scale};
}

Vec3 integrate_step_dt(const Vec3 &pos_cmd, const Vec3 &local_velocity, double dt) {
  require_positive("dt", dt);
  return {pos_cmd[0] + local_velocity[0] * dt,
          pos_cmd[1] + local_velocity[1] * dt,
          pos_cmd[2] + local_velocity[2] * dt};
}

Vec3 integrate_step(const Vec3 &pos_cmd, const Vec3 &local_velocity, double frequency) {
  require_positive("frequency", frequency);
  return integrate_step_dt(pos_cmd, local_velocity, 1.0 / frequency);
}

Vec3 pose_error(const Vec3 &pos_a, const Vec3 &pos_b) {
  return {pos_a[0] - pos_b[0], pos_a[1] - pos_b[1],
          wrap_angle(pos_a[2] - pos_b[2])};
}

Vec2 error_magnitude(const Vec3 &error) {
  return {std::hypot(error[0], error[1]), std::abs(error[2])};
}

Vec2 leash_error(const Vec3 &pos_cmd, const Vec3 &sdk_actual) {
  return {std::hypot(pos_cmd[kX] - sdk_actual[kX],
                     pos_cmd[kY] - sdk_actual[kY]),
          std::abs(pos_cmd[kTheta] - sdk_actual[kTheta])};
}

double pose_jump_distance(const Vec3 &pose_now, const Vec3 &pose_prev) {
  return std::hypot(pose_now[kX] - pose_prev[kX],
                    pose_now[kY] - pose_prev[kY]);
}

double odom_drift(const Vec2 &disp_sdk_xy, const Vec2 &disp_slam_xy) {
  return std::abs(std::hypot(disp_sdk_xy[0], disp_sdk_xy[1]) -
                  std::hypot(disp_slam_xy[0], disp_slam_xy[1]));
}

TickDtResult measure_tick_dt(const double now,
                             const std::optional<double> &previous,
                             const double nominal_dt, const double max_dt) {
  if (nominal_dt <= 0.0) {
    throw std::invalid_argument("nominal_dt must be positive");
  }
  if (max_dt <= 0.0) {
    throw std::invalid_argument("max_dt must be positive");
  }
  if (max_dt < nominal_dt) {
    throw std::invalid_argument("max_dt must be >= nominal_dt");
  }
  if (!previous.has_value()) {
    return {nominal_dt, false, "", 0.0, false};
  }

  const double raw = now - *previous;
  if (raw <= 0.0) {
    char reason[192];
    std::snprintf(reason, sizeof(reason),
                  "时钟未前进（raw=%.6fs），退回标称步长", raw);
    return {nominal_dt, true, reason, raw, true};
  }
  if (raw > max_dt) {
    char reason[256];
    std::snprintf(reason, sizeof(reason),
                  "实测步长 %.4fs 超过上限 %.4fs，已钳位。"
                  "未钳位的话本拍会积出一次位置阶跃",
                  raw, max_dt);
    return {max_dt, true, reason, raw, true};
  }
  return {raw, false, "", raw, true};
}

PoseFrameIntegrator::PoseFrameIntegrator(const Vec3 &pose, double stamp,
                                         const Vec3 &sdk_pose)
    : stamp_(stamp), pose_(pose), anchor_(sdk_pose), integral_{0.0, 0.0, 0.0},
      velocity_{0.0, 0.0, 0.0} {}

bool PoseFrameIntegrator::observe(const Vec3 &pose, double stamp) {
  if (stamp <= stamp_) {
    return false;
  }
  const auto displacement = local_pose_displacement(pose_, pose);
  anchor_[kX] += displacement[0];
  anchor_[kY] += displacement[1];
  anchor_[kTheta] += wrap_angle(pose[kTheta] - pose_[kTheta]);
  stamp_ = stamp;
  pose_ = pose;
  integral_ = {0.0, 0.0, 0.0};
  return true;
}

void PoseFrameIntegrator::reanchor_axis(int axis, double actual) {
  if (axis < 0 || axis >= 3) {
    throw std::out_of_range("axis must be 0, 1 or 2");
  }
  anchor_[axis] = actual;
  integral_[axis] = 0.0;
}

Vec3 PoseFrameIntegrator::target(const Vec3 &velocity, double dt) const {
  const Vec3 base{anchor_[0] + integral_[0], anchor_[1] + integral_[1],
                  anchor_[2] + integral_[2]};
  return integrate_step_dt(base, velocity, dt);
}

void PoseFrameIntegrator::commit(const Vec3 &velocity, double dt) {
  require_positive("dt", dt);
  for (int i = 0; i < 3; ++i) {
    integral_[i] += velocity[i] * dt;
  }
}

Vec3 PoseFrameIntegrator::preview_target(const Vec3 &velocity, double dt,
                                         const Vec3 &times, double xy_limit,
                                         double theta_limit) const {
  require_positive("dt", dt);
  Vec3 delta{};
  for (int i = 0; i < 3; ++i) {
    const double old = integral_[i];
    const double v = velocity[i];
    const double previous = velocity_[i];
    const double h = times[i];
    delta[i] = (v != 0.0 ? ((old * previous > 0.0 ? old : 0.0) + v * dt + v * h)
                         : 0.0);
  }
  const double xy = std::hypot(delta[0], delta[1]);
  if (xy > xy_limit) {
    delta[0] *= xy_limit / xy;
    delta[1] *= xy_limit / xy;
  }
  delta[2] = std::max(-theta_limit, std::min(theta_limit, delta[2]));
  return {anchor_[0] + delta[0], anchor_[1] + delta[1], anchor_[2] + delta[2]};
}

void PoseFrameIntegrator::commit_preview(const Vec3 &velocity, double dt) {
  require_positive("dt", dt);
  for (int i = 0; i < 3; ++i) {
    integral_[i] = (velocity[i] * velocity_[i] > 0.0 ? integral_[i] : 0.0) +
                   velocity[i] * dt;
  }
  velocity_ = velocity;
}

void PoseFrameIntegrator::set_integral(int axis, double value) {
  if (axis < 0 || axis >= 3) {
    throw std::out_of_range("axis must be 0, 1 or 2");
  }
  integral_[axis] = value;
}

SdkPoseHistory::SdkPoseHistory(double duration) : duration_(duration) {}

void SdkPoseHistory::append(double stamp, const Vec3 &pose) {
  if (!samples_.empty() && stamp < samples_.back().first) {
    samples_.clear();
  }
  if (!samples_.empty() && stamp == samples_.back().first) {
    samples_.pop_back();
  }
  samples_.emplace_back(stamp, pose);
  while (samples_.size() > 2 && samples_[1].first < stamp - duration_) {
    samples_.pop_front();
  }
}

Vec3 SdkPoseHistory::at(double stamp) const {
  if (samples_.empty()) {
    throw std::invalid_argument("SDK pose history is empty");
  }
  if (stamp <= samples_.front().first) {
    return samples_.front().second;
  }
  for (auto first = samples_.begin(); std::next(first) != samples_.end(); ++first) {
    const auto second = std::next(first);
    if (stamp <= second->first) {
      const double ratio = (stamp - first->first) / (second->first - first->first);
      Vec3 result{};
      for (int i = 0; i < 3; ++i) {
        result[i] = first->second[i] + ratio * (second->second[i] - first->second[i]);
      }
      return result;
    }
  }
  return samples_.back().second;
}

ChassisOdomSource::ChassisOdomSource(
    const double jump_threshold_m, const std::string &velocity_frame)
    : jump_threshold_m_(jump_threshold_m),
      velocity_body_(velocity_frame == "body"),
      has_previous_pose_(false),
      previous_pose_{0.0, 0.0, 0.0},
      samples_(0),
      jumps_(0),
      max_jump_m_(0.0),
      travelled_m_(0.0),
      jump_history_{} {
  if (!(jump_threshold_m > 0.0)) {
    throw std::invalid_argument("jump_threshold_m must be positive");
  }
  if (velocity_frame != "body" && velocity_frame != "world") {
    throw std::invalid_argument("velocity_frame must be 'body' or 'world'");
  }
}

OdomSample ChassisOdomSource::sample(const Vec3 &pos, const Vec3 &vel) {
  double jump_m = 0.0;
  bool jumped = false;
  if (has_previous_pose_) {
    jump_m = pose_jump_distance(pos, previous_pose_);
    jumped = jump_m > jump_threshold_m_;
  }

  ++samples_;
  if (jumped) {
    ++jumps_;
    max_jump_m_ = std::max(max_jump_m_, jump_m);
    if (jump_history_.size() < 32U) {
      // Python's round(jump_m, 4) uses decimal rounding for this diagnostic
      // field.  The history is not used for control, but keep the visible
      // value at the same precision as the reference implementation.
      jump_history_.push_back(std::nearbyint(jump_m * 1.0e4) / 1.0e4);
    }
  } else if (has_previous_pose_) {
    travelled_m_ += jump_m;
  }

  previous_pose_ = pos;
  has_previous_pose_ = true;

  double vx_body = vel[0];
  double vy_body = vel[1];
  if (!velocity_body_) {
    const double c = std::cos(pos[kTheta]);
    const double s = std::sin(pos[kTheta]);
    vx_body = c * vel[0] + s * vel[1];
    vy_body = -s * vel[0] + c * vel[1];
  }
  return {pos[kX], pos[kY], wrap_angle(pos[kTheta]), vx_body, vy_body,
          vel[kTheta], jump_m, jumped};
}

OdomStats ChassisOdomSource::stats() const {
  return {samples_, jumps_, max_jump_m_, travelled_m_, jump_history_};
}

double ChassisOdomSource::jump_ratio() const {
  return samples_ == 0U ? 0.0
                        : static_cast<double>(jumps_) /
                              static_cast<double>(samples_);
}

}  // namespace astribot_trajectory_bridge_native
