#include "astribot_s1_navigation_policy_native/navigation_math.hpp"
#include "astribot_s1_navigation_policy_native/policy_numeric.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>

namespace astribot::navigation {
namespace {

void require_command(const std::vector<double>& command) {
  if (command.size() != 3U) {
    throw std::invalid_argument("command must contain vx, vy, wz");
  }
}

}  // namespace

double stopping_horizon(const std::vector<double>& command,
                        const double reaction_time,
                        const double linear_brake,
                        const double angular_brake,
                        const double linear_stop_delay) {
  require_command(command);
  const double linear = std::hypot(command[0], command[1]);
  const double linear_tail = linear > 0.0
      ? linear / linear_brake + linear_stop_delay
      : 0.0;
  return reaction_time + std::max(linear_tail, std::abs(command[2]) / angular_brake);
}

std::vector<double> body_pose(const std::vector<double>& command, const double t) {
  require_command(command);
  const double vx = command[0];
  const double vy = command[1];
  const double wz = command[2];
  const double theta = wz * t;
  if (std::abs(wz) < 1e-6) {
    return {vx * t, vy * t, theta};
  }
  return {
      (vx * std::sin(theta) + vy * (std::cos(theta) - 1.0)) / wz,
      (vx * (1.0 - std::cos(theta)) + vy * std::sin(theta)) / wz,
      theta,
  };
}

std::vector<double> body_to_world_xy(const double vx, const double vy,
                                     const double yaw) {
  const double body_speed = std::hypot(vx, vy);
  if (body_speed <= 1e-6) {
    return {0.0, 0.0};
  }
  const double body_angle = std::atan2(vy, vx);
  const double world_angle = body_angle + yaw;
  return {std::cos(world_angle) * body_speed,
          std::sin(world_angle) * body_speed};
}

double sampling_margin(const std::vector<double>& command,
                       const double half_length,
                       const double half_width,
                       const double step) {
  require_command(command);
  return (std::hypot(command[0], command[1]) +
          std::hypot(half_length, half_width) * std::abs(command[2])) * step / 2.0;
}

std::vector<double> footprint_axes(const double length, const double width,
                                   const double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c, s, std::abs(c) * length + std::abs(s) * width,
          std::abs(s) * length + std::abs(c) * width,
          std::hypot(length, width)};
}

bool scan_usable(const std::vector<double>& ranges, const double range_min,
                 const double range_max, const double angle_min,
                 const double angle_increment, const double minimum_fraction) {
  if (ranges.empty() || !std::isfinite(range_min) ||
      !std::isfinite(range_max) || !std::isfinite(angle_min) ||
      !std::isfinite(angle_increment) || !(0.0 <= range_min) ||
      !(range_min < range_max) || !(angle_increment > 0.0)) {
    return false;
  }
  std::size_t valid = 0;
  for (const double range : ranges) {
    if (range == INFINITY ||
        (std::isfinite(range) && range_min <= range && range <= range_max)) {
      ++valid;
    }
  }
  return static_cast<double>(valid) / static_cast<double>(ranges.size()) >=
         minimum_fraction;
}

std::vector<std::array<std::int64_t, 2>> occupied_cells(
    const std::vector<std::array<double, 2>>& points, const double resolution) {
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    throw std::invalid_argument("positive scan occupancy resolution required");
  }
  std::set<std::pair<std::int64_t, std::int64_t>> cells;
  for (const auto& point : points) {
    if (!std::isfinite(point[0]) || !std::isfinite(point[1])) {
      throw std::invalid_argument("finite scan point required");
    }
    cells.emplace(static_cast<std::int64_t>(std::floor(point[0] / resolution)),
                  static_cast<std::int64_t>(std::floor(point[1] / resolution)));
  }
  std::vector<std::array<std::int64_t, 2>> result;
  result.reserve(cells.size());
  for (const auto& cell : cells) result.push_back({cell.first, cell.second});
  return result;
}

bool angular_box_free(const std::vector<std::array<double, 2>>& corners,
                      const std::vector<double>& ranges,
                      const double range_min, const double range_max,
                      const double angle_min, const double angle_increment,
                      const double resolution) {
  if (corners.empty() || !std::isfinite(range_min) ||
      !std::isfinite(range_max) || !std::isfinite(angle_min) ||
      !std::isfinite(angle_increment) || !std::isfinite(resolution) ||
      angle_increment == 0.0) {
    return false;
  }
  double center_x = 0.0;
  double center_y = 0.0;
  for (const auto& corner : corners) {
    if (!std::isfinite(corner[0]) || !std::isfinite(corner[1])) return false;
    center_x += corner[0];
    center_y += corner[1];
  }
  center_x /= static_cast<double>(corners.size());
  center_y /= static_cast<double>(corners.size());
  constexpr double kTwoPi = 6.283185307179586476925286766559;
  constexpr double kPi = 3.1415926535897932384626433832795;
  const double center = std::atan2(center_y, center_x);
  double min_angle = std::numeric_limits<double>::infinity();
  double max_angle = -std::numeric_limits<double>::infinity();
  double far = 0.0;
  double near = std::numeric_limits<double>::infinity();
  for (const auto& corner : corners) {
    const double angle = center + std::remainder(
        std::atan2(corner[1], corner[0]) - center, kTwoPi);
    min_angle = std::min(min_angle, angle);
    max_angle = std::max(max_angle, angle);
    const double distance = std::hypot(corner[0], corner[1]);
    far = std::max(far, distance);
    near = std::min(near, distance);
  }
  if (near < range_min || far >= range_max - 0.2 ||
      max_angle - min_angle >= kPi || far * angle_increment > resolution) {
    return false;
  }
  const auto lo = static_cast<long long>(
      std::floor((min_angle - angle_min) / angle_increment));
  const auto hi = static_cast<long long>(
      std::ceil((max_angle - angle_min) / angle_increment));
  const bool full = std::abs(static_cast<double>(ranges.size()) *
                                 angle_increment - kTwoPi) <=
                    1.5 * angle_increment;
  for (long long index = lo; index <= hi; ++index) {
    long long wrapped = index;
    if (wrapped < 0 || wrapped >= static_cast<long long>(ranges.size())) {
      if (!full) return false;
      if (ranges.empty()) return false;
      wrapped %= static_cast<long long>(ranges.size());
      if (wrapped < 0) wrapped += static_cast<long long>(ranges.size());
    }
    const double observed = ranges[static_cast<std::size_t>(wrapped)];
    if (std::isnan(observed) || observed == -INFINITY ||
        std::min(observed, range_max) < far + 0.15) {
      return false;
    }
  }
  return true;
}

std::pair<bool, std::string> posture_out_of_bounds(
    const double z, const double roll, const double pitch,
    const double normal_height, const double max_height_deviation,
    const double max_tilt_rad) {
  if (!std::isfinite(z) || !std::isfinite(roll) || !std::isfinite(pitch)) {
    return {true, "姿态含非有限数值"};
  }
  const double height_error = std::abs(z - normal_height);
  if (height_error > max_height_deviation) {
    char reason[256];
    std::snprintf(reason, sizeof(reason),
                  "高度偏差 |%.4f - %.4f| = %.4f 超过 %.4f", z,
                  normal_height, height_error, max_height_deviation);
    return {true, reason};
  }
  if (std::abs(roll) > max_tilt_rad) {
    char reason[128];
    std::snprintf(reason, sizeof(reason), "横滚 |%.4f| 超过 %.4f", roll,
                  max_tilt_rad);
    return {true, reason};
  }
  if (std::abs(pitch) > max_tilt_rad) {
    char reason[128];
    std::snprintf(reason, sizeof(reason), "俯仰 |%.4f| 超过 %.4f", pitch,
                  max_tilt_rad);
    return {true, reason};
  }
  return {false, ""};
}

bool is_degenerate_attitude_source(
    const std::vector<std::array<double, 3>>& samples, const double eps,
    const std::size_t min_samples) {
  if (samples.size() < min_samples) return false;
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    double low = samples.front()[axis];
    double high = samples.front()[axis];
    for (const auto& sample : samples) {
      low = std::min(low, sample[axis]);
      high = std::max(high, sample[axis]);
    }
    if (high - low > eps) return false;
  }
  return true;
}

std::string describe_monitor_state(const bool enabled, const bool degenerate,
                                   const bool tripped) {
  if (!enabled) return "姿态监控已由配置显式禁用。";
  if (degenerate) return "姿态数据不可用。";
  if (tripped) return "姿态监控**已止损**：检测到异常姿态，持续下发零速度。";
  return "姿态监控已启用：等待有效 /odom，采样窗口结束后执行阈值判定。";
}

std::pair<std::string, std::string> evaluate_posture(
    const bool enabled, const std::vector<std::array<double, 3>>& samples,
    const double z, const double roll, const double pitch,
    const double normal_height, const double max_height_deviation,
    const double max_tilt_rad, const std::size_t min_samples) {
  if (!enabled) return {"disabled", ""};
  if (samples.size() < min_samples) return {"collecting", ""};
  const auto result = posture_out_of_bounds(
      z, roll, pitch, normal_height, max_height_deviation, max_tilt_rad);
  return result.first ? std::make_pair(std::string("trip"), result.second)
                      : std::make_pair(std::string("pass"), std::string());
}

std::vector<double> lateral_variants(const std::vector<double>& route_xy,
                                     const double maximum) {
  if (route_xy.size() % 2U != 0U || route_xy.size() / 2U < 3U ||
      !(maximum > 0.0) || maximum > 0.2 ||
      !std::all_of(route_xy.begin(), route_xy.end(),
                   [](const double value) { return std::isfinite(value); })) {
    return {};
  }
  const std::size_t count = route_xy.size() / 2U;
  std::vector<double> lengths(count, 0.0);
  for (std::size_t i = 1; i < count; ++i) {
    lengths[i] = lengths[i - 1U] + std::hypot(
        route_xy[2U * i] - route_xy[2U * (i - 1U)],
        route_xy[2U * i + 1U] - route_xy[2U * (i - 1U) + 1U]);
  }
  const double total = lengths.back();
  const double dx = route_xy[route_xy.size() - 2U] - route_xy[0];
  const double dy = route_xy[route_xy.size() - 1U] - route_xy[1];
  const double chord = std::hypot(dx, dy);
  if (!std::isfinite(total) || total < 0.6 || chord < 0.1) return {};
  std::vector<double> weights(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double s = lengths[i] / total;
    weights[i] = 64.0 * s * s * s * (1.0 - s) * (1.0 - s) * (1.0 - s);
  }
  weights.front() = 0.0;
  weights.back() = 0.0;
  const std::array<double, 8> offsets = {
      -maximum / 4.0, maximum / 4.0, -maximum / 2.0, maximum / 2.0,
      -3.0 * maximum / 4.0, 3.0 * maximum / 4.0, -maximum, maximum};
  std::vector<double> result;
  result.reserve(offsets.size() * route_xy.size());
  for (const double offset : offsets) {
    for (std::size_t i = 0; i < count; ++i) {
      result.push_back(route_xy[2U * i] - dy / chord * offset * weights[i]);
      result.push_back(route_xy[2U * i + 1U] + dx / chord * offset * weights[i]);
    }
  }
  return result;
}

std::vector<double> scan_coverage(const std::vector<double>& ranges,
                                  const double range_min,
                                  const double range_max,
                                  const double angle_min,
                                  const double angle_increment,
                                  const double body_yaw) {
  std::vector<double> result;
  bool open = false;
  std::size_t start = 0U;
  for (std::size_t index = 0U; index <= ranges.size(); ++index) {
    const bool good = index < ranges.size() &&
        (std::isfinite(ranges[index]) && range_min <= ranges[index] &&
             ranges[index] <= range_max || ranges[index] == INFINITY);
    if (good && !open) {
      start = index;
      open = true;
    }
    if (!good && open) {
      const double middle = angle_min +
          (static_cast<double>(start + index - 1U)) * 0.5 * angle_increment +
          body_yaw;
      const double half = std::min(
          3.1415926535897932384626433832795,
          static_cast<double>(index - start) * std::abs(angle_increment) * 0.5);
      result.insert(result.end(), {std::cos(middle), std::sin(middle), 0.0, half});
      open = false;
    }
  }
  return result;
}

std::vector<double> movement_directions(const double vx, const double vy,
                                        const double wz) {
  if (std::abs(wz) > 0.02) {
    std::vector<double> result;
    result.reserve(16U);
    for (int i = 0; i < 16; ++i) {
      result.push_back(static_cast<double>(i) *
                       3.1415926535897932384626433832795 / 8.0);
    }
    return result;
  }
  if (policy::euclidean_norm(vx, vy) < 0.01) return {};
  const double angle = std::atan2(vy, vx);
  return {angle - 3.1415926535897932384626433832795 / 4.0, angle,
          angle + 3.1415926535897932384626433832795 / 4.0};
}

bool coverage_allows_motion(const std::vector<double>& cones_xy_half,
                            const double vx, const double vy,
                            const double wz) {
  if (cones_xy_half.size() % 3U != 0U) {
    throw std::invalid_argument("coverage cones must contain x, y, half_angle");
  }
  double left = 0.0;
  double right = 0.0;
  if (std::abs(wz) > 0.02) {
    left = -3.1415926535897932384626433832795;
    right = 3.1415926535897932384626433832795;
  } else if (policy::euclidean_norm(vx, vy) >= 0.01) {
    const double angle = std::atan2(vy, vx);
    left = angle - 3.1415926535897932384626433832795 / 4.0;
    right = angle + 3.1415926535897932384626433832795 / 4.0;
  } else {
    return true;
  }
  std::vector<std::array<double, 2>> intervals;
  intervals.reserve(3U * cones_xy_half.size() / 3U);
  constexpr double kTwoPi = 6.283185307179586476925286766559;
  for (std::size_t i = 0; i < cones_xy_half.size(); i += 3U) {
    const double angle = std::atan2(cones_xy_half[i + 1U],
                                    cones_xy_half[i]);
    const double half = cones_xy_half[i + 2U];
    for (const double offset : {-kTwoPi, 0.0, kTwoPi}) {
      intervals.push_back({angle + offset - half, angle + offset + half});
    }
  }
  std::sort(intervals.begin(), intervals.end(),
            [](const auto& lhs, const auto& rhs) { return lhs[0] < rhs[0]; });
  double cursor = left;
  for (const auto& interval : intervals) {
    if (interval[1] < cursor) continue;
    if (interval[0] > cursor + 1e-6) return false;
    cursor = std::max(cursor, interval[1]);
    if (cursor >= right - 1e-6) return true;
  }
  return false;
}

PathAssessmentResult assess_path(
    const bool known, const bool blocked, const double stamp_s,
    const double received_wall_s, const double distance_m, const double now_s,
    const double wall_s, const double path_risk_timeout_s,
    const double max_speed_m_s, const double clearance_margin_m,
    const double payload_extra_margin_m, const bool legacy_blocked) {
  if (!std::isfinite(now_s) || !std::isfinite(wall_s) ||
      !std::isfinite(stamp_s) || !std::isfinite(received_wall_s) ||
      !std::isfinite(distance_m)) {
    return {true, 0.0, "INVALID", 0.0};
  }
  (void)path_risk_timeout_s;(void)legacy_blocked;
  if (!known || distance_m < 0.0) {
    return {true, 0.0, "UNKNOWN", 0.0};
  }
  if (!blocked) {
    return {false, INFINITY, "CLEAR", 0.0};
  }
  const double distance = std::max(
      0.0, distance_m - clearance_margin_m - payload_extra_margin_m);
  return {true, distance / max_speed_m_s, "OCCUPIED", distance};
}

bool yield_requires_stop(const bool immediate, const bool uncertain,
                         const bool blocked, const double conflict_time_s,
                         const double max_speed_m_s,
                         const double reaction_time_s,
                         const double brake_deceleration_m_s2,
                         const double linear_stop_delay_s) {
  const double stop_time = reaction_time_s +
      (max_speed_m_s > 0.0
           ? max_speed_m_s / brake_deceleration_m_s2 + linear_stop_delay_s
           : 0.0);
  return immediate || uncertain ||
         (blocked && !(conflict_time_s > stop_time + 0.5));
}

YieldPolicy::YieldPolicy(
    const double max_speed_m_s, const double narrow_speed_m_s,
    const double reaction_time_s, const double brake_deceleration_m_s2,
    const double linear_stop_delay_s, const double wait_budget_s,
    const double clear_hold_s)
    : max_speed_m_s_(max_speed_m_s),
      narrow_speed_m_s_(narrow_speed_m_s),
      stop_time_s_(reaction_time_s + (max_speed_m_s > 0.0
                   ? max_speed_m_s / brake_deceleration_m_s2 + linear_stop_delay_s
                   : 0.0)),
      wait_budget_s_(wait_budget_s),
      clear_hold_s_(clear_hold_s),
      has_blocked_at_(false),
      blocked_at_(0.0),
      has_clear_at_(false),
      clear_at_(0.0),
      has_last_time_(false),
      last_time_(0.0),
      held_(true),
      episode_(0),
      in_episode_(false) {
  if (!std::isfinite(max_speed_m_s) || !std::isfinite(narrow_speed_m_s) ||
      !std::isfinite(reaction_time_s) ||
      !std::isfinite(brake_deceleration_m_s2) ||
      !std::isfinite(linear_stop_delay_s) || !std::isfinite(wait_budget_s) ||
      !std::isfinite(clear_hold_s) || max_speed_m_s < 0.0 ||
      narrow_speed_m_s < 0.0 || reaction_time_s < 0.0 ||
      brake_deceleration_m_s2 <= 0.0 || linear_stop_delay_s < 0.0 ||
      wait_budget_s < 0.0 || clear_hold_s < 0.0) {
    throw std::invalid_argument("invalid yield policy parameters");
  }
}

void YieldPolicy::update_profile(
    const double max_speed_m_s, const double narrow_speed_m_s,
    const double reaction_time_s, const double brake_deceleration_m_s2,
    const double linear_stop_delay_s, const double wait_budget_s,
    const double clear_hold_s) {
  const YieldPolicy checked(max_speed_m_s, narrow_speed_m_s, reaction_time_s,
      brake_deceleration_m_s2, linear_stop_delay_s, wait_budget_s, clear_hold_s);
  max_speed_m_s_ = checked.max_speed_m_s_;
  narrow_speed_m_s_ = checked.narrow_speed_m_s_;
  stop_time_s_ = checked.stop_time_s_;
  wait_budget_s_ = checked.wait_budget_s_;
  clear_hold_s_ = checked.clear_hold_s_;
}

SelectionResult YieldPolicy::select(const bool immediate, const bool blocked,
                                    const bool uncertain,
                                    const double conflict_time_s,
                                    const bool valid, const double now) {
  if (!std::isfinite(now)) {
    throw std::invalid_argument("finite policy time required");
  }
  if (has_last_time_ && now < last_time_) {
    has_blocked_at_ = false;
    has_clear_at_ = false;
    held_ = true;
    in_episode_ = false;
  }
  last_time_ = now;
  has_last_time_ = true;
  if (!valid) {
    held_ = true;
    has_clear_at_ = false;
    return {"HOLD", 0.0, "INPUT_UNAVAILABLE", episode_};
  }
  const bool obstructed = immediate || blocked || uncertain;
  if (obstructed) {
    if (!in_episode_) {
      ++episode_;
      in_episode_ = true;
    }
    const bool must_stop = immediate || uncertain ||
        (blocked && !(conflict_time_s > stop_time_s_ + 0.5));
    if (must_stop) {
      if (!has_blocked_at_) {
        blocked_at_ = now;
        has_blocked_at_ = true;
      }
      has_clear_at_ = false;
      held_ = true;
      std::string reason = uncertain ? "UNKNOWN_GEOMETRY"
                          : (immediate ? "IMMEDIATE_RISK" : "YIELD");
      if (now - blocked_at_ > wait_budget_s_) {
        reason = "BLOCKED_CAPABILITY_NOT_ENABLED";
      }
      return {"HOLD", 0.0, reason, episode_};
    }
  }
  if (held_) {
    if (!has_clear_at_) {
      clear_at_ = now;
      has_clear_at_ = true;
    }
    if (now - clear_at_ < clear_hold_s_) {
      return {"HOLD", 0.0, "CLEAR_CONFIRMATION", episode_};
    }
  }
  held_ = false;
  has_clear_at_ = false;
  if (obstructed) {
    has_blocked_at_ = false;
    return {"SLOW", narrow_speed_m_s_, "PREDICTED_CONFLICT", episode_};
  }
  has_blocked_at_ = false;
  in_episode_ = false;
  return {"CONTINUE", max_speed_m_s_, "CLEAR", episode_};
}

std::vector<double> YieldPolicy::state() const {
  return {has_blocked_at_ ? 1.0 : 0.0,
          blocked_at_,
          has_clear_at_ ? 1.0 : 0.0,
          clear_at_,
          has_last_time_ ? 1.0 : 0.0,
          last_time_,
          held_ ? 1.0 : 0.0,
          static_cast<double>(episode_),
          in_episode_ ? 1.0 : 0.0};
}

ExecutionContext::ExecutionContext()
    : goal_id_("idle"),
      path_revision_(0),
      map_epoch_(0),
      envelope_epoch_(0),
      localization_epoch_(0),
      clock_epoch_(0),
      sequence_(0),
      has_sequence_(false),
      map_key_(),
      has_map_key_(false),
      has_transform_(false),
      transform_{0.0, 0.0, 0.0} {}

void ExecutionContext::task(const std::string& identifier,
                            const std::string& state,
                            const std::int64_t sequence) {
  if (sequence < 0) return;
  task_wire(identifier, state, static_cast<std::uint64_t>(sequence));
}

void ExecutionContext::task_wire(const std::string& identifier,
                                 const std::string& state,
                                 const std::uint64_t sequence) {
  if (has_sequence_ && sequence <= sequence_) return;
  sequence_ = sequence;
  has_sequence_ = true;
  if (state == "EXECUTING") {
    if (identifier != goal_id_) {
      goal_id_ = identifier;
      path_revision_ = 0;
    }
  } else if ((state == "SUCCEEDED" || state == "FAILED" ||
              state == "CANCELED" || state == "PREEMPTED") &&
             identifier == goal_id_) {
    goal_id_ = "idle";
  }
}

bool ExecutionContext::map(const std::string& key) {
  if (has_map_key_ && key == map_key_) return false;
  if (map_epoch_ == UINT64_MAX) throw std::overflow_error("map_epoch exceeds uint64 wire range");
  map_key_ = key;
  has_map_key_ = true;
  ++map_epoch_;
  return true;
}

bool ExecutionContext::localization(const std::vector<double>& pose,
                                    const double position_limit,
                                    const double angle_limit) {
  if (pose.size() != 3U ||
      !std::all_of(pose.begin(), pose.end(),
                   [](const double value) { return std::isfinite(value); })) {
    throw std::invalid_argument("finite localization pose required");
  }
  const bool changed = has_transform_ &&
      (policy::euclidean_norm(pose[0] - transform_[0], pose[1] - transform_[1]) >
           position_limit ||
       std::abs(std::remainder(pose[2] - transform_[2],
                               6.283185307179586476925286766559)) >
           angle_limit);
  if (changed && localization_epoch_ == UINT64_MAX)
    throw std::overflow_error("localization_epoch exceeds uint64 wire range");
  transform_ = {pose[0], pose[1], pose[2]};
  has_transform_ = true;
  if (changed) ++localization_epoch_;
  return changed;
}

void ExecutionContext::path() {
  if (path_revision_ == UINT64_MAX) throw std::overflow_error("path_revision exceeds uint64 wire range");
  ++path_revision_;
}

void ExecutionContext::set_version(const std::string& goal_id,
                                   const std::uint64_t path_revision,
                                   const std::uint64_t map_epoch,
                                   const std::uint64_t envelope_epoch,
                                   const std::uint64_t localization_epoch,
                                   const std::uint64_t clock_epoch) {
  goal_id_ = goal_id;
  path_revision_ = path_revision;
  map_epoch_ = map_epoch;
  envelope_epoch_ = envelope_epoch;
  localization_epoch_ = localization_epoch;
  clock_epoch_ = clock_epoch;
}

std::tuple<std::string, std::uint64_t, std::uint64_t, std::uint64_t,
           std::uint64_t, std::uint64_t>
ExecutionContext::version() const {
  return {goal_id_, path_revision_, map_epoch_, envelope_epoch_,
          localization_epoch_, clock_epoch_};
}

PlanningSessionState::PlanningSessionState(
    std::string session_id, const std::int64_t request_timeout_ns,
    const std::int64_t episode_timeout_ns,
    const std::int64_t max_requests_per_goal)
    : session_id_(std::move(session_id)),
      request_timeout_ns_(request_timeout_ns),
      episode_timeout_ns_(episode_timeout_ns),
      max_requests_per_goal_(max_requests_per_goal),
      has_version_(false),
      goal_id_(),
      path_revision_(0),
      map_epoch_(0),
      envelope_epoch_(0),
      localization_epoch_(0),
      clock_epoch_(0),
      has_pending_(false),
      pending_id_(),
      pending_issued_ns_(0),
      pending_issued_epoch_(0),
      pending_valid_until_ns_(0),
      has_last_time_(false),
      last_time_ns_(0),
      last_time_epoch_(0),
      has_blocked_at_(false),
      blocked_at_ns_(0),
      clock_fault_(false),
      serial_(0),
      episode_(0),
      attempts_(0) {
  if (session_id_.empty() || request_timeout_ns_ <= 0 ||
      episode_timeout_ns_ < request_timeout_ns_ || max_requests_per_goal_ <= 0) {
    throw std::invalid_argument("invalid planning session budget");
  }
}

bool PlanningSessionState::same_version(
    const std::string& goal_id, const std::int64_t path_revision,
    const std::int64_t map_epoch, const std::int64_t envelope_epoch,
    const std::int64_t localization_epoch,
    const std::int64_t clock_epoch) const {
  return has_version_ && goal_id == goal_id_ &&
         path_revision == path_revision_ && map_epoch == map_epoch_ &&
         envelope_epoch == envelope_epoch_ &&
         localization_epoch == localization_epoch_ &&
         clock_epoch == clock_epoch_;
}

bool PlanningSessionState::advance(const std::int64_t now_ns,
                                   const std::int64_t clock_epoch) {
  const bool backward = has_last_time_ &&
      (clock_epoch != last_time_epoch_ || now_ns < last_time_ns_);
  has_last_time_ = true;
  last_time_ns_ = now_ns;
  last_time_epoch_ = clock_epoch;
  if (backward) {
    has_pending_ = false;
    clock_fault_ = true;
  }
  return !clock_fault_;
}

void PlanningSessionState::activate(
    const std::string& goal_id, const std::int64_t path_revision,
    const std::int64_t map_epoch, const std::int64_t envelope_epoch,
    const std::int64_t localization_epoch, const std::int64_t clock_epoch,
    const std::int64_t now_ns) {
  if (!has_version_ || goal_id != goal_id_) {
    has_pending_ = false;
    has_blocked_at_ = false;
    attempts_ = 0;
    clock_fault_ = false;
    has_last_time_ = true;
    last_time_ns_ = now_ns;
    last_time_epoch_ = clock_epoch;
  } else {
    advance(now_ns, clock_epoch);
    if (!same_version(goal_id, path_revision, map_epoch, envelope_epoch,
                      localization_epoch, clock_epoch)) {
      has_pending_ = false;
    }
  }
  has_version_ = true;
  goal_id_ = goal_id;
  path_revision_ = path_revision;
  map_epoch_ = map_epoch;
  envelope_epoch_ = envelope_epoch;
  localization_epoch_ = localization_epoch;
  clock_epoch_ = clock_epoch;
}

void PlanningSessionState::finish(const std::string& goal_id) {
  if (has_version_ && goal_id_ == goal_id) {
    has_pending_ = false;
    has_version_ = false;
    has_blocked_at_ = false;
  }
}

bool PlanningSessionState::clear_blockage(const std::int64_t now_ns,
                                          const std::int64_t clock_epoch) {
  if (advance(now_ns, clock_epoch)) {
    has_pending_ = false;
    has_blocked_at_ = false;
  }
  return !clock_fault_;
}

std::tuple<int, std::string, std::int64_t, std::int64_t, std::int64_t>
PlanningSessionState::request(
    const std::string& goal_id, const std::int64_t path_revision,
    const std::int64_t map_epoch, const std::int64_t envelope_epoch,
    const std::int64_t localization_epoch, const std::int64_t clock_epoch,
    const std::int64_t observation_seq, const std::int64_t now_ns) {
  (void)observation_seq;
  if (!same_version(goal_id, path_revision, map_epoch, envelope_epoch,
                    localization_epoch, clock_epoch)) {
    return {5, std::string(), 0, 0, 0};
  }
  if (!advance(now_ns, clock_epoch)) return {2, std::string(), 0, 0, 0};
  if (!has_blocked_at_) {
    has_blocked_at_ = true;
    blocked_at_ns_ = now_ns;
    ++episode_;
  }
  const std::int64_t episode_end = blocked_at_ns_ + episode_timeout_ns_;
  if (now_ns >= episode_end) {
    has_pending_ = false;
    return {3, std::string(), 0, 0, 0};
  }
  if (has_pending_ && now_ns < pending_valid_until_ns_) {
    return {1, pending_id_, episode_, pending_valid_until_ns_,
            pending_issued_epoch_};
  }
  has_pending_ = false;
  if (attempts_ >= max_requests_per_goal_) {
    return {4, std::string(), 0, 0, 0};
  }
  ++serial_;
  ++attempts_;
  pending_id_ = session_id_ + ":" + std::to_string(serial_);
  pending_issued_ns_ = now_ns;
  pending_issued_epoch_ = clock_epoch;
  pending_valid_until_ns_ = std::min(
      episode_end, now_ns + request_timeout_ns_);
  has_pending_ = true;
  return {0, pending_id_, episode_, pending_valid_until_ns_,
          pending_issued_epoch_};
}

bool PlanningSessionState::response_current(
    const std::string& request_id, const std::string& goal_id,
    const std::int64_t path_revision, const std::int64_t map_epoch,
    const std::int64_t envelope_epoch, const std::int64_t localization_epoch,
    const std::int64_t clock_epoch, const std::int64_t now_ns) {
  return advance(now_ns, clock_epoch) && has_version_ && has_pending_ &&
         request_id == pending_id_ &&
         same_version(goal_id, path_revision, map_epoch, envelope_epoch,
                      localization_epoch, clock_epoch) &&
         clock_epoch == pending_issued_epoch_ &&
         now_ns >= pending_issued_ns_ &&
         now_ns < pending_valid_until_ns_;
}

int PlanningSessionState::failure_reason(const std::int64_t now_ns,
                                         const std::int64_t clock_epoch) {
  if (!advance(now_ns, clock_epoch)) return 1;
  if (!has_version_ || !has_blocked_at_) return 0;
  if (now_ns - blocked_at_ns_ >= episode_timeout_ns_) {
    has_pending_ = false;
    return 2;
  }
  if (attempts_ >= max_requests_per_goal_ &&
      (!has_pending_ || now_ns >= pending_valid_until_ns_)) {
    has_pending_ = false;
    return 3;
  }
  return 0;
}

void PlanningSessionState::retire(const std::string& request_id) {
  if (has_pending_ && request_id == pending_id_) has_pending_ = false;
}

ControlTime::ControlTime(const bool simulated, const double stall_timeout_s)
    : simulated_(simulated),
      stall_timeout_s_(stall_timeout_s),
      has_last_(false),
      last_ros_(0.0),
      last_wall_(0.0),
      has_progress_(false),
      progress_wall_(0.0),
      stalled_(false),
      capture_floor_(-INFINITY),
      command_floor_(-INFINITY) {
  if (!std::isfinite(stall_timeout_s) || stall_timeout_s <= 0.0) {
    throw std::invalid_argument("invalid clock watchdog timeout");
  }
}

TimeStepResult ControlTime::advance(const double ros, const double wall) {
  if (!std::isfinite(ros) || !std::isfinite(wall)) {
    throw std::invalid_argument("nonfinite clock");
  }
  const double wall_dt = has_last_ ? wall - last_wall_ : 0.0;
  const double ros_dt = has_last_ ? ros - last_ros_ : 0.0;
  const bool backward = has_last_ && ros_dt < 0.0;
  if (!has_progress_ || ros_dt > 0.0 || backward) {
    progress_wall_ = wall;
    has_progress_ = true;
  }
  const bool stalled = simulated_ &&
      (ros <= 0.0 || wall - progress_wall_ > stall_timeout_s_);
  const bool reset = backward;
  const bool stop_commands = stalled && !stalled_;
  if (backward) {
    capture_floor_ = ros;
    command_floor_ = -INFINITY;
  }
  if (stop_commands) command_floor_ = ros;
  last_ros_ = ros;
  last_wall_ = wall;
  has_last_ = true;
  stalled_ = stalled;
  return {simulated_ ? ros : wall,
          simulated_ ? std::max(0.0, ros_dt) : wall_dt,
          wall_dt,
          !stalled,
          reset,
          stop_commands};
}

bool ControlTime::accepts(const double capture) const {
  return std::isfinite(capture);
}

bool ControlTime::fresh(const double capture, const double received_wall,
                        const double ttl, const double ros,
                        const double wall) const {
  (void)ttl;(void)ros;(void)wall;
  return accepts(capture)&&std::isfinite(received_wall);

}

bool ControlTime::command_fresh(const double received_ros,
                                const double received_wall, const double ttl,
                                const double ros, const double wall) const {
  (void)received_ros;(void)ros;
  return 0.0 <= wall-received_wall && wall-received_wall<=ttl;

}

std::vector<double> ControlTime::state() const {
  return {has_last_ ? 1.0 : 0.0,
          last_ros_,
          last_wall_,
          has_progress_ ? 1.0 : 0.0,
          progress_wall_,
          stalled_ ? 1.0 : 0.0,
          capture_floor_,
          command_floor_};
}

std::vector<double> costmap_clearing_ranges(
    const std::vector<double>& ranges, const double range_max,
    const double max_marking_range) {
  const double endpoint = range_max -
                          std::max(1e-4, std::abs(range_max) * 1e-6);
  if (!std::isfinite(endpoint) || !std::isfinite(max_marking_range) ||
      max_marking_range < 0.0 || endpoint <= max_marking_range) {
    throw std::invalid_argument(
        "clearing endpoint must lie beyond the costmap marking range");
  }
  std::vector<double> result;
  result.reserve(ranges.size());
  for (const double range : ranges) {
    result.push_back(range == range_max || range == INFINITY ? endpoint : range);
  }
  return result;
}

std::vector<double> clearance_many_rect(
    const std::vector<double>& x, const std::vector<double>& y,
    const std::vector<double>& yaw, const std::vector<double>& lower,
    const std::vector<double>& upper, const double half_length,
    const double half_width, const double margin) {
  const std::size_t count = x.size();
  if (y.size() != count || yaw.size() != count || lower.size() != 2U * count ||
      upper.size() != 2U * count) {
    throw std::invalid_argument("clearance arrays have inconsistent lengths");
  }
  if (!std::isfinite(half_length) || !std::isfinite(half_width) ||
      !std::isfinite(margin) || half_length < 0.0 || half_width < 0.0 ||
      margin < 0.0) {
    throw std::invalid_argument("finite nonnegative rectangle parameters required");
  }
  std::vector<double> result(count);
  const double radius = std::hypot(half_length, half_width);
  for (std::size_t i = 0; i < count; ++i) {
    const double c = std::cos(yaw[i]);
    const double s = std::sin(yaw[i]);
    const double ac = std::abs(c);
    const double as = std::abs(s);
    const double lx = lower[2U * i];
    const double ly = lower[2U * i + 1U];
    const double ux = upper[2U * i];
    const double uy = upper[2U * i + 1U];
    if (!std::isfinite(x[i]) || !std::isfinite(y[i]) ||
        !std::isfinite(yaw[i]) || !std::isfinite(lx) || !std::isfinite(ly) ||
        !std::isfinite(ux) || !std::isfinite(uy) || lx > ux || ly > uy) {
      throw std::invalid_argument("finite ordered rectangle values required");
    }
    const double bx = (lx + ux) / 2.0;
    const double by = (ly + uy) / 2.0;
    const double hx = (ux - lx) / 2.0;
    const double hy = (uy - ly) / 2.0;
    const double dx = x[i] - bx;
    const double dy = y[i] - by;
    const double separation = std::max({
        std::abs(dx) - ac * half_length - as * half_width - hx,
        std::abs(dy) - as * half_length - ac * half_width - hy,
        std::abs(dx * c + dy * s) - half_length - hx * ac - hy * as,
        std::abs(-dx * s + dy * c) - half_width - hx * as - hy * ac});
    const double outside_x = std::max({lx - x[i], 0.0, x[i] - ux});
    const double outside_y = std::max({ly - y[i], 0.0, y[i] - uy});
    const double circle = std::hypot(outside_x, outside_y) - radius;
    const double bound = std::max(separation, circle);
    double distance = bound;
    if (separation > 0.0 && bound <= margin + 1e-12) {
      distance = INFINITY;
      for (const int sx : {-1, 1}) {
        for (const int sy : {-1, 1}) {
          const double px = x[i] + c * sx * half_length - s * sy * half_width;
          const double py = y[i] + s * sx * half_length + c * sy * half_width;
          distance = std::min(
              distance,
              std::hypot(std::max({lx - px, 0.0, px - ux}),
                         std::max({ly - py, 0.0, py - uy})));
          const double corner_x = (sx < 0 ? lx : ux) - x[i];
          const double corner_y = (sy < 0 ? ly : uy) - y[i];
          distance = std::min(
              distance,
              std::hypot(std::max(std::abs(c * corner_x + s * corner_y) -
                                      half_length,
                                  0.0),
                         std::max(std::abs(-s * corner_x + c * corner_y) -
                                      half_width,
                                  0.0)));
        }
      }
    }
    result[i] = distance - margin - 1e-12;
  }
  return result;
}

std::vector<double> motion_clearance_rect(
    const std::vector<double>& command, const std::vector<double>& begin,
    const std::vector<double>& end, const std::vector<double>& lower,
    const std::vector<double>& upper, const double half_length,
    const double half_width, const double clearance_margin,
    const double payload_extra_margin, const std::vector<double>& origin) {
  require_command(command);
  if (origin.size() != 3U || begin.size() != end.size() ||
      lower.size() != 2U * begin.size() || upper.size() != 2U * begin.size()) {
    throw std::invalid_argument("motion clearance arrays have inconsistent lengths");
  }
  if (!std::all_of(origin.begin(), origin.end(),
                   [](const double value) { return std::isfinite(value); }) ||
      !std::isfinite(half_length) || !std::isfinite(half_width) ||
      !std::isfinite(clearance_margin) || !std::isfinite(payload_extra_margin) ||
      half_length < 0.0 || half_width < 0.0 || clearance_margin < 0.0 ||
      payload_extra_margin < 0.0 ||
      !std::all_of(command.begin(), command.end(),
                   [](const double value) { return std::isfinite(value); })) {
    throw std::invalid_argument("finite motion and profile values required");
  }
  const std::size_t count = begin.size();
  std::vector<double> start = begin;
  std::vector<double> finish = end;
  std::vector<std::size_t> owners(count);
  std::iota(owners.begin(), owners.end(), 0U);
  for (std::size_t i = 0; i < count; ++i) {
    if (!std::isfinite(start[i]) || !std::isfinite(finish[i]) ||
        start[i] < 0.0 || finish[i] < start[i] ||
        !std::isfinite(lower[2U * i]) || !std::isfinite(lower[2U * i + 1U]) ||
        !std::isfinite(upper[2U * i]) || !std::isfinite(upper[2U * i + 1U]) ||
        lower[2U * i] > upper[2U * i] ||
        lower[2U * i + 1U] > upper[2U * i + 1U]) {
      throw std::invalid_argument("finite ordered sweep intervals and bounds required");
    }
  }
  std::vector<double> result(count, std::numeric_limits<double>::infinity());
  const double speed = std::hypot(command[0], command[1]);
  const double radius = std::hypot(half_length, half_width);
  const double boundary_speed = speed + radius * std::abs(command[2]);
  const double margin = clearance_margin + payload_extra_margin;
  const double c0 = std::cos(origin[2]);
  const double s0 = std::sin(origin[2]);

  for (int depth = 0; depth <= 10 && !owners.empty(); ++depth) {
    std::vector<double> mid(owners.size()), half(owners.size());
    std::vector<double> x(owners.size()), y(owners.size()), yaw(owners.size());
    std::vector<double> lower_batch(2U * owners.size());
    std::vector<double> upper_batch(2U * owners.size());
    for (std::size_t j = 0; j < owners.size(); ++j) {
      const std::size_t owner = owners[j];
      mid[j] = (start[j] + finish[j]) / 2.0;
      half[j] = (finish[j] - start[j]) / 2.0;
      const auto pose = body_pose(command, mid[j]);
      x[j] = origin[0] + c0 * pose[0] - s0 * pose[1];
      y[j] = origin[1] + s0 * pose[0] + c0 * pose[1];
      yaw[j] = origin[2] + pose[2];
      lower_batch[2U * j] = lower[2U * owner];
      lower_batch[2U * j + 1U] = lower[2U * owner + 1U];
      upper_batch[2U * j] = upper[2U * owner];
      upper_batch[2U * j + 1U] = upper[2U * owner + 1U];
    }
    const auto point = clearance_many_rect(
        x, y, yaw, lower_batch, upper_batch, half_length, half_width, margin);
    std::vector<double> bound(owners.size());
    std::vector<bool> safe(owners.size()), rejected(owners.size()), pending(owners.size());
    for (std::size_t j = 0; j < owners.size(); ++j) {
      const std::size_t owner = owners[j];
      const double lx = lower_batch[2U * j];
      const double ly = lower_batch[2U * j + 1U];
      const double ux = upper_batch[2U * j];
      const double uy = upper_batch[2U * j + 1U];
      const double dx = std::max({lx - x[j], 0.0, x[j] - ux});
      const double dy = std::max({ly - y[j], 0.0, y[j] - uy});
      const double circle = std::hypot(dx, dy) - radius - margin -
                            speed * half[j] - 1e-12;
      bound[j] = std::max(point[j] - boundary_speed * half[j], circle);
      safe[j] = bound[j] > 0.0;
      rejected[j] = !safe[j] && (point[j] <= 0.0 || depth == 10);
      if (safe[j] || rejected[j]) result[owner] = std::min(result[owner], bound[j]);
    }
    std::vector<std::size_t> next_owners;
    std::vector<double> next_start, next_finish;
    for (std::size_t j = 0; j < owners.size(); ++j) {
      pending[j] = !safe[j] && !rejected[j] && result[owners[j]] > 0.0;
      if (!pending[j]) continue;
      const double midpoint = mid[j];
      next_owners.push_back(owners[j]);
      next_start.push_back(start[j]);
      next_finish.push_back(midpoint);
      next_owners.push_back(owners[j]);
      next_start.push_back(midpoint);
      next_finish.push_back(finish[j]);
    }
    owners.swap(next_owners);
    start.swap(next_start);
    finish.swap(next_finish);
  }
  return result;
}

std::vector<double> path_position_batch(
    const std::vector<double>& path_xy, const std::vector<double>& distances,
    const std::vector<double>& fallback) {
  if (path_xy.size() % 2U != 0U || fallback.size() != 3U ||
      !std::all_of(fallback.begin(), fallback.end(),
                   [](const double value) { return std::isfinite(value); }) ||
      !std::all_of(path_xy.begin(), path_xy.end(),
                   [](const double value) { return std::isfinite(value); }) ||
      !std::all_of(distances.begin(), distances.end(),
                   [](const double value) { return std::isfinite(value); })) {
    throw std::invalid_argument("finite path, distances and fallback required");
  }
  const std::size_t count = path_xy.size() / 2U;
  std::vector<double> result;
  result.reserve(3U * distances.size());
  for (const double distance : distances) {
    if (count == 0U) {
      result.insert(result.end(), fallback.begin(), fallback.end());
      continue;
    }
    double remaining = distance;
    bool found = false;
    for (std::size_t i = 0; i + 1U < count; ++i) {
      const double ax = path_xy[2U * i];
      const double ay = path_xy[2U * i + 1U];
      const double bx = path_xy[2U * (i + 1U)];
      const double by = path_xy[2U * (i + 1U) + 1U];
      const double length = std::hypot(bx - ax, by - ay);
      if (length > 1e-9 && remaining <= length) {
        const double ratio = remaining / length;
        result.push_back(ax + ratio * (bx - ax));
        result.push_back(ay + ratio * (by - ay));
        result.push_back(std::atan2(by - ay, bx - ax));
        found = true;
        break;
      }
      remaining -= length;
    }
    if (found) continue;
    const double last_x = path_xy[2U * (count - 1U)];
    const double last_y = path_xy[2U * (count - 1U) + 1U];
    double heading = fallback[2];
    for (std::size_t i = count - 1U; i > 0U; --i) {
      const double ax = path_xy[2U * (i - 1U)];
      const double ay = path_xy[2U * (i - 1U) + 1U];
      const double bx = path_xy[2U * i];
      const double by = path_xy[2U * i + 1U];
      if (std::hypot(bx - ax, by - ay) > 1e-9) {
        heading = std::atan2(by - ay, bx - ax);
        break;
      }
    }
    result.push_back(last_x);
    result.push_back(last_y);
    result.push_back(heading);
  }
  return result;
}

std::vector<double> remaining_path(
    const std::vector<double>& path_xy, const double robot_x,
    const double robot_y) {
  if (path_xy.size() % 2U != 0U ||
      !std::isfinite(robot_x) || !std::isfinite(robot_y) ||
      !std::all_of(path_xy.begin(), path_xy.end(),
                   [](const double value) { return std::isfinite(value); })) {
    throw std::invalid_argument("finite path and robot position required");
  }
  const std::size_t count = path_xy.size() / 2U;
  if (count == 0U) return {};
  bool has_best = false;
  double best_distance = 0.0;
  std::size_t best_segment = 0U;
  double best_x = 0.0;
  double best_y = 0.0;
  for (std::size_t i = 0; i + 1U < count; ++i) {
    const double ax = path_xy[2U * i];
    const double ay = path_xy[2U * i + 1U];
    const double dx = path_xy[2U * (i + 1U)] - ax;
    const double dy = path_xy[2U * (i + 1U) + 1U] - ay;
    const double length = dx * dx + dy * dy;
    const double t = length > 0.0
        ? std::max(0.0, std::min(1.0,
            ((robot_x - ax) * dx + (robot_y - ay) * dy) / length))
        : 0.0;
    const double x = ax + t * dx;
    const double y = ay + t * dy;
    const double distance = (x - robot_x) * (x - robot_x) +
                            (y - robot_y) * (y - robot_y);
    if (!has_best || distance < best_distance) {
      has_best = true;
      best_distance = distance;
      best_segment = i;
      best_x = x;
      best_y = y;
    }
  }
  std::vector<double> result;
  result.reserve(2U * (count + 1U));
  result.push_back(robot_x);
  result.push_back(robot_y);
  if (!has_best) {
    result.insert(result.end(), path_xy.begin(), path_xy.end());
    return result;
  }
  result.push_back(best_x);
  result.push_back(best_y);
  for (std::size_t i = best_segment + 1U; i < count; ++i) {
    result.push_back(path_xy[2U * i]);
    result.push_back(path_xy[2U * i + 1U]);
  }
  return result;
}

std::vector<double> path_samples(
    const std::vector<double>& route_xy, const double reach,
    const double half_length, const double half_width,
    const double clearance_margin) {
  if (route_xy.size() % 2U != 0U || !std::isfinite(reach) ||
      !std::all_of(route_xy.begin(), route_xy.end(),
                   [](const double value) { return std::isfinite(value); }) ||
      !std::isfinite(half_length) || !std::isfinite(half_width) ||
      !std::isfinite(clearance_margin) || half_length < 0.0 ||
      half_width < 0.0 || clearance_margin <= 0.0) {
    throw std::invalid_argument("finite route and positive sampling profile required");
  }
  const std::size_t count = route_xy.size() / 2U;
  const double step = std::min(0.025, clearance_margin / 2.0);
  const double radius = std::hypot(half_length, half_width);
  double distance = 0.0;
  bool has_previous_heading = false;
  double previous_heading = 0.0;
  std::vector<double> result;
  const auto append = [&result](const double d, const double x,
                                const double y, const double heading,
                                const double error) {
    result.push_back(d);
    result.push_back(x);
    result.push_back(y);
    result.push_back(heading);
    result.push_back(error);
  };
  for (std::size_t i = 0; i + 1U < count; ++i) {
    const double ax = route_xy[2U * i];
    const double ay = route_xy[2U * i + 1U];
    const double bx = route_xy[2U * (i + 1U)];
    const double by = route_xy[2U * (i + 1U) + 1U];
    const double length = std::hypot(bx - ax, by - ay);
    if (length < 1e-9) continue;
    const double heading = std::atan2(by - ay, bx - ax);
    if (has_previous_heading) {
      const double turn = std::remainder(heading - previous_heading,
                                         2.0 * std::acos(-1.0));
      const int samples = std::max(1, static_cast<int>(std::ceil(
          std::abs(turn) / 0.05)));
      for (int j = 0; j <= samples; ++j) {
        append(distance, ax, ay,
               previous_heading + turn * static_cast<double>(j) / samples,
               radius * std::abs(turn) / (2.0 * samples));
      }
    }
    const double travel = std::min(length, std::max(0.0, reach - distance));
    const int samples = std::max(1, static_cast<int>(std::ceil(travel / step)));
    for (int j = 0; j <= samples; ++j) {
      const double d = travel * static_cast<double>(j) / samples;
      append(distance + d, ax + (bx - ax) * d / length,
             ay + (by - ay) * d / length, heading,
             travel / (2.0 * samples));
    }
    distance += length;
    if (distance > reach) break;
    previous_heading = heading;
    has_previous_heading = true;
  }
  return result;
}

CommandRestriction::CommandRestriction()
    : output_{0.0, 0.0, 0.0}, recovering_(true) {}

void CommandRestriction::reset() {
  output_ = {0.0, 0.0, 0.0};
  recovering_ = true;
}

void CommandRestriction::set_state(const std::vector<double>& output,
                                   const bool recovering) {
  if (output.size() != 3U) {
    throw std::invalid_argument("restriction output must contain vx, vy, wz");
  }
  output_ = output;
  recovering_ = recovering;
}

std::vector<double> CommandRestriction::output() const { return output_; }

std::vector<double> CommandRestriction::apply(
    const std::vector<double>& command, const double cap,
    const double angular_cap, const bool stop, const double dt,
    const double max_acceleration, const double max_angular_acceleration,
    const bool allow_zero_dt) {
  require_command(command);
  if (stop || !std::isfinite(cap) || !std::isfinite(angular_cap) ||
      !std::isfinite(dt) || !std::isfinite(max_acceleration) ||
      !std::isfinite(max_angular_acceleration) ||
      !std::all_of(command.begin(), command.end(),
                   [](const double value) { return std::isfinite(value); })) {
    reset();
    return output_;
  }
  if (cap < 0.0 || angular_cap < 0.0 || dt < 0.0 ||
      (dt == 0.0 && !allow_zero_dt)) {
    reset();
    return output_;
  }

  const double speed = std::hypot(command[0], command[1]);
  const double angular = std::abs(command[2]);
  const double linear_ratio = speed > 0.0 ? cap / speed : 1.0;
  const double angular_ratio = angular > 0.0 ? angular_cap / angular : 1.0;
  const double ratio = std::min({1.0, linear_ratio, angular_ratio});
  std::vector<double> target{command[0] * ratio, command[1] * ratio,
                             command[2] * ratio};
  const bool limited = ratio < 1.0 - 1e-12;
  if (recovering_ || limited) {
    const double step = max_acceleration * std::min(dt, 0.05);
    const double old_speed = std::hypot(output_[0], output_[1]);
    const double target_speed = std::hypot(target[0], target[1]);
    double gain = std::min(1.0, (old_speed + step) /
                                      std::max(target_speed, 1e-12));
    double angular_gain = std::min(
        1.0, (std::abs(output_[2]) +
              max_angular_acceleration * std::min(dt, 0.05)) /
                 std::max(std::abs(target[2]), 1e-12));
    if (allow_zero_dt && dt == 0.0) {
      if (target_speed == 0.0) {
        gain = 1.0;
      }
      if (target[2] == 0.0) {
        angular_gain = 1.0;
      }
    }
    gain = std::min(gain, angular_gain);
    for (double& value : target) {
      value *= gain;
    }
    recovering_ = gain < 1.0 - 1e-12 || limited;
  }
  output_ = target;
  return output_;
}

}  // namespace astribot::navigation
