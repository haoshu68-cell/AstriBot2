#include "astribot_s1_manipulation/controller_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <joint_trajectory_controller/trajectory.hpp>

namespace astribot_s1_manipulation {
namespace {
using Point = trajectory_msgs::msg::JointTrajectoryPoint;
using Hull = std::vector<std::vector<double>>;

Hull positionHull(const Point& a, const Point& b, double duration) {
  const bool velocity = !a.velocities.empty() && !b.velocities.empty();
  const bool acceleration = velocity && !a.accelerations.empty() && !b.accelerations.empty();
  const std::size_t degree = acceleration ? 5 : velocity ? 3 : 1;
  Hull hull(a.positions.size(), std::vector<double>(degree + 1));
  for (std::size_t j = 0; j < hull.size(); ++j) {
    auto& h = hull[j];
    h.front() = a.positions[j];
    h.back() = b.positions[j];
    if (velocity) {
      h[1] = h[0] + a.velocities[j] * duration / degree;
      h[degree - 1] = h[degree] - b.velocities[j] * duration / degree;
    }
    if (acceleration) {
      h[2] = 2 * h[1] - h[0] + a.accelerations[j] * duration * duration / 20;
      h[3] = 2 * h[4] - h[5] + b.accelerations[j] * duration * duration / 20;
    }
  }
  return hull;
}

std::pair<Hull, Hull> split(const Hull& hull, double ratio) {
  Hull left = hull, right = hull;
  for (std::size_t j = 0; j < hull.size(); ++j) {
    auto values = hull[j];
    const auto n = values.size();
    for (std::size_t level = 1; level < n; ++level) {
      for (std::size_t i = 0; i < n - level; ++i)
        values[i] = (1 - ratio) * values[i] + ratio * values[i + 1];
      left[j][level] = values.front();
      right[j][n - level - 1] = values[n - level - 1];
    }
  }
  return {std::move(left), std::move(right)};
}
}  // namespace

bool sampleControllerTrajectory(
  const robot_trajectory::RobotTrajectory& trajectory, double maximum_joint_span,
  std::size_t maximum_samples, std::vector<moveit::core::RobotState>& states,
  std::string& error) {
  states.clear();
  error.clear();
  if (!std::isfinite(maximum_joint_span) || maximum_joint_span <= 0 || maximum_samples == 0) {
    error = "INVALID_CONTROLLER_SAMPLING_LIMIT";
    return false;
  }
  if (!trajectory.getGroup() || trajectory.getWayPointCount() == 0) {
    error = "EMPTY_EXTERNAL_TRAJECTORY";
    return false;
  }
  moveit_msgs::msg::RobotTrajectory message;
  trajectory.getRobotTrajectoryMsg(message);
  const auto& path = message.joint_trajectory;
  // This API validates parameterized plans beginning at their explicit first
  // state. The controller's measured-state-to-first-point interval is a separate
  // execution admission check, and must not be silently guessed here.
  if (rclcpp::Duration(path.points.front().time_from_start).nanoseconds() != 0) {
    error = "CONTROLLER_TRAJECTORY_START_NOT_ZERO";
    return false;
  }
  int64_t previous = -1;
  for (const auto& point : path.points) {
    const auto time = rclcpp::Duration(point.time_from_start).nanoseconds();
    if (time < 0 || time <= previous) {
      error = "CONTROLLER_TRAJECTORY_TIME_NOT_INCREASING";
      return false;
    }
    previous = time;
    for (const auto* values : {&point.positions, &point.velocities, &point.accelerations})
      for (double value : *values)
        if (!std::isfinite(value)) {
          error = "INVALID_EXTERNAL_JOINT_VALUE";
          return false;
        }
  }
  states.push_back(trajectory.getFirstWayPoint());
  joint_trajectory_controller::Trajectory sampler;
  struct Interval { int64_t begin, end; Hull hull; };
  for (std::size_t index = 1; index < path.points.size(); ++index) {
    const auto& a = path.points[index - 1];
    const auto& b = path.points[index];
    const auto begin = rclcpp::Duration(a.time_from_start).nanoseconds();
    const auto end = rclcpp::Duration(b.time_from_start).nanoseconds();
    std::vector<Interval> pending;
    pending.push_back({begin, end, positionHull(a, b, (end - begin) * 1e-9)});
    while (!pending.empty()) {
      auto interval = std::move(pending.back());
      pending.pop_back();
      double span = 0.;
      for (const auto& values : interval.hull) {
        const auto bounds = std::minmax_element(values.begin(), values.end());
        const double width = *bounds.second - *bounds.first;
        if (!std::isfinite(width)) {
          error = "INVALID_CONTROLLER_SPLINE";
          return false;
        }
        span = std::max(span, width);
      }
      if (span > maximum_joint_span) {
        if (interval.end - interval.begin < 2 || states.size() + pending.size() + 2 > maximum_samples) {
          error = "EXTERNAL_PATH_TOO_LARGE";
          return false;
        }
        const auto middle = interval.begin + (interval.end - interval.begin) / 2;
        const double ratio = double(middle - interval.begin) / double(interval.end - interval.begin);
        auto halves = split(interval.hull, ratio);
        pending.push_back({middle, interval.end, std::move(halves.second)});
        pending.push_back({interval.begin, middle, std::move(halves.first)});
        continue;
      }
      if (states.size() == maximum_samples) {
        error = "EXTERNAL_PATH_TOO_LARGE";
        return false;
      }
      Point sample;
      sampler.interpolate_between_points(rclcpp::Time(begin), a, rclcpp::Time(end), b,
        rclcpp::Time(interval.end), sample);
      auto state = trajectory.getWayPoint(index - 1);
      state.setVariablePositions(path.joint_names, sample.positions);
      state.update();
      states.push_back(std::move(state));
    }
  }
  return true;
}
}  // namespace astribot_s1_manipulation
