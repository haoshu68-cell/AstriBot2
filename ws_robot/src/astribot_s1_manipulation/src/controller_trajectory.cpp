#include "astribot_s1_manipulation/controller_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <chrono>
#include <map>
#include <set>
#include <moveit/robot_model/revolute_joint_model.h>
#include <geometric_shapes/shape_operations.h>
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
bool controllerMessage(const robot_trajectory::RobotTrajectory& trajectory,
  moveit_msgs::msg::RobotTrajectory& message, std::string& error) {
  if (!trajectory.getGroup() || trajectory.getWayPointCount() == 0) {
    error = "EMPTY_EXTERNAL_TRAJECTORY";
    return false;
  }
  trajectory.getRobotTrajectoryMsg(message);
  const auto& path = message.joint_trajectory;
  if (path.points.empty() || !message.multi_dof_joint_trajectory.points.empty()) {
    error = "UNSUPPORTED_CONTROLLER_JOINTS";
    return false;
  }
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
  return true;
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
  moveit_msgs::msg::RobotTrajectory message;
  if (!controllerMessage(trajectory, message, error)) return false;
  const auto& path = message.joint_trajectory;
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

namespace {
ControllerSweepResult checkSpline(
  const planning_scene::PlanningSceneConstPtr& scene,
  const moveit::core::RobotState& initial,
  const trajectory_msgs::msg::JointTrajectory& path,
  double clearance_m, std::size_t maximum_states, double budget_seconds) {
  ControllerSweepResult result;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(budget_seconds);
  if(!std::isfinite(clearance_m)||clearance_m<0||!std::isfinite(budget_seconds)||budget_seconds<=0||maximum_states==0) {
    result.reason="INVALID_SWEEP_BUDGET";return result;
  }
  if(scene->getCollisionDetectorName()!="FCL") {
    result.reason="UNSUPPORTED_SWEEP_COLLISION_BACKEND";return result;
  }

  // Bound every collision body's extent from its link origin. The ancestor
  // walk below adds fixed offsets and the full possible prismatic extension.
  std::vector<std::pair<const moveit::core::LinkModel*,double>> bodies;
  auto body=[&](const moveit::core::LinkModel* link,const auto& shapes,const auto& poses,double scale,double padding) {
    double extent=0;
    for(std::size_t i=0;i<shapes.size();++i) {
      const auto type=shapes[i]->type;
      if(type!=shapes::SPHERE&&type!=shapes::BOX&&type!=shapes::CYLINDER&&type!=shapes::CONE&&type!=shapes::MESH) {
        result.reason="UNSUPPORTED_SWEEP_BODY_SHAPE";return false;
      }
      Eigen::Vector3d center;double radius;
      shapes::computeShapeBoundingSphere(shapes[i].get(),center,radius);
      const double size=poses[i].translation().norm()+scale*(center.norm()+radius)+padding;
      if(!std::isfinite(size)){result.reason="INVALID_SWEEP_BODY_EXTENT";return false;}
      extent=std::max(extent,size);
    }
    if(!std::isfinite(extent)){result.reason="INVALID_SWEEP_BODY_EXTENT";return false;}
    if(!shapes.empty())bodies.emplace_back(link,extent);
    return true;
  };
  for(const auto* link:initial.getRobotModel()->getLinkModels())
    if(!body(link,link->getShapes(),link->getCollisionOriginTransforms(),
        std::max(1.,scene->getCollisionEnv()->getLinkScale(link->getName())),
        std::max(0.,scene->getCollisionEnv()->getLinkPadding(link->getName()))))return result;
  std::vector<const moveit::core::AttachedBody*> attached;
  initial.getAttachedBodies(attached);
  for(const auto* item:attached)
    if(!body(item->getAttachedLink(),item->getShapes(),item->getShapePosesInLinkFrame(),1.,0.))return result;
  const auto& acm=scene->getAllowedCollisionMatrix();
  double certified=std::numeric_limits<double>::max();
  auto distances=[&](moveit::core::RobotState& state,double displacement) {
    state.update();++result.checked_states;
    const double world=scene->getCollisionEnv()->distanceRobot(state,acm);
    const double self=scene->getCollisionEnvUnpadded()->distanceSelf(state,acm);
    if(std::chrono::steady_clock::now()>=deadline){result.reason="SWEEP_BUDGET_EXHAUSTED";return -1;}
    if(std::isnan(world)||std::isnan(self)){result.reason="SWEEP_DISTANCE_UNAVAILABLE";return -1;}
    if(world<=clearance_m||self<=clearance_m){result.verdict=SweepVerdict::RISK;result.reason="SWEEP_CLEARANCE_VIOLATION";return -1;}
    const double bound=std::min(world-displacement,self-2*displacement);
    if(bound>clearance_m) {certified=std::min(certified,bound);return 1;}
    return 0;
  };
  auto first=initial;
  if(distances(first,0)<0)return result;
  joint_trajectory_controller::Trajectory sampler;
  struct Interval {int64_t begin,end;Hull hull;};
  for(std::size_t segment=1;segment<path.points.size();++segment) {
    const auto& a=path.points[segment-1];const auto& b=path.points[segment];
    const auto begin=rclcpp::Duration(a.time_from_start).nanoseconds(),end=rclcpp::Duration(b.time_from_start).nanoseconds();
    std::vector<Interval> pending{{begin,end,positionHull(a,b,(end-begin)*1e-9)}};
    while(!pending.empty()) {
      if(std::chrono::steady_clock::now()>=deadline||result.checked_states>=maximum_states) {
        result.reason="SWEEP_BUDGET_EXHAUSTED";return result;
      }
      auto interval=std::move(pending.back());pending.pop_back();
      std::map<std::string,double> spans;
      for(std::size_t j=0;j<interval.hull.size();++j) {
        const auto range=std::minmax_element(interval.hull[j].begin(),interval.hull[j].end());
        const double span=*range.second-*range.first;
        if(!std::isfinite(span)){result.reason="INVALID_CONTROLLER_SPLINE";return result;}
        spans[path.joint_names[j]]=span;
      }
      Point point;sampler.interpolate_between_points(rclcpp::Time(begin),a,rclcpp::Time(end),b,rclcpp::Time(interval.end),point);
      auto state=initial;state.setVariablePositions(path.joint_names,point.positions);state.update();
      double displacement=0;
      for(const auto& item:bodies) {
        double radius=item.second,motion=0;
        for(auto* link=item.first;link;link=link->getParentLinkModel()) {
          const auto* joint=link->getParentJointModel();
          const auto* source=joint;double factor=1.;
          while(source->getMimic()){factor*=source->getMimicFactor();source=source->getMimic();}
          const auto found=spans.find(source->getName());
          const double span=found==spans.end()?0:std::abs(factor)*found->second;
          if(span>0) {
            if(joint->getType()==moveit::core::JointModel::REVOLUTE)motion+=radius*span;
            else if(joint->getType()==moveit::core::JointModel::PRISMATIC)motion+=span;
            else {result.reason="UNSUPPORTED_SWEEP_JOINT";return result;}
          }
          radius+=link->getJointOriginTransform().translation().norm()+state.getJointTransform(joint).translation().norm();
          if(joint->getType()==moveit::core::JointModel::PRISMATIC)radius+=span;
        }
        displacement=std::max(displacement,motion);
      }
      const int checked=distances(state,displacement);
      if(checked<0)return result;
      if(checked==0) {
        if(interval.end-interval.begin<2){result.reason="SWEEP_INTERVAL_UNRESOLVED";return result;}
        const auto middle=interval.begin+(interval.end-interval.begin)/2;
        auto halves=split(interval.hull,double(middle-interval.begin)/double(interval.end-interval.begin));
        pending.push_back({middle,interval.end,std::move(halves.second)});
        pending.push_back({interval.begin,middle,std::move(halves.first)});
      }
    }
  }
  if(std::chrono::steady_clock::now()>=deadline){result.reason="SWEEP_BUDGET_EXHAUSTED";return result;}
  result.verdict=SweepVerdict::CLEAR;result.reason="FIXED_SCENE_SPLINE_CLEAR";
  result.clearance_lower_bound=certified;
  return result;
}
}  // namespace

ControllerSweepResult checkControllerSweep(
  const planning_scene::PlanningSceneConstPtr& scene,
  const robot_trajectory::RobotTrajectory& trajectory,
  double clearance_m, std::size_t maximum_states, double budget_seconds) {
  ControllerSweepResult result;
  moveit_msgs::msg::RobotTrajectory message;
  if (!controllerMessage(trajectory, message, result.reason)) return result;
  return checkSpline(scene, trajectory.getFirstWayPoint(), message.joint_trajectory,
    clearance_m, maximum_states, budget_seconds);
}

ControllerSweepResult checkControllerCommand(
  const planning_scene::PlanningSceneConstPtr& scene,
  const moveit::core::RobotState& reference,
  const trajectory_msgs::msg::JointTrajectory& command,
  const Point& before, const rclcpp::Time& started_at,
  double clearance_m, std::size_t maximum_states, double budget_seconds) {
  ControllerSweepResult result;
  const auto began = std::chrono::steady_clock::now();
  if (!std::isfinite(clearance_m) || clearance_m < 0 || !std::isfinite(budget_seconds) ||
      budget_seconds <= 0 || maximum_states == 0) {
    result.reason = "INVALID_SWEEP_BUDGET"; return result;
  }
  const auto count = command.joint_names.size();
  if (count == 0 || command.points.empty()) {
    result.reason = "EMPTY_CONTROLLER_COMMAND"; return result;
  }
  std::set<std::string> names;
  std::vector<bool> wrap;
  for (const auto& name : command.joint_names) {
    const auto* joint = reference.getRobotModel()->getJointModel(name);
    if (!names.insert(name).second || !joint || joint->getVariableCount() != 1 || joint->getMimic()) {
      result.reason = "INVALID_CONTROLLER_COMMAND_JOINT"; return result;
    }
    wrap.push_back(joint->getType() == moveit::core::JointModel::REVOLUTE &&
      static_cast<const moveit::core::RevoluteJointModel*>(joint)->isContinuous());
  }
  // This is the ROS command boundary. Derivative-only input is not part of the
  // executor's contract; never let JTC infer a different position trajectory.
  auto valid = [count](const Point& point) {
    if (point.positions.size() != count || !point.effort.empty() ||
        (!point.velocities.empty() && point.velocities.size() != count) ||
        (!point.accelerations.empty() && (point.accelerations.size() != count || point.velocities.empty())))
      return false;
    for (const auto* values : {&point.positions, &point.velocities, &point.accelerations})
      for (double value : *values) if (!std::isfinite(value)) return false;
    return true;
  };
  if (!valid(before)) { result.reason = "INVALID_CONTROLLER_START_STATE"; return result; }
  int64_t previous = -1;
  for (const auto& point : command.points) {
    const auto& time = point.time_from_start;
    const int64_t stamp = int64_t(time.sec) * 1000000000LL + time.nanosec;
    if (!valid(point) || time.sec < 0 || time.nanosec >= 1000000000u || stamp <= previous) {
      result.reason = "INVALID_CONTROLLER_COMMAND_POINT"; return result;
    }
    previous = stamp;
  }
  if (started_at.nanoseconds() <= 0 || command.header.stamp.sec < 0 ||
      command.header.stamp.nanosec >= 1000000000u) {
    result.reason = "INVALID_CONTROLLER_START_TIME"; return result;
  }
  const int64_t header = int64_t(command.header.stamp.sec) * 1000000000LL + command.header.stamp.nanosec;
  const int64_t offset = header == 0 ? 0 : header - started_at.nanoseconds();
  const int64_t first = offset + rclcpp::Duration(command.points.front().time_from_start).nanoseconds();
  if (first < 0) { result.reason = "CONTROLLER_START_AFTER_FIRST_POINT"; return result; }
  Point start = before;
  // Match Trajectory::set_point_before_trajectory_msg in the installed JTC.
  if (start.velocities.empty() && !command.points.front().velocities.empty()) start.velocities.assign(count, 0.);
  if (start.accelerations.empty() && !command.points.front().accelerations.empty()) start.accelerations.assign(count, 0.);
  joint_trajectory_controller::wraparound_joint(start.positions, command.points.front().positions, wrap);
  auto initial = reference;
  initial.setVariablePositions(command.joint_names, start.positions);
  initial.update();
  auto path = command;
  path.header.stamp = builtin_interfaces::msg::Time();
  for (auto& point : path.points)
    point.time_from_start = rclcpp::Duration::from_nanoseconds(
      offset + rclcpp::Duration(point.time_from_start).nanoseconds());
  if (first > 0) {
    start.time_from_start = rclcpp::Duration(0, 0);
    path.points.insert(path.points.begin(), start);
  } else {
    for (std::size_t i = 0; i < count; ++i)
      if (std::abs(start.positions[i] - path.points.front().positions[i]) > 1e-12) {
        result.reason = "CONTROLLER_START_POSITION_JUMP"; return result;
      }
  }
  const double remaining = budget_seconds -
    std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
  if (remaining <= 0) { result.reason = "SWEEP_BUDGET_EXHAUSTED"; return result; }
  return checkSpline(scene, initial, path, clearance_m, maximum_states, remaining);
}
}  // namespace astribot_s1_manipulation
