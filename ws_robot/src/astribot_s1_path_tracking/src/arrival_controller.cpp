#include "rclcpp/create_publisher.hpp"
#include "astribot_s1_path_tracking/path_quality.hpp"
// Copyright 2026 Astribot
#include "astribot_s1_path_tracking/arrival_controller.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace astribot_s1_path_tracking
{
namespace
{
bool validPose(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.position.z) && std::isfinite(norm) && std::abs(norm - 1.0) < 0.01;
}
double distance(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return std::hypot(a.position.x-b.position.x, a.position.y-b.position.y);
}
double yawError(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return shortestAngularDiff(tf2::getYaw(a.orientation), tf2::getYaw(b.orientation));
}
double parameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, double value)
{
  nav2_util::declare_parameter_if_not_declared(node, name, rclcpp::ParameterValue(value));
  const double result = node->get_parameter(name).as_double();
  if (!std::isfinite(result) || result <= 0.0) {
    throw std::invalid_argument(name + " must be finite and positive");
  }
  return result;
}
}

void ArrivalGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
  auto node = parent.lock();
  if (!node) {throw std::runtime_error("ArrivalGoalChecker: expired parent");}
  clock_ = node->get_clock();
  xy_ = parameter(node, name + ".xy_goal_tolerance", xy_);
  yaw_ = parameter(node, name + ".yaw_goal_tolerance", yaw_);
  if (xy_ > 0.03 || yaw_ > 0.02617993877991494) {
    throw std::invalid_argument("Arrival tolerances must be <= 0.03m and <= 1.5 degrees");
  }
  reset();
}
void ArrivalGoalChecker::report(bool ready)
{
  ready_ = ready;
  reported_ = clock_->now();
}
bool ArrivalGoalChecker::isGoalReached(const geometry_msgs::msg::Pose &,
  const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Twist & velocity)
{
  const double age = (clock_->now() - reported_).seconds();
  return ready_ && age >= 0.0 && age <= 0.2 &&
         std::hypot(velocity.linear.x, velocity.linear.y) <= 0.01 &&
         std::abs(velocity.angular.z) <= 0.01;
}
bool ArrivalGoalChecker::getTolerances(geometry_msgs::msg::Pose & pose,
  geometry_msgs::msg::Twist & velocity)
{
  pose = geometry_msgs::msg::Pose();
  pose.position.x = pose.position.y = xy_;
  pose.position.z = std::numeric_limits<double>::lowest();
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_);
  pose.orientation = tf2::toMsg(q);
  velocity = geometry_msgs::msg::Twist();
  velocity.linear.x = velocity.linear.y = velocity.angular.z = 0.01;
  return true;
}

void ArrivalController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  auto node = parent.lock();
  if (!node) {throw std::runtime_error("ArrivalController: expired parent");}
  phase_pub_ = rclcpp::create_publisher<std_msgs::msg::String>(node, "path_tracking/phase", rclcpp::SensorDataQoS());
  logger_ = node->get_logger();
  clock_ = node->get_clock(); tf_ = tf; costmap_ = costmap;
  const std::string p = name + ".arrival.";
  nav2_util::declare_parameter_if_not_declared(node, "navigation_policy_enabled", rclcpp::ParameterValue(false));
  policy_enabled_=node->get_parameter("navigation_policy_enabled").as_bool();
  if (policy_enabled_) {
    active_path_pub_=rclcpp::create_publisher<nav_msgs::msg::Path>(node,
      "path_tracking/active_path",rclcpp::QoS(1).transient_local());
    policy_sub_=node->create_subscription<PolicyLease::Message>("navigation_policy/constraint",10,
      [this](PolicyLease::Message::ConstSharedPtr m) {policy_lease_.receive(*m);});
  }
  capture_ = parameter(node, p + "capture_radius", capture_);
  pose_timeout_ = parameter(node, p + "pose_timeout", pose_timeout_);
  refine_timeout_ = parameter(node, p + "refine_timeout", refine_timeout_);
  total_timeout_ = parameter(node, p + "total_timeout", total_timeout_);
  progress_timeout_ = parameter(node, p + "progress_timeout", progress_timeout_);
  settle_time_ = parameter(node, p + "settle_time", settle_time_);
  kp_xy_ = parameter(node, p + "kp_xy", kp_xy_);
  kp_yaw_ = parameter(node, p + "kp_yaw", kp_yaw_);
  max_v_ = parameter(node, p + "max_linear_speed", max_v_);
  max_w_ = parameter(node, p + "max_angular_speed", max_w_);
  if (capture_ <= 0.03 || max_v_ > 0.15 || max_w_ > 0.3) {
    throw std::invalid_argument("Arrival: capture_radius > 0.03, max speeds <= 0.15m/s, 0.3rad/s");
  }
  nav2_util::declare_parameter_if_not_declared(node, p + "source", rclcpp::ParameterValue("nav2_pose"));
  source_ = node->get_parameter(p + "source").as_string();
  if (source_ != "nav2_pose" && source_ != "slam_pose" && source_ != "vision" && source_ != "mark") {
    throw std::invalid_argument("Arrival: unknown localization source " + source_);
  }
  const std::string default_topic = source_ == "slam_pose" ? "/slam_toolbox/pose" :
    "/arrival/" + source_ + "/pose";
  nav2_util::declare_parameter_if_not_declared(node, p + "pose_topic", rclcpp::ParameterValue(default_topic));
  const auto topic = node->get_parameter(p + "pose_topic").as_string();
  if (source_ == "slam_pose") {
    slam_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> guard(observation_mutex_);
        observation_.header = msg->header; observation_.pose = msg->pose.pose; observed_ = true;
      });
  } else if (source_ != "nav2_pose") {
    pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      topic, rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> guard(observation_mutex_);
        observation_ = *msg; observed_ = true;
      });
  }
  ThreePhaseController::configure(parent, name, tf, costmap);
}
void ArrivalController::publishPhase(const char * phase)
{
  if (phase_pub_) {std_msgs::msg::String msg; msg.data = phase; phase_pub_->publish(msg);}
}
void ArrivalController::resetAttempt()
{
  started_ = refining_ = holding_ = completion_logged_ = false;
  policy_paused_=false;policy_tick_=-1;
  error_.clear();
}
void ArrivalController::deactivate()
{
  resetAttempt();
  {std::lock_guard<std::mutex> guard(observation_mutex_); observed_ = false;}
  ThreePhaseController::deactivate();
}
void ArrivalController::cleanup()
{
  tracking_path_ = nav_msgs::msg::Path();
  phase_pub_.reset();
  policy_sub_.reset();
  active_path_pub_.reset();
  pose_sub_.reset(); slam_sub_.reset(); checker_ = nullptr; has_goal_ = false;
  {std::lock_guard<std::mutex> guard(observation_mutex_); observed_ = false;}
  last_tick_ = -1; speed_scale_ = 1.0;
  resetAttempt();
  ThreePhaseController::cleanup();
}
void ArrivalController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty() || path.header.frame_id.empty()) {
    fail("INVALID_PATH: empty path or frame");
  }
  for (const auto & pose : path.poses) {
    if (!validPose(pose.pose) || (!pose.header.frame_id.empty() &&
      pose.header.frame_id != path.header.frame_id)) {fail("INVALID_PATH: invalid pose/frame");}
  }
  auto goal = path.poses.back();
  goal.header.frame_id = path.header.frame_id;
  goal.header.stamp = builtin_interfaces::msg::Time();  // fixed world target, latest TF
  if (last_tick_ >= 0 && clock_->now().seconds() - last_tick_ > 0.5) {resetAttempt();}
  if (!has_goal_ || goal.header.frame_id != goal_.header.frame_id ||
    distance(goal.pose, goal_.pose) > 1e-4 || std::abs(yawError(goal.pose, goal_.pose)) > 1e-4)
  {
    resetAttempt();
  }
  goal_ = goal; has_goal_ = true;
  tracking_path_ = path;
  if (active_path_pub_) {active_path_pub_->publish(path);}
  ThreePhaseController::setPlan(path);
}
void ArrivalController::setSpeedLimit(const double & limit, const bool & percentage)
{
  if (!std::isfinite(limit) || limit < 0 || (percentage && limit > 100)) {
    fail("INVALID_SPEED_LIMIT");
  }
  speed_scale_ = limit == 0 ? 1.0 : std::min(1.0, percentage ? limit / 100.0 : limit / max_v_);
  ThreePhaseController::setSpeedLimit(limit, percentage);
}
[[noreturn]] void ArrivalController::fail(const std::string & reason)
{
  if (error_ != reason) {RCLCPP_ERROR(logger_, "PATH_TRACKING/%s", reason.c_str());}
  error_ = reason;
  throw nav2_core::PlannerException("PATH_TRACKING/" + reason);
}
geometry_msgs::msg::PoseStamped ArrivalController::inFrame(
  const geometry_msgs::msg::PoseStamped & pose, const std::string & frame)
{
  if (pose.header.frame_id.empty() || !validPose(pose.pose)) {fail("INVALID_POSE");}
  try {
    return pose.header.frame_id == frame ? pose : tf_->transform(pose, frame, tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException & e) {fail(std::string("TF_UNAVAILABLE: ") + e.what());}
}

bool ArrivalController::safeCommand(const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & command, const geometry_msgs::msg::Twist & measured)
{
  if (!costmap_->isCurrent()) {return false;}
  auto local = inFrame(pose, costmap_->getGlobalFrameID());
  auto * map = costmap_->getCostmap();
  auto footprint = costmap_->getRobotFootprint();
  if (footprint.size() < 3) {return false;}
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision(map);
  for (const auto & twist : {command, measured}) {
    double x = local.pose.position.x, y = local.pose.position.y;
    double yaw = tf2::getYaw(local.pose.orientation);
    const double radius = costmap_->getLayeredCostmap()->getCircumscribedRadius();
    const double sweep = std::hypot(twist.linear.x, twist.linear.y) + radius*std::abs(twist.angular.z);
    const int steps = std::max(20, static_cast<int>(std::ceil(sweep / (map->getResolution()*0.5))));
    for (int i = 0; i <= steps; ++i) {
      unsigned int mx, my;
      const double cost = collision.footprintCostAtPose(x, y, yaw, footprint);
      if (!map->worldToMap(x, y, mx, my) || cost < 0 || cost >= 254 || map->getCost(mx, my) >= 253) {
        return false;
      }
      x += (std::cos(yaw)*twist.linear.x - std::sin(yaw)*twist.linear.y)/steps;
      y += (std::sin(yaw)*twist.linear.x + std::cos(yaw)*twist.linear.y)/steps;
      yaw += twist.angular.z/steps;
    }
  }
  return true;
}

double ArrivalController::localSharpPathLimit(const geometry_msgs::msg::PoseStamped & pose) const
{
  if (tracking_path_.poses.size()<3) {return std::numeric_limits<double>::infinity();}
  size_t first=0;double best=std::numeric_limits<double>::infinity();
  for (size_t i=0;i<tracking_path_.poses.size();++i) {
    const double d=pathDistance(pose,tracking_path_.poses[i]);
    if (d<best) {best=d;first=i;}
  }
  nav_msgs::msg::Path window;window.header=tracking_path_.header;
  double length=0;
  for(size_t i=first;i<tracking_path_.poses.size();++i) {
    if(i>first) {length+=pathDistance(tracking_path_.poses[i-1],tracking_path_.poses[i]);}
    window.poses.push_back(tracking_path_.poses[i]);
    if(length>=1.0) {break;}
  }
  return sharpPathSpeedLimit(travellingPathQuality(window));
}
geometry_msgs::msg::TwistStamped ArrivalController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * checker)
{
  auto * arrival = dynamic_cast<ArrivalGoalChecker *>(checker);
  if (!arrival) {fail("CHECKER_MISMATCH: ArrivalGoalChecker required");}
  arrival->report(false);  // invalidate before every possible return/exception
  if (checker_ != arrival || generation_ != arrival->generation()) {
    holding_ = false; checker_ = arrival; generation_ = arrival->generation();
  }
  last_tick_ = clock_->now().seconds();
  if (!error_.empty()) {fail(error_);}
  if (!has_goal_) {fail("INVALID_PATH: no target");}
  if (!std::isfinite(velocity.linear.x) || !std::isfinite(velocity.linear.y) ||
    !std::isfinite(velocity.angular.z)) {fail("INVALID_VELOCITY");}
  const double now = clock_->now().seconds();
  const double navigation_age = now - rclcpp::Time(pose.header.stamp).seconds();
  if (navigation_age < -0.05 || navigation_age > pose_timeout_) {fail("NAVIGATION_POSE_STALE");}
  auto current = inFrame(pose, goal_.header.frame_id);
  if (!started_) {
    started_ = true; started_at_ = progress_at_ = now; anchor_ = current;
  }
  if (now < started_at_ || now < progress_at_) {fail("CLOCK_JUMP");}
  if (policy_enabled_) {
    if (!policy_lease_.fresh(clock_->now())) {fail("POLICY_LEASE_EXPIRED");}
    if (policy_paused_ && policy_tick_>=0) {
      const double duration=std::max(0.0,now-policy_tick_);
      started_at_+=duration;progress_at_+=duration;
      if (refining_) {refine_at_+=duration;}
      if (holding_) {hold_at_+=duration;}
      accountPolicyPause(duration,clock_->now());
    }
    policy_tick_=now;policy_paused_=policy_lease_.held(clock_->now());
    if (policy_paused_) {
      publishPhase("POLICY_HOLD");
      geometry_msgs::msg::TwistStamped stopped;stopped.header=pose.header;return stopped;
    }
  }
  if (now - started_at_ > total_timeout_) {fail("GOAL_TIMEOUT");}
  if (!refining_ && distance(current.pose, goal_.pose) <= capture_) {
    refining_ = true; refine_at_ = progress_at_ = now;
    best_error_ = std::numeric_limits<double>::infinity();
    RCLCPP_INFO(logger_, "ARRIVAL_REFINING source=%s", source_.c_str());
  }
  if (!refining_) {
    if (distance(current.pose, anchor_.pose) >= 0.01 ||
      std::abs(yawError(current.pose, anchor_.pose)) >= 0.02)
    {anchor_ = current; progress_at_ = now;}
    if (now-progress_at_ > progress_timeout_) {fail("NO_MOTION_PROGRESS");}
    auto cmd = ThreePhaseController::computeVelocityCommands(current, velocity, checker);
    if (phase() == Phase::kFollow) {
      const double cap = localSharpPathLimit(current);
      const double speed = std::hypot(cmd.twist.linear.x,cmd.twist.linear.y);
      if (speed > cap) {
        const double ratio = cap/speed;
        cmd.twist.linear.x *= ratio;cmd.twist.linear.y *= ratio;cmd.twist.angular.z *= ratio;
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "PATH_QUALITY_SPEED_LIMIT %.3fm/s", cap);
      }
    }
    publishPhase(toString(phase()));
    return cmd;
  }
  publishPhase("REFINE");
  if (now-refine_at_ > refine_timeout_) {fail("REFINEMENT_TIMEOUT");}
  if (source_ != "nav2_pose") {
    geometry_msgs::msg::PoseStamped observation;
    {std::lock_guard<std::mutex> guard(observation_mutex_);
      if (!observed_) {fail("POSE_UNAVAILABLE: " + source_);}
      observation = observation_;
    }
    const double age = now - rclcpp::Time(observation.header.stamp).seconds();
    if (age < -0.05 || age > pose_timeout_ || rclcpp::Time(observation.header.stamp).nanoseconds() == 0) {
      fail("POSE_STALE: " + source_);
    }
    current = inFrame(observation, goal_.header.frame_id);
  }
  const double xy = distance(current.pose, goal_.pose);
  const double yaw = yawError(current.pose, goal_.pose);
  if (xy > capture_*2) {fail("POSE_DISAGREEMENT: outside refinement region");}
  const double metric = xy/arrival->xyTolerance() + std::abs(yaw)/arrival->yawTolerance();
  if (metric < best_error_ - 0.1) {best_error_ = metric; progress_at_ = now;}
  if (now-progress_at_ > progress_timeout_) {fail("REFINEMENT_NO_PROGRESS");}
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now(); cmd.header.frame_id = costmap_->getBaseFrameID();
  const bool within = xy <= arrival->xyTolerance() && std::abs(yaw) <= arrival->yawTolerance();
  const bool stopped = std::hypot(velocity.linear.x, velocity.linear.y) <= 0.01 &&
    std::abs(velocity.angular.z) <= 0.01;
  if (within && stopped) {
    if (!safeCommand(pose, cmd.twist, velocity)) {fail("REFINEMENT_BLOCKED: unsafe final footprint");}
    if (!holding_) {holding_ = true; hold_at_ = now;}
    const bool ready = now-hold_at_ >= settle_time_;
    arrival->report(ready);
    if (ready) {publishPhase("DONE");}
    if (ready && !completion_logged_) {
      completion_logged_ = true;
      RCLCPP_INFO(logger_, "ARRIVAL_REACHED source=%s xy=%.5fm yaw=%.5fdeg settled=%.2fs",
        source_.c_str(), xy, std::abs(yaw)*180.0/M_PI, now-hold_at_);
    }
    return cmd;
  }
  holding_ = false;
  const double heading = tf2::getYaw(current.pose.orientation);
  const double dx = goal_.pose.position.x-current.pose.position.x;
  const double dy = goal_.pose.position.y-current.pose.position.y;
  if (xy > arrival->xyTolerance()*0.7) {
    const double scale = std::min(kp_xy_, max_v_*speed_scale_/std::max(xy, 1e-9));
    cmd.twist.linear.x = scale*(std::cos(heading)*dx + std::sin(heading)*dy);
    cmd.twist.linear.y = scale*(-std::sin(heading)*dx + std::cos(heading)*dy);
  }
  if (std::abs(yaw) > arrival->yawTolerance()*0.7) {
    cmd.twist.angular.z = std::clamp(kp_yaw_*yaw, -max_w_*speed_scale_, max_w_*speed_scale_);
  }
  if (!safeCommand(pose, cmd.twist, velocity)) {fail("REFINEMENT_BLOCKED: unsafe footprint sweep");}
  return cmd;
}
}  // namespace astribot_s1_path_tracking
PLUGINLIB_EXPORT_CLASS(astribot_s1_path_tracking::ArrivalController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(astribot_s1_path_tracking::ArrivalGoalChecker, nav2_core::GoalChecker)
