#include "rclcpp/create_publisher.hpp"
#include "astribot_s1_path_tracking/path_quality.hpp"
// Copyright 2026 Astribot
#include "astribot_s1_path_tracking/arrival_controller.hpp"
#include "astribot_s1_path_tracking/corridor_refinement.hpp"
#include "astribot_s1_path_tracking/arrival_braking.hpp"

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
  const std::string & name, double value, bool allow_zero = false)
{
  nav2_util::declare_parameter_if_not_declared(node, name, rclcpp::ParameterValue(value));
  const double result = node->get_parameter(name).as_double();
  if (!std::isfinite(result) || (allow_zero ? result < 0.0 : result <= 0.0)) {
    throw std::invalid_argument(name + (allow_zero ? " must be finite and nonnegative" : " must be finite and positive"));
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
  stopped_v_ = parameter(node, name + ".stopped_linear_velocity", stopped_v_);
  stopped_w_ = parameter(node, name + ".stopped_angular_velocity", stopped_w_);
  reset();
}
void ArrivalGoalChecker::report(bool ready)
{
  ready_ = ready;
  reported_ = clock_->now();
}
bool ArrivalGoalChecker::isGoalReached(const geometry_msgs::msg::Pose &,
  const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Twist &)
{
  return ready_;
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
  velocity.linear.x = velocity.linear.y = stopped_v_;
  velocity.angular.z = stopped_w_;
  return true;
}

void ArrivalController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  auto node = parent.lock();
  if (!node) {throw std::runtime_error("ArrivalController: expired parent");}
  arrival_progress_ = ArrivalProgress::forNode(node.get());
  phase_pub_ = rclcpp::create_publisher<std_msgs::msg::String>(node, "path_tracking/phase", rclcpp::SensorDataQoS());
  logger_ = node->get_logger();
  clock_ = node->get_clock(); tf_ = tf; costmap_ = costmap;
  geometry_guard_.configure(node,costmap,"controller");
  metrics_rate_ = parameter(node, name + ".metrics.sample_hz", metrics_rate_, true);
  metrics_terminal_radius_ = parameter(node, name + ".metrics.terminal_exclusion_radius", metrics_terminal_radius_);
  if (metrics_rate_ > 20.0) {throw std::invalid_argument("metrics.sample_hz must be <= 20");}
  const std::string p = name + ".arrival.";
  nav2_util::declare_parameter_if_not_declared(node, name+".bridge_status_topic", rclcpp::ParameterValue(""));
  const auto bridge_topic = node->get_parameter(name+".bridge_status_topic").as_string();
  if (!bridge_topic.empty()) {
    using Status = astribot_bridge_msgs::msg::BridgeStatus;
    bridge_status_sub_ = node->create_subscription<Status>(bridge_topic, 10,
      [this](Status::ConstSharedPtr msg) {
        if (msg->node_name != "chassis_cmd_bridge") {return;}
        std::lock_guard<std::mutex> guard(bridge_status_mutex_);
        const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (stamp < bridge_status_stamp_) {return;}
        const char * reason = nullptr;
        switch (msg->state) {
          case Status::OK: bridge_error_.clear();bridge_status_stamp_=stamp;return;
          case Status::LEASH_TRIPPED: reason="BRIDGE_LEASH_TRIPPED";break;
          case Status::SLAM_LOST_STOPPED: reason="BRIDGE_SLAM_LOST_STOPPED";break;
          case Status::SCAN_LOST_STOPPED: reason="BRIDGE_SCAN_LOST_STOPPED";break;
          case Status::NOT_ENABLED: reason="BRIDGE_NOT_ENABLED";break;
          default: return;
        }
        bridge_error_ = std::string(reason)+": "+msg->detail;bridge_status_stamp_=stamp;
      });
  }
  nav2_util::declare_parameter_if_not_declared(node, "navigation_policy_enabled", rclcpp::ParameterValue(false));
  policy_enabled_=node->get_parameter("navigation_policy_enabled").as_bool();
  nav2_util::declare_parameter_if_not_declared(node,"navigation_policy_stage",rclcpp::ParameterValue("off"));
  const auto stage=node->get_parameter("navigation_policy_stage").as_string();
  policy_takeover_=stage=="p3" || stage=="p4" || stage=="p5";
  if (policy_enabled_ && (stage=="p4" || stage=="p5")) {
    corridor_simulated_=node->get_parameter("use_sim_time").as_bool();
    corridor_alignment_sub_=node->create_subscription<CorridorAlignment>(
      "navigation_policy/corridor_alignment",1,[this](CorridorAlignment::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(corridor_mutex_);
        if(corridor_alignment_ && msg->reference_path==corridor_alignment_->reference_path &&
            rclcpp::Time(msg->stamp)<rclcpp::Time(corridor_alignment_->stamp))return;
        corridor_alignment_=msg;corridor_received_=std::chrono::steady_clock::now();
      });
  }
  if (policy_enabled_) {
    active_path_pub_=rclcpp::create_publisher<nav_msgs::msg::Path>(node,
      "path_tracking/active_path",rclcpp::QoS(1).transient_local());
    policy_sub_=node->create_subscription<PolicyLease::Message>("navigation_policy/constraint",10,
      [this](PolicyLease::Message::ConstSharedPtr m) {policy_lease_.receive(*m);});
  }
  capture_ = parameter(node, p + "capture_radius", capture_);
  refine_timeout_ = parameter(node, p + "refine_timeout", refine_timeout_);
  total_timeout_ = parameter(node, p + "total_timeout", total_timeout_);
  progress_timeout_ = parameter(node, p + "progress_timeout", progress_timeout_);
  settle_time_ = parameter(node, p + "settle_time", settle_time_);
  stop_ratio_ = parameter(node, p + "stop_tolerance_ratio", stop_ratio_);
  resume_ratio_ = parameter(node, p + "resume_tolerance_ratio", resume_ratio_);
  settle_drift_ratio_ = parameter(node, p + "settle_drift_ratio", settle_drift_ratio_);
  if (stop_ratio_ > resume_ratio_ || resume_ratio_ > 1.0 || settle_drift_ratio_ > 1.0) {
    throw std::invalid_argument("Arrival: 0 < stop ratio <= resume ratio <= 1; drift ratio <= 1");
  }
  kp_xy_ = parameter(node, p + "kp_xy", kp_xy_);
  kp_yaw_ = parameter(node, p + "kp_yaw", kp_yaw_);
  max_v_ = parameter(node, p + "max_linear_speed", max_v_);
  max_w_ = parameter(node, p + "max_angular_speed", max_w_);
  min_v_ = parameter(node, p + "min_linear_speed", min_v_, true);
  min_w_ = parameter(node, p + "min_angular_speed", min_w_, true);
  coarse_yaw_ = parameter(node, p + "translation_yaw_tolerance", coarse_yaw_, true);
  braking_xy_ = parameter(node, p + "linear_braking_time", braking_xy_, true);
  braking_yaw_ = parameter(node, p + "angular_braking_time", braking_yaw_, true);
  normal_acceleration_=parameter(node,p+"normal_acceleration",normal_acceleration_);
  normal_jerk_=parameter(node,p+"normal_jerk",normal_jerk_);
  angular_acceleration_=parameter(node,p+"normal_angular_acceleration",angular_acceleration_);
  angular_jerk_=parameter(node,p+"normal_angular_jerk",angular_jerk_);
  auto configure_stop_model=[&](const std::string & axis,std::string & model,SettledOffsetCurve & curve) {
    nav2_util::declare_parameter_if_not_declared(node,p+axis+"_braking_model",rclcpp::ParameterValue("constant_time"));
    model=node->get_parameter(p+axis+"_braking_model").as_string();
    if (model!="constant_time" && model!="reference_stop" && model!="position_hold") {
      throw std::invalid_argument("Unknown arrival braking model: "+model);
    }
    if (model=="position_hold") {
      nav2_util::declare_parameter_if_not_declared(node,p+axis+"_stop_speeds",rclcpp::ParameterValue(std::vector<double>{}));
      nav2_util::declare_parameter_if_not_declared(node,p+axis+"_stop_offsets",rclcpp::ParameterValue(std::vector<double>{}));
      curve.configure(node->get_parameter(p+axis+"_stop_speeds").as_double_array(),
        node->get_parameter(p+axis+"_stop_offsets").as_double_array());
    }
  };
  configure_stop_model("linear",braking_model_,linear_offset_);
  configure_stop_model("angular",angular_braking_model_,angular_offset_);
  RCLCPP_INFO(logger_,"ARRIVAL_STOP_MODELS linear=%s angular=%s",braking_model_.c_str(),angular_braking_model_.c_str());
  if (min_v_ > max_v_ || min_w_ > max_w_) {
    throw std::invalid_argument("Arrival: minimum speed exceeds maximum speed");
  }
  if (capture_ <= 0.03 || max_v_ > 0.15 || max_w_ > 0.3) {
    throw std::invalid_argument("Arrival: capture_radius > 0.03, max speeds <= 0.15m/s, 0.3rad/s");
  }
  nav2_util::declare_parameter_if_not_declared(node, p + "source", rclcpp::ParameterValue("nav2_pose"));
  source_ = node->get_parameter(p + "source").as_string();
  if (source_ != "nav2_pose" && source_ != "slam_pose" && source_ != "vision" && source_ != "mark") {
    throw std::invalid_argument("Arrival: unknown localization source " + source_);
  }
  const std::string default_topic = source_ == "slam_pose" ? "/slam/pose" :
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
  if (policy_enabled_ ||
    node->get_parameter(name + ".corner_turn_enabled").as_bool())
  {
    for (const auto & suffix : {".inner.vx_max", ".inner.desired_linear_vel"}) {
      if (node->has_parameter(name+suffix)) {
        nominal_speed_=node->get_parameter(name+suffix).as_double();break;
      }
    }
    if (!std::isfinite(nominal_speed_) || nominal_speed_<=0.) {
      throw std::runtime_error("Policy speed composition requires the inner controller nominal speed");
    }
    external_speed_limit_=0.;external_speed_percentage_=false;
  }
}
void ArrivalController::publishPhase(const char * phase)
{
  if (phase_pub_) {std_msgs::msg::String msg; msg.data = phase; phase_pub_->publish(msg);}
}
void ArrivalController::resetAttempt()
{
  if (arrival_progress_) {arrival_progress_->clear();}
  started_ = refining_ = holding_ = completion_logged_ = false;
  xy_held_ = yaw_held_ = corridor_terminal_coast_ = false;
  xy_coast_.reset(); yaw_coast_.reset();
  xy_settling_.reset(); yaw_settling_.reset();
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
  geometry_guard_.cleanup();
  bridge_status_sub_.reset();
  {std::lock_guard<std::mutex> guard(bridge_status_mutex_);bridge_error_.clear();bridge_status_stamp_=-1;}
  corridor_alignment_sub_.reset();
  {std::lock_guard<std::mutex> lock(corridor_mutex_);corridor_alignment_.reset();}
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
void ArrivalController::applyPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty() || path.header.frame_id.empty()) {
    fail("INVALID_PATH: empty path or frame");
  }
  for (const auto & pose : path.poses) {
    if (!validPose(pose.pose) || (!pose.header.frame_id.empty() &&
      pose.header.frame_id != path.header.frame_id)) {fail("INVALID_PATH: invalid pose/frame");}
  }
  ThreePhaseController::applyPlan(path);
  const bool refresh=planUpdate()==PlanUpdate::EquivalentRefresh;
  const bool takeover=policy_takeover_ && !refining_ &&
    planUpdate()==PlanUpdate::RouteReplacement;
  if (arrival_progress_) {arrival_progress_->clear();}
  auto goal = path.poses.back();
  goal.header.frame_id = path.header.frame_id;
  goal.header.stamp = builtin_interfaces::msg::Time();  // fixed world target, latest TF
  if (planUpdate()==PlanUpdate::NewExecution || !has_goal_ || goal.header.frame_id != goal_.header.frame_id ||
    distance(goal.pose, goal_.pose) > 1e-4 || std::abs(yawError(goal.pose, goal_.pose)) > 1e-4)
  {
    resetAttempt();
  }
  goal_ = goal; has_goal_ = true;
  tracking_path_ = path;
  if (planUpdate()==PlanUpdate::RouteReplacement) {
    // The old terminal maneuver belongs to a different route. Keep the task's
    // elapsed/progress budget, but reacquire the replacement before refining.
    refining_=holding_=completion_logged_=false;
    xy_held_=yaw_held_=corridor_terminal_coast_=false;
    xy_coast_.reset();yaw_coast_.reset();xy_settling_.reset();yaw_settling_.reset();
  }
  ++plan_revision_;
  if (!refresh) {metrics_anchor_.reset();}
  if (active_path_pub_) {active_path_pub_->publish(path);}
  if(takeover) {preparePolicyTakeover();}
}
void ArrivalController::limitCornerTranslation(geometry_msgs::msg::Twist & command)
{
  double cap=nominal_speed_;
  {
    std::lock_guard<std::mutex> lock(speed_limit_mutex_);
    if (external_speed_limit_>0.) {
      cap=std::min(cap,external_speed_percentage_ ?
        nominal_speed_*external_speed_limit_/100. : external_speed_limit_);
    }
  }
  if (policy_enabled_) {cap=std::min(cap,policy_lease_.linearSpeedLimit(clock_->now()));}
  const double speed=std::hypot(command.linear.x,command.linear.y);
  if (speed>cap && speed>0.) {
    command.linear.x*=std::max(0.,cap)/speed;
    command.linear.y*=std::max(0.,cap)/speed;
  }
}

double ArrivalController::cornerStoppingDistance(double speed) const
{
  if (braking_model_=="constant_time") {return braking_xy_*std::abs(speed);}
  try {
    return referenceStopDistance(speed,normal_acceleration_,normal_jerk_)+
      (braking_model_=="position_hold"?linear_offset_.distance(speed):0.);
  } catch(const std::out_of_range & error) {throw nav2_core::PlannerException(error.what());}
}

void ArrivalController::setSpeedLimit(const double & limit, const bool & percentage)
{
  if (!std::isfinite(limit) || limit < 0 || (percentage && limit > 100)) {
    fail("INVALID_SPEED_LIMIT");
  }
  speed_scale_ = limit == 0 ? 1.0 : std::min(1.0, percentage ? limit / 100.0 : limit / max_v_);
  {
    std::lock_guard<std::mutex> lock(speed_limit_mutex_);
    external_speed_limit_=limit;external_speed_percentage_=percentage;
  }
  if (!policy_enabled_) {
    ThreePhaseController::setSpeedLimit(limit, percentage);
  }
}
[[noreturn]] void ArrivalController::fail(const std::string & reason)
{
  if (arrival_progress_) {arrival_progress_->clear();}
  if (error_ != reason) {RCLCPP_ERROR(logger_, "PATH_TRACKING/%s", reason.c_str());}
  error_ = reason;
  throw nav2_core::PlannerException("PATH_TRACKING/" + reason);
}
geometry_msgs::msg::PoseStamped ArrivalController::inFrame(
  const geometry_msgs::msg::PoseStamped & pose, const std::string & frame)
{
  if (pose.header.frame_id.empty() || !validPose(pose.pose)) {fail("INVALID_POSE");}
  try {
    if(pose.header.frame_id==frame)return pose;
    auto latest=pose;latest.header.stamp={};
    return tf_->transform(latest, frame, tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException & e) {fail(std::string("TF_UNAVAILABLE: ") + e.what());}
}

bool ArrivalController::safeCommand(const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & command, const geometry_msgs::msg::Twist & measured)
{
  auto reject = [&](const char * reason, double x, double y, double yaw,
      int step = -1, int steps = 0, double cost = -1.) {
    RCLCPP_WARN(logger_,
      "COMMAND_SAFETY_REJECT reason=%s frame=%s x=%.6f y=%.6f yaw=%.6f step=%d/%d cost=%.1f "
      "cmd=(%.6f,%.6f,%.6f) measured=(%.6f,%.6f,%.6f)",
      reason, costmap_->getGlobalFrameID().c_str(), x, y, yaw, step, steps, cost,
      command.linear.x, command.linear.y, command.angular.z,
      measured.linear.x, measured.linear.y, measured.angular.z);
    return false;
  };
  const double unavailable = std::numeric_limits<double>::quiet_NaN();
  if (!geometry_guard_.ready()) {return reject("ENVELOPE_NOT_READY", unavailable, unavailable, unavailable);}
  if (!costmap_->isCurrent()) {return reject("COSTMAP_NOT_CURRENT", unavailable, unavailable, unavailable);}
  auto local = inFrame(pose, costmap_->getGlobalFrameID());
  auto * map = costmap_->getCostmap();
  auto footprint = costmap_->getRobotFootprint();
  if (footprint.size() < 3) {return reject("FOOTPRINT_TOO_SMALL", unavailable, unavailable, unavailable);}
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision(map);
  for (const auto & twist : {command, measured}) {
    double x = local.pose.position.x, y = local.pose.position.y;
    double yaw = tf2::getYaw(local.pose.orientation);
    const double radius = costmap_->getLayeredCostmap()->getCircumscribedRadius();
    const double sweep = std::hypot(twist.linear.x, twist.linear.y) + radius*std::abs(twist.angular.z);
    const int steps = std::max(20, static_cast<int>(std::ceil(sweep / (map->getResolution()*0.5))));
    const double x0=x,y0=y,yaw0=yaw;
    for (int i = 0; i <= steps; ++i) {
      if (geometry_guard_.enabled()) {
        const double t=double(i)/steps,w=twist.angular.z;
        const double a=std::abs(w)<1e-10 ? t : std::sin(w*t)/w;
        const double b=std::abs(w)<1e-10 ? 0. : (1.-std::cos(w*t))/w;
        const double dx=a*twist.linear.x-b*twist.linear.y,dy=b*twist.linear.x+a*twist.linear.y;
        x=x0+std::cos(yaw0)*dx-std::sin(yaw0)*dy;
        y=y0+std::sin(yaw0)*dx+std::cos(yaw0)*dy;yaw=yaw0+w*t;
      }
      unsigned int mx, my;
      const double cost = collision.footprintCostAtPose(x, y, yaw, footprint);
      if (geometry_guard_.enabled() && astribot_s1_robot_geometry::collision(*map,footprint,x,y,yaw,sweep/(2*steps))) {
        return reject("FILLED_FOOTPRINT_COLLISION", x, y, yaw, i, steps, cost);
      }
      if (!map->worldToMap(x, y, mx, my)) {return reject("SWEEP_OUTSIDE_COSTMAP", x, y, yaw, i, steps, cost);}
      if (cost < 0) {return reject("FOOTPRINT_COST_INVALID", x, y, yaw, i, steps, cost);}
      if (cost >= 254) {return reject("FOOTPRINT_COST_BLOCKED", x, y, yaw, i, steps, cost);}
      if (map->getCost(mx, my) >= 253) {return reject("CENTER_COST_BLOCKED", x, y, yaw, i, steps, map->getCost(mx, my));}
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

void ArrivalController::logMetrics(const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity, const ArrivalGoalChecker & checker,
  const char * phase_name)
{
  const double now = clock_->now().seconds();
  // Sample before projecting: neither the path search nor formatting runs at control frequency.
  if (!metrics_clock_.due(now, metrics_rate_)) {return;}
  const double xy = distance(pose.pose, goal_.pose);
  const double yaw_deg = std::abs(yawError(pose.pose, goal_.pose))*180.0/M_PI;
  const double speed = std::hypot(velocity.linear.x, velocity.linear.y);
  if (refining_) {
    RCLCPP_INFO(logger_,
      "ARRIVAL_METRICS ros_s=%.3f pose_s=%.3f plan=%llu phase=%s source=%s stop_source=slam_pose frame=%s "
      "xy_m=%.6f yaw_deg=%.6f xy_tol_m=%.6f yaw_tol_deg=%.6f "
      "speed_mps=%.6f wz_radps=%.6f xy_held=%d yaw_held=%d hold_s=%.3f sample_hz=%.2f "
      "stop_source=pose_window stop_xy=%d stop_yaw=%d stop_span_xy_s=%.3f stop_span_yaw_s=%.3f "
      "stop_speed_mps=%.6f stop_wz_radps=%.6f stop_drift_m=%.6f stop_drift_rad=%.6f",
      now, rclcpp::Time(pose.header.stamp).seconds(), static_cast<unsigned long long>(plan_revision_),
      phase_name, source_.c_str(), pose.header.frame_id.c_str(), xy, yaw_deg,
      checker.xyTolerance(), checker.yawTolerance()*180.0/M_PI, speed, velocity.angular.z,
      xy_held_, yaw_held_, holding_ ? std::min(xy_settling_.evidence().span,yaw_settling_.evidence().span) : 0.0,
      metrics_rate_, xy_settling_.evidence().stopped, yaw_settling_.evidence().stopped,
      xy_settling_.evidence().span, yaw_settling_.evidence().span,
      xy_settling_.evidence().speed, yaw_settling_.evidence().speed,
      xy_settling_.evidence().drift, yaw_settling_.evidence().drift);
    return;
  }
  const auto projection = pose.header.frame_id == tracking_path_.header.frame_id ?
    projectTrackingPose(tracking_path_, pose.pose.position.x, pose.pose.position.y, metrics_anchor_) :
    std::nullopt;
  const double missing = std::numeric_limits<double>::quiet_NaN();
  const bool follow = projection && phase() == Phase::kFollow;
  const double terminal_radius = std::max(capture_, metrics_terminal_radius_);
  if (projection) {metrics_anchor_ = projection->progress_m;}
  RCLCPP_INFO(logger_,
    "TRACKING_METRICS ros_s=%.3f pose_s=%.3f plan=%llu phase=%s source=nav2_pose frame=%s "
    "path_valid=%d cross_track_m=%.6f heading_error_deg=%.6f progress_m=%.3f curvature_1pm=%.5f "
    "follow_sample=%d travel_sample=%d straight_sample=%d terminal_radius_m=%.3f "
    "goal_xy_m=%.6f goal_yaw_deg=%.6f speed_mps=%.6f wz_radps=%.6f sample_hz=%.2f",
    now, rclcpp::Time(pose.header.stamp).seconds(), static_cast<unsigned long long>(plan_revision_),
    phase_name, pose.header.frame_id.c_str(), static_cast<bool>(projection),
    projection ? projection->cross_track_m : missing,
    projection ? std::remainder(tf2::getYaw(pose.pose.orientation)-projection->tangent_rad, 2*M_PI)*180.0/M_PI : missing,
    projection ? projection->progress_m : missing, projection ? projection->curvature : missing,
    follow, follow && xy > terminal_radius, follow && std::abs(projection->curvature) < 0.1,
    terminal_radius, xy, yaw_deg, speed, velocity.angular.z, metrics_rate_);
}

geometry_msgs::msg::TwistStamped ArrivalController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * checker)
{
  auto command=computeCommand(pose,velocity,checker);
  if (policy_enabled_ && policy_lease_.restrict(command.twist,clock_->now()) &&
    (std::hypot(command.twist.linear.x,command.twist.linear.y)>0. || command.twist.angular.z!=0.) &&
    !safeCommand(pose,command.twist,velocity)) {
    fail("POLICY_RESTRICTED_SWEEP_BLOCKED");
  }
  return command;
}

geometry_msgs::msg::TwistStamped ArrivalController::computeCommand(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * checker)
{
  applyPendingPlan();
  arrival_progress_->clear();
  if (!geometry_guard_.ready()) {fail("ENVELOPE_V2_NOT_READY");}
  auto * arrival = dynamic_cast<ArrivalGoalChecker *>(checker);
  if (!arrival) {fail("CHECKER_MISMATCH: ArrivalGoalChecker required");}
  arrival->report(false);  // invalidate before every possible return/exception
  {
    std::lock_guard<std::mutex> guard(bridge_status_mutex_);
    if (!bridge_error_.empty()) {fail(bridge_error_);}
  }
  if (checker_ != arrival) {
    holding_ = xy_held_ = yaw_held_ = corridor_terminal_coast_ = false; checker_ = arrival; generation_ = arrival->generation();
    xy_coast_.reset(); yaw_coast_.reset();
    xy_settling_.reset(); yaw_settling_.reset();
  }
  generation_=arrival->generation();
  last_tick_ = clock_->now().seconds();
  if (!error_.empty()) {fail(error_);}
  if (!has_goal_) {fail("INVALID_PATH: no target");}
  if (!std::isfinite(velocity.linear.x) || !std::isfinite(velocity.linear.y) ||
    !std::isfinite(velocity.angular.z)) {fail("INVALID_VELOCITY");}
  const double now=steadyNow();
  auto current = inFrame(pose, goal_.header.frame_id);
  observeCornerPose();
  if (!started_) {
    started_ = true; started_at_ = progress_at_ = now; anchor_ = current;
  }
  if (now < started_at_ || now < progress_at_) {fail("CLOCK_JUMP");}
  if (policy_enabled_) {
    const auto policy_status=policy_lease_.status(clock_->now());
    if (policy_status!=PolicyLease::Status::Fresh) {
      if (policy_status==PolicyLease::Status::WaitForClock) {
        xy_settling_.reset();yaw_settling_.reset();holding_=false;
        publishPhase("POLICY_CLOCK_WAIT");
        recordCornerPause(now);
        geometry_msgs::msg::TwistStamped stopped;stopped.header=pose.header;return stopped;
      }
      RCLCPP_ERROR(logger_, "POLICY_LEASE_EVIDENCE %s", policy_lease_.freshnessDetail(clock_->now()).c_str());
      xy_settling_.reset();yaw_settling_.reset();holding_=false;
      publishPhase("POLICY_LEASE_WAIT");
      recordCornerPause(now);
      // Nav2 publishes zero while retrying under its existing failure_tolerance.
      // A transient expiry must not latch error_ and reject a later fresh lease.
      throw nav2_core::PlannerException("PATH_TRACKING/POLICY_LEASE_EXPIRED");
    }
    if (policy_paused_ && policy_tick_>=0) {
      const double duration=std::max(0.0,now-policy_tick_);
      started_at_+=duration;progress_at_+=duration;
      if (refining_) {refine_at_+=duration;}
      accountPolicyPause(duration,clock_->now());
    }
    policy_tick_=now;policy_paused_=policy_lease_.held(clock_->now());
    if (policy_paused_) {
      xy_settling_.reset(); yaw_settling_.reset(); holding_=false;
      publishPhase("POLICY_HOLD");
      recordCornerPause(now);
      geometry_msgs::msg::TwistStamped stopped;stopped.header=pose.header;return stopped;
    }
    const double policy_cap=policy_lease_.linearSpeedLimit(clock_->now());
    if (policy_cap<=0.) {
      xy_settling_.reset(); yaw_settling_.reset(); holding_=false;
      policy_paused_=true;publishPhase("POLICY_HOLD");
      recordCornerPause(now);
      geometry_msgs::msg::TwistStamped stopped;stopped.header=pose.header;return stopped;
    }
    std::lock_guard<std::mutex> lock(speed_limit_mutex_);
    const double external=external_speed_limit_==0. ? nominal_speed_ :
      (external_speed_percentage_ ? nominal_speed_*external_speed_limit_/100. : external_speed_limit_);
    const double effective=std::min({nominal_speed_,external,policy_cap});
    // Reapply each cycle: optimizer recovery may reset its speed constraints.
    // Zero is Nav2's reset sentinel and must never represent a policy stop.
    ThreePhaseController::setSpeedLimit(effective,false);
  }
  // Lease/HOLD processing keeps priority, but every motion owner shares the
  // same replacement stop barrier before corridor or normal tracking.
  if (!cornerReanchorReady(clock_->now())) {
    publishPhase("REANCHOR_SETTLING");
    geometry_msgs::msg::TwistStamped stopped;stopped.header=pose.header;return stopped;
  }
  const bool corridor_tracking=policy_enabled_ && policy_lease_.corridorTrackingRequired(clock_->now());
  geometry_msgs::msg::Pose corridor_heading;
  if (corridor_tracking) {
    std::lock_guard<std::mutex> lock(corridor_mutex_);
    const auto & request=corridor_alignment_;
    bool valid=false;
    if (!policy_lease_.centeringRequired(clock_->now()) && !policy_lease_.alignmentRequired(clock_->now()) &&
        request && request->tracking_required && !request->centering_required &&
        std::isfinite(request->lease_s) && request->lease_s>0 && request->lease_s<=0.3) {
      const double age=(clock_->now()-rclcpp::Time(request->stamp,clock_->get_clock_type())).seconds();
      const double wall_age=std::chrono::duration<double>(std::chrono::steady_clock::now()-corridor_received_).count();
      valid=corridorRequestFresh(corridor_simulated_,age,wall_age,request->lease_s) &&
        request->reference_path==tracking_path_ && validPose(request->anchor.pose) &&
        request->anchor.header.frame_id==current.header.frame_id;
      if (!valid) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
          "CORRIDOR_HEADING_INVALID source_age_s=%.6f wall_age_s=%.6f lease_s=%.6f "
          "path_matches=%d anchor_valid=%d frame_matches=%d",
          age, wall_age, request->lease_s, request->reference_path==tracking_path_,
          validPose(request->anchor.pose), request->anchor.header.frame_id==current.header.frame_id);
      }
      if (valid) {corridor_heading=request->anchor.pose;}
    }
    if (!valid) {
      publishPhase("CORRIDOR_HEADING_PENDING");
      geometry_msgs::msg::TwistStamped stopped;stopped.header=pose.header;return stopped;
    }
  }
  if (policy_enabled_ && policy_lease_.centeringRequired(clock_->now())) {
    std::lock_guard<std::mutex> lock(corridor_mutex_);
    const auto & request=corridor_alignment_;
    geometry_msgs::msg::TwistStamped command;command.header=pose.header;
    if (!refining_ && !policy_lease_.alignmentRequired(clock_->now()) && request &&
        request->centering_required && !request->tracking_required && std::isfinite(request->lease_s) &&
        request->lease_s>0 && request->lease_s<=0.3) {
      const double age=(clock_->now()-rclcpp::Time(request->stamp,clock_->get_clock_type())).seconds();
      const double wall_age=std::chrono::duration<double>(std::chrono::steady_clock::now()-corridor_received_).count();
      const auto & a=request->anchor.pose.position;
      const auto & b=request->target.pose.position;
      const auto & c=current.pose.position;
      const double dx=b.x-a.x, dy=b.y-a.y, length=std::hypot(dx,dy);
      if (corridorRequestFresh(corridor_simulated_,age,wall_age,request->lease_s) &&
          request->reference_path==tracking_path_ && validPose(request->anchor.pose) &&
          validPose(request->target.pose) && request->anchor.header.frame_id==current.header.frame_id &&
          request->target.header.frame_id==current.header.frame_id && length>1e-6 && length<=0.300001 &&
          std::hypot(velocity.linear.x,velocity.linear.y)<=0.08 &&
          std::abs(yawError(current.pose,request->anchor.pose))<=0.05) {
        const double along=((c.x-a.x)*dx+(c.y-a.y)*dy)/length;
        const double side=std::abs((c.x-a.x)*dy-(c.y-a.y)*dx)/length;
        if (along>=-0.04 && along<=length+0.04 && side<=0.04) {
          const double ex=b.x-c.x, ey=b.y-c.y, remaining=std::hypot(ex,ey);
          const double scale=remaining>1e-6 ? std::min(0.05,remaining)/remaining : 0.;
          const auto & q=current.pose.orientation;
          const double heading=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
          command.twist.linear.x=scale*(std::cos(heading)*ex+std::sin(heading)*ey);
          command.twist.linear.y=scale*(-std::sin(heading)*ex+std::cos(heading)*ey);
          if (geometry_guard_.enabled() && !safeCommand(pose,command.twist,velocity)) {fail("CORRIDOR_OFFSET_SWEEP_BLOCKED");}
          progress_at_=now;publishPhase("CORRIDOR_CENTER");
          return command;
        }
      }
    }
    publishPhase("CORRIDOR_CENTER_PENDING");return command;
  }
  if (policy_enabled_ && policy_lease_.alignmentRequired(clock_->now())) {
    std::lock_guard<std::mutex> lock(corridor_mutex_);
    const auto & request=corridor_alignment_;
    if (!refining_ && request && !request->centering_required && !request->tracking_required && std::isfinite(request->lease_s) && request->lease_s>0 && request->lease_s<=0.3) {
      const double age=(clock_->now()-rclcpp::Time(request->stamp,clock_->get_clock_type())).seconds();
      const double wall_age=std::chrono::duration<double>(std::chrono::steady_clock::now()-corridor_received_).count();
      if (corridorRequestFresh(corridor_simulated_,age,wall_age,request->lease_s) &&
          request->reference_path==tracking_path_ && validPose(request->anchor.pose) &&
          request->anchor.header.frame_id==current.header.frame_id &&
          distance(current.pose,request->anchor.pose)<=0.04 &&
          std::hypot(velocity.linear.x,velocity.linear.y)<=0.02) {
        progress_at_=now;publishPhase("CORRIDOR_ALIGN");
        auto command=policyAlignment(yawError(current.pose,request->anchor.pose),velocity.angular.z,pose.header);
        if (geometry_guard_.enabled() && !safeCommand(pose,command.twist,velocity)) {fail("CORRIDOR_ALIGNMENT_SWEEP_BLOCKED");}
        return command;
      }
    }
    publishPhase("CORRIDOR_ALIGNMENT_PENDING");
    geometry_msgs::msg::TwistStamped stopped;stopped.header=pose.header;return stopped;
  }
  if (now - started_at_ > total_timeout_) {fail("GOAL_TIMEOUT");}
  if (!refining_ && distance(current.pose, goal_.pose) <= capture_ &&
    terminalRefinementAllowed(current,capture_))
  {
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
    if (phase() == Phase::kAlignCorner && policy_enabled_) {
      const double cap = policy_lease_.cornerAngularSpeedLimit(clock_->now());
      if (cap <= 0.) {fail("CORNER_ROTATION_REQUIRES_ROUTE: current policy forbids in-place turn");}
      cmd.twist.angular.z = std::clamp(cmd.twist.angular.z, -cap, cap);
    }
    if (phase()==Phase::kCornerApproach || phase()==Phase::kAlignCorner) {
      limitCornerTranslation(cmd.twist);
    }
    if (phase() == Phase::kFollow) {
      double cap = localSharpPathLimit(current);
      if (braking_model_!="constant_time") {
        const double available=std::max(0.,distance(current.pose,goal_.pose)-capture_)+
          referenceStopDistance(max_v_*speed_scale_,normal_acceleration_,normal_jerk_);
        cap=std::min(cap,referenceSpeedForDistance(available,
          std::hypot(cmd.twist.linear.x,cmd.twist.linear.y),normal_acceleration_,normal_jerk_));
      }
      const double speed = std::hypot(cmd.twist.linear.x,cmd.twist.linear.y);
      if (speed > cap) {
        const double ratio = cap/speed;
        cmd.twist.linear.x *= ratio;cmd.twist.linear.y *= ratio;cmd.twist.angular.z *= ratio;
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "PATH_QUALITY_SPEED_LIMIT %.3fm/s", cap);
      }
      if (corridor_tracking) {
        const double angular_cap=std::min(.2,max_w_)*speed_scale_;
        cmd.twist.angular.z=std::clamp(kp_yaw_*yawError(current.pose,corridor_heading),-angular_cap,angular_cap);
        if (!safeCommand(pose,cmd.twist,velocity)) {fail("CORRIDOR_TRACKING_BLOCKED: unsafe footprint sweep");}
      }
    }
    if (geometry_guard_.enabled() && !safeCommand(pose,cmd.twist,velocity)) {fail("FOLLOW_POLYGON_SWEEP_BLOCKED");}
    if (geometry_guard_.enabled() && corridor_tracking && std::hypot(cmd.twist.linear.x,cmd.twist.linear.y)<.005 && std::abs(cmd.twist.angular.z)>.005) {fail("ROTATION_FORBIDDEN_IN_NARROW_PASSAGE");}
    publishPhase(toString(phase()));
    logMetrics(current, velocity, *arrival, toString(phase()));
    return cmd;
  }
  const bool fixed_corridor_refine=geometry_guard_.enabled() && corridor_tracking;
  publishPhase("REFINE");
  if (now-refine_at_ > refine_timeout_) {fail("REFINEMENT_TIMEOUT");}
  if (source_ != "nav2_pose") {
    geometry_msgs::msg::PoseStamped observation;
    {std::lock_guard<std::mutex> guard(observation_mutex_);
      if (!observed_) {fail("POSE_UNAVAILABLE: " + source_);}
      observation = observation_;
    }
    current = inFrame(observation, goal_.header.frame_id);
  }
  const double xy = distance(current.pose, goal_.pose);
  const double yaw = yawError(current.pose, goal_.pose);
  if (fixed_corridor_refine && !corridorRefinementReachable(
      yawError(corridor_heading,goal_.pose),yaw,false,arrival->yawTolerance())) {
    fail("GOAL_HEADING_UNREACHABLE: ROTATION_FORBIDDEN_IN_NARROW_PASSAGE");
  }
  if (xy > capture_*2) {fail("POSE_DISAGREEMENT: outside refinement region");}
  const auto stop_observation=slamObservation();
  const auto &stop_pose=stop_observation.first;
  const double source_time = rclcpp::Time(stop_pose.header.stamp).seconds();
  xy_settling_.observe(source_time,stop_observation.second,{stop_pose.pose.position.x,stop_pose.pose.position.y},
    settle_time_,arrival->stoppedLinear(),settle_drift_ratio_*arrival->xyTolerance());
  yaw_settling_.observe(source_time,stop_observation.second,{tf2::getYaw(stop_pose.pose.orientation)},
    settle_time_,arrival->stoppedAngular(),settle_drift_ratio_*arrival->yawTolerance(),true);
  const auto & xy_evidence=xy_settling_.evidence();
  const auto & yaw_evidence=yaw_settling_.evidence();
  if (fixed_corridor_refine && xy<=arrival->xyTolerance()) {corridor_terminal_coast_=true;}
  const bool corridor_terminal=fixed_corridor_refine && corridor_terminal_coast_;
  auto stop_distance=[](double speed, double time, const std::string & model,
      double acceleration, double jerk, const SettledOffsetCurve & offset) {
      if (model=="constant_time") {return time*speed;}
      return referenceStopDistance(speed,acceleration,jerk)+
        (model=="position_hold" ? offset.distance(speed) : 0.);
    };
  const double measured_speed=std::hypot(velocity.linear.x,velocity.linear.y);
  const double measured_w=std::abs(velocity.angular.z);
  double xy_stop=0.,yaw_stop=0.;
  try {
    xy_stop=stop_distance(measured_speed,braking_xy_,braking_model_,normal_acceleration_,normal_jerk_,linear_offset_);
    yaw_stop=stop_distance(measured_w,braking_yaw_,angular_braking_model_,angular_acceleration_,angular_jerk_,angular_offset_);
  } catch (const std::out_of_range & e) {fail(e.what());}
  const bool brake_xy=braking_xy_>0. || braking_model_!="constant_time";
  const bool brake_yaw=braking_yaw_>0. || angular_braking_model_!="constant_time";
  const double remaining_xy = xy + xy_stop;
  const double remaining_yaw = std::abs(yaw) + yaw_stop;
  // Passing through zero at speed is not better progress than a settled pose.
  const double metric = remaining_xy/arrival->xyTolerance() + remaining_yaw/arrival->yawTolerance();
  if (metric < best_error_ - 0.1) {best_error_ = metric; progress_at_ = now;}
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now(); cmd.header.frame_id = costmap_->getBaseFrameID();
  const bool within = xy <= arrival->xyTolerance() && std::abs(yaw) <= arrival->yawTolerance();
  // Inside the goal bounds, wait for settling rather than further error reduction.
  // The refinement deadline still bounds a pose that never becomes stable.
  if (within) {progress_at_ = now;}
  // Independent Schmitt triggers prevent one settled axis from restarting with the other.
  auto update_hold = [this](bool held, double error, double tolerance) {
      return held ? error <= resume_ratio_*tolerance : error <= stop_ratio_*tolerance;
    };
  const bool was_xy_held = xy_held_;
  xy_held_ = corridor_terminal || update_hold(xy_held_, xy, arrival->xyTolerance());
  // Translating wheels disturb heading: defer fine yaw corrections until XY is held.
  if (xy_held_ != was_xy_held) {
    if (xy_held_) {yaw_held_ = false;}
    // Each change between XY correction and fine yaw correction starts its own
    // progress window. Their combined error is not comparable across phases;
    // refine_timeout_ still bounds the complete refinement, including retries.
    best_error_ = metric; progress_at_ = now;
  }
  if (now-progress_at_ > progress_timeout_) {fail("REFINEMENT_NO_PROGRESS");}
  // A corridor cannot defer fine yaw correction until translation has stopped.
  const double yaw_tolerance = (xy_held_ || fixed_corridor_refine) ? arrival->yawTolerance() :
    std::max(arrival->yawTolerance(), coarse_yaw_);
  yaw_held_ = corridor_terminal || update_hold(yaw_held_, std::abs(yaw), yaw_tolerance);
  const bool stopped = xy_evidence.stopped && yaw_evidence.stopped;
  // Once zero commands are confirmed, use the same pose window for residual motion.
  // Active braking and collision prediction retain the incoming velocity feedback.
  bool residual_inside =
    stopped &&
    (!brake_xy || xy+stop_distance(xy_evidence.speed,braking_xy_,braking_model_,
      normal_acceleration_,normal_jerk_,linear_offset_) <= resume_ratio_*arrival->xyTolerance()) &&
    (!brake_yaw || std::abs(yaw)+stop_distance(yaw_evidence.speed,braking_yaw_,angular_braking_model_,
      angular_acceleration_,angular_jerk_,angular_offset_) <= resume_ratio_*arrival->yawTolerance());
  if (corridor_terminal) {
    residual_inside=stopped && corridorTerminalWithinBounds(xy,yaw,
      brake_xy ? stop_distance(xy_evidence.speed,braking_xy_,braking_model_,normal_acceleration_,normal_jerk_,linear_offset_) : 0.,
      brake_yaw ? stop_distance(yaw_evidence.speed,braking_yaw_,angular_braking_model_,angular_acceleration_,angular_jerk_,angular_offset_) : 0.,
      arrival->xyTolerance(),arrival->yawTolerance());
  }
  // Inside a passage both commands remain zero once the terminal stop begins.
  // Judge the final pose after the existing 0.6 s acquisition-time stop window;
  // do not abort on a transient crossing, or restart one axis in place. The
  // configured 2 mm / 0.1 degree bounds and drift limits remain mandatory.
  if (corridor_terminal && stopped && (!within || !residual_inside)) {
    fail("GOAL_HEADING_UNREACHABLE: CORRIDOR_SETTLED_RESIDUAL_OUTSIDE_TOLERANCE");
  }
  holding_ = within && xy_held_ && yaw_held_;
  if (holding_ && residual_inside) {
    if (!safeCommand(pose, cmd.twist, velocity)) {fail("REFINEMENT_BLOCKED: unsafe final footprint");}
    arrival_progress_->report(clock_->now().seconds());
    xy_coast_.reset(); yaw_coast_.reset();
    arrival->report(true);
    publishPhase("DONE");
    logMetrics(current, velocity, *arrival, "DONE");
    if (!completion_logged_) {
      completion_logged_ = true;
      RCLCPP_INFO(logger_, "ARRIVAL_REACHED source=%s stop_source=slam_pose xy=%.6fm yaw=%.6fdeg settled=%.2fs "
        "ros_s=%.3f pose_s=%.3f plan=%llu frame=%s xy_tol_m=%.6f yaw_tol_deg=%.6f "
        "speed_mps=%.6f wz_radps=%.6f stop_evidence=pose_window stop_speed_mps=%.6f stop_wz_radps=%.6f",
        source_.c_str(), xy, std::abs(yaw)*180.0/M_PI, std::min(xy_evidence.span,yaw_evidence.span), now,
        rclcpp::Time(current.header.stamp).seconds(), static_cast<unsigned long long>(plan_revision_),
        current.header.frame_id.c_str(), arrival->xyTolerance(), arrival->yawTolerance()*180.0/M_PI,
        std::hypot(velocity.linear.x, velocity.linear.y), velocity.angular.z,
        xy_evidence.speed,yaw_evidence.speed);
    }
    return cmd;
  }
  const double heading = tf2::getYaw(current.pose.orientation);
  const double dx = goal_.pose.position.x-current.pose.position.x;
  const double dy = goal_.pose.position.y-current.pose.position.y;
  if (!xy_held_) {
    const double speed = std::min(max_v_*speed_scale_, std::max(min_v_*speed_scale_, kp_xy_*xy));
    const double scale = speed/std::max(xy, 1e-9);
    cmd.twist.linear.x = scale*(std::cos(heading)*dx + std::sin(heading)*dy);
    cmd.twist.linear.y = scale*(-std::sin(heading)*dx + std::cos(heading)*dy);
    if (brake_xy) {
      const auto v = brakingTranslation(
        std::cos(heading)*dx + std::sin(heading)*dy,
        -std::sin(heading)*dx + std::cos(heading)*dy,
        velocity.linear.x, velocity.linear.y, kp_xy_, min_v_*speed_scale_, max_v_*speed_scale_,
        measured_speed>1e-9 ? xy_stop/measured_speed : 0.);
      cmd.twist.linear.x = v[0];cmd.twist.linear.y = v[1];
    }
  }
  if (!yaw_held_) {
    const double minimum_yaw=fixed_corridor_refine ? 0. : min_w_*speed_scale_;
    const double yaw_gain=fixed_corridor_refine ? corridorApproachHeadingGain(xy,
      std::hypot(cmd.twist.linear.x,cmd.twist.linear.y),measured_speed,kp_yaw_,arrival->xyTolerance()) : kp_yaw_;
    const double speed = std::min(max_w_*speed_scale_, std::max(minimum_yaw, yaw_gain*std::abs(yaw)));
    cmd.twist.angular.z = std::copysign(speed, yaw);
    if (brake_yaw) {
      cmd.twist.angular.z = brakingRotation(
        yaw, velocity.angular.z, yaw_gain, minimum_yaw, max_w_*speed_scale_,
        measured_w>1e-9 ? yaw_stop/measured_w : 0.);
    }
  }
  const bool was_xy_coasting = xy_coast_.active(), was_yaw_coasting = yaw_coast_.active();
  if (brake_xy) {
    // Compare directions in the goal frame; body axes rotate during yaw refinement.
    const double c = std::cos(heading), s = std::sin(heading);
    const auto v = xy_coast_.apply(
      {c*cmd.twist.linear.x-s*cmd.twist.linear.y, s*cmd.twist.linear.x+c*cmd.twist.linear.y},
      xy_evidence.stopped);
    cmd.twist.linear.x = c*v[0]+s*v[1]; cmd.twist.linear.y = -s*v[0]+c*v[1];
  }
  if (brake_yaw) {
    cmd.twist.angular.z = applyArrivalYawCoast(yaw_coast_,cmd.twist.angular.z,yaw_evidence.stopped,
      fixed_corridor_refine && corridorAngularCorrectionAllowed(
        std::hypot(cmd.twist.linear.x,cmd.twist.linear.y),measured_speed));
  }
  if (was_xy_coasting != xy_coast_.active() || was_yaw_coasting != yaw_coast_.active()) {
    RCLCPP_INFO(logger_, "ARRIVAL_COAST xy=%d yaw=%d xy_m=%.6f yaw_deg=%.6f pose_s=%.3f",
      xy_coast_.active(), yaw_coast_.active(), xy, yaw*180.0/M_PI, source_time);
  }
  if (fixed_corridor_refine) {
    if (!corridorRefinementReachable(yawError(corridor_heading,goal_.pose),yaw,
        corridor_terminal && stopped,arrival->yawTolerance())) {
      fail("GOAL_HEADING_UNREACHABLE: ROTATION_FORBIDDEN_IN_NARROW_PASSAGE");
    }
    // No turn command from rest, during translation braking/coast, or at the
    // endpoint. The full coupled command still passes the normal sweep below.
    if (!corridorAngularCorrectionAllowed(std::hypot(cmd.twist.linear.x,cmd.twist.linear.y),measured_speed)) {
      cmd.twist.angular.z=0.;
    }
  }
  if (!safeCommand(pose, cmd.twist, velocity)) {fail("REFINEMENT_BLOCKED: unsafe footprint sweep");}
  xy_settling_.command(std::hypot(cmd.twist.linear.x,cmd.twist.linear.y)<=1e-9,now);
  yaw_settling_.command(std::abs(cmd.twist.angular.z)<=1e-9,now);
  if (within) {arrival_progress_->report(clock_->now().seconds());} else {arrival_progress_->clear();}
  logMetrics(current, velocity, *arrival,
    xy_coast_.active() || yaw_coast_.active() ? "COAST" : (holding_ ? "SETTLING" : "REFINE"));
  return cmd;
}
}  // namespace astribot_s1_path_tracking
PLUGINLIB_EXPORT_CLASS(astribot_s1_path_tracking::ArrivalController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(astribot_s1_path_tracking::ArrivalGoalChecker, nav2_core::GoalChecker)
