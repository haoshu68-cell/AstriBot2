#include "astribot_s1_navigation_recovery/departure_controller.hpp"
#include "astribot_s1_path_tracking/arrival_controller.hpp"
#include "astribot_s1_path_tracking/arrival_settling.hpp"
#include "astribot_s1_path_tracking/arrival_progress.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>

namespace astribot_s1_navigation_recovery {
using namespace astribot_s1_path_tracking;
namespace {
using Steady=std::chrono::steady_clock;
bool finitePose(const geometry_msgs::msg::Pose &pose) {
  const auto &p=pose.position;const auto &q=pose.orientation;
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&
    std::isfinite(norm)&&std::abs(norm-1.)<.01;
}
bool finiteTwist(const geometry_msgs::msg::Twist &v) {
  return std::isfinite(v.linear.x)&&std::isfinite(v.linear.y)&&std::isfinite(v.angular.z);
}
}

struct DepartureController::State {
  rclcpp::Clock::SharedPtr clock;
  std::shared_ptr<tf2_ros::Buffer> tf;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub;
  std::shared_ptr<ArrivalProgress> progress;
  std::mutex mutex;
  nav_msgs::msg::Path path;
  std::string fault,last_phase;
  ArrivalGoalChecker *checker{nullptr};
  uint64_t checker_generation{0};
  double length{0.},heading{0.},direction_x{0.},direction_y{0.},started{-1.},last{-1.},progress_at{-1.},best{0.};
  Steady::time_point started_wall,progress_wall;
  std::optional<Steady::time_point> zero_wall;
  ArrivalSettling<2> xy_stop;
  ArrivalSettling<1> yaw_stop;
  bool active{false},capable{false},external_percentage{false};
  double external_limit{0.};
  double speed{.05},max_distance{2.},position_tolerance{.008},lateral_tolerance{.03},heading_tolerance{.03};
  double position_gain{.8},lateral_gain{1.},lateral_speed{.01},heading_gain{3.},angular_damping{.5},angular_speed{.1};
  double stopped_linear{.01},stopped_angular{.02},timeout{120.},progress_timeout{15.};
  double settle_time{.6},pose_timeout{.5};

  void resetStop() {xy_stop.reset();yaw_stop.reset();zero_wall.reset();}
  void reset() {
    path={};fault.clear();last_phase.clear();
    started=last=progress_at=-1.;best=0.;checker=nullptr;resetStop();if(progress)progress->clear();
  }
  [[noreturn]] void fail(const std::string &reason) {
    fault=reason;resetStop();if(progress)progress->clear();throw nav2_core::PlannerException(reason);
  }
  void phase(const char *name) {
    if(last_phase==name)return;
    last_phase=name;std_msgs::msg::String msg;msg.data=name;phase_pub->publish(msg);
  }
  geometry_msgs::msg::PoseStamped inFrame(const geometry_msgs::msg::PoseStamped &pose,const std::string &frame) {
    if(pose.header.frame_id==frame)return pose;
    geometry_msgs::msg::PoseStamped out;
    const auto transform=tf->lookupTransform(frame,pose.header.frame_id,tf2::TimePointZero);
    tf2::doTransform(pose,out,transform);out.header.stamp=pose.header.stamp;return out;
  }

};

DepartureController::DepartureController():state_(std::make_unique<State>()) {}
DepartureController::~DepartureController()=default;
void DepartureController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) {
  auto node=parent.lock();if(!node)throw std::runtime_error("DepartureController: expired parent");
  auto &s=*state_;s.clock=node->get_clock();s.tf=std::move(tf);
  nav2_util::declare_parameter_if_not_declared(node,"navigation_geometry_mode",rclcpp::ParameterValue("legacy"));
  s.capable=node->get_parameter("use_sim_time").as_bool()&&node->get_parameter("navigation_geometry_mode").as_string()=="fixed_v2";
  if(!s.capable){s.reset();return;}
  const auto parameter=[&](const char *key,double &value) {
    nav2_util::declare_parameter_if_not_declared(node,name+"."+key,rclcpp::ParameterValue(value));
    value=node->get_parameter(name+"."+key).as_double();
    if(!std::isfinite(value)||value<=0.)throw std::invalid_argument("invalid Departure parameter: "+std::string(key));
  };
  parameter("speed_m_s",s.speed);parameter("max_distance_m",s.max_distance);
  parameter("position_tolerance_m",s.position_tolerance);parameter("lateral_tolerance_m",s.lateral_tolerance);
  parameter("heading_tolerance_rad",s.heading_tolerance);parameter("position_gain",s.position_gain);
  parameter("lateral_gain",s.lateral_gain);parameter("lateral_speed_m_s",s.lateral_speed);
  parameter("heading_gain",s.heading_gain);parameter("angular_damping",s.angular_damping);
  parameter("angular_speed_rad_s",s.angular_speed);parameter("stopped_linear_m_s",s.stopped_linear);
  parameter("stopped_angular_rad_s",s.stopped_angular);parameter("timeout_s",s.timeout);
  parameter("progress_timeout_s",s.progress_timeout);
  parameter("settle_time_s",s.settle_time);parameter("pose_timeout_s",s.pose_timeout);
  if(s.speed>.05||s.max_distance>2.||s.position_tolerance>.008||s.lateral_tolerance>.03||s.heading_tolerance>.03||
      s.lateral_speed>.01||s.angular_speed>.1||s.stopped_linear>.01||s.stopped_angular>.02||
      s.settle_time<.6||s.progress_timeout>=s.timeout)
    throw std::invalid_argument("DEPARTURE_BOUNDS_EXCEED_START_MANEUVER");
  s.path_pub=rclcpp::create_publisher<nav_msgs::msg::Path>(node,"path_tracking/active_path",rclcpp::QoS(1).transient_local());
  s.phase_pub=rclcpp::create_publisher<std_msgs::msg::String>(node,"path_tracking/phase",rclcpp::SensorDataQoS());
  s.progress=ArrivalProgress::forNode(node.get());s.reset();
}
void DepartureController::activate() {std::lock_guard<std::mutex> lock(state_->mutex);state_->active=true;}
void DepartureController::deactivate() {std::lock_guard<std::mutex> lock(state_->mutex);state_->active=false;state_->reset();}
void DepartureController::cleanup() {
  deactivate();state_->path_pub.reset();state_->phase_pub.reset();state_->tf.reset();state_->clock.reset();
}
void DepartureController::setPlan(const nav_msgs::msg::Path &path) {
  auto &s=*state_;std::lock_guard<std::mutex> lock(s.mutex);
  if(!s.capable)s.fail("DEPARTURE_REQUIRES_SIM_FIXED_V2");
  if(path==s.path&&!s.path.poses.empty()&&s.fault.empty())return;
  s.reset();
  if(path.header.frame_id.empty()||path.poses.empty()||path.poses.size()>2)s.fail("DEPARTURE_INVALID_PATH");
  for(const auto &p:path.poses)if(p.header.frame_id!=path.header.frame_id||!finitePose(p.pose))s.fail("DEPARTURE_INVALID_POSE");
  const auto &a=path.poses.front().pose;const auto &b=path.poses.back().pose;
  s.heading=tf2::getYaw(a.orientation);
  const double dx=b.position.x-a.position.x,dy=b.position.y-a.position.y;
  s.length=std::hypot(dx,dy);
  if(s.length>std::min(s.max_distance,.2)+1e-9||
      std::abs(std::remainder(tf2::getYaw(b.orientation)-s.heading,2*M_PI))>1e-6)
    s.fail("DEPARTURE_PATH_MUST_BE_SHORT_TRANSLATION_SAME_YAW");
  s.direction_x=s.length>0.?dx/s.length:std::cos(s.heading);
  s.direction_y=s.length>0.?dy/s.length:std::sin(s.heading);
  s.path=path;s.path_pub->publish(path);
}
void DepartureController::setSpeedLimit(const double &value,const bool &percentage) {
  if(!std::isfinite(value)||value<0.)throw std::invalid_argument("DEPARTURE_INVALID_SPEED_LIMIT");
  std::lock_guard<std::mutex> lock(state_->mutex);state_->external_limit=value;state_->external_percentage=percentage;
}

geometry_msgs::msg::TwistStamped DepartureController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker) {
  auto &s=*state_;std::lock_guard<std::mutex> lock(s.mutex);
  if(!s.capable)s.fail("DEPARTURE_REQUIRES_SIM_FIXED_V2");
  auto *checker=dynamic_cast<ArrivalGoalChecker *>(goal_checker);
  if(!checker)s.fail("DEPARTURE_REQUIRES_ARRIVAL_GOAL_CHECKER");
  checker->report(false);
  if(s.checker!=checker||s.checker_generation!=checker->generation()) {
    s.checker=checker;s.checker_generation=checker->generation();s.resetStop();
  }
  if(!s.fault.empty())throw nav2_core::PlannerException(s.fault);
  if(!s.active||s.path.poses.empty())s.fail("DEPARTURE_NO_ACTIVE_PATH");
  if(!finitePose(pose.pose)||!finiteTwist(velocity))s.fail("DEPARTURE_INVALID_FEEDBACK");
  const auto now=s.clock->now();const double seconds=now.seconds();const auto wall=Steady::now();
  const double stamp=rclcpp::Time(pose.header.stamp).seconds();
  auto current=s.inFrame(pose,s.path.header.frame_id);
  const double yaw=tf2::getYaw(current.pose.orientation);
  const auto &anchor=s.path.poses.front().pose.position;const auto &goal=s.path.poses.back().pose.position;
  const double dx=current.pose.position.x-anchor.x,dy=current.pose.position.y-anchor.y;
  const double along=dx*s.direction_x+dy*s.direction_y;
  const double side=-dx*s.direction_y+dy*s.direction_x;
  const double angle=std::remainder(s.heading-yaw,2*M_PI),remaining=s.length-along;
  const double distance=std::hypot(current.pose.position.x-goal.x,current.pose.position.y-goal.y);
  if(std::abs(side)>s.lateral_tolerance||std::abs(angle)>s.heading_tolerance)s.fail("DEPARTURE_DRIFT");
  if(along<-s.position_tolerance||remaining<-s.position_tolerance)s.fail("DEPARTURE_OVERSHOOT");
  if(s.started<0.) {
    if(std::hypot(dx,dy)>.04)s.fail("DEPARTURE_START_MOVED");
    if(std::hypot(velocity.linear.x,velocity.linear.y)>s.stopped_linear||std::abs(velocity.angular.z)>s.stopped_angular)
      s.fail("DEPARTURE_START_NOT_STOPPED");
    s.started=s.progress_at=seconds;s.started_wall=s.progress_wall=wall;
  }
  if(std::chrono::duration<double>(wall-s.started_wall).count()>s.timeout)
    s.fail("DEPARTURE_TIMEOUT");
  if(along>s.best+.005){s.best=along;s.progress_at=seconds;s.progress_wall=wall;}
  if(std::chrono::duration<double>(wall-s.progress_wall).count()>s.progress_timeout)
    s.fail("DEPARTURE_NO_PROGRESS");
  // DeparturePlanner admitted this bounded short segment. Execution follows
  // it to a measured stop; EnsureNavigationStart refreshes geometry/environment
  // and reassesses the actual pose before another segment or normal planning.
  geometry_msgs::msg::TwistStamped command;command.header=pose.header;
  if(distance<=s.position_tolerance) {
    s.phase("DEPARTURE_STOPPING");s.progress->report(seconds);
    const bool measured=std::hypot(velocity.linear.x,velocity.linear.y)<=s.stopped_linear&&std::abs(velocity.angular.z)<=s.stopped_angular;
    if(!measured){s.resetStop();return command;}
    if(!s.zero_wall)s.zero_wall=wall;
    const double stop_now=std::chrono::duration<double>(wall.time_since_epoch()).count();
    s.xy_stop.command(true,stop_now);s.yaw_stop.command(true,stop_now);
    s.xy_stop.observe(stamp,stop_now,{current.pose.position.x,current.pose.position.y},s.settle_time,s.pose_timeout,s.stopped_linear,s.position_tolerance);
    s.yaw_stop.observe(stamp,stop_now,{yaw},s.settle_time,s.pose_timeout,s.stopped_angular,s.heading_tolerance,true);
    if(s.xy_stop.evidence().stopped&&s.yaw_stop.evidence().stopped&&
        std::chrono::duration<double>(wall-*s.zero_wall).count()>=s.settle_time) {
      // This confirms only that this short action stopped. The recovery BT
      // must reassess the actual start before admitting normal navigation.
      checker->report(true);s.phase("DEPARTURE_STOPPED");
    }
    return command;
  }
  s.resetStop();s.progress->clear();
  if(remaining<=0.)s.fail("DEPARTURE_TARGET_LATERAL_UNREACHABLE");
  double cap=s.speed;
  if(s.external_limit>0.)cap=std::min(cap,s.external_percentage?s.speed*s.external_limit/100.:s.external_limit);
  const double speed=std::min(cap,s.position_gain*remaining);
  const double lateral=std::clamp(-s.lateral_gain*side,-s.lateral_speed,s.lateral_speed);
  const double wx=speed*s.direction_x-lateral*s.direction_y;
  const double wy=speed*s.direction_y+lateral*s.direction_x;
  auto &v=command.twist;
  v.linear.x=std::cos(yaw)*wx+std::sin(yaw)*wy;
  v.linear.y=-std::sin(yaw)*wx+std::cos(yaw)*wy;
  const double ratio=std::min(1.,cap/std::max(std::hypot(v.linear.x,v.linear.y),1e-12));v.linear.x*=ratio;v.linear.y*=ratio;
  if(std::hypot(velocity.linear.x,velocity.linear.y)>.003)
    v.angular.z=std::clamp(s.heading_gain*angle-s.angular_damping*velocity.angular.z,-s.angular_speed,s.angular_speed);
  s.phase("DEPARTURE_TRANSLATE");return command;
}
}
PLUGINLIB_EXPORT_CLASS(astribot_s1_navigation_recovery::DepartureController,nav2_core::Controller)
