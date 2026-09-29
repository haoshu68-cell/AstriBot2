#include "astribot_s1_path_tracking/workstation_alignment_controller.hpp"
#include "astribot_s1_path_tracking/workstation_alignment_core.hpp"
#include "astribot_s1_path_tracking/arrival_controller.hpp"
#include "astribot_s1_path_tracking/arrival_progress.hpp"
#include "astribot_s1_path_tracking/arrival_settling.hpp"
#include "astribot_s1_path_tracking/layered_collision_reader.hpp"
#include "astribot_navigation_msgs/msg/motion_constraint.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <mutex>

namespace astribot_s1_path_tracking {
namespace {
using Steady=std::chrono::steady_clock;
double steadySeconds() {return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();}
bool validPose(const geometry_msgs::msg::Pose &pose) {
  const auto &p=pose.position;const auto &q=pose.orientation;
  const double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&std::isfinite(n)&&std::abs(n-1.)<.01;
}
WorkstationPose pose2d(const geometry_msgs::msg::Pose &p) {return {p.position.x,p.position.y,tf2::getYaw(p.orientation)};}
}
struct WorkstationAlignmentController::State {
  using Constraint=astribot_navigation_msgs::msg::MotionConstraint;
  std::mutex mutex;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  WorkstationConfig cfg;WorkstationMotionState motion;
  LayeredCollisionReader layers;
  geometry_msgs::msg::PoseStamped slam,goal;
  double received{-1.},started{-1.},last_tick{-1.},last_log{-1.},zero_since{-1.};
  bool active{false},observed{false},have_goal{false},mode_seen{false},have_constraint{false},admitted{false};
  bool external_percentage{false};double external_limit{};
  std::string observation_error,constraint_error,phase_name,geometry_hash;
  uint64_t geometry_epoch{};
  Constraint constraint;
  ArrivalSettling<2> xy_stop;ArrivalSettling<1> yaw_stop;
  std::shared_ptr<ArrivalProgress> progress;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_sub;
  rclcpp::Subscription<Constraint>::SharedPtr constraint_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  void resetStop() {xy_stop.reset();yaw_stop.reset();zero_since=-1.;}
  void reset() {
    motion={};have_goal=admitted=mode_seen=false;started=last_tick=last_log=-1.;
    geometry_hash.clear();phase_name.clear();resetStop();if(progress)progress->clear();
  }
  [[noreturn]] void fail(const std::string &reason) {
    if(progress)progress->clear();
    throw nav2_core::PlannerException(reason);
  }
  void phase(const char *name) {
    if(phase_name==name)return;
    phase_name=name;std_msgs::msg::String msg;msg.data=name;phase_pub->publish(msg);
    RCLCPP_INFO(node->get_logger(),"WORKSTATION_ALIGNMENT phase=%s",name);
  }
  void receivePose(const geometry_msgs::msg::PoseWithCovarianceStamped &msg) {
    const auto stamp=rclcpp::Time(msg.header.stamp).nanoseconds();
    if(observed&&stamp<=rclcpp::Time(slam.header.stamp).nanoseconds())return;
    slam.header=msg.header;slam.pose=msg.pose.pose;observed=true;received=steadySeconds();
    observation_error=msg.header.frame_id=="map"&&validPose(msg.pose.pose)?"":"WORKSTATION_SLAM_POSE_INVALID";
    if(!observation_error.empty()) {resetStop();return;}
    if(zero_since<0.||received<zero_since)return;
    const double source=double(stamp)*1e-9;
    xy_stop.observe(source,received,{slam.pose.position.x,slam.pose.position.y},cfg.settle_time,
      cfg.stopped_linear_velocity,cfg.settle_drift_ratio*cfg.xy_goal_tolerance);
    yaw_stop.observe(source,received,{tf2::getYaw(slam.pose.orientation)},cfg.settle_time,
      cfg.stopped_angular_velocity,cfg.settle_drift_ratio*cfg.yaw_goal_tolerance,true);
  }
};
WorkstationAlignmentController::WorkstationAlignmentController():state_(std::make_unique<State>()) {}
WorkstationAlignmentController::~WorkstationAlignmentController()=default;
void WorkstationAlignmentController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name,std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) {
  auto &s=*state_;s.node=parent.lock();s.costmap=std::move(map);s.cfg=loadWorkstationConfig(s.node,name);
  s.layers.configure(s.node,s.costmap);s.progress=ArrivalProgress::forNode(s.node.get());
  s.phase_pub=rclcpp::create_publisher<std_msgs::msg::String>(s.node,"path_tracking/phase",rclcpp::SensorDataQoS());
  s.path_pub=rclcpp::create_publisher<nav_msgs::msg::Path>(s.node,"path_tracking/active_path",rclcpp::QoS(1).transient_local());
  s.slam_sub=s.node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose",rclcpp::SensorDataQoS(),
    [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
      auto &state=*state_;std::lock_guard<std::mutex> lock(state.mutex);
      if(msg->header.stamp.sec<0||msg->header.stamp.nanosec>=1000000000u) {
        state.observation_error="WORKSTATION_SLAM_STAMP_INVALID";state.resetStop();return;
      }
      state.receivePose(*msg);
    });
  s.constraint_sub=s.node->create_subscription<State::Constraint>("/navigation_policy/constraint",10,
    [this](State::Constraint::ConstSharedPtr msg) {
      auto &state=*state_;std::lock_guard<std::mutex> lock(state.mutex);
      if(msg->stamp.sec<0||msg->stamp.nanosec>=1000000000u) {
        state.constraint_error="WORKSTATION_CONSTRAINT_STAMP_INVALID";return;
      }
      if(state.have_constraint) {
        if(rclcpp::Time(msg->stamp)<rclcpp::Time(state.constraint.stamp))return;
        if(msg->epoch==state.constraint.epoch&&msg->sequence<=state.constraint.sequence)return;
      }
      state.constraint=*msg;state.have_constraint=true;
      state.constraint_error=std::isfinite(msg->max_linear_speed)&&msg->max_linear_speed>=0.&&
        std::isfinite(msg->max_angular_speed)&&msg->max_angular_speed>=0.?"":"WORKSTATION_CONSTRAINT_INVALID";
    });
  s.reset();
}
void WorkstationAlignmentController::activate() {std::lock_guard<std::mutex> lock(state_->mutex);state_->active=true;}
void WorkstationAlignmentController::deactivate() {std::lock_guard<std::mutex> lock(state_->mutex);state_->active=false;state_->reset();}
void WorkstationAlignmentController::cleanup() {
  deactivate();auto &s=*state_;s.slam_sub.reset();s.constraint_sub.reset();s.layers.cleanup();
  s.phase_pub.reset();s.path_pub.reset();s.costmap.reset();s.node.reset();s.observed=s.have_constraint=false;
}
void WorkstationAlignmentController::setPlan(const nav_msgs::msg::Path &path) {
  auto &s=*state_;std::lock_guard<std::mutex> lock(s.mutex);
  if(path.header.frame_id!="map"||path.poses.size()!=2) s.fail("WORKSTATION_PLAN_MUST_BE_CURRENT_AND_GOAL");
  for(const auto &p:path.poses)if(p.header.frame_id!=path.header.frame_id||!validPose(p.pose))s.fail("WORKSTATION_PLAN_POSE_INVALID");
  s.reset();s.goal=path.poses.back();s.have_goal=true;s.started=steadySeconds();
  const auto snapshot=s.layers.snapshot(path.poses.front());
  s.geometry_hash=snapshot->geometryHash();s.geometry_epoch=snapshot->envelopeEpoch();
  s.path_pub->publish(path);
}
void WorkstationAlignmentController::setSpeedLimit(const double &limit,const bool &percentage) {
  auto &s=*state_;std::lock_guard<std::mutex> lock(s.mutex);
  if(!std::isfinite(limit)||limit<0.||(percentage&&limit>100.))s.fail("WORKSTATION_SPEED_LIMIT_INVALID");
  s.external_limit=limit;s.external_percentage=percentage;
}
geometry_msgs::msg::TwistStamped WorkstationAlignmentController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped&,const geometry_msgs::msg::Twist&,nav2_core::GoalChecker *checker) {
  auto &s=*state_;std::lock_guard<std::mutex> lock(s.mutex);
  auto *arrival=dynamic_cast<ArrivalGoalChecker*>(checker);
  if(!arrival)s.fail("WORKSTATION_REQUIRES_ARRIVAL_GOAL_CHECKER");
  arrival->report(false);s.progress->clear();
  if(!s.active||!s.have_goal)s.fail("WORKSTATION_NO_ACTIVE_GOAL");
  if(!s.observed)s.fail("WORKSTATION_SLAM_POSE_MISSING");
  if(!s.observation_error.empty())s.fail(s.observation_error);
  if(!s.constraint_error.empty())s.fail(s.constraint_error);
  if(s.slam.header.frame_id!=s.goal.header.frame_id)s.fail("WORKSTATION_FRAME_MISMATCH");
  if(arrival->xyTolerance()!=s.cfg.xy_goal_tolerance||arrival->yawTolerance()!=s.cfg.yaw_goal_tolerance||
      arrival->stoppedLinear()!=s.cfg.stopped_linear_velocity||arrival->stoppedAngular()!=s.cfg.stopped_angular_velocity)
    s.fail("WORKSTATION_SHARED_GOAL_CONFIG_MISMATCH");
  const double now=steadySeconds();
  if(now-s.started>=s.cfg.refine_timeout)s.fail("WORKSTATION_ALIGNMENT_TIMEOUT");
  const double dt=s.last_tick<0.?1./s.cfg.control_frequency:now-s.last_tick;s.last_tick=now;
  const auto current=pose2d(s.slam.pose),goal=pose2d(s.goal.pose);
  geometry_msgs::msg::TwistStamped output;output.header.stamp=s.node->now();output.header.frame_id=s.costmap->getBaseFrameID();
  if(s.mode_seen&&s.have_constraint&&!s.constraint.workstation_alignment)s.fail("WORKSTATION_MODE_REVOKED");
  if(!s.have_constraint||!s.constraint.workstation_alignment||s.constraint.hold||
      s.constraint.max_linear_speed<=0.||s.constraint.max_angular_speed<=0.) {
    // The outgoing request is zero, but a brief policy stop does not erase
    // the chassis' ongoing jerk-limited braking before permission resumes.
    workstationCommand(current,goal,s.motion,s.cfg,dt,true);
    s.resetStop();s.phase("WORKSTATION_WAIT_CONSTRAINT");return output;
  }
  s.mode_seen=true;
  auto cfg=s.cfg;cfg.max_linear_speed=std::min(cfg.max_linear_speed,s.constraint.max_linear_speed);
  cfg.max_angular_speed=std::min(cfg.max_angular_speed,s.constraint.max_angular_speed);
  if(s.external_limit>0.)cfg.max_linear_speed=std::min(cfg.max_linear_speed,s.external_percentage?
      s.cfg.max_linear_speed*s.external_limit/100.:s.external_limit);
  const auto snapshot=s.layers.snapshot(s.slam);
  if(snapshot->geometryHash()!=s.geometry_hash||snapshot->envelopeEpoch()!=s.geometry_epoch)s.fail("WORKSTATION_GEOMETRY_CHANGED");
  if(!s.admitted) {
    const auto result=evaluateWorkstationAlignment(*snapshot,current,goal,cfg);
    if(!result.clear)s.fail(result.reason);
    s.admitted=true;
  }
  auto next=s.motion;auto command=workstationCommand(current,goal,next,cfg,dt);
  // A newly tightened policy cap limits the outgoing request immediately.
  // Keep the pre-limit motion state for the stopping prediction: lowering a
  // request does not make the already moving chassis lose its inertia.
  const double speed=std::hypot(command[0],command[1]),angular=std::abs(command[2]);
  const double ratio=std::min({1.,speed>0.?cfg.max_linear_speed/speed:1.,
    angular>0.?cfg.max_angular_speed/angular:1.});
  if(ratio<1.)for(auto &value:command)value*=ratio;
  if(!workstationStoppingClear(*snapshot,current,next,cfg))s.fail("WORKSTATION_EXECUTION_SWEEP_COLLISION_OR_UNKNOWN");
  if(ratio<1.) {
    auto limited=next;limited.velocity=command;for(auto &value:limited.acceleration)value*=ratio;
    if(!workstationStoppingClear(*snapshot,current,limited,cfg))s.fail("WORKSTATION_LIMITED_SWEEP_COLLISION_OR_UNKNOWN");
  }
  s.motion=next;
  output.twist.linear.x=command[0];output.twist.linear.y=command[1];output.twist.angular.z=command[2];
  const bool zero=std::hypot(command[0],command[1])<1e-9&&std::abs(command[2])<1e-9;
  if(zero) {
    if(s.zero_since<0.)s.zero_since=now;
    s.xy_stop.command(true,s.zero_since);s.yaw_stop.command(true,s.zero_since);
  } else s.resetStop();
  const double xy=std::hypot(goal.x-current.x,goal.y-current.y),yaw=std::abs(std::remainder(goal.yaw-current.yaw,2.*M_PI));
  const bool within=xy<=cfg.xy_goal_tolerance&&yaw<=cfg.yaw_goal_tolerance;
  const bool done=zero&&within&&s.xy_stop.evidence().stopped&&s.yaw_stop.evidence().stopped;
  if(zero&&within)s.progress->report(s.node->now().seconds());
  arrival->report(done);s.phase(done?"WORKSTATION_DONE":zero?"WORKSTATION_SETTLING":"WORKSTATION_ALIGNING");
  if(done||s.last_log<0.||now-s.last_log>=1.) {
    s.last_log=now;
    RCLCPP_INFO(s.node->get_logger(),"WORKSTATION_METRICS source=slam_pose xy_m=%.9f yaw_rad=%.9f stop_xy=%d stop_yaw=%d map=%s geometry=%s epoch=%lu",
      xy,yaw,s.xy_stop.evidence().stopped,s.yaw_stop.evidence().stopped,snapshot->revision().c_str(),s.geometry_hash.c_str(),
      static_cast<unsigned long>(s.geometry_epoch));
  }
  return output;
}
} // namespace astribot_s1_path_tracking
PLUGINLIB_EXPORT_CLASS(astribot_s1_path_tracking::WorkstationAlignmentController,nav2_core::Controller)
