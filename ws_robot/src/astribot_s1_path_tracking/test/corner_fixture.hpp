#pragma once
#include "astribot_s1_path_tracking/three_phase_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "astribot_s1_path_tracking/arrival_controller.hpp"
#include <iostream>
#include <rcl/time.h>

namespace astribot_s1_path_tracking {
class RecordingInner : public nav2_core::Controller {
public:
  nav_msgs::msg::Path received;
  geometry_msgs::msg::TwistStamped command;
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  void setPlan(const nav_msgs::msg::Path & path) override {received = path;}
  void setSpeedLimit(const double &, const bool &) override {}
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override {return command;}
};
class ThreePhaseControllerTestPeer {
public:
  static rcl_clock_t * budgetClock(ThreePhaseController &c) {return c.budget_clock_.get_clock_handle();}
  static void init(ThreePhaseController & c) {
    c.clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    c.inner_ = std::make_shared<RecordingInner>();
    c.corner_turn_enabled_ = true;
  }
  static void slam(ThreePhaseController &c,const geometry_msgs::msg::PoseStamped &pose) {
    geometry_msgs::msg::PoseWithCovarianceStamped m;m.header=pose.header;m.pose.pose=pose.pose;
    c.receiveSlamPose(m);
  }
  static void received(ThreePhaseController &c,double now) {c.slam_received_=now;}
  static bool stopped(ThreePhaseController &c,double now,double odom_speed=0.) {
    geometry_msgs::msg::PoseStamped unrelated;unrelated.pose.position.x=100.;
    geometry_msgs::msg::Twist velocity;velocity.linear.x=odom_speed;
    c.corner_stop_.command(0.,0.,0.,now);
    return c.cornerStopped();
  }
  static void beginCornerStop(ThreePhaseController &c,double now) {
    c.phase_=Phase::kCornerApproach;c.corner_position_={1.,0.};
    c.corner_active_=true;c.corner_heading_=M_PI/2.;
    c.corner_stop_.command(0.,0.,0.,now);
  }
  static void advanceCorner(ThreePhaseController &c) {
    geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="map";pose.header.stamp=c.clock_->now();
    pose.pose.position.x=1.;pose.pose.orientation.w=1.;geometry_msgs::msg::Twist odom;odom.linear.x=2.;
    c.cornerApproachCommand(pose,pose,odom,nullptr,0.);
  }
  static void costmap(ThreePhaseController &c,const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &map) {c.costmap_ros_=map;c.tf_=map->getTfBuffer();}
  static void monitor(ThreePhaseController &c) {c.observeCornerPose();}
  static auto latest(const ThreePhaseController &c) {return c.slamObservation();}
  static rclcpp::Clock::SharedPtr clock(const ThreePhaseController & c) {return c.clock_;}
  static void time(ThreePhaseController & c,double now) {
    auto * clock=c.clock_->get_clock_handle();
    if(rcl_enable_ros_time_override(clock)!=RCL_RET_OK ||
      rcl_set_ros_time_override(clock,static_cast<rcl_time_point_value_t>(std::llround(now*1e9)))!=RCL_RET_OK) {
      throw std::runtime_error("offline clock override failed");
    }
  }
  static void active(ThreePhaseController & c, std::size_t index) {
    c.has_tick_ = true; c.last_tick_time_ = c.clock_->now();
    c.corner_cursor_ = index; c.corner_active_ = true;
    c.corner_position_ = c.corners_.at(index).position;
    c.corner_heading_ = c.corners_.at(index).outgoing_heading;
    c.phase_ = Phase::kAlignCorner;
  }
  static void diagnosticPose(ThreePhaseController & c) {
    c.has_corner_pose_=true;c.last_corner_pose_.header.frame_id="map";
    c.last_corner_pose_.header.stamp=c.clock_->now();
  }
  static void diagnosticTurn(ThreePhaseController & c) {c.setCornerStage(ThreePhaseController::CornerStage::Turning);}
  static void disableCorners(ThreePhaseController & c) {c.corner_turn_enabled_=false;}
  static std::shared_ptr<ControllerExecution> managed(ThreePhaseController & c) {
    c.execution_=std::make_shared<ControllerExecution>();return c.execution_;
  }
  static void commit(ThreePhaseController & c) {c.applyPendingPlan();}
  static void capture(ThreePhaseController & c) {
    c.phase_=Phase::kCornerApproach;c.corner_position_={1.,0.};
    geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.header.stamp=c.clock_->now();
    p.pose.position.x=1.;p.pose.orientation.w=1.;geometry_msgs::msg::Twist v;
    slam(c,p);c.cornerApproachCommand(p,p,v,nullptr,0.);
  }
  static bool earlyApproach(ThreePhaseController & c,double x,double speed) {
    c.phase_=Phase::kFollow;c.corner_active_=false;c.corner_cursor_=0;
    geometry_msgs::msg::PoseStamped p;p.pose.position.x=x;p.pose.orientation.w=1.;
    return c.maybeStartCorner(p,0.,.002,speed);
  }
  static void plantInit(ThreePhaseController & c,double yaw) {
    c.phase_=Phase::kCornerApproach;c.corner_active_=true;
    c.corner_position_={std::cos(yaw),std::sin(yaw)};c.corner_stop_.reset();
  }
  static geometry_msgs::msg::Twist plantStep(ThreePhaseController & c,double now,double source,
      double source_position,double yaw,double speed) {
    auto * clock=c.clock_->get_clock_handle();
    if(rcl_enable_ros_time_override(clock)!=RCL_RET_OK ||
      rcl_set_ros_time_override(clock,static_cast<rcl_time_point_value_t>(std::llround(now*1e9)))!=RCL_RET_OK) {
      throw std::runtime_error("offline clock override failed");
    }
    geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="map";
    pose.header.stamp=rclcpp::Time(static_cast<int64_t>(std::llround(source*1e9)),RCL_ROS_TIME);
    pose.pose.position.x=source_position*std::cos(yaw);pose.pose.position.y=source_position*std::sin(yaw);
    pose.pose.orientation.z=std::sin(yaw/2.);pose.pose.orientation.w=std::cos(yaw/2.);
    geometry_msgs::msg::Twist velocity;velocity.linear.x=speed;
    auto inner=std::static_pointer_cast<RecordingInner>(c.inner_);inner->command.twist.linear.x=.08;
    slam(c,pose);return c.cornerApproachCommand(pose,pose,velocity,nullptr,std::abs(1.-source_position)).twist;
  }
  static double heading(const ThreePhaseController & c) {return c.start_heading_;}
  static double timer(const ThreePhaseController & c) {return c.phase_started_.seconds();}
  static std::size_t cursor(const ThreePhaseController & c) {return c.corner_cursor_;}
  static void observed(ThreePhaseController & c,double x,double y) {
    c.has_corner_pose_=true;c.last_corner_pose_.header.frame_id="map";
    c.last_corner_pose_.pose.position.x=x;c.last_corner_pose_.pose.position.y=y;
  }
  static bool settling(const ThreePhaseController & c) {return c.corner_reanchor_pending_;}
  static geometry_msgs::msg::Twist hold(ThreePhaseController & c,double x,double y,double yaw) {
    c.corner_position_={1.,0.};geometry_msgs::msg::PoseStamped p;
    p.pose.position.x=x;p.pose.position.y=y;
    p.pose.orientation.z=std::sin(yaw/2.);p.pose.orientation.w=std::cos(yaw/2.);
    p.header.frame_id="map";p.header.stamp=c.clock_->now();slam(c,p);
    return c.cornerRotationCommand(p,{},.5).twist;
  }
  static void gap(ThreePhaseController & c,double seconds) {c.last_tick_time_=c.clock_->now()-rclcpp::Duration::from_seconds(seconds);}
  static void idle(ThreePhaseController & c) {
    c.last_tick_time_ = c.clock_->now() - rclcpp::Duration::from_seconds(1.);
  }
  static const nav_msgs::msg::Path & plan(const ThreePhaseController & c) {return c.plan_;}
  static bool approach(ThreePhaseController & c,std::size_t cursor,double x,double y) {
    c.phase_=Phase::kFollow;c.corner_active_=false;c.corner_cursor_=cursor;
    geometry_msgs::msg::PoseStamped p;p.pose.position.x=x;p.pose.position.y=y;
    return c.maybeStartCorner(p,0.,.002);
  }
  static void observe(ThreePhaseController & c,double x,double yaw,double now,double speed=0.,double wz=0.) {
    geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";
    p.pose.position.x=x;p.pose.orientation.z=std::sin(yaw/2.);p.pose.orientation.w=std::cos(yaw/2.);
    geometry_msgs::msg::Twist v;v.linear.x=speed;v.angular.z=wz;
    p.header.stamp=rclcpp::Time(static_cast<int64_t>(std::llround(now*1e9)),RCL_ROS_TIME);
    slam(c,p);c.observeCornerPose();
  }
  static geometry_msgs::msg::Twist captureBoundary(ThreePhaseController & c) {
    c.corner_position_={1.2,0.};geometry_msgs::msg::PoseStamped p;
    p.pose.position.x=1.2387958175;p.pose.position.y=-.0148100217;
    p.header.frame_id="map";p.header.stamp=c.clock_->now();
    p.pose.orientation.w=1.;slam(c,p);auto inner=std::static_pointer_cast<RecordingInner>(c.inner_);
    inner->command.twist.linear.x=-.003914;inner->command.twist.linear.y=.000897;
    return c.cornerApproachCommand(p,p,{},nullptr,
      std::hypot(p.pose.position.x-1.2,p.pose.position.y)).twist;
  }
  static bool terminal(ThreePhaseController & c,double x,double y,double capture=.3) {
    geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";
    p.pose.position.x=x;p.pose.position.y=y;return c.terminalRefinementAllowed(p,capture);
  }
  static void recovered(ThreePhaseController & c) {c.phase_=Phase::kFollow;c.corner_active_=false;}
  static void completed(ThreePhaseController & c,std::size_t count) {
    c.corner_completed_count_=count;c.corner_cursor_=count;c.updateCornerSegment();
  }
  static nav_msgs::msg::Path segment(const ThreePhaseController & c) {
    return std::static_pointer_cast<RecordingInner>(c.inner_)->received;
  }
};
class ArrivalControllerTestPeer {
public:
  static double startedAt(const ArrivalController &c) {return c.started_at_;}
  static void setup(ArrivalController & c,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap={}) {
    ThreePhaseControllerTestPeer::init(c);
    c.clock_=ThreePhaseControllerTestPeer::clock(c);c.policy_takeover_=true;
    c.costmap_=costmap;if(costmap)c.tf_=costmap->getTfBuffer();
  }

  static geometry_msgs::msg::Twist policyTick(ArrivalController & c, ArrivalGoalChecker & checker,
      double now, const PolicyLease::Message * lease=nullptr) {
    c.arrival_progress_=std::make_shared<ArrivalProgress>();
    c.policy_enabled_=true;c.policy_takeover_=false;c.nominal_speed_=.35;
    checker.clock_=c.clock_;ThreePhaseControllerTestPeer::time(c,now);
    if(lease) {c.policy_lease_.receive(*lease);}
    geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="map";pose.header.stamp=c.clock_->now();
    pose.pose.orientation.w=1.;
    ThreePhaseControllerTestPeer::slam(c,pose);
    return c.computeVelocityCommands(pose,{},&checker).twist;
  }

  static geometry_msgs::msg::Twist corridorTick(ArrivalController & c, ArrivalGoalChecker & checker,
      double now, bool align, bool hold=false) {
    c.arrival_progress_=std::make_shared<ArrivalProgress>();
    c.policy_enabled_=true;c.policy_takeover_=false;c.nominal_speed_=.35;
    checker.clock_=c.clock_;
    auto * clock=c.clock_->get_clock_handle();
    if(rcl_enable_ros_time_override(clock)!=RCL_RET_OK ||
      rcl_set_ros_time_override(clock,static_cast<rcl_time_point_value_t>(std::llround(now*1e9)))!=RCL_RET_OK) {
      throw std::runtime_error("offline arrival clock override failed");
    }
    geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="map";pose.header.stamp=c.clock_->now();
    pose.pose.position.x=1.;pose.pose.position.y=.5;pose.pose.orientation.w=1.;
    PolicyLease::Message lease;lease.stamp=c.clock_->now();lease.sequence=static_cast<uint64_t>(now*1000);
    lease.lease_s=.3;lease.max_linear_speed=.1;lease.max_angular_speed=.4;lease.hold=hold;
    lease.alignment_required=align;lease.centering_required=!align;c.policy_lease_.receive(lease);
    auto request=std::make_shared<ArrivalController::CorridorAlignment>();
    request->stamp=c.clock_->now();request->lease_s=.3;request->reference_path=c.tracking_path_;
    request->anchor=pose;request->target=pose;request->target.pose.position.y+=.1;
    request->centering_required=!align;
    if(align) {request->anchor.pose.orientation.z=std::sin(.25);request->anchor.pose.orientation.w=std::cos(.25);}
    c.corridor_alignment_=request;c.corridor_received_=std::chrono::steady_clock::now();
    ThreePhaseControllerTestPeer::slam(c,pose);
    return c.computeVelocityCommands(pose,{},&checker).twist;
  }

  static void referenceBrake(ArrivalController & c) {c.braking_model_="reference_stop";}
  static void hardwareBrake(ArrivalController & c) {
    c.braking_model_="position_hold";
    c.linear_offset_.configure({0.,.053,.093,.161,.249},{0.,.004,.004,.008,.012});
  }
  static double stopDistance(ArrivalController & c,double speed) {return c.cornerStoppingDistance(speed);}
  static void refine(ArrivalController & c) {c.refining_=true;}
  static bool refining(const ArrivalController & c) {return c.refining_;}
  static geometry_msgs::msg::Twist limited(double limit,bool percentage) {
    ArrivalController c;ThreePhaseControllerTestPeer::init(c);c.nominal_speed_=.35;
    c.setSpeedLimit(limit,percentage);geometry_msgs::msg::Twist command;
    command.linear.x=.03;command.linear.y=.04;command.angular.z=.4;
    c.limitCornerTranslation(command);return command;
  }
};

}

nav_msgs::msg::Path path(std::initializer_list<astribot_s1_path_tracking::PlanarPoint> points,
  int stamp, double final_yaw=0.) {
  nav_msgs::msg::Path p;p.header.frame_id="map";p.header.stamp.sec=stamp;
  for (auto point:points) {
    geometry_msgs::msg::PoseStamped pose;pose.header=p.header;
    pose.pose.position.x=point.x;pose.pose.position.y=point.y;pose.pose.orientation.w=1.;
    p.poses.push_back(pose);
  }
  p.poses.back().pose.orientation.z=std::sin(final_yaw/2.);
  p.poses.back().pose.orientation.w=std::cos(final_yaw/2.);
  return p;
}
