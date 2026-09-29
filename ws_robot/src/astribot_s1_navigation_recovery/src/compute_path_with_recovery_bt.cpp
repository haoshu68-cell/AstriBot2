#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "action_msgs/msg/goal_status_array.hpp"
#include "astribot_navigation_msgs/srv/assess_navigation_start.hpp"
#include "astribot_navigation_msgs/srv/plan_candidate.hpp"
#include "astribot_navigation_msgs/srv/plan_start_recovery.hpp"
#include "astribot_s1_path_tracking/arrival_settling.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "rclcpp/rclcpp.hpp"

namespace astribot_s1_navigation_recovery {
// A single NavigateToPose retains its session and original goal through this
// planning recovery. The only motion owner is the supplied Departure child.
class ComputePathWithRecovery : public BT::DecoratorNode {
  using Plan=astribot_navigation_msgs::srv::PlanCandidate;
  using Recovery=astribot_navigation_msgs::srv::PlanStartRecovery;
  using Assess=astribot_navigation_msgs::srv::AssessNavigationStart;
  using Slam=geometry_msgs::msg::PoseWithCovarianceStamped;
  using Clock=std::chrono::steady_clock;
  enum class Stage {PLAN,STOP,REQUEST_RECOVERY,RECOVER,RECHECK,DONE,FAILED};
public:
  ComputePathWithRecovery(const std::string& name,const BT::NodeConfiguration& config)
  :BT::DecoratorNode(name,config) {
    node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    executor_.add_callback_group(group_,node_->get_node_base_interface());
    plan_=node_->create_client<Plan>("/path_tracking/plan_candidate",rmw_qos_profile_services_default,group_);
    recovery_=node_->create_client<Recovery>("/navigation/plan_start_recovery",rmw_qos_profile_services_default,group_);
    assess_=node_->create_client<Assess>("/navigation/assess_start",rmw_qos_profile_services_default,group_);
    rclcpp::SubscriptionOptions options;options.callback_group=group_;
    slam_=node_->create_subscription<Slam>("/slam/pose",rclcpp::SensorDataQoS(),
      [this](Slam::ConstSharedPtr message) {
        const auto source=rclcpp::Time(message->header.stamp).nanoseconds();
        if(source<=source_stamp_)return;
        source_stamp_=source;
        if(stage_!=Stage::STOP||follow_active_)return;
        const auto &p=message->pose.pose;const auto &q=p.orientation;
        if(message->header.frame_id!=recovery_header_.frame_id||
          !std::isfinite(p.position.x)||!std::isfinite(p.position.y)||
          !std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z)||!std::isfinite(q.w)||
          std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01) {
          observation_error_="START_CONNECTION_SLAM_POSE_INVALID";return;
        }
        const double at=seconds(),stamp=source*1e-9;
        const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        // Net displacement limits imply these mean-speed limits over >=0.6 s;
        // no instantaneous speed or observation-age gate is added.
        xy_stop_.observe(stamp,at,{p.position.x,p.position.y},.6,.005/.6,.005);
        yaw_stop_.observe(stamp,at,{yaw},.6,.01/.6,.01,true);
      },options);
    follow_=node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/follow_path/_action/status",rclcpp::QoS(1).reliable().transient_local(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr message) {
        bool active=false;
        for(const auto &s:message->status_list)
          if(s.status==1||s.status==2||s.status==3)active=true;
        if(stage_==Stage::STOP&&(active||follow_active_))resetStop();
        follow_active_=active;follow_seen_=true;
      },options);
  }
  ~ComputePathWithRecovery() override {clearRequests();}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("session"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<bool>("policy_recovery",false,"Recover a perception obstruction"),
      BT::InputPort<double>("required_heading",0.,"Current route heading in goal frame"),
      BT::OutputPort<nav_msgs::msg::Path>("path"),
      BT::OutputPort<nav_msgs::msg::Path>("departure_path")};
  }
  void halt() override {
    clearRequests();BT::DecoratorNode::halt();stage_=Stage::PLAN;
    heading_.reset();policy_started_=false;observation_error_.clear();resetStop();
    // ReactiveFallback may halt a successful planner when KeepSafePath takes
    // over. Keep this execution's total budget; only a new session resets it.
  }
  BT::NodeStatus tick() override {
    if(stage_==Stage::FAILED)return BT::NodeStatus::FAILURE;
    try {
      std::string session;geometry_msgs::msg::PoseStamped goal;
      if(!getInput("session",session)||session.empty()||!getInput("goal",goal))
        return fail("START_CONNECTION_INPUT_INVALID");
      if(session!=session_||goal!=goal_) {
        if(status()==BT::NodeStatus::RUNNING)return fail("START_CONNECTION_EXECUTION_CHANGED");
        session_=session;goal_=goal;started_.reset();steps_=0;empty_departures_=0;policy_started_=false;
      }
      getInput("policy_recovery",policy_mode_);
      if(policy_mode_&&!policy_started_) {
        double heading=0.;getInput("required_heading",heading);
        if(!std::isfinite(heading)||goal_.header.frame_id.empty())return fail("POLICY_RECOVERY_INPUT_INVALID");
        policy_started_=true;heading_=heading;recovery_header_=goal_.header;
        if(!started_)started_=Clock::now();
        setOutput("path",nav_msgs::msg::Path{});stage_=Stage::STOP;resetStop();
        RCLCPP_WARN(node_->get_logger(),"POLICY_OBSTRUCTION stage=STOP session=%s",session_.c_str());
      }
      if(stage_==Stage::DONE)stage_=Stage::PLAN;
      setStatus(BT::NodeStatus::RUNNING);
      executor_.spin_all(std::chrono::milliseconds(1));
      if(!observation_error_.empty())return fail(observation_error_);
      if(started_&&Clock::now()-*started_>=std::chrono::seconds(120))return fail("START_CONNECTION_TIME_BUDGET");
      if(stage_==Stage::PLAN) {
        if(!plan_pending_) {
          // Do not let the reactive fallback revive an old route while this
          // node is recovering. Only a new successful plan publishes a path.
          setOutput("path",nav_msgs::msg::Path{});
          if(!plan_->service_is_ready())return BT::NodeStatus::RUNNING;
          auto request=std::make_shared<Plan::Request>();request->mode=Plan::Request::INITIAL;request->goal=goal_;
          plan_pending_.emplace(plan_->async_send_request(request));return BT::NodeStatus::RUNNING;
        }
        if(plan_pending_->future.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return BT::NodeStatus::RUNNING;
        const auto response=plan_pending_->future.get();plan_pending_.reset();
        if(response->geometry_valid) {
          if(response->path.poses.empty())return fail("START_CONNECTION_EMPTY_PLAN");
          setOutput("path",response->path);stage_=Stage::DONE;return BT::NodeStatus::SUCCESS;
        }
        if(response->failure_code!=Plan::Response::START_CONNECTION_BLOCKED)return fail(response->reason);
        if(!std::isfinite(response->required_start_heading)||response->evaluated_start.header.frame_id.empty())
          return fail("START_CONNECTION_RECOVERY_INPUT_INVALID");
        heading_=response->required_start_heading;recovery_header_=response->evaluated_start.header;
        if(!started_)started_=Clock::now();
        stage_=Stage::STOP;resetStop();
        RCLCPP_WARN(node_->get_logger(),"START_CONNECTION stage=STOP session=%s reason=%s",session_.c_str(),response->reason.c_str());
        // Return first: the containing ReactiveSequence halts the old FollowPath.
        return BT::NodeStatus::RUNNING;
      }
      if(stage_==Stage::STOP) {
        // With no FollowPath status ever observed there is no prior goal to
        // await. Fresh SLAM samples are still mandatory; silence cannot add span.
        if((policy_mode_&&!follow_seen_)||follow_active_||!xy_stop_.evidence().stopped||!yaw_stop_.evidence().stopped)return BT::NodeStatus::RUNNING;
        stage_=Stage::REQUEST_RECOVERY;
      }
      if(stage_==Stage::REQUEST_RECOVERY) {
        if(!recovery_pending_) {
          if(!recovery_->service_is_ready())return BT::NodeStatus::RUNNING;
          auto request=std::make_shared<Recovery::Request>();request->header=recovery_header_;
          request->execution_id=session_;request->required_heading=heading_.value();
          request->use_policy_obstacles=policy_mode_;
          recovery_pending_.emplace(recovery_->async_send_request(request));return BT::NodeStatus::RUNNING;
        }
        if(recovery_pending_->future.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return BT::NodeStatus::RUNNING;
        const auto response=recovery_pending_->future.get();recovery_pending_.reset();
        if(!response->success)return fail(response->reason);
        if(response->path.poses.empty())return fail("START_CONNECTION_EMPTY_DEPARTURE");
        if(response->path.poses.size()==1) {
          if(++empty_departures_>1)return fail("START_CONNECTION_NO_PROGRESS");
          stage_=Stage::RECHECK;return BT::NodeStatus::RUNNING;
        }
        if(steps_>=10)return fail("START_CONNECTION_STEP_BUDGET");
        setOutput("departure_path",response->path);stage_=Stage::RECOVER;
        RCLCPP_INFO(node_->get_logger(),"START_CONNECTION stage=RECOVER session=%s step=%u",session_.c_str(),steps_+1);
      }
      if(stage_==Stage::RECOVER) {
        const auto result=child_node_->executeTick();
        if(result==BT::NodeStatus::RUNNING)return result;
        if(result!=BT::NodeStatus::SUCCESS)return fail("START_CONNECTION_DEPARTURE_FAILED");
        resetChild();++steps_;empty_departures_=0;
        // A completed short step is not evidence that the policy obstruction
        // is cleared. Recompute against current perception before assessment.
        stage_=policy_mode_?Stage::REQUEST_RECOVERY:Stage::RECHECK;
      }
      if(stage_==Stage::RECHECK) {
        if(!assess_pending_) {
          if(!assess_->service_is_ready())return BT::NodeStatus::RUNNING;
          auto request=std::make_shared<Assess::Request>();request->execution_id=session_;request->goal=goal_;
          assess_pending_.emplace(assess_->async_send_request(request));return BT::NodeStatus::RUNNING;
        }
        if(assess_pending_->future.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return BT::NodeStatus::RUNNING;
        const auto response=assess_pending_->future.get();assess_pending_.reset();
        if(response->state==Assess::Response::READY) {
          heading_.reset();stage_=Stage::PLAN;return BT::NodeStatus::RUNNING;
        }
        if(response->state!=Assess::Response::RECOVERY_REQUIRED)return fail(response->reason);
        if(response->evaluated_start.header.frame_id!=recovery_header_.frame_id)
          return fail("START_CONNECTION_RECOVERY_FRAME_CHANGED");
        // The service reads the new actual pose. Keep only the last real
        // connection heading, never a planned departure endpoint as the start.
        recovery_header_=response->evaluated_start.header;stage_=Stage::REQUEST_RECOVERY;
      }
      return BT::NodeStatus::RUNNING;
    } catch(const std::exception &error) {return fail(std::string("START_CONNECTION_ERROR:")+error.what());}
  }
private:
  static double seconds() {return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
  void resetStop() {
    xy_stop_.reset();yaw_stop_.reset();const auto now=seconds();
    xy_stop_.command(true,now);yaw_stop_.command(true,now);
  }
  void clearRequests() {
    if(plan_pending_){plan_->remove_pending_request(plan_pending_->request_id);plan_pending_.reset();}
    if(recovery_pending_){recovery_->remove_pending_request(recovery_pending_->request_id);recovery_pending_.reset();}
    if(assess_pending_){assess_->remove_pending_request(assess_pending_->request_id);assess_pending_.reset();}
  }
  BT::NodeStatus fail(const std::string &reason) {
    clearRequests();resetChild();stage_=Stage::FAILED;
    RCLCPP_ERROR(node_->get_logger(),"START_CONNECTION_FAILED session=%s reason=%s",session_.c_str(),reason.c_str());
    return BT::NodeStatus::FAILURE;
  }
  Stage stage_=Stage::PLAN;
  std::string session_,observation_error_;
  geometry_msgs::msg::PoseStamped goal_;
  std_msgs::msg::Header recovery_header_;
  std::optional<double> heading_;
  std::optional<Clock::time_point> started_;
  unsigned steps_=0,empty_departures_=0;
  int64_t source_stamp_=0;
  bool follow_active_=false,follow_seen_=false,policy_mode_=false,policy_started_=false;
  astribot_s1_path_tracking::ArrivalSettling<2> xy_stop_;
  astribot_s1_path_tracking::ArrivalSettling<1> yaw_stop_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<Plan>::SharedPtr plan_;
  rclcpp::Client<Recovery>::SharedPtr recovery_;
  rclcpp::Client<Assess>::SharedPtr assess_;
  std::optional<rclcpp::Client<Plan>::FutureAndRequestId> plan_pending_;
  std::optional<rclcpp::Client<Recovery>::FutureAndRequestId> recovery_pending_;
  std::optional<rclcpp::Client<Assess>::FutureAndRequestId> assess_pending_;
  rclcpp::Subscription<Slam>::SharedPtr slam_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr follow_;
};

void registerComputePathWithRecovery(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<ComputePathWithRecovery>("ComputePathWithRecovery");
}
} // namespace astribot_s1_navigation_recovery
