#include <chrono>
#include <cmath>
#include <optional>
#include <thread>
#include <action_msgs/msg/goal_status_array.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/control_node.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_behavior_tree/bt_conversions.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "astribot_s1_path_tracking/arrival_settling.hpp"
#include "astribot_s1_path_tracking/envelope_evidence.hpp"
#include "astribot_s1_path_tracking/route_commit.hpp"
#include "astribot_s1_path_tracking/workstation_alignment_core.hpp"

namespace astribot_s1_path_tracking {
// Owns only the planning-mode handover. Both children use controller_server;
// this node never publishes velocity and never substitutes a target station.
class WorkstationApproach : public BT::ControlNode {
  using Clock=std::chrono::steady_clock;
  using Service=astribot_navigation_msgs::srv::ResolveRoute;
  using Slam=geometry_msgs::msg::PoseWithCovarianceStamped;
  using Maps=astribot_slam_msgs::msg::HeightSliceMaps;
  using Envelope=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
  using Status=action_msgs::msg::GoalStatus;
  enum class Stage {FOLLOW,STOP,COMMIT,ALIGN,FINISH,DONE};
public:
  WorkstationApproach(const std::string& name,const BT::NodeConfiguration& config)
  :BT::ControlNode(name,config) {
    node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    cfg_=loadWorkstationConfig(node_);
    group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    executor_.add_callback_group(group_,node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options;options.callback_group=group_;
    slam_sub_=node_->create_subscription<Slam>("/slam/pose",rclcpp::SensorDataQoS(),
      [this](Slam::ConstSharedPtr msg) {
        if(slam_&&rclcpp::Time(msg->header.stamp)<=rclcpp::Time(slam_->header.stamp))return;
        slam_=msg;
        if(stage_!=Stage::STOP)return;
        const auto &p=msg->pose.pose;const double stamp=rclcpp::Time(msg->header.stamp).seconds(),at=seconds();
        xy_stop_.observe(stamp,at,{p.position.x,p.position.y},cfg_.settle_time,
          cfg_.stopped_linear_velocity,cfg_.xy_goal_tolerance*cfg_.settle_drift_ratio);
        yaw_stop_.observe(stamp,at,{tf2::getYaw(p.orientation)},cfg_.settle_time,
          cfg_.stopped_angular_velocity,cfg_.yaw_goal_tolerance*cfg_.settle_drift_ratio,true);
      },options);
    maps_sub_=node_->create_subscription<Maps>("/height_maps/snapshot",rclcpp::QoS(1).reliable().transient_local(),
      [this](Maps::ConstSharedPtr msg) {
        if(!maps_||rclcpp::Time(msg->header.stamp)>rclcpp::Time(maps_->header.stamp))maps_=msg;
      },options);
    envelope_sub_=node_->create_subscription<Envelope>("/navigation/envelope_v2",10,
      [this](Envelope::ConstSharedPtr msg){envelope_.accept(msg,node_->now().nanoseconds(),Clock::now());},options);
    status_sub_=node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/follow_path/_action/status",rclcpp::QoS(1).reliable().transient_local(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
        follow_terminal_=true;
        for(const auto &s:msg->status_list)
          if(s.status==Status::STATUS_ACCEPTED||s.status==Status::STATUS_EXECUTING||s.status==Status::STATUS_CANCELING)
            follow_terminal_=false;
        status_received_=Clock::now();
      },options);
    client_=node_->create_client<Service>("/navigation_policy/resolve_route",rmw_qos_profile_services_default,group_);
  }
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("session"),BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<nav_msgs::msg::Path>("path"),BT::OutputPort<nav_msgs::msg::Path>("alignment_path")};
  }
  void halt() override {
    BT::ControlNode::halt();
    clearRequest();
    if(committed_||stage_==Stage::COMMIT) {
      bool finished=false;const auto deadline=Clock::now()+std::chrono::seconds(2);
      while(Clock::now()<deadline) {
        auto result=client_->async_send_request(routeRequest(Service::Request::FINISH));
        const auto status=executor_.spin_until_future_complete(result.future,deadline-Clock::now());
        client_->remove_pending_request(result.request_id);
        if(status!=rclcpp::FutureReturnCode::SUCCESS)break;
        const auto response=result.future.get();
        if(response->disposition==Service::Response::FINISHED){finished=true;break;}
        if(response->disposition==Service::Response::BLOCKED) {
          RCLCPP_ERROR(node_->get_logger(),"WORKSTATION_FINISH_REJECTED reason=%s",response->reason.c_str());break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if(!finished)
        RCLCPP_ERROR(node_->get_logger(),"WORKSTATION_FINISH_UNCONFIRMED session=%s",session_.c_str());
    }
    committed_=false;stage_=Stage::FOLLOW;session_.clear();xy_stop_.reset();yaw_stop_.reset();
  }
  BT::NodeStatus tick() override {
    if(childrenCount()!=2)throw BT::RuntimeError("WorkstationApproach requires navigation and alignment children");
    std::string execution;getInput("session",execution);
    if(!session_.empty()&&execution!=session_)halt();
    session_=execution;
    setStatus(BT::NodeStatus::RUNNING);
    executor_.spin_all(std::chrono::milliseconds(1));
    if(stage_==Stage::FOLLOW) {
      try {return children_nodes_[0]->executeTick();}
      catch(const ObstructionDeadline&) {
        getInput("session",session_);getInput("goal",goal_);getInput("path",reference_);
        requirePose();
        if(std::hypot(slam_->pose.pose.position.x-goal_.pose.position.x,
            slam_->pose.pose.position.y-goal_.pose.position.y)>.50)throw;
        // Humble FollowPath::halt requests cancellation, but only logs a wait
        // failure. Observe its terminal action status before the second child.
        haltChild(0);stage_=Stage::STOP;started_=stage_started_=Clock::now();
        xy_stop_.command(true,seconds());yaw_stop_.command(true,seconds());
        RCLCPP_INFO(node_->get_logger(),"WORKSTATION_APPROACH stage=STOP session=%s",session_.c_str());
        return BT::NodeStatus::RUNNING;
      }
    }
    try {
      if(stage_==Stage::DONE)return outcome_;
      if(stage_!=Stage::FINISH && Clock::now()-started_>std::chrono::duration<double>(cfg_.refine_timeout))
        throw BT::RuntimeError("WORKSTATION_APPROACH_TIMEOUT");
      if(stage_==Stage::STOP) {
        pollRoute(Service::Request::NORMAL);
        if(Clock::now()-stage_started_>std::chrono::seconds(10))throw BT::RuntimeError("WORKSTATION_HANDOVER_STOP_TIMEOUT");
        if(!follow_terminal_||status_received_<stage_started_||!xy_stop_.evidence().stopped||!yaw_stop_.evidence().stopped)
          return BT::NodeStatus::RUNNING;
        assess();clearRequest();stage_=Stage::COMMIT;stage_started_=Clock::now();
      }
      if(stage_==Stage::COMMIT) {
        if(!pollRoute(Service::Request::WORKSTATION_ALIGN))return BT::NodeStatus::RUNNING;
        committed_=true;stage_=Stage::ALIGN;
        RCLCPP_INFO(node_->get_logger(),"WORKSTATION_APPROACH stage=ALIGN session=%s",session_.c_str());
      }
      if(stage_==Stage::ALIGN) {
        // Keep the same policy execution context alive; no old-path guard runs
        // here. A policy rejection still terminates this parent navigation.
        pollRoute(Service::Request::WORKSTATION_ALIGN);
        const auto result=children_nodes_[1]->executeTick();
        if(result==BT::NodeStatus::RUNNING)return result;
        outcome_=result;haltChild(1);clearRequest();stage_=Stage::FINISH;stage_started_=Clock::now();
      }
      if(stage_==Stage::FINISH) {
        if(!pollRoute(Service::Request::FINISH))return BT::NodeStatus::RUNNING;
        committed_=false;stage_=Stage::DONE;return outcome_;
      }
    } catch(const ObstructionDeadline&) {
      halt();throw;
    } catch(const std::exception& error) {
      RCLCPP_ERROR(node_->get_logger(),"WORKSTATION_APPROACH_FAILED session=%s reason=%s",session_.c_str(),error.what());
      halt();outcome_=BT::NodeStatus::FAILURE;stage_=Stage::DONE;return outcome_;
    }
    return BT::NodeStatus::RUNNING;
  }
private:
  static double seconds() {return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
  void requirePose() const {
    if(!slam_)throw BT::RuntimeError("WORKSTATION_SLAM_POSE_MISSING");
    const auto&p=slam_->pose.pose;const auto&q=p.orientation;
    if(slam_->header.frame_id!=goal_.header.frame_id||goal_.header.frame_id.empty())
      throw BT::RuntimeError("WORKSTATION_SLAM_GOAL_FRAME_MISMATCH");
    if(!std::isfinite(p.position.x)||!std::isfinite(p.position.y)||
        !std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z)||!std::isfinite(q.w)||
        std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.001)
      throw BT::RuntimeError("WORKSTATION_SLAM_POSE_INVALID");
  }
  void assess() {
    requirePose();
    const auto observed=envelope_.sample(node_->now().nanoseconds()).message;
    if(!maps_||!observed)throw BT::RuntimeError("WORKSTATION_LAYERED_INPUT_MISSING");
    if(maps_->header.frame_id!=slam_->header.frame_id)throw BT::RuntimeError("WORKSTATION_MAP_SLAM_FRAME_MISMATCH");
    const auto& p=slam_->pose.pose;const auto& g=goal_.pose;
    if(std::hypot(p.position.x-g.position.x,p.position.y-g.position.y)>.50)
      throw BT::RuntimeError("WORKSTATION_OUTSIDE_CAPTURE_RADIUS");
    LayeredCollisionSnapshot snapshot(maps_,observed,slam_->header.frame_id);
    const auto result=evaluateWorkstationAlignment(snapshot,
      {p.position.x,p.position.y,tf2::getYaw(p.orientation)},
      {g.position.x,g.position.y,tf2::getYaw(g.orientation)},cfg_);
    if(!result.clear) {
      if(result.reason=="WORKSTATION_GOAL_LAYER_COLLISION_OR_UNKNOWN"||
          result.reason=="WORKSTATION_SWEEP_COLLISION_OR_UNKNOWN")throw ObstructionDeadline(result.reason);
      throw BT::RuntimeError(result.reason);
    }
    nav_msgs::msg::Path path;path.header=slam_->header;
    geometry_msgs::msg::PoseStamped current;current.header=slam_->header;current.pose=p;
    path.poses={current,goal_};setOutput("alignment_path",path);
    RCLCPP_INFO(node_->get_logger(),"WORKSTATION_APPROACH stage=ASSESSED map=%s geometry=%s prediction_s=%.3f steps=%zu",
      snapshot.revision().c_str(),snapshot.geometryHash().c_str(),result.prediction_s,result.steps);
  }
  Service::Request::SharedPtr routeRequest(uint8_t mode) const {
    auto request=std::make_shared<Service::Request>();request->mode=mode;
    request->allow_detour=true;request->session_id=session_;request->reference_path=reference_;request->goal=goal_;
    return request;
  }
  bool pollRoute(uint8_t mode) {
    if(pending_) {
      if(pending_->future.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
        const auto response=pending_->future.get();pending_.reset();
        if(response->disposition==Service::Response::BLOCKED &&
            !(mode==Service::Request::NORMAL&&response->reason_code==Service::Response::OBSTRUCTION_DEADLINE))
          throw BT::RuntimeError(response->reason);
        if((mode==Service::Request::WORKSTATION_ALIGN&&response->disposition==Service::Response::ALIGNMENT_COMMITTED) ||
            (mode==Service::Request::FINISH&&response->disposition==Service::Response::FINISHED) ||
            mode==Service::Request::NORMAL) {
          last_reply_=Clock::now();return true;
        }
      } else if(Clock::now()-sent_at_>std::chrono::seconds(2))throw BT::RuntimeError("WORKSTATION_POLICY_REPLY_TIMEOUT");
    }
    if(!pending_) {
      if(!client_->service_is_ready())throw BT::RuntimeError("WORKSTATION_POLICY_UNAVAILABLE");
      pending_.emplace(client_->async_send_request(routeRequest(mode)));sent_at_=Clock::now();
    }
    if(Clock::now()-stage_started_>std::chrono::seconds(2)&&(stage_==Stage::COMMIT||stage_==Stage::FINISH))
      throw BT::RuntimeError("WORKSTATION_POLICY_COMMIT_TIMEOUT");
    if(stage_==Stage::ALIGN&&Clock::now()-last_reply_>std::chrono::seconds(2))
      throw BT::RuntimeError("WORKSTATION_POLICY_CONTEXT_LOST");
    return false;
  }
  void clearRequest() {if(pending_){client_->remove_pending_request(pending_->request_id);pending_.reset();}}
  Stage stage_=Stage::FOLLOW;BT::NodeStatus outcome_=BT::NodeStatus::FAILURE;
  bool committed_=false,follow_terminal_=false;
  std::string session_;geometry_msgs::msg::PoseStamped goal_;nav_msgs::msg::Path reference_;
  WorkstationConfig cfg_;ArrivalSettling<2> xy_stop_;ArrivalSettling<1> yaw_stop_;
  Clock::time_point started_,stage_started_,status_received_,sent_at_,last_reply_;
  Slam::ConstSharedPtr slam_;Maps::ConstSharedPtr maps_;EnvelopeEvidence envelope_;
  rclcpp::Node::SharedPtr node_;rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<Slam>::SharedPtr slam_sub_;rclcpp::Subscription<Maps>::SharedPtr maps_sub_;
  rclcpp::Subscription<Envelope>::SharedPtr envelope_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Client<Service>::SharedPtr client_;
  std::optional<rclcpp::Client<Service>::FutureAndRequestId> pending_;
};
void registerWorkstationApproach(BT::BehaviorTreeFactory& factory) {
  factory.registerNodeType<WorkstationApproach>("WorkstationApproach");
}
} // namespace astribot_s1_path_tracking
