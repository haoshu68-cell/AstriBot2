#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <optional>
#include <string>
#include <thread>

#include "astribot_navigation_msgs/srv/resolve_route.hpp"
#include "astribot_s1_path_tracking/route_commit.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/control_node.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "rclcpp/rclcpp.hpp"

namespace astribot_s1_navigation_recovery {
// Owns the planning-mode handover only. Both normal tracking and departure
// remain FollowPath actions, never concurrent velocity publishers.
class RecoverPolicyObstruction : public BT::ControlNode {
  using Service=astribot_navigation_msgs::srv::ResolveRoute;
  using Slam=geometry_msgs::msg::PoseWithCovarianceStamped;
  using Clock=std::chrono::steady_clock;
  enum class Stage {FOLLOW,COMMIT,RECOVER,RESUME,FAILED};
public:
  RecoverPolicyObstruction(const std::string &name,const BT::NodeConfiguration &config)
  :BT::ControlNode(name,config) {
    node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    executor_.add_callback_group(group_,node_->get_node_base_interface());
    client_=node_->create_client<Service>("/navigation_policy/resolve_route",rmw_qos_profile_services_default,group_);
    rclcpp::SubscriptionOptions options;options.callback_group=group_;
    slam_sub_=node_->create_subscription<Slam>("/slam/pose",rclcpp::SensorDataQoS(),
      [this](Slam::ConstSharedPtr message) {
        if(!slam_||rclcpp::Time(message->header.stamp)>rclcpp::Time(slam_->header.stamp))slam_=message;
      },options);
  }
  ~RecoverPolicyObstruction() override {clearRequest();}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("session"),BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<nav_msgs::msg::Path>("path"),BT::OutputPort<double>("required_heading")};
  }
  void halt() override {
    BT::ControlNode::halt();finishMode();stage_=Stage::FOLLOW;
    // Keep the same execution's elapsed recovery budget across retries/halts.
  }
  BT::NodeStatus tick() override {
    if(childrenCount()!=2)throw BT::RuntimeError("RecoverPolicyObstruction requires navigation and recovery children");
    if(stage_==Stage::FAILED)return BT::NodeStatus::FAILURE;
    std::string session;geometry_msgs::msg::PoseStamped goal;
    if(!getInput("session",session)||session.empty()||!getInput("goal",goal))return fail("POLICY_RECOVERY_INPUT_INVALID");
    if(session!=session_||goal!=goal_) {
      if(status()==BT::NodeStatus::RUNNING)return fail("POLICY_RECOVERY_EXECUTION_CHANGED");
      session_=session;goal_=goal;started_.reset();
    }
    setStatus(BT::NodeStatus::RUNNING);
    executor_.spin_all(std::chrono::milliseconds(1));
    if(stage_!=Stage::FOLLOW&&started_&&Clock::now()-*started_>=std::chrono::seconds(120))return fail("POLICY_RECOVERY_TIME_BUDGET");
    if(stage_==Stage::FOLLOW) {
      try {return children_nodes_[0]->executeTick();}
      catch(const astribot_s1_path_tracking::ObstructionDeadline &error) {
        if(!getInput("path",reference_)||reference_.poses.empty())return fail("POLICY_RECOVERY_REFERENCE_MISSING");
        if(!slam_||slam_->header.frame_id!=goal_.header.frame_id||
            !std::isfinite(slam_->pose.pose.position.x)||!std::isfinite(slam_->pose.pose.position.y))
          return fail("POLICY_RECOVERY_SLAM_POSE_INVALID");
        const auto &position=slam_->pose.pose.position;
        bool found=false;double heading=0.,nearest=std::numeric_limits<double>::infinity();
        for(size_t i=1;i<reference_.poses.size();++i) {
          const auto &a=reference_.poses[i-1].pose.position,&b=reference_.poses[i].pose.position;
          const double dx=b.x-a.x,dy=b.y-a.y,length_squared=dx*dx+dy*dy;
          if(length_squared<=1e-12)continue;
          const double fraction=std::clamp(((position.x-a.x)*dx+(position.y-a.y)*dy)/length_squared,0.,1.);
          const double distance=std::hypot(position.x-a.x-fraction*dx,position.y-a.y-fraction*dy);
          if(distance<nearest){nearest=distance;heading=std::atan2(dy,dx);found=true;}
        }
        if(!found)return fail("POLICY_RECOVERY_REFERENCE_HAS_NO_DIRECTION");
        setOutput("required_heading",heading);
        // Halt and cancel the normal FollowPath before changing planning mode.
        // The recovery child additionally waits for terminal status and SLAM stop.
        haltChild(0);if(!started_)started_=Clock::now();
        begin(Stage::COMMIT);
        RCLCPP_WARN(node_->get_logger(),"POLICY_OBSTRUCTION stage=COMMIT session=%s reason=%s",session_.c_str(),error.what());
      }
    }
    if(stage_==Stage::COMMIT) {
      const int result=poll(Service::Request::RECOVER,Service::Response::RECOVERY_COMMITTED);
      if(result<0)return fail(service_error_);
      if(result==0)return BT::NodeStatus::RUNNING;
      committed_=true;stage_=Stage::RECOVER;
    }
    if(stage_==Stage::RECOVER) {
      if(poll(Service::Request::RECOVER,Service::Response::RECOVERY_COMMITTED)<0)return fail(service_error_);
      const auto result=children_nodes_[1]->executeTick();
      if(result==BT::NodeStatus::RUNNING)return result;
      // ControlNode::haltChild only invokes halt() for RUNNING children.
      // This completed recovery needs its per-activation state reset as well;
      // ComputePathWithRecovery::halt deliberately retains execution budgets.
      children_nodes_[1]->halt();haltChild(1);
      if(result!=BT::NodeStatus::SUCCESS)return fail("POLICY_RECOVERY_DEPARTURE_OR_REPLAN_FAILED");
      begin(Stage::RESUME);
    }
    if(stage_==Stage::RESUME) {
      const int result=poll(Service::Request::RESUME,Service::Response::RESUMED);
      if(result<0)return fail(service_error_);
      if(result==0)return BT::NodeStatus::RUNNING;
      committed_=false;stage_=Stage::FOLLOW;
      RCLCPP_INFO(node_->get_logger(),"POLICY_OBSTRUCTION stage=RESUMED session=%s",session_.c_str());
    }
    return BT::NodeStatus::RUNNING;
  }
private:
  void clearRequest() {
    if(pending_){client_->remove_pending_request(pending_->request_id);pending_.reset();}
  }
  void begin(Stage stage) {
    clearRequest();stage_=stage;next_request_=Clock::time_point();last_confirmation_=Clock::now();
  }
  int poll(uint8_t mode,uint8_t expected) {
    if(pending_&&pending_->future.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      const auto response=pending_->future.get();pending_.reset();
      if(response->disposition==Service::Response::BLOCKED) {service_error_=response->reason;return -1;}
      if(response->disposition==expected) {
        last_confirmation_=Clock::now();next_request_=Clock::now()+std::chrono::milliseconds(100);return 1;
      }
      // The route service mailbox returns KEEP while its worker evaluates.
      // Only an explicit mode confirmation authorizes the handover.
      next_request_=Clock::now()+std::chrono::milliseconds(10);
    }
    if(Clock::now()-last_confirmation_>std::chrono::seconds(2)) {service_error_="POLICY_RECOVERY_SERVICE_UNAVAILABLE";return -1;}
    if(!pending_&&Clock::now()>=next_request_&&client_->service_is_ready()) {
      auto request=std::make_shared<Service::Request>();request->mode=mode;request->session_id=session_;
      request->goal=goal_;request->reference_path=reference_;request->allow_detour=false;
      pending_.emplace(client_->async_send_request(request));
    }
    return 0;
  }
  void finishMode() {
    const bool required=committed_||stage_==Stage::COMMIT||stage_==Stage::RESUME;
    clearRequest();if(!required)return;
    if(!client_->service_is_ready()) {
      RCLCPP_ERROR(node_->get_logger(),"POLICY_RECOVERY_FINISH_UNAVAILABLE session=%s",session_.c_str());return;
    }
    auto request=std::make_shared<Service::Request>();request->mode=Service::Request::FINISH;
    request->session_id=session_;request->goal=goal_;request->reference_path=reference_;
    bool finished=false;const auto deadline=Clock::now()+std::chrono::seconds(2);
    while(Clock::now()<deadline) {
      auto pending=client_->async_send_request(request);
      const auto result=executor_.spin_until_future_complete(pending.future,deadline-Clock::now());
      client_->remove_pending_request(pending.request_id);
      if(result!=rclcpp::FutureReturnCode::SUCCESS)break;
      const auto response=pending.future.get();
      if(response->disposition==Service::Response::FINISHED){finished=true;break;}
      if(response->disposition==Service::Response::BLOCKED) {
        RCLCPP_ERROR(node_->get_logger(),"POLICY_RECOVERY_FINISH_REJECTED session=%s reason=%s",session_.c_str(),response->reason.c_str());break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if(!finished)RCLCPP_ERROR(node_->get_logger(),"POLICY_RECOVERY_FINISH_UNCONFIRMED session=%s",session_.c_str());
    committed_=false;
  }
  BT::NodeStatus fail(const std::string &reason) {
    haltChildren();finishMode();stage_=Stage::FAILED;
    RCLCPP_ERROR(node_->get_logger(),"POLICY_RECOVERY_FAILED session=%s reason=%s",session_.c_str(),reason.c_str());
    return BT::NodeStatus::FAILURE;
  }
  Stage stage_=Stage::FOLLOW;bool committed_=false;
  std::string session_,service_error_;geometry_msgs::msg::PoseStamped goal_;nav_msgs::msg::Path reference_;
  std::optional<Clock::time_point> started_;Clock::time_point next_request_{},last_confirmation_{};
  rclcpp::Node::SharedPtr node_;rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<Service>::SharedPtr client_;
  Slam::ConstSharedPtr slam_;rclcpp::Subscription<Slam>::SharedPtr slam_sub_;
  std::optional<rclcpp::Client<Service>::FutureAndRequestId> pending_;
};
void registerRecoverPolicyObstruction(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<RecoverPolicyObstruction>("RecoverPolicyObstruction");
}
} // namespace astribot_s1_navigation_recovery
