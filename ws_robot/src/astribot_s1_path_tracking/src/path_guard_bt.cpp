#include <chrono>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "astribot_s1_path_tracking/path_quality.hpp"

namespace astribot_s1_path_tracking {
class KeepSafePath : public BT::ConditionNode {
  using Clock=std::chrono::steady_clock;
  using Service=nav2_msgs::srv::IsPathValid;
public:
  KeepSafePath(const std::string & name,const BT::NodeConfiguration & config):BT::ConditionNode(name,config) {
    node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    executor_.add_callback_group(group_,node_->get_node_base_interface());
    client_=node_->create_client<Service>("path_tracking/check_path",rmw_qos_profile_services_default,group_);
    publisher_=node_->create_publisher<std_msgs::msg::String>("path_tracking/replan_event",10);
    if (!node_->has_parameter("navigation_policy_enabled")) {
      node_->declare_parameter("navigation_policy_enabled",false);
    }
    if (node_->get_parameter("navigation_policy_enabled").as_bool()) {
      policy_enabled_=true;
      path_risk_publisher_=node_->create_publisher<std_msgs::msg::Bool>(
        "navigation_policy/path_blocked",rclcpp::QoS(1).transient_local());
    }
  }
  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("remaining_goals")};
  }
  BT::NodeStatus tick() override {
    nav_msgs::msg::Path path;getInput("path",path);
    std::vector<geometry_msgs::msg::PoseStamped> goals;
    geometry_msgs::msg::PoseStamped goal;
    if(getInput("goal",goal)) {goals.push_back(goal);} else {getInput("goals",goals);}
    if(goals.empty()) {throw BT::RuntimeError("PATH_GUARD: missing goal");}
    bool changed=goals!=goals_;
    if(changed) {
      if(config().output_ports.count("remaining_goals")) {setOutput("remaining_goals",goals);}
      goals_=goals;replans_=0; awaiting_=false; known_=false; clearRequest();
      event(path.poses.empty()?"initial_goal":"new_goal");awaiting_=true;previous_=path;
      return BT::NodeStatus::FAILURE;
    }
    if(path.poses.empty()) {return BT::NodeStatus::FAILURE;}
    if(awaiting_) {
      if(path==previous_) {return BT::NodeStatus::FAILURE;}
      awaiting_=false;known_=false;
    }
    auto now=Clock::now();
    if(!known_ || path!=previous_) {
      previous_=path;known_=true;clearRequest();last_good_=now;next_check_=now;

    }
    executor_.spin_some();
    if(pending_ && future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      auto response=future_.get();pending_=false;
      if(response->is_valid) {last_good_=now;publishPathRisk(false);}
      else if(!response->invalid_pose_indices.empty() && response->invalid_pose_indices.front()>=0) {
        publishPathRisk(true);
        if (policy_enabled_) {
          last_good_=now;next_check_=now+std::chrono::milliseconds(200);
          return BT::NodeStatus::SUCCESS;
        }
        if(++replans_>5) {throw BT::RuntimeError("PATH_GUARD: repeated collision replans exhausted");}
        event("collision");awaiting_=true;return BT::NodeStatus::FAILURE;
      }
      next_check_=now+std::chrono::milliseconds(200);
    }
    if(now-last_good_>std::chrono::seconds(2)) {
      clearRequest();throw BT::RuntimeError("PATH_GUARD: collision check unavailable for 2s");
    }
    if(!pending_ && now>=next_check_ && client_->service_is_ready()) {
      auto req=std::make_shared<Service::Request>();req->path=path;
      auto result=client_->async_send_request(req);future_=result.future.share();
      request_id_=result.request_id;pending_=true;
    }
    return BT::NodeStatus::SUCCESS;
  }
private:
  void publishPathRisk(bool blocked) {
    if (path_risk_publisher_) {std_msgs::msg::Bool msg;msg.data=blocked;path_risk_publisher_->publish(msg);}
  }
  bool policy_enabled_{false};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_risk_publisher_;
  void clearRequest() {if(pending_) {client_->remove_pending_request(request_id_);pending_=false;}}
  void event(const char * reason) {
    std_msgs::msg::String msg;msg.data=std::string("{\"reason\":\"")+reason+"\"}";
    publisher_->publish(msg);RCLCPP_INFO(node_->get_logger(),"REPLAN_EVENT %s",reason);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<Service>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::shared_future<Service::Response::SharedPtr> future_;
  int64_t request_id_{0}; bool pending_{false},awaiting_{false},known_{false};int replans_{0};
  Clock::time_point last_good_,next_check_;
  nav_msgs::msg::Path previous_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};
}
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<astribot_s1_path_tracking::KeepSafePath>("KeepSafePath");
}
