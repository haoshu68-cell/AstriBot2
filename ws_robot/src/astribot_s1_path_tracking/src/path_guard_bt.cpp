#include <chrono>
#include <atomic>
#include "behaviortree_cpp_v3/decorator_node.h"
#include "astribot_s1_path_tracking/route_commit.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "astribot_s1_path_tracking/path_quality.hpp"
#include "astribot_s1_path_tracking/corridor_route.hpp"
#include "astribot_s1_path_tracking/envelope_evidence.hpp"

namespace astribot_s1_path_tracking {
void registerWorkstationApproach(BT::BehaviorTreeFactory & factory);
class RequireNavigationEnvelope : public BT::DecoratorNode {
  using Evidence=EnvelopeEvidence;
  using Message=Evidence::Message;
public:
  RequireNavigationEnvelope(const std::string & name,const BT::NodeConfiguration & config)
  :BT::DecoratorNode(name,config) {
    node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    if(!node_->has_parameter("navigation_geometry_mode"))node_->declare_parameter("navigation_geometry_mode","legacy");
    const auto mode=node_->get_parameter("navigation_geometry_mode").as_string();
    if(mode!="legacy" && mode!="fixed_v2")throw std::invalid_argument("invalid navigation_geometry_mode");
    enabled_=mode=="fixed_v2";
    if(!enabled_)return;
    if(!node_->has_parameter("robot_base_frame"))node_->declare_parameter("robot_base_frame","astribot_torso_base");
    base_=node_->get_parameter("robot_base_frame").as_string();
    group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    executor_.add_callback_group(group_,node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options;options.callback_group=group_;
    subscription_=node_->create_subscription<Message>("/navigation/envelope_v2",10,
      [this](Message::ConstSharedPtr message) {
        const auto now=node_->now().nanoseconds();const auto wall=Evidence::Wall::now();
        // accept() clears a conflicting hash at the same epoch. Retain the
        // identity diagnosis before that sample disappears.
        if(bound_ && failure_.empty() && message->coordinator_session_id==bound_->coordinator_session_id &&
          message->epoch==bound_->epoch && message->installed_geometry_hash!=bound_->installed_geometry_hash)
          failure_="NAVIGATION_ENVELOPE_CHANGED";
        evidence_.accept(message,now,wall);
        // Drain queued heartbeats before judging freshness. An expired older
        // positive sample must not hide the current one. Authority changes and
        // explicit revocations still latch immediately within the same batch.
        if(bound_ && failure_.empty()) {
          const auto sample=evidence_.sample(now);
          if(sample.message && !Evidence::sameExecution(*bound_,*sample.message))
            failure_="NAVIGATION_ENVELOPE_CHANGED";
          else if(sample.message &&
            (!sample.message->navigation_allowed || !sample.message->limits.transport_ready))
            failure_="ENVELOPE_REVOKED: "+sample.message->reason;
          else {
            const auto reason=Evidence::navigationReason(sample,base_,now,wall);
            if(reason!="ENVELOPE_ROS_STALE" && reason!="ENVELOPE_WALL_STALE" &&
              reason!="ENVELOPE_EXPIRED")failure_=reason;
          }
        }
      },options);
  }
  static BT::PortsList providedPorts() {return {BT::OutputPort<std::string>("reason")};}
  void halt() override {
    bound_.reset();failure_.clear();
    // Keep the published failure reason available after Nav2 halts the tree.
    BT::DecoratorNode::halt();
  }
  BT::NodeStatus tick() override {
    const bool report_failure=status()!=BT::NodeStatus::FAILURE;
    setStatus(BT::NodeStatus::RUNNING);
    if(enabled_) {
      // spin_some takes at most one message from this subscription per tick.
      // The bounded exhaustive pass consumes the idle-time depth-10 backlog.
      executor_.spin_all(std::chrono::milliseconds(1));
      const auto now=node_->now().nanoseconds();const auto wall=Evidence::Wall::now();
      const auto sample=evidence_.sample(now);check(sample,now,wall);
      if(!failure_.empty()) {
        setOutput("reason",failure_);
        if(report_failure)RCLCPP_ERROR(node_->get_logger(),"NAVIGATION_ENVELOPE_FAILURE: %s",failure_.c_str());
        resetChild();return BT::NodeStatus::FAILURE;
      }
      if(!bound_) {bound_=sample.message;setOutput("reason",std::string{});}
    }
    const auto result=child_node_->executeTick();
    if(result!=BT::NodeStatus::RUNNING) {bound_.reset();resetChild();}
    return result;
  }
private:
  void check(const Evidence::Sample & sample,int64_t now,Evidence::Wall::time_point wall) {
    if(!failure_.empty())return;
    if(bound_ && sample.message && !Evidence::sameExecution(*bound_,*sample.message))
      failure_="NAVIGATION_ENVELOPE_CHANGED";
    else failure_=Evidence::navigationReason(sample,base_,now,wall);
  }
  bool enabled_{false};std::string base_,failure_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<Message>::SharedPtr subscription_;
  Evidence evidence_;Message::ConstSharedPtr bound_;
};
class RequireCorridorRoute : public BT::ConditionNode {
public:
  RequireCorridorRoute(const std::string & name,const BT::NodeConfiguration & config):BT::ConditionNode(name,config) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("path"),BT::InputPort<std::string>("session"),
      BT::InputPort<std::string>("frame"),BT::InputPort<double>("entry_x"),BT::InputPort<double>("entry_y"),
      BT::InputPort<double>("exit_x"),BT::InputPort<double>("exit_y"),BT::InputPort<double>("width")};
  }
  BT::NodeStatus tick() override {
    nav_msgs::msg::Path path;std::string session,frame;RoutePoint entry{},exit{};double width=0.;
    if(!getInput("path",path) || !getInput("session",session) || session.empty() ||
      !getInput("frame",frame) || frame.empty() || !getInput("entry_x",entry.x) ||
      !getInput("entry_y",entry.y) || !getInput("exit_x",exit.x) ||
      !getInput("exit_y",exit.y) || !getInput("width",width)) {
      throw BT::RuntimeError("CORRIDOR_ROUTE_INVALID_INTENT");
    }
    if(path.header.frame_id!=frame)throw BT::RuntimeError("CORRIDOR_ROUTE_FRAME_MISMATCH");
    std::vector<RoutePoint> points;
    for(const auto & p:path.poses) {
      if(p.header.frame_id!=frame)throw BT::RuntimeError("CORRIDOR_ROUTE_FRAME_MISMATCH");
      points.push_back({p.pose.position.x,p.pose.position.y});
    }
    if(!pathTraversesCorridor(points,entry,exit,width,session!=session_)) {
      throw BT::RuntimeError("PLANNED_ROUTE_BYPASSES_CORRIDOR");
    }
    session_=session;return BT::NodeStatus::SUCCESS;
  }
private:
  std::string session_;
};
class PolicyExecution : public BT::DecoratorNode {
public:
  PolicyExecution(const std::string & name,const BT::NodeConfiguration & config):BT::DecoratorNode(name,config) {}
  static BT::PortsList providedPorts() {return {BT::OutputPort<std::string>("session"),BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals")};}
  void halt() override {
    // Nav2's haltAllActions does not reset the root decorator's status.
    // Execution identity must therefore follow explicit lifecycle events.
    new_execution_=true;
    BT::DecoratorNode::halt();
  }
  BT::NodeStatus tick() override {
    geometry_msgs::msg::PoseStamped goal;std::vector<geometry_msgs::msg::PoseStamped> goals;
    if(getInput("goal",goal)) {goals.push_back(goal);} else {getInput("goals",goals);}
    if(new_execution_ || goals!=goals_) {
      new_execution_=false;
      goals_=goals;
      static std::atomic<uint64_t> serial{0};
      setOutput("session",std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+":"+std::to_string(++serial));
    }
    setStatus(BT::NodeStatus::RUNNING);
    const auto result=child_node_->executeTick();
    if(result!=BT::NodeStatus::RUNNING) {new_execution_=true;resetChild();}
    return result;
  }
private:
  bool new_execution_{true};
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};
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
    if (!node_->has_parameter("navigation_policy_stage")) {node_->declare_parameter("navigation_policy_stage","off");}
    const auto stage=node_->get_parameter("navigation_policy_stage").as_string();
    if (!node_->has_parameter("social_navigation_enabled")) {node_->declare_parameter("social_navigation_enabled",false);}
    if(stage=="p3" || stage=="p4" || stage=="p5" || node_->get_parameter("social_navigation_enabled").as_bool()) {
      route_=std::make_unique<RouteCommit>(node_,group_);
    }
  }
  static BT::PortsList providedPorts() {
    return {BT::BidirectionalPort<nav_msgs::msg::Path>("path"),BT::InputPort<std::string>("session",""),
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
    std::string session;getInput("session",session);
    bool changed=goals!=goals_ || (route_ && session!=session_);
    session_=session;
    if(changed) {
      if(config().output_ports.count("remaining_goals")) {setOutput("remaining_goals",goals);}
      if(route_) {route_->reset();}
      goals_=goals;replans_=0; rejected_=nav_msgs::msg::Path(); awaiting_=false; known_=false; clearRequest();
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
    if(route_) {
      std::string reason;
      if(route_->update(session_,path,goals.back(),goals.size()==1,reason)) {
        setOutput("path",path);previous_=path;clearRequest();last_good_=now;next_check_=now;
        publishPathRisk(false);event(reason.c_str());return BT::NodeStatus::SUCCESS;
      }
    }
    if(pending_ && future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      auto response=future_.get();pending_=false;
      if(response->is_valid) {last_good_=now;rejected_=nav_msgs::msg::Path();publishPathRisk(false);}
      else if(!response->invalid_pose_indices.empty() && response->invalid_pose_indices.front()>=0) {
        publishPathRisk(true);
        if (policy_enabled_) {
          last_good_=now;next_check_=now+std::chrono::milliseconds(200);
          return BT::NodeStatus::SUCCESS;
        }
        if (sameGeometry(path,rejected_)) {
          throw BT::RuntimeError("PATH_GUARD: replanner returned the same blocked path");
        }
        if(++replans_>5) {throw BT::RuntimeError("PATH_GUARD: repeated collision replans exhausted");}
        rejected_=path;event("collision");awaiting_=true;return BT::NodeStatus::FAILURE;
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
  static bool sameGeometry(const nav_msgs::msg::Path & a,const nav_msgs::msg::Path & b) {
    if (a.header.frame_id!=b.header.frame_id || a.poses.size()!=b.poses.size()) {return false;}
    for (size_t i=0;i<a.poses.size();++i) {
      if (a.poses[i].pose!=b.poses[i].pose) {return false;}
    }
    return !a.poses.empty();
  }
  void publishPathRisk(bool blocked) {
    if (path_risk_publisher_) {std_msgs::msg::Bool msg;msg.data=blocked;path_risk_publisher_->publish(msg);}
  }
  std::unique_ptr<RouteCommit> route_;
  std::string session_;
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
  nav_msgs::msg::Path previous_,rejected_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};
}
BT_REGISTER_NODES(factory) {
  astribot_s1_path_tracking::registerWorkstationApproach(factory);
  factory.registerNodeType<astribot_s1_path_tracking::RequireNavigationEnvelope>("RequireNavigationEnvelope");
  factory.registerNodeType<astribot_s1_path_tracking::RequireCorridorRoute>("RequireCorridorRoute");
  factory.registerNodeType<astribot_s1_path_tracking::PolicyExecution>("PolicyExecution");
  factory.registerNodeType<astribot_s1_path_tracking::KeepSafePath>("KeepSafePath");
}
