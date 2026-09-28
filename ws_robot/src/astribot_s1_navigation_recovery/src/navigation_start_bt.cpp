#include <chrono>
#include <cmath>
#include <vector>
#include "astribot_s1_path_tracking/envelope_evidence.hpp"
#include <memory>
#include <optional>
#include <string>

#include "astribot_navigation_msgs/srv/assess_navigation_start.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "rclcpp/rclcpp.hpp"

namespace astribot_s1_navigation_recovery {
class EnsureNavigationStart : public BT::DecoratorNode {
  using Service=astribot_navigation_msgs::srv::AssessNavigationStart;
  using Response=Service::Response;
  using Clock=std::chrono::steady_clock;
  using Diagnostic=diagnostic_msgs::msg::DiagnosticStatus;
  using Evidence=astribot_s1_path_tracking::EnvelopeEvidence;
  enum class Stage {CHECK,WAIT_INPUT,WAIT_PERMISSION,RECOVER,RECHECK,READY,ALARM};
public:
  EnsureNavigationStart(const std::string& name,const BT::NodeConfiguration& config)
  :BT::DecoratorNode(name,config) {
    node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    executor_.add_callback_group(group_,node_->get_node_base_interface());
    client_=node_->create_client<Service>("/navigation/assess_start",rmw_qos_profile_services_default,group_);
    if(!node_->has_parameter("navigation_geometry_mode"))node_->declare_parameter("navigation_geometry_mode","legacy");
    permission_required_=node_->get_parameter("navigation_geometry_mode").as_string()=="fixed_v2";
    if(permission_required_) {
      if(!node_->has_parameter("robot_base_frame"))node_->declare_parameter("robot_base_frame","astribot_torso_base");
      base_=node_->get_parameter("robot_base_frame").as_string();
      rclcpp::SubscriptionOptions options;options.callback_group=group_;
      envelope_=node_->create_subscription<Evidence::Message>("/navigation/envelope_v2",10,
        [this](Evidence::Message::ConstSharedPtr m){evidence_.accept(m,node_->now().nanoseconds(),Clock::now());},options);
    }
    alarm_=node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/navigation/start_alarm",10);
  }
  ~EnsureNavigationStart() override {clearRequest();}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("session"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")};
  }
  void halt() override {
    clearRequest();
    BT::DecoratorNode::halt();
    stage_=Stage::CHECK;execution_.clear();initial_reason_.clear();response_.reset();steps_=0;visited_.clear();movement_pending_=false;
  }
  BT::NodeStatus tick() override {
    if(stage_==Stage::ALARM)return BT::NodeStatus::FAILURE;
    setStatus(BT::NodeStatus::RUNNING);
    try {
      std::string execution;geometry_msgs::msg::PoseStamped goal;
      if(!getInput("session",execution)||execution.empty()||!getInput("goal",goal))
        return fail("NAVIGATION_START_INPUT_INVALID");
      // A completed parent Sequence may reset statuses without calling halt().
      // READY has no pending request or running recovery child. Active recovery
      // and ALARM must still go through the owning navigation cancellation.
      if(stage_==Stage::READY&&execution!=execution_) {
        stage_=Stage::CHECK;execution_.clear();initial_reason_.clear();response_.reset();steps_=0;visited_.clear();movement_pending_=false;
      }
      if(execution_.empty()) {
        execution_=execution;goal_=goal;query_started_=started_=Clock::now();next_query_=query_started_;
        RCLCPP_INFO(node_->get_logger(),"NAVIGATION_START stage=CHECK execution=%s",execution_.c_str());
      } else if(execution!=execution_||goal!=goal_) {
        return fail("NAVIGATION_START_EXECUTION_CHANGED");
      }
      if(stage_==Stage::READY)return BT::NodeStatus::SUCCESS;
      executor_.spin_all(std::chrono::milliseconds(1));
      if(Clock::now()-started_>=std::chrono::seconds(120))return fail("START_RECOVERY_TIME_BUDGET");
      if(stage_==Stage::WAIT_PERMISSION) {
        if(!permissionReady()) {
          if(Clock::now()-query_started_>=std::chrono::seconds(2))return fail("START_PERMISSION_TIMEOUT:"+permission_reason_);
          return BT::NodeStatus::RUNNING;
        }
        // Permission may have taken time. Re-evaluate the actual start, never
        // execute a candidate chosen from the previous observation.
        stage_=steps_?Stage::RECHECK:Stage::CHECK;response_.reset();query_started_=next_query_=Clock::now();
      }
      if(stage_==Stage::RECOVER) {
        const auto child=child_node_->executeTick();
        if(child==BT::NodeStatus::RUNNING)return child;
        if(child!=BT::NodeStatus::SUCCESS)return fail("DEPARTURE_CHILD_FAILURE");
        resetChild();++steps_;movement_pending_=true;stage_=Stage::RECHECK;response_.reset();query_started_=next_query_=Clock::now();
        RCLCPP_INFO(node_->get_logger(),"NAVIGATION_START stage=RECHECK execution=%s child=SUCCEEDED",execution_.c_str());
      }
      if(Clock::now()-query_started_>=std::chrono::seconds(2))
        return fail(stage_==Stage::WAIT_INPUT?"START_INPUT_TIMEOUT":"ASSESSMENT_TIMEOUT");
      if(!pending_) {
        if(Clock::now()<next_query_)return BT::NodeStatus::RUNNING;
        if(!client_->service_is_ready())return BT::NodeStatus::RUNNING;
        auto request=std::make_shared<Service::Request>();
        request->execution_id=execution_;request->goal=goal_;
        pending_.emplace(client_->async_send_request(request));
        return BT::NodeStatus::RUNNING;
      }
      if(pending_->future.wait_for(std::chrono::seconds(0))!=std::future_status::ready)
        return BT::NodeStatus::RUNNING;
      response_=pending_->future.get();pending_.reset();
      if(initial_reason_.empty())initial_reason_=response_->reason;
      if(response_->state==Response::UNAVAILABLE) {
        stage_=Stage::WAIT_INPUT;report(Diagnostic::WARN,response_->reason);
        next_query_=Clock::now()+std::chrono::milliseconds(100);
        return BT::NodeStatus::RUNNING;
      }
      if(response_->state>Response::UNAVAILABLE)return fail("ASSESSMENT_INVALID_STATE:"+response_->reason);
      if(response_->state==Response::BLOCKED)return fail("ASSESSMENT_REJECTED:"+response_->reason);
      const auto &position=response_->evaluated_start.pose.position;
      if(movement_pending_) {
        movement_pending_=false;
        if(response_->state==Response::RECOVERY_REQUIRED) {
          for(const auto &previous:visited_)
            if(std::hypot(position.x-previous.x,position.y-previous.y)<.03)
              return fail("START_RECOVERY_NO_PROGRESS");
        }
      }
      if(!permissionReady()) {
        stage_=Stage::WAIT_PERMISSION;query_started_=Clock::now();
        report(Diagnostic::WARN,permission_reason_);return BT::NodeStatus::RUNNING;
      }
      if(response_->state==Response::READY) {
        report(Diagnostic::OK,response_->reason);stage_=Stage::READY;
        RCLCPP_INFO(node_->get_logger(),"NAVIGATION_START stage=READY execution=%s steps=%u reason=%s",execution_.c_str(),steps_,response_->reason.c_str());
        return BT::NodeStatus::SUCCESS;
      }
      // Ten translations of at most 0.2 m bound the nominal recovery travel.
      if(steps_>=10)return fail("START_RECOVERY_STEP_BUDGET");
      visited_.push_back(position);stage_=Stage::RECOVER;
      report(Diagnostic::WARN,response_->reason);
      RCLCPP_WARN(node_->get_logger(),"NAVIGATION_START stage=RECOVER execution=%s step=%u reason=%s",execution_.c_str(),steps_+1,response_->reason.c_str());
      return BT::NodeStatus::RUNNING;
    } catch(const std::exception& error) {
      return fail(std::string("NAVIGATION_START_ERROR:")+error.what());
    }
  }
private:
  bool permissionReady() {
    if(!permission_required_)return true;
    const auto now=node_->now().nanoseconds();
    permission_reason_=Evidence::navigationReason(evidence_.sample(now),base_,now,Clock::now());
    return permission_reason_.empty();
  }
  const char* stageName() const {
    switch(stage_) {
      case Stage::CHECK:return "CHECK";
      case Stage::WAIT_INPUT:return "WAIT_INPUT";
      case Stage::WAIT_PERMISSION:return "WAIT_PERMISSION";
      case Stage::RECOVER:return "RECOVER";
      case Stage::RECHECK:return "RECHECK";
      case Stage::READY:return "READY";
      case Stage::ALARM:return "ALARM";
    }
    return "ALARM";
  }
  void clearRequest() {
    if(pending_) {client_->remove_pending_request(pending_->request_id);pending_.reset();}
  }
  void report(uint8_t level,const std::string& reason) {
    diagnostic_msgs::msg::DiagnosticArray array;array.header.stamp=node_->now();
    Diagnostic status;status.level=level;status.name="navigation_start";status.message=reason;
    auto add=[&](const std::string& key,const std::string& value) {
      diagnostic_msgs::msg::KeyValue entry;entry.key=key;entry.value=value;status.values.push_back(std::move(entry));
    };
    add("execution_id",execution_);add("stage",stageName());add("goal_frame",goal_.header.frame_id);
    add("initial_reason",initial_reason_);add("completed_steps",std::to_string(steps_));
    if(response_) {
      add("assessment_state",std::to_string(response_->state));add("assessment_reason",response_->reason);
      add("assessment_frame",response_->header.frame_id);
      add("assessment_stamp_ns",std::to_string(rclcpp::Time(response_->header.stamp).nanoseconds()));
      add("height_map_revision",response_->height_map_revision);add("costmap_revision",response_->costmap_revision);
      add("geometry_hash",response_->geometry_hash);add("envelope_epoch",std::to_string(response_->envelope_epoch));
    }
    if(reason=="DEPARTURE_CHILD_FAILURE")
      add("child_reason_source","Humble action result has no error detail; correlate planner/controller logs");
    array.status.push_back(std::move(status));alarm_->publish(array);
  }
  BT::NodeStatus fail(const std::string& reason) {
    clearRequest();resetChild();
    report(Diagnostic::ERROR,reason);
    RCLCPP_ERROR(node_->get_logger(),"NAVIGATION_START_ALARM stage=%s execution=%s reason=%s",stageName(),execution_.c_str(),reason.c_str());
    stage_=Stage::ALARM;return BT::NodeStatus::FAILURE;
  }
  Stage stage_=Stage::CHECK;
  std::string execution_,initial_reason_;
  geometry_msgs::msg::PoseStamped goal_;
  Clock::time_point query_started_,next_query_,started_;
  unsigned steps_=0;
  bool movement_pending_=false,permission_required_=false;
  std::vector<geometry_msgs::msg::Point> visited_;
  Evidence evidence_;
  std::string base_,permission_reason_;
  rclcpp::Subscription<Evidence::Message>::SharedPtr envelope_;
  Response::SharedPtr response_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<Service>::SharedPtr client_;
  std::optional<rclcpp::Client<Service>::FutureAndRequestId> pending_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr alarm_;
};

} // namespace astribot_s1_navigation_recovery

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<astribot_s1_navigation_recovery::EnsureNavigationStart>("EnsureNavigationStart");
}
