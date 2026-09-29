#include "astribot_navigation_zones/client.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "astribot_navigation_msgs/msg/navigation_execution_status.hpp"
#include "astribot_s1_path_tracking/envelope_evidence.hpp"
#include "astribot_s1_task_arbiter_native/ownership.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace astribot_s1_task_arbiter_native {
using Steady = std::chrono::steady_clock;
using Status = action_msgs::msg::GoalStatus;
using ExecutionStatus = astribot_navigation_msgs::msg::NavigationExecutionStatus;
using EnvelopeEvidence = astribot_s1_path_tracking::EnvelopeEvidence;

std::string key(const rclcpp_action::GoalUUID &uuid) {
  constexpr char hex[] = "0123456789abcdef";
  std::string result;
  result.reserve(32);
  for (const auto byte : uuid) { result += hex[byte >> 4]; result += hex[byte & 15]; }
  return result;
}

class TaskArbiter : public rclcpp::Node {
  struct TaskBase {
    std::string id, source, envelope_failure;
    bool preempted{false}, cancel_sent{false}, terminal{false};std::string zone_token;
    virtual ~TaskBase() = default;
    virtual void poll(Steady::time_point now) = 0;
    virtual void request_cancel() = 0;
  };
  template<class Action>
  struct Task : TaskBase, std::enable_shared_from_this<Task<Action>> {
    using Front = rclcpp_action::ServerGoalHandle<Action>;
    using Client = rclcpp_action::Client<Action>;
    using Back = rclcpp_action::ClientGoalHandle<Action>;
    enum class Phase { WaitingOwner, WaitingServer, Sending, Executing };
    TaskArbiter &node;
    std::shared_ptr<Front> front;
    typename Client::SharedPtr client;
    typename Back::SharedPtr backend;
    EnvelopeEvidence::Message::ConstSharedPtr admitted_envelope;
    Phase phase{Phase::WaitingOwner};
    Steady::time_point deadline;
    Task(TaskArbiter &n, std::shared_ptr<Front> f, typename Client::SharedPtr c,
         std::string source) : node(n), front(std::move(f)), client(std::move(c)),
         deadline(Steady::now() + n.timeout_) {
      this->id = key(front->get_goal_id());
      this->source = std::move(source);
      admitted_envelope=node.reserved_envelope_;
      if(auto s=node.zones_.constraints.get())this->zone_token=s->token;
    }
    void request_cancel() override {
      if (!backend || this->cancel_sent || this->terminal) return;
      this->cancel_sent = true;
      // Deliberately retain ownership on ACK, rejection, or missing response.
      // Python also sends exactly once and consumes only the terminal result.
      client->async_cancel_goal(backend);
      node.emit(*this, "CANCELING", !this->envelope_failure.empty() ? this->envelope_failure :
                (this->preempted ? "PREEMPTED" : "USER_CANCEL"));
    }
    void check_envelope() {
      if(!node.require_envelope_ || !this->envelope_failure.empty())return;
      const auto at=node.now().nanoseconds();
      const auto sample=node.envelope_evidence_.sample(at);
      this->envelope_failure=EnvelopeEvidence::navigationReason(sample,node.base_frame_,at,Steady::now());
      if(this->envelope_failure.empty() && !EnvelopeEvidence::sameExecution(*admitted_envelope,*sample.message))
        this->envelope_failure="NAVIGATION_ENVELOPE_CHANGED";
    }
    void finish(std::string state, const std::string &reason,
                std::shared_ptr<typename Action::Result> result = nullptr, uint8_t status = 0) {
      if (this->terminal) return;
      this->terminal = true;
      if (!result) result = std::make_shared<typename Action::Result>();
      if (front->is_canceling()) { front->canceled(result); state = "CANCELED"; }
      else if (state == "SUCCEEDED") front->succeed(result);
      else front->abort(result);
      node.emit(*this, state, reason, status);
      node.ownership_.release(this->id);
      node.tasks_.erase(this->id);
    }
    void poll(Steady::time_point now) override {
      if (this->terminal) return;
      check_envelope();
      if(!this->envelope_failure.empty()) {
        if(phase==Phase::WaitingOwner || phase==Phase::WaitingServer)
          finish("FAILED",this->envelope_failure);
        else request_cancel();
        return;
      }
      if(node.require_zones_){auto s=node.zones_.constraints.get();if(!node.zones_.ready()||!s||s->token!=this->zone_token){
        this->preempted=true;
        if(phase==Phase::WaitingOwner||phase==Phase::WaitingServer){finish("FAILED","ZONES_CHANGED_OR_UNAVAILABLE");return;}
        request_cancel();return;
      }}
      if (phase == Phase::WaitingOwner) {
        if (front->is_canceling()) { finish("CANCELED", "USER_CANCEL"); return; }
        if (node.ownership_.acquire(this->id)) {
          phase = Phase::WaitingServer;
          deadline = now + node.timeout_;
        } else {
          if (now >= deadline) finish("FAILED", "PREVIOUS_TASK_NOT_TERMINAL");
          return;
        }
      }
      if (phase != Phase::WaitingServer) return;
      if (!client->action_server_is_ready()) {
        if (now >= deadline) finish("FAILED", "EXECUTOR_UNAVAILABLE");
        return;
      }
      if (this->preempted || front->is_canceling()) {
        finish(this->preempted ? "PREEMPTED" : "CANCELED", "CANCELED_BEFORE_DISPATCH");
        return;
      }
      phase = Phase::Sending;
      std::weak_ptr<Task> weak = this->shared_from_this();
      typename Client::SendGoalOptions options;
      options.feedback_callback = [weak](typename Back::SharedPtr,
          const std::shared_ptr<const typename Action::Feedback> feedback) {
        if (auto task = weak.lock(); task && !task->terminal && task->front->is_active())
          task->front->publish_feedback(std::make_shared<typename Action::Feedback>(*feedback));
      };
      options.goal_response_callback = [weak](typename Back::SharedPtr backend) {
        if (auto task = weak.lock(); task && !task->terminal) {
          task->backend = backend;
          task->check_envelope();
          if (!backend) {
            task->finish("FAILED",task->envelope_failure.empty()?"EXECUTOR_REJECTED":task->envelope_failure);
            return;
          }
          task->phase = Phase::Executing;
          if (!task->envelope_failure.empty() || task->preempted || task->front->is_canceling()) task->request_cancel();
          if(task->envelope_failure.empty())task->node.emit(*task, "EXECUTING");
          task->client->async_get_result(backend, [weak](const typename Back::WrappedResult &result) {
            if (auto current = weak.lock(); current && !current->terminal) {
              const auto status = static_cast<uint8_t>(result.code);
              current->check_envelope();
              if(!current->envelope_failure.empty())
                current->finish("FAILED",current->envelope_failure,result.result,status);
              else if (current->preempted)
                current->finish("PREEMPTED", "HIGHER_OR_EQUAL_PRIORITY_TASK", result.result, status);
              else current->finish(status == Status::STATUS_SUCCEEDED ? "SUCCEEDED" : "FAILED",
                                   "EXECUTOR_RESULT", result.result, status);
            }
          });
        }
      };
      client->async_send_goal(*front->get_goal(), options);
    }
  };
 public:
  TaskArbiter() : Node("navigation_task_arbiter") {
    require_zones_=declare_parameter("require_navigation_zones",false);if(require_zones_)zones_.init(*this);
    rcl_interfaces::msg::ParameterDescriptor immutable;immutable.read_only=true;
    const auto mode=declare_parameter("navigation_geometry_mode",std::string("legacy"),immutable);
    if(mode!="legacy" && mode!="fixed_v2")throw std::invalid_argument("invalid navigation_geometry_mode");
    require_envelope_=mode=="fixed_v2";
    base_frame_=declare_parameter("robot_base_frame",std::string("astribot_torso_base"),immutable);
    if(require_envelope_) {
      envelope_sub_=create_subscription<EnvelopeEvidence::Message>("/navigation/envelope_v2",10,
        [this](EnvelopeEvidence::Message::ConstSharedPtr message) {
          envelope_evidence_.accept(message,now().nanoseconds(),Steady::now());
          // Observe revocation before a later heartbeat can overwrite it.
          poll_tasks();
        });
    }
    const double timeout = declare_parameter("handover_timeout_s", 10.0);
    if (!std::isfinite(timeout) || timeout <= 0 || timeout > 60)
      throw std::invalid_argument("invalid handover_timeout_s");
    // Snapshot at construction, exactly like Python's self.timeout. The ROS
    // parameter store remains writable; updates do not affect this snapshot.
    timeout_ = std::chrono::duration_cast<Steady::duration>(std::chrono::duration<double>(timeout));
    sequence_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        Steady::now().time_since_epoch()).count();
    status_ = create_publisher<ExecutionStatus>("/navigation/execution_status",
                                               rclcpp::QoS(10).transient_local());
    create_routes<nav2_msgs::action::NavigateToPose>("navigate_to_pose");
    create_routes<nav2_msgs::action::NavigateThroughPoses>("navigate_through_poses");
    // No waits/sleeps in callbacks. A missing terminal result keeps its owner,
    // while other requests, cancellation and pending deadlines stay responsive.
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] {poll_tasks();});
  }
 private:
  void poll_tasks() {
    std::vector<std::shared_ptr<TaskBase>> snapshot;
    for(const auto &[id,task]:tasks_)snapshot.push_back(task);
    for(const auto &task:snapshot)task->poll(Steady::now());
  }
  template<class Action> void create_routes(const std::string &name) {
    auto client = rclcpp_action::create_client<Action>(this, "/navigation_executor/" + name);
    clients_.push_back(client);
    struct Route { const char *source; int priority; const char *prefix; };
    for (const auto &route : {Route{"operator", 100, ""}, Route{"route", 50, "/route"},
                              Route{"exploration", 10, "/exploration"}}) {
      auto server = rclcpp_action::create_server<Action>(this, std::string(route.prefix) + "/" + name,
        [this, priority = route.priority, source=std::string(route.source)](const rclcpp_action::GoalUUID &uuid,
                                        std::shared_ptr<const typename Action::Goal>) {
          if(require_zones_&&!zones_.ready())return rclcpp_action::GoalResponse::REJECT;
          EnvelopeEvidence::Sample sample;
          if(require_envelope_) {
            const auto at=now().nanoseconds();sample=envelope_evidence_.sample(at);
            const auto reason=EnvelopeEvidence::navigationReason(sample,base_frame_,at,Steady::now());
            if(!reason.empty()) {
              emit(key(uuid),source,"REJECTED",reason);
              return rclcpp_action::GoalResponse::REJECT;
            }
          }
          if(!ownership_.reserve(priority))return rclcpp_action::GoalResponse::REJECT;
          reserved_envelope_=sample.message;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> front) {
          const auto it = tasks_.find(key(front->get_goal_id()));
          if (it == tasks_.end()) return rclcpp_action::CancelResponse::REJECT;
          it->second->request_cancel();
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this, client, route](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> front) {
          auto task = std::make_shared<Task<Action>>(*this, front, client, route.source);
          reserved_envelope_.reset();
          ownership_.accept(task->id, route.priority);
          tasks_.emplace(task->id, task);
          if (ownership_.active()) {
            auto old = tasks_.at(ownership_.active()->id);
            old->preempted = true;
            old->request_cancel();
          }
          emit(*task, "ACCEPTED");
          task->poll(Steady::now());
        });
      servers_.push_back(server);
    }
  }
  void emit(const TaskBase &task, const std::string &state, const std::string &reason = "",
            uint8_t status = 0) {
    emit(task.id,task.source,state,reason,status);
  }
  void emit(const std::string &id,const std::string &source,const std::string &state,
            const std::string &reason,uint8_t status=0) {
    ExecutionStatus message;
    message.stamp = now(); message.task_id = id; message.source = source;
    message.sequence = ++sequence_; message.state = state; message.reason = reason;
    message.action_status = status;
    status_->publish(message);
  }
  bool require_zones_{false};astribot_navigation_zones::Gate zones_;
  bool require_envelope_{false};std::string base_frame_;
  EnvelopeEvidence envelope_evidence_;
  EnvelopeEvidence::Message::ConstSharedPtr reserved_envelope_;
  rclcpp::Subscription<EnvelopeEvidence::Message>::SharedPtr envelope_sub_;
  Ownership ownership_;
  Steady::duration timeout_;
  uint64_t sequence_{0};
  std::map<std::string, std::shared_ptr<TaskBase>> tasks_;
  std::vector<std::shared_ptr<rclcpp_action::ServerBase>> servers_;
  std::vector<std::shared_ptr<rclcpp_action::ClientBase>> clients_;
  rclcpp::Publisher<ExecutionStatus>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace astribot_s1_task_arbiter_native

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    // Single executor and default mutually-exclusive callback group serialize
    // ownership, callbacks, and timer. No detached task or worker lifetime.
    rclcpp::spin(std::make_shared<astribot_s1_task_arbiter_native::TaskArbiter>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("navigation_task_arbiter"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
