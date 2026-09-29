#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <astribot_transport_msgs/msg/execution_guard_status.hpp>
#include <astribot_transport_msgs/srv/set_execution_guard.hpp>
#include "astribot_s1_transport_mtc/execution_guard.hpp"

namespace {
using Status=astribot_transport_msgs::msg::ExecutionGuardStatus;
using Service=astribot_transport_msgs::srv::SetExecutionGuard;
using State=control_msgs::msg::JointTrajectoryControllerState;
class Guard : public rclcpp::Node {
  rclcpp::Subscription<State>::SharedPtr subscription_;
  rclcpp::Publisher<Status>::SharedPtr publisher_;
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  State::ConstSharedPtr state_;
  Status status_;
  std::vector<std::string> joints_;
  std::string fault_;
  double joint_limit_;
  std::chrono::steady_clock::time_point armed_wall_;
  bool observed_{false};
  void publish() {status_.stamp=now();publisher_->publish(status_);}
  void set(const std::shared_ptr<Service::Request> req,std::shared_ptr<Service::Response> response) {
    if(req->context_id.empty() || (status_.active && req->context_id!=status_.context_id)) {
      response->reason="EXECUTION_GUARD_CONTEXT_CONFLICT";return;
    }
    if(!req->enable) {
      if(req->context_id!=status_.context_id) {response->reason="EXECUTION_GUARD_CONTEXT_MISMATCH";return;}
      status_.active=false;status_.healthy=false;response->accepted=true;response->reason=fault_;publish();return;
    }
    if(status_.active) {response->accepted=fault_.empty();response->reason=fault_;return;}
    if(req->joint_names.empty() ||
       std::set<std::string>(req->joint_names.begin(),req->joint_names.end()).size()!=req->joint_names.size()) {
      response->reason="EXECUTION_GUARD_INVALID_REFERENCE";return;
    }
    joints_=req->joint_names;fault_.clear();observed_=false;
    armed_wall_=std::chrono::steady_clock::now();
    status_=Status();status_.context_id=req->context_id;status_.active=true;
    status_.reason="WAITING_FOR_EXECUTION_EVIDENCE";response->accepted=true;publish();
  }
  void tick() {
    if(!status_.active){publish();return;}
    const auto wall=std::chrono::steady_clock::now();
    std::string problem;
    if(fault_.empty()) {
      if(!state_)problem="MANIPULATION_TRACKING_STATE_UNAVAILABLE";
      else if(state_->header.stamp.sec<0||state_->header.stamp.nanosec>=1000000000u||(state_->header.stamp.sec==0&&state_->header.stamp.nanosec==0))problem="MANIPULATION_TRACKING_STATE_INVALID";
      else {
        status_.joint_stamp=state_->header.stamp;
        problem=astribot_s1_transport_mtc::trackingFault(joints_,state_->joint_names,
          state_->desired.positions,state_->actual.positions,joint_limit_,status_.maximum_joint_error_rad);
      }
      const bool waiting=problem=="MANIPULATION_TRACKING_STATE_UNAVAILABLE";
      if(!problem.empty() && (!waiting || observed_ || std::chrono::duration<double>(wall-armed_wall_).count()>.3))fault_=problem;
      if(problem.empty())observed_=true;
    }
    status_.healthy=fault_.empty() && problem.empty() && observed_;
    status_.reason=!fault_.empty()?fault_:(status_.healthy?"EXECUTION_WITHIN_BOUNDS":"WAITING_FOR_EXECUTION_EVIDENCE");
    publish();
  }
public:
  Guard():Node("manipulation_execution_guard") {
    joint_limit_=declare_parameter("maximum_joint_error_rad",.05);
    if(!std::isfinite(joint_limit_) || joint_limit_<=0. || joint_limit_>.05)throw std::runtime_error("Invalid joint tracking safety bound");
    publisher_=create_publisher<Status>("/transport/execution_guard/status",10);
    subscription_=create_subscription<State>("/arm_left_controller/state",10,[this](State::ConstSharedPtr s){
      if(state_&&s->header.frame_id==state_->header.frame_id&&s->header.stamp.sec>=0&&s->header.stamp.nanosec<1000000000u&&
         (s->header.stamp.sec>0||s->header.stamp.nanosec>0)&&
         int64_t(s->header.stamp.sec)*1000000000+s->header.stamp.nanosec<=int64_t(state_->header.stamp.sec)*1000000000+state_->header.stamp.nanosec)return;
      state_=s;
    });
    service_=create_service<Service>("/transport/execution_guard/set",std::bind(&Guard::set,this,std::placeholders::_1,std::placeholders::_2));
    timer_=create_wall_timer(std::chrono::milliseconds(20),std::bind(&Guard::tick,this));
  }
};
}
int main(int argc,char**argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Guard>());rclcpp::shutdown();}
