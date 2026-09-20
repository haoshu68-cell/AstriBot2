#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <astribot_transport_msgs/msg/execution_guard_status.hpp>
#include <astribot_transport_msgs/srv/set_execution_guard.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "astribot_s1_transport_mtc/execution_guard.hpp"

namespace {
using Status=astribot_transport_msgs::msg::ExecutionGuardStatus;
using Service=astribot_transport_msgs::srv::SetExecutionGuard;
using State=control_msgs::msg::JointTrajectoryControllerState;
class Guard : public rclcpp::Node {
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<State>::SharedPtr subscription_;
  rclcpp::Publisher<Status>::SharedPtr publisher_;
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  State::ConstSharedPtr state_;
  Status status_;
  std::vector<std::string> joints_;
  geometry_msgs::msg::PoseStamped reference_;
  std::string base_frame_,fault_;
  double joint_limit_;
  int64_t armed_ns_{0},previous_ns_{0};
  std::chrono::steady_clock::time_point armed_wall_,received_wall_;
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
    const auto & p=req->base_reference.pose.position;const auto & q=req->base_reference.pose.orientation;
    if(req->base_reference.header.frame_id.empty() || req->joint_names.empty() ||
       std::set<std::string>(req->joint_names.begin(),req->joint_names.end()).size()!=req->joint_names.size() ||
       !astribot_s1_transport_mtc::executionPoseValid(p.x,p.y,p.z,q.x,q.y,q.z,q.w)) {
      response->reason="EXECUTION_GUARD_INVALID_REFERENCE";return;
    }
    joints_=req->joint_names;reference_=req->base_reference;fault_.clear();observed_=false;
    armed_ns_=previous_ns_=now().nanoseconds();armed_wall_=std::chrono::steady_clock::now();
    status_=Status();status_.context_id=req->context_id;status_.active=true;
    status_.reason="WAITING_FOR_EXECUTION_EVIDENCE";response->accepted=true;publish();
  }
  void tick() {
    if(!status_.active){publish();return;}
    const auto stamp=now().nanoseconds();const auto wall=std::chrono::steady_clock::now();
    if(stamp<previous_ns_)fault_="MANIPULATION_CLOCK_ROLLBACK";
    previous_ns_=stamp;
    std::string problem;
    if(fault_.empty()) {
      if(!state_ || !astribot_s1_transport_mtc::executionSourceFresh(stamp,rclcpp::Time(state_->header.stamp).nanoseconds(),armed_ns_) ||
          std::chrono::duration<double>(wall-received_wall_).count()>.3)problem="MANIPULATION_TRACKING_STATE_STALE";
      else {
        status_.joint_stamp=state_->header.stamp;
        problem=astribot_s1_transport_mtc::trackingFault(joints_,state_->joint_names,
          state_->desired.positions,state_->actual.positions,joint_limit_,status_.maximum_joint_error_rad);
        try {
          auto tf=buffer_.lookupTransform(reference_.header.frame_id,base_frame_,tf2::TimePointZero);
          status_.base_stamp=tf.header.stamp;
          const auto & p=tf.transform.translation;const auto & q=tf.transform.rotation;
          if(!astribot_s1_transport_mtc::executionPoseValid(p.x,p.y,p.z,q.x,q.y,q.z,q.w)) {
            problem="MANIPULATION_BASE_STATE_INVALID";
          } else if(!astribot_s1_transport_mtc::executionSourceFresh(stamp,rclcpp::Time(tf.header.stamp).nanoseconds(),armed_ns_)) {
            if(problem.empty())problem="MANIPULATION_BASE_STATE_STALE";
          } else {
            tf2::Transform current,reference;tf2::fromMsg(tf.transform,current);tf2::fromMsg(reference_.pose,reference);
            const auto delta=reference.inverse()*current;
            status_.base_translation_m=delta.getOrigin().length();status_.base_rotation_rad=delta.getRotation().getAngleShortestPath();
            if(status_.base_translation_m>.02 || status_.base_rotation_rad>.02)problem="MTC_BASE_MOVED_DURING_EXECUTION";
          }
        } catch(const tf2::TransformException&) {if(problem.empty())problem="MANIPULATION_BASE_STATE_STALE";}
      }
      const bool waiting=problem=="MANIPULATION_TRACKING_STATE_STALE" || problem=="MANIPULATION_BASE_STATE_STALE";
      if(!problem.empty() && (!waiting || observed_ || std::chrono::duration<double>(wall-armed_wall_).count()>.3))fault_=problem;
      if(problem.empty())observed_=true;
    }
    status_.healthy=fault_.empty() && problem.empty() && observed_;
    status_.reason=!fault_.empty()?fault_:(status_.healthy?"EXECUTION_WITHIN_BOUNDS":"WAITING_FOR_EXECUTION_EVIDENCE");
    publish();
  }
public:
  Guard():Node("manipulation_execution_guard"),buffer_(get_clock()),listener_(buffer_) {
    base_frame_=declare_parameter("base_frame",std::string("astribot_torso_base"));
    joint_limit_=declare_parameter("maximum_joint_error_rad",.05);
    if(!std::isfinite(joint_limit_) || joint_limit_<=0. || joint_limit_>.05)throw std::runtime_error("Invalid joint tracking safety bound");
    publisher_=create_publisher<Status>("/transport/execution_guard/status",10);
    subscription_=create_subscription<State>("/arm_left_controller/state",10,[this](State::ConstSharedPtr s){state_=s;received_wall_=std::chrono::steady_clock::now();});
    service_=create_service<Service>("/transport/execution_guard/set",std::bind(&Guard::set,this,std::placeholders::_1,std::placeholders::_2));
    timer_=create_wall_timer(std::chrono::milliseconds(20),std::bind(&Guard::tick,this));
  }
};
}
int main(int argc,char**argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Guard>());rclcpp::shutdown();}
