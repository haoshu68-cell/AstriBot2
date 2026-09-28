#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <astribot_transport_msgs/msg/execution_guard_status.hpp>
#include <astribot_transport_msgs/srv/set_execution_guard.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "astribot_s1_transport_mtc/execution_guard.hpp"
#include "astribot_s1_transport_mtc/base_motion_limits.hpp"

namespace {
using Status=astribot_transport_msgs::msg::ExecutionGuardStatus;
using Service=astribot_transport_msgs::srv::SetExecutionGuard;
using State=control_msgs::msg::JointTrajectoryControllerState;
class Guard : public rclcpp::Node {
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr slam_pose_;
  rclcpp::Subscription<State>::SharedPtr subscription_;
  rclcpp::Publisher<Status>::SharedPtr publisher_;
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  State::ConstSharedPtr state_;
  Status status_;
  std::vector<std::string> joints_;
  geometry_msgs::msg::PoseStamped reference_;
  std::string fault_;
  double joint_limit_;
  astribot_s1_transport_mtc::BaseMotionLimits base_limits_;
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
    const auto & p=req->base_reference.pose.position;const auto & q=req->base_reference.pose.orientation;
    if(req->base_reference.header.frame_id!="map" || req->joint_names.empty() ||
       std::set<std::string>(req->joint_names.begin(),req->joint_names.end()).size()!=req->joint_names.size() ||
       !astribot_s1_transport_mtc::executionPoseValid(p.x,p.y,p.z,q.x,q.y,q.z,q.w)) {
      response->reason="EXECUTION_GUARD_INVALID_REFERENCE";return;
    }
    joints_=req->joint_names;reference_=req->base_reference;fault_.clear();observed_=false;
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
        if(!slam_pose_) {if(problem.empty())problem="MANIPULATION_BASE_STATE_UNAVAILABLE";}
        else {
          status_.base_stamp=slam_pose_->header.stamp;
          const auto &p=slam_pose_->pose.pose.position;const auto &q=slam_pose_->pose.pose.orientation;
          if(slam_pose_->header.frame_id!="map"||slam_pose_->header.stamp.sec<0||slam_pose_->header.stamp.nanosec>=1000000000u||
             (slam_pose_->header.stamp.sec==0&&slam_pose_->header.stamp.nanosec==0)||
             !astribot_s1_transport_mtc::executionPoseValid(p.x,p.y,p.z,q.x,q.y,q.z,q.w)) {
            problem="MANIPULATION_BASE_STATE_INVALID";
          } else {
            tf2::Transform current,reference;tf2::fromMsg(slam_pose_->pose.pose,current);tf2::fromMsg(reference_.pose,reference);
            const auto delta=reference.inverse()*current;
            status_.base_translation_m=delta.getOrigin().length();status_.base_rotation_rad=delta.getRotation().getAngleShortestPath();
            if(status_.base_translation_m>base_limits_.translation_m || status_.base_rotation_rad>base_limits_.rotation_rad)problem="MTC_BASE_MOVED_DURING_EXECUTION";
          }
        }
      }
      const bool waiting=problem=="MANIPULATION_TRACKING_STATE_UNAVAILABLE" || problem=="MANIPULATION_BASE_STATE_UNAVAILABLE";
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
    rcl_interfaces::msg::ParameterDescriptor simulation_descriptor;
    simulation_descriptor.read_only=true;
    const bool relaxed=declare_parameter<bool>("simulation_relaxed_base_motion",false,simulation_descriptor);
    base_limits_=astribot_s1_transport_mtc::baseMotionLimits(relaxed,get_parameter("use_sim_time").as_bool());
    publisher_=create_publisher<Status>("/transport/execution_guard/status",10);
    subscription_=create_subscription<State>("/arm_left_controller/state",10,[this](State::ConstSharedPtr s){
      if(state_&&s->header.frame_id==state_->header.frame_id&&s->header.stamp.sec>=0&&s->header.stamp.nanosec<1000000000u&&
         (s->header.stamp.sec>0||s->header.stamp.nanosec>0)&&
         int64_t(s->header.stamp.sec)*1000000000+s->header.stamp.nanosec<=int64_t(state_->header.stamp.sec)*1000000000+state_->header.stamp.nanosec)return;
      state_=s;
    });
    pose_subscription_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose",rclcpp::SensorDataQoS(),[this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr s){
      if(slam_pose_&&int64_t(s->header.stamp.sec)*1000000000+s->header.stamp.nanosec<=int64_t(slam_pose_->header.stamp.sec)*1000000000+slam_pose_->header.stamp.nanosec)return;
      slam_pose_=s;
    });
    service_=create_service<Service>("/transport/execution_guard/set",std::bind(&Guard::set,this,std::placeholders::_1,std::placeholders::_2));
    timer_=create_wall_timer(std::chrono::milliseconds(20),std::bind(&Guard::tick,this));
  }
};
}
int main(int argc,char**argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Guard>());rclcpp::shutdown();}
