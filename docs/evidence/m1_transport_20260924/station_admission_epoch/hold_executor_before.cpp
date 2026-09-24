#include "astribot_s1_transport_native/arm_hold.hpp"
#include "astribot_s1_transport_native/resource_authority.hpp"
#include "astribot_s1_transport_native/resource_journal.hpp"
#include "astribot_s1_transport_native/child_actions.hpp"
#ifdef PLAN_TO_HOLD_EXECUTOR
#include "astribot_s1_transport_native/mtc_plan.hpp"
#include "astribot_s1_transport_native/scene_binding.hpp"
#include "astribot_s1_transport_native/payload_client.hpp"
#include "astribot_s1_transport_native/payload_frames.hpp"
#include "astribot_s1_transport_native/payload_scene.hpp"
#include "astribot_s1_transport_native/fixed_station_scene.hpp"
#include "astribot_s1_transport_native/fixed_station_navigation.hpp"
#include <astribot_s1_payload_state/consumer.hpp>
#include <astribot_transport_msgs/srv/revalidate_payload_transition.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <astribot_s1_transport_native/action/plan_to_hold.hpp>
#include <astribot_s1_transport_native/action/manipulation_to_hold.hpp>
#include <astribot_s1_transport_native/action/fixed_station_transfer.hpp>
#include <astribot_transport_msgs/action/plan_manipulation.hpp>
#include <astribot_transport_msgs/srv/set_execution_guard.hpp>
#include <astribot_transport_msgs/srv/revalidate_manipulation.hpp>
#include <astribot_transport_msgs/msg/execution_guard_status.hpp>
#include <astribot_navigation_msgs/msg/navigation_envelope_v2.hpp>
#include <astribot_navigation_msgs/srv/set_robot_envelope.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <nav_msgs/msg/odometry.hpp>
#endif
#include <astribot_s1_transport_native/action/hold_resources.hpp>
#include <astribot_s1_transport_native/srv/renew_hold.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <map>
#include <mutex>
#include <atomic>
namespace astribot::transport {
namespace {
constexpr int64_t MS=1000000;
int64_t wall(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int64_t ns(const builtin_interfaces::msg::Time &t){return int64_t(t.sec)*1000000000+t.nanosec;}
#ifdef PLAN_TO_HOLD_EXECUTOR
using Action=astribot_s1_transport_native::action::PlanToHold;
constexpr const char *NODE_NAME="task_trajectory_executor",*ACTION_NAME="/transport/plan_to_hold";
using FullAction=astribot_s1_transport_native::action::ManipulationToHold;
using FullParent=rclcpp_action::ServerGoalHandle<FullAction>;
using Transfer=astribot_s1_transport_native::action::FixedStationTransfer;
using TransferParent=rclcpp_action::ServerGoalHandle<Transfer>;
using Planner=astribot_transport_msgs::action::PlanManipulation;
using PlannerHandle=rclcpp_action::ClientGoalHandle<Planner>;
using SceneQuery=moveit_msgs::srv::GetPlanningScene;
using GuardService=astribot_transport_msgs::srv::SetExecutionGuard;
using GuardState=astribot_transport_msgs::msg::ExecutionGuardStatus;
using Envelope=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
using Revoke=astribot_navigation_msgs::srv::SetRobotEnvelope;
using Revalidate=astribot_transport_msgs::srv::RevalidateManipulation;
using PayloadRevalidate=astribot_transport_msgs::srv::RevalidatePayloadTransition;
using SceneApply=moveit_msgs::srv::ApplyPlanningScene;
using Observation=astribot::payload::Observation;
using PayloadState=astribot::payload::State;
#else
using Action=astribot_s1_transport_native::action::HoldResources;
constexpr const char *NODE_NAME="task_hold_executor",*ACTION_NAME="/transport/hold_resources";
#endif
using Parent=rclcpp_action::ServerGoalHandle<Action>;
using Trajectory=control_msgs::action::FollowJointTrajectory;
using Child=rclcpp_action::ClientGoalHandle<Trajectory>;
using Query=controller_manager_msgs::srv::ListControllers;
std::map<std::string,std::vector<std::string>> resources(){
 std::map<std::string,std::vector<std::string>> result;
 for(auto side:{"left","right"}) {
  for(int i=1;i<=7;++i)result[std::string("arm_")+side+"_controller"].push_back(std::string("astribot_arm_")+side+"_joint_"+std::to_string(i));
  result[std::string("gripper_")+side+"_controller"]={std::string("astribot_gripper_")+side+"_joint_L1"};
 }
 for(int i=1;i<=4;++i)result["torso_controller"].push_back("astribot_torso_joint_"+std::to_string(i));
 for(int i=1;i<=2;++i)result["head_controller"].push_back("astribot_head_joint_"+std::to_string(i));
 return result;
}
std::string boot_epoch(){std::ifstream input("/proc/sys/kernel/random/uuid");std::string s;input>>s;if(s.empty())throw std::runtime_error("EPOCH_UNAVAILABLE");return s;}
}
// This executor commissions simulation holds only. Real controllers require an
// execution-end epoch gate and a separately accepted hardware ownership bridge.
class HoldExecutor:public rclcpp::Node {
public:
 HoldExecutor():Node(NODE_NAME),groups_(resources()) {
  if(!get_parameter("use_sim_time").as_bool()||!declare_parameter("simulation_commissioning",false))throw std::runtime_error("SIMULATION_COMMISSIONING_REQUIRED");
  const char *domain_env=std::getenv("ROS_DOMAIN_ID");std::string domain=domain_env?domain_env:"0";
  domain=canonical_domain(domain);
  const std::string lock="/tmp/astribot_transport_domain_"+domain+".lock";
  // Canonical persistent location prevents an alternate journal parameter from
  // bypassing unresolved resources after a process restart.
  const char *home=std::getenv("HOME");if(!home)throw std::runtime_error("STATE_HOME_UNAVAILABLE");
  auto directory=std::filesystem::path(home)/".local/state/astribot/transport";
  std::filesystem::create_directories(directory);
  journal_=std::make_unique<ResourceJournal>(lock,(directory/("domain_"+domain+".jsonl")).string());
  for(const auto &[name,joints]:groups_) {
   names_.push_back(name);all_joints_.insert(joints.begin(),joints.end());
   clients_[name]=rclcpp_action::create_client<Trajectory>(this,"/"+name+"/follow_joint_trajectory");
  }
  authority_=std::make_unique<ResourceAuthority>(boot_epoch(),all_joints_,[this](const auto &record){journal_->append(record);},journal_->restored());
  hold_pub_=create_publisher<Hold>("/navigation/arm_hold",10);
  state_pub_=create_publisher<std_msgs::msg::String>("/transport/hold_executor/status",rclcpp::QoS(1).transient_local());
  geometry_sub_=create_subscription<Geometry>("/navigation/geometry_state",10,[this](Geometry::SharedPtr value){receive(*value);});
  controllers_=create_client<Query>("/controller_manager/list_controllers");
  renew_=create_service<astribot_s1_transport_native::srv::RenewHold>("/transport/hold_executor/renew",[this](astribot_s1_transport_native::srv::RenewHold::Request::SharedPtr request,astribot_s1_transport_native::srv::RenewHold::Response::SharedPtr response){
   response->accepted=authority_->renew(request->lease_id,request->resource_epoch,request->sequence,now().nanoseconds(),wall());
   response->reason=response->accepted?"RENEWED":authority_->reason();
  });
#ifdef PLAN_TO_HOLD_EXECUTOR
  planner_=rclcpp_action::create_client<Planner>(this,"/transport/plan_manipulation");
  scenes_=create_client<SceneQuery>("/get_planning_scene");
  guard_client_=create_client<GuardService>("/transport/execution_guard/set");
  revoke_client_=create_client<Revoke>("/navigation/set_robot_envelope");
  revalidate_client_=create_client<Revalidate>("/transport/revalidate_manipulation");
  odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::SharedPtr m){odom_=*m;odom_received_=wall();});
  envelope_sub_=create_subscription<Envelope>("/navigation/envelope_v2",10,[this](Envelope::SharedPtr m){envelope_=*m;envelope_received_=wall();++envelope_sequence_;if(owned()&&!hold_confirmed_&&!manipulation_ready(now().nanoseconds(),wall()))stopping(reason_);});
  guard_sub_=create_subscription<GuardState>("/transport/execution_guard/status",10,[this](GuardState::SharedPtr m){guard_state_=*m;guard_received_=wall();});
  initialize_payload();
  rcl_interfaces::msg::ParameterDescriptor immutable;immutable.read_only=true;
  navigation_position_tolerance_=declare_parameter("fixed_station_position_tolerance_m",.002,immutable);
  navigation_yaw_tolerance_=declare_parameter("fixed_station_yaw_tolerance_rad",.0017453292519943296,immutable);
  if(!std::isfinite(navigation_position_tolerance_)||navigation_position_tolerance_<=0||navigation_position_tolerance_>.002||
     !std::isfinite(navigation_yaw_tolerance_)||navigation_yaw_tolerance_<=0||navigation_yaw_tolerance_>.0017453292519943296)
    throw std::runtime_error("FIXED_STATION_TOLERANCE_INVALID");
#endif
  auto admit=[this](const auto goal,bool full){
#ifndef PLAN_TO_HOLD_EXECUTOR
    (void)full;
#endif
    const auto ros=now().nanoseconds(),steady=wall();
    if(owned()||!geometry_fresh(ros,steady)||!claims_fresh(ros,steady)||!exclusive_graph())return rclcpp_action::GoalResponse::REJECT;
#ifdef PLAN_TO_HOLD_EXECUTOR
    if(goal->context_id.empty()||goal->object_id.empty()||(goal->operation!="PICK"&&goal->operation!="PLACE")||!manipulation_ready(ros,steady)||!planner_->action_server_is_ready()||!scenes_->service_is_ready()||!guard_client_->service_is_ready()||!revoke_client_->service_is_ready())return rclcpp_action::GoalResponse::REJECT;
#endif
#ifdef PLAN_TO_HOLD_EXECUTOR
    if(full&&!full_ready(goal->object_id,ros,steady))return rclcpp_action::GoalResponse::REJECT;
#endif
    for(const auto &[name,client]:clients_)if(!client->action_server_is_ready()){reason_="CONTROLLER_ACTION_UNAVAILABLE";return rclcpp_action::GoalResponse::REJECT;}
    auto acquired=authority_->acquire(goal->task_id,goal->request_id,ros,steady);reason_=acquired.reason;
    return acquired.accepted?rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE:rclcpp_action::GoalResponse::REJECT;
  };
  server_=rclcpp_action::create_server<Action>(this,ACTION_NAME,
   [admit](const auto &,const auto goal){return admit(goal,false);},
   [this](const auto handle){if(handle!=parent_)return rclcpp_action::CancelResponse::REJECT;stopping("TASK_CANCELED");return rclcpp_action::CancelResponse::ACCEPT;},
   [this](const auto handle){parent_=handle;
#ifdef PLAN_TO_HOLD_EXECUTOR
    full_operation_=false;input_=*handle->get_goal();
#endif
    start();});
#ifdef PLAN_TO_HOLD_EXECUTOR
  full_server_=rclcpp_action::create_server<FullAction>(this,"/transport/manipulate_to_hold",
   [admit](const auto &,const auto goal){return admit(goal,true);},
   [this](const auto handle){if(handle!=full_parent_)return rclcpp_action::CancelResponse::REJECT;stopping("TASK_CANCELED");return rclcpp_action::CancelResponse::ACCEPT;},
   [this](const auto handle){full_parent_=handle;full_operation_=true;const auto &goal=*handle->get_goal();
    input_.task_id=goal.task_id;input_.request_id=goal.request_id;input_.context_id=goal.context_id;
    input_.operation=goal.operation;input_.object_id=goal.object_id;input_.pre_target=goal.pre_target;
    input_.target=goal.target;input_.exit_targets=goal.exit_targets;input_.touch_links=goal.touch_links;input_.grasp_width_m=goal.grasp_width_m;
    start();});
  transfer_server_=rclcpp_action::create_server<Transfer>(this,"/transport/fixed_station_transfer",
   [this,admit](const auto &,const auto goal){
    try {
     if(owned())return rclcpp_action::GoalResponse::REJECT;
     validate_transfer_goal(*goal);
     auto request=std::make_shared<Action::Goal>();request->task_id=goal->task_id;request->request_id=goal->request_id;
     request->context_id=goal->context_id;request->operation="PICK";request->object_id=goal->object_id;
     return admit(request,true);
    }catch(const std::exception &error){reason_=error.what();return rclcpp_action::GoalResponse::REJECT;}
   },
   [this](const auto handle){if(handle!=transfer_parent_)return rclcpp_action::CancelResponse::REJECT;stopping("TASK_CANCELED");return rclcpp_action::CancelResponse::ACCEPT;},
   [this](const auto handle){
    transfer_parent_=handle;full_operation_=true;operation_generation_=1;
    const auto &goal=*handle->get_goal();input_.task_id=goal.task_id;input_.request_id=goal.request_id;
    input_.object_id=goal.object_id;input_.operation="PICK";input_.context_id=goal.context_id+"_op_1_PICK";
    input_.touch_links=goal.touch_links;input_.grasp_width_m=goal.grasp_width_m;
    transfer_started_=wall();transfer_budget_=int64_t(goal.timeout_s*1000000000.);
    transfer_phase_=TransferPhase::PICK_PREPARE;navigation_started_=false;navigation_cleanup_revoke_=false;station_pending_=false;station_scene_unresolved_=false;station_ready_=false;final_scene_pending_=false;
    navigation_=std::make_unique<FixedStationNavigation>(*this,"/navigate_to_pose");
    start();hold_id_=authority_->lease_id()+"_hold_1_0";operation_started_ros_=now().nanoseconds();
    const auto *state=payload_consumer_->current(now().nanoseconds(),wall());
    if(!state){stopping("TRANSFER_INITIAL_INVENTORY_EXPIRED");return;}
    payload_observation_=state->observation;payload_observation_bound_=true;
    try{check_payload_raw();}catch(const std::exception &error){stopping(error.what());}
   });
#endif
  timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
 }
private:
#ifdef PLAN_TO_HOLD_EXECUTOR
 struct RawObservation {Observation value;PayloadCommand::Receipt receipt;};
 struct Diagnostic {nlohmann::json value;PayloadCommand::Receipt receipt;};
 enum class PayloadPhase {NONE,FRAME,PHYSICAL,APPLY,LEDGER,REVALIDATE,READBACK};
#endif
 bool owned() const {
#ifdef PLAN_TO_HOLD_EXECUTOR
  return parent_||full_parent_||transfer_parent_;
#else
  return bool(parent_);
#endif
 }
 bool parent_active() const {
#ifdef PLAN_TO_HOLD_EXECUTOR
  if(transfer_parent_)return transfer_parent_->is_active();
  if(full_parent_)return full_parent_->is_active();
#endif
  return parent_&&parent_->is_active();
 }
 void finish_parent(bool released) {
  auto finish=[&](auto handle,auto result) {
   if(!handle||!handle->is_active())return;
   const auto &cause=stop_at_?stop_reason_:reason_;
   result->resources_released=released;result->reason=released?cause:"RESOURCE_RECOVERY_REQUIRED:"+cause;
   result->lease_id=authority_->lease_id();
   if(released&&handle->is_canceling())handle->canceled(result);else handle->abort(result);
  };
  finish(parent_,std::make_shared<Action::Result>());
#ifdef PLAN_TO_HOLD_EXECUTOR
  finish(full_parent_,std::make_shared<FullAction::Result>());
  if(transfer_parent_&&transfer_parent_->is_active()) {
   auto result=transfer_result(false,released);
   if(released&&transfer_parent_->is_canceling())transfer_parent_->canceled(result);else transfer_parent_->abort(result);
  }
#endif
 }
#ifdef PLAN_TO_HOLD_EXECUTOR
 const Action::Goal &input() const{return input_;}
 std::string transfer_phase_name() const {
  switch(transfer_phase_) {
   case TransferPhase::PICK_PREPARE:case TransferPhase::PICK:return "PICK";
   case TransferPhase::NAVIGATE:return "NAVIGATE";
   case TransferPhase::PLACE_REVOKE:case TransferPhase::PLACE_PREPARE:case TransferPhase::PLACE:return "PLACE";
   case TransferPhase::FINAL_VERIFY:return "FINAL_VERIFY";
   case TransferPhase::COMPLETE:return "COMPLETE";
  }
  throw std::logic_error("FIXED_STATION_PHASE_INVALID");
 }
 std::shared_ptr<Transfer::Result> transfer_result(bool success,bool released) {
  auto result=std::make_shared<Transfer::Result>();result->success=success;result->resources_released=released;
  result->reason=success?"TRANSFER_COMPLETE":(released?stop_reason_:"RESOURCE_RECOVERY_REQUIRED:"+stop_reason_);
  result->cleanup_reason=stop_at_&&reason_!=stop_reason_?reason_:"";result->lease_id=authority_->lease_id();
  if(navigation_)result->navigation_goal_uuid=navigation_->status().nav_goal_uuid;
  if(payload_consumer_)if(const auto *state=payload_consumer_->current(now().nanoseconds(),wall()))result->final_attachment_revision=state->attachment_revision;
  return result;
 }
 FixedStations observe_stations(const Transfer::Goal &goal,int64_t ros,int64_t steady,bool bind) {
  (void)payload_world_base(ros,ros,steady);
  if(!world_snapshot_||ros<world_snapshot_stamp_||ros-world_snapshot_stamp_>=300*MS||
     steady-world_snapshot_->second.steady>=300*MS-(world_snapshot_->second.ros-world_snapshot_stamp_))throw std::runtime_error("STATION_WORLD_OBSERVATION_STALE");
  FixedStations result;
  for(size_t i=0;i<2;++i) {
   const auto &registered=goal.stations[i];size_t count=0;
   for(const auto &pose:world_snapshot_->first.pose())if(pose.name()==registered.id) {
    ++count;if(!pose.id())throw std::runtime_error("STATION_ENTITY_INVALID");
    if(bind)station_entities_[i]=pose.id();else if(station_entities_[i]!=pose.id())throw std::runtime_error("STATION_ENTITY_CHANGED");
    auto &value=result[i];value.id=registered.id;
    value.world_pose.position.x=pose.position().x();value.world_pose.position.y=pose.position().y();value.world_pose.position.z=pose.position().z();
    value.world_pose.orientation.x=pose.orientation().x();value.world_pose.orientation.y=pose.orientation().y();value.world_pose.orientation.z=pose.orientation().z();value.world_pose.orientation.w=pose.orientation().w();
    const auto normalized=fixed_station_target_in_base(value.world_pose,Eigen::Isometry3d::Identity());
    Eigen::Isometry3d actual,expected;tf2::fromMsg(normalized,actual);tf2::fromMsg(registered.pose,expected);
    const Eigen::Isometry3d difference=expected.inverse()*actual;
    if(difference.translation().norm()>1e-6||Eigen::AngleAxisd(difference.linear()).angle()>1e-6)throw std::runtime_error("FIXED_STATION_MOVED");
    for(size_t k=0;k<3;++k)value.size_xyz[k]=registered.primitives[0].dimensions[k];
   }
   if(count!=1)throw std::runtime_error("STATION_MODEL_NOT_UNIQUE");
  }
  return result;
 }
 void validate_transfer_goal(const Transfer::Goal &goal) {
  if(!payload_client_||goal.context_id.empty()||!std::isfinite(goal.timeout_s)||goal.timeout_s<=0||goal.timeout_s>540||
     goal.stations.size()!=2||goal.stations[0].id==goal.stations[1].id||goal.navigation_target.header.frame_id!="map")throw std::runtime_error("FIXED_STATION_GOAL_INVALID");
  // ResourceAuthority accepts at most 128 characters; the PLACE suffix uses 11.
  if(goal.context_id.size()>117||!std::all_of(goal.context_id.begin(),goal.context_id.end(),[](unsigned char c){return std::isalnum(c)||c=='_'||c=='-'||c=='.';}))
   throw std::runtime_error("FIXED_STATION_CONTEXT_INVALID");
  auto pose=[](const geometry_msgs::msg::PoseStamped &target){
   if(target.header.frame_id!="gazebo_world")throw std::runtime_error("STATION_TARGET_FRAME_INVALID");
   (void)fixed_station_target_in_base(target.pose,Eigen::Isometry3d::Identity());
  };
  pose(goal.pick_pre_target);pose(goal.pick_target);pose(goal.place_pre_target);pose(goal.place_target);
  if(goal.pick_exit_targets.empty()||goal.place_exit_targets.empty())throw std::runtime_error("STATION_EXIT_TARGET_REQUIRED");
  for(const auto &target:goal.pick_exit_targets)pose(target);for(const auto &target:goal.place_exit_targets)pose(target);
  (void)fixed_station_target_in_base(goal.navigation_target.pose,Eigen::Isometry3d::Identity());
  for(const auto &station:goal.stations) {
   if(station.id.empty()||station.id==goal.object_id||station.header.frame_id!="gazebo_world"||station.operation!=station.ADD||
      station.primitives.size()!=1||station.primitive_poses.size()!=1||station.primitives[0].type!=shape_msgs::msg::SolidPrimitive::BOX||
      station.primitives[0].dimensions.size()!=3||!station.meshes.empty()||!station.planes.empty())throw std::runtime_error("STATION_REGISTERED_BOX_REQUIRED");
   for(double size:station.primitives[0].dimensions)if(!std::isfinite(size)||size<=0)throw std::runtime_error("STATION_SIZE_INVALID");
   const auto &shape=station.primitive_poses[0];
   if(shape.position.x!=0||shape.position.y!=0||shape.position.z!=0||shape.orientation.x!=0||shape.orientation.y!=0||shape.orientation.z!=0||std::abs(shape.orientation.w)!=1)throw std::runtime_error("STATION_MODEL_ORIGIN_UNREGISTERED");
   (void)fixed_station_target_in_base(station.pose,Eigen::Isometry3d::Identity());
  }
  const auto ros=now().nanoseconds(),steady=wall();
  const auto *state=payload_consumer_->current(ros,steady);
  if(!state||!state->observation.full_inventory||state->observation.status!=Observation::EMPTY||!state->observation.objects.empty())throw std::runtime_error("TRANSFER_REQUIRES_CONFIRMED_EMPTY");
  (void)observe_stations(goal,ros,steady,true);
 }
 bool navigation_revoked(int64_t ros,int64_t steady) {
  if(revoke_pending_&&steady-revoke_sent_>3000*MS){revoke_client_->remove_pending_request(revoke_request_);revoke_pending_=false;throw std::runtime_error("NAVIGATION_REVOCATION_TIMEOUT");}
  if(!revoke_ack_)return false;
  if(revocation_verified_)return true;
  if(!revoke_wait_negative_)revocation_verified_=true;
  else if(envelope_&&envelope_sequence_>revoke_envelope_sequence_&&ns(envelope_->header.stamp)>=revoke_ros_&&ros>=ns(envelope_->header.stamp)&&ros-ns(envelope_->header.stamp)<=300*MS&&steady-envelope_received_<=300*MS) {
   if(envelope_->epoch!=revoke_epoch_||(!reference_envelope_.coordinator_session_id.empty()&&envelope_->coordinator_session_id!=reference_envelope_.coordinator_session_id))throw std::runtime_error("MTC_COORDINATOR_CHANGED");
   if(envelope_->mode!=Envelope::HOLD&&envelope_->mode!=Envelope::FIXED_POSTURE)throw std::runtime_error("MTC_NAVIGATION_NOT_REVOKED");
   if(!envelope_->navigation_allowed){reference_envelope_=*envelope_;revocation_verified_=true;}
  }
  if(!revocation_verified_&&steady-revoke_ack_at_>3000*MS)throw std::runtime_error("NAVIGATION_REVOCATION_READBACK_TIMEOUT");
  return revocation_verified_;
 }
 void prepare_station_scene() {
  if(station_pending_||station_ready_)return;
  station_pending_=true;station_sent_=wall();const auto lease=authority_->lease_id(),context=input().context_id;
  const auto generation=stage_generation_;
  auto request=std::make_shared<SceneQuery::Request>();request->components.components=1023;
  scenes_->async_send_request(request,[this,lease,context,generation](rclcpp::Client<SceneQuery>::SharedFuture future){
   if(!owned()||lease!=authority_->lease_id()||generation!=stage_generation_||context!=input().context_id)return;
   if(stop_at_){station_pending_=false;return;}
   try {
    const auto ros=now().nanoseconds(),steady=wall();check_payload_raw();
    if(!geometry_fresh(ros,steady)||!manipulation_ready(ros,steady)||!navigation_revoked(ros,steady))throw std::runtime_error("STATION_PREPARATION_EVIDENCE_EXPIRED:"+reason_);
    const auto &goal=*transfer_parent_->get_goal();const auto stations=observe_stations(goal,ros,steady,false);
    const auto capture=world_snapshot_stamp_;auto world_base=payload_world_base(capture,ros,steady);
    if(capture<operation_started_ros_||!world_base)throw std::runtime_error("STATION_WORLD_BASE_UNCONFIRMED");
    station_before_=future.get()->scene;
    if(input().operation=="PICK"&&!station_before_.robot_state.attached_collision_objects.empty())throw std::runtime_error("TRANSFER_INITIAL_SCENE_NOT_EMPTY");
    station_diff_=fixed_station_scene_diff(station_before_,stations,*world_base,known_links_);
    auto convert=[&](const geometry_msgs::msg::PoseStamped &target){auto converted=target;converted.header.frame_id=payload_base_;converted.header.stamp=rclcpp::Time(capture);converted.pose=fixed_station_target_in_base(target.pose,*world_base);return converted;};
    const bool pick=input().operation=="PICK";
    input_.pre_target=convert(pick?goal.pick_pre_target:goal.place_pre_target);input_.target=convert(pick?goal.pick_target:goal.place_target);
    input_.exit_targets.clear();for(const auto &target:pick?goal.pick_exit_targets:goal.place_exit_targets)input_.exit_targets.push_back(convert(target));
    reference_base_=*odom_;reference_geometry_=*geometry_;
    if(authority_->phase()!=ResourcePhase::RESERVED)throw std::runtime_error("STATION_SCENE_REQUIRES_RESERVED_OPERATION");
    authority_->submitted(now().nanoseconds(),wall());
    if(!authority_->record("station_scene_submission",{{"context",context},{"operation",operation_generation_},{"capture",capture},{"station_ids",{stations[0].id,stations[1].id}}}))throw std::runtime_error(authority_->reason());
    station_scene_unresolved_=true;station_sent_=wall();
    auto apply=std::make_shared<SceneApply::Request>();apply->scene=station_diff_;
    apply_client_->async_send_request(apply,[this,lease,context,generation](rclcpp::Client<SceneApply>::SharedFuture applied){
     if(!owned()||lease!=authority_->lease_id()||generation!=stage_generation_||context!=input().context_id)return;
     try {
      if(!applied.get()->success)throw std::runtime_error("STATION_SCENE_APPLY_REJECTED");
      station_sent_=wall();auto read=std::make_shared<SceneQuery::Request>();read->components.components=1023;
      scenes_->async_send_request(read,[this,lease,context,generation](rclcpp::Client<SceneQuery>::SharedFuture reply){
       if(!owned()||lease!=authority_->lease_id()||generation!=stage_generation_||context!=input().context_id)return;
       try {
        check_payload_raw();validate_fixed_station_scene(station_before_,reply.get()->scene,station_diff_,known_links_);
        station_scene_unresolved_=false;station_pending_=false;station_ready_=true;
        if(!authority_->record("station_scene_confirmed",{{"context",context},{"operation",operation_generation_}}))throw std::runtime_error(authority_->reason());
        if(stop_at_)return;
        transfer_phase_=input().operation=="PICK"?TransferPhase::PICK:TransferPhase::PLACE;query_scene(false);
       }catch(const std::exception &error){stopping(error.what());}
      });
     }catch(const std::exception &error){stopping(error.what());}
    });
   }catch(const std::exception &error){station_pending_=false;stopping(error.what());}
  });
 }
 void continue_place(int64_t ros,int64_t steady) {
  check_payload_raw();
  const std::string context=transfer_parent_->get_goal()->context_id+"_op_2_PLACE";
  if(!navigation_->status().cleanup_complete||!children_->all_terminal()||!hold_.status(ros,steady).hold_confirmed||!manipulation_ready(ros,steady))throw std::runtime_error("PLACE_CONTINUATION_NOT_STATIONARY");
  if(!authority_->continue_from_hold(authority_->lease_id(),authority_->epoch(),context,1,true,true,ros,steady))throw std::runtime_error("PLACE_CONTINUATION_NOT_COMMITTED:"+authority_->reason());
  ++operation_generation_;++stage_generation_;input_.operation="PLACE";input_.context_id=context;
  // Retain the lease, its renewal sequence, inventory binding/raw failure and
  // motion history. Only completed per-operation child/planner state changes.
  auto child_names=names_;child_names.push_back("mtc");children_=std::make_unique<ChildActions>(child_names);
  handles_.clear();cancel_sent_.clear();planner_handle_.reset();planner_cancel_sent_=false;plan_.reset();
  hold_.cancel();hold_started_=false;hold_confirmed_=false;terminal_barrier_=false;settling_.clear();
  hold_id_=authority_->lease_id()+"_hold_2_0";motion_sent_=false;scene_verified_=false;scene_final_verified_=false;scene_pending_=false;endpoint_.clear();
  guard_ack_=false;guard_requested_=false;guard_pending_=false;guard_disarm_requested_=false;guard_disarmed_=false;
  scene_stabilizing_=false;scene_retry_at_=0;revalidation_pending_=false;revalidation_readback_=false;
  reference_geometry_=*geometry_;reference_base_=*odom_;reference_envelope_=*envelope_;
  station_ready_=false;station_pending_=false;operation_started_ros_=ros;started_=steady;
  transfer_phase_=TransferPhase::PLACE_PREPARE;
 }
 void verify_transfer_complete() {
  if(final_scene_pending_)return;
  const auto ros=now().nanoseconds(),steady=wall();check_payload_raw();
  const auto *state=payload_consumer_->current(ros,steady);
  if(!state||!state->observation.full_inventory||state->observation.status!=Observation::EMPTY||!state->observation.objects.empty()||
     state->attachment_revision!=geometry_->attachment_revision||!geometry_->attachment_ids.empty())throw std::runtime_error("TRANSFER_FINAL_EMPTY_UNCONFIRMED");
  require_payload_observation_bound(payload_observation_,state->observation);
  auto physical=payload_client_->observation(ros,steady);if(!physical)return;
  if(physical->at("attached").get<bool>()||physical->at("command_id").get<uint32_t>()!=payload_client_->id()||
     (physical_object(*physical).translation()-placement_requested_world_).norm()>placement_tolerance_||
     (physical_object(*physical).translation()-placement_first_).norm()>.005)throw std::runtime_error("TRANSFER_FINAL_PLACEMENT_CHANGED");
  if(!hold_.status(ros,steady).hold_confirmed||!manipulation_ready(ros,steady)||!navigation_->status().cleanup_complete||!children_->all_terminal())throw std::runtime_error("TRANSFER_FINAL_HOLD_UNCONFIRMED");
  final_scene_pending_=true;final_checked_at_=steady;
  const auto lease=authority_->lease_id();const auto generation=stage_generation_;
  auto request=std::make_shared<SceneQuery::Request>();request->components.components=1023;
  scenes_->async_send_request(request,[this,lease,generation](rclcpp::Client<SceneQuery>::SharedFuture response){
   if(!owned()||lease!=authority_->lease_id()||generation!=stage_generation_)return;
   final_scene_pending_=false;if(stop_at_)return;
   try {
    check_payload_raw();const auto scene=response.get()->scene;
    if(!scene.robot_state.attached_collision_objects.empty())throw std::runtime_error("TRANSFER_FINAL_SCENE_NOT_EMPTY");
    validate_payload_scene(payload_before_,scene,payload_observation_,payload_object_,false,payload_base_object_,payload_base_,known_links_);
    const auto ros=now().nanoseconds(),steady=wall();const auto *state=payload_consumer_->current(ros,steady);
    if(!state||!state->observation.full_inventory||state->observation.status!=Observation::EMPTY||!state->observation.objects.empty()||
       !geometry_fresh(ros,steady)||state->attachment_revision!=geometry_->attachment_revision||!geometry_->attachment_ids.empty())throw std::runtime_error("TRANSFER_FINAL_EMPTY_EXPIRED");
    require_payload_observation_bound(payload_observation_,state->observation);
    auto physical=payload_client_->observation(ros,steady);
    if(!physical||physical->at("attached").get<bool>()||physical->at("command_id").get<uint32_t>()!=payload_client_->id()||
       (physical_object(*physical).translation()-placement_requested_world_).norm()>placement_tolerance_||
       (physical_object(*physical).translation()-placement_first_).norm()>.005)throw std::runtime_error("TRANSFER_FINAL_PHYSICAL_UNCONFIRMED");
    if(!hold_.status(ros,steady).hold_confirmed||!manipulation_ready(ros,steady)||!children_->all_terminal()||!navigation_->status().cleanup_complete||
       payload_scene_unresolved_||station_scene_unresolved_||payload_client_->unresolved())throw std::runtime_error("TRANSFER_FINAL_BARRIER_UNCONFIRMED");
    if(!authority_->record("transfer_empty_confirmed",{{"attachment_revision",state->attachment_revision},{"physical_revision",state->observation.revision},{"navigation_goal_uuid",navigation_->status().nav_goal_uuid}}))throw std::runtime_error(authority_->reason());
    if(!authority_->complete(lease,authority_->epoch(),true,true,true,ros,steady))throw std::runtime_error("TRANSFER_COMPLETION_NOT_COMMITTED:"+authority_->reason());
    transfer_phase_=TransferPhase::COMPLETE;reason_="TRANSFER_COMPLETE";
    auto feedback=std::make_shared<Transfer::Feedback>();feedback->phase="COMPLETE";feedback->reason=reason_;feedback->lease_id=lease;
    feedback->resource_epoch=authority_->epoch();feedback->context_id=transfer_parent_->get_goal()->context_id;feedback->object_id=input().object_id;feedback->navigation_goal_uuid=navigation_->status().nav_goal_uuid;
    transfer_parent_->publish_feedback(feedback);transfer_parent_->succeed(transfer_result(true,true));transfer_parent_.reset();hold_.cancel();hold_confirmed_=false;
   }catch(const std::exception &error){stopping(error.what());}
  });
 }
 void advance_transfer(int64_t ros,int64_t steady) {
  if(!transfer_parent_)return;
  if(steady-transfer_started_>transfer_budget_)throw std::runtime_error("TRANSFER_TIMEOUT");
  check_payload_raw();
  if(station_pending_&&steady-station_sent_>3000*MS)throw std::runtime_error("STATION_SCENE_UNCONFIRMED");
  if(final_scene_pending_&&steady-final_checked_at_>3000*MS)throw std::runtime_error("TRANSFER_FINAL_SCENE_TIMEOUT");
  if(transfer_phase_==TransferPhase::PICK_PREPARE||transfer_phase_==TransferPhase::PLACE_PREPARE) {
   (void)payload_world_base(ros,ros,steady);
   if(!world_snapshot_||world_snapshot_stamp_<operation_started_ros_)return;
   if(navigation_revoked(ros,steady))prepare_station_scene();return;
  }
  if(transfer_phase_==TransferPhase::PICK&&hold_confirmed_) {
   const auto *state=payload_consumer_->current(ros,steady);
   if(!state||!state->observation.full_inventory||state->observation.status!=Observation::ATTACHED||state->observation.objects.size()!=1||state->observation.objects[0].object.id!=payload_object_)throw std::runtime_error("TRANSFER_ATTACHED_BOX_UNCONFIRMED");
   auto limits=transfer_parent_->get_goal()->navigation_limits;limits.payload_mass_kg=state->observation.objects[0].weight;
   FixedStationNavigation::Request request;request.task_id=input().task_id;request.context_id=transfer_parent_->get_goal()->context_id+"_NAV";
   request.lease_id=authority_->lease_id();request.resource_epoch=authority_->epoch();request.reference_geometry=*geometry_;
   request.fixed=hold_.request(request.context_id,limits,ros,steady);request.navigation_target=transfer_parent_->get_goal()->navigation_target;
   request.robot_base_frame=payload_base_;request.position_tolerance_m=navigation_position_tolerance_;request.yaw_tolerance_rad=navigation_yaw_tolerance_;
   if(!authority_->record("navigation_leg_submission",{{"context",request.context_id},{"hold_id",request.fixed.hold_id}}))throw std::runtime_error(authority_->reason());
   navigation_started_=true;transfer_phase_=TransferPhase::NAVIGATE;navigation_->start(std::move(request));return;
  }
  if(transfer_phase_==TransferPhase::NAVIGATE) {
   if(!hold_.status(ros,steady).hold_confirmed)throw std::runtime_error("TRANSFER_NAVIGATION_HOLD_LOST");
   navigation_->tick();const auto &status=navigation_->status();
   if(status.phase==FixedStationNavigation::Phase::FAILED||status.phase==FixedStationNavigation::Phase::UNRESOLVED||status.phase==FixedStationNavigation::Phase::CANCELING)throw std::runtime_error(status.reason);
   if(status.phase==FixedStationNavigation::Phase::SUCCEEDED) {
    if(!authority_->record("navigation_leg_confirmed",{{"uuid",status.nav_goal_uuid},{"session",status.envelope_session},{"epoch",status.envelope_epoch},{"geometry_hash",status.geometry_hash}}))throw std::runtime_error(authority_->reason());
    transfer_phase_=TransferPhase::PLACE_REVOKE;revoke_ack_=false;revocation_verified_=false;revoke_navigation();
   }return;
  }
  if(transfer_phase_==TransferPhase::PLACE_REVOKE) {if(navigation_revoked(ros,steady))continue_place(ros,steady);return;}
  if(transfer_phase_==TransferPhase::PLACE&&hold_confirmed_)transfer_phase_=TransferPhase::FINAL_VERIFY;
  if(transfer_phase_==TransferPhase::FINAL_VERIFY)verify_transfer_complete();
 }
 void check_payload_raw() {
  if(!payload_raw_failure_.empty())throw std::runtime_error(payload_raw_failure_);
  if(!payload_observation_bound_)return;
  try {require_payload_observation_bound(payload_observation_,observations_.back().value);}
  catch(const std::exception &error){payload_raw_failure_=error.what();throw;}
 }
 void initialize_payload() {
  rcl_interfaces::msg::ParameterDescriptor fixed;fixed.read_only=true;
  payload_model_=declare_parameter<std::string>("payload_model","",fixed);
  if(payload_model_.empty())return;
  payload_object_=declare_parameter<std::string>("payload_object_id","",fixed);
  payload_world_=declare_parameter<std::string>("payload_world","",fixed);
  payload_robot_=declare_parameter<std::string>("payload_robot_model","astribot_s1",fixed);
  payload_base_=declare_parameter<std::string>("payload_base_frame","astribot_torso_base",fixed);
  payload_tcp_=declare_parameter<std::string>("payload_tcp_frame","astribot_arm_left_tcp_link",fixed);
  payload_xml_=declare_parameter<std::string>("robot_description","",fixed);
  payload_config_.environment="simulation";
  payload_config_.session=declare_parameter<std::string>("payload_session_id","",fixed);
  payload_config_.source=declare_parameter<std::string>("payload_source_id","",fixed);
  payload_config_.allowed_links={"astribot_arm_left_tcp_link","astribot_arm_right_tcp_link"};
  payload_size_=declare_parameter<std::vector<double>>("payload_size_xyz",{},fixed);
  const auto root_pose=declare_parameter<std::vector<double>>("payload_model_from_root",{},fixed);
  placement_tolerance_=declare_parameter("placement_tolerance_m",.025,fixed);
  if(payload_object_.empty()||payload_world_.empty()||payload_xml_.empty()||payload_config_.session.empty()||payload_config_.source.empty()||
     payload_base_!="astribot_torso_base"||payload_tcp_!="astribot_arm_left_tcp_link"||payload_size_.size()!=3||root_pose.size()!=7||
     !std::isfinite(placement_tolerance_)||placement_tolerance_<=0||placement_tolerance_>.025)throw std::runtime_error("PAYLOAD_REGISTRATION_REQUIRED");
  double radius2=0;for(double d:payload_size_){if(!std::isfinite(d)||d<=0)throw std::runtime_error("PAYLOAD_SIZE_INVALID");radius2+=d*d;}
  payload_radius_=std::sqrt(radius2)/2.;
  for(double d:root_pose)if(!std::isfinite(d))throw std::runtime_error("PAYLOAD_MODEL_ROOT_TRANSFORM_INVALID");
  Eigen::Quaterniond q(root_pose[6],root_pose[3],root_pose[4],root_pose[5]);
  if(std::abs(q.norm()-1.)>1e-6)throw std::runtime_error("PAYLOAD_MODEL_ROOT_TRANSFORM_INVALID");
  model_from_root_.translation()=Eigen::Vector3d(root_pose[0],root_pose[1],root_pose[2]);model_from_root_.linear()=q.toRotationMatrix();
  payload_consumer_=std::make_unique<astribot::payload::Consumer>(payload_config_);
  const auto clock=get_clock();payload_client_=std::make_unique<PayloadClient>(payload_model_,[clock]{return clock->now().nanoseconds();});
  tf_=std::make_unique<tf2_ros::Buffer>(clock);tf_listener_=std::make_unique<tf2_ros::TransformListener>(*tf_,this,false);
  apply_client_=create_client<SceneApply>("/apply_planning_scene");
  payload_revalidate_=create_client<PayloadRevalidate>("/transport/revalidate_payload_transition");
  payload_state_sub_=create_subscription<PayloadState>("/payload/attachment_state",rclcpp::QoS(4).reliable(),[this](PayloadState::ConstSharedPtr value){payload_consumer_->receive(*value,now().nanoseconds(),wall());});
  observation_sub_=create_subscription<Observation>("/payload/attachment_observation",rclcpp::QoS(32).reliable(),[this](Observation::ConstSharedPtr value){
   PayloadCommand::Receipt receipt{now().nanoseconds(),wall()};
   if(value->environment!=payload_config_.environment||value->session_id!=payload_config_.session||value->source_id!=payload_config_.source)return;
   if(full_operation_&&owned()&&(value->source_epoch!=payload_epoch_||value->clock_epoch!=payload_clock_)){if(payload_observation_bound_)payload_raw_failure_="PAYLOAD_RAW_CONTEXT_CHANGED";stopping("PAYLOAD_SOURCE_CONTEXT_CHANGED");return;}
   if(!observations_.empty()) {
    const auto &last=observations_.back();
    if(value->source_epoch==last.value.source_epoch&&value->clock_epoch==last.value.clock_epoch) {
     if(value->sequence<last.value.sequence)return;
     if(value->sequence==last.value.sequence) {
      if(full_operation_&&owned()&&payload_observation_bound_&&*value!=last.value){payload_raw_failure_="PAYLOAD_RAW_CAPTURE_CONFLICT";stopping(payload_raw_failure_);}
      return;
     }
     if(value->observed_at==last.value.observed_at)receipt=last.receipt;
    }
   }
   observations_.push_back({*value,receipt});while(observations_.size()>32)observations_.pop_front();
   if(full_operation_&&owned()&&payload_observation_bound_)try {check_payload_raw();}catch(const std::exception &error){stopping(error.what());}
  });
  diagnostic_sub_=create_subscription<std_msgs::msg::String>("/payload/simulation_inventory_diagnostics",rclcpp::QoS(32).reliable(),[this](std_msgs::msg::String::ConstSharedPtr message){
   try {
    auto value=nlohmann::json::parse(message->data);PayloadCommand::Receipt receipt{now().nanoseconds(),wall()};
    if(value.at("world")!=payload_world_||value.at("robot_model")!=payload_robot_||value.at("policy")!="kinematic_inventory_v1")throw std::runtime_error("PAYLOAD_INVENTORY_SCOPE_CHANGED");
    if(full_operation_&&owned()&&(value.at("source_epoch")!=payload_epoch_||value.at("clock_epoch").get<uint64_t>()!=payload_clock_))throw std::runtime_error("PAYLOAD_SOURCE_CONTEXT_CHANGED");
    if(!diagnostics_.empty()) {
     const auto &last=diagnostics_.back();
     if(value.at("source_epoch")==last.value.at("source_epoch")&&value.at("clock_epoch")==last.value.at("clock_epoch")) {
      if(value.at("sequence").get<uint64_t>()<=last.value.at("sequence").get<uint64_t>())return;
      if(value.at("stamp_ns")==last.value.at("stamp_ns"))receipt=last.receipt;
     }
    }
    diagnostics_.push_back({std::move(value),receipt});while(diagnostics_.size()>32)diagnostics_.pop_front();
   }catch(const std::exception &error){if(full_operation_&&owned())stopping(error.what());else RCLCPP_ERROR(get_logger(),"Payload diagnostic: %s",error.what());}
  });
  joints_sub_=create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::ConstSharedPtr value){
   const auto ros=now().nanoseconds(),steady=wall(),at=ns(value->header.stamp);
   if(at>ros)return;
   if(actual_joints_&&at<=ns(actual_joints_->header.stamp))return;
   if(at<=0||ros-at>=300*MS||value->name.size()!=value->position.size()){actual_joints_.reset();return;}
   sensor_msgs::msg::JointState sample;sample.header=value->header;std::set<std::string> seen;
   for(size_t i=0;i<value->name.size();++i)if(all_joints_.count(value->name[i])) {
    if(!seen.insert(value->name[i]).second||!std::isfinite(value->position[i])){actual_joints_.reset();return;}
    sample.name.push_back(value->name[i]);sample.position.push_back(value->position[i]);
   }
   if(seen!=all_joints_){actual_joints_.reset();return;}
   actual_joints_=std::move(sample);joints_deadline_=steady+(300*MS-(ros-at));
  });
  world_inbox_=std::make_shared<PayloadWorldInbox>();world_node_=std::make_unique<ignition::transport::Node>();
  std::weak_ptr<PayloadWorldInbox> weak=world_inbox_;
  std::function<void(const ignition::msgs::Pose_V&)> receive=[weak,clock](const auto &message){
   auto inbox=weak.lock();if(!inbox)return;
   const auto ticket=inbox->ticket();
   inbox->receive(message,{clock->now().nanoseconds(),wall()},ticket);
  };
  if(!world_node_->Subscribe("/world/"+payload_world_+"/pose/info",receive))throw std::runtime_error("PAYLOAD_WORLD_SUBSCRIPTION_FAILED");
 }
 std::pair<const RawObservation*,const Diagnostic*> payload_sample(int64_t ros,int64_t steady) const {
  for(auto o=observations_.rbegin();o!=observations_.rend();++o)for(auto d=diagnostics_.rbegin();d!=diagnostics_.rend();++d) {
   const auto &v=o->value;const auto &j=d->value;
   if(j.at("source_epoch")!=v.source_epoch||j.at("clock_epoch").get<uint64_t>()!=v.clock_epoch||j.at("revision").get<uint64_t>()!=v.revision||j.at("sequence").get<uint64_t>()!=v.sequence||j.at("stamp_ns").get<int64_t>()!=ns(v.observed_at))continue;
   const auto at=astribot::payload::ns(v.observed_at),until=astribot::payload::ns(v.valid_until);
   if(at<=0||until<=ros||ros<at||until-at>300*MS||until<=at)continue;
   bool fresh=true;for(const auto &r:{o->receipt,d->receipt})fresh=fresh&&r.ros>=at&&r.ros<=ros&&r.steady<=steady&&steady-r.steady<until-r.ros;
   if(fresh)return {&*o,&*d};
  }
  return {};
 }
 bool full_ready(const std::string &object,int64_t ros,int64_t steady) {
  try {
   if(!payload_client_||object!=payload_object_||!apply_client_->service_is_ready()||!payload_revalidate_->service_is_ready())throw std::runtime_error("FULL_MANIPULATION_REGISTRATION_OR_SERVICES_MISSING");
   const auto *state=payload_consumer_->current(ros,steady);const auto sample=payload_sample(ros,steady);
   if(!state||!sample.first||state->attachment_revision!=geometry_->attachment_revision||!sample.first->value.full_inventory||
      state->observation.source_epoch!=sample.first->value.source_epoch||state->observation.clock_epoch!=sample.first->value.clock_epoch||state->observation.revision!=sample.first->value.revision)throw std::runtime_error("PAYLOAD_INITIAL_INVENTORY_UNCONFIRMED");
   uint64_t entity=0;size_t matches=0;
   for(const auto &model:sample.second->value.at("models"))if(model.at(1)==payload_robot_){entity=model.at(0).get<uint64_t>();++matches;}
   if(matches!=1||!entity)throw std::runtime_error("PAYLOAD_ROBOT_ENTITY_AMBIGUOUS");
   payload_frames_=std::make_unique<PayloadFrames>(payload_xml_,payload_base_,payload_tcp_,payload_robot_,entity,model_from_root_);
   known_links_=payload_frames_->known_links();
   // Epoch binding and its inbox are one boundary. Callbacks that started on
   // the previous generation cannot append after this clear, and source
   // captures older than the new binding cannot seed the new world buffer.
   world_inbox_->bind(ros);
   payload_epoch_=state->observation.source_epoch;payload_clock_=state->observation.clock_epoch;payload_ledger_epoch_=state->ledger_epoch;
   return true;
  }catch(const std::exception &error){reason_=error.what();return false;}
 }
 bool payload_stopped(int64_t ros,int64_t steady) {
  if(!actual_joints_||ros<ns(actual_joints_->header.stamp)||ros-ns(actual_joints_->header.stamp)>=300*MS||steady>=joints_deadline_){reason_="PAYLOAD_JOINT_STATE_STALE";return false;}
  for(size_t i=0;i<actual_joints_->name.size();++i)if(std::abs(actual_joints_->position[i]-payload_stopped_joints_.at(actual_joints_->name[i]))>payload_joint_bounds_.at(actual_joints_->name[i])){reason_="PAYLOAD_JOINTS_MOVED";return false;}
  return claims_fresh(ros,steady)&&exclusive_graph()&&manipulation_ready(ros,steady);
 }
 void begin_payload(const moveit_msgs::msg::PlanningScene &scene) {
  if(!full_operation_)throw std::runtime_error("PAYLOAD_REQUIRES_FULL_OPERATION");
  payload_before_=scene;payload_started_=wall();payload_phase_=PayloadPhase::FRAME;
  payload_physical_applied_=false;payload_scene_unresolved_=false;payload_query_pending_=false;
  payload_transaction_=authority_->lease_id()+":payload:"+(transfer_parent_?std::to_string(operation_generation_)+":":"")+std::to_string(plan_->index());
  payload_stopped_joints_.clear();payload_joint_bounds_.clear();placement_first_ros_=placement_first_wall_=0;
  for(size_t i=0;i<geometry_->joints.name.size();++i){payload_stopped_joints_[geometry_->joints.name[i]]=geometry_->joints.position[i];payload_joint_bounds_[geometry_->joints.name[i]]=geometry_->joint_position_error_bounds[i];}
  reason_="WAITING_FOR_PHYSICAL_PAYLOAD_FRAME";
 }
 std::optional<Eigen::Isometry3d> payload_world_base(int64_t capture,int64_t ros,int64_t steady) {
  const auto samples=world_inbox_->take();
  for(const auto &[value,receipt]:samples) {
   const auto &stamp=value.header().stamp();
   if(stamp.sec()<0||stamp.sec()>INT32_MAX||stamp.nsec()<0||stamp.nsec()>=1000000000)throw std::runtime_error("PAYLOAD_WORLD_STAMP_INVALID");
   const int64_t at=stamp.sec()*1000000000+stamp.nsec();
   if(at<=0||receipt.ros<at||ros<at||ros-at>=300*MS||steady-receipt.steady>=300*MS-(receipt.ros-at))continue;
   if(payload_frames_)payload_frames_->observe(value,receipt);
   if(!world_snapshot_||at>world_snapshot_stamp_){world_snapshot_=std::make_pair(value,receipt);world_snapshot_stamp_=at;}
  }
  return payload_frames_?payload_frames_->world_from_base(capture,ros,steady):std::nullopt;
 }
 Eigen::Isometry3d physical_object(const nlohmann::json &state) const {
  const auto pose=state.at("actual_world_xyzw").get<std::vector<double>>();
  Eigen::Isometry3d result=Eigen::Isometry3d::Identity();
  result.translation()=Eigen::Vector3d(pose.at(0),pose.at(1),pose.at(2));
  result.linear()=Eigen::Quaterniond(pose.at(6),pose.at(3),pose.at(4),pose.at(5)).toRotationMatrix();return result;
 }
 void payload_query_scene(bool final) {
  if(payload_query_pending_)return;
  auto request=std::make_shared<SceneQuery::Request>();request->components.components=1023;
  const auto lease=authority_->lease_id();const auto generation=stage_generation_;
  payload_query_pending_=true;payload_query_final_=final;payload_sent_=wall();
  auto pending=scenes_->async_send_request(request,[this,lease,generation,final](rclcpp::Client<SceneQuery>::SharedFuture future){
   if(!owned()||lease!=authority_->lease_id()||generation!=stage_generation_)return;
   payload_query_pending_=false;
   try {
    check_payload_raw();
    auto scene=future.get()->scene;
    validate_payload_scene(payload_before_,scene,payload_observation_,payload_object_,input().operation=="PICK",payload_base_object_,payload_base_,known_links_);
    if(final&&bind_scene(scene,known_links_)!=payload_revalidated_binding_)throw std::runtime_error("PAYLOAD_SCENE_CHANGED_AFTER_REVALIDATION");
    payload_readback_=std::move(scene);payload_checked_at_=wall();
    if(final)payload_phase_=PayloadPhase::READBACK;else payload_phase_=PayloadPhase::LEDGER;
   }catch(const std::exception &error){stopping(error.what());}
  });payload_request_=pending.request_id;
 }
 void advance_payload(int64_t ros,int64_t steady) {
  if(payload_phase_==PayloadPhase::NONE)return;
  check_payload_raw();
  if(payload_phase_==PayloadPhase::FRAME&&stop_at_){payload_phase_=PayloadPhase::NONE;return;}
  if(!payload_stopped(ros,steady))throw std::runtime_error(reason_);
  if(steady-payload_started_>30000*MS)throw std::runtime_error("PAYLOAD_TRANSACTION_TIMEOUT");
  const auto sample=payload_sample(ros,steady);
  if(!sample.first){if(steady-payload_started_>3000*MS)throw std::runtime_error("PAYLOAD_INVENTORY_NOT_FRESH");return;}
  const auto &raw=sample.first->value;const auto &diagnostic=sample.second->value;
  if(raw.source_epoch!=payload_epoch_||raw.clock_epoch!=payload_clock_)throw std::runtime_error("PAYLOAD_SOURCE_CONTEXT_CHANGED");
  const bool attach=input().operation=="PICK";
  if(payload_phase_==PayloadPhase::FRAME) {
   if(steady-payload_started_>3000*MS)throw std::runtime_error("PAYLOAD_FRAME_TIMEOUT");
   auto physical=payload_client_->observation(ros,steady);if(!physical)return;
   const auto at=physical->at("stamp_ns").get<int64_t>();
   auto world_base=payload_world_base(at,ros,steady);if(!world_base)return;
   const tf2::TimePoint stamp{std::chrono::nanoseconds(at)};
   if(!tf_->canTransform(payload_base_,payload_frames_->parent_link(),stamp))return;
   const auto base_parent=tf2::transformToEigen(tf_->lookupTransform(payload_base_,payload_frames_->parent_link(),stamp));
   const auto world_object=physical_object(*physical);const Eigen::Isometry3d base_object=world_base->inverse()*world_object;
   Eigen::Isometry3d command=world_object;
   if(attach) {
    const auto &objects=payload_before_.world.collision_objects;
    auto object=std::find_if(objects.begin(),objects.end(),[this](const auto &o){return o.id==payload_object_;});
    if(object==objects.end()||object->header.frame_id!=payload_base_||object->primitives.size()!=1||object->primitive_poses.size()!=1||object->primitives[0].type!=shape_msgs::msg::SolidPrimitive::BOX||!object->meshes.empty()||!object->planes.empty())throw std::runtime_error("PAYLOAD_REGISTERED_BOX_REQUIRED");
    for(size_t i=0;i<3;++i)if(std::abs(object->primitives[0].dimensions.at(i)-payload_size_[i])>1e-9)throw std::runtime_error("PAYLOAD_REGISTERED_SIZE_CHANGED");
    Eigen::Isometry3d object_pose,shape_pose;tf2::fromMsg(object->pose,object_pose);tf2::fromMsg(object->primitive_poses[0],shape_pose);
    if((shape_pose.matrix()-Eigen::Isometry3d::Identity().matrix()).norm()>1e-12)throw std::runtime_error("PAYLOAD_MODEL_OBJECT_ORIGIN_UNREGISTERED");
    const Eigen::Isometry3d difference=object_pose.inverse()*base_object;
    if(difference.translation().norm()>.025||Eigen::AngleAxisd(difference.linear()).angle()>.05)throw std::runtime_error("PAYLOAD_PHYSICAL_PLANNING_POSE_MISMATCH");
    command=base_parent.inverse()*base_object;
    if((payload_frames_->parent_from_tcp().inverse()*command).translation().norm()>.14)throw std::runtime_error("GRASP_TCP_OBJECT_DISTANCE");
   }else {
    const auto &objects=payload_before_.robot_state.attached_collision_objects;
    auto object=std::find_if(objects.begin(),objects.end(),[this](const auto &o){return o.object.id==payload_object_;});
    if(object==objects.end()||object->link_name!=payload_tcp_||object->object.header.frame_id!=payload_tcp_)throw std::runtime_error("PAYLOAD_CURRENT_ATTACHMENT_REQUIRED");
    if(!tf_->canTransform(payload_base_,input().target.header.frame_id,stamp))return;
    Eigen::Isometry3d target,offset;tf2::fromMsg(input().target.pose,target);tf2::fromMsg(object->object.pose,offset);
    const auto base_target=tf2::transformToEigen(tf_->lookupTransform(payload_base_,input().target.header.frame_id,stamp))*target*offset;
    if((base_target.translation()-base_object.translation()).norm()>placement_tolerance_)throw std::runtime_error("PAYLOAD_PLACEMENT_TARGET_NOT_REACHED");
   }
   ignition::msgs::Pose request;if(attach)request.set_name(payload_robot_+"::"+payload_frames_->parent_link());
   const auto pose=tf2::toMsg(command);auto p=request.mutable_position();p->set_x(pose.position.x);p->set_y(pose.position.y);p->set_z(pose.position.z);
   auto q=request.mutable_orientation();q->set_x(pose.orientation.x);q->set_y(pose.orientation.y);q->set_z(pose.orientation.z);q->set_w(pose.orientation.w);
   if(!authority_->record("physical_submission",{{"transaction",payload_transaction_},{"context",input().context_id},{"stage",stage_id_},{"source_epoch",payload_epoch_},{"clock_epoch",payload_clock_},{"execution",diagnostic.at("execution")},{"capture",at}}))throw std::runtime_error(authority_->reason());
   const auto send_ros=now().nanoseconds(),send_steady=wall();
   if(!authority_->grant(send_ros,send_steady)||!payload_stopped(send_ros,send_steady)||!payload_client_->observation(send_ros,send_steady)||!payload_world_base(at,send_ros,send_steady))throw std::runtime_error("PAYLOAD_SEND_EVIDENCE_EXPIRED");
   placement_requested_world_=world_object.translation();
   // Only this validated, task-owned new physical command may replace the
   // previous operation's observation binding. A latched raw failure is never cleared.
   check_payload_raw();const bool previous_bound=payload_observation_bound_;payload_observation_bound_=false;
   payload_phase_=PayloadPhase::PHYSICAL;payload_sent_=send_steady;
   try {payload_client_->begin(attach,request,payload_radius_,diagnostic,sample.second->receipt,send_ros,send_steady);}
   catch(...) {if(!payload_client_->unresolved()){payload_phase_=PayloadPhase::NONE;payload_observation_bound_=previous_bound;}throw;}
   reason_="WAITING_FOR_PHYSICAL_APPLICATION";return;
  }
  if(payload_phase_==PayloadPhase::PHYSICAL) {
   if(!payload_client_->applied(diagnostic,sample.second->receipt,ros,steady))return;
   payload_physical_applied_=true;
   auto physical=payload_client_->observation(ros,steady);if(!physical)return;
   auto world_base=payload_world_base(physical->at("stamp_ns").get<int64_t>(),ros,steady);if(!world_base)return;
   payload_base_object_=tf2::toMsg(world_base->inverse()*physical_object(*physical));
   payload_observation_=raw;payload_observation_bound_=true;check_payload_raw();
   auto request=std::make_shared<SceneApply::Request>();request->scene=payload_scene_diff(payload_before_,raw,payload_object_,attach,payload_base_object_,payload_base_,known_links_);
   if(!authority_->record("physical_applied_scene_submission",{{"transaction",payload_transaction_},{"command_id",payload_client_->id()},{"source_revision",raw.revision},{"source_sequence",raw.sequence}}))throw std::runtime_error(authority_->reason());
   payload_scene_unresolved_=true;payload_phase_=PayloadPhase::APPLY;payload_sent_=wall();
   const auto lease=authority_->lease_id();const auto generation=stage_generation_;
   auto pending=apply_client_->async_send_request(request,[this,lease,generation](rclcpp::Client<SceneApply>::SharedFuture future){
    if(!owned()||lease!=authority_->lease_id()||generation!=stage_generation_)return;
    try {if(!future.get()->success)throw std::runtime_error("PAYLOAD_SCENE_APPLY_REJECTED");payload_query_scene(false);}
    catch(const std::exception &error){stopping(error.what());}
   });payload_request_=pending.request_id;reason_="WAITING_FOR_PAYLOAD_SCENE_READBACK";return;
  }
  if(payload_query_pending_){if(steady-payload_sent_>3000*MS)throw std::runtime_error("PAYLOAD_SCENE_READBACK_TIMEOUT");return;}
  if(payload_phase_==PayloadPhase::APPLY){if(steady-payload_sent_>3000*MS)throw std::runtime_error("PAYLOAD_SCENE_APPLY_UNCONFIRMED");return;}
  if(payload_phase_==PayloadPhase::REVALIDATE&&steady-payload_sent_>10000*MS)throw std::runtime_error("PAYLOAD_REVALIDATION_TIMEOUT");
  const auto *state=payload_consumer_->current(ros,steady);
  if(!state||!geometry_fresh(ros,steady))return;
  if(!payload_revision_ready(*state,payload_observation_,payload_ledger_epoch_))return;
  if(state->attachment_revision!=geometry_->attachment_revision)return;
  if(geometry_->source_id!=reference_geometry_.source_id||geometry_->clock_epoch!=reference_geometry_.clock_epoch||geometry_->model_revision!=reference_geometry_.model_revision)throw std::runtime_error("PAYLOAD_GEOMETRY_SOURCE_CHANGED");
  if(!astribot::payload::scene_matches(payload_observation_.objects,state->observation.objects,payload_config_.allowed_links))throw std::runtime_error("PAYLOAD_RECONCILED_OBJECT_CHANGED");
  std::set<std::string> physical_ids;for(const auto &o:state->observation.objects)physical_ids.insert(o.object.id);
  if(physical_ids!=std::set<std::string>(geometry_->attachment_ids.begin(),geometry_->attachment_ids.end()))throw std::runtime_error("PAYLOAD_GEOMETRY_INVENTORY_MISMATCH");
  if(!attach) {
   auto physical=payload_client_->observation(ros,steady);if(!physical)return;
   if(physical->at("attached").get<bool>()||physical->at("command_id").get<uint32_t>()!=payload_client_->id())throw std::runtime_error("PAYLOAD_DETACH_STATE_CHANGED");
   const auto at=physical->at("stamp_ns").get<int64_t>();const auto position=physical_object(*physical).translation();
   if((position-placement_requested_world_).norm()>placement_tolerance_)throw std::runtime_error("PAYLOAD_PLACEMENT_POSE_CHANGED");
   if(!placement_first_ros_){placement_first_ros_=at;placement_first_wall_=steady;placement_first_=position;return;}
   if(at-placement_first_ros_<250*MS||steady-placement_first_wall_<250*MS)return;
   if((position-placement_first_).norm()>.005)throw std::runtime_error("PAYLOAD_PLACEMENT_NOT_STABLE");
  }
  if(payload_phase_==PayloadPhase::LEDGER) {
   if(steady-payload_checked_at_>250*MS){payload_query_scene(false);return;}
   auto request=std::make_shared<PayloadRevalidate::Request>();request->context_id=input().context_id;request->transaction_id=payload_transaction_;request->start_index=plan_->index()+1;request->scene=payload_readback_;
   payload_revalidated_binding_=bind_scene(payload_readback_,known_links_);payload_phase_=PayloadPhase::REVALIDATE;payload_sent_=wall();
   const auto lease=authority_->lease_id();const auto generation=stage_generation_;
   auto pending=payload_revalidate_->async_send_request(request,[this,lease,generation](rclcpp::Client<PayloadRevalidate>::SharedFuture future){
    if(!owned()||lease!=authority_->lease_id()||generation!=stage_generation_)return;
    try {const auto response=future.get();if(!response->success||response->context_id!=input().context_id||response->transaction_id!=payload_transaction_)throw std::runtime_error("PAYLOAD_REVALIDATION_REJECTED:"+response->reason);payload_query_scene(true);}
    catch(const std::exception &error){stopping(error.what());}
   });payload_request_=pending.request_id;reason_="REVALIDATING_ACTUAL_PAYLOAD_REMAINING_PATH";return;
  }
  if(payload_phase_==PayloadPhase::REVALIDATE)return;
  if(payload_phase_==PayloadPhase::READBACK) {
   if(steady-payload_checked_at_>250*MS){payload_query_scene(true);return;}
   if(!authority_->record("payload_transaction_confirmed",{{"transaction",payload_transaction_},{"stage",stage_id_},{"command_id",payload_client_->id()},{"attachment_revision",state->attachment_revision},{"geometry_sequence",geometry_->sequence}}))throw std::runtime_error(authority_->reason());
   scene_binding_=bind_scene(payload_readback_,known_links_);reference_geometry_.attachment_revision=geometry_->attachment_revision;
   payload_scene_unresolved_=false;payload_phase_=PayloadPhase::NONE;
   if(!stop_at_){plan_->acknowledge();next_stage();}
  }
 }
 std::string guard_context() const {return full_operation_?input().context_id+":stage:"+std::to_string(plan_->index()):input().context_id;}
 std::string stage_name() const {const auto &id=plan_->current().stage_id;return id.substr(0,id.find(':'));}
 void next_stage() {
  ++stage_generation_;hold_.cancel();hold_started_=false;hold_confirmed_=false;
  hold_id_=authority_->lease_id()+"_hold_"+(transfer_parent_?std::to_string(operation_generation_)+"_":"")+std::to_string(plan_->index());
  children_=std::make_unique<ChildActions>(names_);handles_.clear();cancel_sent_.clear();
  motion_sent_=false;scene_verified_=false;scene_final_verified_=false;endpoint_.clear();
  guard_ack_=false;guard_requested_=false;guard_pending_=false;guard_disarm_requested_=false;guard_disarmed_=false;
  scene_stabilizing_=false;scene_retry_at_=0;revalidation_pending_=false;revalidation_readback_=false;
  reason_="CHECKING_NEXT_STAGE:"+plan_->current().stage_id;query_scene(true);
 }

 bool manipulation_ready(int64_t ros,int64_t steady) {
  if(!odom_||ns(odom_->header.stamp)<=0||ros<ns(odom_->header.stamp)||ros-ns(odom_->header.stamp)>300*MS||steady-odom_received_>300*MS){reason_="MTC_BASE_STATE_STALE";return false;}
  const auto &v=odom_->twist.twist;const auto &p=odom_->pose.pose.position;const auto &q=odom_->pose.pose.orientation;
  if(!std::isfinite(v.linear.x+v.linear.y+v.angular.z+p.x+p.y+p.z+q.x+q.y+q.z+q.w)||std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.001||odom_->header.frame_id.empty()||std::hypot(v.linear.x,v.linear.y)>=.02||std::abs(v.angular.z)>=.03){reason_="ARM_REQUIRES_STOPPED_BASE";return false;}
  // A revoked fixed envelope is a latched denial, not a positive lease. The
  // coordinator has no V2 output at cold start, and retains FIXED_POSTURE with
  // valid_until==now after revocation. Only this task's service ACK establishes
  // the barrier; absence of a message alone never authorizes an arm command.
  if(owned()&&revocation_verified_&&envelope_) {
   if(envelope_->navigation_allowed||(envelope_->mode!=Envelope::HOLD&&envelope_->mode!=Envelope::FIXED_POSTURE)){reason_="MTC_NAVIGATION_NOT_REVOKED";return false;}
   if(envelope_->epoch!=revoke_epoch_||(!reference_envelope_.coordinator_session_id.empty()&&envelope_->coordinator_session_id!=reference_envelope_.coordinator_session_id)){reason_="MTC_COORDINATOR_CHANGED";return false;}
   if(reference_envelope_.coordinator_session_id.empty())reference_envelope_=*envelope_;
  }
  return true;
 }
 void revoke_navigation() {
  auto request=std::make_shared<Revoke::Request>();request->envelope.transport_ready=false;
  const auto lease=authority_->lease_id();const auto generation=++revoke_generation_;
  revoke_pending_=true;revoke_sent_=wall();revoke_ros_=now().nanoseconds();revoke_envelope_sequence_=envelope_sequence_;
  try{auto pending=revoke_client_->async_send_request(request,[this,lease,generation](rclcpp::Client<Revoke>::SharedFuture f){
   if(!owned()||lease!=authority_->lease_id()||generation!=revoke_generation_)return;
   revoke_pending_=false;
   try {auto result=f.get();if(!result->accepted){stopping("NAVIGATION_REVOCATION_REJECTED:"+result->reason);return;}
    revoke_ack_=true;revoke_epoch_=result->epoch;revoke_ack_at_=wall();revoke_wait_negative_=bool(envelope_);
    if(!authority_->record("navigation_revoked",{{"epoch",result->epoch}}))throw std::runtime_error(authority_->reason());
    if(!stop_at_)reason_="WAITING_FOR_NAVIGATION_REVOCATION_READBACK";
   }catch(const std::exception&e){stopping(std::string("NAVIGATION_REVOCATION_ERROR:")+e.what());}
  });revoke_request_=pending.request_id;}catch(const std::exception&e){revoke_pending_=false;stopping(std::string("NAVIGATION_REVOCATION_UNCONFIRMED:")+e.what());}
 }
 void query_scene(bool recheck) {
  auto request=std::make_shared<SceneQuery::Request>();request->components.components=1023;
  scene_pending_=true;scene_sent_=wall();const auto lease=authority_->lease_id();const auto stage_generation=stage_generation_;
  auto pending=scenes_->async_send_request(request,[this,lease,recheck,stage_generation](rclcpp::Client<SceneQuery>::SharedFuture f){
   if(!owned()||lease!=authority_->lease_id()||stage_generation!=stage_generation_)return;
   scene_pending_=false;
   if(stop_at_)return;
   try {
    auto scene=f.get()->scene;
    if(!geometry_fresh(now().nanoseconds(),wall())||!manipulation_ready(now().nanoseconds(),wall()))throw std::runtime_error(reason_);
    auto signature=bind_scene(scene,known_links_);
    if(recheck){
     if(revalidation_readback_){
      if(signature!=revalidation_binding_)throw std::runtime_error("MTC_SCENE_CHANGED_DURING_REVALIDATION");
      scene_binding_=std::move(signature);revalidation_readback_=false;scene_stabilizing_=false;
      if(!authority_->record("remaining_plan_revalidated",{{"context",input().context_id},{"start_index",plan_->index()}}))throw std::runtime_error(authority_->reason());
     }else if(signature!=scene_binding_){
      // The existing MTC service only replaces occupancy in its cached stage
      // scenes. It does NOT authorize changed world/ACM/attached-body geometry.
      if(signature.first!=scene_binding_.first)throw std::runtime_error("MTC_SCENE_CHANGED");
      if(!scene_stabilizing_){scene_stabilizing_=true;scene_stability_started_=scene_stable_since_=wall();revalidation_binding_=signature;}
      else if(signature!=revalidation_binding_){revalidation_binding_=signature;scene_stable_since_=wall();}
      if(wall()-scene_stable_since_<1000*MS){scene_retry_at_=wall()+100*MS;return;}
      revalidate_scene(scene);return;
     }else {scene_stabilizing_=false;scene_retry_at_=0;}
     std::set<std::string> attached;for(const auto&o:scene.robot_state.attached_collision_objects)attached.insert(o.object.id);
     const auto&stage=plan_->check(stage_name(),plan_->current().kind,wall(),geometry_->joints,attached);
     stage_id_=stage.stage_id;
     if(stage.kind=="ATTACH"||stage.kind=="DETACH"){begin_payload(scene);return;}
     trajectory_=stage.trajectory.joint_trajectory;stage_kind_=stage.kind;
     active_controller_=stage.kind=="ARM"?"arm_left_controller":"gripper_left_controller";
     if(!stage.trajectory.multi_dof_joint_trajectory.points.empty()||trajectory_.points.empty()||trajectory_.joint_names!=groups_.at(active_controller_))throw std::runtime_error("MTC_UNSUPPORTED_ARM_TRAJECTORY");
     int64_t previous=-1;
     for(const auto&point:trajectory_.points){
      const auto duration=int64_t(point.time_from_start.sec)*1000000000+point.time_from_start.nanosec;
      if(duration<0||duration<=previous||duration>120000000000LL||point.positions.size()!=trajectory_.joint_names.size()||(!point.velocities.empty()&&point.velocities.size()!=point.positions.size())||(!point.accelerations.empty()&&point.accelerations.size()!=point.positions.size())||!point.effort.empty())throw std::runtime_error("MTC_TRAJECTORY_FIELDS_INVALID");
      for(const auto*values:{&point.positions,&point.velocities,&point.accelerations})for(double value:*values)if(!std::isfinite(value))throw std::runtime_error("MTC_TRAJECTORY_NONFINITE");
      previous=duration;
     }
     for(size_t i=0;i<trajectory_.joint_names.size();++i)endpoint_[trajectory_.joint_names[i]]=trajectory_.points.back().positions[i];
     scene_verified_=true;scene_verified_at_=wall();
     if(stage_kind_=="GRIPPER")guard_ack_=true;
     if(!guard_ack_)arm_guard();
     else {scene_final_verified_=true;advance_plan(now().nanoseconds(),wall());}
    }else {
     scene_binding_=std::move(signature);scene.robot_state.joint_state=geometry_->joints;reference_geometry_=*geometry_;
     const auto&request_goal=input();Planner::Goal goal;
     goal.operation=request_goal.operation;goal.object_id=request_goal.object_id;goal.context_id=request_goal.context_id;goal.scene=scene;
     goal.pre_target=request_goal.pre_target;goal.target=request_goal.target;goal.exit_targets=request_goal.exit_targets;goal.touch_links=request_goal.touch_links;goal.grasp_width_m=request_goal.grasp_width_m;goal.timeout_s=45.;
     if(!transfer_parent_||authority_->phase()==ResourcePhase::RESERVED)authority_->submitted(now().nanoseconds(),wall());
     else if(authority_->phase()!=ResourcePhase::EXECUTING)throw std::runtime_error("PLANNER_OPERATION_PHASE_INVALID");
     children_->submitted("mtc");
     if(!authority_->record("planner_submission",{{"context",goal.context_id},{"geometry_sequence",geometry_->sequence}}))throw std::runtime_error(authority_->reason());
     rclcpp_action::Client<Planner>::SendGoalOptions options;
     const auto operation=operation_generation_;const auto context=goal.context_id;
     options.goal_response_callback=[this,lease,operation,context](PlannerHandle::SharedPtr h){if(!owned()||lease!=authority_->lease_id()||operation!=operation_generation_||context!=input().context_id)return;try{children_->response("mtc",h?canonical_goal_id(h->get_goal_id()):"");planner_handle_=h;if(!h)stopping("MTC_PLAN_REJECTED");}catch(const std::exception&e){stopping(e.what());}};
     options.result_callback=[this,lease,operation,context](const PlannerHandle::WrappedResult &result){
      if(!owned()||lease!=authority_->lease_id()||operation!=operation_generation_||context!=input().context_id)return;
      try {
       const bool terminal=result.result&&(result.code==rclcpp_action::ResultCode::SUCCEEDED||result.code==rclcpp_action::ResultCode::ABORTED||result.code==rclcpp_action::ResultCode::CANCELED);
       const bool okay=result.code==rclcpp_action::ResultCode::SUCCEEDED&&result.result&&result.result->success;
       children_->result("mtc",canonical_goal_id(result.goal_id),okay,terminal);
       if(!authority_->record("planner_terminal",{{"context",input().context_id},{"success",okay}}))throw std::runtime_error(authority_->reason());
       if(stop_at_)return;
       if(!okay)throw std::runtime_error("MTC_PLAN_FAILED:"+(result.result?result.result->reason:std::string("UNKNOWN_RESULT")));
       consume_plan(*result.result);
      }catch(const std::exception&e){stopping(e.what());}
     };
     planner_->async_send_goal(goal,options);reason_="PLANNING_COMPLETE_OPERATION";
    }
   }catch(const std::exception&e){stopping(e.what());}
  });scene_request_=pending.request_id;
 }
 void revalidate_scene(const moveit_msgs::msg::PlanningScene &scene) {
  if(!revalidate_client_->service_is_ready())throw std::runtime_error("MTC_REVALIDATION_UNAVAILABLE");
  auto request=std::make_shared<Revalidate::Request>();request->context_id=input().context_id;request->start_index=plan_->index();request->scene=scene;
  const auto lease=authority_->lease_id();const auto stage_generation=stage_generation_;revalidation_pending_=true;revalidation_sent_=wall();scene_retry_at_=0;
  auto pending=revalidate_client_->async_send_request(request,[this,lease,stage_generation](rclcpp::Client<Revalidate>::SharedFuture f){
   if(!owned()||lease!=authority_->lease_id()||stage_generation!=stage_generation_)return;
   revalidation_pending_=false;if(stop_at_)return;
   try{
    auto result=f.get();
    if(!result->success||result->context_id!=input().context_id)throw std::runtime_error("MTC_REVALIDATION_REJECTED:"+result->reason);
    revalidation_readback_=true;query_scene(true);
   }catch(const std::exception&e){stopping(e.what());}
  });revalidation_request_=pending.request_id;
  reason_="REVALIDATING_SAME_REMAINING_PLAN";
 }
 // The owning caller may later supply M3's already-validated complete result at
 // this internal boundary. There is no ROS endpoint accepting arbitrary paths.
 void consume_plan(const Planner::Result &result) {
  const auto &goal=input();
  plan_=std::make_unique<MtcPlan>(goal.operation,result.stages,goal.context_id,result.context_id,wall(),all_joints_);
  query_scene(true);
 }
 void arm_guard() {
  auto request=std::make_shared<GuardService::Request>();request->enable=true;request->context_id=guard_context();
  request->joint_names=trajectory_.joint_names;request->base_reference.header=reference_base_.header;request->base_reference.pose=reference_base_.pose.pose;
  guard_pending_=true;guard_requested_=true;const auto lease=authority_->lease_id();const auto stage_generation=stage_generation_;
  guard_client_->async_send_request(request,[this,lease,stage_generation](rclcpp::Client<GuardService>::SharedFuture f){
   if(!owned()||lease!=authority_->lease_id()||stage_generation!=stage_generation_)return;
   guard_pending_=false;
   try{auto r=f.get();if(!r->accepted){guard_requested_=false;stopping("EXECUTION_GUARD_REJECTED:"+r->reason);}else guard_ack_=true;}catch(const std::exception&e){stopping(e.what());}
  });reason_="WAITING_FOR_EXECUTION_GUARD";
 }
 void advance_plan(int64_t ros,int64_t steady) {
  check_payload_raw();
  if(hold_confirmed_)return; // M2 may now grant navigation with this same hold.
  if(!manipulation_ready(ros,steady)){stopping(reason_);return;}
  if(geometry_->source_id!=reference_geometry_.source_id||geometry_->clock_epoch!=reference_geometry_.clock_epoch||geometry_->model_revision!=reference_geometry_.model_revision||(payload_phase_==PayloadPhase::NONE&&geometry_->attachment_revision!=reference_geometry_.attachment_revision)){stopping("MTC_CONTEXT_CHANGED");return;}
  const auto&a=odom_->pose.pose;const auto&b=reference_base_.pose.pose;
  const double dot=std::abs(a.orientation.x*b.orientation.x+a.orientation.y*b.orientation.y+a.orientation.z*b.orientation.z+a.orientation.w*b.orientation.w);
  if(odom_->header.frame_id!=reference_base_.header.frame_id||std::hypot(std::hypot(a.position.x-b.position.x,a.position.y-b.position.y),a.position.z-b.position.z)>.02||2*std::acos(std::min(1.,dot))>.02){stopping("MTC_BASE_MOVED");return;}
  if(payload_phase_!=PayloadPhase::NONE)return;
  if(!revocation_verified_) {
   if(!navigation_revoked(ros,steady))return;
   if(!transfer_parent_)query_scene(false);
  }
  if(scene_pending_&&steady-scene_sent_>3000*MS){scenes_->remove_pending_request(scene_request_);scene_pending_=false;stopping("MTC_SCENE_TIMEOUT");return;}
  if(revalidation_pending_&&steady-revalidation_sent_>10000*MS){revalidate_client_->remove_pending_request(revalidation_request_);revalidation_pending_=false;stopping("MTC_REVALIDATION_TIMEOUT");return;}
  if(scene_stabilizing_&&!revalidation_pending_&&!revalidation_readback_&&steady-scene_stability_started_>15000*MS){stopping("MTC_SCENE_NOT_STABLE");return;}
  if(scene_retry_at_&&!scene_pending_&&!revalidation_pending_&&steady>=scene_retry_at_){scene_retry_at_=0;query_scene(true);}
  if(scene_stabilizing_||revalidation_pending_||revalidation_readback_)return;
  if(scene_verified_&&!motion_sent_){
   if(steady-scene_verified_at_>2000*MS){stopping("EXECUTION_GUARD_TIMEOUT");return;}
   if(!guard_ack_||!revoke_ack_)return;
   if(stage_kind_=="ARM") {
    if(!guard_state_||guard_state_->context_id!=guard_context()||!guard_state_->active)return;
    if(!guard_state_->healthy){if(guard_state_->reason!="WAITING_FOR_EXECUTION_EVIDENCE")stopping(guard_state_->reason);return;}
    if(steady-guard_received_>300*MS||ros<ns(guard_state_->stamp)||ros-ns(guard_state_->stamp)>300*MS)return;
   }
   if(!scene_final_verified_){if(!scene_pending_)query_scene(true);return;}
   std::set<std::string> attached(geometry_->attachment_ids.begin(),geometry_->attachment_ids.end());
   plan_->check(stage_name(),plan_->current().kind,steady,geometry_->joints,attached);
   motion_sent_=true;send_children();reason_=full_operation_?"EXECUTING_MTC_STAGE:"+stage_id_:"EXECUTING_FIRST_MTC_STAGE";
  }
  if(motion_sent_&&stage_kind_=="ARM"&&!guard_disarm_requested_){
   if(!guard_state_||guard_state_->context_id!=guard_context()||!guard_state_->active||!guard_state_->healthy||steady-guard_received_>300*MS||ros<ns(guard_state_->stamp)||ros-ns(guard_state_->stamp)>300*MS){stopping("EXECUTION_GUARD_UNHEALTHY");return;}
  }
 }
 bool finish_guard() {
  if(!guard_requested_&&!guard_pending_)return true;
  if(guard_disarmed_)return true;
  if(guard_pending_||guard_disarm_requested_)return false;
  guard_disarm_requested_=true;auto request=std::make_shared<GuardService::Request>();request->context_id=guard_context();request->enable=false;
  const auto lease=authority_->lease_id();const auto stage_generation=stage_generation_;
  guard_client_->async_send_request(request,[this,lease,stage_generation](rclcpp::Client<GuardService>::SharedFuture f){
   if(!owned()||lease!=authority_->lease_id()||stage_generation!=stage_generation_)return;
   try{auto response=f.get();guard_disarmed_=response->accepted;if(!response->accepted)stopping("EXECUTION_GUARD_DISARM_REJECTED:"+response->reason);}
   catch(const std::exception&e){stopping(std::string("EXECUTION_GUARD_DISARM_UNCONFIRMED:")+e.what());}
  });return false;
 }
#endif
 bool geometry_fresh(int64_t ros,int64_t steady) {
  if(!geometry_||!geometry_->complete||!geometry_->attachment_state_confirmed){reason_="GEOMETRY_UNCONFIRMED";return false;}
  const auto &g=*geometry_;const auto at=ns(g.header.stamp),until=ns(g.valid_until);
  if(at<=0||at>ros||ros>=until||steady>=geometry_deadline_||until-at>500*MS||g.source_id.empty()||g.model_revision.empty()||g.attachment_revision.empty()||g.sequence==0||
    g.joints.name.size()!=all_joints_.size()||g.joints.position.size()!=all_joints_.size()||g.joint_position_error_bounds.size()!=all_joints_.size()||g.joint_source_stamps.size()!=all_joints_.size()||
    std::set<std::string>(g.joints.name.begin(),g.joints.name.end())!=all_joints_){reason_="GEOMETRY_INVALID_OR_EXPIRED";return false;}
  for(size_t i=0;i<g.joints.name.size();++i)if(!std::isfinite(g.joints.position[i])||!std::isfinite(g.joint_position_error_bounds[i])||g.joint_position_error_bounds[i]<=0||g.joint_position_error_bounds[i]>.025||ns(g.joint_source_stamps[i])>ros||ns(g.joint_source_stamps[i])<at){reason_="GEOMETRY_JOINT_INVALID";return false;}
  return true;
 }
 void receive(const Geometry &g) {
  auto ros=now().nanoseconds(),steady=wall();
  if(g.complete&&g.attachment_state_confirmed&&ns(g.header.stamp)>ros)return;
  if(geometry_&&g.source_id==geometry_->source_id&&g.clock_epoch==geometry_->clock_epoch&&g.sequence<=geometry_->sequence){
   if(g.sequence==geometry_->sequence&&g!=*geometry_)stopping("GEOMETRY_REPLAY_CONFLICT");
   return;
  }
  geometry_=g;geometry_deadline_=steady+std::max<int64_t>(0,ns(g.valid_until)-ros);
  if(hold_started_&&(!g.complete||!g.attachment_state_confirmed||ns(g.header.stamp)>completed_at_))hold_.geometry(g,ros,steady);
  if(stop_at_&&geometry_fresh(ros,steady)&&ns(g.header.stamp)>stop_ros_) {
   if(!settling_.empty()&&(ns(g.header.stamp)-ns(settling_.back().first.header.stamp)>300*MS||steady-settling_.back().second>300*MS))settling_.clear();
   settling_.emplace_back(g,steady);while(settling_.size()>2&&ns(g.header.stamp)-ns(settling_[1].first.header.stamp)>=500*MS)settling_.pop_front();
   if(settling_.size()>128)settling_.clear();
  }
 }
 bool claims_fresh(int64_t ros,int64_t steady) {
  if(claims_ros_<0||ros<claims_ros_||ros-claims_ros_>=500*MS||steady<claims_wall_||steady-claims_wall_>=500*MS){reason_="CONTROLLER_CLAIMS_EXPIRED";return false;}
  std::set<std::string> actual,seen;
  for(const auto &controller:claims_)if(controller.state=="active") {
   if(groups_.count(controller.name)) {
    seen.insert(controller.name);
    if(controller.type!="joint_trajectory_controller/JointTrajectoryController"){reason_="CONTROLLER_TYPE_CHANGED";return false;}
    std::set<std::string> expected;for(const auto &n:groups_.at(controller.name))expected.insert(n+"/position");
    if(std::set<std::string>(controller.claimed_interfaces.begin(),controller.claimed_interfaces.end())!=expected){reason_="CONTROLLER_RESOURCE_CHANGED";return false;}
   }
   for(const auto &interface:controller.claimed_interfaces)if(!actual.insert(interface).second){reason_="CONTROLLER_RESOURCE_CONFLICT";return false;}
  }
  if(seen.size()!=groups_.size()){reason_="CONTROLLER_NOT_ACTIVE";return false;}
  for(const auto &n:all_joints_)if(!actual.count(n+"/position")){reason_="CONTROLLER_CLAIM_MISSING";return false;}
  return true;
 }
 bool exclusive_graph() {
  const auto self=std::string(get_name()),space=std::string(get_namespace());
  const auto nodes=get_node_graph_interface()->get_node_names_and_namespaces();
  if(nodes.size()>256||std::count(nodes.begin(),nodes.end(),std::make_pair(self,space))!=1){reason_="TASK_OWNER_NODE_NOT_UNIQUE";return false;}
  for(const auto &[name,ns]:nodes)if(name!=self||ns!=space) {
   for(const auto &[service,types]:get_node_graph_interface()->get_client_names_and_types_by_node(name,ns))
    for(const auto &controller:names_)if(service=="/"+controller+"/follow_joint_trajectory/_action/send_goal") {reason_="FOREIGN_CONTROLLER_SERVICE_CLIENT:"+name;return false;}
  }
  for(const auto &[name,client]:clients_) {
   const auto action="/"+name+"/follow_joint_trajectory";
   for(const auto &endpoint:get_subscriptions_info_by_topic(action+"/_action/status"))if(endpoint.node_name()!=self||endpoint.node_namespace()!=space){reason_="FOREIGN_CONTROLLER_CLIENT:"+endpoint.node_name();return false;}
   if(get_publishers_info_by_topic("/"+name+"/joint_trajectory").size()){reason_="DIRECT_TRAJECTORY_WRITER:"+name;return false;}
   if(get_publishers_info_by_topic(action+"/_action/status").size()!=1){reason_="CONTROLLER_SERVER_NOT_UNIQUE:"+name;return false;}
  }
  for(const auto &endpoint:get_publishers_info_by_topic("/navigation/arm_hold"))if(endpoint.node_name()!=self||endpoint.node_namespace()!=space){reason_="FOREIGN_HOLD_AUTHORITY";return false;}
  return true;
 }
 void start() {
  auto child_names=names_;
#ifdef PLAN_TO_HOLD_EXECUTOR
  payload_observation_bound_=false;payload_raw_failure_.clear();
  payload_phase_=PayloadPhase::NONE;payload_physical_applied_=false;payload_scene_unresolved_=false;payload_query_pending_=false;
  if(!full_operation_)known_links_.clear();
  ++stage_generation_;stage_kind_="ARM";active_controller_="arm_left_controller";
  child_names.push_back("mtc");planner_handle_.reset();planner_cancel_sent_=false;plan_.reset();scene_pending_=false;guard_pending_=false;guard_ack_=false;guard_requested_=false;guard_disarm_requested_=false;guard_disarmed_=false;motion_sent_=false;scene_verified_=false;scene_final_verified_=false;stage_id_.clear();endpoint_.clear();
  reference_geometry_=*geometry_;reference_base_=*odom_;reference_envelope_=envelope_.value_or(Envelope());
  scene_stabilizing_=false;scene_retry_at_=0;revalidation_pending_=false;revalidation_readback_=false;
#endif
  children_=std::make_unique<ChildActions>(child_names);handles_.clear();cancel_sent_.clear();hold_started_=false;hold_confirmed_=false;terminal_barrier_=false;stop_at_=0;stop_reason_.clear();settling_.clear();
  motion_ever_sent_=false;started_=wall();hold_id_=authority_->lease_id()+"_hold";
#ifdef PLAN_TO_HOLD_EXECUTOR
  revoke_ack_=false;revoke_pending_=false;revocation_verified_=false;revoke_navigation();
#else
  send_children();
#endif
 }
 void send_children() {
  try {
   if(!geometry_fresh(now().nanoseconds(),wall())||!claims_fresh(now().nanoseconds(),wall())||!exclusive_graph()){stopping(reason_);return;}
   std::map<std::string,double> positions;for(size_t i=0;i<geometry_->joints.name.size();++i)positions[geometry_->joints.name[i]]=geometry_->joints.position[i];
#ifndef PLAN_TO_HOLD_EXECUTOR
   authority_->submitted(now().nanoseconds(),wall());
#endif
   for(const auto &[name,joints]:groups_) {
    children_->submitted(name);if(!authority_->record("child_submission",{{"controller",name}}))throw std::runtime_error(authority_->reason());
    Trajectory::Goal goal;goal.trajectory.joint_names=joints;
    trajectory_msgs::msg::JointTrajectoryPoint point;for(const auto &joint:joints)point.positions.push_back(positions.at(joint));
    point.velocities.assign(joints.size(),0.);point.time_from_start.sec=1;goal.trajectory.points.push_back(point);goal.goal_time_tolerance.sec=1;
#ifdef PLAN_TO_HOLD_EXECUTOR
    if(name==active_controller_)goal.trajectory=trajectory_;
    else goal.trajectory.points.front().time_from_start=trajectory_.points.back().time_from_start;
#endif
    auto options=rclcpp_action::Client<Trajectory>::SendGoalOptions();
    const auto lease=authority_->lease_id();
#ifdef PLAN_TO_HOLD_EXECUTOR
    const auto stage_generation=stage_generation_;
#else
    const uint64_t stage_generation=0;
#endif
    options.goal_response_callback=[this,name,lease,stage_generation](Child::SharedPtr child){
     if(!owned()||lease!=authority_->lease_id())return;
#ifdef PLAN_TO_HOLD_EXECUTOR
     if(stage_generation!=stage_generation_)return;
#else
     (void)stage_generation;
#endif
     try{auto id=child?canonical_goal_id(child->get_goal_id()):"";children_->response(name,id);handles_[name]=child;
      if(!authority_->record("child_response",{{"controller",name},{"uuid",id}}))stopping(authority_->reason());
      if(!child)stopping("CHILD_GOAL_REJECTED:"+name);
      // Cancellation is dispatched by tick(), outside action-client callbacks.
     }catch(const std::exception &e){stopping(e.what());}
    };
    options.result_callback=[this,name,lease,stage_generation](const Child::WrappedResult &result){
     if(!owned()||lease!=authority_->lease_id())return;
#ifdef PLAN_TO_HOLD_EXECUTOR
     if(stage_generation!=stage_generation_)return;
#else
     (void)stage_generation;
#endif
     try{const auto uuid=canonical_goal_id(result.goal_id);const bool okay=result.code==rclcpp_action::ResultCode::SUCCEEDED&&result.result&&result.result->error_code==Trajectory::Result::SUCCESSFUL;
      const bool terminal=result.result&&(result.code==rclcpp_action::ResultCode::SUCCEEDED||result.code==rclcpp_action::ResultCode::ABORTED||result.code==rclcpp_action::ResultCode::CANCELED);
      children_->result(name,uuid,okay,terminal);
      if(!authority_->record("child_terminal",{{"controller",name},{"uuid",uuid},{"success",okay},{"result_code",int(result.code)}}))stopping(authority_->reason());
      if(!okay)stopping("CHILD_RESULT_FAILED:"+name);
     }catch(const std::exception &e){stopping(e.what());}
    };
    motion_ever_sent_=true;clients_.at(name)->async_send_goal(goal,options);
   }
  }catch(const std::exception &e){stopping(e.what());}
 }
 void cancel_children() {
#ifdef PLAN_TO_HOLD_EXECUTOR
  if(planner_handle_&&!planner_cancel_sent_&&planner_handle_->get_status()<4){planner_cancel_sent_=true;planner_->async_cancel_goal(planner_handle_);}
#endif
  for(const auto &[name,handle]:handles_)if(handle&&!cancel_sent_.count(name)&&handle->get_status()<4) {
   cancel_sent_.insert(name);try{clients_.at(name)->async_cancel_goal(handle);}catch(const std::exception &e){authority_->record("cancel_unconfirmed",{{"controller",name},{"error",e.what()}});}
  }
 }
 void stopping(const std::string &reason) {
  if(!owned())return;
  if(!stop_at_){stop_at_=wall();stop_ros_=now().nanoseconds();stop_reason_=reason;reason_=reason;authority_->stop(stop_reason_,stop_ros_,stop_at_);hold_.cancel();settling_.clear();
#ifdef PLAN_TO_HOLD_EXECUTOR
   if(transfer_parent_&&navigation_started_){navigation_->cancel(stop_reason_);navigation_cleanup_revoke_=false;}
   revoke_navigation();
#endif
  }
  // Humble invokes result callbacks while holding the goal-handle mutex.
  // Inspecting/canceling any goal here can deadlock. The wall timer dispatches
  // cancellation after this callback returns; revocation above is immediate.
 }
 bool measured_safe(int64_t ros,int64_t steady) {
  if(!geometry_fresh(ros,steady)||!claims_fresh(ros,steady)||settling_.size()<3)return false;
  const auto &last=settling_.back().first;
  if(ns(last.header.stamp)-ns(settling_.front().first.header.stamp)<500*MS||settling_.back().second-settling_.front().second<500*MS)return false;
  for(const auto &[g,w]:settling_) {
   if(g.source_id!=last.source_id||g.model_revision!=last.model_revision||g.attachment_revision!=last.attachment_revision||g.clock_epoch!=last.clock_epoch||g.joints.name!=last.joints.name||g.joint_position_error_bounds!=last.joint_position_error_bounds)return false;
  }
  for(size_t i=0;i<last.joints.name.size();++i) {
   double low=last.joints.position[i],high=low;
   for(const auto &[g,w]:settling_){low=std::min(low,g.joints.position[i]);high=std::max(high,g.joints.position[i]);}
   if(high-low>last.joint_position_error_bounds[i]/4.)return false;
  }
  return true;
 }
 void query_controllers() {
  auto ros=now().nanoseconds(),steady=wall();
  if(query_pending_){if(steady-query_sent_wall_>500*MS){controllers_->remove_pending_request(query_id_);query_pending_=false;}else return;}
  if(!controllers_->service_is_ready())return;
  query_pending_=true;query_sent_wall_=steady;
  auto result=controllers_->async_send_request(std::make_shared<Query::Request>(),[this,ros,steady](rclcpp::Client<Query>::SharedFuture response){
   query_pending_=false;try{claims_=response.get()->controller;claims_ros_=ros;claims_wall_=steady;
    if(hold_started_)hold_.controllers(claims_,ros,steady,now().nanoseconds(),wall());
   }catch(...){claims_ros_=claims_wall_=-1;}
  });query_id_=result.request_id;
 }
 void tick() {
  auto ros=now().nanoseconds(),steady=wall();
  try {
   query_controllers();
   if(owned()) {
    auto grant=authority_->grant(ros,steady);
    bool geometry_ready=true;
#ifdef PLAN_TO_HOLD_EXECUTOR
    if(payload_phase_!=PayloadPhase::NONE&&payload_phase_!=PayloadPhase::FRAME)geometry_ready=payload_stopped(ros,steady);
    else
#endif
    geometry_ready=geometry_fresh(ros,steady);
    if(!stop_at_&&(!grant||!geometry_ready||!claims_fresh(ros,steady)||!exclusive_graph()))stopping(grant?reason_:authority_->reason());
#ifdef PLAN_TO_HOLD_EXECUTOR
    if(!stop_at_&&(!transfer_parent_||transfer_phase_==TransferPhase::PICK||transfer_phase_==TransferPhase::PLACE))advance_plan(ros,steady);
    if(payload_phase_!=PayloadPhase::NONE) {
     try {advance_payload(ros,steady);}catch(const std::exception &error){stopping(error.what());}
    }
#endif
    if(!stop_at_&&children_->all_successful()&&!hold_started_) {
     completed_at_=ros;HoldCompletion completion{grant->owner_id,grant->lease_id,grant->epoch,children_->proof(),ros,true,true};
     hold_.begin(hold_id_,*grant,completion,ros,steady);hold_started_=true;
     hold_.controllers(claims_,claims_ros_,claims_wall_,ros,steady);
    }
    if(!stop_at_&&hold_started_) {
     hold_.resource(*grant,ros,steady);auto state=hold_.status(ros,steady);
#ifdef PLAN_TO_HOLD_EXECUTOR
     if(state.hold_confirmed&&!hold_confirmed_) {
      std::map<std::string,double> positions;for(size_t i=0;i<geometry_->joints.name.size();++i)positions[geometry_->joints.name[i]]=geometry_->joints.position[i];
      for(const auto &[name,target]:endpoint_)if(std::abs(positions.at(name)-target)>.02)stopping("MTC_ENDPOINT_NOT_REACHED");
     }
     const bool completion_ready=!stop_at_&&state.hold_confirmed&&finish_guard();
     if(completion_ready&&!hold_confirmed_) {
      if(!authority_->record("stage_confirmed",{{"context",input().context_id},{"stage_id",stage_id_},{"index",plan_->index()},{"children",children_->proof()}}))throw std::runtime_error(authority_->reason());
      plan_->acknowledge();
      if(full_operation_&&!plan_->complete()) {next_stage();}
      else {authority_->holding(ros,steady);hold_confirmed_=true;reason_=full_operation_?"MANIPULATION_HOLD_CONFIRMED":"FIRST_STAGE_HOLD_CONFIRMED";}
     }
#else
     if(state.hold_confirmed&&!hold_confirmed_){authority_->holding(ros,steady);hold_confirmed_=true;reason_="HOLD_CONFIRMED";}
#endif
     if(hold_confirmed_&&!state.hold_confirmed)stopping(hold_.reason());
     if(!state.hold_confirmed&&hold_.reason().rfind("WAITING_",0)!=0)stopping(hold_.reason());
    }
#ifdef PLAN_TO_HOLD_EXECUTOR
    if(!stop_at_&&!hold_confirmed_&&steady-started_>120000*MS)stopping("PLAN_TO_HOLD_TIMEOUT");
#else
    if(!stop_at_&&!hold_confirmed_&&steady-started_>15000*MS)stopping("HOLD_COMPLETION_TIMEOUT");
#endif
#ifdef PLAN_TO_HOLD_EXECUTOR
    if(!stop_at_&&transfer_parent_)advance_transfer(now().nanoseconds(),wall());
#endif
    if(stop_at_) {
     // stopping() may have sampled newer clocks inside this same tick. Keep
     // real rollback detection intact; cleanup must use its own current pair.
     ros=now().nanoseconds();steady=wall();
     cancel_children();
#ifdef PLAN_TO_HOLD_EXECUTOR
     if(transfer_parent_&&navigation_started_) {
      navigation_->cancel(stop_reason_);navigation_->tick();
      if(navigation_->status().fixed_request_terminal&&!navigation_cleanup_revoke_) {
       navigation_cleanup_revoke_=true;revoke_ack_=false;revocation_verified_=false;revoke_navigation();
      }
     }
#endif
     if(children_->all_terminal()&&!terminal_barrier_){terminal_barrier_=true;stop_ros_=ros;settling_.clear();}
     bool ready=children_->all_terminal()&&(!motion_ever_sent_||measured_safe(ros,steady));
#ifdef PLAN_TO_HOLD_EXECUTOR
     ready=ready&&payload_phase_==PayloadPhase::NONE&&!payload_scene_unresolved_&&(!full_operation_||!payload_client_->unresolved())&&finish_guard();
     if(transfer_parent_)ready=ready&&!station_scene_unresolved_&&(!navigation_started_||
       (navigation_->status().cleanup_complete&&navigation_cleanup_revoke_&&navigation_revoked(now().nanoseconds(),wall())));
#endif
     if(authority_->release(ready,ready,ros,steady)) {
      finish_parent(true);parent_.reset();
#ifdef PLAN_TO_HOLD_EXECUTOR
      full_parent_.reset();transfer_parent_.reset();
#endif
     } else if(steady-stop_at_>10000*MS&&parent_active()) {
      authority_->uncertain_result("RESOURCE_RECOVERY_REQUIRED:"+stop_reason_,ros,steady);finish_parent(false);

      // Keep the task/child context and deny new admission. Late real results
      // remain evidence, but never re-execute a child after an uncertain send.
     }
    }
   }
  }catch(const std::exception &e){stopping(e.what());}
  ros=now().nanoseconds();steady=wall();
  auto state=hold_.status(ros,steady);if(stop_at_)state.hold_confirmed=false;
#ifdef PLAN_TO_HOLD_EXECUTOR
  if(!hold_confirmed_)state.hold_confirmed=false;
#endif
  hold_pub_->publish(state);
  const auto phase=std::to_string(int(authority_->phase()));
  const auto &reported_reason=stop_at_?stop_reason_:reason_;
  if(owned()&&parent_active()) {
   auto publish=[&](auto handle,auto feedback) {
    if(!handle)return;
    feedback->phase=phase;feedback->reason=reported_reason;feedback->lease_id=authority_->lease_id();
    feedback->resource_epoch=authority_->epoch();feedback->hold_id=hold_id_;feedback->hold_confirmed=state.hold_confirmed;
#ifdef PLAN_TO_HOLD_EXECUTOR
    feedback->context_id=input().context_id;feedback->stage_id=stage_id_;
#endif
    handle->publish_feedback(feedback);
   };
   publish(parent_,std::make_shared<Action::Feedback>());
#ifdef PLAN_TO_HOLD_EXECUTOR
   publish(full_parent_,std::make_shared<FullAction::Feedback>());
   if(transfer_parent_) {
    auto feedback=std::make_shared<Transfer::Feedback>();feedback->phase=transfer_phase_name();feedback->reason=reported_reason;
    feedback->lease_id=authority_->lease_id();feedback->resource_epoch=authority_->epoch();feedback->context_id=transfer_parent_->get_goal()->context_id;
    feedback->object_id=input().object_id;feedback->stage_id=stage_id_;feedback->hold_id=hold_id_;feedback->hold_confirmed=state.hold_confirmed;
    if(navigation_)feedback->navigation_goal_uuid=navigation_->status().nav_goal_uuid;
    transfer_parent_->publish_feedback(feedback);
   }
#endif
  }
  nlohmann::json status={{"phase",phase},{"reason",reported_reason},{"cleanup_reason",stop_at_&&reason_!=stop_reason_?reason_:""},{"authority_reason",authority_->reason()},{"hold_reason",hold_.reason()},{"lease_id",authority_->lease_id()},{"epoch",authority_->epoch()},{"hold_id",hold_id_},{"hold_confirmed",state.hold_confirmed}};
#ifdef PLAN_TO_HOLD_EXECUTOR
  if(transfer_parent_) {status["task_id"]=input().task_id;status["context_id"]=input().context_id;status["object_id"]=input().object_id;status["task_phase"]=transfer_phase_name();status["stage_id"]=stage_id_;status["navigation_goal_uuid"]=navigation_?navigation_->status().nav_goal_uuid:"";}
#endif
  std_msgs::msg::String message;message.data=status.dump();state_pub_->publish(message);
 }
 std::map<std::string,std::vector<std::string>> groups_;std::set<std::string> all_joints_,cancel_sent_;std::vector<std::string> names_;
 std::unique_ptr<ResourceJournal> journal_;std::unique_ptr<ResourceAuthority> authority_;ArmHold hold_;
 std::unique_ptr<ChildActions> children_;std::map<std::string,Child::SharedPtr> handles_;
 std::map<std::string,rclcpp_action::Client<Trajectory>::SharedPtr> clients_;
 rclcpp_action::Server<Action>::SharedPtr server_;std::shared_ptr<Parent> parent_;
 rclcpp::Client<Query>::SharedPtr controllers_;rclcpp::Service<astribot_s1_transport_native::srv::RenewHold>::SharedPtr renew_;
 rclcpp::Publisher<Hold>::SharedPtr hold_pub_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
 rclcpp::Subscription<Geometry>::SharedPtr geometry_sub_;rclcpp::TimerBase::SharedPtr timer_;
 std::optional<Geometry> geometry_;std::vector<controller_manager_msgs::msg::ControllerState> claims_;
 std::deque<std::pair<Geometry,int64_t>> settling_;
 int64_t geometry_deadline_=0,claims_ros_=-1,claims_wall_=-1,query_sent_wall_=0,query_id_=0,started_=0,completed_at_=0,stop_at_=0,stop_ros_=0;
 bool motion_ever_sent_=false;
 bool query_pending_=false,hold_started_=false,hold_confirmed_=false,terminal_barrier_=false;
 std::string reason_="WAITING_FOR_TASK",stop_reason_,hold_id_;
#ifdef PLAN_TO_HOLD_EXECUTOR

 PayloadPhase payload_phase_=PayloadPhase::NONE;
 std::string payload_model_,payload_object_,payload_world_,payload_robot_,payload_xml_,payload_base_,payload_tcp_;
 astribot::payload::Config payload_config_;
 std::unique_ptr<astribot::payload::Consumer> payload_consumer_;
 std::unique_ptr<PayloadClient> payload_client_;std::unique_ptr<PayloadFrames> payload_frames_;
 std::shared_ptr<PayloadWorldInbox> world_inbox_;std::unique_ptr<ignition::transport::Node> world_node_;
 std::unique_ptr<tf2_ros::Buffer> tf_;std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
 std::set<std::string> known_links_;
 Eigen::Isometry3d model_from_root_=Eigen::Isometry3d::Identity();
 std::deque<RawObservation> observations_;std::deque<Diagnostic> diagnostics_;
 rclcpp::Subscription<Observation>::SharedPtr observation_sub_;rclcpp::Subscription<PayloadState>::SharedPtr payload_state_sub_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr diagnostic_sub_;rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
 rclcpp::Client<SceneApply>::SharedPtr apply_client_;rclcpp::Client<PayloadRevalidate>::SharedPtr payload_revalidate_;
 std::optional<sensor_msgs::msg::JointState> actual_joints_;int64_t joints_deadline_=0;
 std::map<std::string,double> payload_stopped_joints_,payload_joint_bounds_;
 std::string payload_epoch_,payload_ledger_epoch_,payload_transaction_;uint64_t payload_clock_=0;
 double payload_radius_=0,placement_tolerance_=0.025;std::vector<double> payload_size_;
 int64_t payload_started_=0,payload_sent_=0,payload_request_=0,payload_checked_at_=0;
 bool payload_observation_bound_=false;std::string payload_raw_failure_;
 bool payload_physical_applied_=false,payload_scene_unresolved_=false,payload_query_pending_=false,payload_query_final_=false;
 moveit_msgs::msg::PlanningScene payload_before_,payload_readback_;Observation payload_observation_;
 geometry_msgs::msg::Pose payload_base_object_;Eigen::Vector3d placement_first_=Eigen::Vector3d::Zero(),placement_requested_world_=Eigen::Vector3d::Zero();
 int64_t placement_first_ros_=0,placement_first_wall_=0;
 SceneBinding payload_revalidated_binding_;
 rclcpp_action::Server<FullAction>::SharedPtr full_server_;std::shared_ptr<FullParent> full_parent_;
 Action::Goal input_;bool full_operation_=false;uint64_t stage_generation_=0,operation_generation_=0;
 enum class TransferPhase {PICK_PREPARE,PICK,NAVIGATE,PLACE_REVOKE,PLACE_PREPARE,PLACE,FINAL_VERIFY,COMPLETE};
 TransferPhase transfer_phase_=TransferPhase::PICK_PREPARE;
 rclcpp_action::Server<Transfer>::SharedPtr transfer_server_;std::shared_ptr<TransferParent> transfer_parent_;
 std::unique_ptr<FixedStationNavigation> navigation_;
 bool navigation_started_=false,navigation_cleanup_revoke_=false,station_pending_=false,station_scene_unresolved_=false,station_ready_=false,final_scene_pending_=false;
 int64_t transfer_started_=0,transfer_budget_=0,station_sent_=0,operation_started_ros_=0,final_checked_at_=0;
 moveit_msgs::msg::PlanningScene station_before_,station_diff_;
 std::optional<std::pair<ignition::msgs::Pose_V,PayloadCommand::Receipt>> world_snapshot_;int64_t world_snapshot_stamp_=0;
 std::array<uint64_t,2> station_entities_{};
 double navigation_position_tolerance_=0.002,navigation_yaw_tolerance_=0.0017453292519943296;
 rclcpp_action::Client<Planner>::SharedPtr planner_;PlannerHandle::SharedPtr planner_handle_;
 rclcpp::Client<SceneQuery>::SharedPtr scenes_;rclcpp::Client<GuardService>::SharedPtr guard_client_;rclcpp::Client<Revoke>::SharedPtr revoke_client_;
 rclcpp::Client<Revalidate>::SharedPtr revalidate_client_;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;rclcpp::Subscription<Envelope>::SharedPtr envelope_sub_;rclcpp::Subscription<GuardState>::SharedPtr guard_sub_;
 std::optional<nav_msgs::msg::Odometry> odom_;std::optional<Envelope> envelope_;std::optional<GuardState> guard_state_;
 nav_msgs::msg::Odometry reference_base_;Geometry reference_geometry_;Envelope reference_envelope_;
 SceneBinding scene_binding_;std::unique_ptr<MtcPlan> plan_;trajectory_msgs::msg::JointTrajectory trajectory_;
 SceneBinding revalidation_binding_;
 std::map<std::string,double> endpoint_;std::string stage_id_,stage_kind_,active_controller_;
 int64_t odom_received_=0,envelope_received_=0,guard_received_=0,scene_sent_=0,scene_request_=0,scene_verified_at_=0;
 bool planner_cancel_sent_=false,scene_pending_=false,guard_pending_=false,guard_ack_=false,motion_sent_=false,scene_verified_=false,scene_final_verified_=false,revoke_ack_=false,guard_requested_=false,guard_disarm_requested_=false,guard_disarmed_=false;
 bool scene_stabilizing_=false,revalidation_pending_=false,revalidation_readback_=false;
 int64_t scene_stability_started_=0,scene_stable_since_=0,scene_retry_at_=0,revalidation_sent_=0,revalidation_request_=0;
 bool revoke_pending_=false,revocation_verified_=false,revoke_wait_negative_=false;
 uint64_t revoke_epoch_=0,revoke_generation_=0,envelope_sequence_=0,revoke_envelope_sequence_=0;
 int64_t revoke_sent_=0,revoke_request_=0,revoke_ack_at_=0,revoke_ros_=0;
#endif
};
}
int main(int argc,char **argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<astribot::transport::HoldExecutor>());}catch(const std::exception &e){std::fprintf(stderr,"hold_executor: %s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
