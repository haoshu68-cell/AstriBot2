#include "astribot_s1_transport_native/arm_hold.hpp"
#include "astribot_s1_transport_native/resource_authority.hpp"
#include "astribot_s1_transport_native/resource_journal.hpp"
#include "astribot_s1_transport_native/child_actions.hpp"
#ifdef PLAN_TO_HOLD_EXECUTOR
#include "astribot_s1_transport_native/mtc_plan.hpp"
#include "astribot_s1_transport_native/scene_binding.hpp"
#include <astribot_s1_transport_native/action/plan_to_hold.hpp>
#include <astribot_transport_msgs/action/plan_manipulation.hpp>
#include <astribot_transport_msgs/srv/set_execution_guard.hpp>
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
#include <fstream>
#include <filesystem>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <map>
namespace astribot::transport {
namespace {
constexpr int64_t MS=1000000;
int64_t wall(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int64_t ns(const builtin_interfaces::msg::Time &t){return int64_t(t.sec)*1000000000+t.nanosec;}
#ifdef PLAN_TO_HOLD_EXECUTOR
using Action=astribot_s1_transport_native::action::PlanToHold;
constexpr const char *NODE_NAME="task_trajectory_executor",*ACTION_NAME="/transport/plan_to_hold";
using Planner=astribot_transport_msgs::action::PlanManipulation;
using PlannerHandle=rclcpp_action::ClientGoalHandle<Planner>;
using SceneQuery=moveit_msgs::srv::GetPlanningScene;
using GuardService=astribot_transport_msgs::srv::SetExecutionGuard;
using GuardState=astribot_transport_msgs::msg::ExecutionGuardStatus;
using Envelope=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
using Revoke=astribot_navigation_msgs::srv::SetRobotEnvelope;
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
  odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::SharedPtr m){odom_=*m;odom_received_=wall();});
  envelope_sub_=create_subscription<Envelope>("/navigation/envelope_v2",10,[this](Envelope::SharedPtr m){envelope_=*m;envelope_received_=wall();if(parent_&&!hold_confirmed_&&!manipulation_ready(now().nanoseconds(),wall()))stopping(reason_);});
  guard_sub_=create_subscription<GuardState>("/transport/execution_guard/status",10,[this](GuardState::SharedPtr m){guard_state_=*m;guard_received_=wall();});
#endif
  server_=rclcpp_action::create_server<Action>(this,ACTION_NAME,
   [this](const auto &,const auto goal){
    const auto ros=now().nanoseconds(),steady=wall();
    if(parent_||!geometry_fresh(ros,steady)||!claims_fresh(ros,steady)||!exclusive_graph())return rclcpp_action::GoalResponse::REJECT;
#ifdef PLAN_TO_HOLD_EXECUTOR
    if(goal->context_id.empty()||goal->object_id.empty()||(goal->operation!="PICK"&&goal->operation!="PLACE")||!manipulation_ready(ros,steady)||!planner_->action_server_is_ready()||!scenes_->service_is_ready()||!guard_client_->service_is_ready()||!revoke_client_->service_is_ready())return rclcpp_action::GoalResponse::REJECT;
#endif
    for(const auto &[name,client]:clients_)if(!client->action_server_is_ready()){reason_="CONTROLLER_ACTION_UNAVAILABLE";return rclcpp_action::GoalResponse::REJECT;}
    auto acquired=authority_->acquire(goal->task_id,goal->request_id,ros,steady);reason_=acquired.reason;
    return acquired.accepted?rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE:rclcpp_action::GoalResponse::REJECT;
   },
   [this](const auto handle){if(handle!=parent_)return rclcpp_action::CancelResponse::REJECT;stopping("TASK_CANCELED");return rclcpp_action::CancelResponse::ACCEPT;},
   [this](const auto handle){start(handle);});
  timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
 }
private:
#ifdef PLAN_TO_HOLD_EXECUTOR
 bool manipulation_ready(int64_t ros,int64_t steady) {
  if(!odom_||ns(odom_->header.stamp)<=0||ros<ns(odom_->header.stamp)||ros-ns(odom_->header.stamp)>300*MS||steady-odom_received_>300*MS){reason_="MTC_BASE_STATE_STALE";return false;}
  const auto &v=odom_->twist.twist;const auto &p=odom_->pose.pose.position;const auto &q=odom_->pose.pose.orientation;
  if(!std::isfinite(v.linear.x+v.linear.y+v.angular.z+p.x+p.y+p.z+q.x+q.y+q.z+q.w)||std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.001||odom_->header.frame_id.empty()||std::hypot(v.linear.x,v.linear.y)>=.02||std::abs(v.angular.z)>=.03){reason_="ARM_REQUIRES_STOPPED_BASE";return false;}
  if(!envelope_||envelope_->coordinator_session_id.empty()||envelope_->mode!=Envelope::HOLD||envelope_->navigation_allowed||ns(envelope_->header.stamp)>ros||ros>=ns(envelope_->valid_until)||steady-envelope_received_>=ns(envelope_->valid_until)-ns(envelope_->header.stamp)){reason_="MTC_REQUIRES_FRESH_FIXED_HOLD";return false;}
  return true;
 }
 void revoke_navigation() {
  auto request=std::make_shared<Revoke::Request>();request->envelope.transport_ready=false;
  const auto lease=authority_->lease_id();
  try{revoke_client_->async_send_request(request,[this,lease](rclcpp::Client<Revoke>::SharedFuture f){
   if(!parent_||lease!=authority_->lease_id())return;
   try {auto result=f.get();if(!result->accepted){stopping("NAVIGATION_REVOCATION_REJECTED:"+result->reason);return;}
    revoke_ack_=true;authority_->record("navigation_revoked",{{"epoch",result->epoch}});
   }catch(const std::exception&e){stopping(std::string("NAVIGATION_REVOCATION_ERROR:")+e.what());}
  });}catch(const std::exception&e){reason_=std::string("NAVIGATION_REVOCATION_UNCONFIRMED:")+e.what();authority_->record("navigation_revocation_unconfirmed",{{"error",e.what()}});}
 }
 void query_scene(bool recheck) {
  auto request=std::make_shared<SceneQuery::Request>();request->components.components=1023;
  scene_pending_=true;scene_sent_=wall();const auto lease=authority_->lease_id();
  auto pending=scenes_->async_send_request(request,[this,lease,recheck](rclcpp::Client<SceneQuery>::SharedFuture f){
   if(!parent_||lease!=authority_->lease_id())return;
   scene_pending_=false;
   if(stop_at_)return;
   try {
    auto scene=f.get()->scene;
    if(!geometry_fresh(now().nanoseconds(),wall())||!manipulation_ready(now().nanoseconds(),wall()))throw std::runtime_error(reason_);
    auto signature=bind_scene(scene);
    if(recheck){
     if(signature!=scene_binding_)throw std::runtime_error("MTC_SCENE_CHANGED");
     std::set<std::string> attached;for(const auto&o:scene.robot_state.attached_collision_objects)attached.insert(o.object.id);
     const auto&stage=plan_->check(parent_->get_goal()->operation=="PICK"?"PREGRASP":"PREPLACE","ARM",wall(),geometry_->joints,attached);
     trajectory_=stage.trajectory.joint_trajectory;stage_id_=stage.stage_id;
     if(!stage.trajectory.multi_dof_joint_trajectory.points.empty()||trajectory_.points.empty()||trajectory_.joint_names!=groups_.at("arm_left_controller"))throw std::runtime_error("MTC_UNSUPPORTED_ARM_TRAJECTORY");
     int64_t previous=-1;
     for(const auto&point:trajectory_.points){
      const auto duration=int64_t(point.time_from_start.sec)*1000000000+point.time_from_start.nanosec;
      if(duration<0||duration<=previous||duration>120000000000LL||point.positions.size()!=trajectory_.joint_names.size()||(!point.velocities.empty()&&point.velocities.size()!=point.positions.size())||(!point.accelerations.empty()&&point.accelerations.size()!=point.positions.size())||!point.effort.empty())throw std::runtime_error("MTC_TRAJECTORY_FIELDS_INVALID");
      for(const auto*values:{&point.positions,&point.velocities,&point.accelerations})for(double value:*values)if(!std::isfinite(value))throw std::runtime_error("MTC_TRAJECTORY_NONFINITE");
      previous=duration;
     }
     for(size_t i=0;i<trajectory_.joint_names.size();++i)endpoint_[trajectory_.joint_names[i]]=trajectory_.points.back().positions[i];
     scene_verified_=true;scene_verified_at_=wall();
     if(!guard_ack_)arm_guard();
     else {scene_final_verified_=true;advance_plan(now().nanoseconds(),wall());}
    }else {
     scene_binding_=std::move(signature);scene.robot_state.joint_state=geometry_->joints;reference_geometry_=*geometry_;
     const auto&input=*parent_->get_goal();Planner::Goal goal;
     goal.operation=input.operation;goal.object_id=input.object_id;goal.context_id=input.context_id;goal.scene=scene;
     goal.pre_target=input.pre_target;goal.target=input.target;goal.exit_targets=input.exit_targets;goal.touch_links=input.touch_links;goal.grasp_width_m=input.grasp_width_m;goal.timeout_s=45.;
     authority_->submitted(now().nanoseconds(),wall());children_->submitted("mtc");
     if(!authority_->record("planner_submission",{{"context",goal.context_id},{"geometry_sequence",geometry_->sequence}}))throw std::runtime_error(authority_->reason());
     rclcpp_action::Client<Planner>::SendGoalOptions options;
     options.goal_response_callback=[this,lease](PlannerHandle::SharedPtr h){if(!parent_||lease!=authority_->lease_id())return;try{children_->response("mtc",h?canonical_goal_id(h->get_goal_id()):"");planner_handle_=h;if(!h)stopping("MTC_PLAN_REJECTED");}catch(const std::exception&e){stopping(e.what());}};
     options.result_callback=[this,lease](const PlannerHandle::WrappedResult &result){
      if(!parent_||lease!=authority_->lease_id())return;
      try {
       const bool terminal=result.result&&(result.code==rclcpp_action::ResultCode::SUCCEEDED||result.code==rclcpp_action::ResultCode::ABORTED||result.code==rclcpp_action::ResultCode::CANCELED);
       const bool okay=result.code==rclcpp_action::ResultCode::SUCCEEDED&&result.result&&result.result->success;
       children_->result("mtc",canonical_goal_id(result.goal_id),okay,terminal);
       if(!authority_->record("planner_terminal",{{"context",parent_->get_goal()->context_id},{"success",okay}}))throw std::runtime_error(authority_->reason());
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
 // The owning caller may later supply M3's already-validated complete result at
 // this internal boundary. There is no ROS endpoint accepting arbitrary paths.
 void consume_plan(const Planner::Result &result) {
  const auto &goal=*parent_->get_goal();
  plan_=std::make_unique<MtcPlan>(goal.operation,result.stages,goal.context_id,result.context_id,wall(),all_joints_);
  query_scene(true);
 }
 void arm_guard() {
  auto request=std::make_shared<GuardService::Request>();request->enable=true;request->context_id=parent_->get_goal()->context_id;
  request->joint_names=trajectory_.joint_names;request->base_reference.header=reference_base_.header;request->base_reference.pose=reference_base_.pose.pose;
  guard_pending_=true;guard_requested_=true;const auto lease=authority_->lease_id();
  guard_client_->async_send_request(request,[this,lease](rclcpp::Client<GuardService>::SharedFuture f){
   if(!parent_||lease!=authority_->lease_id())return;
   guard_pending_=false;
   try{auto r=f.get();if(!r->accepted){guard_requested_=false;stopping("EXECUTION_GUARD_REJECTED:"+r->reason);}else guard_ack_=true;}catch(const std::exception&e){stopping(e.what());}
  });reason_="WAITING_FOR_EXECUTION_GUARD";
 }
 void advance_plan(int64_t ros,int64_t steady) {
  if(hold_confirmed_)return; // M2 may now grant navigation with this same hold.
  if(!manipulation_ready(ros,steady)){stopping(reason_);return;}
  if(geometry_->source_id!=reference_geometry_.source_id||geometry_->clock_epoch!=reference_geometry_.clock_epoch||geometry_->model_revision!=reference_geometry_.model_revision||geometry_->attachment_revision!=reference_geometry_.attachment_revision||envelope_->coordinator_session_id!=reference_envelope_.coordinator_session_id){stopping("MTC_CONTEXT_CHANGED");return;}
  const auto&a=odom_->pose.pose;const auto&b=reference_base_.pose.pose;
  const double dot=std::abs(a.orientation.x*b.orientation.x+a.orientation.y*b.orientation.y+a.orientation.z*b.orientation.z+a.orientation.w*b.orientation.w);
  if(odom_->header.frame_id!=reference_base_.header.frame_id||std::hypot(std::hypot(a.position.x-b.position.x,a.position.y-b.position.y),a.position.z-b.position.z)>.02||2*std::acos(std::min(1.,dot))>.02){stopping("MTC_BASE_MOVED");return;}
  if(scene_pending_&&steady-scene_sent_>3000*MS){scenes_->remove_pending_request(scene_request_);scene_pending_=false;stopping("MTC_SCENE_TIMEOUT");return;}
  if(scene_verified_&&!motion_sent_){
   if(steady-scene_verified_at_>2000*MS){stopping("EXECUTION_GUARD_TIMEOUT");return;}
   if(!guard_ack_||!revoke_ack_)return;
   if(!guard_state_||guard_state_->context_id!=parent_->get_goal()->context_id||!guard_state_->active)return;
   if(!guard_state_->healthy){if(guard_state_->reason!="WAITING_FOR_EXECUTION_EVIDENCE")stopping(guard_state_->reason);return;}
   if(steady-guard_received_>300*MS||ros<ns(guard_state_->stamp)||ros-ns(guard_state_->stamp)>300*MS)return;
   if(!scene_final_verified_){if(!scene_pending_)query_scene(true);return;}
   std::set<std::string> attached(geometry_->attachment_ids.begin(),geometry_->attachment_ids.end());
   plan_->check(parent_->get_goal()->operation=="PICK"?"PREGRASP":"PREPLACE","ARM",steady,geometry_->joints,attached);
   motion_sent_=true;send_children();reason_="EXECUTING_FIRST_MTC_STAGE";
  }
  if(motion_sent_&&!guard_disarm_requested_){
   if(!guard_state_||guard_state_->context_id!=parent_->get_goal()->context_id||!guard_state_->active||!guard_state_->healthy||steady-guard_received_>300*MS||ros<ns(guard_state_->stamp)||ros-ns(guard_state_->stamp)>300*MS){stopping("EXECUTION_GUARD_UNHEALTHY");return;}
  }
 }
 bool finish_guard() {
  if(!guard_requested_&&!guard_pending_)return true;
  if(guard_disarmed_)return true;
  if(guard_pending_||guard_disarm_requested_)return false;
  guard_disarm_requested_=true;auto request=std::make_shared<GuardService::Request>();request->context_id=parent_->get_goal()->context_id;request->enable=false;
  const auto lease=authority_->lease_id();
  guard_client_->async_send_request(request,[this,lease](rclcpp::Client<GuardService>::SharedFuture f){
   if(!parent_||lease!=authority_->lease_id())return;
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
 void start(std::shared_ptr<Parent> handle) {
  parent_=handle;auto child_names=names_;
#ifdef PLAN_TO_HOLD_EXECUTOR
  child_names.push_back("mtc");planner_handle_.reset();planner_cancel_sent_=false;plan_.reset();scene_pending_=false;guard_pending_=false;guard_ack_=false;guard_requested_=false;guard_disarm_requested_=false;guard_disarmed_=false;motion_sent_=false;scene_verified_=false;scene_final_verified_=false;stage_id_.clear();endpoint_.clear();
  reference_geometry_=*geometry_;reference_base_=*odom_;reference_envelope_=*envelope_;
#endif
  children_=std::make_unique<ChildActions>(child_names);handles_.clear();cancel_sent_.clear();hold_started_=false;hold_confirmed_=false;terminal_barrier_=false;stop_at_=0;settling_.clear();
  started_=wall();hold_id_=authority_->lease_id()+"_hold";
#ifdef PLAN_TO_HOLD_EXECUTOR
  revoke_ack_=false;revoke_navigation();query_scene(false);
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
    if(name=="arm_left_controller")goal.trajectory=trajectory_;
    else goal.trajectory.points.front().time_from_start=trajectory_.points.back().time_from_start;
#endif
    auto options=rclcpp_action::Client<Trajectory>::SendGoalOptions();
    const auto lease=authority_->lease_id();
    options.goal_response_callback=[this,name,lease](Child::SharedPtr child){
     if(!parent_||lease!=authority_->lease_id())return;
     try{auto id=child?canonical_goal_id(child->get_goal_id()):"";children_->response(name,id);handles_[name]=child;
      if(!authority_->record("child_response",{{"controller",name},{"uuid",id}}))stopping(authority_->reason());
      if(!child)stopping("CHILD_GOAL_REJECTED:"+name);
      // Cancellation is dispatched by tick(), outside action-client callbacks.
     }catch(const std::exception &e){stopping(e.what());}
    };
    options.result_callback=[this,name,lease](const Child::WrappedResult &result){
     if(!parent_||lease!=authority_->lease_id())return;
     try{const auto uuid=canonical_goal_id(result.goal_id);const bool okay=result.code==rclcpp_action::ResultCode::SUCCEEDED&&result.result&&result.result->error_code==Trajectory::Result::SUCCESSFUL;
      const bool terminal=result.result&&(result.code==rclcpp_action::ResultCode::SUCCEEDED||result.code==rclcpp_action::ResultCode::ABORTED||result.code==rclcpp_action::ResultCode::CANCELED);
      children_->result(name,uuid,okay,terminal);
      if(!authority_->record("child_terminal",{{"controller",name},{"uuid",uuid},{"success",okay},{"result_code",int(result.code)}}))stopping(authority_->reason());
      if(!okay)stopping("CHILD_RESULT_FAILED:"+name);
     }catch(const std::exception &e){stopping(e.what());}
    };
    clients_.at(name)->async_send_goal(goal,options);
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
  if(!parent_)return;
  if(!stop_at_){stop_at_=wall();stop_ros_=now().nanoseconds();reason_=reason;authority_->stop(reason,stop_ros_,stop_at_);hold_.cancel();settling_.clear();
#ifdef PLAN_TO_HOLD_EXECUTOR
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
  const auto ros=now().nanoseconds(),steady=wall();
  try {
   query_controllers();
   if(parent_) {
    auto grant=authority_->grant(ros,steady);
    if(!stop_at_&&(!grant||!geometry_fresh(ros,steady)||!claims_fresh(ros,steady)||!exclusive_graph()))stopping(grant?reason_:authority_->reason());
#ifdef PLAN_TO_HOLD_EXECUTOR
    if(!stop_at_)advance_plan(ros,steady);
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
     if(completion_ready&&!hold_confirmed_){authority_->holding(ros,steady);hold_confirmed_=true;plan_->acknowledge();reason_="FIRST_STAGE_HOLD_CONFIRMED";}
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
    if(stop_at_) {
     cancel_children();
     if(children_->all_terminal()&&!terminal_barrier_){terminal_barrier_=true;stop_ros_=ros;settling_.clear();}
     bool ready=children_->all_terminal()&&(!children_->sent()||measured_safe(ros,steady));
#ifdef PLAN_TO_HOLD_EXECUTOR
     ready=ready&&finish_guard();
#endif
     if(authority_->release(ready,ready,ros,steady)) {
      auto result=std::make_shared<Action::Result>();result->resources_released=true;result->reason=reason_;result->lease_id=authority_->lease_id();
      if(parent_->is_active()){if(parent_->is_canceling())parent_->canceled(result);else parent_->abort(result);}parent_.reset();
     } else if(steady-stop_at_>10000*MS&&parent_->is_active()) {
      authority_->uncertain_result("RESOURCE_RECOVERY_REQUIRED:"+reason_,ros,steady);
      auto result=std::make_shared<Action::Result>();result->resources_released=false;result->reason="RESOURCE_RECOVERY_REQUIRED:"+reason_;result->lease_id=authority_->lease_id();parent_->abort(result);
      // Keep the task/child context and deny new admission. Late real results
      // remain evidence, but never re-execute a child after an uncertain send.
     }
    }
   }
  }catch(const std::exception &e){stopping(e.what());}
  auto state=hold_.status(ros,steady);if(stop_at_)state.hold_confirmed=false;
#ifdef PLAN_TO_HOLD_EXECUTOR
  if(!hold_confirmed_)state.hold_confirmed=false;
#endif
  hold_pub_->publish(state);
  const auto phase=std::to_string(int(authority_->phase()));
  if(parent_&&parent_->is_active()) {auto feedback=std::make_shared<Action::Feedback>();feedback->phase=phase;feedback->reason=reason_;feedback->lease_id=authority_->lease_id();feedback->resource_epoch=authority_->epoch();feedback->hold_id=hold_id_;feedback->hold_confirmed=state.hold_confirmed;
#ifdef PLAN_TO_HOLD_EXECUTOR
   feedback->context_id=parent_->get_goal()->context_id;feedback->stage_id=stage_id_;
#endif
   parent_->publish_feedback(feedback);}
  std_msgs::msg::String message;message.data=nlohmann::json({{"phase",phase},{"reason",reason_},{"authority_reason",authority_->reason()},{"hold_reason",hold_.reason()},{"lease_id",authority_->lease_id()},{"epoch",authority_->epoch()},{"hold_id",hold_id_},{"hold_confirmed",state.hold_confirmed}}).dump();state_pub_->publish(message);
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
 bool query_pending_=false,hold_started_=false,hold_confirmed_=false,terminal_barrier_=false;
 std::string reason_="WAITING_FOR_TASK",hold_id_;
#ifdef PLAN_TO_HOLD_EXECUTOR
 rclcpp_action::Client<Planner>::SharedPtr planner_;PlannerHandle::SharedPtr planner_handle_;
 rclcpp::Client<SceneQuery>::SharedPtr scenes_;rclcpp::Client<GuardService>::SharedPtr guard_client_;rclcpp::Client<Revoke>::SharedPtr revoke_client_;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;rclcpp::Subscription<Envelope>::SharedPtr envelope_sub_;rclcpp::Subscription<GuardState>::SharedPtr guard_sub_;
 std::optional<nav_msgs::msg::Odometry> odom_;std::optional<Envelope> envelope_;std::optional<GuardState> guard_state_;
 nav_msgs::msg::Odometry reference_base_;Geometry reference_geometry_;Envelope reference_envelope_;
 SceneBinding scene_binding_;std::unique_ptr<MtcPlan> plan_;trajectory_msgs::msg::JointTrajectory trajectory_;
 std::map<std::string,double> endpoint_;std::string stage_id_;
 int64_t odom_received_=0,envelope_received_=0,guard_received_=0,scene_sent_=0,scene_request_=0,scene_verified_at_=0;
 bool planner_cancel_sent_=false,scene_pending_=false,guard_pending_=false,guard_ack_=false,motion_sent_=false,scene_verified_=false,scene_final_verified_=false,revoke_ack_=false,guard_requested_=false,guard_disarm_requested_=false,guard_disarmed_=false;
#endif
};
}
int main(int argc,char **argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<astribot::transport::HoldExecutor>());}catch(const std::exception &e){std::fprintf(stderr,"hold_executor: %s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
