#include "astribot_s1_transport_native/arm_hold.hpp"
#include "astribot_s1_transport_native/resource_authority.hpp"
#include "astribot_s1_transport_native/resource_journal.hpp"
#include "astribot_s1_transport_native/child_actions.hpp"
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
using Action=astribot_s1_transport_native::action::HoldResources;
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
 HoldExecutor():Node("task_hold_executor"),groups_(resources()) {
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
  server_=rclcpp_action::create_server<Action>(this,"/transport/hold_resources",
   [this](const auto &,const auto goal){
    const auto ros=now().nanoseconds(),steady=wall();
    if(parent_||!geometry_fresh(ros,steady)||!claims_fresh(ros,steady)||!exclusive_graph())return rclcpp_action::GoalResponse::REJECT;
    for(const auto &[name,client]:clients_)if(!client->action_server_is_ready()){reason_="CONTROLLER_ACTION_UNAVAILABLE";return rclcpp_action::GoalResponse::REJECT;}
    auto acquired=authority_->acquire(goal->task_id,goal->request_id,ros,steady);reason_=acquired.reason;
    return acquired.accepted?rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE:rclcpp_action::GoalResponse::REJECT;
   },
   [this](const auto handle){if(handle!=parent_)return rclcpp_action::CancelResponse::REJECT;stopping("TASK_CANCELED");return rclcpp_action::CancelResponse::ACCEPT;},
   [this](const auto handle){start(handle);});
  timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
 }
private:
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
  parent_=handle;children_=std::make_unique<ChildActions>(names_);handles_.clear();cancel_sent_.clear();hold_started_=false;hold_confirmed_=false;terminal_barrier_=false;stop_at_=0;settling_.clear();
  started_=wall();hold_id_=authority_->lease_id()+"_hold";
  try {
   if(!geometry_fresh(now().nanoseconds(),wall())||!claims_fresh(now().nanoseconds(),wall())||!exclusive_graph()){stopping(reason_);return;}
   std::map<std::string,double> positions;for(size_t i=0;i<geometry_->joints.name.size();++i)positions[geometry_->joints.name[i]]=geometry_->joints.position[i];
   authority_->submitted(now().nanoseconds(),wall());
   for(const auto &[name,joints]:groups_) {
    children_->submitted(name);if(!authority_->record("child_submission",{{"controller",name}}))throw std::runtime_error(authority_->reason());
    Trajectory::Goal goal;goal.trajectory.joint_names=joints;
    trajectory_msgs::msg::JointTrajectoryPoint point;for(const auto &joint:joints)point.positions.push_back(positions.at(joint));
    point.velocities.assign(joints.size(),0.);point.time_from_start.sec=1;goal.trajectory.points.push_back(point);goal.goal_time_tolerance.sec=1;
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
  for(const auto &[name,handle]:handles_)if(handle&&!cancel_sent_.count(name)&&handle->get_status()<4) {
   cancel_sent_.insert(name);try{clients_.at(name)->async_cancel_goal(handle);}catch(const std::exception &e){authority_->record("cancel_unconfirmed",{{"controller",name},{"error",e.what()}});}
  }
 }
 void stopping(const std::string &reason) {
  if(!parent_)return;
  if(!stop_at_){stop_at_=wall();stop_ros_=now().nanoseconds();reason_=reason;authority_->stop(reason,stop_ros_,stop_at_);hold_.cancel();settling_.clear();}
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
    if(!stop_at_&&children_->all_successful()&&!hold_started_) {
     completed_at_=ros;HoldCompletion completion{grant->owner_id,grant->lease_id,grant->epoch,children_->proof(),ros,true,true};
     hold_.begin(hold_id_,*grant,completion,ros,steady);hold_started_=true;
     hold_.controllers(claims_,claims_ros_,claims_wall_,ros,steady);
    }
    if(!stop_at_&&hold_started_) {
     hold_.resource(*grant,ros,steady);auto state=hold_.status(ros,steady);
     if(state.hold_confirmed&&!hold_confirmed_){authority_->holding(ros,steady);hold_confirmed_=true;reason_="HOLD_CONFIRMED";}
     if(hold_confirmed_&&!state.hold_confirmed)stopping(hold_.reason());
     if(!state.hold_confirmed&&hold_.reason().rfind("WAITING_",0)!=0)stopping(hold_.reason());
    }
    if(!stop_at_&&!hold_confirmed_&&steady-started_>15000*MS)stopping("HOLD_COMPLETION_TIMEOUT");
    if(stop_at_) {
     cancel_children();
     if(children_->all_terminal()&&!terminal_barrier_){terminal_barrier_=true;stop_ros_=ros;settling_.clear();}
     if(authority_->release(children_->all_terminal(),!children_->sent()||measured_safe(ros,steady),ros,steady)) {
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
  auto state=hold_.status(ros,steady);if(stop_at_)state.hold_confirmed=false;hold_pub_->publish(state);
  const auto phase=std::to_string(int(authority_->phase()));
  if(parent_&&parent_->is_active()) {auto feedback=std::make_shared<Action::Feedback>();feedback->phase=phase;feedback->reason=reason_;feedback->lease_id=authority_->lease_id();feedback->resource_epoch=authority_->epoch();feedback->hold_id=hold_id_;feedback->hold_confirmed=state.hold_confirmed;parent_->publish_feedback(feedback);}
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
};
}
int main(int argc,char **argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<astribot::transport::HoldExecutor>());}catch(const std::exception &e){std::fprintf(stderr,"hold_executor: %s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
