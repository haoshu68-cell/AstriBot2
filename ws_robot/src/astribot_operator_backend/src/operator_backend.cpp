#include "astribot_operator_backend/arm_preview.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <astribot_operator_msgs/srv/operator_command.hpp>
#include <astribot_operator_msgs/srv/exploration_command.hpp>
#include <astribot_operator_msgs/srv/start_loop_route.hpp>
#include <astribot_operator_msgs/srv/cancel_loop_route.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <astribot_navigation_msgs/msg/navigation_execution_status.hpp>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <random>
#include <map>
#include <set>
#include <cmath>
using Json=nlohmann::json;
class OperatorBackend : public rclcpp::Node {
 using Command=astribot_operator_msgs::srv::OperatorCommand;
 using Explore=astribot_operator_msgs::srv::ExplorationCommand;
 using Start=astribot_operator_msgs::srv::StartLoopRoute;
 using Cancel=astribot_operator_msgs::srv::CancelLoopRoute;
 using Trigger=std_srvs::srv::Trigger;
 using Nav=nav2_msgs::action::NavigateToPose;
 using Handle=rclcpp_action::ClientGoalHandle<Nav>;
 using Clock=std::chrono::steady_clock;
 struct Observation {Json value;Clock::time_point at{};rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;};
 struct Record {std::string operation,owner,fingerprint,state{"ACCEPTED"},code{"COMMAND.ACCEPTED"},message;Json request=Json::object(),result=Json::object();Clock::time_point deadline;};
 std::string robot_,boot_,lease_,owner_,acquire_id_,acquire_payload_,nav_id_,route_id_;
 std::set<std::string> acquired_ids_;
 std::unique_ptr<ArmPreview> arm_;
 std::string armContext()const {return fresh("map_catalog")?observations_.at("map_catalog").value.value("active_map",Json()).dump()+observations_.at("map_catalog").value.value("active_scene",Json()).dump()+std::to_string(observations_.at("map_catalog").value.value("scene_ready",true))+(require_zones_?(zonesReady()?observations_.at("navigation_zones").value.value("token",std::string()):"zones-unavailable"):""):"unknown";}
 double ttl_;
 Clock::time_point lease_until_{};
 std::map<std::string,Observation> observations_;
 std::map<std::string,Record> records_;size_t records_bytes_{0};
 bool nav_outstanding_{false},route_pending_{false},exploration_owned_{false},exploration_pending_{false},stopping_{false};
 std::string exploration_boot_,last_canceled_route_;uint64_t exploration_revision_{0},last_pause_revision_{UINT64_MAX};
 Handle::SharedPtr nav_handle_;
 std::map<std::string,std::string> owned_navigation_,navigation_reasons_;
 rclcpp::Subscription<astribot_navigation_msgs::msg::NavigationExecutionStatus>::SharedPtr navigation_status_;
 rclcpp_action::Client<Nav>::SharedPtr nav_;
 rclcpp::Client<Explore>::SharedPtr explore_;
 rclcpp::Client<Command>::SharedPtr zones_client_;bool zone_pending_{false},require_zones_{false};std::string owned_zone_token_;
 bool zonesReady()const{return fresh("navigation_zones")&&observations_.at("navigation_zones").value.value("ready",false)&&!zone_pending_;}
 rclcpp::Client<Command>::SharedPtr maps_;bool map_pending_{false},require_maps_{false},require_sim_transport_{false},sim_start_pending_{false};
 rclcpp::Client<Start>::SharedPtr route_start_;
 rclcpp::Client<Cancel>::SharedPtr route_cancel_;
 std::map<std::string,rclcpp::Client<Trigger>::SharedPtr> triggers_;
 rclcpp::Service<Command>::SharedPtr command_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_,events_;
 rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_;
 Json battery_value_=nullptr;Clock::time_point battery_at_{};
 rclcpp::TimerBase::SharedPtr timer_;
 static std::string uuid(){std::random_device r;return std::to_string(r())+"-"+std::to_string(r())+"-"+std::to_string(r());}
 bool fresh(const std::string & name)const {const auto & o=observations_.at(name);return o.sub->get_publisher_count()==1&&!o.value.is_null()&&Clock::now()-o.at<std::chrono::seconds(3);}
 void event(const std::string & id){const auto & r=records_.at(id);std_msgs::msg::String m;
   m.data=Json({{"schema_version",1},{"boot_id",boot_},{"robot_id",robot_},{"command_id",id},
     {"operation",r.operation},{"control_owner",r.owner},{"state",r.state},{"reason_code",r.code},{"message",r.message},{"request",r.request},{"result",r.result}}).dump();events_->publish(m);RCLCPP_INFO(get_logger(),"%s",m.data.c_str());}
 void result(const std::string & id,const std::string & state,const std::string & code,const std::string & message="",Json data=Json::object()) {
   auto & r=records_.at(id);r.state=state;r.code=code;r.message=message.substr(0,4096);r.result=std::move(data);event(id);
 }
 geometry_msgs::msg::PoseStamped pose(const Json & p) {
   geometry_msgs::msg::PoseStamped out;out.header.frame_id=p.at("frame").get<std::string>();
   double x=p.at("x"),y=p.at("y"),yaw=p.at("yaw");
   if(out.header.frame_id.empty()||out.header.frame_id.front()=='/'||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw))throw std::runtime_error("Invalid pose");
   out.pose.position.x=x;out.pose.position.y=y;out.pose.orientation.z=std::sin(yaw/2);out.pose.orientation.w=std::cos(yaw/2);return out;
 }
 void cancelNav(){if(nav_handle_)nav_->async_cancel_goal(nav_handle_);}
 void cancelRoute(){if(route_id_.empty()||route_id_==last_canceled_route_||!route_cancel_->service_is_ready())return;
   auto req=std::make_shared<Cancel::Request>();req->route_id=route_id_;route_cancel_->async_send_request(req);last_canceled_route_=route_id_;}
 void pauseExplore(){if(!exploration_owned_||!fresh("exploration")||!explore_->service_is_ready())return;
   const auto & s=observations_.at("exploration").value;
   if(!s.value("can_pause",false)||s.at("revision").get<uint64_t>()==last_pause_revision_)return;
   auto req=std::make_shared<Explore::Request>();req->command_id="lease-stop-"+boot_+"-"+std::to_string(s.at("revision").get<uint64_t>());
   req->expected_boot_id=s.at("boot_id");req->expected_revision=s.at("revision");req->operation="pause";explore_->async_send_request(req);last_pause_revision_=req->expected_revision;
 }
 void stopOwned(){arm_->invalidate("ARM.CONTROL_LOST");stopping_=true;lease_.clear();owner_.clear();cancelNav();cancelRoute();pauseExplore();}
 void dispatch(const std::string & id,const std::string & operation,const Json & payload) {
   const bool movement=operation=="navigate"||operation=="start_route"||operation=="explore_resume"||operation=="arm_plan"||operation=="simulation_transport_start";
   if(movement&&require_zones_){
     if(!zonesReady())throw std::runtime_error("ZONES.NOT_READY");
     if(payload.value("zone_token",std::string())!=observations_.at("navigation_zones").value.at("token"))throw std::runtime_error("ZONES.VERSION_MISMATCH");
   }
   if(operation=="zones_replace"){
     if(zone_pending_||map_pending_||!fresh("navigation_zones")||!zones_client_->service_is_ready())throw std::runtime_error("ZONES.BUSY_OR_UNAVAILABLE");
     const auto &z=observations_.at("navigation_zones").value;
     if(!z.value("can_edit",false)||nav_outstanding_||route_pending_||exploration_pending_||!route_id_.empty()||sim_start_pending_||arm_->status().value("state","")=="PLANNING")throw std::runtime_error("ZONES.STOP_REQUIRED");
     if(payload.at("expected_zones_boot")!=z.at("boot_id")||payload.at("context_id")!=z.at("context_id")||payload.at("expected_revision")!=z.at("revision"))throw std::runtime_error("ZONES.VERSION_MISMATCH");
     auto q=std::make_shared<Command::Request>();q->schema_version=1;q->robot_id=robot_;q->command_id=id;q->expected_boot_id=z.at("boot_id");q->operation=operation;
     auto p=payload;p.erase("expected_zones_boot");q->payload_json=p.dump();zone_pending_=true;
     zones_client_->async_send_request(q,[this,id,boot=q->expected_boot_id](rclcpp::Client<Command>::SharedFuture f){auto r=f.get();zone_pending_=false;observations_.at("navigation_zones").at={};
       if(r->boot_id!=boot){result(id,"UNKNOWN","ZONES.BACKEND_RESTART");return;}
       result(id,r->accepted?"SUCCEEDED":"FAILED",r->reason_code,r->message,r->result_json.empty()?Json::object():Json::parse(r->result_json));});return;
   }
   if(operation=="simulation_transport_start"&&(nav_outstanding_||route_pending_||!route_id_.empty()||exploration_pending_||exploration_owned_))throw std::runtime_error("SIM.MOTION_ACTIVE");
   if(operation=="simulation_transport_start"&&!fresh("simulation_transport"))throw std::runtime_error("SIM.UNAVAILABLE");
   if(operation=="simulation_transport_start"&&(sim_start_pending_||observations_.at("simulation_transport").value.value("motion_blocked",true)))throw std::runtime_error("SIM.TRANSPORT_BUSY_OR_UNKNOWN");
   if(operation=="navigate"||operation=="start_route"||operation=="explore_resume"||operation=="new_mapping_session"||operation=="arm_plan"||operation=="simulation_transport_start"){
     if(map_pending_||(require_maps_&&!fresh("map_catalog")))throw std::runtime_error("MAP.CONTEXT_BLOCKED");
     if(fresh("map_catalog")){
       const auto & catalog=observations_.at("map_catalog").value;
       if(catalog.value("motion_blocked",true))throw std::runtime_error("MAP.CONTEXT_BLOCKED");
       if(!catalog.value("scene_ready",true))throw std::runtime_error("SCENE.CONTEXT_INVALID");
       const auto scene=catalog.value("active_scene",Json());
       if(scene.is_null()){if(!payload.value("scene_id",std::string()).empty()||payload.value("scene_version",0)!=0)throw std::runtime_error("SCENE.VERSION_MISMATCH");}
       else if(payload.value("scene_id",std::string())!=scene.at("scene_id")||payload.value("scene_version",0)!=scene.at("version"))throw std::runtime_error("SCENE.VERSION_MISMATCH");
     }
   }
   if(operation=="arm_execute")throw std::runtime_error("ARM.EXECUTION_ADAPTER_UNAVAILABLE");
   if(operation=="arm_plan"){
     arm_->plan(id,payload,armContext(),[this,id](std::string state,std::string code){result(id,state,code);});return;
   }
   if(operation=="arm_discard"){
     if(payload.value("plan_id","")!=arm_->status().at("plan_id"))throw std::runtime_error("ARM.PLAN_MISMATCH");
     arm_->invalidate("ARM.USER_DISCARDED");result(id,"SUCCEEDED","ARM.DISCARDED","Planning service cannot be interrupted; any late response is ignored");return;
   }
   const bool map_operation=operation=="device_put"||operation=="scene_put"||operation=="scene_load"||operation=="map_import"||operation=="station_put"||operation=="map_switch_begin"||operation=="map_transfer_confirm"||operation=="map_abort"||operation=="map_recover";
   if(map_operation){
     if(sim_start_pending_||(require_sim_transport_&&(!fresh("simulation_transport")||observations_.at("simulation_transport").value.value("motion_blocked",true))))throw std::runtime_error("SIM.TRANSPORT_BUSY_OR_UNKNOWN");
     if(zone_pending_||map_pending_||!fresh("map_catalog")||!maps_->service_is_ready())throw std::runtime_error("MAP.BUSY_OR_UNAVAILABLE");
     const auto & catalog=observations_.at("map_catalog").value;
     if(payload.at("expected_catalog_boot")!=catalog.at("boot_id")||payload.at("expected_revision")!=catalog.at("revision"))throw std::runtime_error("STATE.REVISION_MISMATCH");
     if((operation=="map_switch_begin"||operation=="device_put"||operation=="scene_put"||operation=="scene_load")&&(nav_outstanding_||route_pending_||exploration_pending_||!route_id_.empty()))throw std::runtime_error("MAP.MOTION_ACTIVE");
     if((operation=="device_put"||operation=="scene_put"||operation=="scene_load")&&arm_->status().value("state","")=="PLANNING")throw std::runtime_error("SCENE.ARM_BUSY");
     auto q=std::make_shared<Command::Request>();q->schema_version=1;q->robot_id=robot_;q->command_id=id;q->expected_boot_id=catalog.at("boot_id");q->operation=operation;
     auto p=payload;p.erase("expected_catalog_boot");q->payload_json=p.dump();map_pending_=true;
     maps_->async_send_request(q,[this,id,boot=q->expected_boot_id](rclcpp::Client<Command>::SharedFuture f){auto r=f.get();map_pending_=false;
       if(r->boot_id!=boot){result(id,"UNKNOWN","MAP.BACKEND_RESTART");return;}
       // Wait for a post-response catalog observation before allowing movement.
       observations_.at("map_catalog").at={};
       result(id,r->accepted?"SUCCEEDED":"FAILED",r->reason_code,r->message,r->result_json.empty()?Json::object():Json::parse(r->result_json));});return;
   }
   if(operation=="navigate"||operation=="start_route"||operation=="explore_resume"||operation=="new_mapping_session"){
     if(sim_start_pending_||(require_sim_transport_&&(!fresh("simulation_transport")||observations_.at("simulation_transport").value.value("motion_blocked",true))))throw std::runtime_error("SIM.TRANSPORT_BUSY_OR_UNKNOWN");
     if(map_pending_||(require_maps_&&!fresh("map_catalog"))||(fresh("map_catalog")&&observations_.at("map_catalog").value.value("motion_blocked",true)))throw std::runtime_error("MAP.CONTEXT_BLOCKED");
     if(fresh("map_catalog")&&!observations_.at("map_catalog").value.value("scene_ready",true))throw std::runtime_error("SCENE.CONTEXT_INVALID");
     if(fresh("map_catalog")&&!observations_.at("map_catalog").value.at("active_map").is_null()){
       const auto & active=observations_.at("map_catalog").value.at("active_map");
       if(operation=="new_mapping_session")throw std::runtime_error("MAP.ACTIVE_CATALOG_SESSION");
       if(payload.value("map_version","")!=active.at("version"))throw std::runtime_error("MAP.VERSION_MISMATCH");
     }
   }
   if(operation=="navigate") {
     if(nav_outstanding_)throw std::runtime_error("NAV.BUSY");
     if(!nav_->action_server_is_ready())throw std::runtime_error("CAPABILITY.UNAVAILABLE");
     Nav::Goal goal;goal.pose=pose(payload);if(require_zones_)owned_zone_token_=payload.at("zone_token");nav_id_=id;nav_outstanding_=true;
     rclcpp_action::Client<Nav>::SendGoalOptions options;
     options.goal_response_callback=[this,id](Handle::SharedPtr h){nav_handle_=h;
       if(!h){nav_outstanding_=false;result(id,"FAILED","NAV.GOAL_REJECTED");return;}
       std::ostringstream uuid;for(auto byte:h->get_goal_id())uuid<<std::hex<<std::setfill('0')<<std::setw(2)<<static_cast<unsigned>(byte);owned_navigation_[uuid.str()]=id;
       result(id,"RUNNING","NAV.ACCEPTED");if(stopping_||lease_.empty())cancelNav();};
     options.result_callback=[this,id](const Handle::WrappedResult & r){nav_outstanding_=false;nav_handle_.reset();
       if(navigation_reasons_[id]=="PREEMPTED")result(id,"PREEMPTED","NAV.PREEMPTED");
       else result(id,r.code==rclcpp_action::ResultCode::SUCCEEDED?"SUCCEEDED":r.code==rclcpp_action::ResultCode::CANCELED?"CANCELED":"FAILED",
         r.code==rclcpp_action::ResultCode::SUCCEEDED?"NAV.SUCCEEDED":r.code==rclcpp_action::ResultCode::CANCELED?"NAV.CANCELED":"NAV.ABORTED");};
     nav_->async_send_goal(goal,options);return;
   }
   if(operation=="cancel_navigation") {
     if(!nav_outstanding_||payload.value("operation_id","")!=nav_id_)throw std::runtime_error("NAV.NO_MATCHING_OWNED_GOAL");
     // Latch cancellation even when goal acceptance is delayed.
     stopping_=true;cancelNav();result(id,"SUCCEEDED","CANCEL.REQUESTED","Await navigation terminal, not physical stop");return;
   }
   if(operation=="start_route") {
     if(route_pending_||!route_id_.empty())throw std::runtime_error("ROUTE.BUSY");
     if(!fresh("route")||!route_start_->service_is_ready())throw std::runtime_error("STATE.STALE");
     auto req=std::make_shared<Start::Request>();req->request_id=id;req->expected_boot_id=observations_.at("route").value.at("boot_id");
     req->dwell_sec=payload.value("dwell_sec",0.5);for(const auto & p:payload.at("points"))req->waypoints.push_back(pose(p));
     if(req->waypoints.size()<2||req->waypoints.size()>200||!std::isfinite(req->dwell_sec)||req->dwell_sec<0||req->dwell_sec>3600)throw std::runtime_error("REQUEST.INVALID_ROUTE");
     route_pending_=true;if(require_zones_)owned_zone_token_=payload.at("zone_token");
     route_start_->async_send_request(req,[this,id](rclcpp::Client<Start>::SharedFuture f){auto r=f.get();route_pending_=false;
       if(r->accepted&&r->active)route_id_=r->route_id;
       result(id,r->accepted?"SUCCEEDED":"FAILED",r->accepted?"ROUTE.ACCEPTED":"ROUTE.REJECTED",r->reason,{{"route_id",r->route_id}});
       if(stopping_||lease_.empty())cancelRoute();});return;
   }
   if(operation=="cancel_route") {
     if(route_id_.empty()||payload.value("route_id","")!=route_id_)throw std::runtime_error("ROUTE.NO_MATCHING_OWNED_ROUTE");
     cancelRoute();result(id,"SUCCEEDED","CANCEL.REQUESTED","Await route terminal");return;
   }
   if(operation=="explore_pause"||operation=="explore_resume"||operation=="explore_cancel_save") {
     if(exploration_pending_)throw std::runtime_error("EXPLORATION.COMMAND_PENDING");
     if(!fresh("exploration")||!explore_->service_is_ready())throw std::runtime_error("STATE.STALE");
     // When mapping_runtime is present, cancel-save must belong to an owned
     // active SLAM session.  Otherwise an idle/static-map coordinator could
     // accept the command and only fail much later in mapping_session.
     if(operation=="explore_cancel_save"&&fresh("mapping_runtime")&&
       observations_.at("mapping_runtime").value.value("state","")!="RUNNING")
       throw std::runtime_error("MAP.NO_ACTIVE_MAPPING_SESSION");
     auto req=std::make_shared<Explore::Request>();const auto & s=observations_.at("exploration").value;
     req->command_id=id;req->expected_boot_id=payload.at("expected_exploration_boot");req->expected_revision=payload.at("expected_exploration_revision");req->operation=operation.substr(8);
     if(req->expected_boot_id!=s.at("boot_id")||req->expected_revision!=s.at("revision"))throw std::runtime_error("STATE.REVISION_MISMATCH");
     if(req->operation!="pause"&&(!fresh("mapping")||observations_.at("mapping").value.at("state")!="IDLE"))throw std::runtime_error("MAP.SESSION_NOT_IDLE");
     if(!s.value("can_"+req->operation,false))throw std::runtime_error("EXPLORATION.OPERATION_BLOCKED");
     const bool previously_owned=exploration_owned_;if(operation=="explore_resume"&&require_zones_)owned_zone_token_=payload.at("zone_token");
     exploration_owned_=true;exploration_pending_=true;exploration_boot_=req->expected_boot_id;exploration_revision_=req->expected_revision;
     explore_->async_send_request(req,[this,id,previously_owned](rclcpp::Client<Explore>::SharedFuture f){auto r=f.get();exploration_pending_=false;if(!r->accepted)exploration_owned_=previously_owned;
       result(id,r->accepted?"SUCCEEDED":"FAILED",r->reason_code,r->message);
       if(stopping_||lease_.empty())pauseExplore();});return;
   }
   auto trigger=triggers_.find(operation);
   if(trigger!=triggers_.end()) {
     if(!trigger->second->service_is_ready())throw std::runtime_error("CAPABILITY.UNAVAILABLE");
     if(operation=="new_mapping_session"&&(nav_outstanding_||route_pending_||!route_id_.empty()||exploration_owned_))throw std::runtime_error("MAP.MOTION_ACTIVE");
     if(operation=="retry_map"&&(!fresh("mapping")||observations_.at("mapping").value.at("state")!="FAILED"))throw std::runtime_error("MAP.NOT_FAILED");
     if(operation=="simulation_transport_start")sim_start_pending_=true;
     trigger->second->async_send_request(std::make_shared<Trigger::Request>(),[this,id,operation](rclcpp::Client<Trigger>::SharedFuture f){auto r=f.get();
       if(operation=="simulation_transport_start"){sim_start_pending_=false;observations_.at("simulation_transport").at={};}
       result(id,r->success?"SUCCEEDED":"FAILED",r->success?"COMMAND.DOWNSTREAM_ACCEPTED":"COMMAND.DOWNSTREAM_REJECTED",r->message);});return;
   }
   throw std::runtime_error("REQUEST.UNSUPPORTED");
 }
 void command(Command::Request::SharedPtr q,Command::Response::SharedPtr response) {
   arm_->tick(armContext());
   if(!lease_.empty()&&Clock::now()>=lease_until_)stopOwned();
   auto & s=*response;s.boot_id=boot_;s.operation_id=q->command_id;
   auto reject=[&](const std::string & code){s.accepted=false;s.reason_code=code;s.state="REJECTED";};
   if(q->command_id.empty()||q->command_id.size()>128||q->operation.size()>64||q->payload_json.size()>(q->operation=="zones_replace"?131072u:65536u)||q->lease_id.size()>256){reject("REQUEST.INVALID");return;}
   if(q->schema_version!=1){reject("REQUEST.SCHEMA_MISMATCH");return;}
   if(q->robot_id!=robot_){reject("REQUEST.ROBOT_MISMATCH");return;}
   if(q->expected_boot_id!=boot_){reject("REQUEST.BOOT_MISMATCH");return;}
   if(q->operation=="query") {
     auto it=records_.find(q->command_id);if(it==records_.end()){reject("REQUEST.NOT_FOUND");return;}
     s.accepted=true;s.state=it->second.state;s.reason_code=it->second.code;s.message=it->second.message;s.result_json=it->second.result.dump();return;
   }
   if(q->expected_boot_id!=boot_){reject("REQUEST.BOOT_MISMATCH");return;}
   if(q->operation=="acquire") {
     if(q->command_id.empty()||q->command_id.size()>128){reject("REQUEST.INVALID_ID");return;}
     if(!stopping_&&!lease_.empty()&&q->command_id==acquire_id_) {
       if(q->payload_json!=acquire_payload_){reject("REQUEST.CONFLICT");return;}
       s.accepted=true;s.state="HELD";s.reason_code="CONTROL.ACQUIRED";s.result_json=Json({{"lease_id",lease_},{"ttl_sec",std::chrono::duration<double>(lease_until_-Clock::now()).count()}}).dump();return;
     }
     if(stopping_||(!lease_.empty()&&Clock::now()<lease_until_)){reject("CONTROL.BUSY");return;}
     if(acquired_ids_.count(q->command_id)){reject("REQUEST.RETIRED");return;}
     if(acquired_ids_.size()>=4096){reject("REQUEST.CAPACITY");return;}
     auto payload=Json::parse(q->payload_json.empty()?"{}":q->payload_json,nullptr,false);
     if(!payload.is_object()||(payload.contains("client_name")&&!payload.at("client_name").is_string())){reject("REQUEST.INVALID_PAYLOAD");return;}
     owner_=payload.value("client_name","operator").substr(0,64);acquire_id_=q->command_id;acquire_payload_=q->payload_json;acquired_ids_.insert(acquire_id_);
     lease_=uuid();lease_until_=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(ttl_));
     s.accepted=true;s.state="HELD";s.reason_code="CONTROL.ACQUIRED";s.result_json=Json({{"lease_id",lease_},{"ttl_sec",ttl_}}).dump();return;
   }
   if(lease_.empty()||q->lease_id!=lease_||Clock::now()>=lease_until_){reject("CONTROL.NOT_OWNER");return;}
   if(q->operation=="renew") {lease_until_=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(ttl_));s.accepted=true;s.state="HELD";s.reason_code="CONTROL.RENEWED";return;}
   if(q->operation=="release"){stopOwned();s.accepted=true;s.state="RELEASING";s.reason_code="CONTROL.STOP_REQUESTED";return;}
   if(stopping_){reject("CONTROL.STOP_UNCONFIRMED");return;}
   if(q->command_id.empty()||q->command_id.size()>128||q->payload_json.size()>(q->operation=="zones_replace"?131072u:65536u)){reject("REQUEST.INVALID");return;}
   const auto signature=Json({q->operation,q->payload_json,q->lease_id}).dump();
   auto old=records_.find(q->command_id);
   if(old!=records_.end()) {
     if(old->second.fingerprint!=signature){reject("REQUEST.CONFLICT");return;}
   } else {
     if(records_.size()>=10000||records_bytes_+signature.size()+512>16*1024*1024){reject("REQUEST.CAPACITY");return;}
     Record record;record.operation=q->operation;record.owner=owner_;record.fingerprint=signature;records_bytes_+=signature.size()+512;record.deadline=Clock::now()+std::chrono::seconds(q->operation=="navigate"?300:q->operation=="arm_plan"?25:5);
     auto request_json=Json::parse(q->payload_json.empty()?"{}":q->payload_json,nullptr,false);
     if(request_json.is_object())for(const auto * key:{"frame","x","y","yaw","points","dwell_sec","operation_id","route_id","expected_exploration_boot","expected_exploration_revision","expected_catalog_boot","expected_revision","map_id","map_version","station_id","transaction_id","group","named_target","plan_id","scene_id","scene_version","device_id","expected_device_version","expected_scene_version","zone_token","expected_zones_boot","context_id","regions","reviewed"})if(request_json.contains(key))record.request[key]=request_json.at(key);
     records_.emplace(q->command_id,record);event(q->command_id);
     try {dispatch(q->command_id,q->operation,q->payload_json.empty()?Json::object():Json::parse(q->payload_json));}
     catch(const std::exception & e){std::string code=e.what();
       if(code.empty()||!std::all_of(code.begin(),code.end(),[](char c){return (c>='A'&&c<='Z')||c=='.'||c=='_';}))code="REQUEST.INVALID_PAYLOAD";
       result(q->command_id,"FAILED",code,e.what());}
   }
   const auto & r=records_.at(q->command_id);s.accepted=r.state!="FAILED";s.state=r.state;s.reason_code=r.code;s.message=r.message;s.result_json=r.result.dump();
 }
 void tick() {
   if(require_zones_&&!owned_zone_token_.empty()&&(!zonesReady()||observations_.at("navigation_zones").value.value("token",std::string())!=owned_zone_token_)&&(nav_outstanding_||route_pending_||exploration_owned_||!route_id_.empty())){stopping_=true;cancelNav();cancelRoute();pauseExplore();}
   arm_->tick(armContext());
   if(!lease_.empty()&&Clock::now()>=lease_until_)stopOwned();
   if(stopping_){cancelRoute();pauseExplore();}
   if(!route_id_.empty()&&fresh("route")) {
     const auto & r=observations_.at("route").value;
     if(r.value("route_id","")==route_id_&&!r.value("active",true))route_id_.clear();
   }
   if(exploration_owned_&&!exploration_pending_&&fresh("exploration")) {
     const auto & e=observations_.at("exploration").value;
     if(e.value("boot_id","")==exploration_boot_&&e.value("revision",uint64_t(0))>exploration_revision_&&(e.value("manual_pause",false)||e.value("session_ending",false))&&!e.value("goal_in_flight",true))exploration_owned_=false;
   }
   if(stopping_&&!nav_outstanding_&&!route_pending_&&route_id_.empty()&&!exploration_owned_)stopping_=false;
   for(auto & [id,r]:records_)if((r.state=="ACCEPTED"||r.state=="RUNNING")&&Clock::now()>r.deadline){
     result(id,"UNKNOWN","COMMAND.TIMEOUT","No terminal confirmation; do not repeat with a new ID");
     if(id==nav_id_&&nav_outstanding_){stopping_=true;cancelNav();}
   }
   Json snapshot={{"schema_version",1},{"robot_id",robot_},{"boot_id",boot_},{"control_owner",owner_},
     {"control_state",stopping_?"STOP_UNCONFIRMED":lease_.empty()?"OBSERVER":"HELD"},
     {"control_session",lease_.empty()?"":acquire_id_},{"nav_outstanding",nav_outstanding_},{"nav_operation_id",nav_id_},{"route_id",route_id_},{"battery",{{"quality",Clock::now()-battery_at_<std::chrono::seconds(3)?"VALID":"UNKNOWN"},{"value",battery_value_}}}};
   if(!nav_id_.empty()&&records_.count(nav_id_))snapshot["navigation"]={{"operation_id",nav_id_},{"state",records_.at(nav_id_).state},{"reason_code",records_.at(nav_id_).code}};
   for(const auto & [name,o]:observations_)snapshot[name]={{"quality",fresh(name)?"VALID":o.sub->get_publisher_count()>1?"CONFLICT":"STALE"},{"value",o.value}};
   snapshot["arm_preview"]=arm_->status();
   snapshot["scene_resource_busy"]=sim_start_pending_||nav_outstanding_||route_pending_||exploration_pending_||!route_id_.empty()||arm_->status().value("state","")=="PLANNING"||(require_sim_transport_&&(!fresh("simulation_transport")||observations_.at("simulation_transport").value.value("motion_blocked",true)));
   snapshot["capabilities"]={{"map_catalog",fresh("map_catalog")&&maps_->service_is_ready()},{"navigate",nav_->action_server_is_ready()},{"loop_route",route_start_->service_is_ready()},
     {"exploration",fresh("exploration")&&explore_->service_is_ready()},{"new_mapping_session",fresh("mapping_runtime")&&observations_.at("mapping_runtime").value.value("can_start",false)&&triggers_.at("new_mapping_session")->service_is_ready()},
     {"recording",triggers_.at("record_start")->service_is_ready()}};
   std_msgs::msg::String message;message.data=snapshot.dump();status_->publish(message);
 }
public:
 explicit OperatorBackend(const rclcpp::NodeOptions & options=rclcpp::NodeOptions()):Node("operator_backend",options) {
   arm_=std::make_unique<ArmPreview>(*this);
   require_zones_=declare_parameter("require_navigation_zones",false);
   require_maps_=declare_parameter("require_map_manager",false);
   robot_=declare_parameter("robot_id","astribot");ttl_=declare_parameter("lease_ttl_sec",6.0);
   if(robot_.empty()||!std::isfinite(ttl_)||ttl_<1||ttl_>60)throw std::runtime_error("Invalid identity or TTL");
   boot_=uuid();
   status_=create_publisher<std_msgs::msg::String>("~/status",rclcpp::QoS(1).transient_local());events_=create_publisher<std_msgs::msg::String>("~/events",100);
   for(const auto & item:std::map<std::string,std::string>{{"navigation_zones","/navigation_zones/status"},{"simulation_transport","/simulation_transport/status"},{"transport","/transport/status"},{"map_catalog","/map_manager/status"},{"exploration","/exploration_coordinator_node/operator_status"},{"mapping","/mapping_session/status"},{"route","/loop_route_executor/status"},{"recording","/diagnostics_recorder/status"},{"mapping_runtime","/mapping_runtime/status"}}) {
     observations_[item.first].sub=create_subscription<std_msgs::msg::String>(item.second,item.first=="transport"?rclcpp::QoS(10):rclcpp::QoS(1).transient_local(),[this,key=item.first](std_msgs::msg::String::ConstSharedPtr m){
       try {if(m->data.size()>(key=="map_catalog"?524288u:key=="navigation_zones"?131072u:65536u))throw std::runtime_error("Oversized status");auto value=Json::parse(m->data);if(!value.is_object())return;if(key=="map_catalog")require_maps_=true;if(key=="navigation_zones")require_zones_=true;if(key=="simulation_transport")require_sim_transport_=true;observations_.at(key).value=std::move(value);observations_.at(key).at=Clock::now();}catch(...) {observations_.at(key).at={};}
     });
   }
   navigation_status_=create_subscription<astribot_navigation_msgs::msg::NavigationExecutionStatus>("/navigation/execution_status",rclcpp::QoS(10).transient_local(),
     [this](astribot_navigation_msgs::msg::NavigationExecutionStatus::ConstSharedPtr m){
       if(m->source!="operator"||m->state!="PREEMPTED")return;
       auto owned=owned_navigation_.find(m->task_id);if(owned==owned_navigation_.end())return;
       navigation_reasons_[owned->second]="PREEMPTED";
       if(records_.at(owned->second).state=="CANCELED"||records_.at(owned->second).state=="FAILED")result(owned->second,"PREEMPTED","NAV.PREEMPTED",m->reason);
     });
   battery_=create_subscription<sensor_msgs::msg::BatteryState>("/battery_state",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::BatteryState::ConstSharedPtr b){
     if(battery_->get_publisher_count()!=1||!b->present||!std::isfinite(b->percentage)||b->percentage<0||b->percentage>1){battery_at_={};return;}
     battery_value_=b->percentage;battery_at_=Clock::now();});
   zones_client_=create_client<Command>("/navigation_zones/command");
   maps_=create_client<Command>("/map_manager/command");
   const auto navigation_action = declare_parameter<std::string>("navigation_action", "/navigate_to_pose");
   nav_=rclcpp_action::create_client<Nav>(this,navigation_action);explore_=create_client<Explore>("/exploration_coordinator_node/command");
   route_start_=create_client<Start>("/loop_route_executor/start");route_cancel_=create_client<Cancel>("/loop_route_executor/cancel");
   for(const auto & item:std::map<std::string,std::string>{{"simulation_transport_start","/simulation_transport/start"},{"simulation_transport_cancel","/simulation_transport/cancel"},{"record_start","/diagnostics_recorder/start"},{"record_stop","/diagnostics_recorder/stop"},{"record_mark","/diagnostics_recorder/mark"},{"record_snapshot","/diagnostics_recorder/snapshot"},{"retry_map","/mapping_session/retry"},{"new_mapping_session","/mapping_runtime/start"}})triggers_[item.first]=create_client<Trigger>(item.second);
   command_=create_service<Command>("~/command",[this](Command::Request::SharedPtr q,Command::Response::SharedPtr r){command(q,r);});
   timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{try{tick();}catch(const std::exception & e){RCLCPP_ERROR(get_logger(),"status: %s",e.what());}});
 }
};
#ifndef ASTRIBOT_OPERATOR_BACKEND_NO_MAIN
int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<OperatorBackend>());rclcpp::shutdown();}
#endif
