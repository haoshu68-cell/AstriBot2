#include "astribot_map_manager/catalog.hpp"
#include "astribot_map_manager/storage_path.hpp"
#include <rclcpp/rclcpp.hpp>
#include <astribot_operator_msgs/srv/operator_command.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_map_server/map_io.hpp>
#include <spdlog/spdlog.h>
#include <random>
#include <algorithm>
namespace astribot_map_manager {
class MapManager:public rclcpp::Node {
 using Command=astribot_operator_msgs::srv::OperatorCommand;using Clock=std::chrono::steady_clock;
 struct Observation {Json data;Clock::time_point at{};rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;};
 std::unique_ptr<Catalog> catalog_;std::string robot_,boot_,adapter_boot_,attempt_,storage_root_;bool accepted_{false};double deadline_sec_;
 Clock::time_point deadline_{},odom_at_{},still_since_{},ready_since_{};int samples_{0};int64_t last_stamp_{0};
 std::map<std::string,Observation> observed_;rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
 rclcpp::Client<Command>::SharedPtr adapter_;rclcpp::Service<Command>::SharedPtr command_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_,events_;rclcpp::TimerBase::SharedPtr timer_;
 Json static_map_;nav_msgs::msg::OccupancyGrid expected_static_grid_;
 rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_sub_;
 bool static_grid_verified_{false};
 bool staticReady()const{return static_map_sub_&&static_grid_verified_&&static_map_sub_->get_publisher_count()==1;}
 void configureStaticSimulationMap(const std::string & yaml){
  if(yaml.empty())return;
  if(!get_parameter("use_sim_time").as_bool())throw std::runtime_error("MAP.STATIC_CONTEXT_REQUIRES_SIMULATION");
  if(!catalog_->snapshot().at("active_map").is_null()||Catalog::blocked(catalog_->snapshot().at("transaction")))
    throw std::runtime_error("MAP.STATIC_CONTEXT_CONFLICT");
  const auto file=fs::canonical(yaml);
  const auto parameters=nav2_map_server::loadMapYaml(file.string());
  if(nav2_map_server::loadMapFromYaml(file.string(),expected_static_grid_)!=nav2_map_server::LOAD_MAP_SUCCESS)
    throw std::runtime_error("MAP.STATIC_CONTEXT_LOAD_FAILED");
  static_map_={{"map_id","simulation_static_map"},{"version",Catalog::hash(file)+"-"+Catalog::hash(parameters.image_file_name)},
    {"source_directory",file.parent_path().string()},{"map_yaml",file.string()},{"read_only",true},{"source","simulation_static_map"}};
  static_map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid){
      const auto & expected=expected_static_grid_.info;
      static_grid_verified_=grid->header.frame_id=="map"&&grid->info.width==expected.width&&
        grid->info.height==expected.height&&grid->info.resolution==expected.resolution&&
        grid->info.origin==expected.origin&&grid->data==expected_static_grid_.data;
    });
 }
 bool fresh(const std::string & key)const{const auto & o=observed_.at(key);return !o.data.is_null()&&o.sub->get_publisher_count()==1&&Clock::now()-o.at<std::chrono::seconds(2);}
 bool stopped()const{return samples_>=3&&odom_->get_publisher_count()==1&&Clock::now()-odom_at_<std::chrono::milliseconds(500)&&Clock::now()-still_since_>=std::chrono::seconds(1);}
 bool idle()const{
 if(!stopped()||!fresh("operator")||!fresh("route")||!fresh("exploration"))return false;
 const auto & b=observed_.at("operator").data;const auto & e=observed_.at("exploration").data;
 return !b.value("nav_outstanding",true)&&b.value("control_state","")=="HELD"&&!b.value("scene_resource_busy",false)&&!observed_.at("route").data.value("active",true)&&!e.value("goal_in_flight",true)&&(e.value("manual_pause",false)||e.value("session_ending",false));
 }
 bool capable()const{return static_map_.is_null()&&fresh("adapter")&&adapter_->service_is_ready()&&observed_.at("adapter").data.value("supports_voxel_sessions",false);}
 bool activeReady()const{
 const auto & active=catalog_->snapshot().at("active_map");if(active.is_null())return static_map_.is_null()||staticReady();
 if(!fresh("adapter"))return false;
 const auto & a=observed_.at("adapter").data;
 if(a.value("state","")!="READY"||a.value("map_id","")!=active.at("map_id")||a.value("map_version","")!=active.at("version"))return false;
 for(auto key:{"localization_ready","tf_ready","map_ready","global_costmap_ready","local_costmap_ready"})if(!a.value(key,false))return false;
 return true;
 }
 void publish(){auto s=catalog_->snapshot();s.erase("commands");s.erase("history");s["storage_root"]=storage_root_;
 if(static_map_sub_&&static_map_sub_->get_publisher_count()!=1)static_grid_verified_=false;
 if(!static_map_.is_null()&&staticReady())s["active_map"]=static_map_;
 for(auto & map:s["maps"])map.erase("sha256");
 for(auto key:{"devices","scenes"})for(auto & versions:s[key]){auto latest=versions.back();versions=std::move(latest);}
 s["scene_ready"]=static_map_.is_null()&&catalog_->sceneReady()&&activeReady();
 for(auto & station:s["stations"]){auto latest=station.back();station=std::move(latest);}
 if(!s["active_map"].is_null())s["active_map"].erase("sha256");
 if(!s["transaction"].is_null()){s["transaction"]["target"].erase("sha256");if(!s["transaction"]["previous"].is_null())s["transaction"]["previous"].erase("sha256");}s["boot_id"]=boot_;s["robot_id"]=robot_;s["state"]=Catalog::blocked(s.at("transaction"))?s.at("transaction").value("state",""):"READY";
 s["motion_blocked"]=Catalog::blocked(s.at("transaction"))||!activeReady();s["active_map_ready"]=activeReady();s["switch_available"]=capable();s["stop_evidence"]=stopped();s["idle_evidence"]=idle();
 // Catalog listings remain bounded separately from the immutable disk history.
 std_msgs::msg::String m;m.data=s.dump();status_->publish(m);}
 static Json compact(Json value){
 if(value.is_object()){value.erase("sha256");for(auto key:{"target","previous"})if(value.contains(key))value[key]=compact(value[key]);}return value;
 }
 void event(const std::string & id,const std::string & op,const std::string & state,const std::string & code,Json context=Json::object()){std_msgs::msg::String m;m.data=Json({{"schema_version",1},{"boot_id",boot_},{"command_id",id},{"operation",op},{"state",state},{"reason_code",code},{"revision",catalog_->snapshot().at("revision")},{"transaction",compact(catalog_->snapshot().at("transaction"))},{"result",compact(context)}}).dump();events_->publish(m);spdlog::info("{}",m.data);}
 void recover(const std::string & reason){const auto & t=catalog_->snapshot().at("transaction");if(Catalog::blocked(t)&&t.at("state")!="RECOVERY_REQUIRED"){auto id=t.at("transaction_id").get<std::string>();catalog_->transition(id,"RECOVERY_REQUIRED",reason);event(id,"map_switch","RECOVERY_REQUIRED",reason);}accepted_=false;}
 void load(){
 const auto t=catalog_->snapshot().at("transaction");if(!idle()||!capable()){recover("MAP.PREREQUISITE_LOST");return;}
 auto target=catalog_->verifiedMap(t.at("target").at("map_id"));auto id=t.at("transaction_id").get<std::string>();
 adapter_boot_=observed_.at("adapter").data.at("boot_id");attempt_=boot_+"-"+std::to_string(catalog_->snapshot().at("revision").get<uint64_t>());
 auto q=std::make_shared<Command::Request>();q->schema_version=1;q->robot_id=robot_;q->command_id=attempt_;q->expected_boot_id=adapter_boot_;q->operation="load_session";
 q->payload_json=Json({{"transaction_id",id},{"target",target},{"attempt_id",attempt_}}).dump();
 catalog_->transition(id,"LOADING","MAP.LOAD_INTENT_DURABLE");accepted_=false;ready_since_={};deadline_=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(deadline_sec_));
 adapter_->async_send_request(q,[this,attempt=attempt_](rclcpp::Client<Command>::SharedFuture f){if(attempt!=attempt_||catalog_->snapshot().at("transaction").at("state")!="LOADING")return;
 auto r=f.get();if(!r->accepted||r->boot_id!=adapter_boot_){recover("MAP.ADAPTER_REJECTED");return;}accepted_=true;});event(id,"map_switch","LOADING","MAP.LOAD_REQUESTED");
 }
 void tick(){
 const auto t=catalog_->snapshot().at("transaction");if(!t.is_null()&&t.at("state")=="LOAD_INTENT")load();
 else if(!t.is_null()&&t.at("state")=="LOADING"){
 if(!idle()||(t.value("manual_transfer",false)&&(!fresh("adapter")||!observed_.at("adapter").data.value("cargo_known",false)||!observed_.at("adapter").data.value("transport_ready",false)))){recover("MAP.PREREQUISITE_LOST");}
 else if(Clock::now()>deadline_){recover("MAP.VERIFICATION_TIMEOUT");}
 else if(fresh("adapter")&&observed_.at("adapter").data.value("boot_id","")!=adapter_boot_){recover("MAP.ADAPTER_RESTART");}
 else {
 bool ready=false;if(accepted_&&fresh("adapter")){const auto & a=observed_.at("adapter").data;
 ready=a.value("attempt_id","")==attempt_&&a.value("transaction_id","")==t.at("transaction_id")&&a.value("map_id","")==t.at("target").at("map_id")&&a.value("map_version","")==t.at("target").at("version")&&a.value("state","")=="READY";
 for(auto key:{"localization_ready","tf_ready","map_ready","global_costmap_ready","local_costmap_ready"})ready=ready&&a.value(key,false);
 }
 if(!ready)ready_since_={};else if(ready_since_==Clock::time_point{})ready_since_=Clock::now();else if(Clock::now()-ready_since_>=std::chrono::seconds(1)){
 catalog_->verifiedMap(t.at("target").at("map_id"));catalog_->transition(t.at("transaction_id"),"COMMITTED","MAP.VERIFIED");event(t.at("transaction_id"),"map_switch","COMMITTED","MAP.VERIFIED");}
 }
 }publish();
 }
public:
 explicit MapManager(const rclcpp::NodeOptions & options=rclcpp::NodeOptions()):Node("map_manager",options){
 robot_=declare_parameter("robot_id","astribot");std::random_device random;boot_=std::to_string(random())+"-"+std::to_string(random());
 deadline_sec_=declare_parameter("verification_timeout_sec",30.0);if(!std::isfinite(deadline_sec_)||deadline_sec_<1||deadline_sec_>300)throw std::runtime_error("Invalid verification timeout");
 storage_root_=catalogStorageRoot(declare_parameter("storage_root",std::string()));
 catalog_=std::make_unique<Catalog>(storage_root_,declare_parameter("import_root","/tmp/astribot_slam_sessions"));
 configureStaticSimulationMap(declare_parameter("simulation_static_map_yaml",std::string()));
 status_=create_publisher<std_msgs::msg::String>("~/status",rclcpp::QoS(1).transient_local());events_=create_publisher<std_msgs::msg::String>("~/events",100);
 adapter_=create_client<Command>(declare_parameter("adapter_service","/map_session_adapter/command"));
 for(const auto & item:std::map<std::string,std::string>{{"operator","/operator_backend/status"},{"route","/loop_route_executor/status"},{"exploration","/exploration_coordinator_node/operator_status"},{"adapter",declare_parameter("adapter_status","/map_session_adapter/status")}}){
 observed_[item.first].sub=create_subscription<std_msgs::msg::String>(item.second,rclcpp::QoS(1).transient_local(),[this,key=item.first](std_msgs::msg::String::ConstSharedPtr m){try{if(m->data.size()>65536)throw std::runtime_error("too large");auto j=Json::parse(m->data);if(!j.is_object())throw std::runtime_error("invalid");observed_[key].data=j;observed_[key].at=Clock::now();}catch(...){observed_[key].at={};}});}
 odom_=create_subscription<nav_msgs::msg::Odometry>(declare_parameter("odom_topic","/odom"),rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){auto at=Clock::now();auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();const auto & v=m->twist.twist;double age=(now()-rclcpp::Time(m->header.stamp)).seconds();
 bool valid=stamp>last_stamp_&&age>=0&&age<.5&&std::isfinite(v.linear.x)&&std::isfinite(v.linear.y)&&std::isfinite(v.angular.z)&&std::hypot(v.linear.x,v.linear.y)<.01&&std::abs(v.angular.z)<.01;
 if(!valid||samples_==0||at-odom_at_>std::chrono::milliseconds(500)){samples_=0;still_since_=at;}if(valid)++samples_;last_stamp_=stamp;odom_at_=at;});
 command_=create_service<Command>("~/command",[this](Command::Request::SharedPtr q,Command::Response::SharedPtr r){r->boot_id=boot_;r->operation_id=q->command_id;try{
 if(q->schema_version!=1||q->expected_boot_id!=boot_||q->robot_id!=robot_)throw std::runtime_error("REQUEST.CONTEXT_MISMATCH");
 if(q->payload_json.size()>65536)throw std::runtime_error("REQUEST.TOO_LARGE");
 auto p=Json::parse(q->payload_json);
 const bool scene_edit=q->operation=="device_put"||q->operation=="scene_put";
 const bool scene_load=q->operation=="scene_load";
 if(!fresh("operator")||observed_.at("operator").data.value("control_state","")!="HELD")throw std::runtime_error("CONTROL.NOT_OWNER");
 auto actor=observed_.at("operator").data.value("control_owner","");
 bool duplicate=catalog_->snapshot().at("commands").contains(q->command_id);
 if(!duplicate&&(scene_edit||scene_load)){
 if(!idle())throw std::runtime_error("SCENE.MOTION_OR_STATE_UNCONFIRMED");
 if(scene_load){const auto & scene=catalog_->snapshot().at("scenes").at(p.at("scene_id").get<std::string>()).back();
 const auto & active=catalog_->snapshot().at("active_map");
 p["reuse_current_map"]=!p.value("manual_transfer",false)&&!active.is_null()&&active.at("map_id")==scene.at("map_id")&&active.at("version")==scene.at("map_version")&&activeReady();
 if(!p.at("reuse_current_map").get<bool>()&&!capable())throw std::runtime_error("MAP.ADAPTER_UNAVAILABLE");
 if(p.value("manual_transfer",false)){const auto & a=observed_.at("adapter").data;if(!a.value("transport_ready",false)||!a.value("cargo_known",false)||!a.value("handover_ready",false))throw std::runtime_error("CARGO.TRANSPORT_UNCONFIRMED");}
 }}
 if(duplicate&&scene_load){p["reuse_current_map"]=Json::parse(catalog_->snapshot().at("commands").at(q->command_id).at("signature").get<std::string>()).at(1).value("reuse_current_map",false);}
 if(!duplicate&&(q->operation=="map_switch_begin"||q->operation=="map_transfer_confirm"||q->operation=="map_recover")){
 if(!idle())throw std::runtime_error("MAP.MOTION_OR_STATE_UNCONFIRMED");
 if(!capable())throw std::runtime_error("MAP.ADAPTER_UNAVAILABLE");
 if((q->operation=="map_switch_begin"&&p.value("manual_transfer",false))||q->operation=="map_transfer_confirm"||(q->operation=="map_recover"&&catalog_->snapshot().at("transaction").value("manual_transfer",false))){
 const auto & a=observed_.at("adapter").data;if(!a.value("transport_ready",false)||!a.value("cargo_known",false)||!a.value("handover_ready",false))throw std::runtime_error("CARGO.TRANSPORT_UNCONFIRMED");}}
 if(!duplicate&&q->operation=="map_abort"){
 const auto & active=catalog_->snapshot().at("active_map");
 if(!idle()||!capable()||active.is_null()||!observed_.at("adapter").data.value("localization_ready",false)||observed_.at("adapter").data.value("map_version","")!=active.at("version"))
   throw std::runtime_error("MAP.SOURCE_LOCALIZATION_UNCONFIRMED");
 }
 auto answer=catalog_->command(q->command_id,q->operation,p,actor);r->accepted=true;r->state="SUCCEEDED";r->reason_code="MAP.COMMAND_RECORDED";r->result_json=answer.dump();event(q->command_id,q->operation,r->state,r->reason_code,answer);publish();
 }catch(const std::exception & e){r->accepted=false;r->state="REJECTED";r->reason_code=e.what();r->message=e.what();
 if(r->reason_code.empty()||!std::all_of(r->reason_code.begin(),r->reason_code.end(),[](char c){return (c>='A'&&c<='Z')||c=='.'||c=='_';}))r->reason_code="MAP.INVALID_REQUEST_OR_ASSET";
 if(r->reason_code.rfind("STORE.",0)==0&&r->reason_code!="STORE.CAPACITY")rclcpp::shutdown();}});
 timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{try{tick();}catch(const std::exception & e){spdlog::error("Map transaction: {}",e.what());try{recover("MAP.INTERNAL_ERROR");publish();}catch(...){rclcpp::shutdown();}}});
 }
};
}
#ifndef ASTRIBOT_MAP_MANAGER_NO_MAIN
int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<astribot_map_manager::MapManager>());rclcpp::shutdown();}
#endif
