#include "astribot_navigation_zones/store.hpp"
#include "astribot_navigation_zones/client.hpp"
#include <astribot_operator_msgs/srv/operator_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <sstream>
#include <fstream>
#include <set>
namespace astribot_navigation_zones {
class ZoneServer:public rclcpp::Node {
 using Command=astribot_operator_msgs::srv::OperatorCommand;
 struct Observation{Json data;double at{-1e30};rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;};
 struct Ack{Json data;double at;};
 std::map<std::string,Observation> observed_;std::map<std::string,std::map<std::string,Ack>> acks_;
 std::unique_ptr<Store> store_;std::string root_,configured_root_,boot_,robot_,context_,token_,fault_;bool context_valid_{false},context_ready_{false},ready_{false},editable_{false};uint64_t sequence_{0};
 double odom_at_{-1e30},still_since_{-1e30},odom_stamp_{0};int still_samples_{0};std::vector<std::string> consumers_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr constraints_,status_,events_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ack_sub_;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;rclcpp::Service<Command>::SharedPtr service_;rclcpp::TimerBase::SharedPtr timer_;
 bool fresh(const std::string&key)const{const auto&o=observed_.at(key);return o.sub->get_publisher_count()==1&&steadySeconds()-o.at<2.5&&o.data.is_object();}
 bool idle()const{
 if(!fresh("operator")||odom_->get_publisher_count()!=1||steadySeconds()-odom_at_>.5||still_samples_<3||steadySeconds()-still_since_<1)return false;
 const auto &o=observed_.at("operator").data;if(o.value("control_state","")!="HELD"||o.value("nav_outstanding",true)||o.value("scene_resource_busy",false)||!o.value("route_id",std::string()).empty())return false;
 if(observed_.at("exploration").sub->get_publisher_count()>0){if(!fresh("exploration"))return false;auto e=observed_.at("exploration").data;if(e.value("goal_in_flight",true)||!(e.value("manual_pause",false)||e.value("session_ending",false)))return false;}
 if(observed_.at("route").sub->get_publisher_count()>0&&(!fresh("route")||observed_.at("route").data.value("active",true)))return false;
 return true;
 }
 void selectContext(){
 context_valid_=context_ready_=editable_=false;context_.clear();
 if(!fresh("map"))return;const auto &map=observed_.at("map").data;
 const auto expected_root=(std::filesystem::path(map.value("storage_root",defaultStore().parent_path().string()))/"navigation_zones").string();
 if(!store_){root_=configured_root_.empty()?expected_root:configured_root_;store_=std::make_unique<Store>(root_);}
 else if(configured_root_.empty()&&root_!=expected_root){fault_="ZONES.STORAGE_CONTEXT_CHANGED";return;}
 Json selected=map.value("active_map",Json());auto tx=map.value("transaction",Json());
 const std::string stage=tx.is_null()?"":tx.value("state","");
 if(stage=="LOADING"||stage=="LOAD_INTENT"||stage=="RECOVERY_REQUIRED")selected=tx.at("target");
 if(!selected.is_null()){
 context_="map:"+selected.at("map_id").get<std::string>()+":"+selected.at("version").get<std::string>();
 Json archived;
 if(!store_->data().at("contexts").contains(context_)&&selected.contains("directory")){
   auto path=std::filesystem::path(selected.at("directory").get<std::string>())/"manifest.json";
   if(std::filesystem::exists(path)){if(std::filesystem::file_size(path)>16*1024*1024)throw std::runtime_error("ZONES.INVALID_ARCHIVE");std::ifstream f(path);Json manifest;f>>manifest;archived=manifest.value("navigation_zones",Json());}
 }
 store_->ensure(context_,"",selected.value("source_directory",std::string()),archived);context_valid_=true;context_ready_=!map.value("motion_blocked",true)&&map.value("active_map_ready",false);
 editable_=stage.empty()||stage=="COMMITTED"||stage=="ABORTED";
 }else if(fresh("mapping")){
 const auto &m=observed_.at("mapping").data;auto id=m.value("session_id",std::string());if(id.empty())return;
 context_="mapping:"+id;store_->ensure(context_,m.value("directory",std::string()));
 context_valid_=m.value("state","")!="FAILED";context_ready_=context_valid_&&(m.value("state","")=="IDLE"||m.value("state","")=="SAVED");editable_=context_ready_&&m.value("state","")=="IDLE";
 }
 }
 void publish(){
 try{selectContext();}catch(const std::exception&e){fault_=e.what();context_valid_=false;}
 const auto c=store_?store_->context(context_):Json{{"revision",0},{"regions",Json::array()},{"review_required",false}};
 token_=boot_+":"+context_+":"+c.at("revision").dump();std_msgs::msg::String m;
 m.data=Json({{"schema_version",1},{"frame","map"},{"boot_id",boot_},{"context_id",context_},{"revision",c.at("revision")},{"regions",c.at("regions")},{"valid",context_valid_&&fault_.empty()},{"stamp",now().seconds()},{"sequence",++sequence_}}).dump();constraints_->publish(m);
 ready_=context_valid_&&context_ready_&&fault_.empty()&&!c.value("review_required",false);Json applied=Json::object();
 for(const auto&consumer:consumers_){bool ok=false;int alive=0;for(const auto &[source,ack]:acks_[consumer])if(steadySeconds()-ack.at<3.){++alive;ok=ack.data.value("applied",false)&&ack.data.value("token",std::string())==token_;}ok=ok&&alive==1;applied[consumer]=ok;ready_=ready_&&ok;}
 const auto reason=!fault_.empty()?fault_:!context_valid_?"ZONES.NO_CONTEXT":c.value("review_required",false)?"ZONES.REVIEW_REQUIRED":!context_ready_?"ZONES.MAP_NOT_READY":!ready_?"ZONES.WAIT_CONSUMERS":"ZONES.READY";
 Json s={{"schema_version",1},{"robot_id",robot_},{"boot_id",boot_},{"context_id",context_},{"token",token_},{"revision",c.at("revision")},{"regions",c.at("regions")},{"review_required",c.value("review_required",false)},{"source_context",c.value("source_context",std::string())},{"ready",ready_},{"can_edit",editable_&&fault_.empty()&&idle()},{"reason_code",reason},{"applied",applied},{"storage_root",root_},{"stamp",now().seconds()}};
 m.data=s.dump();status_->publish(m);
 }
public:
 explicit ZoneServer(const rclcpp::NodeOptions &options=rclcpp::NodeOptions()):Node("navigation_zones",options){
 robot_=declare_parameter("robot_id",std::string("astribot"));configured_root_=declare_parameter("storage_root",std::string());
 consumers_=declare_parameter<std::vector<std::string>>("required_consumers",{"global_costmap","local_costmap"});
 if(consumers_.empty()||std::set<std::string>(consumers_.begin(),consumers_.end()).size()!=consumers_.size())throw std::runtime_error("ZONES.INVALID_CONSUMERS");
 std::random_device rd;boot_=std::to_string(rd())+"-"+std::to_string(rd());
 constraints_=create_publisher<std_msgs::msg::String>("/navigation_zones/constraints",rclcpp::QoS(1).transient_local());status_=create_publisher<std_msgs::msg::String>("/navigation_zones/status",rclcpp::QoS(1).transient_local());events_=create_publisher<std_msgs::msg::String>("/navigation_zones/events",10);
 for(const auto &[key,topic]:std::map<std::string,std::string>{{"map","/map_manager/status"},{"mapping","/mapping_session/status"},{"operator","/operator_backend/status"},{"route","/loop_route_executor/status"},{"exploration","/exploration_coordinator_node/operator_status"}}){observed_[key].sub=create_subscription<std_msgs::msg::String>(topic,rclcpp::QoS(1).transient_local(),[this,key](std_msgs::msg::String::ConstSharedPtr m){try{if(m->data.size()>1048576)throw std::runtime_error("oversized status");auto value=Json::parse(m->data);if(!value.is_object())throw std::runtime_error("invalid status");observed_[key].data=std::move(value);observed_[key].at=steadySeconds();}catch(...){observed_[key].at=-1e30;}});}
 ack_sub_=create_subscription<std_msgs::msg::String>("/navigation_zones/applied",10,[this](std_msgs::msg::String::ConstSharedPtr m,const rclcpp::MessageInfo&info){try{if(m->data.size()>4096)return;auto value=Json::parse(m->data);std::string consumer=value.at("consumer");if(std::find(consumers_.begin(),consumers_.end(),consumer)==consumers_.end())return;std::ostringstream id;for(auto byte:info.get_rmw_message_info().publisher_gid.data)id<<int(byte)<<'.';auto&sources=acks_[consumer];for(auto it=sources.begin();it!=sources.end();)if(steadySeconds()-it->second.at>3.)it=sources.erase(it);else++it;if(sources.size()<8)sources[id.str()]={value,steadySeconds()};}catch(...){} });
 odom_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){auto stamp=rclcpp::Time(m->header.stamp).seconds(),t=steadySeconds();const auto&v=m->twist.twist;double age=now().seconds()-stamp;bool still=stamp>odom_stamp_&&age>=-.1&&age<.5&&std::isfinite(v.linear.x)&&std::isfinite(v.linear.y)&&std::isfinite(v.angular.z)&&std::hypot(v.linear.x,v.linear.y)<.01&&std::abs(v.angular.z)<.02;if(!still){still_samples_=0;still_since_=t;return;}if(still_samples_==0||t-odom_at_>.5){still_since_=t;still_samples_=0;}odom_stamp_=stamp;odom_at_=t;++still_samples_;});
 service_=create_service<Command>("/navigation_zones/command",[this](Command::Request::SharedPtr q,Command::Response::SharedPtr r){r->boot_id=boot_;try{
 if(q->schema_version!=1||q->robot_id!=robot_||q->expected_boot_id!=boot_||q->operation!="zones_replace"||q->payload_json.size()>131072)throw std::runtime_error("ZONES.REQUEST_CONTEXT");
 publish();auto p=Json::parse(q->payload_json);if(!store_||!context_valid_||!editable_||!fault_.empty()||p.at("context_id")!=context_)throw std::runtime_error("ZONES.CONTEXT_MISMATCH");
 if(!idle())throw std::runtime_error("ZONES.STOP_OR_AUTHORITY_REQUIRED");
 auto answer=store_->replace(q->command_id,context_,p.at("expected_revision"),p.at("regions"),observed_.at("operator").data.value("control_owner",std::string()),p.value("reviewed",false));
 r->accepted=true;r->state="SUCCEEDED";r->reason_code="ZONES.SAVED_WAIT_APPLICATION";r->result_json=answer.dump();acks_.clear();
 std_msgs::msg::String event;event.data=Json({{"command_id",q->command_id},{"context_id",context_},{"actor",observed_.at("operator").data.value("control_owner",std::string())},{"result",answer}}).dump();events_->publish(event);RCLCPP_INFO(get_logger(),"%s",event.data.c_str());publish();
 }catch(const std::exception&e){r->accepted=false;r->state="REJECTED";r->reason_code="ZONES.REJECTED";r->message=e.what();if(std::string(e.what()).find("ZONES.STORE_")==0)fault_=e.what();}});
 timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{publish();});
 }
};
}
#ifndef ASTRIBOT_ZONE_SERVER_NO_MAIN
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<astribot_navigation_zones::ZoneServer>());rclcpp::shutdown();}
#endif
