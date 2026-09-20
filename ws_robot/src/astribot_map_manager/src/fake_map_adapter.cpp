#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <astribot_operator_msgs/srv/operator_command.hpp>
#include <nlohmann/json.hpp>
using Json=nlohmann::json;
class FakeMapAdapter:public rclcpp::Node {
 using Command=astribot_operator_msgs::srv::OperatorCommand;
 Json state_={{"boot_id","fake-map-adapter"},{"supports_voxel_sessions",true},{"state","IDLE"},{"handover_ready",true},{"cargo_known",true},{"transport_ready",true}};
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;rclcpp::Service<Command>::SharedPtr command_;rclcpp::TimerBase::SharedPtr timer_;
public:FakeMapAdapter():Node("fake_map_adapter"){
 declare_parameter("outcome","ready");status_=create_publisher<std_msgs::msg::String>("/operator_fake/map_adapter/status",rclcpp::QoS(1).transient_local());
 command_=create_service<Command>("/operator_fake/map_adapter/command",[this](Command::Request::SharedPtr q,Command::Response::SharedPtr r){r->boot_id=state_.at("boot_id");try{
 if(q->expected_boot_id!=r->boot_id||q->operation!="load_session")throw std::runtime_error("REQUEST.CONTEXT_MISMATCH");
 auto p=Json::parse(q->payload_json);auto outcome=get_parameter("outcome").as_string();
 if(outcome=="reject")throw std::runtime_error("MAP.FAKE_REJECTED");
 state_["attempt_id"]=p.at("attempt_id");state_["transaction_id"]=p.at("transaction_id");state_["map_id"]=p.at("target").at("map_id");state_["map_version"]=p.at("target").at("version");state_["state"]="READY";
 for(auto key:{"localization_ready","tf_ready","map_ready","global_costmap_ready","local_costmap_ready"})state_[key]=outcome=="ready";
 r->accepted=true;
 }catch(const std::exception & e){r->reason_code=e.what();}});
 timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{std_msgs::msg::String m;m.data=state_.dump();status_->publish(m);});
 }
};
int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<FakeMapAdapter>());rclcpp::shutdown();}
