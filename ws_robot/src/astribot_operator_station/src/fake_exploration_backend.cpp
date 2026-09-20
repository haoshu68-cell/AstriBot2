// Development backend: all endpoints are confined to /operator_fake.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <astribot_operator_msgs/srv/exploration_command.hpp>
#include <nlohmann/json.hpp>
#include <map>
#include <random>
class FakeExplorationBackend : public rclcpp::Node {
 using Command=astribot_operator_msgs::srv::ExplorationCommand;
 using Json=nlohmann::json;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_,map_;
 rclcpp::Service<Command>::SharedPtr command_;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr retry_;
 rclcpp::TimerBase::SharedPtr timer_;
 std::string boot_,scenario_;
 uint64_t revision_{0};
 std::map<std::string,std::pair<std::string,Command::Response>> requests_;
 Json snapshot() {
   const auto scenario=get_parameter("scenario").as_string();
   if(scenario!=scenario_){scenario_=scenario;++revision_;}
   const bool paused=scenario_=="paused";
   const bool ending=scenario_=="saving"||scenario_=="save_failed"||scenario_=="saved";
   const bool ready=scenario_!="waiting_map";
   return {{"schema_version",1},{"boot_id",boot_},{"revision",revision_},{"state",paused?"PAUSED":"IDLE"},
    {"reason_code",!ready?"EXPLORATION.NOT_READY":paused?"EXPLORATION.PAUSED":"EXPLORATION.RUNNING"},
    {"manual_pause",paused},{"session_ending",ending},{"cancel_pending",scenario_=="cancel_pending"},
    {"can_pause",!paused&&!ending},{"can_resume",paused},{"can_cancel_save",!ending},
    {"progress",scenario_=="unreachable"?"UNREACHABLE_FRONTIERS":scenario_=="completing"?"CONFIRMING_COMPLETE":"EXPLORING"},
    {"readiness_detail",ready?"":"FAKE: waiting for map"},{"transition_reason","FAKE / NO ROBOT"}};
 }
 void publish() {
   auto state=snapshot();if(scenario_=="disconnected")return;
   std_msgs::msg::String message;message.data=state.dump();state_->publish(message);
   message.data=Json({{"state",scenario_=="save_failed"?"FAILED":scenario_=="saved"?"SAVED":scenario_=="saving"?"WAIT_STOP":"IDLE"},
    {"directory",scenario_=="saved"?"/fake/no-map-written":""},{"detail","FAKE: no files or robot actions"},{"exploration_outcome","CANCELED_PARTIAL"}}).dump();map_->publish(message);
 }
public:
 FakeExplorationBackend():Node("fake_exploration_backend") {
   std::random_device entropy;boot_="fake-"+std::to_string(entropy());declare_parameter("scenario","waiting_map");
   state_=create_publisher<std_msgs::msg::String>("/operator_fake/exploration_status",rclcpp::QoS(1).transient_local());
   map_=create_publisher<std_msgs::msg::String>("/operator_fake/mapping_status",rclcpp::QoS(1).transient_local());
   command_=create_service<Command>("/operator_fake/command",[this](Command::Request::SharedPtr r,Command::Response::SharedPtr s){
     auto state=snapshot();s->boot_id=boot_;s->revision=revision_;
     const auto content=Json({r->operation,r->expected_revision}).dump();
     if(r->expected_boot_id!=boot_){s->reason_code="REQUEST.BOOT_MISMATCH";return;}
     auto old=requests_.find(r->command_id);
     if(old!=requests_.end()){if(old->second.first!=content)s->reason_code="REQUEST.CONFLICT";else *s=old->second.second;return;}
     if(r->command_id.empty()||requests_.size()>=4096){s->reason_code="REQUEST.INVALID_ID_OR_CAPACITY";return;}
     if(r->expected_revision!=revision_){s->reason_code="STATE.REVISION_MISMATCH";return;}
     if(!state.value("can_"+r->operation,false)){s->reason_code="EXPLORATION.OPERATION_BLOCKED";return;}
     set_parameter(rclcpp::Parameter("scenario",r->operation=="pause"?"paused":r->operation=="resume"?"running":"saving"));
     snapshot();s->revision=revision_;s->accepted=true;s->reason_code="COMMAND.ACCEPTED";s->message="FAKE: no robot action";
     requests_[r->command_id]={content,*s};publish();
   });
   retry_=create_service<std_srvs::srv::Trigger>("/operator_fake/retry",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr r){
     snapshot();r->success=scenario_=="save_failed";r->message="FAKE retry only";
     if(r->success)set_parameter(rclcpp::Parameter("scenario","saving"));
     publish();
   });
   timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{publish();});
 }
};
int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<FakeExplorationBackend>());rclcpp::shutdown();}
