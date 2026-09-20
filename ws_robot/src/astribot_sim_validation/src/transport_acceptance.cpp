#include "astribot_sim_validation/session_evidence.hpp"
#include <rclcpp/rclcpp.hpp>
#include <astribot_operator_msgs/srv/operator_command.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <thread>
#include <cmath>
using Json=nlohmann::json;using Command=astribot_operator_msgs::srv::OperatorCommand;
int main(int argc,char ** argv){rclcpp::init(argc,argv);try{
 auto env=[](const char * k){auto v=std::getenv(k);return v?std::string(v):std::string();};
 if(!astribot_sim_validation::isolationValid(env("ROS_DOMAIN_ID"),env("IGN_PARTITION"),env("ROS_LOCALHOST_ONLY"),true))throw std::runtime_error("SIM.ISOLATION_REQUIRED");
 auto node=std::make_shared<rclcpp::Node>("transport_acceptance_client");
 const double cancel_after=node->declare_parameter("cancel_after_sec",0.0);
 const double max_wait=node->declare_parameter("max_wait_sec",900.0);
 if(!std::isfinite(max_wait)||max_wait<=0||max_wait>3600||!std::isfinite(cancel_after)||cancel_after<0)throw std::runtime_error("SIM.INVALID_TIMEOUT");
 const auto report_path=node->declare_parameter("report_path",std::string("/tmp/astribot-sim-completion/transport_acceptance.json"));
 Json gateway,session,start_event;std::string start_id;using Clock=std::chrono::steady_clock;Clock::time_point gateway_at{},session_at{};
 auto events_sub=node->create_subscription<std_msgs::msg::String>("/operator_backend/events",100,[&](std_msgs::msg::String::ConstSharedPtr m){auto e=Json::parse(m->data);if(!start_id.empty()&&e.value("command_id","")==start_id)start_event=std::move(e);});
 auto gateway_sub=node->create_subscription<std_msgs::msg::String>("/operator_backend/status",rclcpp::QoS(1).transient_local(),[&](std_msgs::msg::String::ConstSharedPtr m){gateway=Json::parse(m->data);gateway_at=Clock::now();});
 auto session_sub=node->create_subscription<std_msgs::msg::String>("/simulation_transport/status",rclcpp::QoS(1).transient_local(),[&](std_msgs::msg::String::ConstSharedPtr m){session=Json::parse(m->data);session_at=Clock::now();});
 auto client=node->create_client<Command>("/operator_backend/command");auto spin=[&]{rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(20));};
 auto wait_end=Clock::now()+std::chrono::seconds(30);while(rclcpp::ok()&&Clock::now()<wait_end&&(gateway.empty()||session.empty()||!client->service_is_ready()))spin();
 if(gateway.empty()||session.empty()||gateway_sub->get_publisher_count()!=1||session_sub->get_publisher_count()!=1)throw std::runtime_error("SIM.BACKEND_UNAVAILABLE");
 auto boot=gateway.at("boot_id").get<std::string>();std::string lease;int sequence=0;
 const auto prefix="validation-"+std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
 auto call=[&](const std::string & op){auto q=std::make_shared<Command::Request>();q->robot_id=gateway.at("robot_id");q->expected_boot_id=boot;q->lease_id=lease;q->command_id=prefix+"-"+std::to_string(sequence++);q->operation=op;q->payload_json=R"({"client_name":"simulation_acceptance"})";
   if(op=="simulation_transport_start")start_id=q->command_id;
   auto f=client->async_send_request(q);if(rclcpp::spin_until_future_complete(node,f,std::chrono::seconds(3))!=rclcpp::FutureReturnCode::SUCCESS)throw std::runtime_error("SIM.COMMAND_RESPONSE_UNKNOWN");
   auto r=f.get();if(!r->accepted)throw std::runtime_error(r->reason_code+":"+r->message);return r;};
 lease=Json::parse(call("acquire")->result_json).at("lease_id");
 // Let the runtime observe the new control session before submitting the task.
 auto settle=Clock::now()+std::chrono::milliseconds(400);while(Clock::now()<settle)spin();
 if(session.value("motion_blocked",true))throw std::runtime_error("SIM.TRANSPORT_BUSY_OR_UNKNOWN");
 const auto previous_run=session.value("run",std::string());
 call("simulation_transport_start");auto started=Clock::now(),renew=started;bool active=false,cancel_sent=false;std::string outcome="UNKNOWN";
 while(rclcpp::ok()&&std::chrono::duration<double>(Clock::now()-started).count()<max_wait){
   spin();if(gateway_sub->get_publisher_count()!=1||session_sub->get_publisher_count()!=1||Clock::now()-gateway_at>std::chrono::seconds(2)||Clock::now()-session_at>std::chrono::seconds(2)||gateway.at("boot_id")!=boot)break;
   if(start_event.is_object()&&start_event.value("boot_id","")==boot&&start_event.value("state","")=="FAILED"){outcome="START_REJECTED";break;}
   if(Clock::now()>=renew){call("renew");renew=Clock::now()+std::chrono::seconds(1);}
   auto state=session.value("state","");active=active||state=="RUNNING"||state=="STARTING"||state=="CANCELING";
   if(session_at>=started&&astribot_sim_validation::terminalForNewRun(session,previous_run)){outcome=state;break;}
   if(active&&!cancel_sent&&cancel_after>0&&std::chrono::duration<double>(Clock::now()-started).count()>=cancel_after){call("simulation_transport_cancel");cancel_sent=true;}
 }
 call("release");Json result={{"state",outcome},{"session",session},{"start_command",start_event},{"cancel_requested",cancel_sent},{"elapsed_sec",std::chrono::duration<double>(Clock::now()-started).count()},{"evidence","gazebo_kinematic_attachment"}};
 std::ofstream out(report_path);out<<result.dump(2)<<'\n';out.flush();if(!out)throw std::runtime_error("SIM.REPORT_WRITE_FAILED");
 RCLCPP_INFO(node->get_logger(),"%s",result.dump().c_str());rclcpp::shutdown();return outcome=="SUCCEEDED"?0:2;
 }catch(const std::exception & e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 3;}}
