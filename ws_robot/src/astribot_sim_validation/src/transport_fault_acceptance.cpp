#include "astribot_sim_validation/fault_evidence.hpp"
#include "astribot_sim_validation/session_evidence.hpp"
#include <astribot_navigation_msgs/msg/robot_geometry_state.hpp>
#include <astribot_operator_msgs/srv/operator_command.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <fstream>
#include <thread>
#include <algorithm>

using Json=nlohmann::json;
using Command=astribot_operator_msgs::srv::OperatorCommand;
using Geometry=astribot_navigation_msgs::msg::RobotGeometryState;
using Clock=std::chrono::steady_clock;
using astribot_sim_validation::StabilityWindow;
static double wall() {return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
static Json readLedger(const std::string & run) {
  try {Json j;std::ifstream in(std::filesystem::path(run)/"ledger/state.json");in>>j;return j;}
  catch(const std::exception &) {return Json();}
}
static Json parse(const std::string & text) {return Json::parse(text,nullptr,false);}

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("transport_fault_acceptance");
  const auto mode=node->declare_parameter("fault_mode",std::string("cancel"));
  const auto stage=node->declare_parameter("fault_stage",std::string("TRANSPORT"));
  const auto report_path=node->declare_parameter("report_path",std::string("/tmp/astribot_fault_result.json"));
  const auto object_id=node->declare_parameter("object_id",std::string("transport_box_01"));
  const double minimum_speed=node->declare_parameter("minimum_speed_mps",.02);
  const double minimum_joint_speed=node->declare_parameter("minimum_joint_speed_radps",0.);
  const double max_wait=node->declare_parameter("max_wait_sec",300.);
  Json report={{"passed",false},{"fault_mode",mode},{"fault_stage",stage},
    {"parameters",{{"minimum_speed_mps",minimum_speed},{"minimum_joint_speed_radps",minimum_joint_speed},{"max_wait_sec",max_wait},{"object_id",object_id},
      {"base_stop_mps",.005},{"angular_stop_radps",.01},{"joint_stop_radps",.03},
      {"payload_error_m",.006},{"sample_max_age_sec",.3},{"stop_window_sec",.5},{"verification_hold_sec",1.}}}};
  Json gateway,session,start_event,payload;
  std::string boot,lease,start_id,run,control_session;
  double gateway_at=0,session_at=0,payload_at=0,odom_at=0,geometry_at=0;
  double speed=0,angular_speed=0,joint_speed=0,arm_motion_speed=0,joint_at=0,command_at=0,payload_radius=0;
  int64_t odom_stamp=0,joint_stamp=0;
  bool injected=false,lease_dropped=false,command_zero=false,control_lost=false;
  Geometry::ConstSharedPtr geometry;
  StabilityWindow base_stop,arm_stop;
  auto client=node->create_client<Command>("/operator_backend/command");
  unsigned sequence=0;
  const auto prefix="fault-"+std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
  auto spin=[&]{rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(20));};
  auto call=[&](const std::string & op) {
    auto q=std::make_shared<Command::Request>();q->robot_id=gateway.at("robot_id");q->expected_boot_id=boot;
    q->lease_id=lease;q->command_id=prefix+"-"+std::to_string(sequence++);q->operation=op;
    q->payload_json=R"({"client_name":"simulation_fault_acceptance"})";
    if(op=="simulation_transport_start")start_id=q->command_id;
    auto future=client->async_send_request(q);
    if(rclcpp::spin_until_future_complete(node,future,std::chrono::seconds(3))!=rclcpp::FutureReturnCode::SUCCESS)throw std::runtime_error("SIM.COMMAND_RESPONSE_UNKNOWN");
    auto response=future.get();if(!response->accepted)throw std::runtime_error(response->reason_code+":"+response->message);
    return response;
  };
  auto gateway_sub=node->create_subscription<std_msgs::msg::String>("/operator_backend/status",rclcpp::QoS(1).transient_local(),[&](std_msgs::msg::String::ConstSharedPtr m){gateway=parse(m->data);gateway_at=wall();});
  auto session_sub=node->create_subscription<std_msgs::msg::String>("/simulation_transport/status",rclcpp::QoS(1).transient_local(),[&](std_msgs::msg::String::ConstSharedPtr m){session=parse(m->data);session_at=wall();});
  auto events_sub=node->create_subscription<std_msgs::msg::String>("/operator_backend/events",100,[&](std_msgs::msg::String::ConstSharedPtr m){auto e=parse(m->data);if(e.is_object()&&e.value("command_id","")==start_id)start_event=e;});
  auto odom_sub=node->create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),[&](nav_msgs::msg::Odometry::ConstSharedPtr m){
    speed=std::hypot(m->twist.twist.linear.x,m->twist.twist.linear.y);angular_speed=std::abs(m->twist.twist.angular.z);odom_at=wall();
    odom_stamp=rclcpp::Time(m->header.stamp).nanoseconds();const auto now=node->get_clock()->now().nanoseconds();
    base_stop.observe(odom_stamp,odom_at,odom_stamp<=now&&now-odom_stamp<=300000000&&std::isfinite(speed)&&std::isfinite(angular_speed)&&speed<=.005&&angular_speed<=.01);
  });
  auto joints_sub=node->create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),[&](sensor_msgs::msg::JointState::ConstSharedPtr m){
    bool valid=!m->name.empty()&&m->velocity.size()==m->name.size();joint_speed=0.;
    arm_motion_speed=0.;joint_at=wall();
    for(size_t i=0;valid&&i<m->name.size();++i)if(m->name[i].find("arm_left_joint_")!=std::string::npos)arm_motion_speed=std::max(arm_motion_speed,std::abs(m->velocity[i]));
    for(double v:m->velocity){valid=valid&&std::isfinite(v);joint_speed=std::max(joint_speed,std::abs(v));}
    const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds(),now=node->get_clock()->now().nanoseconds();
    joint_stamp=valid?stamp:0;
    arm_stop.observe(stamp,wall(),stamp<=now&&now-stamp<=300000000&&valid&&joint_speed<=.03);
  });
  auto cmd_sub=node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",rclcpp::SensorDataQoS(),[&](geometry_msgs::msg::Twist::ConstSharedPtr m){
    command_at=wall();command_zero=true;
    for(double v:{m->linear.x,m->linear.y,m->linear.z,m->angular.x,m->angular.y,m->angular.z})command_zero=command_zero&&std::isfinite(v)&&std::abs(v)<=1e-6;
  });
  auto geometry_sub=node->create_subscription<Geometry>("/navigation/geometry_state",10,[&](Geometry::ConstSharedPtr m){geometry=m;geometry_at=wall();});
  auto payload_sub=node->create_subscription<std_msgs::msg::String>("/model/"+object_id+"/kinematic_attachment/state",10,[&](std_msgs::msg::String::ConstSharedPtr m){payload=parse(m->data);payload_at=wall();});
  auto unique=[&]{return gateway_sub->get_publisher_count()==1&&session_sub->get_publisher_count()==1;};
  double began=wall(),injected_at=0;
  try {
    auto env=[](const char * k){auto v=std::getenv(k);return v?std::string(v):std::string();};
    if(!astribot_sim_validation::isolationValid(env("ROS_DOMAIN_ID"),env("IGN_PARTITION"),env("ROS_LOCALHOST_ONLY"),node->get_parameter("use_sim_time").as_bool()))throw std::runtime_error("SIM.ISOLATION_REQUIRED");
    if((mode!="cancel"&&mode!="lease_loss")||(stage!="TRANSPORT"&&stage!="TRANSPORT_POSTURE")||!std::isfinite(minimum_speed)||minimum_speed<0||minimum_speed>.2||!std::isfinite(minimum_joint_speed)||minimum_joint_speed<0||minimum_joint_speed>1||!std::isfinite(max_wait)||max_wait<=0||max_wait>1800)throw std::runtime_error("SIM.INVALID_FAULT_CONFIG");
    while(rclcpp::ok()&&wall()-began<30&&(!gateway.is_object()||!session.is_object()||!client->service_is_ready()))spin();
    if(!unique()||!gateway.is_object()||!session.is_object())throw std::runtime_error("SIM.BACKEND_UNAVAILABLE");
    if(session.value("motion_blocked",true))throw std::runtime_error("SIM.SESSION_NOT_IDLE");
    boot=gateway.at("boot_id");const auto previous_run=session.value("run",std::string());
    lease=Json::parse(call("acquire")->result_json).at("lease_id");
    const auto settle=wall()+.4;while(wall()<settle)spin();control_session=gateway.at("control_session");
    call("simulation_transport_start");began=wall();double renew=began,terminal_at=0,stopped_at=0;
    Json before,ledger;
    while(rclcpp::ok()&&wall()-began<max_wait) {
      spin();double now=wall();
      if(!unique()||!gateway.is_object()||!session.is_object()||now-gateway_at>2||now-session_at>2||gateway.at("boot_id")!=boot)throw std::runtime_error("SIM.STATUS_UNAVAILABLE");
      if(!lease_dropped&&now>=renew){call("renew");renew=now+1;}
      if(start_event.is_object()&&start_event.value("state","")=="FAILED")throw std::runtime_error("SIM.START_REJECTED:"+start_event.value("message",""));
      const auto observed_run=session.value("run",std::string());
      if(!observed_run.empty()&&observed_run!=previous_run){if(run.empty())run=observed_run;else if(run!=observed_run)throw std::runtime_error("SIM.RUN_CHANGED");}
      if(run.empty())continue;
      ledger=readLedger(run);
      const auto source_now=node->get_clock()->now().nanoseconds();
      if(!injected&&odom_sub->get_publisher_count()==1&&now-odom_at<.5&&odom_stamp>0&&odom_stamp<=source_now&&source_now-odom_stamp<=300000000&&(minimum_joint_speed==0.||(joints_sub->get_publisher_count()==1&&now-joint_at<.3&&joint_stamp>0&&joint_stamp<=source_now&&source_now-joint_stamp<=300000000&&std::isfinite(arm_motion_speed)&&arm_motion_speed>=minimum_joint_speed))&&astribot_sim_validation::faultStageReady(ledger,stage,speed,minimum_speed)){
        before=ledger.at("object");if(before.at("object_id")!=object_id)throw std::runtime_error("SIM.OBJECT_MISMATCH");
        Json scenario;std::ifstream scenario_file(std::filesystem::path(run)/"scenario.json");scenario_file>>scenario;
        const auto dimensions=scenario.at("size_xyz").get<std::vector<double>>();
        if(dimensions.size()!=3)throw std::runtime_error("SIM.PAYLOAD_DIMENSIONS_INVALID");
        for(double d:dimensions){if(!std::isfinite(d)||d<=0)throw std::runtime_error("SIM.PAYLOAD_DIMENSIONS_INVALID");payload_radius+=d*d;}
        payload_radius=.5*std::sqrt(payload_radius);
        report["injection"]={{"ledger",ledger},{"measured_speed_mps",speed},{"measured_arm_speed_radps",arm_motion_speed},{"elapsed_sec",now-began}};
        if(mode=="cancel")call("simulation_transport_cancel");else lease_dropped=true;
        injected=true;injected_at=wall();base_stop.reset();arm_stop.reset();
      }
      // Synchronous command calls spin callbacks. Their receipt times may be
      // newer than the loop's initial timestamp; compare against a fresh clock.
      now=wall();
      if(injected&&(gateway.value("control_state","")!="HELD"||gateway.value("control_session","")!=control_session))control_lost=true;
      if(!astribot_sim_validation::terminalForNewRun(session,previous_run))continue;
      if(!terminal_at){terminal_at=now;base_stop.reset();arm_stop.reset();}
      report["ledger"]=ledger;report["session"]=session;
      if(!injected)throw std::runtime_error("SIM.TERMINATED_BEFORE_INJECTION");
      if(!astribot_sim_validation::canceledWithPayload(ledger,before))throw std::runtime_error("SIM.CANCEL_OR_PAYLOAD_NOT_CONFIRMED");
      const auto ros_now=node->get_clock()->now().nanoseconds();
      bool geometry_ok=geometry&&geometry_sub->get_publisher_count()==1&&now-geometry_at<.5&&
        astribot_sim_validation::geometryAdmission(geometry->complete,geometry->attachment_state_confirmed,rclcpp::Time(geometry->header.stamp).nanoseconds(),rclcpp::Time(geometry->valid_until).nanoseconds(),ros_now)&&
        std::find(geometry->attachment_ids.begin(),geometry->attachment_ids.end(),object_id)!=geometry->attachment_ids.end();
      bool payload_ok=false;
      if(payload.is_object()&&payload_sub->get_publisher_count()==1&&now-payload_at<.5){
        const auto stamp=payload.value("stamp_ns",int64_t(0));
        const double position=payload.value("position_error_m",-1.),rotation=payload.value("rotation_error_rad",-1.);
        payload_ok=stamp>0&&stamp<=ros_now&&ros_now-stamp<=300000000&&payload.value("attached",false)&&payload.value("error",std::string("missing")).empty()&&std::isfinite(position)&&std::isfinite(rotation)&&position>=0&&rotation>=0&&position+payload_radius*rotation<=.006;
      }
      bool stopped=odom_sub->get_publisher_count()==1&&joints_sub->get_publisher_count()==1&&cmd_sub->get_publisher_count()==1&&
        base_stop.ready(now)&&arm_stop.ready(now)&&command_zero&&now-command_at<=.5&&geometry_ok&&payload_ok&&session.value("motion_blocked",false)&&session.value("owned_pid",0)==-1&&(mode!="lease_loss"||control_lost);
      if(!stopped)stopped_at=0.;else if(!stopped_at)stopped_at=now;
      const Json diagnostics={{"odom_publishers",odom_sub->get_publisher_count()},{"joint_publishers",joints_sub->get_publisher_count()},{"command_publishers",cmd_sub->get_publisher_count()},{"base",base_stop.ready(now)},{"joints",arm_stop.ready(now)},{"geometry",geometry_ok},{"payload",payload_ok},{"command",command_zero&&now-command_at<=.5}};
      if(report.value("diagnostics",Json())!=diagnostics){RCLCPP_INFO(node->get_logger(),"STOP_EVIDENCE %s",diagnostics.dump().c_str());report["diagnostics"]=diagnostics;}
      if(!stopped){for(auto it=diagnostics.begin();it!=diagnostics.end();++it)if(it.value().is_boolean()&&!it.value().get<bool>())report["rejected_samples"][it.key()]=report.value("rejected_samples",Json::object()).value(it.key(),0)+1;}
      report["stop_evidence"]={{"base_window",base_stop.ready(now)},{"joint_window",arm_stop.ready(now)},{"zero_command",command_zero&&now-command_at<=.5},{"geometry_attached",geometry_ok},{"gazebo_attached",payload_ok},{"linear_speed_mps",speed},{"angular_speed_radps",angular_speed},{"max_joint_speed_radps",joint_speed},{"control_lost",control_lost}};
      if(stopped_at&&now-stopped_at>=1.){report["passed"]=true;report["cancel_to_verified_sec"]=now-injected_at;break;}
      if(now-terminal_at>15)throw std::runtime_error("SIM.STOP_VERIFICATION_TIMEOUT");
    }
    if(!report.at("passed").get<bool>())throw std::runtime_error("SIM.FAULT_ACCEPTANCE_TIMEOUT");
  } catch(const std::exception & e) {report["error"]=e.what();}
  if(!lease.empty()&&!lease_dropped){try{call("release");}catch(const std::exception & e){report["release_error"]=e.what();report["passed"]=false;}}
  report["run"]=run;report["fault_injected"]=injected;report["elapsed_sec"]=wall()-began;
  report["evidence"]="isolated_gazebo_kinematic_attachment";
  std::ofstream out(report_path);out<<report.dump(2)<<'\n';out.flush();
  const bool passed=report.at("passed").get<bool>()&&bool(out);
  RCLCPP_INFO(node->get_logger(),"%s",report.dump().c_str());rclcpp::shutdown();return passed?0:2;
}
