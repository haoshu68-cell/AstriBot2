#include "astribot_sim_validation/session_evidence.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <astribot_navigation_msgs/msg/robot_geometry_state.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <spawn.h>
#include <sys/wait.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <optional>
extern char ** environ;
namespace fs=std::filesystem;using Json=nlohmann::json;
class TransportSession:public rclcpp::Node {
 using Clock=std::chrono::steady_clock;using Trigger=std_srvs::srv::Trigger;
 fs::path root_,scenario_,run_;int lock_=-1;pid_t child_=-1;
 std::string state_="IDLE",detail_,lease_,boot_,geometry_mode_;Json operator_;Clock::time_point seen_{},clock_seen_{};int64_t clock_value_=0;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operator_sub_;
 using Geometry=astribot_navigation_msgs::msg::RobotGeometryState;
 rclcpp::Subscription<Geometry>::SharedPtr geometry_sub_;Geometry::ConstSharedPtr geometry_;Clock::time_point geometry_seen_{};
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
 rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
 rclcpp::Service<Trigger>::SharedPtr start_,cancel_;rclcpp::TimerBase::SharedPtr timer_;
 rclcpp::Client<Trigger>::SharedPtr navigation_client_;
 std::optional<int64_t> navigation_request_;
 Clock::time_point navigation_sent_{},navigation_seen_{};
 uint64_t navigation_generation_=0;
 bool navigation_active_=false;
 bool navigationReady() const {
   return navigation_active_ && navigation_client_->service_is_ready() &&
     Clock::now()-navigation_seen_<std::chrono::seconds(2);
 }
 void pollNavigation() {
   const auto wall=Clock::now();
   if(navigation_request_ && wall-navigation_sent_>=std::chrono::seconds(1)) {
     navigation_client_->remove_pending_request(*navigation_request_);
     navigation_request_.reset();++navigation_generation_;navigation_active_=false;
   }
   if(navigation_request_ || wall-navigation_sent_<std::chrono::seconds(1))return;
   navigation_sent_=wall;
   if(!navigation_client_->service_is_ready()){navigation_active_=false;return;}
   const auto generation=++navigation_generation_;
   try {
     auto pending=navigation_client_->async_send_request(std::make_shared<Trigger::Request>(),
       [this,generation](rclcpp::Client<Trigger>::SharedFuture response) {
         if(generation!=navigation_generation_)return;
         navigation_request_.reset();navigation_seen_=Clock::now();
         try{navigation_active_=response.get()->success;}catch(...){navigation_active_=false;}
       });
     navigation_request_=pending.request_id;
   }catch(const std::exception&){navigation_active_=false;}
 }
 bool authority(){return operator_sub_->get_publisher_count()==1&&Clock::now()-seen_<std::chrono::seconds(2)&&operator_.value("control_state","")=="HELD"&&!operator_.value("control_session","").empty();}
 void checkpoint(){
   auto text=Json({{"state",state_},{"detail",detail_},{"run",run_.string()},{"pid",child_}}).dump(2);
   const auto temp=root_/"runtime.json.tmp";int fd=open(temp.c_str(),O_WRONLY|O_CREAT|O_TRUNC|O_NOFOLLOW,0600);
   if(fd<0)throw std::runtime_error("Cannot open runtime checkpoint");
   size_t n=0;while(n<text.size()){auto w=write(fd,text.data()+n,text.size()-n);if(w<=0){close(fd);throw std::runtime_error("Cannot persist runtime intent");}n+=w;}
   if(fsync(fd)!=0){close(fd);throw std::runtime_error("Cannot sync runtime intent");}close(fd);fs::rename(temp,root_/"runtime.json");
   int dir=open(root_.c_str(),O_DIRECTORY|O_RDONLY);if(dir<0||fsync(dir)!=0){if(dir>=0)close(dir);throw std::runtime_error("Cannot sync runtime directory");}close(dir);
 }
 void requestCancel(){if(child_>0&&state_!="CANCELING"){if(kill(-child_,SIGINT)!=0)throw std::runtime_error("Owned task signal failed");state_="CANCELING";detail_="Await child terminal and ledger; never auto-release payload";checkpoint();}}
 void begin(){
   if(state_!="IDLE"&&state_!="SUCCEEDED")throw std::runtime_error("SIM.RECOVERY_OR_TASK_PENDING");
   if(!navigationReady())throw std::runtime_error("SIM.NAVIGATION_NOT_READY");
   if(!authority()||operator_.value("nav_outstanding",true)||!operator_.value("route_id",std::string()).empty())throw std::runtime_error("SIM.OPERATOR_NOT_IDLE");
   if(Clock::now()-clock_seen_>std::chrono::seconds(1)||clock_value_<=0)throw std::runtime_error("SIM.CLOCK_NOT_ADVANCING");
   if(count_publishers("/clock")!=1||count_publishers("/odom")!=1||count_publishers("/joint_states")!=1)throw std::runtime_error("SIM.SOURCE_CONFLICT_OR_MISSING");
   if(count_publishers("/transport/status")!=0)throw std::runtime_error("SIM.EXTERNAL_TRANSPORT");
   if(geometry_mode_=="fixed_v2"){
     if(!geometry_||geometry_sub_->get_publisher_count()!=1||Clock::now()-geometry_seen_>std::chrono::seconds(1))throw std::runtime_error("SIM.GEOMETRY_UNAVAILABLE");
     const auto source=rclcpp::Time(geometry_->header.stamp).nanoseconds(),until=rclcpp::Time(geometry_->valid_until).nanoseconds();
     if(!astribot_sim_validation::geometryAdmission(geometry_->complete,geometry_->attachment_state_confirmed,source,until,get_clock()->now().nanoseconds()))throw std::runtime_error("SIM.GEOMETRY_NOT_READY: "+geometry_->reason);
   }
   std::ifstream input(scenario_);Json config;input>>config;if(config.value("environment","")!="simulation")throw std::runtime_error("SIM.INVALID_SCENARIO");
   run_=root_/("task_"+std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));if(!fs::create_directory(run_))throw std::runtime_error("SIM.RUN_COLLISION");
   config["navigation_geometry_mode"]=geometry_mode_;
   std::ofstream scenario_out(run_/"scenario.json");scenario_out<<config.dump(2)<<'\n';scenario_out.close();
   if(!scenario_out)throw std::runtime_error("SIM.SCENARIO_WRITE_FAILED");
   lease_=operator_.value("control_session","");boot_=operator_.value("boot_id","");
   state_="STARTING";detail_="Existing transport owns planning, controllers, payload and resource lock";checkpoint();
   std::vector<std::string> args={"ros2","run","astribot_s1_transport","transport_task","--scenario",(run_/"scenario.json").string(),"--output",(run_/"ledger").string(),"--navigation-geometry-mode",geometry_mode_};
   std::vector<char *> argv;for(auto & a:args)argv.push_back(a.data());argv.push_back(nullptr);
   posix_spawnattr_t attr;posix_spawnattr_init(&attr);posix_spawnattr_setflags(&attr,POSIX_SPAWN_SETPGROUP);posix_spawnattr_setpgroup(&attr,0);
   int error=posix_spawnp(&child_,"ros2",nullptr,&attr,argv.data(),environ);posix_spawnattr_destroy(&attr);
   if(error){child_=-1;state_="RECOVERY_REQUIRED";detail_="Spawn failed; inspect persisted intent";checkpoint();throw std::runtime_error(detail_);}
   state_="RUNNING";checkpoint();
 }
 void tick(){
   pollNavigation();
   auto now=get_clock()->now().nanoseconds();if(now>clock_value_){clock_seen_=Clock::now();}else if(now<clock_value_){clock_seen_={};}clock_value_=now;
   if(child_>0){int status;auto pid=waitpid(child_,&status,WNOHANG);
     if(pid==child_){child_=-1;state_="RECOVERY_REQUIRED";detail_="Child exited; reconcile payload and ledger before retry";
       try{Json ledger;std::ifstream in(run_/"ledger/state.json");in>>ledger;
         if(astribot_sim_validation::completedLedger(ledger,WIFEXITED(status)&&WEXITSTATUS(status)==0)){state_="SUCCEEDED";detail_="Simulation placement confirmed by existing transport ledger";}}
       catch(const std::exception &){}checkpoint();
     }else if(!authority()||operator_.value("control_session","")!=lease_||operator_.value("boot_id","")!=boot_||Clock::now()-clock_seen_>std::chrono::seconds(2))requestCancel();
   }
   std_msgs::msg::String message;message.data=Json({{"state",state_},{"detail",detail_},{"run",run_.string()},{"owned_pid",child_},
     {"motion_blocked",state_!="IDLE"&&state_!="SUCCEEDED"},{"navigation_ready",navigationReady()},
     {"simulation_only",true},{"evidence_level","gazebo_kinematic_attachment"}}).dump();status_->publish(message);
 }
public:
 explicit TransportSession():Node("simulation_transport"){
   auto env=[](const char * key){auto p=std::getenv(key);return p?std::string(p):std::string();};
   if(!astribot_sim_validation::isolationValid(env("ROS_DOMAIN_ID"),env("IGN_PARTITION"),env("ROS_LOCALHOST_ONLY"),get_parameter("use_sim_time").as_bool()))throw std::runtime_error("SIM.ISOLATION_REQUIRED");
   rcl_interfaces::msg::ParameterDescriptor fixed;fixed.read_only=true;
   root_=declare_parameter("output_root",std::string("/tmp/astribot_sim_sessions"),fixed);scenario_=declare_parameter("scenario",std::string(),fixed);
   geometry_mode_=declare_parameter("navigation_geometry_mode",std::string("fixed_v2"),fixed);
   if(geometry_mode_!="fixed_v2"&&geometry_mode_!="legacy")throw std::runtime_error("SIM.INVALID_GEOMETRY_MODE");
   parameter_guard_=add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> & values){rcl_interfaces::msg::SetParametersResult r;r.successful=true;for(const auto & p:values)if(p.get_name()=="use_sim_time"&&(p.get_type()!=rclcpp::ParameterType::PARAMETER_BOOL||!p.as_bool())){r.successful=false;r.reason="Simulation time must remain enabled";}return r;});
   if(!fs::is_regular_file(scenario_))throw std::runtime_error("SIM.SCENARIO_REQUIRED");
   fs::create_directories(root_);
   lock_=open((root_/"runtime.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
   if(lock_<0||flock(lock_,LOCK_EX|LOCK_NB)!=0){if(lock_>=0)close(lock_);lock_=-1;throw std::runtime_error("SIM.RUNTIME_ALREADY_OWNED");}
   if(fs::exists(root_/"runtime.json")){Json old;std::ifstream in(root_/"runtime.json");in>>old;run_=old.value("run","");state_="RECOVERY_REQUIRED";detail_="Previous session exists; no automatic replay on restart";}
   status_=create_publisher<std_msgs::msg::String>("~/status",rclcpp::QoS(1).transient_local());
   navigation_client_=create_client<Trigger>("/lifecycle_manager_navigation/is_active");
   geometry_sub_=create_subscription<Geometry>("/navigation/geometry_state",10,[this](Geometry::ConstSharedPtr m){geometry_=m;geometry_seen_=Clock::now();});
   operator_sub_=create_subscription<std_msgs::msg::String>("/operator_backend/status",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){try{if(m->data.size()>1048576)return;operator_=Json::parse(m->data);seen_=Clock::now();}catch(...){seen_={};}});
   start_=create_service<Trigger>("~/start",[this](Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){try{begin();r->success=true;r->message=run_.string();}catch(const std::exception & e){r->message=e.what();}});
   cancel_=create_service<Trigger>("~/cancel",[this](Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){try{if(child_<=0)throw std::runtime_error("SIM.NO_OWNED_TASK");requestCancel();r->success=true;r->message="Cancel requested; await terminal";}catch(const std::exception & e){r->message=e.what();}});
   timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{try{tick();}catch(const std::exception & e){state_="RECOVERY_REQUIRED";detail_=e.what();if(child_>0)kill(-child_,SIGINT);}});
 }
 ~TransportSession()override{if(child_>0)kill(-child_,SIGINT);if(lock_>=0)close(lock_);}
};
int main(int argc,char **argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<TransportSession>());rclcpp::shutdown();return 0;}catch(const std::exception & e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 2;}}
