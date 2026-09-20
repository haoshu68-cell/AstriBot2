#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <filesystem>
#include <chrono>
#include <cmath>
#include <set>
extern char ** environ;
class MappingRuntime : public rclcpp::Node {
 using Clock=std::chrono::steady_clock;
 std::string map_topic_{"/map"},odom_topic_{"/odom"};
 std::string profile_,root_,state_{"IDLE"},session_,detail_,map_state_;
 std::vector<std::string> deployment_args_;bool configured_{false};
 pid_t child_{-1};Clock::time_point odom_at_{},still_since_{},map_at_{},deadline_{};
 bool still_{false};bool restart_{false};
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapping_;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_;
 rclcpp::TimerBase::SharedPtr timer_;
 bool stopped()const{return still_&&odom_->get_publisher_count()==1&&Clock::now()-odom_at_<std::chrono::seconds(1)&&Clock::now()-still_since_>std::chrono::seconds(2);}
 bool externalNodes() {if(count_publishers(map_topic_)>0)return true;for(const auto & name:get_node_names())if(name=="/voxelslam"||name=="/exploration_coordinator_node"||name=="/mapping_session"||name=="/map_odom_tf")return true;return false;}
 void spawn() {
   if(externalNodes())throw std::runtime_error("Existing SLAM/exploration instance; stop it through its owner first");
   if(!stopped())throw std::runtime_error("Fresh odometry stop evidence missing");
   map_at_={};map_state_.clear();
   session_="map_"+std::to_string(std::chrono::system_clock::now().time_since_epoch().count())+"_"+std::to_string(getpid());
   if(std::filesystem::exists(std::filesystem::path(root_)/session_))throw std::runtime_error("Session directory collision");
   std::vector<std::string> arguments={"ros2","launch","astribot_operator_backend","owned_mapping.launch.xml","profile:="+profile_,"save_path:="+root_,"map_name:="+session_,"use_sim_time:="+std::string(profile_=="sim"?"true":"false")};
   arguments.insert(arguments.end(),deployment_args_.begin(),deployment_args_.end());
   std::vector<char *> argv;for(auto & a:arguments)argv.push_back(a.data());argv.push_back(nullptr);
   posix_spawnattr_t attr;posix_spawnattr_init(&attr);posix_spawnattr_setflags(&attr,POSIX_SPAWN_SETPGROUP);posix_spawnattr_setpgroup(&attr,0);
   auto error=posix_spawnp(&child_,"ros2",nullptr,&attr,argv.data(),environ);posix_spawnattr_destroy(&attr);
   if(error){child_=-1;throw std::runtime_error("Cannot spawn owned mapping launch");}
   state_="STARTING";detail_="Owned session starts paused; wait for readiness before explicit resume";deadline_=Clock::now()+std::chrono::seconds(30);
 }
 void tick(){
   if(child_>0){int result=0;auto ended=waitpid(child_,&result,WNOHANG);if(ended==child_){child_=-1;
     if(restart_){restart_=false;state_="WAIT_OLD_GRAPH";deadline_=Clock::now()+std::chrono::seconds(10);}else{state_="EXITED";detail_="Owned launch exited; inspect logs, no automatic restart";}}
   }
   if(state_=="WAIT_OLD_GRAPH"&&!externalNodes())spawn();
   if(state_=="STARTING"&&mapping_->get_publisher_count()==1&&Clock::now()-map_at_<std::chrono::seconds(2)&&map_state_=="IDLE")state_="RUNNING";
   if((state_=="STARTING"||state_=="STOPPING_OWNED"||state_=="WAIT_OLD_GRAPH")&&Clock::now()>deadline_){state_="FAILED";detail_="Lifecycle timeout; owned process retained for diagnosis, no force kill";}
   std_msgs::msg::String message;message.data=nlohmann::json({{"state",state_},{"session_id",session_},{"detail",detail_},{"configured",configured_},{"can_start",configured_&&stopped()&&((child_<0&&!externalNodes())||(child_>0&&state_=="RUNNING"&&mapping_->get_publisher_count()==1&&Clock::now()-map_at_<std::chrono::seconds(3)&&map_state_=="SAVED"))},{"owned_pid",child_},{"stop_evidence",stopped()}}).dump();status_->publish(message);
 }
public:
 explicit MappingRuntime(const rclcpp::NodeOptions & options=rclcpp::NodeOptions()):Node("mapping_runtime",options) {
   // Hardware profiles require deployment-specific calibration; never guess these values.
   profile_=declare_parameter("profile","");root_=declare_parameter("save_path","/tmp/astribot_slam_sessions");
   deployment_args_=declare_parameter<std::vector<std::string>>("launch_arguments",std::vector<std::string>{});
   const std::set<std::string> allowed={"lidar_topic","lidar_topic_back","imu_topic","point_notime","imu_extrinsic_tran","back_extrinsic_tran","back_extrinsic_rota","map_topic","odom_topic","robot_base_frame"};
   std::set<std::string> provided;
   for(const auto & arg:deployment_args_){auto at=arg.find(":=");auto key=arg.substr(0,at);
     if(key=="map_topic"&&at!=std::string::npos)map_topic_=arg.substr(at+2);
     if(key=="odom_topic"&&at!=std::string::npos)odom_topic_=arg.substr(at+2);
     if(at==std::string::npos||!allowed.count(key)||!provided.insert(key).second||arg.substr(at+2).empty())throw std::runtime_error("Invalid/duplicate mapping launch argument");
   }
   configured_=profile_=="sim"||(profile_=="hardware"&&provided==allowed);
   status_=create_publisher<std_msgs::msg::String>("~/status",rclcpp::QoS(1).transient_local());
   mapping_=create_subscription<std_msgs::msg::String>("/mapping_session/status",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){try{map_state_=nlohmann::json::parse(m->data).at("state");map_at_=Clock::now();}catch(...){map_at_={};}});
   odom_=create_subscription<nav_msgs::msg::Odometry>(odom_topic_,rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){const auto & t=m->twist.twist;
     bool value=std::isfinite(t.linear.x)&&std::isfinite(t.linear.y)&&std::isfinite(t.angular.z)&&std::hypot(t.linear.x,t.linear.y)<.01&&std::abs(t.angular.z)<.01;
     auto now=Clock::now();if(!value||!still_||now-odom_at_>std::chrono::seconds(1))still_since_=now;still_=value;odom_at_=now;});
   start_=create_service<std_srvs::srv::Trigger>("~/start",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr r){try{
     if(!configured_)throw std::runtime_error("No validated runtime profile configured; hardware calibration must be provisioned");
     if(!stopped())throw std::runtime_error("No fresh stop evidence");
     if(child_>0){if(state_!="RUNNING"||restart_)throw std::runtime_error("Restart already pending");
       if(mapping_->get_publisher_count()!=1||Clock::now()-map_at_>std::chrono::seconds(3)||map_state_!="SAVED")throw std::runtime_error("Current session not confirmed saved");
       if(kill(-child_,SIGINT)!=0)throw std::runtime_error("Cannot stop owned launch");
       restart_=true;state_="STOPPING_OWNED";deadline_=Clock::now()+std::chrono::seconds(15);
     }else spawn();r->success=true;r->message="New mapping session accepted; starts paused";
   }catch(const std::exception & e){r->success=false;r->message=e.what();}});
   timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{try{tick();}catch(const std::exception & e){state_="FAILED";detail_=e.what();}});
 }
 ~MappingRuntime() override {if(child_>0)kill(-child_,SIGINT);}
 // Only the process group created by this instance is signaled; never force-kill.
};

#ifndef ASTRIBOT_MAPPING_RUNTIME_NO_MAIN
int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<MappingRuntime>());rclcpp::shutdown();}

#endif
