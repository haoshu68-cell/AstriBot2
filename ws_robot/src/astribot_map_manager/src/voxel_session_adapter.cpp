#include "astribot_map_manager/voxel_activation.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <astribot_operator_msgs/srv/operator_command.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <spdlog/spdlog.h>
#include <spawn.h>
#include <sys/wait.h>
#include <sys/file.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <future>
#include <random>
#include <set>
#include <cmath>
#include <algorithm>
extern char ** environ;
namespace astribot_map_manager {
class VoxelSessionAdapter:public rclcpp::Node {
 using Clock=std::chrono::steady_clock;using Command=astribot_operator_msgs::srv::OperatorCommand;using Manage=nav2_msgs::srv::ManageLifecycleNodes;
 std::string robot_,boot_,profile_,assets_,runtime_,map_topic_,base_,odom_frame_,state_{"IDLE"},reason_,attempt_,transaction_;
 Json target_=Json::object(),manager_=Json::object(),operator_=Json::object();Clock::time_point manager_at_{},operator_at_{},odom_at_{},still_since_{},map_at_{},global_at_{},local_at_{},deadline_{};
 int samples_{0},lock_{-1};int64_t last_stamp_{0},epoch_{0};pid_t child_{-1};bool enabled_{false},pending_{false},rpc_pending_{false},config_checked_{false},reset_{false},started_{false};uint64_t generation_{0};
 std::vector<std::string> args_;std::map<std::string,std::string> requests_;size_t request_bytes_{0};Clock::time_point accepted_at_{};
 std::future<SessionLoadPlan> materialized_;SessionLoadPlan plan_;std::string slam_state_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manager_sub_,operator_sub_,slam_sub_;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
 rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
 rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_sub_,local_sub_;
 rclcpp::Client<Manage>::SharedPtr lifecycle_;std::shared_ptr<rclcpp::AsyncParametersClient> nav_parameters_;
 rclcpp::Service<Command>::SharedPtr command_;rclcpp::TimerBase::SharedPtr timer_;
 std::shared_ptr<tf2_ros::Buffer> tf_;std::shared_ptr<tf2_ros::TransformListener> listener_;
 template<class T>bool fresh(const std::shared_ptr<T> & sub,Clock::time_point at,double seconds=2)const{return sub->get_publisher_count()==1&&Clock::now()-at<std::chrono::duration<double>(seconds);}
 bool stampFresh(const builtin_interfaces::msg::Time & stamp)const {auto ns=rclcpp::Time(stamp).nanoseconds();auto age=(now().nanoseconds()-ns)/1e9;return ns>epoch_&&age>=0&&age<2;}
 bool stopped()const{return samples_>=3&&fresh(odom_sub_,odom_at_,.5)&&Clock::now()-still_since_>std::chrono::seconds(1);}
 bool permitted()const{return stopped()&&fresh(manager_sub_,manager_at_)&&fresh(operator_sub_,operator_at_)&&operator_.value("control_state","")=="HELD"&&manager_.value("motion_blocked",false)&&!manager_.value("transaction",Json()).is_null()&&manager_.at("transaction").value("transaction_id","")==transaction_&&manager_.at("transaction").value("state","")=="LOADING"&&manager_.at("transaction").contains("target")&&manager_.at("transaction").at("target").value("map_id","")==target_.value("map_id","")&&manager_.at("transaction").at("target").value("version","")==target_.value("version","");}
 bool external()const {for(const auto & name:get_node_names())if(name=="/voxelslam"||name=="/nav_prob_grid_node"||name=="/map_odom_tf")return true;return count_publishers(map_topic_)>0||count_publishers("/slam/status")>0;}
 void fail(const std::string & reason){state_="FAILED";reason_=reason;pending_=false;++generation_;spdlog::error("Voxel activation {}: {}",attempt_,reason);}
 void launch(){
 if(external())throw std::runtime_error("MAP.EXTERNAL_SLAM_OWNED_BY_OTHER");
 std::vector<std::string> command={"ros2","launch","astribot_s1_perception","voxel_slam.launch.py","mode:=localization","save_map:=0","save_path:="+plan_.root.string(),"previous_map:="+plan_.session+":0.5","map_name:=","use_sim_time:="+std::string(profile_=="sim"?"true":"false"),"map_topic:="+map_topic_};command.insert(command.end(),args_.begin(),args_.end());
 child_=spawnOwned(command);if(child_<=0)throw std::runtime_error("MAP.SPAWN_FAILED");
 epoch_=now().nanoseconds();map_at_={};slam_state_.clear();global_at_={};local_at_={};listener_.reset();tf_=std::make_shared<tf2_ros::Buffer>(get_clock());listener_=std::make_shared<tf2_ros::TransformListener>(*tf_);state_="WAIT_LOCALIZATION";
 }
 void lifecycle(uint8_t operation,const std::string & next){
 if(!lifecycle_->service_is_ready()){fail("MAP.LIFECYCLE_UNAVAILABLE");return;}pending_=true;rpc_pending_=true;auto q=std::make_shared<Manage::Request>();q->command=operation;auto generation=generation_;
 lifecycle_->async_send_request(q,[this,next,operation,generation](rclcpp::Client<Manage>::SharedFuture f){rpc_pending_=false;if(generation!=generation_)return;try{pending_=false;if(!f.get()->success){fail("MAP.LIFECYCLE_REJECTED");return;}
 if(!permitted()){fail("MAP.AUTHORITY_LOST");return;}if(operation==Manage::Request::RESET){reset_=true;launch();}else{started_=true;epoch_=now().nanoseconds();global_at_={};local_at_={};state_=next;}}catch(const std::exception & e){fail(e.what());}});
 }
 ActivationEvidence evidence()const{
 ActivationEvidence e;e.nav_reset=reset_;e.nav_started=started_;e.tracking=child_>0&&slam_state_=="TRACKING"&&slam_sub_->get_publisher_count()==1;e.map=fresh(map_sub_,map_at_,5);
 e.global_costmap=fresh(global_sub_,global_at_);e.local_costmap=fresh(local_sub_,local_at_);
 try{auto t=tf_->lookupTransform("map",base_,tf2::TimePointZero);e.tf=stampFresh(t.header.stamp);}catch(const tf2::TransformException &){}
 try{auto slam=tf_->lookupTransform("camera_init","aft_mapped",tf2::TimePointZero);e.tracking=e.tracking&&stampFresh(slam.header.stamp);}catch(const tf2::TransformException &){e.tracking=false;}return e;
 }
 void publish(){auto e=evidence();std_msgs::msg::String m;m.data=Json({{"boot_id",boot_},{"supports_voxel_sessions",enabled_},{"state",state_=="VERIFYING"&&e.ready()?"READY":state_},{"reason_code",reason_},{"attempt_id",attempt_},{"transaction_id",transaction_},{"map_id",target_.value("map_id","")},{"map_version",target_.value("version","")},{"localization_ready",e.tracking},{"tf_ready",e.tf},{"map_ready",e.map},{"global_costmap_ready",e.nav_reset&&e.nav_started&&e.global_costmap},{"local_costmap_ready",e.nav_reset&&e.nav_started&&e.local_costmap},{"cargo_known",false},{"transport_ready",false},{"handover_ready",false},{"owned_pid",child_}}).dump();status_->publish(m);}
 void tick(){
 if(child_>0){int result=0;if(waitpid(child_,&result,WNOHANG)==child_){child_=-1;if(state_=="STOP_OWNED")state_="WAIT_OLD_GRAPH";else fail("MAP.CHILD_EXITED");}}
 if(state_!="IDLE"&&state_!="FAILED"&&state_!="VERIFYING"&&!(state_=="MATERIALIZING"&&Clock::now()-accepted_at_<std::chrono::seconds(3))&&!permitted()){fail("MAP.AUTHORITY_LOST");}
 if(state_!="IDLE"&&state_!="FAILED"&&state_!="VERIFYING"&&Clock::now()>deadline_)fail("MAP.ACTIVATION_TIMEOUT");
 if(state_=="MATERIALIZING"&&permitted()&&materialized_.wait_for(std::chrono::seconds(0))==std::future_status::ready){plan_=materialized_.get();if(child_>0){if(kill(-child_,SIGINT))throw std::runtime_error("MAP.OWNED_STOP_FAILED");state_="STOP_OWNED";}else state_="WAIT_OLD_GRAPH";}
 if(state_=="WAIT_OLD_GRAPH"&&!pending_&&!external()){
 if(!config_checked_){
   if(!nav_parameters_->service_is_ready()){fail("MAP.NAV_CONFIG_UNAVAILABLE");}
   else{pending_=true;rpc_pending_=true;auto generation=generation_;
     nav_parameters_->get_parameters({"static_layer.map_topic","global_frame","always_send_full_costmap"},[this,generation](std::shared_future<std::vector<rclcpp::Parameter>> f){
       rpc_pending_=false;if(generation!=generation_)return;pending_=false;try{auto p=f.get();
       if(p.size()!=3||p[0].as_string()!=map_topic_||p[1].as_string()!="map"||!p[2].as_bool()){fail("MAP.NAV_CONFIG_MISMATCH");return;}config_checked_=true;
       }catch(const std::exception &){fail("MAP.NAV_CONFIG_MISMATCH");}});
   }
 }else lifecycle(Manage::Request::RESET,"WAIT_LOCALIZATION");
 }
 if(state_=="WAIT_LOCALIZATION"&&!pending_){auto e=evidence();if(e.tracking&&e.map&&e.tf)lifecycle(Manage::Request::STARTUP,"VERIFYING");}
 // After commitment, continue reporting observed quality, independent of the old control lease.
 if(state_=="VERIFYING"&&!evidence().ready()&&Clock::now()>deadline_&&manager_.value("state","")!="READY")fail("MAP.VERIFICATION_TIMEOUT");
 publish();
 }
protected:
 virtual pid_t spawnOwned(std::vector<std::string> command){
 std::vector<char *> argv;for(auto & a:command)argv.push_back(a.data());argv.push_back(nullptr);posix_spawnattr_t attr;posix_spawnattr_init(&attr);posix_spawnattr_setflags(&attr,POSIX_SPAWN_SETPGROUP);posix_spawnattr_setpgroup(&attr,0);
 pid_t pid=-1;int result=posix_spawnp(&pid,argv[0],nullptr,&attr,argv.data(),environ);posix_spawnattr_destroy(&attr);return result?-1:pid;
 }
public:
 explicit VoxelSessionAdapter(const rclcpp::NodeOptions & options=rclcpp::NodeOptions()):Node("map_session_adapter",options){
 std::random_device random;boot_=std::to_string(random())+"-"+std::to_string(random());robot_=declare_parameter("robot_id","astribot");profile_=declare_parameter("profile","");assets_=declare_parameter("asset_root","/tmp/astribot_map_catalog/assets");runtime_=declare_parameter("runtime_root","/tmp/astribot_voxel_activation");map_topic_=declare_parameter("map_topic","/map");base_=declare_parameter("robot_base_frame","astribot_torso_base");odom_frame_=declare_parameter("odom_frame","odom");
 enabled_=declare_parameter("allow_navigation_reconfigure",false)&&(profile_=="sim"||profile_=="hardware");args_=declare_parameter<std::vector<std::string>>("launch_arguments",std::vector<std::string>{});
 const std::set<std::string> allowed={"lidar_topic","lidar_topic_back","imu_topic","point_notime","imu_extrinsic_tran","back_extrinsic_tran","back_extrinsic_rota"};std::set<std::string> provided;
 for(const auto & arg:args_){auto pos=arg.find(":=");auto key=arg.substr(0,pos);if(pos==std::string::npos||!allowed.count(key)||!provided.insert(key).second||arg.substr(pos+2).empty())throw std::runtime_error("Invalid deployment arguments");}
 if(profile_=="hardware"&&provided!=allowed)enabled_=false;
 fs::create_directories(runtime_);lock_=open((fs::path(runtime_)/"adapter.lock").c_str(),O_CREAT|O_RDWR|O_NOFOLLOW,0600);if(lock_<0||flock(lock_,LOCK_EX|LOCK_NB))throw std::runtime_error("Adapter runtime already owned");
 tf_=std::make_shared<tf2_ros::Buffer>(get_clock());listener_=std::make_shared<tf2_ros::TransformListener>(*tf_);
 status_=create_publisher<std_msgs::msg::String>("~/status",rclcpp::QoS(1).transient_local());
 auto observe=[this](const std::string & topic,Json & value,Clock::time_point & at){return create_subscription<std_msgs::msg::String>(topic,rclcpp::QoS(1).transient_local(),[&value,&at](std_msgs::msg::String::ConstSharedPtr m){try{if(m->data.size()>524288)throw std::runtime_error("oversized");value=Json::parse(m->data);at=Clock::now();}catch(...){at={};}});};
 manager_sub_=observe("/map_manager/status",manager_,manager_at_);operator_sub_=observe("/operator_backend/status",operator_,operator_at_);
 slam_sub_=create_subscription<std_msgs::msg::String>("/slam/status",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){slam_state_=m->data;});
 map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic_,rclcpp::QoS(1).transient_local(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){if(m->header.frame_id=="map"&&stampFresh(m->header.stamp)&&m->info.width>0&&m->info.height>0&&m->data.size()==uint64_t(m->info.width)*m->info.height&&std::any_of(m->data.begin(),m->data.end(),[](int8_t v){return v>=0;}))map_at_=Clock::now();else map_at_={};});
 auto costmap=[this](std::string topic,std::string frame,Clock::time_point & at){return create_subscription<nav2_msgs::msg::Costmap>(topic,rclcpp::QoS(1).reliable().transient_local(),[this,frame,&at](nav2_msgs::msg::Costmap::ConstSharedPtr m){if(started_&&m->header.frame_id==frame&&stampFresh(m->metadata.update_time)&&m->metadata.size_x&&m->metadata.size_y&&m->data.size()==uint64_t(m->metadata.size_x)*m->metadata.size_y&&std::any_of(m->data.begin(),m->data.end(),[](uint8_t v){return v!=255;}))at=Clock::now();else at={};});};
 global_sub_=costmap("/global_costmap/costmap_raw","map",global_at_);local_sub_=costmap("/local_costmap/costmap_raw",odom_frame_,local_at_);
 odom_sub_=create_subscription<nav_msgs::msg::Odometry>(declare_parameter("odom_topic","/odom"),rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){auto at=Clock::now();auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();auto age=(now().nanoseconds()-stamp)/1e9;const auto & v=m->twist.twist;
 bool valid=stamp>last_stamp_&&age>=0&&age<.5&&std::isfinite(v.linear.x)&&std::isfinite(v.linear.y)&&std::isfinite(v.angular.z)&&std::hypot(v.linear.x,v.linear.y)<.01&&std::abs(v.angular.z)<.01;if(!valid||!samples_||at-odom_at_>std::chrono::milliseconds(500)){samples_=0;still_since_=at;}if(valid)++samples_;last_stamp_=stamp;odom_at_=at;});
 lifecycle_=create_client<Manage>(declare_parameter("navigation_manager","/lifecycle_manager_navigation/manage_nodes"));
 nav_parameters_=std::make_shared<rclcpp::AsyncParametersClient>(this,"/global_costmap/global_costmap");
 command_=create_service<Command>("~/command",[this](Command::Request::SharedPtr q,Command::Response::SharedPtr r){r->boot_id=boot_;r->operation_id=q->command_id;try{
 if(!enabled_)throw std::runtime_error("MAP.DEPLOYMENT_NOT_CONFIGURED");
 if(q->schema_version!=1||q->robot_id!=robot_||q->expected_boot_id!=boot_||q->operation!="load_session"||q->payload_json.size()>16*1024*1024)throw std::runtime_error("REQUEST.CONTEXT_MISMATCH");
 if(requests_.count(q->command_id)){if(requests_.at(q->command_id)!=q->payload_json)throw std::runtime_error("REQUEST.CONFLICT");r->accepted=true;r->state=state_;return;}
 if(requests_.size()>=128||request_bytes_+q->payload_json.size()>16*1024*1024||(materialized_.valid()&&materialized_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)||(state_!="IDLE"&&state_!="FAILED"&&state_!="VERIFYING")||pending_||rpc_pending_)throw std::runtime_error("MAP.ADAPTER_BUSY");
 auto p=Json::parse(q->payload_json);if(p.at("attempt_id")!=q->command_id)throw std::runtime_error("REQUEST.CONTEXT_MISMATCH");
 if(!stopped()||!lifecycle_->service_is_ready())throw std::runtime_error("MAP.PREREQUISITE_MISSING");
 if(child_<0&&external())throw std::runtime_error("MAP.EXTERNAL_SLAM_OWNED_BY_OTHER");
 attempt_=q->command_id;transaction_=p.at("transaction_id");target_=p.at("target");requests_[attempt_]=q->payload_json;request_bytes_+=q->payload_json.size();accepted_at_=Clock::now();++generation_;reset_=false;started_=false;config_checked_=false;state_="MATERIALIZING";reason_.clear();deadline_=Clock::now()+std::chrono::seconds(120);
 materialized_=std::async(std::launch::async,[target=target_,assets=assets_,root=runtime_,attempt=attempt_]{return materializeSession(target,assets,root,attempt);});r->accepted=true;r->state="ACCEPTED";
 }catch(const std::exception & e){r->accepted=false;r->reason_code=e.what();r->state="REJECTED";}});
 timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{try{tick();}catch(const std::exception & e){fail(e.what());publish();}});
 }
 ~VoxelSessionAdapter()override{if(child_>0)kill(-child_,SIGINT);if(lock_>=0){flock(lock_,LOCK_UN);close(lock_);}}
};
}
#ifndef ASTRIBOT_VOXEL_ADAPTER_NO_MAIN
int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<astribot_map_manager::VoxelSessionAdapter>());rclcpp::shutdown();}
#endif
