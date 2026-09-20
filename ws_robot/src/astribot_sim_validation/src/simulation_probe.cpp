#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <set>
#include <thread>
#include <cmath>
using Json=nlohmann::json;
class SimulationProbe : public rclcpp::Node {
 using Steady=std::chrono::steady_clock;
 struct Stream {uint64_t samples=0,advances=0;int64_t last=-1;Steady::time_point at{},advanced_at{};};
 std::map<std::string,Stream> streams_;
 std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
 rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controllers_;
 Json controller_state_=Json::object();Steady::time_point controller_at_{};
 bool pending_=false;
 void sample(const std::string & name,const builtin_interfaces::msg::Time & stamp){
   if(stamp.sec<0||stamp.nanosec>=1000000000u)return;
   auto & s=streams_[name];int64_t t=int64_t(stamp.sec)*1000000000+stamp.nanosec;
   if(t<s.last){s.advances=0;s.samples=0;}if(t>s.last){++s.advances;s.advanced_at=Steady::now();}s.last=t;++s.samples;s.at=Steady::now();
 }
public:
 SimulationProbe():Node("simulation_acceptance_probe"){
   const auto domain=std::getenv("ROS_DOMAIN_ID"),partition=std::getenv("IGN_PARTITION");
   if(!domain||std::string(domain)!="213"||!partition||std::string(partition)!="astribot_operator_validation_213")throw std::runtime_error("Probe requires validation domain 213 and its Gazebo partition");
   subscriptions_.push_back(create_subscription<rosgraph_msgs::msg::Clock>("/clock",rclcpp::SensorDataQoS(),[this](rosgraph_msgs::msg::Clock::ConstSharedPtr m){sample("/clock",m->clock);}));
   subscriptions_.push_back(create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::ConstSharedPtr m){if(m->name.empty()||m->name.size()!=m->position.size())return;for(auto v:m->position)if(!std::isfinite(v))return;sample("/joint_states",m->header.stamp);}));
   subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){if(std::isfinite(m->pose.pose.position.x)&&std::isfinite(m->pose.pose.position.y))sample("/odom",m->header.stamp);}));
   subscriptions_.push_back(create_subscription<sensor_msgs::msg::LaserScan>("/scan_from_cloud",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){if(!m->ranges.empty())sample("/scan_from_cloud",m->header.stamp);}));
   controllers_=create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
 }
 void query(){if(pending_||!controllers_->service_is_ready())return;pending_=true;
   controllers_->async_send_request(std::make_shared<controller_manager_msgs::srv::ListControllers::Request>(),[this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture f){pending_=false;controller_state_=Json::object();for(const auto & c:f.get()->controller)controller_state_[c.name]=c.state;controller_at_=Steady::now();});}
 Json report(){Json out={{"evidence","gazebo_streams_and_ros2_control"},{"domain",213},{"partition","astribot_operator_validation_213"},{"ready",true},{"controllers",controller_state_}};
   for(const auto & name:{"/clock","/joint_states","/odom","/scan_from_cloud"}){const auto & s=streams_[name];bool valid=s.samples>=5&&s.advances>=5&&Steady::now()-s.at<std::chrono::seconds(2)&&Steady::now()-s.advanced_at<std::chrono::seconds(2)&&count_publishers(name)==1;
     out["streams"][name]={{"samples",s.samples},{"advances",s.advances},{"valid",valid}};if(!valid)out["ready"]=false;}
   for(const auto & name:{"joint_state_broadcaster","arm_left_controller","arm_right_controller","gripper_left_controller","gripper_right_controller","head_controller","torso_controller","wheel_effort_controller"})if(controller_state_.value(name,"")!="active")out["ready"]=false;
   if(Steady::now()-controller_at_>std::chrono::seconds(2))out["ready"]=false;
   return out;
 }
};
int main(int argc,char ** argv){rclcpp::init(argc,argv);try{
 auto node=std::make_shared<SimulationProbe>();auto path=node->declare_parameter("report_path",std::string("/tmp/astribot-sim-completion/readiness.json"));
 rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);auto end=std::chrono::steady_clock::now()+std::chrono::seconds(15),next=std::chrono::steady_clock::now();
 while(rclcpp::ok()&&std::chrono::steady_clock::now()<end){executor.spin_some();if(std::chrono::steady_clock::now()>=next){node->query();next+=std::chrono::seconds(1);}std::this_thread::sleep_for(std::chrono::milliseconds(10));}
 auto result=node->report();std::ofstream file(path);file<<result.dump(2)<<'\n';file.flush();if(!file)throw std::runtime_error("Cannot write readiness report");RCLCPP_INFO(node->get_logger(),"%s",result.dump().c_str());rclcpp::shutdown();return result.at("ready").get<bool>()?0:2;
 }catch(const std::exception & e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 3;}}
