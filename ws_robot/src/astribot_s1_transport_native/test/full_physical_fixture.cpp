// Protocol fixture only: no Gazebo, hardware, or controller commands.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <nlohmann/json.hpp>
#include <mutex>
class PhysicalFixture:public rclcpp::Node {
public:
 PhysicalFixture():Node("m1_full_physical_fixture") {
  events_=create_publisher<std_msgs::msg::String>("/fixture/physical_command",10);
  states_=ign_.Advertise<ignition::msgs::StringMsg>("/model/fixture/kinematic_attachment/state");
  world_=ign_.Advertise<ignition::msgs::Pose_V>("/world/fixture_world/pose/info");
  std::function<bool(const ignition::msgs::Pose&,ignition::msgs::Boolean&)> command=[this](const auto &request,auto &reply) {
   std::lock_guard<std::mutex> lock(mutex_);
   if(!initialized_||request.id()!=counter_+1){reply.set_data(false);return true;}
   counter_=request.id();attached_=!request.name().empty();
   std_msgs::msg::String event;event.data=nlohmann::json({{"command",counter_},{"attached",attached_},{"parent",request.name()},
    {"position",{request.position().x(),request.position().y(),request.position().z()}},
    {"quaternion",{request.orientation().x(),request.orientation().y(),request.orientation().z(),request.orientation().w()}}}).dump();
   events_->publish(event);reply.set_data(true);return true;
  };
  if(!ign_.Advertise("/model/fixture/kinematic_attachment/command",command))throw std::runtime_error("FIXTURE_SERVICE_FAILED");
  input_=create_subscription<std_msgs::msg::String>("/fixture/physical_input",10,[this](const std_msgs::msg::String &m){
   const auto value=nlohmann::json::parse(m.data);std::lock_guard<std::mutex> lock(mutex_);
   if(!initialized_){attached_=value.at("initial_attached");initialized_=true;}
   const auto stamp=value.at("stamp_ns").get<int64_t>();
   nlohmann::json state={{"stamp_ns",stamp},{"command_id",counter_},{"attached",attached_},{"error",""},
    {"position_error_m",0.},{"rotation_error_rad",0.},{"actual_world_xyzw",{.1,0.,1.13,0.,0.,0.,1.}}};
   ignition::msgs::StringMsg message;message.set_data(state.dump());states_.Publish(message);
   ignition::msgs::Pose_V poses;poses.mutable_header()->mutable_stamp()->set_sec(stamp/1000000000);poses.mutable_header()->mutable_stamp()->set_nsec(stamp%1000000000);
   auto model=poses.add_pose();model->set_name("fixture_robot");model->set_id(136);model->mutable_position()->set_z(.13);model->mutable_orientation()->set_w(1.);world_.Publish(poses);
  });
 }
private:
 ignition::transport::Node ign_;ignition::transport::Node::Publisher states_,world_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr input_;
 std::mutex mutex_;bool initialized_=false,attached_=false;uint32_t counter_=7;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<PhysicalFixture>());}catch(const std::exception &e){std::fprintf(stderr,"physical_fixture: %s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();}
