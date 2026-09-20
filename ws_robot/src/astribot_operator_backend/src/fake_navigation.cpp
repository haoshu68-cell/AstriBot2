#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <chrono>
class FakeNavigation : public rclcpp::Node {
 using Nav=nav2_msgs::action::NavigateToPose;using Handle=rclcpp_action::ServerGoalHandle<Nav>;
 rclcpp_action::Server<Nav>::SharedPtr server_;rclcpp::TimerBase::SharedPtr timer_;std::shared_ptr<Handle> goal_;
 std::chrono::steady_clock::time_point since_;
public:FakeNavigation():Node("fake_navigation"){
 declare_parameter("outcome","success");
 server_=rclcpp_action::create_server<Nav>(this,"/operator_fake/navigate_to_pose",
 [this](const auto &,auto){return goal_||get_parameter("outcome").as_string()=="reject"?rclcpp_action::GoalResponse::REJECT:rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
 [](auto){return rclcpp_action::CancelResponse::ACCEPT;},[this](auto h){goal_=h;since_=std::chrono::steady_clock::now();});
 timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{if(!goal_)return;auto mode=get_parameter("outcome").as_string();auto result=std::make_shared<Nav::Result>();
 if(goal_->is_canceling()&&mode!="cancel_unconfirmed"){goal_->canceled(result);goal_.reset();}
 else if(mode!="hold"&&mode!="cancel_unconfirmed"&&std::chrono::steady_clock::now()-since_>std::chrono::seconds(1)){
 if(mode=="success")goal_->succeed(result);else goal_->abort(result);goal_.reset();}
 });
 }
};
int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<FakeNavigation>());rclcpp::shutdown();}
