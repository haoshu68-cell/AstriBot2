#include "astribot_operator_station/workstation_panel.hpp"
#include "astribot_operator_station/replay_policy.hpp"
#include <gtest/gtest.h>
#include <QApplication>
#include <QMessageBox>
#include <QTimer>
using namespace astribot_operator_station;
TEST(ReplayPolicy, ClosedAllowlistRejectsMotionAndSpoofedTypes) {
 EXPECT_TRUE(replay_allowed("/tf","tf2_msgs/msg/TFMessage"));
 EXPECT_TRUE(replay_allowed("/map","nav_msgs/msg/OccupancyGrid"));
 EXPECT_FALSE(replay_allowed("/cmd_vel","geometry_msgs/msg/Twist"));
 EXPECT_FALSE(replay_allowed("/navigate_to_pose/_action/feedback","nav2_msgs/action/NavigateToPose_FeedbackMessage"));
 EXPECT_FALSE(replay_allowed("/map","geometry_msgs/msg/Twist"));
 EXPECT_FALSE(replay_allowed("/arm_controller/joint_trajectory","trajectory_msgs/msg/JointTrajectory"));
}
TEST(Workstation, LeaseRequiredAndRestartInvalidatesControl) {
 int argc=1;char name[]="workstation_test";char * argv[]={name,nullptr};QApplication app(argc,argv);rclcpp::init(0,nullptr);
 auto node=std::make_shared<rclcpp::Node>("fake_gateway");using Command=astribot_operator_msgs::srv::OperatorCommand;using Json=nlohmann::json;
 Json state={{"schema_version",1},{"boot_id","boot1"},{"robot_id","robot"},{"control_state","OBSERVER"},{"control_owner",""},
  {"capabilities",{{"navigate",true},{"loop_route",false},{"new_mapping_session",false}}}};
 auto publisher=node->create_publisher<std_msgs::msg::String>("/operator_backend/status",rclcpp::QoS(1).transient_local());
 auto points=node->create_publisher<geometry_msgs::msg::PoseStamped>("/operator/route_point",10);
 auto timer=node->create_wall_timer(std::chrono::milliseconds(50),[&]{std_msgs::msg::String m;m.data=state.dump();publisher->publish(m);});
 state["arm_preview"]={{"state","EMPTY"},{"planning_available",true}};
 int nav=0,acquire=0,plans=0;
 auto server=node->create_service<Command>("/operator_backend/command",[&](Command::Request::SharedPtr q,Command::Response::SharedPtr r){
  r->boot_id=state.at("boot_id");r->accepted=true;r->state="ACCEPTED";r->result_json="{}";r->operation_id=q->command_id;
  if(q->operation=="acquire"){++acquire;r->result_json=R"({"lease_id":"token","ttl_sec":6})";state["control_state"]="HELD";}
  if(q->operation=="arm_plan"){++plans;EXPECT_EQ(q->lease_id,"token");EXPECT_EQ(Json::parse(q->payload_json).at("group"),"arm_left");EXPECT_EQ(Json::parse(q->payload_json).at("named_target"),"transport_compact");}
  if(q->operation=="navigate"){++nav;EXPECT_EQ(q->lease_id,"token");EXPECT_EQ(q->robot_id,"robot");EXPECT_EQ(Json::parse(q->payload_json).at("x"),2);}
 });
 rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
 {
 WorkstationPanel panel;auto button=[&](const QString & prefix){for(auto b:panel.findChildren<QPushButton *>())if(b->text().startsWith(prefix))return b;throw std::runtime_error("button");};
 auto spin=[&](auto condition){auto until=std::chrono::steady_clock::now()+std::chrono::seconds(5);while(std::chrono::steady_clock::now()<until){app.processEvents();executor.spin_some();if(condition())return true;std::this_thread::sleep_for(std::chrono::milliseconds(5));}return false;};
 ASSERT_TRUE(spin([&]{return button("申请控制权")->isEnabled();}));EXPECT_FALSE(button("导航到选中")->isEnabled());
 button("申请控制权")->click();ASSERT_TRUE(spin([&]{return acquire==1&&button("导航到选中")->isEnabled();}));
 EXPECT_FALSE(button("执行（")->isEnabled());
 ASSERT_TRUE(spin([&]{return button("规划收臂")->isEnabled();}));
 QTimer::singleShot(20,[]{for(auto w:QApplication::topLevelWidgets())if(auto box=qobject_cast<QMessageBox *>(w))box->button(QMessageBox::Yes)->click();});
 button("规划收臂")->click();ASSERT_TRUE(spin([&]{return plans==1;}));
 ASSERT_TRUE(spin([&]{return button("导航到选中")->isEnabled();}));
 geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=2;p.pose.orientation.w=1;points->publish(p);
 ASSERT_TRUE(spin([&]{return panel.findChild<QTableWidget *>()->rowCount()==1;}));
 QTimer::singleShot(20,[]{for(auto w:QApplication::topLevelWidgets())if(auto box=qobject_cast<QMessageBox *>(w))box->button(QMessageBox::Yes)->click();});
 button("导航到选中")->click();ASSERT_TRUE(spin([&]{return nav==1;}));
 state["boot_id"]="boot2";state["control_state"]="OBSERVER";
 EXPECT_TRUE(spin([&]{return !button("导航到选中")->isEnabled()&&button("申请控制权")->isEnabled();}));
 panel.resize(900,1100);panel.show();app.processEvents();EXPECT_TRUE(panel.grab().save("/tmp/astribot_workstation.png"));
 }
 executor.remove_node(node);rclcpp::shutdown();
}
