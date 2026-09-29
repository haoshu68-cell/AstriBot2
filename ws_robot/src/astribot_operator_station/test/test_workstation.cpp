#include "astribot_operator_station/workstation_panel.hpp"
#include "astribot_operator_station/replay_policy.hpp"
#include <gtest/gtest.h>
#include <QApplication>
#include <QMessageBox>
#include <QTimer>
using namespace astribot_operator_station;
TEST(Workstation, StartupWithoutBackendKeepsControlDisabled) {
 int argc=1;char name[]="workstation_startup_test";char * argv[]={name,nullptr};QApplication app(argc,argv);rclcpp::init(0,nullptr);
 {
  WorkstationPanel panel;
  const auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(700);
  while(std::chrono::steady_clock::now()<until){app.processEvents();std::this_thread::sleep_for(std::chrono::milliseconds(10));}
  ASSERT_EQ(panel.findChildren<QPushButton *>("start_navigation").size(),1);
  for(auto b:panel.findChildren<QPushButton *>())EXPECT_NE(b->text(),QString::fromUtf8("按列表循环导航"));
  for(auto b:panel.findChildren<QPushButton *>())
   if(b->text().startsWith("申请控制权")||b->text().startsWith("导航到选中")||b->text().startsWith("开始仿真搬运"))EXPECT_FALSE(b->isEnabled());
 }
 rclcpp::shutdown();
}
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
 state["map_catalog"]={{"quality","VALID"},{"value",{{"boot_id","catalog"},{"revision",1},{"active_map",{{"map_id","lab"},{"version","v1"}}},{"active_scene",nullptr},{"scene_ready",true},{"state","READY"},{"motion_blocked",false},{"idle_evidence",true},{"transaction",nullptr},{"stations",Json::object()},{"maps",{{"lab",{{"map_id","lab"},{"floor",1},{"version","v1"}}}}},{"devices",Json::object()},{"scenes",Json::object()}}}};
 ASSERT_TRUE(spin([&]{return panel.findChild<QPushButton *>("pick_pose")->isEnabled()&&panel.findChild<QTableWidget *>("route_points")->rowCount()==0;}));
 panel.findChild<QPushButton *>("pick_pose")->click();points->publish(p);
 ASSERT_TRUE(spin([&]{return panel.findChild<QLabel *>("device_coordinates")->text().startsWith("设备：X 2.000 m");}));
 EXPECT_EQ(panel.findChild<QTableWidget *>("route_points")->rowCount(),0);
 state["boot_id"]="boot2";state["control_state"]="OBSERVER";
 EXPECT_TRUE(spin([&]{return !button("导航到选中")->isEnabled()&&button("申请控制权")->isEnabled();}));
 panel.resize(900,1100);panel.show();app.processEvents();EXPECT_TRUE(panel.grab().save("/tmp/astribot_workstation.png"));
 }
 executor.remove_node(node);rclcpp::shutdown();
}
