#include "astribot_operator_station/panel.hpp"
#include <gtest/gtest.h>
#include <QApplication>
#include <QMessageBox>
#include <QTimer>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <astribot_operator_msgs/srv/exploration_command.hpp>
#include <QLabel>
using astribot_operator_station::OperatorPanel;
TEST(Panel, FakeNavigationAndExplorationControls) {
 int argc=1;char name[]="operator_panel_test";char * argv[]={name,nullptr};
 QApplication app(argc,argv);
 const auto prefix=std::string("/operator_fake_")+std::to_string(getpid());
 const auto nav_remap=std::string("/navigate_to_pose:=")+prefix+"/navigate_to_pose";
 const auto pause_remap=std::string("/exploration_coordinator_node/command:=")+prefix+"/pause";
 const char * ros_args[]={"test","--ros-args","-r",nav_remap.c_str(),"-r",pause_remap.c_str()};
 rclcpp::init(6,ros_args);
 auto node=std::make_shared<rclcpp::Node>("operator_fake_server");
 using Nav=nav2_msgs::action::NavigateToPose;using GH=rclcpp_action::ServerGoalHandle<Nav>;
 std::shared_ptr<GH> goal;unsigned goals=0,cancels=0,pauses=0;
 auto server=rclcpp_action::create_server<Nav>(node,"/navigate_to_pose",
  [&](const auto &,auto){++goals;return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
  [&](auto){++cancels;return rclcpp_action::CancelResponse::ACCEPT;},
  [&](auto h){goal=h;});
 using Command=astribot_operator_msgs::srv::ExplorationCommand;
 nlohmann::json state={{"schema_version",1},{"boot_id","fake-boot"},{"revision",1},{"reason_code","EXPLORATION.RUNNING"},
   {"progress","EXPLORING"},{"manual_pause",false},{"session_ending",false},{"cancel_pending",false},
   {"can_pause",true},{"can_resume",false},{"can_cancel_save",true}};
 nlohmann::json mapping={{"state","IDLE"},{"directory",""}};
 auto status=node->create_publisher<std_msgs::msg::String>("/exploration_coordinator_node/operator_status",rclcpp::QoS(1).transient_local());
 auto map_status=node->create_publisher<std_msgs::msg::String>("/mapping_session/status",rclcpp::QoS(1).transient_local());
 bool publish_status=true;
 auto heartbeat=node->create_wall_timer(std::chrono::milliseconds(50),[&]{if(!publish_status)return;
   std_msgs::msg::String m;m.data=state.dump();status->publish(m);m.data=mapping.dump();map_status->publish(m);});
 auto trigger=node->create_service<Command>("/exploration_coordinator_node/command",[&](Command::Request::SharedPtr req,Command::Response::SharedPtr response){
   EXPECT_EQ(req->expected_boot_id,"fake-boot");EXPECT_EQ(req->expected_revision,1u);
   EXPECT_EQ(req->operation,"pause");++pauses;response->accepted=true;response->boot_id="fake-boot";response->reason_code="COMMAND.ACCEPTED";
   state["revision"]=2;state["manual_pause"]=true;state["can_pause"]=false;state["can_resume"]=true;state["reason_code"]="EXPLORATION.PAUSED";
 });
 auto tick=node->create_wall_timer(std::chrono::milliseconds(10),[&]{if(goal && goal->is_canceling()){goal->canceled(std::make_shared<Nav::Result>());goal.reset();}});
 rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
 {
 OperatorPanel panel;
 auto find=[&](const QString & text)->QPushButton *{for(auto b:panel.findChildren<QPushButton *>())if(b->text().startsWith(text))return b;throw std::runtime_error("Button missing");};
 auto send=find("确认并提交");auto cancel=find("取消本面板");auto pause=find("暂停探索");
 auto spin=[&](auto condition){auto end=std::chrono::steady_clock::now()+std::chrono::seconds(5);do {app.processEvents();executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));if(condition())return true;}while(std::chrono::steady_clock::now()<end);return false;};
 EXPECT_TRUE(spin([&]{return !send->isEnabled();}));
 panel.findChild<QCheckBox *>()->setChecked(true);
 EXPECT_TRUE(spin([&]{return send->isEnabled()&&pause->isEnabled();}));
 QTimer::singleShot(20,[]{for(auto w:QApplication::topLevelWidgets())if(auto box=qobject_cast<QMessageBox *>(w))box->button(QMessageBox::Yes)->click();});
 send->click();EXPECT_TRUE(spin([&]{return goals==1&&cancel->isEnabled();}));
 cancel->click();EXPECT_TRUE(spin([&]{return cancels==1&&send->isEnabled();}));
 pause->click();EXPECT_TRUE(spin([&]{return pauses==1;}));
 EXPECT_EQ(goals,1u);EXPECT_EQ(cancels,1u);
 auto resume=find("恢复当前探索");auto end=find("取消探索并保存");auto retry=find("重试地图保存");
 auto summary=panel.findChild<QLabel *>("exploration_summary");
 EXPECT_TRUE(spin([&]{return resume->isEnabled()&&!pause->isEnabled();}));
 state["revision"]=3;state["cancel_pending"]=true;state["can_resume"]=false;
 EXPECT_TRUE(spin([&]{return !resume->isEnabled()&&summary->text().contains("等待导航取消终态");}));
 state["revision"]=4;state["cancel_pending"]=false;state["reason_code"]="EXPLORATION.NOT_READY";state["progress"]="INSUFFICIENT_KNOWN_MAP";
 EXPECT_TRUE(spin([&]{return summary->text().contains("不是建图完成")&&!resume->isEnabled();}));
 state["revision"]=5;state["reason_code"]="EXPLORATION.RUNNING";state["manual_pause"]=false;state["progress"]="UNREACHABLE_FRONTIERS";
 EXPECT_TRUE(spin([&]{return summary->text().contains("有边界但不可达");}));
 state["revision"]=6;state["session_ending"]=true;state["can_cancel_save"]=false;mapping["state"]="WAIT_STOP";
 EXPECT_TRUE(spin([&]{return !end->isEnabled()&&!resume->isEnabled()&&summary->text().contains("收尾阶段");}));
 mapping["state"]="FAILED";mapping["detail"]="disk unavailable";
 EXPECT_TRUE(spin([&]{return summary->text().contains("地图保存失败")&&!resume->isEnabled();}));
 EXPECT_FALSE(retry->isEnabled()); // A failed status alone does not invent a retry service.
 mapping["state"]="SAVED";mapping["directory"]="/tmp/fake_map";mapping["exploration_outcome"]="CANCELED_PARTIAL";
 EXPECT_TRUE(spin([&]{return summary->text().contains("CANCELED_PARTIAL")&&!resume->isEnabled();}));
 state["revision"]=7;state["session_ending"]=false;state["manual_pause"]=true;state["can_resume"]=true;state["can_cancel_save"]=true;
 state["reason_code"]="EXPLORATION.PAUSED";mapping["state"]="IDLE";
 EXPECT_TRUE(spin([&]{return resume->isEnabled();}));
 QTimer::singleShot(20,[&]{
   state["boot_id"]="new-fake-boot";state["revision"]=1;
   std_msgs::msg::String message;message.data=state.dump();status->publish(message);
   QTimer::singleShot(200,[]{for(auto w:QApplication::topLevelWidgets())if(auto box=qobject_cast<QMessageBox *>(w))box->button(QMessageBox::Yes)->click();});
 });
 resume->click();
 EXPECT_TRUE(spin([&]{return panel.findChild<QPlainTextEdit *>()->toPlainText().contains("确认期间探索状态变化");}));
 EXPECT_EQ(pauses,1u);
 state["readiness_detail"]=42;
 EXPECT_TRUE(spin([&]{return panel.findChild<QPlainTextEdit *>()->toPlainText().contains("状态协议错误");}));
 EXPECT_FALSE(resume->isEnabled());state["readiness_detail"]="";
 EXPECT_TRUE(spin([&]{return resume->isEnabled();}));
 publish_status=false;
 EXPECT_TRUE(spin([&]{return summary->text().contains("过期")&&!end->isEnabled()&&!pause->isEnabled();}));
 EXPECT_EQ(pauses,1u);

 EXPECT_TRUE(spin([&]{return panel.findChild<QPlainTextEdit *>()->toPlainText().contains("导航取消完成");}));
 panel.resize(680,1000);panel.show();app.processEvents();
 EXPECT_TRUE(panel.grab().save("/tmp/astribot_operator_panel_test.png"));
 }
 executor.remove_node(node);rclcpp::shutdown();
}
