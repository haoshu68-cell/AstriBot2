#include "astribot_operator_station/loop_route_panel.hpp"
#include <gtest/gtest.h>
#include <QApplication>
#include <QMessageBox>
#include <QTimer>
#include <nlohmann/json.hpp>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/tool.hpp>
using namespace astribot_operator_station;
TEST(LoopPanel, PointsReorderingStartCancelAndReconnectView) {
 int argc=1;char name[]="loop_panel_test";char * argv[]={name,nullptr};QApplication app(argc,argv);
 rclcpp::init(0,nullptr);
 auto fake=std::make_shared<rclcpp::Node>("fake_loop_backend");
 using Start=astribot_operator_msgs::srv::StartLoopRoute;using Cancel=astribot_operator_msgs::srv::CancelLoopRoute;
 Start::Request captured;int starts=0,cancels=0;std::string canceled_id,boot="test-boot";
 auto start=fake->create_service<Start>("/loop_route_executor/start",[&](Start::Request::SharedPtr r,Start::Response::SharedPtr s){captured=*r;++starts;s->accepted=true;s->boot_id=boot;s->active=true;s->state="NAVIGATING";s->route_id="test/route";s->reason="accepted";});
 auto cancel=fake->create_service<Cancel>("/loop_route_executor/cancel",[&](Cancel::Request::SharedPtr r,Cancel::Response::SharedPtr s){++cancels;canceled_id=r->route_id;s->accepted=true;});
 auto status=fake->create_publisher<std_msgs::msg::String>("/loop_route_executor/status",rclcpp::QoS(1).transient_local());
 auto points=fake->create_publisher<geometry_msgs::msg::PoseStamped>("/operator/route_point",10);
 rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(fake);
 auto publish=[&](bool active){std_msgs::msg::String s;s.data=nlohmann::json({{"boot_id",boot},{"route_id",active?"test/route":""},{"active",active},{"state",active?"NAVIGATING":"IDLE"},{"reason","test"},{"index",0},{"completed_cycles",0},{"dwell_sec",.5},{"waypoints",active?nlohmann::json::array({{{"frame","map"},{"x",2},{"y",0},{"yaw",0}},{{"frame","map"},{"x",1},{"y",0},{"yaw",0}}}):nlohmann::json::array()}}).dump();status->publish(s);};
 {
 LoopRoutePanel panel;
 auto button=[&](const QString & text){for(auto b:panel.findChildren<QPushButton *>())if(b->text().startsWith(text))return b;throw std::runtime_error("Missing button");};
 auto table=panel.findChild<QTableWidget *>("route_points");
 auto spin=[&](auto condition){auto end=std::chrono::steady_clock::now()+std::chrono::seconds(5);do{app.processEvents();executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));if(condition())return true;}while(std::chrono::steady_clock::now()<end);return false;};
 publish(false);EXPECT_TRUE(spin([&]{return points->get_subscription_count()>0;}));
 geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.orientation.w=1;
 for(double x:{1.,2.}){p.pose.position.x=x;points->publish(p);}
 ASSERT_TRUE(spin([&]{return table->rowCount()==2;}));EXPECT_EQ(starts,0);
 table->selectRow(1);button("上移")->click();EXPECT_DOUBLE_EQ(table->item(0,0)->text().toDouble(),2);
 for(auto box:panel.findChildren<QCheckBox *>())if(box->text()=="允许启动循环导航")box->setChecked(true);
 publish(false);ASSERT_TRUE(spin([&]{return button("确认并开始")->isEnabled();}));
 QTimer::singleShot(20,[]{for(auto w:QApplication::topLevelWidgets())if(auto box=qobject_cast<QMessageBox *>(w))box->button(QMessageBox::Yes)->click();});
 button("确认并开始")->click();ASSERT_TRUE(spin([&]{return starts==1;}));
 ASSERT_EQ(captured.waypoints.size(),2u);EXPECT_DOUBLE_EQ(captured.waypoints[0].pose.position.x,2);EXPECT_FALSE(captured.request_id.empty());EXPECT_EQ(captured.expected_boot_id,"test-boot");
 publish(true);ASSERT_TRUE(spin([&]{return button("取消当前")->isEnabled()&&table->editTriggers()==QAbstractItemView::NoEditTriggers;}));
 p.pose.position.x=99;points->publish(p);
 button("取消当前")->click();EXPECT_TRUE(spin([&]{return cancels==1;}));EXPECT_EQ(canceled_id,"test/route");EXPECT_EQ(table->rowCount(),2);
 panel.resize(740,820);panel.show();app.processEvents();EXPECT_TRUE(panel.grab().save("/tmp/astribot_loop_route_panel.png"));
 // A terminal duplicate response must not turn the route back into an active one.
 panel.startReply(QString::fromStdString(nlohmann::json({{"request_id",captured.request_id},{"boot_id",boot},{"active",false},{"accepted",true},{"route_id","test/route"},{"reason","Duplicate request; CANCELED"}}).dump()));
 EXPECT_TRUE(spin([&]{return !button("取消当前")->isEnabled();}));
 boot="new-boot";publish(false);
 EXPECT_TRUE(spin([&]{for(auto box:panel.findChildren<QCheckBox *>())if(box->text()=="允许启动循环导航")return !box->isChecked();return false;}));
 panel.startReply(QString::fromStdString(nlohmann::json({{"request_id",captured.request_id},{"boot_id","test-boot"},{"active",true},{"accepted",true},{"route_id","test/route"},{"reason","late old response"}}).dump()));
 EXPECT_TRUE(spin([&]{return panel.findChild<QPlainTextEdit *>()->toPlainText().contains("忽略旧请求");}));
 EXPECT_FALSE(button("取消当前")->isEnabled());EXPECT_EQ(starts,1);
 // A restart while the modal confirmation is open invalidates that confirmation.
 for(auto box:panel.findChildren<QCheckBox *>())if(box->text()=="允许启动循环导航")box->setChecked(true);
 publish(false);ASSERT_TRUE(spin([&]{return button("确认并开始")->isEnabled();}));
 QTimer::singleShot(20,[&]{
   boot="confirmation-new-boot";publish(false);
   QTimer::singleShot(200,[]{for(auto w:QApplication::topLevelWidgets())if(auto box=qobject_cast<QMessageBox *>(w))box->button(QMessageBox::Yes)->click();});
 });
 button("确认并开始")->click();
 EXPECT_TRUE(spin([&]{return panel.findChild<QPlainTextEdit *>()->toPlainText().contains("确认期间");}));
 EXPECT_EQ(starts,1);
 }
 // A new panel loads the robot's active route and can cancel using its route ID.
 {
 LoopRoutePanel panel;publish(true);
 auto end=std::chrono::steady_clock::now()+std::chrono::seconds(3);
 while(panel.findChild<QTableWidget *>()->rowCount()!=2 && std::chrono::steady_clock::now()<end){app.processEvents();executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));}
 EXPECT_EQ(panel.findChild<QTableWidget *>()->rowCount(),2);
 }
 pluginlib::ClassLoader<rviz_common::Tool> loader("rviz_common","rviz_common::Tool");
 EXPECT_NO_THROW({auto tool=loader.createSharedInstance("astribot_operator_station/RoutePointTool");});
 executor.remove_node(fake);rclcpp::shutdown();
}
