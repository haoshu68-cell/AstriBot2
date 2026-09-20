// Explicit graphical smoke test: isolated DDS domain, no navigation requests.
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_rendering/render_window.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include <QTableWidget>
#include <QMouseEvent>
#include <QCheckBox>
#include <QScreen>
#include <fstream>
#include <thread>
#include <iostream>
#include <cmath>
int main(int argc,char ** argv) {
 QApplication app(argc,argv);rclcpp::init(argc,argv);
 auto ros=std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("route_viewport_smoke");
 auto observer=std::make_shared<rclcpp::Node>("route_viewport_observer");
 std::vector<geometry_msgs::msg::PoseStamped> points;
 auto sub=observer->create_subscription<geometry_msgs::msg::PoseStamped>("/operator/route_point",10,
  [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr p){points.push_back(*p);});
 rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(observer);
 const char * config="/tmp/astribot_route_viewport_smoke.rviz";
 {std::ofstream file(config);file<<R"(Panels:
  - Class: astribot_operator_station/WorkstationPanel
    Name: Loop route test
Visualization Manager:
  Class: ""
  Global Options:
    Fixed Frame: map
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
    - Class: rviz_default_plugins/MarkerArray
      Name: Route draft
      Enabled: true
      Topic:
        Value: /operator/route_preview
        Durability Policy: Transient Local
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: astribot_operator_station/RoutePointTool
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Scale: 80
      X: 0
      Y: 0
      Angle: 0
Window Geometry:
  Width: 1200
  Height: 850
)";}
 int result=0;
 try {
  rviz_common::VisualizationFrame frame(ros);frame.setApp(&app);frame.initialize(ros,config);frame.show();
  auto manager=frame.getManager();auto panel=manager->getRenderPanel();
  auto spin=[&](int ms){auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);while(std::chrono::steady_clock::now()<end){app.processEvents();executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));}};
  spin(1200);
  auto tools=manager->getToolManager();rviz_common::Tool * selected=nullptr;
  for(int i=0;i<tools->numTools();++i)if(tools->getTool(i)->getClassId()=="astribot_operator_station/RoutePointTool")selected=tools->getTool(i);
  if(!selected)throw std::runtime_error("Route tool not loaded");
  tools->setCurrentTool(selected);
  auto table=frame.findChild<QTableWidget *>("route_points");if(!table)throw std::runtime_error("Route panel missing");
  auto event=[&](QEvent::Type type,QPointF at,Qt::MouseButton button,Qt::MouseButtons buttons){QMouseEvent mouse(type,at,button,buttons,Qt::NoModifier);QApplication::sendEvent(panel,&mouse);app.processEvents();};
  const double cx=panel->width()/2.,cy=panel->height()/2.;
  event(QEvent::MouseButtonPress,{cx-70,cy},Qt::LeftButton,Qt::LeftButton);
  event(QEvent::MouseMove,{cx-20,cy},Qt::NoButton,Qt::LeftButton);
  event(QEvent::MouseButtonRelease,{cx-20,cy},Qt::LeftButton,Qt::NoButton);spin(250);
  if(points.size()!=1||table->rowCount()!=1||tools->getCurrentTool()!=selected)throw std::runtime_error("First pose/continuous selection failed");
  event(QEvent::MouseButtonPress,{cx+50,cy+60},Qt::LeftButton,Qt::LeftButton);
  event(QEvent::MouseMove,{cx+50,cy+10},Qt::NoButton,Qt::LeftButton);
  event(QEvent::MouseButtonRelease,{cx+50,cy+10},Qt::LeftButton,Qt::NoButton);spin(350);
  if(points.size()!=2||table->rowCount()!=2||tools->getCurrentTool()!=selected)throw std::runtime_error("Second pose/continuous selection failed");
  const auto yaw=[](const auto & p){return 2*std::atan2(p.pose.orientation.z,p.pose.orientation.w);};
  if(std::hypot(points[1].pose.position.x-points[0].pose.position.x,points[1].pose.position.y-points[0].pose.position.y)<.1||std::abs(std::remainder(yaw(points[0])-yaw(points[1]),2*M_PI))<.5)throw std::runtime_error("Position or dragged orientation did not change");
  if(!frame.screen()->grabWindow(frame.winId()).save("/tmp/astribot_route_viewport_smoke.png"))throw std::runtime_error("Screenshot failed");
  std::cout<<"PASS: real Ogre viewport, two drag poses, continuous tool, two panel rows; no start request\n";
  manager->stopUpdate();
 }catch(const std::exception & e){std::cerr<<e.what()<<'\n';result=1;}
 executor.remove_node(observer);rclcpp::shutdown();return result;
}
