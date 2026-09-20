#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
namespace astribot_operator_station {
class RoutePointTool : public rviz_default_plugins::tools::PoseTool {
 rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
public:
 void onInitialize() override {
   PoseTool::onInitialize();setName("添加路线点");
   publisher_=context_->getRosNodeAbstraction().lock()->get_raw_node()->create_publisher<geometry_msgs::msg::PoseStamped>("/operator/route_point",10);
 }
 int processMouseEvent(rviz_common::ViewportMouseEvent & event) override {
   const int result=PoseTool::processMouseEvent(event);
   if(result & Finished) {deactivate();activate();}
   return result & ~Finished;
 }
 void onPoseSet(double x,double y,double theta) override {
   geometry_msgs::msg::PoseStamped pose;
   pose.header.frame_id=context_->getFixedFrame().toStdString();pose.pose.position.x=x;pose.pose.position.y=y;
   pose.pose.orientation.z=std::sin(theta/2);pose.pose.orientation.w=std::cos(theta/2);
   publisher_->publish(pose);
 }
};
}
PLUGINLIB_EXPORT_CLASS(astribot_operator_station::RoutePointTool,rviz_common::Tool)
