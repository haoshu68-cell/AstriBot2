#include <rviz_common/tool.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace astribot_operator_station {
class ZonePointTool : public rviz_common::Tool {
 rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
public:
 void onInitialize() override{setName("绘制虚拟墙与禁区");publisher_=context_->getRosNodeAbstraction().lock()->get_raw_node()->create_publisher<geometry_msgs::msg::PointStamped>("/operator/zone_point",10);}
 void activate()override{}void deactivate()override{}
 int processMouseEvent(rviz_common::ViewportMouseEvent &event) override{
   if(!event.leftDown()||context_->getFixedFrame()!="map"||!event.panel)return Render;
   rviz_rendering::ViewportProjectionFinder finder;
   const auto hit=finder.getViewportPointProjectionOnXYPlane(event.panel->getRenderWindow(),event.x,event.y);
   if(hit.first){const auto p=hit.second;geometry_msgs::msg::PointStamped m;m.header.frame_id="map";m.point.x=p.x;m.point.y=p.y;publisher_->publish(m);}return Render;
 }
};
}
PLUGINLIB_EXPORT_CLASS(astribot_operator_station::ZonePointTool,rviz_common::Tool)
