#pragma once
#include "astribot_navigation_zones/client.hpp"
#include <nav2_costmap_2d/layer.hpp>
namespace astribot_navigation_zones {
class ZoneLayer:public nav2_costmap_2d::Layer {
 Client client_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ack_;std::string consumer_;
public:
 void onInitialize()override;
 void updateBounds(double,double,double,double*,double*,double*,double*)override;
 void updateCosts(nav2_costmap_2d::Costmap2D&,int,int,int,int)override;
 void reset()override{current_=false;}
 bool isClearable()override{return false;}
};
}
