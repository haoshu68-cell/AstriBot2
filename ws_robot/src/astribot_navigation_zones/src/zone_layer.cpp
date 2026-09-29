#include "astribot_navigation_zones/zone_layer.hpp"
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <rclcpp/create_publisher.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace astribot_navigation_zones {
void ZoneLayer::onInitialize(){auto node=node_.lock();if(!node)throw std::runtime_error("ZONES.NO_NODE");enabled_=true;current_=false;declareParameter("consumer",rclcpp::ParameterValue(std::string()));node->get_parameter(name_+".consumer",consumer_);if(consumer_!="global_costmap"&&consumer_!="local_costmap")throw std::runtime_error("ZONES.INVALID_CONSUMER");client_.init(*node,callback_group_);ack_=rclcpp::create_publisher<std_msgs::msg::String>(node,"/navigation_zones/applied",rclcpp::QoS(10));}
void ZoneLayer::updateBounds(double,double,double,double*minx,double*miny,double*maxx,double*maxy){
 auto m=layered_costmap_->getCostmap();*minx=std::min(*minx,m->getOriginX());*miny=std::min(*miny,m->getOriginY());*maxx=std::max(*maxx,m->getOriginX()+m->getSizeInMetersX()+m->getResolution());*maxy=std::max(*maxy,m->getOriginY()+m->getSizeInMetersY()+m->getResolution());
}
void ZoneLayer::updateCosts(nav2_costmap_2d::Costmap2D&m,int minx,int miny,int maxx,int maxy){
 auto s=client_.get();double tx=0,ty=0,yaw=0;bool valid=bool(s);std::string reason=valid?"":"ZONES.UNAVAILABLE";
 if(valid&&layered_costmap_->getGlobalFrameID()!="map")try{
 auto transform=tf_->lookupTransform("map",layered_costmap_->getGlobalFrameID(),tf2::TimePointZero);auto stamp=rclcpp::Time(transform.header.stamp).seconds();double age=clock_->now().seconds()-stamp;
 if(stamp>0&&(age>.5||age<-.1))throw std::runtime_error("ZONES.TF_STALE");const auto&q=transform.transform.rotation;tx=transform.transform.translation.x;ty=transform.transform.translation.y;yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
 if(!std::isfinite(tx)||!std::isfinite(ty)||!std::isfinite(yaw))throw std::runtime_error("ZONES.TF_INVALID");
 }catch(...){valid=false;reason="ZONES.TF_UNAVAILABLE";}
 const double c=std::cos(yaw),sn=std::sin(yaw),radius=m.getResolution()*std::sqrt(2.)/2;
 minx=std::max(0,minx);miny=std::max(0,miny);maxx=std::min(maxx,int(m.getSizeInCellsX()));maxy=std::min(maxy,int(m.getSizeInCellsY()));
 for(int y=miny;y<maxy;++y)for(int x=minx;x<maxx;++x){double wx,wy;m.mapToWorld(x,y,wx,wy);if(!valid||blocked(s->regions,c*wx-sn*wy+tx,sn*wx+c*wy+ty,radius))m.setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);}
 current_=valid;if(s)ack_->publish(acknowledgement(*s,consumer_,valid,reason));
}
}
PLUGINLIB_EXPORT_CLASS(astribot_navigation_zones::ZoneLayer,nav2_costmap_2d::Layer)
