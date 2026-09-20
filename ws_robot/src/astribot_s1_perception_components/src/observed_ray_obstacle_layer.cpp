#include "astribot_s1_autonomy/observed_ray.hpp"
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace astribot_s1_autonomy
{
class ObservedRayObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
protected:
  void raytraceFreespace(const nav2_costmap_2d::Observation &observation,
                        double *min_x,double *min_y,double *max_x,double *max_y) override
  {
    ObstacleLayer::raytraceFreespace(observation,min_x,min_y,max_x,max_y);
    const double resolution=getResolution(),ox=getOriginX(),oy=getOriginY();
    const auto &origin=observation.origin_;
    unsigned int mx,my;
    if(!worldToMap(origin.x,origin.y,mx,my))return;
    const auto &cloud=*observation.cloud_;
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud,"x"),y(cloud,"y");
    for(;x!=x.end();++x,++y) {
      observedRay((origin.x-ox)/resolution,(origin.y-oy)/resolution,
                  (*x-ox)/resolution,(*y-oy)/resolution,
                  observation.raytrace_min_range_/resolution,
                  observation.raytrace_max_range_/resolution,
                  getSizeInCellsX(),getSizeInCellsY(),[&](int cx,int cy) {
        if(getCost(cx,cy)==nav2_costmap_2d::FREE_SPACE)return;
        setCost(cx,cy,nav2_costmap_2d::FREE_SPACE);
        *min_x=std::min(*min_x,ox+cx*resolution);
        *min_y=std::min(*min_y,oy+cy*resolution);
        *max_x=std::max(*max_x,ox+(cx+1)*resolution);
        *max_y=std::max(*max_y,oy+(cy+1)*resolution);
      });
    }
    // ObstacleLayer marks this cycle's current hits after all clearing passes.
  }
};
}
PLUGINLIB_EXPORT_CLASS(astribot_s1_autonomy::ObservedRayObstacleLayer,nav2_costmap_2d::Layer)
