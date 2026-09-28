#pragma once
#include "astribot_s1_robot_geometry/filled_collision.hpp"
namespace astribot_s1_path_tracking {
// The caller holds the costmap mutex. Check the filled live footprint through
// the whole remaining turn, including the space between angular samples.
inline bool cornerRotationClear(nav2_costmap_2d::Costmap2D & map,
  const astribot_s1_robot_geometry::Polygon & footprint,
  double x, double y, double yaw, double target_yaw)
{
  if (!std::isfinite(yaw) || !std::isfinite(target_yaw) ||
    !astribot_s1_robot_geometry::convex(footprint)) {return false;}
  const double turn = std::remainder(target_yaw - yaw, 2. * M_PI);
  const double swept = astribot_s1_robot_geometry::radius(footprint) * std::abs(turn);
  const double resolution = map.getResolution();
  if (!std::isfinite(swept) || !std::isfinite(resolution) || resolution <= 0. ||
    swept / (resolution * .5) > 10000.) {return false;}
  const int steps = std::max(1, static_cast<int>(std::ceil(swept / (resolution * .5))));
  for (int i = 0; i <= steps; ++i) {
    if (astribot_s1_robot_geometry::collision(
        map, footprint, x, y, yaw + turn * i / steps, swept / (2. * steps))) {return false;}
  }
  return true;
}
inline bool cornerCommandClear(nav2_costmap_2d::Costmap2D & map,
  const astribot_s1_robot_geometry::Polygon & footprint,
  double x,double y,double yaw,double vx,double vy,double wz)
{
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz) ||
    !std::isfinite(yaw) || !astribot_s1_robot_geometry::convex(footprint)) {return false;}
  const double motion=std::hypot(vx,vy)+astribot_s1_robot_geometry::radius(footprint)*std::abs(wz);
  const double resolution=map.getResolution();
  if (!std::isfinite(motion) || resolution<=0. || motion/(resolution*.5)>10000.) {return false;}
  const int steps=std::max(1,static_cast<int>(std::ceil(motion/(resolution*.5))));
  // Same one-second command/feedback horizon as the existing Arrival sweep.
  for (int i=0;i<=steps;++i) {
    const double t=double(i)/steps;
    const double a=std::abs(wz)<1e-10?t:std::sin(wz*t)/wz;
    const double b=std::abs(wz)<1e-10?0.:(1.-std::cos(wz*t))/wz;
    const double dx=a*vx-b*vy,dy=b*vx+a*vy;
    if (astribot_s1_robot_geometry::collision(map,footprint,
        x+std::cos(yaw)*dx-std::sin(yaw)*dy,
        y+std::sin(yaw)*dx+std::cos(yaw)*dy,yaw+wz*t,motion/(2.*steps))) {return false;}
  }
  return true;
}

}
