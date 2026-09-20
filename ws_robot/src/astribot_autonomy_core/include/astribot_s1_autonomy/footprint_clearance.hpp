#ifndef ASTRIBOT_S1_AUTONOMY__FOOTPRINT_CLEARANCE_HPP_
#define ASTRIBOT_S1_AUTONOMY__FOOTPRINT_CLEARANCE_HPP_
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>
#include "astribot_s1_autonomy/frontier_search.hpp"

namespace astribot_s1_autonomy {
using PlanarFootprint = std::vector<std::array<double, 2>>;

inline bool validGoalFootprint(const PlanarFootprint & polygon)
{
  if (polygon.size()<3 || polygon.size()>32) {return false;}
  double winding=0;
  for (std::size_t i=0;i<polygon.size();++i) {
    const auto & a=polygon[i];const auto & b=polygon[(i+1)%polygon.size()];
    if (!std::isfinite(a[0]) || !std::isfinite(a[1]) || std::hypot(a[0],a[1])>5.) {return false;}
    const double ex=b[0]-a[0],ey=b[1]-a[1];
    if (std::hypot(ex,ey)<1e-8) {return false;}
    const double origin_side=ex*(-a[1])-ey*(-a[0]);
    if (std::abs(origin_side)<1e-8) {return false;}
    if (winding==0) {winding=origin_side;}
    if (origin_side*winding<=0) {return false;}
    for (std::size_t j=0;j<polygon.size();++j) {
      const double side=ex*(polygon[j][1]-a[1])-ey*(polygon[j][0]-a[0]);
      if (side*winding<-1e-10) {return false;}
    }
  }
  return true;
}

// Test the complete convex footprint against intersecting grid squares, including
// boundary cells and interior holes. Margin expands each cell conservatively.
inline bool knownFreeFootprint(const GridMap & map, double x, double y, double yaw,
  const PlanarFootprint & body, double margin, int free_threshold)
{
  if (!map.consistent() || !std::isfinite(map.resolution) || map.resolution<=0 ||
    !std::isfinite(map.origin_x) || !std::isfinite(map.origin_y) ||
    !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
    !std::isfinite(margin) || margin<0 ||
    map.width>static_cast<unsigned>(std::numeric_limits<int>::max()) ||
    map.height>static_cast<unsigned>(std::numeric_limits<int>::max()) ||
    !validGoalFootprint(body)) {return false;}
  PlanarFootprint polygon;
  double min_x=INFINITY,max_x=-INFINITY,min_y=INFINITY,max_y=-INFINITY;
  const double c=std::cos(yaw),s=std::sin(yaw);
  for (const auto & point:body) {
    const double px=x+c*point[0]-s*point[1],py=y+s*point[0]+c*point[1];
    polygon.push_back({px,py});
    min_x=std::min(min_x,px);max_x=std::max(max_x,px);
    min_y=std::min(min_y,py);max_y=std::max(max_y,py);
  }
  min_x-=margin;max_x+=margin;min_y-=margin;max_y+=margin;
  if (min_x<=map.origin_x || min_y<=map.origin_y ||
    max_x>=map.origin_x+map.width*map.resolution ||
    max_y>=map.origin_y+map.height*map.resolution) {return false;}
  const int x0=static_cast<int>(std::floor(std::nextafter(
      (min_x-map.origin_x)/map.resolution,-INFINITY)));
  const int x1=static_cast<int>(std::floor((max_x-map.origin_x)/map.resolution));
  const int y0=static_cast<int>(std::floor(std::nextafter(
      (min_y-map.origin_y)/map.resolution,-INFINITY)));
  const int y1=static_cast<int>(std::floor((max_y-map.origin_y)/map.resolution));
  if (int64_t(x1-x0+1)*(y1-y0+1)>20000) {return false;}
  const double half=map.resolution*.5+margin;
  for (int my=y0;my<=y1;++my) for (int mx=x0;mx<=x1;++mx) {
    const auto value=map.data[map.index(mx,my)];
    if (value>=0 && value<=free_threshold) {continue;}
    const double cx=map.worldX(mx),cy=map.worldY(my);
    bool separated=false;
    for (std::size_t edge=0;edge<polygon.size();++edge) {
      const auto & a=polygon[edge];const auto & b=polygon[(edge+1)%polygon.size()];
      const double ax=-(b[1]-a[1]),ay=b[0]-a[0];
      double lo=INFINITY,hi=-INFINITY;
      for (const auto & point:polygon) {
        const double projection=ax*(point[0]-cx)+ay*(point[1]-cy);
        lo=std::min(lo,projection);hi=std::max(hi,projection);
      }
      const double radius=half*(std::abs(ax)+std::abs(ay));
      if (lo>radius+1e-10 || hi<-radius-1e-10) {separated=true;break;}
    }
    if (!separated) {return false;}
  }
  return true;
}
}  // namespace astribot_s1_autonomy
#endif
