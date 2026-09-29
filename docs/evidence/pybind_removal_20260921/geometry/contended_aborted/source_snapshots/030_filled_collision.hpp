#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace astribot_s1_robot_geometry {
using Polygon=std::vector<geometry_msgs::msg::Point>;
inline bool convex(const Polygon & p) {
  if(p.size()<3 || p.size()>256) return false;
  double area=0;
  for(size_t i=0;i<p.size();++i) {
    const auto & a=p[i];const auto & b=p[(i+1)%p.size()];
    if(!std::isfinite(a.x)||!std::isfinite(a.y)||std::hypot(b.x-a.x,b.y-a.y)<1e-9) return false;
    area+=a.x*b.y-a.y*b.x;
  }
  if(std::abs(area)<1e-12) return false;
  for(size_t i=0;i<p.size();++i) {
    const auto & a=p[i];const auto & b=p[(i+1)%p.size()];
    for(const auto & q:p) if(((b.x-a.x)*(q.y-a.y)-(b.y-a.y)*(q.x-a.x))*area < -1e-12) return false;
  }
  return true;
}
inline double radius(const Polygon & p) {double r=0;for(auto & q:p) r=std::max(r,std::hypot(q.x,q.y));return r;}
inline bool intersects(const Polygon & p,double lx,double ly,double ux,double uy) {
  for(size_t i=0;i<p.size();++i) {
    auto & a=p[i];auto & b=p[(i+1)%p.size()];double nx=-(b.y-a.y),ny=b.x-a.x;
    double low=1e100,high=-1e100;
    for(auto & v:p) {double d=nx*v.x+ny*v.y;low=std::min(low,d);high=std::max(high,d);}
    double c=nx*(lx+ux)/2+ny*(ly+uy)/2,h=std::abs(nx)*(ux-lx)/2+std::abs(ny)*(uy-ly)/2;
    if(high<c-h-1e-12 || low>c+h+1e-12) return false;
  }
  double minx=1e100,maxx=-1e100,miny=1e100,maxy=-1e100;
  for(auto & v:p) {minx=std::min(minx,v.x);maxx=std::max(maxx,v.x);miny=std::min(miny,v.y);maxy=std::max(maxy,v.y);}
  return maxx>=lx && minx<=ux && maxy>=ly && miny<=uy;
}
// Call with the costmap mutex held. Every occupied cell intersecting the filled
// polygon is checked, including cells wholly inside it. Expanded cell AABBs
// conservatively account for displacement between adjacent samples.
inline bool collision(nav2_costmap_2d::Costmap2D & map,const Polygon & footprint,
                      double x,double y,double yaw,double sampling=0.) {
  if(!convex(footprint)||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||!std::isfinite(sampling)||sampling<0) return true;
  Polygon p=footprint;double c=std::cos(yaw),s=std::sin(yaw);
  double lx=1e100,ly=1e100,ux=-1e100,uy=-1e100;
  for(auto & v:p) {double xx=x+c*v.x-s*v.y;v.y=y+s*v.x+c*v.y;v.x=xx;lx=std::min(lx,v.x);ly=std::min(ly,v.y);ux=std::max(ux,v.x);uy=std::max(uy,v.y);}
  unsigned int ix0,iy0,ix1,iy1;
  if(!map.worldToMap(lx-sampling,ly-sampling,ix0,iy0)||!map.worldToMap(ux+sampling,uy+sampling,ix1,iy1)) return true;
  double half=map.getResolution()/2+sampling;
  for(unsigned int iy=iy0;iy<=iy1;++iy) for(unsigned int ix=ix0;ix<=ix1;++ix) {
    if(map.getCost(ix,iy)<254) continue;
    double cx,cy;map.mapToWorld(ix,iy,cx,cy);
    if(intersects(p,cx-half,cy-half,cx+half,cy+half)) return true;
  }
  return false;
}
}
