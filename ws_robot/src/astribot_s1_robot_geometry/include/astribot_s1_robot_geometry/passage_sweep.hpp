#pragma once
#include "astribot_s1_robot_geometry/geometry_kernels.hpp"

namespace astribot_s1_robot_geometry {
// Filled convex sweep of the measured envelope. Inflation covers both the
// required clearance and the vertex motion between adjacent angular samples.
inline Polygon2 passageSweep(const Polygon2 & footprint,const std::array<double,3> & start,
    const std::array<double,3> & end,double margin) {
  finiteValues(start);finiteValues(end);
  geometryRequire(std::isfinite(margin)&&margin>=0.,"invalid passage margin");
  const auto polygon=validatePolygon(footprint);
  const double turn=std::remainder(end[2]-start[2],2*pi);
  const int steps=std::max(1,static_cast<int>(std::ceil(std::abs(turn)/.05)));
  double radius=0.;Polygon2 points;
  for(const auto & p:polygon)radius=std::max(radius,std::hypot(p[0],p[1]));
  for(int i=0;i<=steps;++i) {
    const double t=double(i)/steps,a=start[2]+turn*t,c=std::cos(a),s=std::sin(a);
    for(const auto & p:polygon)points.push_back({start[0]+t*(end[0]-start[0])+c*p[0]-s*p[1],
      start[1]+t*(end[1]-start[1])+s*p[0]+c*p[1]});
  }
  // Linear translation lies in the endpoint hull. Rotation deviates from its
  // chord by at most radius*(1-cos(delta/2)).
  return inflatePolygon(points,margin+radius*(1-std::cos(turn/(2*steps))));
}
inline bool passageGridClear(const Polygon2 & polygon,const std::array<double,3> & origin,
    double resolution,int width,int height,const std::vector<int> & cells) {
  geometryRequire(width>0&&height>0&&cells.size()==static_cast<size_t>(width)*height&&
    std::isfinite(resolution)&&resolution>0.,"invalid passage map");
  finiteValues(origin);
  Polygon2 local;const double c=std::cos(origin[2]),s=std::sin(origin[2]);
  double lx=INFINITY,ly=INFINITY,ux=-INFINITY,uy=-INFINITY;
  for(const auto & p:polygon) {
    const double x=c*(p[0]-origin[0])+s*(p[1]-origin[1]),y=-s*(p[0]-origin[0])+c*(p[1]-origin[1]);
    local.push_back({x,y});lx=std::min(lx,x);ly=std::min(ly,y);ux=std::max(ux,x);uy=std::max(uy,y);
  }
  if(lx<0.||ly<0.||ux>=width*resolution||uy>=height*resolution)return false;
  std::vector<std::array<double,7>> boxes;
  for(int y=static_cast<int>(ly/resolution);y<=static_cast<int>(uy/resolution);++y)
    for(int x=static_cast<int>(lx/resolution);x<=static_cast<int>(ux/resolution);++x) {
      const auto value=cells[y*width+x];
      if(value<0||value>=65)boxes.push_back({0.,0.,0.,x*resolution,y*resolution,(x+1)*resolution,(y+1)*resolution});
    }
  for(double distance:boxDistances(local,boxes))if(distance<=0.)return false;
  return true;
}
}
