#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace astribot_s1_path_tracking {
struct RoutePoint {double x,y;};
// This is a route-intent check, not collision clearance. The planner/controller
// must still validate the complete installed polygon and its continuous sweep.
inline bool pathTraversesCorridor(const std::vector<RoutePoint> & path,
  RoutePoint entry, RoutePoint exit, double width, bool initial) {
  const double length=std::hypot(exit.x-entry.x,exit.y-entry.y);
  if(path.size()<2 || !std::isfinite(length) || length<.1 ||
    !std::isfinite(width) || width<=0.)return false;
  const double ux=(exit.x-entry.x)/length, uy=(exit.y-entry.y)/length;
  std::vector<RoutePoint> local;local.reserve(path.size());
  for(const auto & p:path) {
    if(!std::isfinite(p.x) || !std::isfinite(p.y))return false;
    const double x=p.x-entry.x,y=p.y-entry.y;
    local.push_back({x*ux+y*uy,-x*uy+y*ux});
  }
  if(local.back().x<length || (initial && local.front().x>=0.))return false;
  // After the exit a replan may lie wholly outside the constrained strip.
  // Actual entry/exit traversal is independently witnessed by the task.
  bool crossed=false;
  for(size_t i=1;i<local.size();++i) {
    auto a=local[i-1],b=local[i];const double dx=b.x-a.x;
    double low=0.,high=1.;
    if(std::abs(dx)<1e-12) {
      if(a.x<0. || a.x>length)continue;
    } else {
      auto t0=-a.x/dx,t1=(length-a.x)/dx;
      if(t0>t1)std::swap(t0,t1);
      low=std::max(0.,t0);high=std::min(1.,t1);
      if(low>high)continue;
    }
    crossed=true;
    if(dx<-.01)return false;
    for(double t:{low,high})if(std::abs(a.y+t*(b.y-a.y))>=width/2.)return false;
  }
  return crossed || (!initial && local.front().x>length);
}
}
