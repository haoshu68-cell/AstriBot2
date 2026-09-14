// Copyright 2026 Astribot. Apache-2.0.
#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include "nav_msgs/msg/path.hpp"

namespace astribot_s1_path_tracking {
struct PathQuality {double curvature{0}, curvature_rate{0};};
inline double pathDistance(const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{return std::hypot(a.pose.position.x-b.pose.position.x, a.pose.position.y-b.pose.position.y);}
inline nav_msgs::msg::Path resamplePath(const nav_msgs::msg::Path & path, double step = 0.10)
{
  if (!std::isfinite(step) || step <= 0 || path.poses.size()>100000) {throw std::invalid_argument("invalid path sampling input");}
  auto out = path; out.poses.clear();
  if (path.poses.empty()) {return out;}
  out.poses.push_back(path.poses.front());
  double remaining = step;
  for (size_t i=1; i<path.poses.size(); ++i) {
    auto a=path.poses[i-1]; const auto & b=path.poses[i];
    double d=pathDistance(a,b);
    if (!std::isfinite(d)) {throw std::invalid_argument("nonfinite path");}
    while (d>=remaining && d>1e-8) {
      double f=remaining/d;
      a.pose.position.x += f*(b.pose.position.x-a.pose.position.x);
      a.pose.position.y += f*(b.pose.position.y-a.pose.position.y);
      if (out.poses.size()>=100000) {throw std::invalid_argument("path exceeds sampling budget");}
      out.poses.push_back(a); d=pathDistance(a,b); remaining=step;
    }
    remaining-=d;
  }
  if (pathDistance(out.poses.back(),path.poses.back())>1e-6) {
    out.poses.push_back(path.poses.back());
  } else {out.poses.back()=path.poses.back();}
  return out;
}
inline nav_msgs::msg::Path restorePathDensity(const nav_msgs::msg::Path & candidate,
  const nav_msgs::msg::Path & original)
{
  if(original.poses.size()<2) {return candidate;}
  double length=0;
  for(size_t i=1;i<candidate.poses.size();++i) {length+=pathDistance(candidate.poses[i-1],candidate.poses[i]);}
  if(length<1e-6) {return candidate;}
  return resamplePath(candidate,length/(original.poses.size()-1));
}
inline PathQuality pathQuality(const nav_msgs::msg::Path & path)
{
  auto p=resamplePath(path); PathQuality out; double last=0; bool have=false;
  // Endpoint capture is handled by ArrivalController; measure the travelling part.
  for (size_t i=2; i+2<p.poses.size(); ++i) {
    const auto & a=p.poses[i-1].pose.position;
    const auto & b=p.poses[i].pose.position;
    const auto & c=p.poses[i+1].pose.position;
    double ab=std::hypot(b.x-a.x,b.y-a.y), bc=std::hypot(c.x-b.x,c.y-b.y);
    if (ab<1e-6 || bc<1e-6) {continue;}
    double turn=std::remainder(std::atan2(c.y-b.y,c.x-b.x)-std::atan2(b.y-a.y,b.x-a.x),2*M_PI);
    double k=turn/(0.5*(ab+bc)); out.curvature=std::max(out.curvature,std::abs(k));
    if (have) {out.curvature_rate=std::max(out.curvature_rate,std::abs(k-last)/ab);}
    last=k; have=true;
  }
  return out;
}
inline bool acceptableQuality(const PathQuality & q, double max_k, double max_rate)
{return std::isfinite(q.curvature) && std::isfinite(q.curvature_rate) && q.curvature<=max_k && q.curvature_rate<=max_rate;}
// Preserve ordinary commands. A sharp safe corridor is traversed at reduced speed.
inline double sharpPathSpeedLimit(const PathQuality & q)
{
  if (acceptableQuality(q,3.0,12.0)) {return std::numeric_limits<double>::infinity();}
  if (!std::isfinite(q.curvature) || !std::isfinite(q.curvature_rate)) {return 0.0;}
  return std::min({0.15, std::sqrt(0.20/std::max(q.curvature,1e-6)),
    std::sqrt(0.25/std::max(q.curvature_rate,1e-6))});
}
inline void smoothPathStep(nav_msgs::msg::Path & path, const nav_msgs::msg::Path & reference,
  double max_displacement)
{
  auto next=path;
  for (size_t i=1; i+1<path.poses.size(); ++i) {
    auto & p=next.poses[i].pose.position;
    const auto & old=path.poses[i].pose.position;
    const auto & a=path.poses[i-1].pose.position; const auto & b=path.poses[i+1].pose.position;
    const auto & ref=reference.poses[i].pose.position;
    p.x=old.x+0.35*(a.x+b.x-2*old.x)+0.02*(ref.x-old.x);
    p.y=old.y+0.35*(a.y+b.y-2*old.y)+0.02*(ref.y-old.y);
    double d=std::hypot(p.x-ref.x,p.y-ref.y);
    if (d>max_displacement) {p.x=ref.x+(p.x-ref.x)*max_displacement/d; p.y=ref.y+(p.y-ref.y)*max_displacement/d;}
  }
  path=next;
}
// Suppress sub-cell endpoint seams in the speed diagnostic, without changing the path.
inline PathQuality travellingPathQuality(const nav_msgs::msg::Path & path)
{
  auto reference=resamplePath(path), filtered=reference;
  for(int i=0;i<3;++i) {smoothPathStep(filtered,reference,0.03);}
  return pathQuality(filtered);
}
}  // namespace astribot_s1_path_tracking
