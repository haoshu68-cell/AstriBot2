#ifndef ASTRIBOT_S1_AUTONOMY__FRONTIER_OBSERVATION_HPP_
#define ASTRIBOT_S1_AUTONOMY__FRONTIER_OBSERVATION_HPP_
#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>
#include "astribot_s1_autonomy/frontier_search.hpp"
namespace astribot_s1_autonomy {
// Conservative observation proxy: each ray stops at an obstacle or its first unknown cell.
// Never treats unknown space as free or replaces the motion safety validator.
struct ObservationGain {std::size_t cells{0}; double yaw{0}; bool valid{false};};
inline ObservationGain frontierObservation(const GridMap & map, double x, double y,
  double fallback_yaw, double range, double fov, int rays, int free_threshold,
  const std::function<bool()> & canceled = {})
{
  ObservationGain result{0, fallback_yaw, false};
  if (!map.consistent() || !std::isfinite(map.resolution) || map.resolution<=0 ||
    !std::isfinite(map.origin_x) || !std::isfinite(map.origin_y) || !std::isfinite(fallback_yaw) ||
    !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(range) || range<0 ||
    !std::isfinite(fov) || fov<=0 || fov>6.283185307179587 || rays<8 || rays>4096) {return result;}
  if (range==0) {result.valid=true;return result;}
  unsigned int mx,my;if(!map.worldToMap(x,y,mx,my)){return result;}
  double nx=0,ny=0;
  for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
    const int64_t cx=static_cast<int64_t>(mx)+dx,cy=static_cast<int64_t>(my)+dy;
    if(cx>=0&&cy>=0&&cx<map.width&&cy<map.height&&map.data[map.index(cx,cy)]<0){nx+=dx;ny+=dy;}
  }
  if(std::hypot(nx,ny)>1e-6) {result.yaw=std::atan2(ny,nx);}
  const double steps=std::ceil(range/(map.resolution*.5));
  if(steps>4096) {return result;}  // bounded work; unavailable evidence cannot boost a candidate
  std::vector<std::size_t> seen;
  for(int ray=0;ray<rays;++ray) {
    if(canceled&&canceled()) {return result;}
    const double angle=result.yaw-fov*.5+(ray+.5)*fov/rays;
    for(int step=1;step<=static_cast<int>(steps);++step) {
      const double distance=std::min(range,step*map.resolution*.5);
      unsigned int gx,gy;
      if(!map.worldToMap(x+std::cos(angle)*distance,y+std::sin(angle)*distance,gx,gy)){break;}
      const auto index=map.index(gx,gy);const auto value=map.data[index];
      if(value<0){seen.push_back(index);break;}
      if(value>free_threshold){break;}
    }
  }
  std::sort(seen.begin(),seen.end());
  result.cells=static_cast<std::size_t>(std::unique(seen.begin(),seen.end())-seen.begin());
  result.valid=true;return result;
}
}  // namespace astribot_s1_autonomy
#endif
