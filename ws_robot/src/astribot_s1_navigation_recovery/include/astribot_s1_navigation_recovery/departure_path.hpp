#pragma once
#include "astribot_s1_robot_geometry/layered_collision.hpp"
#include <array>
#include <optional>
#include <stdexcept>

namespace astribot_s1_navigation_recovery {
struct DeparturePose {double x,y,yaw;};
struct DepartureSelection {
  DeparturePose target;
  double distance;
  double exit_distance{0.};
  const char *direction{"NONE"};
};

inline bool navigationStartClear(nav2_costmap_2d::Costmap2D & map,
    const astribot_s1_robot_geometry::Polygon & footprint,DeparturePose p,double reserve=0.) {
  return !astribot_s1_robot_geometry::collision(map,footprint,p.x,p.y,p.yaw,reserve);
}

// Each direction must have a layered-safe exit to normal navigation. Execute
// only its short prefix: after a measured stop the BT obtains a fresh snapshot
// and checks again. A short step is not itself a claim that recovery is done.
inline DepartureSelection selectDeparture(
    const astribot_s1_robot_geometry::LayeredCollisionSnapshot & layers,
    nav2_costmap_2d::Costmap2D & map,
    const astribot_s1_robot_geometry::Polygon & footprint,
    DeparturePose start,double max_distance,double step) {
  if(layers.collision(start.x,start.y,start.yaw))
    throw std::runtime_error("DEPARTURE_START_LAYER_COLLISION");
  if(navigationStartClear(map,footprint,start))return {start,0.};
  // Existing cross-track/stop tolerances and planar handoff sampling reserve.
  const double reserve=.03+.008+map.getResolution()*.25;
  struct Direction {double x,y;const char *name;};
  const std::array<Direction,4> directions{{{-1.,0.,"BACKWARD"},{0.,1.,"LEFT"},
    {0.,-1.,"RIGHT"},{1.,0.,"FORWARD"}}};
  const double c=std::cos(start.yaw),s=std::sin(start.yaw);
  const int count=int(std::floor(max_distance/step+1e-9));
  std::optional<DepartureSelection> best;
  for(const auto &direction:directions) {
    const double ux=c*direction.x-s*direction.y,uy=s*direction.x+c*direction.y;
    for(int i=1;i<=count;++i) {
      const double distance=i*step;
      DeparturePose exit{start.x+distance*ux,start.y+distance*uy,start.yaw};
      // A blocked/unknown prefix rules out every longer exit in this direction.
      if(layers.edgeCollision(start.x,start.y,start.yaw,exit.x,exit.y,exit.yaw))break;
      if(!navigationStartClear(map,footprint,exit,reserve))continue;
      const double travel=std::min(distance,.2);
      DeparturePose target{start.x+travel*ux,start.y+travel*uy,start.yaw};
      DepartureSelection candidate{target,travel,distance,direction.name};
      // Fixed direction order resolves equal-distance candidates.
      if(!best||distance<best->exit_distance-1e-9)best=candidate;
      break;
    }
  }
  if(!best)throw std::runtime_error("DEPARTURE_NO_SAFE_POSITION");
  return *best;
}
} // namespace astribot_s1_navigation_recovery
