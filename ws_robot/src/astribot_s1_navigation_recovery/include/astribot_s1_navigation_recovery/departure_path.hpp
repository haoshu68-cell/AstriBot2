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

// Policy boxes already include their observation covariance. The installed
// robot footprint already includes the navigation margin. Keep the boxes in
// their common source frame so a rotated map transform adds no AABB padding.
struct RecoveryObstacles {
  std::vector<std::array<double,4>> boxes;
  DeparturePose source_in_map{0.,0.,0.};
};

inline bool policyDepartureClear(const RecoveryObstacles & obstacles,
    const astribot_s1_robot_geometry::Polygon & footprint,
    DeparturePose start,DeparturePose end,double reserve=0.) {
  if(obstacles.boxes.empty())return true;
  using namespace astribot_s1_robot_geometry;
  Polygon2 swept;
  const double tc=std::cos(obstacles.source_in_map.yaw),ts=std::sin(obstacles.source_in_map.yaw);
  for(const auto &pose:{start,end}) {
    const double dx=pose.x-obstacles.source_in_map.x,dy=pose.y-obstacles.source_in_map.y;
    const double x=tc*dx+ts*dy,y=-ts*dx+tc*dy;
    const double yaw=pose.yaw-obstacles.source_in_map.yaw,c=std::cos(yaw),s=std::sin(yaw);
    for(const auto &point:footprint)swept.push_back({x+c*point.x-s*point.y,y+s*point.x+c*point.y});
  }
  double lx=INFINITY,ly=INFINITY,ux=-INFINITY,uy=-INFINITY;
  for(const auto &point:swept) {lx=std::min(lx,point[0]);ly=std::min(ly,point[1]);
    ux=std::max(ux,point[0]);uy=std::max(uy,point[1]);}
  std::vector<std::array<double,7>> queries;
  for(const auto &box:obstacles.boxes) {
    if(box[2]<lx-reserve||box[0]>ux+reserve||box[3]<ly-reserve||box[1]>uy+reserve)continue;
    queries.push_back({0.,0.,0.,box[0],box[1],box[2],box[3]});
  }
  if(queries.empty())return true;
  // For a fixed-yaw straight segment this hull is the exact continuous sweep.
  for(const double distance:boxDistances(convexHull(swept),queries))if(distance<=reserve)return false;
  return true;
}

inline bool navigationStartClear(nav2_costmap_2d::Costmap2D & map,
    const astribot_s1_robot_geometry::Polygon & footprint,DeparturePose p,double reserve=0.) {
  return !astribot_s1_robot_geometry::collision(map,footprint,p.x,p.y,p.yaw,reserve);
}

// Only a heading returned by an actual failed start connection activates this
// condition. Ordinary current-pose admission never assumes a future turn.
inline bool departureExitClear(
    const astribot_s1_robot_geometry::LayeredCollisionSnapshot & layers,
    nav2_costmap_2d::Costmap2D & map,
    const astribot_s1_robot_geometry::Polygon & footprint,
    DeparturePose pose,std::optional<double> required_heading,double reserve=0.,
    const RecoveryObstacles & obstacles={}) {
  if(!required_heading)return navigationStartClear(map,footprint,pose,reserve)&&
    policyDepartureClear(obstacles,footprint,pose,pose,reserve);
  const double turn=std::remainder(*required_heading-pose.yaw,2*M_PI);
  if(layers.edgeCollision(pose.x,pose.y,pose.yaw,pose.x,pose.y,*required_heading))return false;
  const int samples=std::max(1,int(std::ceil(std::abs(turn)/.05)));
  const double margin=reserve+astribot_s1_robot_geometry::radius(footprint)*std::abs(turn)/(2*samples);
  for(int i=0;i<=samples;++i) {
    const DeparturePose sample{pose.x,pose.y,pose.yaw+turn*double(i)/samples};
    if(!navigationStartClear(map,footprint,sample,margin)||
        !policyDepartureClear(obstacles,footprint,sample,sample,margin))return false;
  }
  return true;
}

// Each direction must have a layered-safe exit to normal navigation. Execute
// only its short prefix: after a measured stop the BT obtains a fresh snapshot
// and checks again. A short step is not itself a claim that recovery is done.
inline DepartureSelection selectDeparture(
    const astribot_s1_robot_geometry::LayeredCollisionSnapshot & layers,
    nav2_costmap_2d::Costmap2D & map,
    const astribot_s1_robot_geometry::Polygon & footprint,
    DeparturePose start,double max_distance,double step,
    std::optional<double> required_heading=std::nullopt,
    const RecoveryObstacles & obstacles={}) {
  if(layers.collision(start.x,start.y,start.yaw))
    throw std::runtime_error("DEPARTURE_START_LAYER_COLLISION");
  if(!policyDepartureClear(obstacles,footprint,start,start))
    throw std::runtime_error("DEPARTURE_START_POLICY_COLLISION");
  if(departureExitClear(layers,map,footprint,start,required_heading,0.,obstacles))return {start,0.};
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
      if(!policyDepartureClear(obstacles,footprint,start,exit))break;
      if(!departureExitClear(layers,map,footprint,exit,required_heading,reserve,obstacles))continue;
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
