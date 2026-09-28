#include "astribot_s1_navigation_recovery/departure_path.hpp"
#include <cassert>
#include <iostream>
namespace geometry=astribot_s1_robot_geometry;
using namespace astribot_s1_navigation_recovery;
using Snapshot=geometry::LayeredCollisionSnapshot;
struct Fixture {
  std::shared_ptr<Snapshot::Maps> maps=std::make_shared<Snapshot::Maps>();
  std::shared_ptr<Snapshot::Envelope> envelope=std::make_shared<Snapshot::Envelope>();
  nav2_costmap_2d::Costmap2D planar{160,160,.025,-2.,-2.,0};
  geometry::Polygon footprint;
  Fixture() {
    maps->header.frame_id="map";maps->map_revision="departure-scene";maps->profile_revision=std::string(64,'a');
    maps->ground_reference="world_horizontal_ground";maps->evidence_kind="gazebo_collision_geometry";
    maps->height_edges={.05,.5,1.5};maps->layer_names={"base","arm"};
    nav_msgs::msg::OccupancyGrid grid;grid.header=maps->header;grid.info.width=grid.info.height=160;
    grid.info.resolution=.025;grid.info.origin.position.x=grid.info.origin.position.y=-2.;grid.info.origin.orientation.w=1.;grid.data.assign(25600,0);
    maps->grids={grid,grid};envelope->header.frame_id="base";envelope->epoch=7;
    envelope->height_profile_revision=maps->profile_revision;envelope->limits.height_m=1.4;
    auto rectangle=[](double x,double y) {
      geometry_msgs::msg::Polygon p;
      for(auto xy:std::vector<std::pair<double,double>>{{-x,-y},{x,-y},{x,y},{-x,y}}) {
        geometry_msgs::msg::Point32 v;v.x=xy.first;v.y=xy.second;p.points.push_back(v);
      }
      return p;
    };
    // Low robot body is short, raised arm projects farther in 2D.
    envelope->installed_footprint=rectangle(.6,.2);
    for(size_t i=0;i<2;++i) {
      astribot_navigation_msgs::msg::EnvelopeSlice s;
      s.z_min_m=maps->height_edges[i];s.z_max_m=maps->height_edges[i+1];
      s.footprint=rectangle(i==0?.2:.6,.2);envelope->height_slices.push_back(s);
    }
    envelope->height_geometry_hash=geometry::layeredGeometryHash(geometry::envelopePolygonPoints(envelope->installed_footprint),
      envelope->height_slices,"base",0.,envelope->height_profile_revision,0.);
    for(auto p:envelope->installed_footprint.points) {geometry_msgs::msg::Point q;q.x=p.x;q.y=p.y;footprint.push_back(q);}
  }
  void obstacle(size_t layer,double x,double y,int value=100) {
    unsigned ix,iy;assert(planar.worldToMap(x,y,ix,iy));
    maps->grids[layer].data[iy*160+ix]=value;planar.setCost(ix,iy,value<0?255:254);
  }
  DepartureSelection select(double max=2.,DeparturePose start={0,0,0}) {
    Snapshot s(maps,envelope,"map");
    return selectDeparture(s,planar,footprint,start,max,.05);
  }
  void rejected(const std::string &reason,double max=2.) {
    try {select(max);}catch(const std::runtime_error &e) {assert(e.what()==reason);return;}
    throw std::runtime_error("expected rejection: "+reason);
  }
};
int main() {
  {Fixture f;assert(f.select().distance==0.);}
  {Fixture f;f.obstacle(0,0.,.45);assert(f.select().distance==0.);
    assert(navigationStartClear(f.planar,f.footprint,{0,0,0}));}
  {Fixture f;f.obstacle(0,.45,.0);auto selected=f.select();
    assert(selected.distance>0.&&selected.distance<=.2);assert(selected.target.x<0.&&selected.target.y==0.&&selected.target.yaw==0.);
    assert(std::string(selected.direction)=="BACKWARD"&&selected.exit_distance>=selected.distance);
    Snapshot s(f.maps,f.envelope,"map");assert(!s.edgeCollision(0,0,0,selected.target.x,0,0));}
  {Fixture f;f.obstacle(1,.45,.0);f.rejected("DEPARTURE_START_LAYER_COLLISION");}
  for(const int occupied: {100,-1}) {
    Fixture f;f.obstacle(0,.45,-.01);f.obstacle(1,-.68,.0,occupied);auto selected=f.select();
    assert(std::string(selected.direction)=="LEFT");assert(selected.target.x==0.&&selected.target.y>0.&&selected.distance<=.2);
    Snapshot s(f.maps,f.envelope,"map");assert(!s.edgeCollision(0,0,0,selected.target.x,selected.target.y,0.));
  }
  {Fixture f;f.obstacle(0,.45,.0);f.obstacle(1,-.68,.0);f.obstacle(1,0.,.28);f.obstacle(1,0.,-.28);
    f.rejected("DEPARTURE_NO_SAFE_POSITION");}
  {Fixture f;f.obstacle(0,.45,.0);f.rejected("DEPARTURE_NO_SAFE_POSITION",.05);}
  {Fixture f;f.obstacle(0,.45,.0);auto first=f.select();f.obstacle(1,-.68,.0);
    Snapshot changed(f.maps,f.envelope,"map");assert(changed.edgeCollision(0,0,0,first.target.x,0,0));}
  {Fixture f;f.obstacle(0,.30,0.);f.obstacle(0,0.,.3);f.obstacle(0,0.,-.3);
    auto first=f.select();assert(std::string(first.direction)=="BACKWARD");assert(first.distance==.2);
    assert(!navigationStartClear(f.planar,f.footprint,first.target));
    auto second=f.select(2.,first.target);assert(second.distance>0.&&second.distance<=.2);
    assert(std::string(second.direction)=="BACKWARD");
    assert(navigationStartClear(f.planar,f.footprint,second.target));
    assert(f.select(2.,second.target).distance==0.);
  }
  {Fixture f;f.obstacle(0,0.,.45);auto selected=f.select(2.,{0,0,M_PI/2});
    assert(std::string(selected.direction)=="BACKWARD");assert(selected.target.y<0.);
    assert(std::abs(selected.target.x)<1e-9&&selected.target.yaw==M_PI/2);}
  std::cout<<"departure: current-pose admission, short reverse/side steps, repeated recovery, real collision and unknown rejection passed\n";
}
