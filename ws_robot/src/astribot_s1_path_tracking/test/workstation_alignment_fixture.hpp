#pragma once
#include "astribot_s1_path_tracking/workstation_alignment_core.hpp"
namespace workstation_test {
using Snapshot=astribot_s1_path_tracking::LayeredCollisionSnapshot;
inline geometry_msgs::msg::Polygon rectangle(double x,double y) {
  geometry_msgs::msg::Polygon p;
  for(const auto &xy:std::vector<std::pair<double,double>>{{-x,-y},{x,-y},{x,y},{-x,y}}) {
    geometry_msgs::msg::Point32 v;v.x=xy.first;v.y=xy.second;p.points.push_back(v);
  }
  return p;
}
struct Geometry {
  std::shared_ptr<Snapshot::Maps> maps=std::make_shared<Snapshot::Maps>();
  std::shared_ptr<Snapshot::Envelope> envelope=std::make_shared<Snapshot::Envelope>();
  Geometry() {
    maps->header.frame_id="map";maps->map_revision="workstation-1";maps->profile_revision=std::string(64,'a');
    maps->ground_reference="world_horizontal_ground";maps->evidence_kind="gazebo_collision_geometry";
    maps->height_edges={.05,.5,1.5};maps->layer_names={"base","arm"};
    nav_msgs::msg::OccupancyGrid grid;grid.header=maps->header;grid.info.width=grid.info.height=200;
    grid.info.resolution=.02;grid.info.origin.position.x=grid.info.origin.position.y=-2.;grid.info.origin.orientation.w=1.;grid.data.assign(40000,0);
    maps->grids={grid,grid};
    envelope->header.frame_id="base";envelope->epoch=7;envelope->mode=envelope->FIXED_POSTURE;
    envelope->coordinator_session_id="workstation-fixture";envelope->height_profile_revision=maps->profile_revision;
    envelope->installed_geometry_hash="geometry";envelope->navigation_allowed=envelope->limits.transport_ready=true;
    envelope->limits.height_m=1.4;envelope->ground_in_base_m=-.1;envelope->installed_footprint=rectangle(.3,.1);
    for(size_t i=0;i<2;++i) {
      astribot_navigation_msgs::msg::EnvelopeSlice slice;
      slice.z_min_m=maps->height_edges[i]-.1;slice.z_max_m=maps->height_edges[i+1]-.1;
      slice.footprint=i?rectangle(.3,.03):rectangle(.1,.1);envelope->height_slices.push_back(slice);
    }
    rehash();
  }
  void rehash() {
    envelope->height_geometry_hash=astribot_s1_robot_geometry::layeredGeometryHash(
      astribot_s1_robot_geometry::envelopePolygonPoints(envelope->installed_footprint),envelope->height_slices,
      "base",0.,envelope->height_profile_revision,envelope->ground_in_base_m);
  }
  void obstacle(size_t layer,double x,double y,int8_t value=100) {
    auto &g=maps->grids.at(layer);const unsigned ix=unsigned((x+2.)/g.info.resolution),iy=unsigned((y+2.)/g.info.resolution);
    g.data.at(size_t(iy)*g.info.width+ix)=value;maps->map_revision+="x";
  }
  Snapshot snapshot() const {return Snapshot(maps,envelope,"map");}
};
} // namespace workstation_test
