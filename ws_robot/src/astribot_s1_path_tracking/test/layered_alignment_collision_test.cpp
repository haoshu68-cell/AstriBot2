#include "astribot_s1_robot_geometry/layered_collision.hpp"
#include <cassert>
#include <iostream>

namespace geometry=astribot_s1_robot_geometry;
using Snapshot=geometry::LayeredCollisionSnapshot;
geometry_msgs::msg::Polygon rectangle(double x,double y) {
  geometry_msgs::msg::Polygon p;
  for(const auto &xy:std::vector<std::pair<double,double>>{{-x,-y},{x,-y},{x,y},{-x,y}}) {
    geometry_msgs::msg::Point32 v;v.x=xy.first;v.y=xy.second;p.points.push_back(v);
  }
  return p;
}
struct Fixture {
  std::shared_ptr<Snapshot::Maps> maps=std::make_shared<Snapshot::Maps>();
  std::shared_ptr<Snapshot::Envelope> envelope=std::make_shared<Snapshot::Envelope>();
  Fixture() {
    maps->header.frame_id="map";maps->map_revision="scene-1";maps->profile_revision=std::string(64,'a');
    maps->ground_reference="world_horizontal_ground";maps->evidence_kind="gazebo_collision_geometry";
    maps->height_edges={.05,.5,1.5,2.};maps->layer_names={"base","arm","empty"};
    nav_msgs::msg::OccupancyGrid grid;grid.header=maps->header;grid.info.width=grid.info.height=100;
    grid.info.resolution=.05;grid.info.origin.position.x=grid.info.origin.position.y=-2.5;grid.info.origin.orientation.w=1.;grid.data.assign(10000,0);
    maps->grids={grid,grid,grid};maps->grids[2].data.assign(10000,-1);
    envelope->header.frame_id="base";envelope->epoch=7;envelope->height_profile_revision=maps->profile_revision;
    envelope->limits.height_m=1.4;envelope->ground_in_base_m=-.1;envelope->installed_footprint=rectangle(.9,.2);
    for(size_t i=0;i<3;++i) {
      astribot_navigation_msgs::msg::EnvelopeSlice slice;
      slice.z_min_m=maps->height_edges[i]-.1;slice.z_max_m=maps->height_edges[i+1]-.1;
      if(i==0)slice.footprint=rectangle(.2,.2);
      if(i==1)slice.footprint=rectangle(.9,.08);
      envelope->height_slices.push_back(slice);
    }
    rehash();
  }
  void rehash() {envelope->height_geometry_hash=geometry::layeredGeometryHash(geometry::envelopePolygonPoints(envelope->installed_footprint),
    envelope->height_slices,"base",0.,envelope->height_profile_revision,envelope->ground_in_base_m);}
  void obstacle(size_t layer,double x,double y,int8_t value=100) {
    auto &g=maps->grids.at(layer);const unsigned ix=unsigned((x+2.5)/g.info.resolution),iy=unsigned((y+2.5)/g.info.resolution);
    g.data.at(size_t(iy)*g.info.width+ix)=value;
  }
  Snapshot snapshot() const {return Snapshot(maps,envelope,"map");}
};
int main() {
  {
    // SC-T-GATE-HEIGHT-SEPARATION: 1.0 m below the arms, 2.3 m at arm height.
    // Conservative rectangular envelopes include clearance: base 0.8 m,
    // raised arms 1.8 m. This is a geometry fixture, not a measured robot model.
    Fixture f;
    f.maps->height_edges={.05,.68,1.63,2.30};
    f.envelope->installed_footprint=rectangle(.55,.9);
    for(size_t i=0;i<3;++i) {
      f.envelope->height_slices[i].z_min_m=f.maps->height_edges[i]-.1;
      f.envelope->height_slices[i].z_max_m=f.maps->height_edges[i+1]-.1;
    }
    f.envelope->height_slices[0].footprint=rectangle(.4,.4);
    f.envelope->height_slices[1].footprint=rectangle(.55,.9);
    // Occupy every raster cell intersecting either side of the physical gate.
    for(size_t layer=0;layer<2;++layer) {
      auto &g=f.maps->grids[layer];const double resolution=g.info.resolution;
      const double inner_y=layer?1.15:.5;
      for(unsigned iy=0;iy<g.info.height;++iy)for(unsigned ix=0;ix<g.info.width;++ix) {
        const double x=g.info.origin.position.x+ix*resolution;
        const double y=g.info.origin.position.y+iy*resolution;
        if(x<1.125&&x+resolution>.775&&
            ((y<1.45&&y+resolution>inner_y)||(y<-inner_y&&y+resolution>-1.45)))
          g.data[iy*g.info.width+ix]=100;
      }
    }
    f.rehash();
    assert(!f.snapshot().edgeCollision(0.,0.,0.,1.8,0.,0.));
    const auto arm=f.envelope->height_slices[1].footprint;
    f.envelope->height_slices[1].footprint.points.clear();f.rehash();
    assert(!f.snapshot().edgeCollision(0.,0.,0.,1.8,0.,0.)); // Base alone.
    f.envelope->height_slices[1].footprint=arm;
    const auto base=f.envelope->height_slices[0].footprint;
    f.envelope->height_slices[0].footprint.points.clear();f.rehash();
    assert(!f.snapshot().edgeCollision(0.,0.,0.,1.8,0.,0.)); // Raised arm layer alone.
    f.envelope->height_slices[0].footprint=base;f.rehash();
    const auto &g=f.maps->grids[0];
    nav2_costmap_2d::Costmap2D planar(g.info.width,g.info.height,g.info.resolution,-2.5,-2.5,0);
    for(unsigned iy=0;iy<g.info.height;++iy)for(unsigned ix=0;ix<g.info.width;++ix)
      if(g.data[iy*g.info.width+ix]>=65)planar.setCost(ix,iy,254);
    geometry::Polygon full;
    for(const auto &p:f.envelope->installed_footprint.points) {
      geometry_msgs::msg::Point v;v.x=p.x;v.y=p.y;full.push_back(v);
    }
    assert(!geometry::collision(planar,full,0.,0.,0.));
    assert(geometry::collision(planar,full,.95,0.,0.)); // Same path crosses this pose.
    assert(!geometry::collision(planar,full,1.8,0.,0.));
    // Move the same side obstacles up to the arm layer: passage must be rejected.
    f.maps->grids[1].data=g.data;f.maps->map_revision="t-gate-arm-blocked";
    assert(f.snapshot().edgeCollision(0.,0.,0.,1.8,0.,0.));
    std::cout<<"SC-T-GATE-HEIGHT-SEPARATION: base=clear arms=clear layered=clear planar=blocked; raised obstruction=blocked\n";
  }
  {
    Fixture f;f.obstacle(0,.55,0.);assert(!f.snapshot().collision(0.,0.,0.));
    f.obstacle(1,.55,0.);assert(f.snapshot().collision(0.,0.,0.));
  }
  {
    Fixture f;f.obstacle(1,.5,.5);auto s=f.snapshot();
    assert(!s.collision(0.,0.,0.));assert(!s.collision(0.,0.,M_PI/2));
    assert(s.edgeCollision(0.,0.,0.,0.,0.,M_PI/2));
    assert(s.edgeCollision(0.,0.,M_PI/2,0.,0.,0.));
  }
  {
    Fixture f;assert(!f.snapshot().collision(0.,0.,0.)); // Empty high layer is unknown.
    f.obstacle(1,.55,0.,-1);assert(f.snapshot().collision(0.,0.,0.));
    assert(f.snapshot().collision(2.4,0.,0.));
  }
  {
    Fixture f;f.obstacle(1,.5,.5);auto s=f.snapshot();
    const double a=.3,c=std::cos(a),t=std::sin(a);
    auto transformed=s.inFrame("odom",.4,-.2,a);
    const double x=-c*.4+t*.2,y=t*.4+c*.2;
    assert(!transformed->collision(x,y,-a));
    assert(transformed->edgeCollision(x,y,-a,x,y,M_PI/2-a));
    unsigned mx,my;assert(transformed->worldToMap(x,y,mx,my));
    double wx,wy;transformed->mapToWorld(mx,my,wx,wy);unsigned nx,ny;
    assert(transformed->worldToMap(wx,wy,nx,ny)&&nx==mx&&ny==my);
  }
  {
    Fixture f;assert(!f.snapshot().edgeCollision(0.,0.,M_PI-.01,0.,0.,-M_PI+.01));
    auto reject=[](Fixture &v,const char *name,const char *reason) {
      try {v.snapshot();}
      catch(const std::exception &error) {
        if(std::string(error.what()).find(reason)==std::string::npos)
          throw std::runtime_error(std::string(name)+": unexpected rejection: "+error.what());
        return;
      }
      throw std::runtime_error(std::string(name)+": invalid input was accepted");
    };
    f.maps->evidence_kind="static_archive_occupied_endpoints_only";
    assert(!f.snapshot().collision(0.,0.,0.));
    f.obstacle(1,.55,0.,-1);assert(f.snapshot().collision(0.,0.,0.));
    f.maps->evidence_kind="unclassified";reject(f,"unknown source","LAYERED_MAP_SOURCE_MISMATCH");
    f.maps->evidence_kind="gazebo_collision_geometry";f.maps->ground_reference.clear();reject(f,"missing ground reference","LAYERED_MAP_SOURCE_MISMATCH");
    f.maps->ground_reference="world_horizontal_ground";
    f.maps->profile_revision=std::string(64,'b');reject(f,"profile mismatch","LAYERED_PROFILE_MISMATCH");
    f.maps->profile_revision=f.envelope->height_profile_revision;f.maps->height_edges[1]+=.02;reject(f,"height edge mismatch","LAYERED_HEIGHT_ALIGNMENT_MISMATCH");
    f.maps->height_edges[1]-=.02;f.envelope->height_slices[1].footprint.points[0].x-=.1f;reject(f,"slice hash mismatch","HEIGHT_GEOMETRY_HASH_MISMATCH");
  }
  {
    Fixture f;f.obstacle(1,.5,.5);auto s=f.snapshot();
    assert(s.commandCollision(0.,0.,0.,0.,0.,M_PI/2,1.));
    assert(!s.commandCollision(0.,0.,0.,-.1,0.,0.,1.));
    Fixture translated;translated.obstacle(1,-1.1,0.);
    assert(!translated.snapshot().collision(0.,0.,0.));
    assert(translated.snapshot().commandCollision(0.,0.,0.,-.5,0.,0.,1.));
  }
  {
    Fixture f;
    // A triangular arm layer exercises the loose cached bounding box. The
    // oracle scans occupied cells without the new broad-phase shortcut.
    f.envelope->height_slices[1].footprint.points.pop_back();f.rehash();
    for(int i=0;i<15;++i) {
      f.obstacle(i%2,-1.4+.19*i,.73*std::sin(i),i%3?100:-1);
    }
    auto snapshot=f.snapshot();
    const auto oracle=[&](double x,double y,double yaw,double margin) {
      for(size_t layer=0;layer<f.envelope->height_slices.size();++layer) {
        const auto &slice=f.envelope->height_slices[layer];if(slice.footprint.points.empty())continue;
        const auto &grid=f.maps->grids[layer];const double res=grid.info.resolution;
        geometry::Polygon polygon;
        for(const auto &point:slice.footprint.points) {
          geometry_msgs::msg::Point p;
          p.x=x-grid.info.origin.position.x+std::cos(yaw)*point.x-std::sin(yaw)*point.y;
          p.y=y-grid.info.origin.position.y+std::sin(yaw)*point.x+std::cos(yaw)*point.y;
          if(p.x-margin<0||p.y-margin<0||p.x+margin>=grid.info.width*res||p.y+margin>=grid.info.height*res)return true;
          polygon.push_back(p);
        }
        for(unsigned iy=0;iy<grid.info.height;++iy)for(unsigned ix=0;ix<grid.info.width;++ix) {
          const auto cell=grid.data[iy*grid.info.width+ix];if(cell>=0&&cell<65)continue;
          if(geometry::intersects(polygon,ix*res-margin,iy*res-margin,(ix+1)*res+margin,(iy+1)*res+margin))return true;
        }
      }
      return false;
    };
    for(int i=0;i<1000;++i) {
      const double x=(i%19)*.117-.97,y=(i%23)*.083-.91,a=i*.073,margin=(i%3)*.009;
      assert(snapshot.collision(x,y,a,margin)==oracle(x,y,a,margin));
    }
  }
  std::cout<<"layered alignment: height separation, swept rotation, unknown, bounds, TF and profile/hash checks passed\n";
}
