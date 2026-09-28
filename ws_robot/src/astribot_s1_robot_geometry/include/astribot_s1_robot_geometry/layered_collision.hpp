#pragma once
#include "astribot_s1_robot_geometry/filled_collision.hpp"
#include "astribot_s1_robot_geometry/layered_envelope.hpp"
#include <astribot_navigation_msgs/msg/navigation_envelope_v2.hpp>
#include <astribot_slam_msgs/msg/height_slice_maps.hpp>
#include <memory>
#include <limits>
#include <stdexcept>

namespace astribot_s1_robot_geometry {
// One immutable map/envelope pair. Coordinates supplied by consumers are in
// query_frame; tx,ty,yaw map that frame into the height-map frame.
class LayeredCollisionSnapshot {
public:
  using Maps=astribot_slam_msgs::msg::HeightSliceMaps;
  using Envelope=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
  LayeredCollisionSnapshot(Maps::ConstSharedPtr maps,Envelope::ConstSharedPtr envelope,
      std::string query_frame,double tx=0.,double ty=0.,double yaw=0.)
    : maps_(std::move(maps)),envelope_(std::move(envelope)),frame_(std::move(query_frame)) {
    auto require=[](bool ok,const char *why) {if(!ok)throw std::invalid_argument(why);};
    require(bool(maps_)&&bool(envelope_),"LAYERED_INPUT_MISSING");
    validateLayeredEnvelope(*envelope_);
    require(!frame_.empty()&&!maps_->header.frame_id.empty()&&!maps_->map_revision.empty(),"LAYERED_MAP_IDENTITY_MISSING");
    require(!maps_->ground_reference.empty()&&
      (maps_->evidence_kind=="gazebo_collision_geometry"||
       maps_->evidence_kind=="static_archive_occupied_endpoints_only"),"LAYERED_MAP_SOURCE_MISMATCH");
    require(maps_->profile_revision==envelope_->height_profile_revision,"LAYERED_PROFILE_MISMATCH");
    require(std::isfinite(tx)&&std::isfinite(ty)&&std::isfinite(yaw)&&std::isfinite(maps_->ground_z),"LAYERED_TRANSFORM_INVALID");
    const auto count=envelope_->height_slices.size();
    require(count==maps_->grids.size()&&maps_->height_edges.size()==count+1&&maps_->layer_names.size()==count,
      "LAYERED_LAYER_COUNT_MISMATCH");
    require(count>0,"LAYERED_MAP_EMPTY");
    const auto &info=maps_->grids.front().info;
    const auto &q=info.origin.orientation;
    require(info.width>0&&info.height>0&&std::isfinite(info.resolution)&&info.resolution>0.,"LAYERED_GRID_INVALID");
    require(std::isfinite(info.origin.position.x)&&std::isfinite(info.origin.position.y)&&
      q.x==0.&&q.y==0.&&std::isfinite(q.z)&&std::isfinite(q.w)&&std::abs(q.z*q.z+q.w*q.w-1.)<1e-6,"LAYERED_GRID_ORIGIN_INVALID");
    resolution_=info.resolution;width_=info.width;height_=info.height;
    const double origin_yaw=2.*std::atan2(q.z,q.w),c=std::cos(origin_yaw),s=std::sin(origin_yaw);
    // Compose query->map->grid once, rather than performing TF per sample.
    tx_=c*(tx-info.origin.position.x)+s*(ty-info.origin.position.y);
    ty_=-s*(tx-info.origin.position.x)+c*(ty-info.origin.position.y);
    angle_=yaw-origin_yaw;c_=std::cos(angle_);s_=std::sin(angle_);
    bool nonempty=false;
    auto layers=std::make_shared<std::vector<Layer>>();
    for(size_t i=0;i<count;++i) {
      const auto &grid=maps_->grids[i];const auto &slice=envelope_->height_slices[i];
      require(grid.header.frame_id==maps_->header.frame_id&&grid.info==info,"LAYERED_GRID_ALIGNMENT_MISMATCH");
      require(grid.data.size()==uint64_t(width_)*height_,"LAYERED_GRID_DATA_SIZE");
      require(std::isfinite(maps_->height_edges[i])&&std::isfinite(maps_->height_edges[i+1])&&
        std::abs(slice.z_min_m-envelope_->ground_in_base_m-maps_->height_edges[i])<1e-6&&
        std::abs(slice.z_max_m-envelope_->ground_in_base_m-maps_->height_edges[i+1])<1e-6,"LAYERED_HEIGHT_ALIGNMENT_MISMATCH");
      Layer layer;layer.grid=&grid;
      for(const auto &p:slice.footprint.points) {geometry_msgs::msg::Point v;v.x=p.x;v.y=p.y;layer.footprint.push_back(v);}
      if(!layer.footprint.empty()) {
        require(convex(layer.footprint),"LAYERED_FOOTPRINT_NONCONVEX");nonempty=true;
        radius_=std::max(radius_,astribot_s1_robot_geometry::radius(layer.footprint));
        double lx=layer.footprint.front().x,ux=lx,ly=layer.footprint.front().y,uy=ly;
        for(const auto &p:layer.footprint) {
          lx=std::min(lx,p.x);ux=std::max(ux,p.x);ly=std::min(ly,p.y);uy=std::max(uy,p.y);
        }
        min_x_=std::min(min_x_,lx);max_x_=std::max(max_x_,ux);
        min_y_=std::min(min_y_,ly);max_y_=std::max(max_y_,uy);
      }
      // Integral occupancy makes empty-space samples constant-time. Unknown
      // remains blocked; a layer with no robot geometry is never queried.
      layer.blocked.assign((size_t(width_)+1)*(size_t(height_)+1),0);
      for(unsigned y=0;y<height_;++y) {
        uint64_t row=0;
        for(unsigned x=0;x<width_;++x) {
          const int value=grid.data[size_t(y)*width_+x];
          require(value>=-1&&value<=100,"LAYERED_GRID_VALUE_INVALID");
          row+=value<0||value>=65;
          layer.blocked[(size_t(y)+1)*(width_+1)+x+1]=layer.blocked[size_t(y)*(width_+1)+x+1]+row;
        }
      }
      layers->push_back(std::move(layer));
    }
    require(nonempty,"LAYERED_ROBOT_GEOMETRY_EMPTY");
    layers_=std::move(layers);
    auto occupied=std::make_shared<std::vector<uint64_t>>((size_t(width_)+1)*(size_t(height_)+1),0);
    for(const auto &layer:*layers_)if(!layer.footprint.empty())
      for(size_t i=0;i<occupied->size();++i)(*occupied)[i]+=layer.blocked[i];
    occupied_=std::move(occupied);
  }
  std::shared_ptr<const LayeredCollisionSnapshot> inFrame(const std::string &frame,double tx,double ty,double yaw) const {
    auto result=std::make_shared<LayeredCollisionSnapshot>(*this);
    const auto &origin=maps_->grids.front().info.origin;
    const double a=2.*std::atan2(origin.orientation.z,origin.orientation.w),c=std::cos(a),s=std::sin(a);
    result->frame_=frame;
    result->tx_=c*(tx-origin.position.x)+s*(ty-origin.position.y);
    result->ty_=-s*(tx-origin.position.x)+c*(ty-origin.position.y);
    result->angle_=yaw-a;result->c_=std::cos(result->angle_);result->s_=std::sin(result->angle_);
    return result;
  }
  double resolution() const {return resolution_;}
  double radius() const {return radius_;}
  unsigned sizeX() const {return width_;}
  unsigned sizeY() const {return height_;}
  const std::string &frame() const {return frame_;}
  const std::string &revision() const {return maps_->map_revision;}
  uint64_t envelopeEpoch() const {return envelope_->epoch;}
  const std::string &geometryHash() const {return envelope_->height_geometry_hash;}
  Polygon installedFootprint() const {
    Polygon result;
    for(const auto &p:envelope_->installed_footprint.points) {
      geometry_msgs::msg::Point point;point.x=p.x;point.y=p.y;result.push_back(point);
    }
    return result;
  }
  bool worldToMap(double x,double y,unsigned &mx,unsigned &my) const {
    const double gx=tx_+c_*x-s_*y,gy=ty_+s_*x+c_*y;
    if(!std::isfinite(gx)||!std::isfinite(gy)||gx<0.||gy<0.||gx>=width_*resolution_||gy>=height_*resolution_)return false;
    mx=static_cast<unsigned>(gx/resolution_);my=static_cast<unsigned>(gy/resolution_);return true;
  }
  void mapToWorld(unsigned mx,unsigned my,double &x,double &y) const {
    const double gx=(mx+.5)*resolution_-tx_,gy=(my+.5)*resolution_-ty_;
    x=c_*gx+s_*gy;y=-s_*gx+c_*gy;
  }
  bool collision(double x,double y,double yaw,double sampling_margin=0.) const {
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||!std::isfinite(sampling_margin)||sampling_margin<0.)return true;
    const double gx=tx_+c_*x-s_*y,gy=ty_+s_*x+c_*y;
    const double c=std::cos(yaw+angle_),s=std::sin(yaw+angle_);
    // One empty-space query across active height layers avoids per-layer polygon work.
    const double bx0=gx+c*(c>=0?min_x_:max_x_)-s*(s>=0?max_y_:min_y_)-sampling_margin;
    const double bx1=gx+c*(c>=0?max_x_:min_x_)-s*(s>=0?min_y_:max_y_)+sampling_margin;
    const double by0=gy+s*(s>=0?min_x_:max_x_)+c*(c>=0?min_y_:max_y_)-sampling_margin;
    const double by1=gy+s*(s>=0?max_x_:min_x_)+c*(c>=0?max_y_:min_y_)+sampling_margin;
    if(bx0>=0.&&by0>=0.&&bx1<width_*resolution_&&by1<height_*resolution_) {
      const unsigned x0=unsigned(bx0/resolution_),y0=unsigned(by0/resolution_);
      const unsigned x1=unsigned(bx1/resolution_),y1=unsigned(by1/resolution_);
      const auto at=[&](unsigned ix,unsigned iy){return (*occupied_)[size_t(iy)*(width_+1)+ix];};
      if(at(x1+1,y1+1)+at(x0,y0)==at(x0,y1+1)+at(x1+1,y0))return false;
    }
    for(const auto &layer:*layers_) {
      if(layer.footprint.empty())continue;
      Polygon p=layer.footprint;
      double lx=std::numeric_limits<double>::infinity(),ly=lx,ux=-lx,uy=-lx;
      for(auto &v:p) {const double px=gx+c*v.x-s*v.y;v.y=gy+s*v.x+c*v.y;v.x=px;
        lx=std::min(lx,v.x);ly=std::min(ly,v.y);ux=std::max(ux,v.x);uy=std::max(uy,v.y);}
      lx-=sampling_margin;ly-=sampling_margin;ux+=sampling_margin;uy+=sampling_margin;
      if(lx<0.||ly<0.||ux>=width_*resolution_||uy>=height_*resolution_)return true;
      const unsigned x0=unsigned(lx/resolution_),y0=unsigned(ly/resolution_),x1=unsigned(ux/resolution_),y1=unsigned(uy/resolution_);
      const auto at=[&](unsigned ix,unsigned iy){return layer.blocked[size_t(iy)*(width_+1)+ix];};
      if(at(x1+1,y1+1)+at(x0,y0)==at(x0,y1+1)+at(x1+1,y0))continue;
      for(unsigned iy=y0;iy<=y1;++iy)for(unsigned ix=x0;ix<=x1;++ix) {
        const int value=layer.grid->data[size_t(iy)*width_+ix];if(value>=0&&value<65)continue;
        if(intersects(p,ix*resolution_-sampling_margin,iy*resolution_-sampling_margin,
            (ix+1)*resolution_+sampling_margin,(iy+1)*resolution_+sampling_margin))return true;
      }
    }
    return false;
  }
  bool edgeCollision(double ax,double ay,double ayaw,double bx,double by,double byaw) const {
    const double distance=std::hypot(bx-ax,by-ay),turn=std::remainder(byaw-ayaw,2*M_PI);
    if(!std::isfinite(distance)||!std::isfinite(turn))return true;
    const double sweep=distance+radius_*std::abs(turn);
    const double samples=std::max(1.,std::ceil(sweep/(resolution_*.5)));
    if(samples>std::numeric_limits<int>::max())return true;
    const int steps=int(samples);const double margin=sweep/(2.*steps);
    for(int i=0;i<=steps;++i) {const double t=double(i)/steps;
      if(collision(ax+t*(bx-ax),ay+t*(by-ay),ayaw+t*turn,margin))return true;}
    return false;
  }
  // Exact constant body-twist arc. The caller selects the prediction horizon;
  // this geometric operation does not claim a braking model or motion authority.
  bool commandCollision(double x,double y,double yaw,double vx,double vy,double wz,double duration) const {
    const double sweep=(std::hypot(vx,vy)+radius_*std::abs(wz))*duration;
    const int steps=std::max(1,int(std::ceil(sweep/(resolution_*.5))));
    const double c=std::cos(yaw),s=std::sin(yaw),margin=sweep/(2.*steps);
    for(int i=0;i<=steps;++i) {
      const double t=duration*double(i)/steps;
      const double a=std::abs(wz)<1e-10?t:std::sin(wz*t)/wz;
      const double b=std::abs(wz)<1e-10?0.:(1.-std::cos(wz*t))/wz;
      const double dx=a*vx-b*vy,dy=b*vx+a*vy;
      if(collision(x+c*dx-s*dy,y+s*dx+c*dy,yaw+wz*t,margin))return true;
    }
    return false;
  }
private:
  struct Layer {
    const nav_msgs::msg::OccupancyGrid *grid;Polygon footprint;std::vector<uint64_t> blocked;
  };
  Maps::ConstSharedPtr maps_;Envelope::ConstSharedPtr envelope_;std::string frame_;
  std::shared_ptr<const std::vector<Layer>> layers_;double resolution_{},radius_{},tx_{},ty_{},angle_{},c_{1.},s_{};
  std::shared_ptr<const std::vector<uint64_t>> occupied_;
  double min_x_{std::numeric_limits<double>::infinity()},max_x_{-std::numeric_limits<double>::infinity()};
  double min_y_{std::numeric_limits<double>::infinity()},max_y_{-std::numeric_limits<double>::infinity()};
  unsigned width_{},height_{};
};
} // namespace astribot_s1_robot_geometry
