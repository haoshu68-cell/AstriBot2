#include "astribot_s1_navigation_policy_native/policy_observer_core.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace astribot::navigation::policy {
double observer_yaw(const std::array<double,4>& q) {
  return std::atan2(2.*(q[3]*q[2]+q[0]*q[1]),1.-2.*(q[1]*q[1]+q[2]*q[2]));
}
std::array<double,3> observer_point(const std::array<double,3>& p,
                                  const std::array<double,7>& t) {
  if(!std::all_of(p.begin(),p.end(),[](double v){return std::isfinite(v);}))
    throw std::invalid_argument("finite transform input required");
  const double x=p[0],y=p[1],z=p[2],qx=t[3],qy=t[4],qz=t[5],qw=t[6];
  const double tx=2.*(qy*z-qz*y),ty=2.*(qz*x-qx*z),tz=2.*(qx*y-qy*x);
  const std::array<double,3> rotated{x+qw*tx+qy*tz-qz*ty,y+qw*ty+qz*tx-qx*tz,z+qw*tz+qx*ty-qy*tx};
  if(!std::all_of(rotated.begin(),rotated.end(),[](double v){return std::isfinite(v);}))
    throw std::invalid_argument("finite transform rotation required");
  return {rotated[0]+t[0],rotated[1]+t[1],rotated[2]+t[2]};
}
Version execution_version(const navigation::ExecutionContext& context) {
  const auto v=context.version();
  return Version(std::get<0>(v),std::get<1>(v),std::get<2>(v),std::get<3>(v),std::get<4>(v),std::get<5>(v));
}
bool ObserverMap::accept(const std::string& frame,std::uint32_t width,std::uint32_t height,
                         double resolution,const std::array<double,7>& origin,
                         const std::vector<std::int8_t>& cells) {
  if(frame!="map" || !std::isfinite(resolution) || resolution<=0. || cells.empty() ||
     static_cast<std::uint64_t>(width)*height!=cells.size())return false;
  const std::array<double,9> metadata{static_cast<double>(width),static_cast<double>(height),resolution,
    origin[0],origin[1],origin[3],origin[4],origin[5],origin[6]};
  if(!std::all_of(metadata.begin(),metadata.end(),[](double v){return std::isfinite(v);}))return false;
  width_=width;height_=height;metadata_=metadata;cells_=cells;has_map_=true;mask_.clear();return true;
}
std::array<double,5> ObserverMap::projection_info() const {
  if(!has_map_)throw std::logic_error("map required");
  const double theta=observer_yaw({metadata_[5],metadata_[6],metadata_[7],metadata_[8]});
  return {std::cos(theta),std::sin(theta),metadata_[3],metadata_[4],metadata_[2]};
}
const std::vector<std::vector<bool>>& ObserverMap::mask() const {
  if(!has_map_)throw std::logic_error("map required");
  if(mask_.empty()) {
    const auto width=static_cast<std::size_t>(width_),height=static_cast<std::size_t>(height_);
    mask_.assign(height+4,std::vector<bool>(width+4,false));
    // Dilate each occupied cell into its exact 5x5 neighbourhood, retaining
    // the two exterior cells on every side used by the original lookup.
    for(std::size_t y=0;y<height;++y)for(std::size_t x=0;x<width;++x) {
      if(cells_[y*width+x]<65)continue;
      for(std::size_t dy=0;dy<5;++dy)for(std::size_t dx=0;dx<5;++dx)mask_[y+dy][x+dx]=true;
    }
  }
  return mask_;
}
bool ObserverMap::static_at(double x,double y) const {
  if(!has_map_)return false;
  const auto info=projection_info();const double dx=x-info[2],dy=y-info[3];
  const double ix=std::floor((info[0]*dx+info[1]*dy)/info[4])+2.;
  const double iy=std::floor((-info[1]*dx+info[0]*dy)/info[4])+2.;
  if(std::isnan(ix)||std::isnan(iy))throw std::invalid_argument("cannot convert float NaN to integer");
  if(!std::isfinite(ix)||!std::isfinite(iy))throw std::invalid_argument("cannot convert float infinity to integer");
  const auto& values=mask();
  if(ix<0. || iy<0. || ix>=values.front().size() || iy>=values.size())return false;
  return values[static_cast<std::size_t>(iy)][static_cast<std::size_t>(ix)];
}
std::string ObserverMap::context_key() const {
  if(!has_map_)throw std::logic_error("map required");
  // Internal equality key only, never an external identity/hash. Keep the full
  // bytes so a metadata/cell change cannot be hidden by a hash collision; +0
  // and -0 compare equal as in Python's metadata tuple.
  std::string result;result.reserve(sizeof(double)*metadata_.size()+cells_.size());
  for(double value:metadata_) {
    if(value==0.)value=0.;
    result.append(reinterpret_cast<const char*>(&value),sizeof(value));
  }
  result.append(reinterpret_cast<const char*>(cells_.data()),cells_.size());return result;
}
}  // namespace astribot::navigation::policy
