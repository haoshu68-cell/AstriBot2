#pragma once
// Pure C++ geometry kernels. No Python interpreter, array or robot authority.
#include <algorithm>
#include <array>
#include <cmath>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include <openssl/sha.h>
#include <vector>
namespace astribot_s1_robot_geometry {
using Point2=std::array<double,2>;
using Polygon2=std::vector<Point2>;
inline constexpr double pi=3.14159265358979323846;
inline void geometryRequire(bool ok,const char * reason) {if(!ok)throw std::invalid_argument(reason);}
template<class Values> inline void finiteValues(const Values & values) {
  for(double v:values) geometryRequire(std::isfinite(v),"finite geometry required");
}
template<class Rows> inline void finiteRows(const Rows & rows) {for(const auto & row:rows)finiteValues(row);}
inline Polygon2 convexHull(const Polygon2 & input) {
  finiteRows(input);const auto p=[&](std::size_t i,std::size_t j){return input[i][j];};
  using Point=std::array<double,2>;std::vector<Point> points,lower,upper;
  points.reserve(static_cast<std::ptrdiff_t>(input.size()));
  {
    for(std::ptrdiff_t i=0;i<static_cast<std::ptrdiff_t>(input.size());++i)points.push_back({p(i,0),p(i,1)});
    std::sort(points.begin(),points.end());points.erase(std::unique(points.begin(),points.end()),points.end());
    auto append=[](std::vector<Point> & chain,const Point & point) {
      while(chain.size()>=2) {
        const auto & a=chain[chain.size()-2];const auto & b=chain.back();
        const double cross=(b[0]-a[0])*(point[1]-a[1])-(b[1]-a[1])*(point[0]-a[0]);
        // Never delete an outward vertex using an absolute area epsilon.
        if(cross>0.)break;
        chain.pop_back();
      }
      chain.push_back(point);
    };
    for(const auto & point:points)append(lower,point);
    for(auto it=points.rbegin();it!=points.rend();++it)append(upper,*it);
    if(!lower.empty())lower.pop_back();
    if(!upper.empty())upper.pop_back();
    lower.insert(lower.end(),upper.begin(),upper.end());
  }
  geometryRequire(lower.size()>=3,"nondegenerate polygon required");
  return lower;
}

// rows: x, y, yaw, lower_x, lower_y, upper_x, upper_y.
inline std::vector<double> boxDistances(const Polygon2 & polygon,
    const std::vector<std::array<double,7>> & rows) {
  geometryRequire(polygon.size()>=3 && polygon.size()<=256,"polygon must have 3..256 vertices");
  finiteRows(polygon);finiteRows(rows);
  const auto p=[&](std::size_t i,std::size_t j){return polygon[i][j];};
  const auto a=[&](std::size_t i,std::size_t j){return rows[i][j];};
  struct Edge {double x,y,length2,nx,ny,lo,hi;};
  const auto n=static_cast<std::ptrdiff_t>(polygon.size());std::vector<Edge> edges;
  for (std::ptrdiff_t j=0;j<n;++j) {
    const auto k=(j+1)%n;const double ex=p(k,0)-p(j,0),ey=p(k,1)-p(j,1);
    const double length=std::hypot(ex,ey);
    geometryRequire(length>0., "polygon edges must be nonzero");
    Edge e{ex,ey,length*length,ey/length,-ex/length,
      std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()};
    for (std::ptrdiff_t v=0;v<n;++v) {
      const double projection=p(v,0)*e.nx+p(v,1)*e.ny;
      e.lo=std::min(e.lo,projection);e.hi=std::max(e.hi,projection);
    }
    edges.push_back(e);
  }
  for(std::ptrdiff_t i=0;i<static_cast<std::ptrdiff_t>(rows.size());++i) {
    geometryRequire(a(i,3)<=a(i,5) && a(i,4)<=a(i,6), "ordered box bounds required");
  }
  std::vector<double> result(rows.size());
  {
    for (std::ptrdiff_t i=0;i<static_cast<std::ptrdiff_t>(rows.size());++i) {
      const double x=a(i,0),y=a(i,1),c=std::cos(a(i,2)),s=std::sin(a(i,2));
      const double lx=a(i,3),ly=a(i,4),ux=a(i,5),uy=a(i,6);
      double xmin=INFINITY,ymin=INFINITY,xmax=-INFINITY,ymax=-INFINITY;
      double distance=INFINITY,separation=-INFINITY;
      for(std::ptrdiff_t j=0;j<n;++j) {
        const double px=x+c*p(j,0)-s*p(j,1),py=y+s*p(j,0)+c*p(j,1);
        xmin=std::min(xmin,px);xmax=std::max(xmax,px);
        ymin=std::min(ymin,py);ymax=std::max(ymax,py);
        distance=std::min(distance,std::hypot(std::max({lx-px,0.,px-ux}),
          std::max({ly-py,0.,py-uy})));
        const auto & e=edges[j];const double ex=c*e.x-s*e.y,ey=s*e.x+c*e.y;
        const double nx=c*e.nx-s*e.ny,ny=s*e.nx+c*e.ny;
        const double origin=nx*x+ny*y,center=nx*(lx+ux)/2+ny*(ly+uy)/2;
        const double extent=std::abs(nx)*(ux-lx)/2+std::abs(ny)*(uy-ly)/2;
        separation=std::max({separation,origin+e.lo-center-extent,center-extent-origin-e.hi});
        for (double bx:{lx,ux}) {for (double by:{ly,uy}) {
          const double dx=bx-px,dy=by-py;
          const double t=std::clamp((dx*ex+dy*ey)/e.length2,0.,1.);
          distance=std::min(distance,std::hypot(dx-t*ex,dy-t*ey));
        }}
      }
      separation=std::max({separation,xmin-ux,lx-xmax,ymin-uy,ly-ymax});
      result[i]=separation>0. ? distance : separation;
    }
  }
  return result;
}

// boxes: center_x, center_y, size_x, size_y. Transform is capture-time
// tracking->scan, xyz then quaternion xyzw. Unknown rays always retain cells.
inline std::vector<bool> scanBoxesFree(const std::vector<std::array<double,4>> & boxes,
    const std::array<double,7> & transform, const std::vector<double> & ranges,
    double range_min,double range_max,double angle_min,double angle_increment,double resolution) {
  geometryRequire(!ranges.empty(), "nonempty ranges required");
  finiteRows(boxes);finiteValues(transform);
  geometryRequire(std::isfinite(range_min) && std::isfinite(range_max) && range_min>=0. &&
    range_max>range_min && std::isfinite(angle_min) && std::isfinite(angle_increment) &&
    angle_increment>0. && std::isfinite(resolution) && resolution>0., "invalid scan geometry");
  const auto b=[&](std::size_t i,std::size_t j){return boxes[i][j];};
  const auto tf=[&](std::size_t i){return transform[i];};
  const auto r=[&](std::size_t i){return ranges[i];};
  geometryRequire(std::abs(tf(3)*tf(3)+tf(4)*tf(4)+tf(5)*tf(5)+tf(6)*tf(6)-1.)<1e-5,
    "unit transform quaternion required");
  for(std::ptrdiff_t i=0;i<static_cast<std::ptrdiff_t>(boxes.size());++i) {
    geometryRequire(b(i,2)>=0. && b(i,3)>=0., "nonnegative box size required");
  }
  std::vector<bool> result(boxes.size());
  {
    const bool full=std::abs(ranges.size()*angle_increment-2*pi)<=1.5*angle_increment;
    const double qx=tf(3),qy=tf(4),qz=tf(5),qw=tf(6);
    for(std::ptrdiff_t i=0;i<static_cast<std::ptrdiff_t>(boxes.size());++i) {
      std::array<std::array<double,2>,4> corners{};int index=0;double cx=0.,cy=0.;
      for(double dx:{-1.,1.}) {for(double dy:{-1.,1.}) {
        const double x=b(i,0)+dx*b(i,2)/2,y=b(i,1)+dy*b(i,3)/2;
        const double tx=-2*qz*y,ty=2*qz*x,tz=2*(qx*y-qy*x);
        const double px=x+qw*tx+qy*tz-qz*ty+tf(0);
        const double py=y+qw*ty+qz*tx-qx*tz+tf(1);
        corners[index++]={px,py};cx+=px/4;cy+=py/4;
      }}
      const double center=std::atan2(cy,cx);
      double low=INFINITY,high=-INFINITY,far=0.,near=INFINITY;
      for(const auto & p:corners) {
        const double angle=center+std::remainder(std::atan2(p[1],p[0])-center,2*pi);
        low=std::min(low,angle);high=std::max(high,angle);
        far=std::max(far,std::hypot(p[0],p[1]));near=std::min(near,std::hypot(p[0],p[1]));
      }
      result[i]=false;
      if(near<range_min || far>=range_max-.2 || high-low>=pi || far*angle_increment>resolution) {continue;}
      const double flo=std::floor((low-angle_min)/angle_increment),fhi=std::ceil((high-angle_min)/angle_increment);
      // Bound arithmetic before integer conversion. Excessive spans cannot
      // provide distinct measured rays and therefore cannot clear an obstacle.
      if (!std::isfinite(flo) || !std::isfinite(fhi) || std::abs(flo)>9e15 ||
          std::abs(fhi)>9e15 || fhi-flo>ranges.size()+1) {continue;}
      bool free=true;
      for(auto j=static_cast<int64_t>(flo);j<=static_cast<int64_t>(fhi);++j) {
        auto k=j;
        if(k<0 || k>=static_cast<int64_t>(ranges.size())) {
          if(!full) {free=false;break;}
          k=(k%static_cast<int64_t>(ranges.size())+static_cast<int64_t>(ranges.size()))%static_cast<int64_t>(ranges.size());
        }
        const double observed=r(k);
        if(std::isnan(observed) || observed==-INFINITY || std::min(observed,range_max)<far+.15) {
          free=false;break;
        }
      }
      result[i]=free;
    }
  }
  return result;
}

// Capture-time scan projection and map filtering. The mask is the adapter's
// existing five-by-five occupied neighborhood, padded by two map cells.
inline std::vector<std::array<int64_t,2>> scanOccupiedCells(const std::vector<double> & ranges,
    double range_min,double range_max,double angle_min,double angle_increment,
    const std::array<double,7> & tracking_tf,const std::array<double,7> & map_tf,
    const std::array<double,5> & map_info,const std::vector<std::vector<bool>> & mask,double resolution) {
  geometryRequire(!ranges.empty(),"nonempty ranges required");
  geometryRequire(!mask.empty() && !mask.front().empty(),"map metadata and mask required");
  for(const auto & row:mask) geometryRequire(row.size()==mask.front().size(),"rectangular map mask required");
  finiteValues(tracking_tf);finiteValues(map_tf);finiteValues(map_info);
  geometryRequire(std::isfinite(range_min) && range_min>=0. && std::isfinite(range_max) &&
    range_max>range_min && std::isfinite(angle_min) && std::isfinite(angle_increment) &&
    angle_increment>0. && std::isfinite(resolution) && resolution>0.,"invalid scan geometry");
  const auto r=[&](std::size_t i){return ranges[i];};
  const auto track=[&](std::size_t i){return tracking_tf[i];};
  const auto map=[&](std::size_t i){return map_tf[i];};
  const auto info=[&](std::size_t i){return map_info[i];};
  for(const auto * tf:{&tracking_tf,&map_tf}) {
    const auto q=[&](std::size_t i){return (*tf)[i];};
    geometryRequire(std::abs(q(3)*q(3)+q(4)*q(4)+q(5)*q(5)+q(6)*q(6)-1.)<1e-5,
      "unit transform quaternion required");
  }
  geometryRequire(info(4)>0. && std::abs(info(0)*info(0)+info(1)*info(1)-1.)<1e-5,
    "valid map resolution and rotation required");
  std::set<std::pair<int64_t,int64_t>> cells;
  {
    auto project=[](double x,double y,const auto & q) {
      const double tx=-2*q(5)*y,ty=2*q(5)*x,tz=2*(q(3)*y-q(4)*x);
      return std::array<double,2>{x+q(6)*tx+q(4)*tz-q(5)*ty+q(0),
        y+q(6)*ty+q(5)*tx-q(3)*tz+q(1)};
    };
    auto cell=[](double v) {
      geometryRequire(std::isfinite(v) && std::abs(v)<9e15,"finite bounded scan cell required");
      return static_cast<int64_t>(std::floor(v));
    };
    for(std::ptrdiff_t i=0;i<static_cast<std::ptrdiff_t>(ranges.size());++i) {
      if(!std::isfinite(r(i)) || r(i)<range_min || r(i)>=range_max-.05)continue;
      const double angle=angle_min+i*angle_increment;
      const double x=r(i)*std::cos(angle),y=r(i)*std::sin(angle);
      const auto m=project(x,y,map);const double dx=m[0]-info(2),dy=m[1]-info(3);
      const auto mx=cell((info(0)*dx+info(1)*dy)/info(4))+2;
      const auto my=cell((-info(1)*dx+info(0)*dy)/info(4))+2;
      if(my>=0 && my<static_cast<int64_t>(mask.size()) && mx>=0 && mx<static_cast<int64_t>(mask.front().size()) && mask[my][mx])continue;
      const auto p=project(x,y,track);
      cells.emplace(cell(p[0]/resolution),cell(p[1]/resolution));
    }
  }
  std::vector<std::array<int64_t,2>> result;
  for(const auto & cell:cells) result.push_back({cell.first,cell.second});
  return result;
}

inline Polygon2 validatePolygon(const Polygon2 & points) {
  geometryRequire(points.size()>=3 && points.size()<=256,"invalid polygon vertices");
  const auto hull=convexHull(points);
  geometryRequire(hull.size()==points.size(),"strictly convex polygon required");
  const auto n=points.size();
  for(std::size_t shift=0;shift<n;++shift) for(int sign:{-1,1}) {
    bool equal=true;
    for(std::size_t i=0;i<n;++i) {
      const auto j=(shift+(sign>0?i:n-i))%n;
      equal=equal && std::abs(points[i][0]-hull[j][0])<=1e-9 && std::abs(points[i][1]-hull[j][1])<=1e-9;
    }
    if(equal)return hull;
  }
  throw std::invalid_argument("cyclic convex polygon required");
}
inline Polygon2 inflatePolygon(const Polygon2 & points,double radius,std::size_t sides=32) {
  geometryRequire(std::isfinite(radius) && radius>=0,"invalid inflation radius");
  geometryRequire(sides>=3,"at least three inflation sides required");
  auto p=convexHull(points);
  if(radius==0)return p;
  Polygon2 disk;
  for(std::size_t i=0;i<sides;++i) {
    const double theta=i*2*pi/sides,scale=radius/std::cos(pi/sides);
    disk.push_back({scale*std::cos(theta),scale*std::sin(theta)});
  }
  auto rotate=[](Polygon2 & poly) {
    const auto first=std::min_element(poly.begin(),poly.end(),[](const auto &a,const auto &b){return a[1]==b[1]?a[0]<b[0]:a[1]<b[1];});
    std::rotate(poly.begin(),first,poly.end());
  };
  rotate(p);rotate(disk);
  Polygon2 out;
  const auto n=p.size(),m=disk.size();std::size_t i=0,j=0;
  while(i<n || j<m) {
    out.push_back({p[i%n][0]+disk[j%m][0],p[i%n][1]+disk[j%m][1]});
    if(i==n) {++j;continue;}
    if(j==m) {++i;continue;}
    const double ax=p[(i+1)%n][0]-p[i][0],ay=p[(i+1)%n][1]-p[i][1];
    const double bx=disk[(j+1)%m][0]-disk[j][0],by=disk[(j+1)%m][1]-disk[j][1];
    const double cross=ax*by-ay*bx;
    if(cross>=0)++i;
    if(cross<=0)++j;
  }
  return convexHull(out);
}
inline bool containsPolygon(const Polygon2 & outer,const Polygon2 & inner,double tolerance=1e-9) {
  const auto p=validatePolygon(outer);finiteRows(inner);
  geometryRequire(std::isfinite(tolerance) && tolerance>=0,"invalid containment tolerance");
  for(std::size_t i=0;i<p.size();++i) {
    const auto &a=p[i];const auto &b=p[(i+1)%p.size()];
    const double x=b[0]-a[0],y=b[1]-a[1],threshold=-tolerance*std::hypot(x,y);
    for(const auto &q:inner)if(x*(q[1]-a[1])-y*(q[0]-a[0])<threshold)return false;
  }
  return true;
}
inline Polygon2 serializedPolygon(const Polygon2 & points) {
  double scale=1.;for(const auto &p:points)for(double v:p)scale=std::max(scale,std::abs(v));
  auto out=inflatePolygon(points,8*std::numeric_limits<float>::epsilon()*scale);
  for(auto &p:out)for(auto &v:p) {
    // The wire representation must exist before hull ordering. A cast back to
    // double can retain excess precision in an optimized/vectorized tail;
    // observable float storage makes Release and Debug transport identical.
    volatile float wire=static_cast<float>(v);
    v=wire;
  }
  return convexHull(out);
}
inline std::string sha256(const std::string & bytes) {
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(bytes.data()),bytes.size(),digest);
  const char *hex="0123456789abcdef";std::string out;out.reserve(64);
  for(auto v:digest) {out+=hex[v>>4];out+=hex[v&15];}
  return out;
}
// Python json.dumps(sort_keys=True) is part of the installed geometry identity.
// Keep shortest-roundtrip doubles, Python's fixed/scientific thresholds and
// negative zero. Values are never quantized for collision computations.
inline std::string pythonFloat(double value) {
  geometryRequire(std::isfinite(value),"finite JSON float required");
  if(value==0)return std::signbit(value)?"-0.0":"0.0";
  char buffer[64];const auto result=std::to_chars(buffer,buffer+64,value,std::chars_format::general);
  geometryRequire(result.ec==std::errc(),"JSON float formatting failed");
  std::string raw(buffer,result.ptr),sign;
  if(raw.front()=='-') {sign="-";raw.erase(0,1);}
  const auto epos=raw.find('e');int exponent=0;
  if(epos!=std::string::npos) {exponent=std::stoi(raw.substr(epos+1));raw.resize(epos);}
  const auto dot=raw.find('.');int decimal=dot==std::string::npos?static_cast<int>(raw.size()):static_cast<int>(dot);
  if(dot!=std::string::npos)raw.erase(dot,1);
  decimal+=exponent;
  while(raw.size()>1 && raw.front()=='0') {raw.erase(0,1);--decimal;}
  const int magnitude=decimal-1;
  if(magnitude>=-4 && magnitude<16) {
    if(decimal<=0)return sign+"0."+std::string(-decimal,'0')+raw;
    if(decimal>=static_cast<int>(raw.size()))return sign+raw+std::string(decimal-raw.size(),'0')+".0";
    raw.insert(decimal,".");return sign+raw;
  }
  while(raw.size()>1 && raw.back()=='0')raw.pop_back();
  std::string exp=std::to_string(std::abs(magnitude));if(exp.size()<2)exp="0"+exp;
  return sign+raw.substr(0,1)+(raw.size()>1?"."+raw.substr(1):"")+"e"+(magnitude<0?"-":"+")+exp;
}
inline std::string pythonJson(const nlohmann::json &value) {
  if(value.is_number_float())return pythonFloat(value.get<double>());
  if(value.is_array()) {
    std::string out="[";bool first=true;
    for(const auto &v:value) {if(!first)out+=", ";first=false;out+=pythonJson(v);}return out+"]";
  }
  if(value.is_object()) {
    std::string out="{";bool first=true;
    for(auto it=value.begin();it!=value.end();++it) {if(!first)out+=", ";first=false;out+=nlohmann::json(it.key()).dump(-1,' ',true)+": "+pythonJson(it.value());}return out+"}";
  }
  return value.dump(-1,' ',true);
}
inline std::string geometryHash(const Polygon2 & points,const std::string &frame,double clearance) {
  auto vertices=validatePolygon(points);
  for(auto &p:vertices)for(auto &v:p)v=std::nearbyint(v*1e6)/1e6;
  return sha256(pythonJson(nlohmann::json{{"frame",frame},{"clearance",clearance},{"vertices",vertices}}));
}

} // namespace astribot_s1_robot_geometry
