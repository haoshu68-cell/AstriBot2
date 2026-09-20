// Batch geometry kernels. Acquisition freshness and ownership remain with the
// caller; no kernel changes observations, leases, padding or collision limits.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "astribot_s1_robot_geometry/source_time.hpp"
#include "astribot_s1_robot_geometry/joint_snapshot.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <set>
#include <vector>

namespace py = pybind11;
using Array = py::array_t<double, py::array::c_style | py::array::forcecast>;
constexpr double pi = 3.14159265358979323846;

void require(bool ok, const char * reason) {
  if (!ok) {throw std::invalid_argument(reason);}
}
void finite(const Array & a) {
  for (py::ssize_t i=0; i<a.size(); ++i) {
    require(std::isfinite(a.data()[i]), "finite geometry required");
  }
}

py::array_t<double> convexHull(const Array & input) {
  require(input.ndim()==2 && input.shape(1)==2,"finite XY vertices required");
  finite(input);const auto p=input.unchecked<2>();
  using Point=std::array<double,2>;std::vector<Point> points,lower,upper;
  points.reserve(input.shape(0));
  {
    py::gil_scoped_release release;
    for(py::ssize_t i=0;i<input.shape(0);++i)points.push_back({p(i,0),p(i,1)});
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
  require(lower.size()>=3,"nondegenerate polygon required");
  py::array_t<double> result({static_cast<py::ssize_t>(lower.size()),py::ssize_t{2}});
  auto out=result.mutable_unchecked<2>();
  for(std::size_t i=0;i<lower.size();++i) {out(i,0)=lower[i][0];out(i,1)=lower[i][1];}
  return result;
}

// rows: x, y, yaw, lower_x, lower_y, upper_x, upper_y.
py::array_t<double> boxDistance(const Array & polygon, const Array & rows) {
  require(polygon.ndim()==2 && polygon.shape(1)==2 && polygon.shape(0)>=3 &&
    polygon.shape(0)<=256, "polygon must be N x 2 with 3..256 vertices");
  require(rows.ndim()==2 && rows.shape(1)==7, "distance rows must be N x 7");
  finite(polygon);finite(rows);
  const auto p=polygon.unchecked<2>();const auto a=rows.unchecked<2>();
  struct Edge {double x,y,length2,nx,ny,lo,hi;};
  const auto n=polygon.shape(0);std::vector<Edge> edges;
  for (py::ssize_t j=0;j<n;++j) {
    const auto k=(j+1)%n;const double ex=p(k,0)-p(j,0),ey=p(k,1)-p(j,1);
    const double length=std::hypot(ex,ey);
    require(length>0., "polygon edges must be nonzero");
    Edge e{ex,ey,length*length,ey/length,-ex/length,
      std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()};
    for (py::ssize_t v=0;v<n;++v) {
      const double projection=p(v,0)*e.nx+p(v,1)*e.ny;
      e.lo=std::min(e.lo,projection);e.hi=std::max(e.hi,projection);
    }
    edges.push_back(e);
  }
  for(py::ssize_t i=0;i<rows.shape(0);++i) {
    require(a(i,3)<=a(i,5) && a(i,4)<=a(i,6), "ordered box bounds required");
  }
  py::array_t<double> result(rows.shape(0));auto out=result.mutable_unchecked<1>();
  {
    py::gil_scoped_release release;
    for (py::ssize_t i=0;i<rows.shape(0);++i) {
      const double x=a(i,0),y=a(i,1),c=std::cos(a(i,2)),s=std::sin(a(i,2));
      const double lx=a(i,3),ly=a(i,4),ux=a(i,5),uy=a(i,6);
      double xmin=INFINITY,ymin=INFINITY,xmax=-INFINITY,ymax=-INFINITY;
      double distance=INFINITY,separation=-INFINITY;
      for(py::ssize_t j=0;j<n;++j) {
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
      out(i)=separation>0. ? distance : separation;
    }
  }
  return result;
}

// boxes: center_x, center_y, size_x, size_y. Transform is capture-time
// tracking->scan, xyz then quaternion xyzw. Unknown rays always retain cells.
py::array_t<bool> scanBoxesFree(const Array & boxes, const Array & transform,
    const Array & ranges,double range_min,double range_max,double angle_min,
    double angle_increment,double resolution) {
  require(boxes.ndim()==2 && boxes.shape(1)==4, "boxes must be N x 4");
  require(transform.ndim()==1 && transform.size()==7, "transform must have 7 values");
  require(ranges.ndim()==1 && ranges.size()>0, "nonempty ranges required");
  finite(boxes);finite(transform);
  require(std::isfinite(range_min) && std::isfinite(range_max) && range_min>=0. &&
    range_max>range_min && std::isfinite(angle_min) && std::isfinite(angle_increment) &&
    angle_increment>0. && std::isfinite(resolution) && resolution>0., "invalid scan geometry");
  const auto b=boxes.unchecked<2>();const auto tf=transform.unchecked<1>();
  const auto r=ranges.unchecked<1>();
  require(std::abs(tf(3)*tf(3)+tf(4)*tf(4)+tf(5)*tf(5)+tf(6)*tf(6)-1.)<1e-5,
    "unit transform quaternion required");
  for(py::ssize_t i=0;i<boxes.shape(0);++i) {
    require(b(i,2)>=0. && b(i,3)>=0., "nonnegative box size required");
  }
  py::array_t<bool> result(boxes.shape(0));auto out=result.mutable_unchecked<1>();
  {
    py::gil_scoped_release release;
    const bool full=std::abs(ranges.size()*angle_increment-2*pi)<=1.5*angle_increment;
    const double qx=tf(3),qy=tf(4),qz=tf(5),qw=tf(6);
    for(py::ssize_t i=0;i<boxes.shape(0);++i) {
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
      out(i)=false;
      if(near<range_min || far>=range_max-.2 || high-low>=pi || far*angle_increment>resolution) {continue;}
      const double flo=std::floor((low-angle_min)/angle_increment),fhi=std::ceil((high-angle_min)/angle_increment);
      // Bound arithmetic before integer conversion. Excessive spans cannot
      // provide distinct measured rays and therefore cannot clear an obstacle.
      if (!std::isfinite(flo) || !std::isfinite(fhi) || std::abs(flo)>9e15 ||
          std::abs(fhi)>9e15 || fhi-flo>ranges.size()+1) {continue;}
      bool free=true;
      for(auto j=static_cast<int64_t>(flo);j<=static_cast<int64_t>(fhi);++j) {
        auto k=j;
        if(k<0 || k>=ranges.size()) {
          if(!full) {free=false;break;}
          k=(k%ranges.size()+ranges.size())%ranges.size();
        }
        const double observed=r(k);
        if(std::isnan(observed) || observed==-INFINITY || std::min(observed,range_max)<far+.15) {
          free=false;break;
        }
      }
      out(i)=free;
    }
  }
  return result;
}

// Capture-time scan projection and map filtering. The mask is the adapter's
// existing five-by-five occupied neighborhood, padded by two map cells.
py::array_t<int64_t> scanOccupiedCells(const Array & ranges, double range_min,
    double range_max, double angle_min, double angle_increment,
    const Array & tracking_tf, const Array & map_tf, const Array & map_info,
    const py::array_t<bool, py::array::c_style | py::array::forcecast> & mask,
    double resolution) {
  require(ranges.ndim()==1 && ranges.size()>0,"nonempty ranges required");
  require(tracking_tf.ndim()==1 && tracking_tf.size()==7 &&
    map_tf.ndim()==1 && map_tf.size()==7,"transforms must have 7 values");
  require(map_info.ndim()==1 && map_info.size()==5 && mask.ndim()==2 &&
    mask.shape(0)>0 && mask.shape(1)>0,"map metadata and mask required");
  finite(tracking_tf);finite(map_tf);finite(map_info);
  require(std::isfinite(range_min) && range_min>=0. && std::isfinite(range_max) &&
    range_max>range_min && std::isfinite(angle_min) && std::isfinite(angle_increment) &&
    angle_increment>0. && std::isfinite(resolution) && resolution>0.,"invalid scan geometry");
  const auto r=ranges.unchecked<1>();const auto track=tracking_tf.unchecked<1>();
  const auto map=map_tf.unchecked<1>();const auto info=map_info.unchecked<1>();
  const auto occupied=mask.unchecked<2>();
  for(const auto * tf:{&tracking_tf,&map_tf}) {
    const auto q=tf->unchecked<1>();
    require(std::abs(q(3)*q(3)+q(4)*q(4)+q(5)*q(5)+q(6)*q(6)-1.)<1e-5,
      "unit transform quaternion required");
  }
  require(info(4)>0. && std::abs(info(0)*info(0)+info(1)*info(1)-1.)<1e-5,
    "valid map resolution and rotation required");
  std::set<std::pair<int64_t,int64_t>> cells;
  {
    py::gil_scoped_release release;
    auto project=[](double x,double y,const auto & q) {
      const double tx=-2*q(5)*y,ty=2*q(5)*x,tz=2*(q(3)*y-q(4)*x);
      return std::array<double,2>{x+q(6)*tx+q(4)*tz-q(5)*ty+q(0),
        y+q(6)*ty+q(5)*tx-q(3)*tz+q(1)};
    };
    auto cell=[](double v) {
      require(std::isfinite(v) && std::abs(v)<9e15,"finite bounded scan cell required");
      return static_cast<int64_t>(std::floor(v));
    };
    for(py::ssize_t i=0;i<ranges.size();++i) {
      if(!std::isfinite(r(i)) || r(i)<range_min || r(i)>=range_max-.05)continue;
      const double angle=angle_min+i*angle_increment;
      const double x=r(i)*std::cos(angle),y=r(i)*std::sin(angle);
      const auto m=project(x,y,map);const double dx=m[0]-info(2),dy=m[1]-info(3);
      const auto mx=cell((info(0)*dx+info(1)*dy)/info(4))+2;
      const auto my=cell((-info(1)*dx+info(0)*dy)/info(4))+2;
      if(my>=0 && my<mask.shape(0) && mx>=0 && mx<mask.shape(1) && occupied(my,mx))continue;
      const auto p=project(x,y,track);
      cells.emplace(cell(p[0]/resolution),cell(p[1]/resolution));
    }
  }
  py::array_t<int64_t> result({static_cast<py::ssize_t>(cells.size()),py::ssize_t{2}});
  auto out=result.mutable_unchecked<2>();py::ssize_t i=0;
  for(const auto & c:cells) {out(i,0)=c.first;out(i,1)=c.second;++i;}
  return result;
}

PYBIND11_MODULE(_geometry_native,m) {
  m.doc()="Conservative batch geometry kernels; no clocks or robot authority";
  m.def("box_distance",&boxDistance);
  m.def("convex_hull",&convexHull);
  m.def("scan_boxes_free",&scanBoxesFree);
  m.def("scan_occupied_cells",&scanOccupiedCells);
  m.def("select_source_sample",&astribot_s1_robot_geometry::selectSourceSample,
    py::arg("stamps"),py::arg("now"),py::arg("maximum_age"),py::arg("maximum_future")=10000000);
  using Snapshot=astribot_s1_robot_geometry::JointSnapshot;
  py::class_<Snapshot>(m,"JointSnapshot")
    .def(py::init<std::vector<std::string>,int64_t,int64_t>(),py::arg("required"),
      py::arg("max_age_ns")=300000000,py::arg("max_skew_ns")=100000000)
    .def("update",&Snapshot::update).def("receive",&Snapshot::receive)
    .def("snapshot",&Snapshot::snapshot).def("observe_clock",&Snapshot::observeClock)
    .def("clear",&Snapshot::clear).def_property_readonly("epoch",&Snapshot::epoch);
}
