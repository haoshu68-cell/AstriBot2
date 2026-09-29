// Batch geometry kernels. Acquisition freshness and ownership remain with the
// caller; no kernel changes observations, leases, padding or collision limits.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "astribot_s1_robot_geometry/geometry_kernels.hpp"
#include "astribot_s1_robot_geometry/source_time.hpp"
#include "astribot_s1_robot_geometry/lease_time.hpp"
#include "astribot_s1_robot_geometry/joint_snapshot.hpp"
#include "astribot_s1_robot_geometry/fusion_snapshot.hpp"
#include "astribot_s1_robot_geometry/snapshot_objects.hpp"
#include "astribot_s1_robot_geometry/corridor_turns.hpp"
#include "astribot_s1_robot_geometry/passage_sweep.hpp"
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

template<std::size_t N> auto rows(const Array & input) {
  require(input.ndim()==2 && input.shape(1)==N,"invalid geometry row shape");
  std::vector<std::array<double,N>> out(static_cast<std::size_t>(input.shape(0)));
  for(py::ssize_t i=0;i<input.shape(0);++i)for(std::size_t j=0;j<N;++j)out[i][j]=input.data()[i*N+j];
  return out;
}
template<std::size_t N> auto values(const Array & input) {
  require(input.ndim()==1 && input.size()==N,"invalid geometry vector shape");
  std::array<double,N> out;std::copy(input.data(),input.data()+N,out.begin());return out;
}
auto rangesVector(const Array & input) {
  require(input.ndim()==1,"one-dimensional ranges required");
  return std::vector<double>(input.data(),input.data()+input.size());
}
template<class T,std::size_t N> py::array_t<T> toArray(const std::vector<std::array<T,N>> & input) {
  py::array_t<T> result({static_cast<py::ssize_t>(input.size()),static_cast<py::ssize_t>(N)});
  auto out=result.template mutable_unchecked<2>();
  for(std::size_t i=0;i<input.size();++i)for(std::size_t j=0;j<N;++j)out(i,j)=input[i][j];
  return result;
}
template<class T> py::array_t<T> toArray(const std::vector<T> & input) {
  py::array_t<T> result(input.size());auto out=result.template mutable_unchecked<1>();
  for(std::size_t i=0;i<input.size();++i)out(i)=input[i];
  return result;
}
py::array_t<double> convexHull(const Array & input) {
  auto decoded=rows<2>(input);astribot_s1_robot_geometry::Polygon2 result;
  {py::gil_scoped_release release;result=astribot_s1_robot_geometry::convexHull(decoded);}
  return toArray(result);
}
py::array_t<double> boxDistance(const Array & polygon,const Array & input) {
  auto p=rows<2>(polygon);auto decoded=rows<7>(input);std::vector<double> result;
  {py::gil_scoped_release release;result=astribot_s1_robot_geometry::boxDistances(p,decoded);}
  return toArray(result);
}
py::array_t<bool> scanBoxesFree(const Array & boxes,const Array & transform,const Array & ranges,
    double range_min,double range_max,double angle_min,double angle_increment,double resolution) {
  auto b=rows<4>(boxes);auto tf=values<7>(transform);auto r=rangesVector(ranges);std::vector<bool> result;
  {py::gil_scoped_release release;result=astribot_s1_robot_geometry::scanBoxesFree(b,tf,r,range_min,range_max,angle_min,angle_increment,resolution);}
  return toArray(result);
}
py::array_t<int64_t> scanOccupiedCells(const Array & ranges,double range_min,double range_max,
    double angle_min,double angle_increment,const Array & tracking_tf,const Array & map_tf,
    const Array & map_info,const py::array_t<bool,py::array::c_style|py::array::forcecast> & mask,double resolution) {
  auto r=rangesVector(ranges);auto track=values<7>(tracking_tf);auto map=values<7>(map_tf);auto info=values<5>(map_info);
  require(mask.ndim()==2,"two-dimensional map mask required");
  std::vector<std::vector<bool>> m(mask.shape(0),std::vector<bool>(mask.shape(1)));
  auto source=mask.unchecked<2>();
  for(py::ssize_t y=0;y<mask.shape(0);++y)for(py::ssize_t x=0;x<mask.shape(1);++x)m[y][x]=source(y,x);
  std::vector<std::array<int64_t,2>> result;
  {py::gil_scoped_release release;result=astribot_s1_robot_geometry::scanOccupiedCells(r,range_min,range_max,angle_min,angle_increment,track,map,info,m,resolution);}
  return toArray(result);
}

// Thin bridge for the fusion snapshot kernel. Input rows are decoded into
// SnapshotTrackInput; the kernel derives per-track center/size/covariance/
// velocity/variance/relevance. Row layout (32 columns):
//   [0]=capture_ns  [1..3]=center  [4..6]=size  [7..15]=pos_cov
//   [16]=spatial_occupancy  [17]=velocity_observable  [18]=has_velocity_m_s
//   [19]=has_velocity_covariance  [20..22]=track_velocity  [23..31]=velocity_covariance
py::dict snapshotTracks(const Array & rows, int64_t now_ns, const py::dict & params,
                        const py::array_t<int64_t, py::array::c_style | py::array::forcecast> & sample_offsets,
                        const Array & sample_t, const Array & sample_x, const Array & sample_y,
                        const std::vector<int64_t> & capture_stamps) {
  require(rows.ndim()==2 && rows.shape(1)==32, "snapshot rows must be N x 32");
  const auto a = rows.unchecked<2>();
  const py::ssize_t n = rows.shape(0);
  require(capture_stamps.empty() || capture_stamps.size()==static_cast<std::size_t>(n),
    "capture stamps must have N entries");
  require(sample_offsets.ndim()==1 && sample_offsets.size()==n+1,
          "sample offsets must have N+1 entries");
  require(sample_t.ndim()==1 && sample_x.ndim()==1 && sample_y.ndim()==1 &&
          sample_t.size()==sample_x.size() && sample_t.size()==sample_y.size(),
          "sample arrays must be 1-D and equal length");
  const auto soff = sample_offsets.unchecked<1>();
  require(soff(0)==0 && soff(n)==sample_t.size(),"sample offsets must span all samples");
  const auto st = sample_t.unchecked<1>();const auto sx = sample_x.unchecked<1>();const auto sy = sample_y.unchecked<1>();
  std::vector<astribot_s1_robot_geometry::SnapshotTrackInput> inputs(static_cast<std::size_t>(n));
  for (py::ssize_t i=0;i<n;++i) {
    auto & in = inputs[static_cast<std::size_t>(i)];
    if(capture_stamps.empty()) {
      // Legacy analytical input only. Runtime timestamps use the integer column
      // to retain single-nanosecond lease boundaries above 2^53.
      require(std::isfinite(a(i,0)) && a(i,0)>=0. && a(i,0)<=9007199254740991. &&
        a(i,0)==std::floor(a(i,0)),"capture_ns requires exact integer stamps");
      in.capture_ns=static_cast<int64_t>(a(i,0));
    } else in.capture_ns=capture_stamps[static_cast<std::size_t>(i)];
    for(int k=16;k<=19;++k)require(a(i,k)==0. || a(i,k)==1.,"snapshot flags must be boolean");
    for (int k=0;k<3;++k) in.center[k]=a(i,1+k);
    for (int k=0;k<3;++k) in.size[k]=a(i,4+k);
    for (int k=0;k<9;++k) in.pos_cov[k]=a(i,7+k);
    in.spatial_occupancy = a(i,16)!=0.0;
    in.velocity_observable = a(i,17)!=0.0;
    in.has_velocity_m_s = a(i,18)!=0.0;
    in.has_velocity_covariance = a(i,19)!=0.0;
    for (int k=0;k<3;++k) in.track_velocity[k]=a(i,20+k);
    for (int k=0;k<9;++k) in.velocity_covariance[k]=a(i,23+k);
    const int64_t lo = soff(i), hi = soff(i+1);
    require(lo>=0 && hi>=lo && hi<=sample_t.size(), "invalid sample offset range");
    for (int64_t j=lo;j<hi;++j) {
      in.samples.push_back({st(j), sx(j), sy(j)});
    }
  }

  astribot_s1_robot_geometry::SnapshotParams profile;
  auto getd = [&](const char * key, double dflt) {
    return (params.contains(key) && !params[key].is_none())
        ? params[key].cast<double>() : dflt;
  };
  profile.track_memory_s = getd("track_memory_s", profile.track_memory_s);
  profile.velocity_confirmation_s = getd("velocity_confirmation_s", profile.velocity_confirmation_s);
  profile.min_tracked_speed_m_s = getd("min_tracked_speed_m_s", profile.min_tracked_speed_m_s);
  profile.velocity_fit_max_residual_m = getd("velocity_fit_max_residual_m", profile.velocity_fit_max_residual_m);
  profile.stationary_velocity_variance_m2_s2 = getd("stationary_velocity_variance_m2_s2", profile.stationary_velocity_variance_m2_s2);
  profile.prediction_horizon_s = getd("prediction_horizon_s", profile.prediction_horizon_s);
  profile.half_length_m = getd("half_length_m", profile.half_length_m);
  profile.half_width_m = getd("half_width_m", profile.half_width_m);
  profile.clearance_margin_m = getd("clearance_margin_m", profile.clearance_margin_m);
  profile.payload_extra_margin_m = getd("payload_extra_margin_m", profile.payload_extra_margin_m);
  profile.has_region = params.contains("region") && !params["region"].is_none();
  if (profile.has_region) {
    const auto r = params["region"].cast<std::array<double,3>>();
    profile.region_x=r[0];profile.region_y=r[1];profile.region_travel=r[2];
  }

  std::vector<astribot_s1_robot_geometry::SnapshotTrackOutput> outputs;
  {
    py::gil_scoped_release release;
    astribot_s1_robot_geometry::snapshotTracks(inputs, now_ns, profile, outputs);
  }

  py::array_t<double> centers({n,py::ssize_t{3}}), sizes({n,py::ssize_t{3}}),
      covs({n,py::ssize_t{9}}), vels({n,py::ssize_t{3}});
  py::array_t<double> variances(n);
  py::array_t<bool> relevants(n);
  auto c=centers.mutable_unchecked<2>();auto s=sizes.mutable_unchecked<2>();
  auto cv=covs.mutable_unchecked<2>();auto v=vels.mutable_unchecked<2>();
  auto va=variances.mutable_unchecked<1>();auto rl=relevants.mutable_unchecked<1>();
  for (py::ssize_t i=0;i<n;++i) {
    const auto & o = outputs[static_cast<std::size_t>(i)];
    for (int k=0;k<3;++k){c(i,k)=o.center[k];s(i,k)=o.size[k];v(i,k)=o.velocity[k];}
    for (int k=0;k<9;++k) cv(i,k)=o.covariance[k];
    va(i)=o.variance;rl(i)=o.relevant;
  }
  py::dict result;
  result["centers"]=centers;result["sizes"]=sizes;result["covariances"]=covs;
  result["velocities"]=vels;result["variances"]=variances;result["relevant"]=relevants;
  return result;
}

PYBIND11_MODULE(_geometry_native,m) {
  m.def("corridor_turns_outside", [](const Array & polygon, const Array & path,
      double length, double width, double margin, double heading_limit) {
    auto p=rows<2>(polygon);auto route=rows<2>(path);
    py::gil_scoped_release release;
    return astribot_s1_robot_geometry::corridorTurnsOutside(p,route,length,width,margin,heading_limit);
  });
  m.doc()="Conservative batch geometry kernels; no clocks or robot authority";
  m.def("box_distance",&boxDistance);
  m.def("passage_sweep",[](const Array & polygon,const Array & start,const Array & end,double margin) {
    return toArray(astribot_s1_robot_geometry::passageSweep(rows<2>(polygon),values<3>(start),values<3>(end),margin));
  });
  m.def("passage_grid_clear",[](const Array & polygon,const Array & origin,double resolution,
      int width,int height,const std::vector<int> & cells) {
    return astribot_s1_robot_geometry::passageGridClear(rows<2>(polygon),values<3>(origin),resolution,width,height,cells);
  });
  m.def("convex_hull",&convexHull);
  m.def("scan_boxes_free",&scanBoxesFree);
  m.def("scan_occupied_cells",&scanOccupiedCells);
  m.def("snapshot_tracks",&snapshotTracks,py::arg("rows"),py::arg("now_ns"),py::arg("params"),
        py::arg("sample_offsets"),py::arg("sample_t"),py::arg("sample_x"),py::arg("sample_y"),
        py::arg("capture_stamps")=std::vector<int64_t>{});
  m.def("snapshot_objects", &astribot_s1_robot_geometry::snapshot_objects_detail::snapshotObjects,
        py::arg("fusion"), py::arg("now"), py::arg("region"), py::arg("fallback"));
  m.def("select_source_sample",&astribot_s1_robot_geometry::selectSourceSample,
    py::arg("stamps"),py::arg("now"),py::arg("maximum_age"),py::arg("maximum_future")=10000000);
  using Lease=astribot_s1_robot_geometry::LeaseSample;
  py::class_<Lease>(m,"LeaseSample").def(py::init([](int64_t stamp,int64_t limits_stamp,
    int64_t until,double lease,bool allowed,std::string context){
      return Lease{stamp,limits_stamp,until,lease,allowed,std::move(context)};
    }));
  py::class_<astribot_s1_robot_geometry::LeaseSelector>(m,"LeaseSelector")
    .def(py::init<>()).def("select",&astribot_s1_robot_geometry::LeaseSelector::select);
  using Snapshot=astribot_s1_robot_geometry::JointSnapshot;
  py::class_<Snapshot>(m,"JointSnapshot")
    .def(py::init<std::vector<std::string>,int64_t,int64_t>(),py::arg("required"),
      py::arg("max_age_ns")=300000000,py::arg("max_skew_ns")=100000000)
    .def("update",&Snapshot::update).def("receive",&Snapshot::receive)
    .def("snapshot",&Snapshot::snapshot).def("observe_clock",&Snapshot::observeClock)
    .def("clear",&Snapshot::clear).def_property_readonly("epoch",&Snapshot::epoch);
}
