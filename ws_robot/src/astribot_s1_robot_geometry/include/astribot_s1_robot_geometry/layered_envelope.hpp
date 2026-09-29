#pragma once
#include "astribot_s1_robot_geometry/geometry_kernels.hpp"
#include <astribot_navigation_msgs/msg/envelope_slice.hpp>

namespace astribot_s1_robot_geometry {
inline Polygon2 envelopePolygonPoints(const geometry_msgs::msg::Polygon &polygon) {
  Polygon2 points;
  for(const auto &p:polygon.points) {
    geometryRequire(std::isfinite(p.z) && p.z==0.,"HEIGHT_SLICE_NONPLANAR_POLYGON");
    points.push_back({p.x,p.y});
  }
  return points;
}
inline void validateHeightSlices(const std::vector<astribot_navigation_msgs::msg::EnvelopeSlice> &slices) {
  geometryRequire(!slices.empty(),"HEIGHT_SLICES_REQUIRED");
  for(std::size_t i=0;i<slices.size();++i) {
    const auto &s=slices[i];
    geometryRequire(std::isfinite(s.z_min_m) && std::isfinite(s.z_max_m) && s.z_min_m<s.z_max_m,
      "HEIGHT_SLICE_INVALID_RANGE");
    geometryRequire(!i || slices[i-1].z_max_m==s.z_min_m,"HEIGHT_SLICE_GAP_OR_OVERLAP");
    if(!s.footprint.points.empty())validatePolygon(envelopePolygonPoints(s.footprint));
  }
}
inline void validateHeightProfile(const std::string &revision,double ground_in_base) {
  geometryRequire(revision.size()==64 && std::all_of(revision.begin(),revision.end(),[](char c) {
    return (c>='0' && c<='9') || (c>='a' && c<='f');
  }),"HEIGHT_PROFILE_REVISION_INVALID");
  geometryRequire(std::isfinite(ground_in_base),"GROUND_REFERENCE_INVALID");
}
// Hash the actual wire polygons, preserving all float32 coordinates without
// micrometre rounding. Canonical hull order makes vertex rotation immaterial.
inline std::string layeredGeometryHash(const Polygon2 &installed_footprint,
    const std::vector<astribot_navigation_msgs::msg::EnvelopeSlice> &slices,
    const std::string &frame,double clearance,const std::string &profile_revision,double ground_in_base) {
  auto canonical=[](const Polygon2 &points) {
    auto vertices=validatePolygon(points);
    for(auto &p:vertices)for(auto &v:p)if(v==0.)v=0.;
    return vertices;
  };
  nlohmann::json layers=nlohmann::json::array();
  for(const auto &s:slices)layers.push_back({
    {"z_min",s.z_min_m==0.?0.:s.z_min_m},{"z_max",s.z_max_m==0.?0.:s.z_max_m},
    {"vertices",s.footprint.points.empty()?Polygon2{}:canonical(envelopePolygonPoints(s.footprint))}});
  return sha256(pythonJson(nlohmann::json{{"frame",frame},{"clearance",clearance==0.?0.:clearance},
    {"height_profile_revision",profile_revision},{"ground_in_base",ground_in_base==0.?0.:ground_in_base},
    {"vertices",canonical(installed_footprint)},{"height_slices",layers}}));
}
// Geometry identity only: callers retain source leases, ownership and map gates.
template<class Envelope> inline void validateLayeredEnvelope(const Envelope &e) {
  validateHeightSlices(e.height_slices);validateHeightProfile(e.height_profile_revision,e.ground_in_base_m);
  geometryRequire(!e.header.frame_id.empty(),"GEOMETRY_FRAME_REQUIRED");
  geometryRequire(std::isfinite(e.clearance_m) && e.clearance_m>=0.,"INVALID_CLEARANCE");
  geometryRequire(std::isfinite(e.limits.height_m) && e.limits.height_m>0. &&
    e.limits.height_m<=e.height_slices.back().z_max_m,"GEOMETRY_EXCEEDS_HEIGHT_PROFILE");
  geometryRequire(e.height_geometry_hash==layeredGeometryHash(envelopePolygonPoints(e.installed_footprint),
    e.height_slices,e.header.frame_id,e.clearance_m,e.height_profile_revision,e.ground_in_base_m),
    "HEIGHT_GEOMETRY_HASH_MISMATCH");
}
} // namespace astribot_s1_robot_geometry
