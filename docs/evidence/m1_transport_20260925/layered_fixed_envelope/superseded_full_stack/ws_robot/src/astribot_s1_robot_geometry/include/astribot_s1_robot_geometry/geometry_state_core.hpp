#pragma once
#include "astribot_s1_robot_geometry/geometry_kernels.hpp"
#include <optional>
namespace astribot_s1_robot_geometry {
struct GeometryWorkContext {
  uint64_t input_generation,model_generation;
  std::string attachment_revision;
  uint64_t clock_epoch;
};
inline void validateGeometryCompletion(const GeometryWorkContext &captured,const GeometryWorkContext &current,
    bool attachments_available,int64_t source_stamp,int64_t valid_until,int64_t now,double height,
    std::optional<double> coverage_max,int64_t coverage_at,const std::string &filter_revision,double filter_age,double coverage_remaining) {
  geometryRequire(captured.input_generation==current.input_generation && captured.model_generation==current.model_generation &&
    captured.attachment_revision==current.attachment_revision && captured.clock_epoch==current.clock_epoch && attachments_available,
    "GEOMETRY_INPUT_CHANGED_DURING_COMPUTE");
  geometryRequire(source_stamp<=now && now<valid_until,"GEOMETRY_COMPUTE_EXCEEDED_SOURCE_LEASE");
  geometryRequire(coverage_max && coverage_at<=now && now-coverage_at<=1500000000 && coverage_remaining>=0.,"HEIGHT_MAP_COVERAGE_UNAVAILABLE");
  geometryRequire(height<=*coverage_max,"GEOMETRY_EXCEEDS_CONFIGURED_HEIGHT_PROJECTION");
  geometryRequire(filter_revision==captured.attachment_revision && filter_age<=.5,"ATTACHMENT_FILTER_UNCONFIRMED");
}
template<class Snapshot> inline double heightMapCoverage(const Snapshot &map,const std::string &profile_revision,
    const std::string &frame,const std::string &ground_reference,double ground_in_base,
    const std::vector<double> &base_edges,const std::vector<std::string> &layer_names) {
  geometryRequire(map.evidence_kind=="gazebo_collision_geometry","HEIGHT_MAP_COMPLETE_GEOMETRY_REQUIRED");
  geometryRequire(map.header.frame_id==frame && !map.map_revision.empty() && map.profile_revision==profile_revision,
    "HEIGHT_MAP_IDENTITY_MISMATCH");
  geometryRequire(std::isfinite(map.ground_z) && map.ground_reference==ground_reference,"HEIGHT_MAP_GROUND_REFERENCE_MISMATCH");
  geometryRequire(map.height_edges.size()==base_edges.size() && map.layer_names==layer_names &&
    map.grids.size()==layer_names.size() && map.point_counts.size()==layer_names.size(),"HEIGHT_MAP_LAYER_MISMATCH");
  for(std::size_t i=0;i<base_edges.size();++i)
    geometryRequire(map.height_edges[i]+ground_in_base==base_edges[i],"HEIGHT_MAP_EDGE_MISMATCH");
  for(const auto &grid:map.grids) {
    const auto &info=grid.info;const auto &p=info.origin.position;const auto &q=info.origin.orientation;
    geometryRequire(grid.header==map.header && info==map.grids.front().info,"HEIGHT_MAP_NONATOMIC_GRID");
    geometryRequire(info.width>0 && info.height>0 && std::isfinite(info.resolution) && info.resolution>0. &&
      grid.data.size()==uint64_t(info.width)*info.height,"HEIGHT_MAP_INVALID_GRID");
    geometryRequire(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
      std::isfinite(q.z) && std::isfinite(q.w) && q.x==0. && q.y==0. &&
      std::abs(q.z*q.z+q.w*q.w-1.)<=1e-9,"HEIGHT_MAP_INVALID_ORIGIN");
  }
  return base_edges.back();
}
} // namespace astribot_s1_robot_geometry
