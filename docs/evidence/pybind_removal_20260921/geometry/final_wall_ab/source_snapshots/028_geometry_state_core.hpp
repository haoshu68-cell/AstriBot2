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
    std::optional<double> coverage_max,int64_t coverage_at,const std::string &filter_revision,double filter_age) {
  geometryRequire(captured.input_generation==current.input_generation && captured.model_generation==current.model_generation &&
    captured.attachment_revision==current.attachment_revision && captured.clock_epoch==current.clock_epoch && attachments_available,
    "GEOMETRY_INPUT_CHANGED_DURING_COMPUTE");
  geometryRequire(source_stamp<=now && now<valid_until,"GEOMETRY_COMPUTE_EXCEEDED_SOURCE_LEASE");
  geometryRequire(coverage_max && coverage_at<=now && now-coverage_at<=1500000000,"HEIGHT_FILTER_CONFIGURATION_UNAVAILABLE");
  geometryRequire(height<=*coverage_max,"GEOMETRY_EXCEEDS_CONFIGURED_HEIGHT_PROJECTION");
  geometryRequire(filter_revision==captured.attachment_revision && filter_age<=.5,"ATTACHMENT_FILTER_UNCONFIRMED");
}
inline const std::vector<std::string> &projectionParameters() {
  static const auto names=[] {
    std::vector<std::string> out{"input_cloud_topic","base_frame","enable_outlier_filter"};
    for(const auto *slice:{"low_obstacle","main_nav","torso_high","overhead"})for(const auto *field:{"enabled","z_min","z_max","min_points"})
      out.push_back(std::string("slices.")+slice+"."+field);
    return out;
  }();return names;
}
inline double projectionCeiling(const nlohmann::json &values,const std::string &frame) {
  geometryRequire(values.value("input_cloud_topic",std::string())=="/map_scan" && values.value("base_frame",std::string())==frame,
    "HEIGHT_PROJECTION_UNCLIPPED_SOURCE_REQUIRED");
  geometryRequire(values.contains("enable_outlier_filter") && values["enable_outlier_filter"].is_boolean() && !values["enable_outlier_filter"].get<bool>(),
    "HEIGHT_PROJECTION_SPARSE_OBSTACLES_MAY_BE_REMOVED");
  double end=-.03;
  for(const auto *slice:{"low_obstacle","main_nav","torso_high","overhead"}) {
    const auto prefix=std::string("slices.")+slice+".";
    geometryRequire(values.contains(prefix+"enabled") && values[prefix+"enabled"].is_boolean() && values[prefix+"enabled"].get<bool>() &&
      values.contains(prefix+"min_points") && values[prefix+"min_points"]==1,"HEIGHT_PROJECTION_SLICE_INCOMPLETE");
    geometryRequire(values.contains(prefix+"z_min") && values[prefix+"z_min"].is_number() && values.contains(prefix+"z_max") && values[prefix+"z_max"].is_number(),"HEIGHT_PROJECTION_GAP_OR_INVALID_RANGE");
    const double low=values[prefix+"z_min"],high=values[prefix+"z_max"];
    geometryRequire(std::isfinite(low) && std::isfinite(high) && low<=end+1e-9 && high>low,"HEIGHT_PROJECTION_GAP_OR_INVALID_RANGE");end=high;
  }
  return end;
}
} // namespace astribot_s1_robot_geometry
