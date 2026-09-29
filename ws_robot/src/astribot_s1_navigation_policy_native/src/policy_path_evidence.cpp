#include "astribot_s1_navigation_policy_native/policy_path_evidence.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace astribot::navigation::policy {
namespace {
PathHeaderIdentity identity(const std_msgs::msg::Header& h) { return {h.frame_id,h.stamp.sec,h.stamp.nanosec}; }
}
PathIdentity path_identity(const nav_msgs::msg::Path& path) {
  PathIdentity result{identity(path.header),{}};result.poses.reserve(path.poses.size());
  for(const auto& point:path.poses) {
    const auto& p=point.pose.position;const auto& q=point.pose.orientation;
    const std::array<double,7> values{p.x,p.y,p.z,q.x,q.y,q.z,q.w};
    if(!std::all_of(values.begin(),values.end(),[](double v){return std::isfinite(v);}))throw std::invalid_argument("nonfinite path geometry");
    result.poses.push_back({identity(point.header),values});
  }
  return result;
}
navigation::PathAssessmentResult assess_path(const std::optional<PathEvidence>& evidence,
    const std::optional<PathIdentity>& path_key,double now_s,double wall_s,
    std::int64_t epoch,bool legacy_blocked,const PathRiskProfile& profile) {
  if(!evidence||!path_key||!(evidence->path_key==*path_key)||evidence->epoch!=epoch)
    return {legacy_blocked,legacy_blocked?0.:INFINITY,"LEGACY",0.};
  return navigation::assess_path(evidence->known,evidence->blocked,evidence->stamp_s,
      evidence->received_wall_s,evidence->distance_m,now_s,wall_s,
      profile.path_risk_timeout_s,profile.max_speed_m_s,profile.clearance_margin_m,
      profile.payload_extra_margin_m,legacy_blocked);
}
}  // namespace astribot::navigation::policy
