#pragma once
#include "astribot_s1_navigation_policy_native/navigation_math.hpp"
#include <nav_msgs/msg/path.hpp>
#include <array>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace astribot::navigation::policy {
struct PathHeaderIdentity {
  std::string frame;
  std::int32_t sec;
  std::uint32_t nanosec;
  bool operator==(const PathHeaderIdentity& other) const {
    return std::tie(frame,sec,nanosec)==std::tie(other.frame,other.sec,other.nanosec);
  }
};
struct PathPoseIdentity {
  PathHeaderIdentity header;
  std::array<double,7> geometry;
  bool operator==(const PathPoseIdentity& other) const { return header==other.header&&geometry==other.geometry; }
};
struct PathIdentity {
  PathHeaderIdentity header;
  std::vector<PathPoseIdentity> poses;
  bool operator==(const PathIdentity& other) const { return header==other.header&&poses==other.poses; }
};
PathIdentity path_identity(const nav_msgs::msg::Path& path);
struct PathEvidence {
  PathIdentity path_key;
  double stamp_s,received_wall_s;
  std::int64_t epoch;
  bool known,blocked;
  double distance_m;
};
struct PathRiskProfile {
  double path_risk_timeout_s,max_speed_m_s,clearance_margin_m,payload_extra_margin_m;
};
navigation::PathAssessmentResult assess_path(const std::optional<PathEvidence>& evidence,
    const std::optional<PathIdentity>& path_key,double now_s,double wall_s,
    std::int64_t epoch,bool legacy_blocked,const PathRiskProfile& profile);
}  // namespace astribot::navigation::policy
