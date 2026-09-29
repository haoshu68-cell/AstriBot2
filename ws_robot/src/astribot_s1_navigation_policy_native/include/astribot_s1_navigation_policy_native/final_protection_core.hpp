#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <astribot_navigation_msgs/msg/robot_envelope.hpp>
#include <astribot_navigation_msgs/msg/navigation_envelope_v2.hpp>
#include <astribot_s1_robot_geometry/geometry_kernels.hpp>

namespace astribot::navigation {
using ProtectionEnvelope = astribot_navigation_msgs::msg::RobotEnvelope;
using ProtectionEnvelopeV2 = astribot_navigation_msgs::msg::NavigationEnvelopeV2;
using ProtectionPolygon = astribot_s1_robot_geometry::Polygon2;

// Source and receipt clocks are explicit. Malformed V2 updates revoke the
// current authorization; legacy validation preserves the last valid message.
class ProtectionProfile {
public:
  explicit ProtectionProfile(nlohmann::json baseline, bool fixed);
  double value(const char* key) const;
  const std::string& base_frame() const { return base_frame_; }
  bool simulated() const { return baseline_.at("environment") == "simulation"; }
  bool accept(const ProtectionEnvelope& message, std::int64_t now_ns, std::uint64_t clock_epoch);
  bool accept(const ProtectionEnvelopeV2& message, std::int64_t now_ns, std::uint64_t clock_epoch);
  bool ready(std::int64_t now_ns, std::uint64_t clock_epoch) const;
  const ProtectionPolygon& polygon() const { return polygon_; }

private:
  void validate(const ProtectionEnvelope& message) const;
  nlohmann::json baseline_;
  std::string base_frame_;
  bool fixed_;
  std::optional<ProtectionEnvelope> envelope_;
  std::optional<ProtectionEnvelopeV2> v2_;
  ProtectionPolygon polygon_;
  std::int64_t received_ns_{0};
  std::uint64_t received_epoch_{0};
};

bool protection_swept_collision(const std::vector<std::array<double, 2>>& points,
                                const std::vector<double>& command,
                                const ProtectionProfile& profile);
}  // namespace astribot::navigation
