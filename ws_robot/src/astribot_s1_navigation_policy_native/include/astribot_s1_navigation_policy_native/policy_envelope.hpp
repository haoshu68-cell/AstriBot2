#pragma once

#include "astribot_s1_navigation_policy_native/policy_contracts.hpp"
#include "astribot_s1_robot_geometry/geometry_kernels.hpp"
#include <astribot_navigation_msgs/msg/navigation_envelope_v2.hpp>
#include <astribot_navigation_msgs/msg/robot_envelope.hpp>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace astribot::navigation::policy {

// Policy consumption state. A heartbeat confirmation never applies or refreshes it.
// The baseline must already have been validated by load_policy_profile.
class PolicyEnvelopeProfile {
 public:
  using Envelope = astribot_navigation_msgs::msg::RobotEnvelope;
  using EnvelopeV2 = astribot_navigation_msgs::msg::NavigationEnvelopeV2;
  using Polygon = astribot_s1_robot_geometry::Polygon2;
  explicit PolicyEnvelopeProfile(nlohmann::json validated_baseline, bool fixed = false);
  const nlohmann::json baseline;
  double value(const std::string& key) const;
  const std::string& base_frame() const;
  const std::string& tracking_frame() const;
  const std::optional<Polygon>& polygon() const { return polygon_; }
  const std::optional<Envelope>& envelope() const { return envelope_; }
  bool accept(const Envelope& message, const Stamp& now);
  bool accept(const EnvelopeV2& message, const Stamp& now);
  bool ready(const Stamp& now) const;
  bool confirms_applied(const EnvelopeV2& message, const Stamp& now) const;
  double stopping_distance(double speed) const;

 private:
  void validate(const Envelope& message) const;
  void revoke();
  const bool fixed_;
  std::optional<Envelope> envelope_;
  std::optional<EnvelopeV2> v2_;
  std::optional<Stamp> received_;
  std::optional<Polygon> polygon_;
};
}  // namespace astribot::navigation::policy
