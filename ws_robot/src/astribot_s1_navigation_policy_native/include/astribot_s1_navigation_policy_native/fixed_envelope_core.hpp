#pragma once
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <nlohmann/json.hpp>
#include <astribot_navigation_msgs/msg/arm_hold_status.hpp>
#include <astribot_navigation_msgs/msg/envelope_apply_status.hpp>
#include <astribot_navigation_msgs/msg/navigation_envelope_v2.hpp>
#include <astribot_navigation_msgs/msg/robot_geometry_state.hpp>
#include <astribot_navigation_msgs/srv/set_fixed_envelope.hpp>

namespace astribot::navigation {
// No executor, Python object, or receipt-time renewal inside the transaction.
class FixedEnvelopeCore {
public:
  using State = astribot_navigation_msgs::msg::RobotGeometryState;
  using Hold = astribot_navigation_msgs::msg::ArmHoldStatus;
  using Ack = astribot_navigation_msgs::msg::EnvelopeApplyStatus;
  using Envelope = astribot_navigation_msgs::msg::NavigationEnvelopeV2;
  using Request = astribot_navigation_msgs::srv::SetFixedEnvelope::Request;
  FixedEnvelopeCore(nlohmann::json baseline, std::string session, std::uint64_t epoch);
  void state(const State& message);
  void hold(const Hold& message);
  void revoke(const std::string& reason);
  const Envelope& propose(const Request& request, std::int64_t now, bool stopped);
  void acknowledge(const Ack& message, std::int64_t now);
  const std::optional<Envelope>& tick(std::int64_t now);
  const std::optional<Envelope>& output() const { return output_; }
  const std::map<std::string,std::int64_t>& acknowledgements() const { return acks_; }
  const std::string& fault() const { return fault_; }
  const std::string& session() const { return session_; }
  std::uint64_t epoch() const { return epoch_; }
  const nlohmann::json& baseline() const { return baseline_; }
private:
  void validate_state(const std::shared_ptr<State>& state, std::int64_t now) const;
  std::int64_t hold_until(const std::string& id, const std::string& revision, std::int64_t now) const;
  nlohmann::json baseline_;
  std::string session_;
  std::uint64_t epoch_;
  std::shared_ptr<State> current_, reference_;
  using Key = std::pair<std::string,std::uint64_t>;
  std::map<Key,std::shared_ptr<State>> history_;
  std::deque<Key> history_order_;
  std::optional<Hold> hold_;
  std::map<std::string,std::int64_t> acks_;
  // Positive evidence must be acquired strictly after this consumer revoked it.
  // Equal stamps are ambiguous without a consumer sequence, so remain denied.
  std::map<std::string,std::int64_t> ack_revocations_;
  std::optional<Envelope> output_;
  std::string fault_{"NO_FIXED_ENVELOPE"};
  std::int64_t last_now_{0};
};
std::int64_t fixed_stamp_ns(const builtin_interfaces::msg::Time& stamp);
builtin_interfaces::msg::Time fixed_stamp(std::int64_t nanoseconds);
}  // namespace astribot::navigation
