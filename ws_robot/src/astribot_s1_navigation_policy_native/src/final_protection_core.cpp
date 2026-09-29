#include "astribot_s1_navigation_policy_native/final_protection_core.hpp"
#include "astribot_s1_navigation_policy_native/navigation_math.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace astribot::navigation {
namespace {
namespace geo = astribot_s1_robot_geometry;
using Field = std::pair<const char*, double ProtectionEnvelope::*>;
const std::array<Field, 8> fields{{
  {"half_length_m", &ProtectionEnvelope::half_length_m},
  {"half_width_m", &ProtectionEnvelope::half_width_m},
  {"height_m", &ProtectionEnvelope::height_m},
  {"payload_mass_kg", &ProtectionEnvelope::payload_mass_kg},
  {"max_speed_m_s", &ProtectionEnvelope::max_speed_m_s},
  {"max_angular_speed_rad_s", &ProtectionEnvelope::max_angular_speed_rad_s},
  {"max_acceleration_m_s2", &ProtectionEnvelope::max_acceleration_m_s2},
  {"brake_deceleration_m_s2", &ProtectionEnvelope::brake_deceleration_m_s2}}};

std::int64_t ns(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
void require(bool condition, const char* reason) {
  if (!condition) throw std::invalid_argument(reason);
}
bool same_fields(const ProtectionEnvelope& a, const ProtectionEnvelope& b) {
  return std::all_of(fields.begin(), fields.end(), [&](const Field& f) {
    return a.*(f.second) == b.*(f.second);
  });
}
ProtectionPolygon vertices(const geometry_msgs::msg::Polygon& message) {
  ProtectionPolygon out;
  for (const auto& p : message.points) out.push_back({p.x, p.y});
  return geo::validatePolygon(out);
}
}  // namespace

ProtectionProfile::ProtectionProfile(nlohmann::json baseline, bool fixed)
    : baseline_(std::move(baseline)), base_frame_(baseline_.at("base_frame")), fixed_(fixed) {}

double ProtectionProfile::value(const char* key) const {
  if (envelope_) {
    for (const auto& f : fields) if (std::string(key) == f.first) return (*envelope_).*(f.second);
  }
  if (std::string(key) == "linear_stop_delay_s") return baseline_.value(key, 0.0);
  return baseline_.at(key).get<double>();
}

void ProtectionProfile::validate(const ProtectionEnvelope& message) const {
  require(!message.posture_id.empty() && message.frame_id == base_frame_, "invalid posture/frame");
  require(message.lease_s > 0.0 && message.lease_s <= .5, "invalid envelope lease");
  for (std::size_t i = 0; i < fields.size(); ++i) {
    const auto& f = fields[i];
    const double value = message.*(f.second), baseline = baseline_.at(f.first);
    require(std::isfinite(value) && (i == 3 ? value >= 0.0 : value > 0.0), f.first);
    if (i < 3) require(value >= baseline, "envelope cannot undercut baseline");
    if (i > 3) require(value <= baseline, "unvalidated limit increase");
  }
}

bool ProtectionProfile::accept(const ProtectionEnvelope& message, std::int64_t now,
                               std::uint64_t epoch) {
  require(!fixed_, "legacy envelope cannot refresh V2 profile");
  validate(message);
  if (envelope_ && message.epoch < envelope_->epoch) return false;
  if (envelope_ && message.epoch == envelope_->epoch) {
    require(same_fields(message, *envelope_) && message.posture_id == envelope_->posture_id &&
            message.frame_id == envelope_->frame_id, "geometry and limits require a new envelope epoch");
  }
  if(envelope_ && message.epoch==envelope_->epoch && ns(message.stamp)<ns(envelope_->stamp))return false;
  envelope_ = message; received_ns_ = now; received_epoch_ = epoch;
  return true;
}

bool ProtectionProfile::accept(const ProtectionEnvelopeV2& message, std::int64_t now,
                               std::uint64_t epoch) {
  require(fixed_, "V2 envelope cannot refresh legacy profile");
  if(v2_ && message.coordinator_session_id==v2_->coordinator_session_id &&
      message.clock_epoch==v2_->clock_epoch && message.epoch==v2_->epoch &&
      ns(message.header.stamp)<ns(v2_->header.stamp))return false;
  try {
    validate(message.limits);
    require(message.header.frame_id == base_frame_ && message.mode == message.FIXED_POSTURE,
            "invalid V2 frame/mode");
    require(!message.coordinator_session_id.empty() && !message.hold_id.empty(), "missing V2 identity");
    require(std::isfinite(message.clearance_m) && message.clearance_m >=
              baseline_.at("clearance_margin_m").get<double>() + baseline_.at("payload_extra_margin_m").get<double>(),
            "V2 clearance undercut");
    const auto reserved = vertices(message.reserved_footprint), installed = vertices(message.installed_footprint);
    require(geo::containsPolygon(installed, geo::inflatePolygon(reserved, message.clearance_m), 2e-6),
            "V2 installed footprint undercut");
    require(geo::geometryHash(installed, message.header.frame_id, message.clearance_m) == message.installed_geometry_hash,
            "V2 hash mismatch");
    for (const auto& point : reserved) {
      require(std::abs(point[0]) <= message.limits.half_length_m + 1e-6 &&
              std::abs(point[1]) <= message.limits.half_width_m + 1e-6, "V2 broad-phase undercut");
    }
    if (v2_ && message.coordinator_session_id == v2_->coordinator_session_id) {
      if (message.epoch < v2_->epoch) return false;
      if (message.epoch == v2_->epoch) {
        require(message.installed_geometry_hash == v2_->installed_geometry_hash &&
                same_fields(message.limits, *envelope_), "V2 mutated epoch");
      }
    }
    envelope_ = message.limits; v2_ = message; polygon_ = reserved;
    received_ns_ = now; received_epoch_ = epoch;
    return true;
  } catch (const std::exception&) {
    // Revocation removes authority, not the last collision geometry. Match the
    // Python profile's retained footprint rather than silently substituting a
    // rectangle while a malformed V2 update is being rejected.
    envelope_.reset(); v2_.reset();
    throw;
  }
}

bool ProtectionProfile::ready(std::int64_t now, std::uint64_t epoch) const {
  if (!envelope_ || !envelope_->transport_ready || epoch != received_epoch_) return false;
  (void)now;
  return !fixed_ || (v2_ && v2_->navigation_allowed);

}

bool protection_swept_collision(const std::vector<std::array<double, 2>>& points,
                                const std::vector<double>& command,
                                const ProtectionProfile& p) {
  if (command.size() != 3 || !std::all_of(command.begin(), command.end(),
      [](double value) { return std::isfinite(value); })) return true;
  const double length = p.value("half_length_m"), width = p.value("half_width_m");
  const double duration = stopping_horizon(command, p.value("reaction_time_s"),
      p.value("brake_deceleration_m_s2"), p.value("angular_brake_deceleration_rad_s2"),
      p.value("linear_stop_delay_s"));
  const double margin = p.value("clearance_margin_m") + p.value("payload_extra_margin_m");
  const double speed = std::hypot(command[0], command[1]), radius = std::hypot(length, width);
  const double reach = speed * duration + radius + margin + sampling_margin(command, length, width, .05);
  std::vector<std::array<double, 2>> nearby;
  for (const auto& point : points) if (std::hypot(point[0], point[1]) <= reach + 1e-9) nearby.push_back(point);
  if (nearby.empty()) return false;
  const int steps = std::max(1, static_cast<int>(std::ceil(duration / .05)));
  if (p.polygon().empty()) {
    std::vector<double> begin, end, bounds;
    for (int i = 0; i <= steps; ++i) {
      for (const auto& point : nearby) {
        begin.push_back(i ? std::min(duration, (i - 1) * .05) : 0.);
        end.push_back(std::min(duration, i * .05));
        bounds.push_back(point[0]); bounds.push_back(point[1]);
      }
    }
    const auto gaps = motion_clearance_rect(command, begin, end, bounds, bounds, length, width,
      p.value("clearance_margin_m"), p.value("payload_extra_margin_m"), {0., 0., 0.});
    return std::any_of(gaps.begin(), gaps.end(), [](double gap) { return gap <= 0.; });
  }
  // Preserve Python continuous_sweep's interval certificate and depth bound.
  // Exact filled-polygon distance is used near obstacles, not its rectangle.
  double polygon_radius = 0.;
  for (const auto& point : p.polygon()) polygon_radius = std::max(polygon_radius, std::hypot(point[0], point[1]));
  const double boundary_speed = speed + radius * std::abs(command[2]);
  struct Interval { double begin, end; std::array<double, 2> point; int depth; };
  std::vector<Interval> pending;
  for (int i = 0; i <= steps; ++i) for (const auto& point : nearby) {
    pending.push_back({i ? std::min(duration, (i - 1) * .05) : 0., std::min(duration, i * .05), point, 0});
  }
  while (!pending.empty()) {
    const auto item = pending.back(); pending.pop_back();
    const double mid = (item.begin + item.end) / 2., half = (item.end - item.begin) / 2.;
    const auto pose = body_pose(command, mid);
    const double distance = std::hypot(pose[0] - item.point[0], pose[1] - item.point[1]);
    const double circle_point = distance - polygon_radius;
    double point_gap = circle_point - margin - 1e-12;
    if (circle_point <= margin + 1e-12) {
      point_gap = geo::boxDistances(p.polygon(), {{pose[0], pose[1], pose[2],
          item.point[0], item.point[1], item.point[0], item.point[1]}}).front() - margin - 1e-12;
    }
    const double circle = distance - radius - margin - speed * half - 1e-12;
    const double bound = std::max(point_gap - boundary_speed * half, circle);
    if (bound > 0.) continue;
    if (point_gap <= 0. || item.depth == 10) return true;
    pending.push_back({item.begin, mid, item.point, item.depth + 1});
    pending.push_back({mid, item.end, item.point, item.depth + 1});
  }
  return false;
}
}  // namespace astribot::navigation
