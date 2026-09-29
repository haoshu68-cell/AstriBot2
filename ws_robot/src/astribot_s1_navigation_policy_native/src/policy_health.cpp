#include "astribot_s1_navigation_policy_native/policy_health.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace astribot::navigation::policy {
namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;

std::int64_t timeout_nanoseconds(double seconds) {
  const double ns = seconds * 1e9;
  if (!std::isfinite(ns) || ns < -0x1p63 || ns >= 0x1p63) {
    throw std::invalid_argument("timeout_s outside native nanosecond range");
  }
  return static_cast<std::int64_t>(ns);  // Python int() truncates towards zero.
}

Stamp expiry(const Stamp& stamp, std::int64_t timeout_ns) {
  if ((timeout_ns > 0 && stamp.ns > std::numeric_limits<std::int64_t>::max() - timeout_ns) ||
      (timeout_ns < 0 && stamp.ns < std::numeric_limits<std::int64_t>::min() - timeout_ns)) {
    throw std::overflow_error("health.valid_until outside native nanosecond range");
  }
  return Stamp(stamp.ns + timeout_ns, stamp.clock, stamp.epoch);
}

std::vector<BearingCone> depth_coverage(const std::vector<SensorHealth>& health) {
  std::vector<BearingCone> cones;
  for (const auto& item : health) {
    if (item.health == Health::VALID && item.depth_available) {
      for (const auto& cone : item.coverage) cones.push_back(cone);
    }
  }
  return cones;
}
}  // namespace

SensorHealthRegistry::SensorHealthRegistry(double timeout_s, std::set<std::string> required)
    : timeout_ns_(timeout_nanoseconds(timeout_s)), required_(std::move(required)) {}

bool SensorHealthRegistry::record(const std::string& sensor, const Stamp& capture,
                                  const Stamp& now, const std::string& frame,
                                  const std::vector<BearingCone>& coverage,
                                  bool depth, Integer calibration_epoch) {
  (void)now;
  const auto clock = std::make_pair(capture.clock, capture.epoch);
  if (!clock_ || *clock_ != clock) {
    records_.clear();
    clock_ = clock;
  }
  const auto old = records_.find(sensor);
  if (old != records_.end() &&
      (capture.ns <= old->second.stamp.ns ||
       calibration_epoch < old->second.calibration_epoch)) return false;

  // Construct before replacing: invalid metadata leaves any old sample intact.
  // Clock adoption above intentionally precedes validation, matching Python.
  SensorHealth item(sensor, coverage.empty() ? Health::DEGRADED : Health::VALID,
                    capture, expiry(capture, timeout_ns_), frame, coverage, depth,
                    calibration_epoch,
                    coverage.empty() ? "NO_DECLARED_COVERAGE" : "VALID");
  if (old != records_.end()) records_.erase(old);
  records_.emplace(sensor, std::move(item));
  return true;
}

std::vector<SensorHealth> SensorHealthRegistry::health(const Stamp& now) const {
  auto sensors = required_;
  for (const auto& record : records_) sensors.insert(record.first);
  std::vector<SensorHealth> out;
  out.reserve(sensors.size());
  for (const auto& sensor : sensors) {
    const auto found = records_.find(sensor);
    if (found == records_.end()) {
      out.emplace_back(sensor, Health::UNAVAILABLE, now, expiry(now, timeout_ns_),
                       "unknown", std::vector<BearingCone>{}, false, 0, "NO_CURRENT_DATA");
      continue;
    }
    const auto& item = found->second;
    out.push_back(item);
  }
  return out;
}

bool SensorHealthRegistry::required_valid(const Stamp& now) const {
  for (const auto& item : health(now)) {
    if (required_.count(item.sensor_id) && item.health != Health::VALID) return false;
  }
  return true;
}

bool SensorHealthRegistry::allows_motion(const Stamp& now, double vx, double vy,
                                        double wz) const {
  if (!required_valid(now)) return false;
  return coverage_allows_motion(depth_coverage(health(now)), vx, vy, wz);
}

bool SensorHealthRegistry::allows(const Stamp& now,
                                 const std::vector<double>& directions) const {
  if (!required_valid(now)) return false;
  const auto cones = depth_coverage(health(now));
  return std::all_of(directions.begin(), directions.end(), [&cones](double angle) {
    return std::any_of(cones.begin(), cones.end(), [angle](const BearingCone& cone) {
      return std::abs(std::remainder(angle - std::atan2(cone.direction.y, cone.direction.x),
                                     2. * kPi)) <= cone.half_angle_rad + 1e-6;
    });
  });
}

void CameraCalibrationRegistry::register_calibration(const CameraCalibration& calibration) {
  const auto found = records_.find(calibration.camera_id);
  if (found != records_.end() && calibration.calibration_epoch <= found->second.calibration_epoch) {
    if (!(found->second == calibration)) {
      throw std::invalid_argument("calibration epoch must increase on change");
    }
    return;
  }
  if (found != records_.end()) records_.erase(found);
  records_.emplace(calibration.camera_id, calibration);
}

CameraCalibration CameraCalibrationRegistry::calibration(const std::string& camera_id,
                                                         const Integer& epoch) const {
  const auto found = records_.find(camera_id);
  if (found == records_.end()) throw std::out_of_range(camera_id);
  if (found->second.calibration_epoch != epoch) {
    throw std::invalid_argument("calibration version mismatch");
  }
  return found->second;
}

std::vector<BearingCone> scan_coverage(const std::vector<double>& ranges,
                                     double range_min, double range_max,
                                     double angle_min, double angle_increment,
                                     double body_yaw) {
  const auto flat = ::astribot::navigation::scan_coverage(
      ranges, range_min, range_max, angle_min, angle_increment, body_yaw);
  std::vector<BearingCone> cones;
  cones.reserve(flat.size() / 4);
  for (std::size_t i = 0; i < flat.size(); i += 4) {
    cones.emplace_back(Vec3(flat[i], flat[i + 1], flat[i + 2]), flat[i + 3]);
  }
  return cones;
}

bool coverage_allows_motion(const std::vector<BearingCone>& cones,
                            double vx, double vy, double wz) {
  std::vector<double> flat;
  flat.reserve(cones.size() * 3);
  for (const auto& cone : cones) {
    flat.insert(flat.end(), {cone.direction.x, cone.direction.y, cone.half_angle_rad});
  }
  return ::astribot::navigation::coverage_allows_motion(flat, vx, vy, wz);
}
}  // namespace astribot::navigation::policy
