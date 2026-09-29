#pragma once

#include "astribot_s1_navigation_policy_native/navigation_math.hpp"
#include "astribot_s1_navigation_policy_native/policy_contracts.hpp"

#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace astribot::navigation::policy {

// Acquisition time, clock identity and calibration epoch remain authoritative.
// A health query never mutates the registry or adopts a different clock.
class SensorHealthRegistry {
 public:
  explicit SensorHealthRegistry(double timeout_s,
                                std::set<std::string> required = {"scan"});
  bool record(const std::string& sensor, const Stamp& capture, const Stamp& now,
              const std::string& frame, const std::vector<BearingCone>& coverage,
              bool depth, Integer calibration_epoch);
  std::vector<SensorHealth> health(const Stamp& now) const;
  bool required_valid(const Stamp& now) const;
  bool allows_motion(const Stamp& now, double vx, double vy, double wz) const;
  bool allows(const Stamp& now, const std::vector<double>& directions) const;
  const std::map<std::string, SensorHealth>& records() const { return records_; }

 private:
  std::int64_t timeout_ns_;
  std::set<std::string> required_;
  std::map<std::string, SensorHealth> records_;
  std::optional<std::pair<std::string, std::int64_t>> clock_;
};

class CameraCalibrationRegistry {
 public:
  void register_calibration(const CameraCalibration& calibration);
  const std::map<std::string, CameraCalibration>& records() const { return records_; }
  // Return a snapshot by value, so later registrations cannot invalidate a caller.
  CameraCalibration calibration(const std::string& camera_id,
                                const Integer& epoch) const;

 private:
  std::map<std::string, CameraCalibration> records_;
};

// Typed adapters only: numerical rules stay in the existing navigation_math core.
std::vector<BearingCone> scan_coverage(const std::vector<double>& ranges,
                                     double range_min, double range_max,
                                     double angle_min, double angle_increment,
                                     double body_yaw = 0.);
using ::astribot::navigation::movement_directions;
bool coverage_allows_motion(const std::vector<BearingCone>& cones,
                            double vx, double vy, double wz);

}  // namespace astribot::navigation::policy
