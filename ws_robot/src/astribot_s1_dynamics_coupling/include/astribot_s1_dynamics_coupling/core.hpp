#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace astribot_s1_dynamics_coupling {
using JointMap = std::unordered_map<std::string, double>;

// External JointState boundary: all fourteen monitored joints must be measured.
inline bool complete_joint_sample(const std::vector<std::string> & names,
    const std::vector<double> & positions,const std::vector<double> & velocities,
    const std::vector<std::string> & required) {
  if(names.size()!=positions.size() || names.size()!=velocities.size())return false;
  std::unordered_set<std::string> seen;
  for(std::size_t i=0;i<names.size();++i) {
    if(!seen.insert(names[i]).second || !std::isfinite(positions[i]) ||
      !std::isfinite(velocities[i]))return false;
  }
  return std::all_of(required.begin(),required.end(),[&seen](const auto &name){return seen.count(name)!=0;});
}

inline JointMap zip_map(const std::vector<std::string> & names,
                       const std::vector<double> & values) {
  JointMap result;
  for (std::size_t i = 0; i < std::min(names.size(), values.size()); ++i) {
    result[names[i]] = values[i];  // Python dict(zip(...)): last duplicate wins.
  }
  return result;
}

inline void validate_reach_thresholds(double folded, double full) {
  if (folded < 0.0 || full <= folded) {
    throw std::invalid_argument("reach_folded_m must be nonnegative and smaller than reach_full_m");
  }
}

inline double horizontal_reach(double x, double y) {
  const double xx = x * x, yy = y * y;
  // Python float ** 2 raises for a finite operand whose square overflows.
  if ((std::isfinite(x) && !std::isfinite(xx)) ||
      (std::isfinite(y) && !std::isfinite(yy))) {
    throw std::overflow_error("horizontal reach square overflow");
  }
  return std::sqrt(xx + yy);
}

inline double reach_activity(double reach, double folded, double full) {
  const double span = full - folded;
  if (span <= 0.0) throw std::invalid_argument("invalid reach interval");
  return std::max(0.0, std::min(1.0, (reach - folded) / span));
}

inline double joint_deviation_activity(const JointMap & positions,
    const std::vector<std::string> & names, const std::vector<double> & reference, double full) {
  if (full <= 1e-6) return 0.0;
  double activity = 0.0;
  for (std::size_t i = 0; i < std::min(names.size(), reference.size()); ++i) {
    const auto found = positions.find(names[i]);
    if (found != positions.end()) {
      // Deliberately raw subtraction: the legacy metric does not wrap angles.
      activity = std::max(activity, std::min(1.0, std::abs(found->second - reference[i]) / full));
    }
  }
  return activity;
}

inline double velocity_activity(const JointMap & velocities,
    const std::vector<std::string> & names, double full) {
  if (full <= 1e-6) return 0.0;
  double activity = 0.0;
  for (const auto & name : names) {
    const auto found = velocities.find(name);
    if (found != velocities.end()) {
      activity = std::max(activity, std::min(1.0, std::abs(found->second) / full));
    }
  }
  return activity;
}

inline double scale_from_activity(double activity, double minimum) {
  return std::max(minimum, std::min(1.0, 1.0 - activity * (1.0 - minimum)));
}
}  // namespace astribot_s1_dynamics_coupling
