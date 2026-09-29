#pragma once
#include "astribot_s1_navigation_policy_native/navigation_math.hpp"
#include "astribot_s1_navigation_policy_native/policy_contracts.hpp"
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace astribot::navigation::policy {
// Raw xyzw quaternion arithmetic matches the observation adapter: no implicit
// normalization and no change to the source frame or capture-time convention.
double observer_yaw(const std::array<double,4>& quaternion);
std::array<double,3> observer_point(const std::array<double,3>& point,
                                  const std::array<double,7>& transform);
Version execution_version(const navigation::ExecutionContext& context);

class ObserverMap {
 public:
  // Invalid metadata leaves the previously accepted map unchanged. Source z,
  // header stamp and map_load_time are not part of the original map identity.
  bool accept(const std::string& frame,std::uint32_t width,std::uint32_t height,
              double resolution,const std::array<double,7>& origin,
              const std::vector<std::int8_t>& cells);
  bool has_map() const {return has_map_;}
  bool static_at(double x,double y) const;
  const std::vector<std::vector<bool>>& mask() const;
  std::array<double,5> projection_info() const;
  std::string context_key() const;
 private:
  bool has_map_=false;
  std::uint32_t width_=0,height_=0;
  std::array<double,9> metadata_{};
  std::vector<std::int8_t> cells_;
  mutable std::vector<std::vector<bool>> mask_;
};
}  // namespace astribot::navigation::policy
