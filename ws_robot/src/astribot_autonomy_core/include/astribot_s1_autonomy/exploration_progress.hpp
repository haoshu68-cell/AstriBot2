// Pure snapshot and completion policies shared by ROS exploration adapters.
#ifndef ASTRIBOT_S1_AUTONOMY__EXPLORATION_PROGRESS_HPP_
#define ASTRIBOT_S1_AUTONOMY__EXPLORATION_PROGRESS_HPP_
#include <cmath>
#include <cstdint>
#include "astribot_s1_autonomy/frontier_search.hpp"
namespace astribot_s1_autonomy {
inline bool sameMapGeometry(const GridMap & a, const GridMap & b) {
  return a.width == b.width && a.height == b.height && a.resolution == b.resolution &&
    a.origin_x == b.origin_x && a.origin_y == b.origin_y;
}
inline bool sameMapContent(const GridMap & a, const GridMap & b) {
  return sameMapGeometry(a, b) && a.data == b.data;
}
inline bool usableSearchSnapshot(const GridMap & searched, const GridMap & current,
  double age_sec, double displacement_m, double max_age_sec, double max_displacement_m)
{
  return sameMapGeometry(searched, current) && std::isfinite(age_sec) &&
    age_sec >= 0.0 && age_sec <= max_age_sec && std::isfinite(displacement_m) &&
    displacement_m >= 0.0 && displacement_m <= max_displacement_m;
}
// This is a semantic frontier check, not a collision or path safety certificate.
inline bool isCurrentFrontier(const GridMap & map, double x, double y,
  int free_threshold, bool eight_connectivity)
{
  unsigned int mx, my;
  if (!map.consistent() || !std::isfinite(x) || !std::isfinite(y) ||
    !map.worldToMap(x, y, mx, my)) {return false;}
  const auto value = map.data[map.index(mx, my)];
  if (value < 0 || value > free_threshold) {return false;}
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if ((dx == 0 && dy == 0) || (!eight_connectivity && dx != 0 && dy != 0)) {continue;}
      const auto nx = static_cast<int64_t>(mx) + dx;
      const auto ny = static_cast<int64_t>(my) + dy;
      if (nx >= 0 && ny >= 0 && nx < map.width && ny < map.height &&
        map.data[map.index(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny))] < 0)
      {return true;}
    }
  }
  return false;
}
class CompletionTracker {
public:
  void reset() {observations_ = 0;}
  bool observe(uint64_t revision, double steady_seconds, bool eligible,
    unsigned int required_observations, double stable_seconds)
  {
    if (!eligible || !std::isfinite(steady_seconds)) {reset(); return false;}
    if (observations_ == 0 || revision != revision_ || steady_seconds < first_seen_) {
      revision_ = revision; first_seen_ = steady_seconds; observations_ = 1;
    } else if (observations_ < required_observations) {++observations_;}
    return observations_ >= required_observations && steady_seconds - first_seen_ >= stable_seconds;
  }
private:
  uint64_t revision_{0};
  unsigned int observations_{0};
  double first_seen_{0.0};
};
}  // namespace astribot_s1_autonomy
#endif
