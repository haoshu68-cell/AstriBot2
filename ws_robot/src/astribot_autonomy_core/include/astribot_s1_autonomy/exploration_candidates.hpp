#ifndef ASTRIBOT_S1_AUTONOMY__EXPLORATION_CANDIDATES_HPP_
#define ASTRIBOT_S1_AUTONOMY__EXPLORATION_CANDIDATES_HPP_
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
#include "astribot_s1_autonomy/path_validator.hpp"
namespace astribot_s1_autonomy {
struct PathEffort {double length{0.0}; double turning{0.0};};
inline PathEffort pathEffort(const std::vector<PlanarPoint> & path) {
  PathEffort result; double heading=0.0; bool has_heading=false;
  for (const auto & p : path) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      return {std::numeric_limits<double>::infinity(), 0.0};
    }
  }
  for (std::size_t i=1; i<path.size(); ++i) {
    const double dx=path[i].x-path[i-1].x, dy=path[i].y-path[i-1].y;
    const double distance=std::hypot(dx,dy);
    if (distance <= 1e-6) {continue;}
    const double next=std::atan2(dy,dx);
    result.length+=distance;
    if (has_heading) {result.turning+=std::abs(std::atan2(std::sin(next-heading),std::cos(next-heading)));}
    heading=next; has_heading=true;
  }
  return result;
}
// Failed observations are not successful visits. Bounded, expiring, version-aware memory.
class CandidateFailureMemory {
public:
  struct Record {double x,y,expires; uint64_t revision; bool map_dependent; std::string reason;};
  void record(double x,double y,double now,double ttl,uint64_t revision,bool map_dependent,
    const std::string & reason)
  {
    records_.erase(std::remove_if(records_.begin(),records_.end(),
      [now](const Record & r){return now>=r.expires;}),records_.end());
    if(records_.size()>=100) {records_.erase(records_.begin());}
    records_.push_back({x,y,now+ttl,revision,map_dependent,reason});
  }
  bool blocked(double x,double y,double now,uint64_t revision,double radius) const {
    return std::any_of(records_.begin(),records_.end(),[&](const Record & r){
      return now<r.expires && (!r.map_dependent || revision==r.revision) &&
        std::hypot(x-r.x,y-r.y)<=radius;
    });
  }
private:
  std::vector<Record> records_;
};
}  // namespace astribot_s1_autonomy
#endif
