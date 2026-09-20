#ifndef ASTRIBOT_S1_PATH_TRACKING__ARRIVAL_PROGRESS_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__ARRIVAL_PROGRESS_HPP_

#include <cmath>
#include <map>
#include <memory>
#include <mutex>

namespace astribot_s1_path_tracking {
// Shared only by plugins in the same controller server. Goal/reset and stale
// reports cannot extend the pose progress deadline for a subsequent attempt.
class ArrivalProgress {
public:
  static std::shared_ptr<ArrivalProgress> forNode(const void * node) {
    static std::mutex mutex;
    static std::map<const void *, std::weak_ptr<ArrivalProgress>> states;
    std::lock_guard<std::mutex> lock(mutex);
    for (auto it=states.begin();it!=states.end();) {
      if (it->second.expired()) {it=states.erase(it);} else {++it;}
    }
    auto state=states[node].lock();
    if (!state) {state=std::make_shared<ArrivalProgress>();states[node]=state;}
    return state;
  }
  void clear() {std::lock_guard<std::mutex> lock(mutex_);reported_=-1.;}
  void report(double now) {std::lock_guard<std::mutex> lock(mutex_);reported_=now;}
  bool settling(double now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const double age=now-reported_;
    return reported_>=0. && std::isfinite(age) && age>=0. && age<=0.2;
  }
private:
  mutable std::mutex mutex_;
  double reported_{-1.};
};
}
#endif
