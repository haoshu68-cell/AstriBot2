#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>

namespace astribot_s1_path_tracking {
// In-process lifecycle handoff. Nav2 resets the progress checker once when a
// FollowPath execution starts, AFTER setPlan; path preemption does not reset it.
class ControllerExecution {
public:
  static std::shared_ptr<ControllerExecution> forNode(const void * node) {
    static std::mutex mutex;
    static std::map<const void *, std::weak_ptr<ControllerExecution>> states;
    std::lock_guard<std::mutex> lock(mutex);
    for (auto it=states.begin();it!=states.end();) {
      if(it->second.expired()) {it=states.erase(it);} else {++it;}
    }
    auto state=states[node].lock();
    if(!state) {state=std::make_shared<ControllerExecution>();states[node]=state;}
    return state;
  }
  void begin() {generation_.fetch_add(1);}
  uint64_t generation() const {return generation_.load();}
private:
  std::atomic<uint64_t> generation_{0};
};
}
