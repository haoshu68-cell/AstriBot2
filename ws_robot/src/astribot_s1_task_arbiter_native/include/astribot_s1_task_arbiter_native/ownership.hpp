#pragma once
#include <optional>
#include <stdexcept>
#include <string>

namespace astribot_s1_task_arbiter_native {
// Navigation-only admission state. IDs are frontend action UUIDs, not a shared
// robot resource bus. A cancel acknowledgement never calls release().
class Ownership {
 public:
  struct Slot { std::string id; int priority; };
  bool reserve(int priority) {
    if (reserved_ || pending_ || (active_ && priority < active_->priority)) return false;
    reserved_ = true;
    return true;
  }
  void accept(const std::string &id, int priority) {
    if (!reserved_ || pending_) throw std::logic_error("goal accepted without reservation");
    pending_ = Slot{id, priority};
    reserved_ = false;
  }
  bool acquire(const std::string &id) {
    if (active_ || !pending_ || pending_->id != id) return false;
    active_ = pending_;
    pending_.reset();
    return true;
  }
  void release(const std::string &id) {
    if (active_ && active_->id == id) active_.reset();
    if (pending_ && pending_->id == id) pending_.reset();
  }
  const std::optional<Slot> &active() const { return active_; }
 private:
  bool reserved_{false};
  std::optional<Slot> active_, pending_;
};
}  // namespace astribot_s1_task_arbiter_native
