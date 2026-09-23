#pragma once
#include "astribot_s1_payload_state/ledger.hpp"
namespace astribot::payload {
class Consumer {
public:
  explicit Consumer(Config config):config_(std::move(config)) {}
  bool receive(const State &value,int64_t ros_now,int64_t steady_now);
  const State *current(int64_t ros_now,int64_t steady_now);
  const std::string &reason() const {return reason_;}
  uint64_t generation() const {return generation_;}
private:
  void revoke(const std::string &reason);
  bool clock(int64_t ros_now,int64_t steady_now);
  Config config_;std::optional<State> value_;
  std::string reason_="ATTACHMENT_STATE_UNAVAILABLE";
  std::set<std::string> retired_;
  uint64_t generation_=0;
  int64_t last_ros_=-1,last_steady_=-1,deadline_=-1,published_deadline_=-1;
  std::string previous_epoch_,previous_revision_;
  uint64_t previous_ledger_revision_=0, previous_clock_epoch_=0;
  std::string previous_source_epoch_;
  int64_t capture_floor_=-1;
};
}
