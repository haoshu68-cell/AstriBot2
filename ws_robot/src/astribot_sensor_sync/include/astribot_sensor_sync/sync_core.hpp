#pragma once
#include <algorithm>
#include <cstdint>
#include <deque>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
namespace astribot::sync {
struct Trigger {std::string group, epoch, clock_epoch; uint64_t sequence{}; int64_t stamp_ns{}; bool simulated{};};
struct Frame {
  std::string source, source_epoch, group, trigger_epoch, clock_epoch;
  uint64_t sequence{}, trigger_sequence{}; int64_t capture_ns{}, uncertainty_ns{};
  bool hardware_associated{}, simulated{}, clock_locked{};
};
struct Config {
  std::vector<std::string> sources;
  int64_t max_skew_ns{2000000}, max_uncertainty_ns{1000000}, max_age_ns{250000000};
  size_t history{256}; bool allow_simulated{false};
  std::string reference_clock_epoch{"UNCONFIGURED"};
  std::set<std::string> clock_only_sources{"lidar_front", "lidar_back", "imu"};
};
struct Result {bool valid{}, hardware{}; std::string reason; int64_t skew_ns{}; uint64_t dropped{};};
// Pure bounded metadata validator. Never changes acquisition stamps or grants motion.
class Monitor {
 public:
  explicit Monitor(Config c): config_(std::move(c)) {
    const std::set<std::string> unique(config_.sources.begin(),config_.sources.end());
    if(config_.reference_clock_epoch.empty() || config_.sources.empty() || unique.size()!=config_.sources.size() || unique.count("") ||
       config_.history==0 || config_.history>4096 || config_.max_skew_ns<0 ||
       config_.max_uncertainty_ns<0 || config_.max_age_ns<=0)
      throw std::invalid_argument("invalid synchronization budget or source list");
  }
  bool trigger(const Trigger& t) {
    if(t.group.empty()||t.epoch.empty()||t.clock_epoch.empty()||!t.sequence||t.stamp_ns<=0) return false;
    for(const auto& old: triggers_) if(key(old,t)) {
      if(old.stamp_ns!=t.stamp_ns || old.simulated!=t.simulated || old.clock_epoch!=t.clock_epoch) {
        conflict_=true;return false;
      }
      return true;
    }
    triggers_.push_back(t);while(triggers_.size()>config_.history)triggers_.pop_front();return true;
  }
  Result frame(const Frame& f,int64_t now) {
    if(last_now_>0 && now<last_now_)rollback_=true;
    last_now_=now;
    auto fail=[](const char* r){return Result{false,false,r,0,0};};
    if(rollback_)return fail("CLOCK_ROLLBACK");
    if(config_.reference_clock_epoch=="UNCONFIGURED")return fail("REFERENCE_CLOCK_UNCONFIGURED");
    if(conflict_)return fail("TRIGGER_CONFLICT");
    if(std::find(config_.sources.begin(),config_.sources.end(),f.source)==config_.sources.end())return fail("UNKNOWN_SOURCE");
    if(f.source_epoch.empty() || f.clock_epoch.empty() || !f.sequence || f.capture_ns<=0 || now<=0)return fail("INVALID_METADATA");
    if(!f.clock_locked)return fail("CLOCK_UNLOCKED");
    if(f.clock_epoch!=config_.reference_clock_epoch)return fail("CLOCK_EPOCH_MISMATCH");
    if(f.uncertainty_ns<0 || f.uncertainty_ns>config_.max_uncertainty_ns)return fail("CLOCK_UNCERTAIN");
    if(f.capture_ns>now)return fail("FUTURE_SAMPLE");
    if(now-f.capture_ns>config_.max_age_ns)return fail("STALE_SAMPLE");
    if(f.simulated&&!config_.allow_simulated)return fail("SIMULATED_EVIDENCE");
    auto& previous=frames_[f.source];
    if(!previous.source_epoch.empty() && (previous.source_epoch!=f.source_epoch || previous.clock_epoch!=f.clock_epoch))
      return fail("SOURCE_EPOCH_CHANGED"); // explicit monitor restart, no cross-epoch reuse
    if(f.sequence<=previous.sequence || f.capture_ns<=previous.capture_ns)return fail("NON_MONOTONIC_FRAME");
    bool hardware=false;int64_t skew=0;
    const bool clock_only=config_.clock_only_sources.count(f.source)>0;
    if(!clock_only) {
      if(!f.hardware_associated || !f.trigger_sequence)return fail("NO_HARDWARE_ASSOCIATION");
      if(previous.trigger_sequence && (previous.trigger_epoch!=f.trigger_epoch || previous.group!=f.group))return fail("TRIGGER_EPOCH_CHANGED");
      if(previous.trigger_sequence && f.trigger_sequence<=previous.trigger_sequence)return fail("TRIGGER_REUSED");
      const auto it=std::find_if(triggers_.begin(),triggers_.end(),[&](const auto& t){return t.group==f.group&&t.epoch==f.trigger_epoch&&t.sequence==f.trigger_sequence;});
      if(it==triggers_.end())return fail("TRIGGER_UNKNOWN");
      if(it->clock_epoch!=f.clock_epoch)return fail("CLOCK_EPOCH_MISMATCH");
      if(it->simulated!=f.simulated)return fail("EVIDENCE_MODE_MISMATCH");
      // Positive bounded int64 values make subtraction safe.
      skew=f.capture_ns-it->stamp_ns;
      if(skew>config_.max_skew_ns || skew< -config_.max_skew_ns)return fail("EXPOSURE_SKEW");
      hardware=!f.simulated;
    }
    const uint64_t dropped=previous.sequence?f.sequence-previous.sequence-1:0;
    previous=f;
    return {true,hardware,clock_only?"CLOCK_ONLY":(f.simulated?"SIMULATED_TRIGGER_MATCH":"HARDWARE_TRIGGER_MATCH"),skew,dropped};
  }
  void reset(){triggers_.clear();frames_.clear();last_now_=0;rollback_=false;conflict_=false;}
  size_t trigger_count()const{return triggers_.size();}
 private:
  static bool key(const Trigger&a,const Trigger&b){return a.group==b.group&&a.epoch==b.epoch&&a.sequence==b.sequence;}
  Config config_;std::deque<Trigger>triggers_;std::map<std::string,Frame>frames_;
  int64_t last_now_{};bool rollback_{},conflict_{};
};
// Vendor-specific implementation must own one physical timing device. ROS timers
// may simulate readback but must never implement this as a physical pulse source.
struct TriggerProgram {uint32_t period_us{}, pulse_width_us{}, phase_us{};std::string group,epoch;};
class HardwareTriggerPort {
 public:
  virtual ~HardwareTriggerPort()=default;
  virtual bool configure(const TriggerProgram&)=0; // validate voltage/polarity/capability in backend
  virtual bool arm()=0;
  virtual void disarm()noexcept=0;
  virtual bool read_edge(Trigger&)=0; // actual timestamped device readback, not scheduled time
};
}
