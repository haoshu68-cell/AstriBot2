#pragma once

// Diagnostics only. This recorder never selects a frame or grants permission.
// ROS and monotonic nanoseconds are retained separately without subtracting
// different clock domains. Fixed storage bounds also apply to diagnostic text.
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <string>

namespace astribot::navigation {
struct ScanTimes { std::int64_t ros_ns=0, steady_ns=0; };
struct ScanTimingRecord {
  std::uint64_t sequence=0, tf_checks=0;
  std::int64_t clock_epoch=0, capture_ros_ns=0;
  std::string frame_id, reason;
  ScanTimes receive;
  std::optional<ScanTimes> first_tf_ready, selected, finished, last_tf_check, deferred_at;
  int tracking_tf=-1, map_tf=-1;  // -1 means not checked, not a failed lookup.
  bool processing_success=false;
};
struct ScanTimingSnapshot {
  std::int64_t clock_epoch=0;
  std::size_t retained=0;
  std::uint64_t evictions=0, missing_updates=0, resets=0;
  std::map<std::string,std::uint64_t> drop_counts, defer_counts;
  std::optional<ScanTimingRecord> received, waiting, selected, finished, successful, dropped, deferred;
};
class ScanTiming {
 public:
  static constexpr std::size_t capacity=32;
  std::uint64_t receive(std::int64_t capture, std::int64_t ros,
                        std::int64_t steady, const std::string& frame, std::int64_t epoch) {
    std::lock_guard<std::mutex> lock(mutex_);
    ScanTimingRecord record;
    record.sequence=++sequence_; record.clock_epoch=epoch;
    if(epoch<state_.clock_epoch) {
      // Still return a unique diagnostic ID; never alter the caller's scan.
      ++state_.drop_counts["obsolete_epoch"];
      return record.sequence;
    }
    record.capture_ros_ns=capture; record.receive={ros,steady};
    record.frame_id=bounded_text(frame);
    if (records_.size()==capacity) { records_.pop_front(); ++state_.evictions; }
    records_.push_back(record);state_.received=record;
    return record.sequence;
  }
  void tf_check(std::uint64_t sequence, int tracking, int map,
                std::int64_t ros, std::int64_t steady) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto* r=find(sequence)) {
      ++r->tf_checks;r->tracking_tf=tracking;r->map_tf=map;r->last_tf_check=ScanTimes{ros,steady};
      if (tracking==1 && map==1) {
        if (!r->first_tf_ready)r->first_tf_ready=ScanTimes{ros,steady};
      } else state_.waiting=*r;
    }
  }
  void select(std::uint64_t sequence,std::int64_t ros,std::int64_t steady) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto* r=find(sequence)) { r->selected=ScanTimes{ros,steady};state_.selected=*r; }
  }
  void finish(std::uint64_t sequence,bool success,std::int64_t ros,
              std::int64_t steady,const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto* r=find(sequence)) {
      r->finished=ScanTimes{ros,steady};r->processing_success=success;r->reason=bounded_text(reason);
      state_.finished=*r;if(success)state_.successful=*r;
    }
  }
  void drop(std::uint64_t sequence,const std::string& reason,
            std::int64_t ros,std::int64_t steady) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto key=(reason=="invalid" || reason=="expired" || reason=="queue_capacity" ||
                    reason=="superseded" || reason=="epoch_reset") ? reason : "other";
    ++state_.drop_counts[key];
    if (auto* r=find(sequence)) {
      r->reason=key;r->finished=ScanTimes{ros,steady};state_.dropped=*r;
    }
  }
  void defer(std::uint64_t sequence,const std::string& reason,
             std::int64_t ros,std::int64_t steady) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto key=reason=="no_map" || reason=="future" ? reason : "other";
    ++state_.defer_counts[key];
    if(auto* r=find(sequence)) {
      // A pending frame has not finished or been discarded.
      auto copy=*r;copy.reason=key;copy.deferred_at=ScanTimes{ros,steady};state_.deferred=copy;
    }
  }
  void reset(std::int64_t epoch) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto resets=state_.resets+1;
    records_.clear();state_=ScanTimingSnapshot{};state_.clock_epoch=epoch;state_.resets=resets;
    // Sequence does not restart: a delayed callback cannot name a new record.
  }
  ScanTimingSnapshot snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto copy=state_;copy.retained=records_.size();return copy;
  }
 private:
  static std::string bounded_text(const std::string& text) {
    // Preserve whole Unicode scalar values within the byte budget. Malformed
    // bytes become ASCII '?' so diagnostic conversion cannot fail in pybind.
    constexpr std::size_t limit=256;
    std::string result;result.reserve(limit);
    for(std::size_t at=0;at<text.size() && result.size()<limit;) {
      const auto lead=static_cast<unsigned char>(text[at]);
      unsigned width=1;std::uint32_t value=lead,minimum=0;bool valid=true;
      if(lead<0x80) {}
      else if(lead>=0xc2 && lead<=0xdf) {width=2;value=lead&31;minimum=0x80;}
      else if(lead>=0xe0 && lead<=0xef) {width=3;value=lead&15;minimum=0x800;}
      else if(lead>=0xf0 && lead<=0xf4) {width=4;value=lead&7;minimum=0x10000;}
      else valid=false;
      if(width>text.size()-at)valid=false;
      for(unsigned offset=1;valid && offset<width;++offset) {
        const auto next=static_cast<unsigned char>(text[at+offset]);
        if((next&0xc0)!=0x80)valid=false;
        else value=(value<<6)|(next&63);
      }
      if(value<minimum || value>0x10ffff || (value>=0xd800 && value<=0xdfff))valid=false;
      if(!valid) {result+='?';++at;continue;}
      if(width>limit-result.size())break;
      result.append(text,at,width);at+=width;
    }
    return result;
  }
  ScanTimingRecord* find(std::uint64_t sequence) {
    for(auto& record:records_)if(record.sequence==sequence)return &record;
    ++state_.missing_updates;return nullptr;
  }
  mutable std::mutex mutex_;
  std::uint64_t sequence_=0;
  std::deque<ScanTimingRecord> records_;
  ScanTimingSnapshot state_;
};
}  // namespace astribot::navigation
