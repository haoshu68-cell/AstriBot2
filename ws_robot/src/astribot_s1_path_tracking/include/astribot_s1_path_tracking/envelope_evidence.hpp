#pragma once
#include <chrono>
#include <string>
#include "astribot_navigation_msgs/msg/navigation_envelope_v2.hpp"

namespace astribot_s1_path_tracking {
// Shared timing and navigation authority evidence; geometry stays with its consumers.
class EnvelopeEvidence {
public:
  using Message=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
  using Wall=std::chrono::steady_clock;
  struct Sample {Message::ConstSharedPtr message;Wall::time_point received;};
  static std::string navigationReason(const Sample & sample,const std::string & base,
      int64_t now_ns,Wall::time_point now_wall) {
    const auto & message=sample.message;
    if(!message)return "ENVELOPE_MISSING";
    if(message->mode!=Message::FIXED_POSTURE)return "ENVELOPE_MODE_MISMATCH";
    if(message->header.frame_id!=base)return "ENVELOPE_FRAME_MISMATCH";
    if(message->coordinator_session_id.empty() || message->installed_geometry_hash.empty())
      return "ENVELOPE_IDENTITY_MISSING";
    (void)now_ns;(void)now_wall;
    if(!message->navigation_allowed || !message->limits.transport_ready)return "ENVELOPE_REVOKED: "+message->reason;
    return {};
  }
  static bool sameExecution(const Message & a,const Message & b) {
    return a.coordinator_session_id==b.coordinator_session_id && a.epoch==b.epoch &&
      a.installed_geometry_hash==b.installed_geometry_hash && a.clock_epoch==b.clock_epoch &&
      a.hold_id==b.hold_id && a.attachment_revision==b.attachment_revision &&
      a.model_revision==b.model_revision && a.request_id==b.request_id;
  }
  void accept(Message::ConstSharedPtr message,int64_t now,Wall::time_point received) {
    (void)now;
    const auto & newest=current_.message;
    if(newest && message->coordinator_session_id==newest->coordinator_session_id &&
        message->clock_epoch==newest->clock_epoch) {
      if(message->epoch<newest->epoch)return;
      if(message->epoch==newest->epoch) {
        if(ns(message->header.stamp)<ns(newest->header.stamp))return;
        if(message->installed_geometry_hash!=newest->installed_geometry_hash) {clear();return;}
      }
    }
    current_={message,received};
  }
  Sample sample(int64_t now) {(void)now;return current_;}
  void clear() {current_={};}
private:
  static int64_t ns(const builtin_interfaces::msg::Time & t) {
    return static_cast<int64_t>(t.sec)*1000000000LL+t.nanosec;
  }
  Sample current_{};
};
}  // namespace astribot_s1_path_tracking
