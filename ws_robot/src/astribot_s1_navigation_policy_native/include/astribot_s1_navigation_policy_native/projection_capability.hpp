#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <stdexcept>
#include <map>
#include <optional>
#include <cmath>
#include <cctype>
#include <algorithm>
namespace astribot::navigation {
struct CameraCapabilitySample {
  std::string camera,frame,epoch;
  bool valid{false};
  int64_t header_ns{0},capture_ns{0},until_ns{0};
};
struct ProjectionCapabilityStatus {
  bool ready{false};
  std::string reason{"CAMERA_PROJECTION_UNAVAILABLE"};
  uint64_t revision{0};
  uint64_t future_samples_rejected{0};
  int64_t released_ns{0};
};
class ProjectionCapabilityGate {
 public:
  explicit ProjectionCapabilityGate(const std::vector<std::string>& cameras) {
    if(cameras.size()>16)throw std::invalid_argument("too many projection sources");
    for(const auto& camera:cameras) {
      if(camera.empty()||camera.size()>64||!std::all_of(camera.begin(),camera.end(),[](unsigned char c){return std::isalnum(c)||c=='_';})||
         !sources_.emplace(camera,Source{}).second)throw std::invalid_argument("invalid/duplicate projection camera");
    }
  }
  bool enabled()const{return !sources_.empty();}
  void raw(const CameraCapabilitySample& s,int64_t now,double wall) {receive(s,now,wall,false);}
  void processing(const CameraCapabilitySample& s,int64_t now,double wall) {receive(s,now,wall,true);}
  ProjectionCapabilityStatus evaluate(int64_t now,double wall) {
    if(!enabled()){status_.ready=true;status_.reason="DISABLED";return status_;}
    for(const auto& [camera,source]:sources_) {
      if(!fresh(source.raw,now,wall)){block("CAMERA_HEALTH_INVALID:"+camera,now);return status_;}
      if(!fresh(source.processing,now,wall)){block("PROJECTION_HEALTH_INVALID:"+camera,now);return status_;}
      if(source.raw->sample.frame!=source.processing->sample.frame){block("CAMERA_PROJECTION_FRAME_MISMATCH:"+camera,now);return status_;}

    }
    if(!status_.ready){status_.ready=true;++status_.revision;status_.released_ns=now;}
    status_.reason="CAMERA_PROJECTION_READY";return status_;
  }
 private:
  struct Received {CameraCapabilitySample sample;double received_wall,capture_wall;};
  struct Source {std::optional<Received> raw,processing;};
  std::map<std::string,Source> sources_;
  ProjectionCapabilityStatus status_;
  void block(const std::string& reason,int64_t now) {
    if(status_.ready){++status_.revision;status_.released_ns=0;}
    status_.ready=false;status_.reason=reason;
  }
  static bool fresh(const std::optional<Received>& value,int64_t now,double wall) {
    if(!value)return false;
    (void)now;(void)wall;
    return value->sample.valid;
  }

  void receive(const CameraCapabilitySample& s,int64_t now,double wall,bool processing) {
    auto found=sources_.find(s.camera);if(found==sources_.end())return;
    auto& slot=processing?found->second.processing:found->second.raw;
    const bool changed=slot&&(s.epoch!=slot->sample.epoch||s.frame!=slot->sample.frame);
    if(slot&&!changed&&(s.header_ns<slot->sample.header_ns||s.capture_ns<slot->sample.capture_ns))return;
    if(s.frame.empty()||s.frame.size()>512||s.epoch.empty()||s.epoch.size()>512||
       !std::isfinite(wall)) {
      slot.reset();block("CAMERA_PROJECTION_INVALID_SAMPLE:"+s.camera,now);return;
    }
    if(!s.valid||changed)block(changed?"CAMERA_PROJECTION_CONTEXT_CHANGED:"+s.camera:"CAMERA_PROJECTION_REVOKED:"+s.camera,now);
    const double capture_wall=slot&&!changed&&s.capture_ns==slot->sample.capture_ns?slot->capture_wall:wall;
    slot=Received{s,wall,capture_wall};
  }
};
}
