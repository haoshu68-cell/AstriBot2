#include "astribot_s1_transport_native/arm_hold.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
namespace astribot::transport {
namespace {
constexpr int64_t MS=1000000,MAX_TIME=int64_t(INT32_MAX)*1000000000+999999999;
void require(bool value,const char *reason) {if(!value)throw std::invalid_argument(reason);}
int64_t ns(const builtin_interfaces::msg::Time &t) {require(t.sec>=0 && t.nanosec<1000000000,"INVALID_TIME");return int64_t(t.sec)*1000000000+t.nanosec;}
builtin_interfaces::msg::Time stamp(int64_t n) {
  require(n>=0 && n<=MAX_TIME,"INVALID_TIME");builtin_interfaces::msg::Time t;t.sec=n/1000000000;t.nanosec=n%1000000000;return t;
}
bool identity(const Geometry &a,const Geometry &b) {
  return a.source_id==b.source_id && a.clock_epoch==b.clock_epoch && a.model_revision==b.model_revision &&
    a.attachment_revision==b.attachment_revision && a.header.frame_id==b.header.frame_id &&
    a.joints.name==b.joints.name && a.joint_position_error_bounds==b.joint_position_error_bounds;
}
Geometry normalized(Geometry g) {
  const auto count=g.joints.name.size();std::set<std::string> names(g.joints.name.begin(),g.joints.name.end());
  require(count>0 && count<=128 && names.size()==count && g.joints.position.size()==count &&
          g.joint_source_stamps.size()==count && g.joint_position_error_bounds.size()==count,"INVALID_JOINT_ARRAYS");
  require(!g.source_id.empty() && !g.model_revision.empty() && !g.attachment_revision.empty() && !g.header.frame_id.empty(),"MISSING_GEOMETRY_IDENTITY");
  std::map<std::string,std::tuple<double,double,builtin_interfaces::msg::Time>> sorted;
  int64_t low=MAX_TIME,high=0;
  for(std::size_t i=0;i<count;++i) {
    require(!g.joints.name[i].empty() && std::isfinite(g.joints.position[i]) &&
            std::isfinite(g.joint_position_error_bounds[i]) && g.joint_position_error_bounds[i]>0 && g.joint_position_error_bounds[i]<=.025,"INVALID_JOINT_VALUE");
    const auto at=ns(g.joint_source_stamps[i]);low=std::min(low,at);high=std::max(high,at);
    sorted[g.joints.name[i]]={g.joints.position[i],g.joint_position_error_bounds[i],g.joint_source_stamps[i]};
  }
  require(low>0 && ns(g.header.stamp)>0,"JOINT_SOURCE_TIMING_INVALID");
  g.joints.name.clear();g.joints.position.clear();g.joint_position_error_bounds.clear();g.joint_source_stamps.clear();
  for(const auto &[name,value]:sorted) {g.joints.name.push_back(name);g.joints.position.push_back(std::get<0>(value));
    g.joint_position_error_bounds.push_back(std::get<1>(value));g.joint_source_stamps.push_back(std::get<2>(value));}
  return g;
}
}
void ArmHold::revoke(const std::string &reason) {active_=false;reason_=reason;samples_.clear();}
bool ArmHold::clocks(int64_t now,int64_t steady) {
  const bool valid=steady>=0 && (last_steady_<0 || steady>=last_steady_);
  last_ros_=now;last_steady_=steady;if(!valid)revoke("HOLD_CLOCK_RESET");return valid;
}
void ArmHold::begin(const std::string &id,const ResourceGrant &grant,const HoldCompletion &completed,int64_t now,int64_t steady) {
  require(clocks(now,steady),"HOLD_CLOCK_RESET");require(!active_,"HOLD_ALREADY_ACTIVE");
  require(!id.empty() && id.size()<=256 && !used_ids_.count(id) && used_ids_.size()<64,"HOLD_ID_REUSE_OR_EXHAUSTED");
  require(!grant.owner_id.empty() && !grant.lease_id.empty() && !grant.epoch.empty() && !grant.joints.empty(),"RESOURCE_GRANT_REQUIRED");
  require(grant.valid_until>grant.issued_at && grant.valid_until-grant.issued_at<=5000*MS &&
          grant.received_steady>=0 && grant.received_steady<=steady &&
          steady-grant.received_steady<grant.valid_until-grant.received_at,"RESOURCE_LEASE_EXPIRED");
  require(completed.successful && completed.terminal && !completed.action_id.empty() && completed.owner_id==grant.owner_id &&
          completed.lease_id==grant.lease_id && completed.resource_epoch==grant.epoch,"TASK_OWNED_HOLD_COMPLETION_REQUIRED");
  used_ids_.insert(id);id_=id;grant_=grant;after_=steady;active_=true;reference_.reset();latest_.reset();
  samples_.clear();claims_.clear();claims_ros_=claims_steady_=-1;confirmed_once_=false;reason_="WAITING_FOR_MEASURED_SETTLING";
}
bool ArmHold::resource(const ResourceGrant &g,int64_t now,int64_t steady) {
  if(!clocks(now,steady) || !active_)return false;
  (void)status(now,steady);if(!active_)return false;
  try {
    require(g.owner_id==grant_->owner_id && g.lease_id==grant_->lease_id && g.epoch==grant_->epoch && g.joints==grant_->joints,"HOLD_RESOURCE_IDENTITY_CHANGED");
    if(g.received_steady<grant_->received_steady)return false;
    if(g.received_steady==grant_->received_steady) {require(g.valid_until==grant_->valid_until,"HOLD_RESOURCE_REPLAY_CONFLICT");return false;}
    require(g.valid_until>g.issued_at && g.valid_until-g.issued_at<=5000*MS && g.received_steady>=0 && g.received_steady<=steady &&
      steady-g.received_steady<g.valid_until-g.received_at,"HOLD_RESOURCE_EXPIRED");
    grant_=g;return true;
  }catch(const std::exception &e) {revoke(e.what());return false;}
}
bool ArmHold::geometry(const Geometry &input,int64_t now,int64_t steady) {
  if(!clocks(now,steady) || !active_)return false;
  (void)status(now,steady);if(!active_)return false;
  try {
    require(input.sequence>0,"GEOMETRY_SEQUENCE_INVALID");
    if(latest_ && input.source_id==latest_->source_id && input.clock_epoch==latest_->clock_epoch &&
       (input.sequence<latest_->sequence || ns(input.header.stamp)<ns(latest_->header.stamp)))return false;
    require(input.complete && input.attachment_state_confirmed,"GEOMETRY_UNCONFIRMED");
    auto g=normalized(input);const auto at=ns(g.header.stamp);
    if(latest_ && g.source_id==latest_->source_id && g.clock_epoch==latest_->clock_epoch && g.sequence==latest_->sequence) {
      auto duplicate=g;duplicate.valid_until=latest_->valid_until;
      require(duplicate==*latest_,"HOLD_CONFLICTING_SEQUENCE");
    }
    require(at>0 && steady>=after_,"GEOMETRY_SOURCE_INVALID");
    for(const auto &joint:g.joints.name)require(grant_->joints.count(joint),"JOINT_RESOURCE_NOT_OWNED");
    if(reference_) {
      require(identity(g,*reference_),"HOLD_GEOMETRY_VERSION_CHANGED");
      for(std::size_t i=0;i<g.joints.position.size();++i)
        require(std::abs(g.joints.position[i]-reference_->joints.position[i])<=reference_->joint_position_error_bounds[i],"HOLD_POSITION_DRIFT");
    }
    if(latest_ && identity(g,*latest_) && at<=ns(latest_->header.stamp)) {
      if(at==ns(latest_->header.stamp)) {
        require(g.joints.position==latest_->joints.position && g.joint_source_stamps==latest_->joint_source_stamps,"HOLD_CONFLICTING_CAPTURE");
        latest_->sequence=std::max(latest_->sequence,g.sequence);
      }
      return false;
    }
    if(latest_ && !identity(g,*latest_))samples_.clear();
    if(!samples_.empty() && steady-samples_.back().steady>300*MS)samples_.clear();
    samples_.push_back({at,steady,g.joints.position});
    while(samples_.size()>2 && steady-samples_[1].steady>=500*MS)samples_.pop_front();
    if(samples_.size()>128) {revoke("HOLD_SAMPLE_CAPACITY_EXCEEDED");return false;}
    latest_=g;
    if(!reference_ && samples_.size()>=3 && steady-samples_.front().steady>=500*MS) {
      bool settled=true;
      for(std::size_t i=0;i<g.joints.position.size();++i) {
        double low=g.joints.position[i],high=low;
        for(const auto &sample:samples_) {low=std::min(low,sample.positions[i]);high=std::max(high,sample.positions[i]);}
        settled=settled && high-low<=g.joint_position_error_bounds[i]/4.;
      }
      if(settled)reference_=g;
    }
    return true;
  }catch(const std::exception &e) {revoke(e.what());return false;}
}
void ArmHold::controllers(const std::vector<controller_manager_msgs::msg::ControllerState> &controllers,
    int64_t request_at,int64_t request_steady,int64_t now,int64_t steady) {
  if(!clocks(now,steady) || !active_)return;
  (void)status(now,steady);if(!active_)return;
  if(request_steady<claims_steady_)return;
  std::set<std::string> interfaces;
  bool valid=request_steady>=0 && request_steady<=steady && steady-request_steady<500*MS;
  for(const auto &controller:controllers)if(controller.state=="active" && controller.type=="joint_trajectory_controller/JointTrajectoryController")
    for(const auto &interface:controller.claimed_interfaces)if(!interfaces.insert(interface).second)valid=false;
  if(!valid) {claims_.clear();if(confirmed_once_)revoke("HOLD_CONTROLLER_STATE_INVALID");return;}
  claims_=std::move(interfaces);claims_ros_=request_at;claims_steady_=request_steady;
  if(confirmed_once_)for(const auto &joint:reference_->joints.name)if(!claims_.count(joint+"/position")) {revoke("HOLD_CONTROLLER_CLAIM_LOST");break;}
}
void ArmHold::cancel() {revoke("HOLD_CANCELED");}
Hold ArmHold::status(int64_t now,int64_t steady) {
  clocks(now,steady);Hold result;result.header.stamp=stamp(std::clamp<int64_t>(now,0,MAX_TIME));result.hold_id=id_;
  if(grant_)result.owner_id=grant_->owner_id;
  if(reference_)result.attachment_revision=reference_->attachment_revision;
  if(!active_)return result;
  if(steady<grant_->received_steady || steady-grant_->received_steady>=grant_->valid_until-grant_->received_at) {revoke("HOLD_RESOURCE_EXPIRED");return result;}
  if(!reference_ || !latest_)return result;
  if(claims_steady_<0 || steady<claims_steady_ || steady-claims_steady_>=500*MS) {
    if(confirmed_once_)revoke("HOLD_CONTROLLER_STATE_EXPIRED");else reason_="WAITING_FOR_CONTROLLER_STATE";return result;
  }
  for(const auto &joint:reference_->joints.name)if(!claims_.count(joint+"/position")) {
    if(confirmed_once_)revoke("HOLD_CONTROLLER_CLAIM_LOST");else reason_="WAITING_FOR_CONTROLLER_CLAIMS";return result;
  }
  const auto remaining=std::min({300*MS,grant_->valid_until-grant_->received_at-(steady-grant_->received_steady),
    500*MS-(steady-claims_steady_)});
  result.lease_s=double(remaining)/1e9;result.hold_confirmed=remaining>0;confirmed_once_=result.hold_confirmed;reason_="HOLD_CONFIRMED";return result;
}
bool ArmHold::observationMatches(const Hold &message,int64_t first_published,int64_t received_ros,
    int64_t received_steady,int64_t now,int64_t steady) const {
  if(!active_ || !confirmed_once_ || !message.hold_confirmed ||
     message.owner_id!=grant_->owner_id || message.hold_id!=id_ ||
     message.attachment_revision!=latest_->attachment_revision ||
     message.header.stamp.sec<0 || message.header.stamp.nanosec>=1000000000 ||
     !std::isfinite(message.lease_s) || message.lease_s<=0 || message.lease_s>.5)return false;
  const auto source=ns(message.header.stamp);
  (void)received_ros;(void)now;
  return source>=first_published && received_steady<=steady;
}
astribot_navigation_msgs::srv::SetFixedEnvelope::Request ArmHold::request(const std::string &request_id,
    const astribot_navigation_msgs::msg::RobotEnvelope &limits,int64_t now,int64_t steady) {
  require(status(now,steady).hold_confirmed,"ARM_HOLD_UNCONFIRMED");require(!request_id.empty(),"REQUEST_ID_REQUIRED");
  astribot_navigation_msgs::srv::SetFixedEnvelope::Request request;request.request_id=request_id;
  request.hold_id=id_;request.geometry_sequence=latest_->sequence;request.limits=limits;return request;
}
} // namespace astribot::transport
