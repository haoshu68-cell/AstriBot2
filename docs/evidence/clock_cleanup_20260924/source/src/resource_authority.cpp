#include "astribot_s1_transport_native/resource_authority.hpp"
#include <algorithm>
#include <cctype>
#include <limits>
#include <stdexcept>
namespace astribot::transport {
namespace {
constexpr int64_t LEASE=2000000000,MAX_TIME=int64_t(INT32_MAX)*1000000000+999999999;
bool name(const std::string &v){return !v.empty() && v.size()<=128 && std::all_of(v.begin(),v.end(),[](unsigned char c){return std::isalnum(c)||c=='_'||c=='-'||c=='.';});}
}
ResourceAuthority::ResourceAuthority(std::string epoch,std::set<std::string> joints,Commit commit,std::optional<nlohmann::json> restored)
 :epoch_(std::move(epoch)),joints_(std::move(joints)),commit_(std::move(commit)) {
 if(!name(epoch_)||joints_.empty()||joints_.size()>128||!commit_)throw std::invalid_argument("INVALID_RESOURCE_AUTHORITY");
 for(const auto &joint:joints_)if(!name(joint))throw std::invalid_argument("INVALID_RESOURCE_JOINT");
 if(restored) {
  const auto &s=*restored;
  if(s.at("schema")!="astribot.resource/1" || s.at("joints").get<std::set<std::string>>()!=joints_ ||
     !s.at("requests").is_array() || s.at("requests").size()>128)throw std::invalid_argument("RESOURCE_RECOVERY_RECORD_INVALID");
  for(const auto &r:s.at("requests")) {
   auto task=r.at(0).get<std::string>(),request=r.at(1).get<std::string>();
   if(!name(task)||!name(request)||!requests_.emplace(task,request).second)throw std::invalid_argument("RESOURCE_RECOVERY_RECORD_INVALID");
  }
  const int phase=s.at("phase");if(phase<0||phase>5)throw std::invalid_argument("RESOURCE_RECOVERY_RECORD_INVALID");
  if(phase!=int(ResourcePhase::IDLE)){phase_=ResourcePhase::QUARANTINED;reason_="RESOURCE_RECOVERY_REQUIRED";side_effects_=true;}
 }
}
nlohmann::json ResourceAuthority::snapshot(const std::string &event)const {
 return {{"schema","astribot.resource/1"},{"event",event},{"phase",int(phase_)},{"reason",reason_},
  {"epoch",epoch_},{"owner",lease_.owner_id},{"lease_id",lease_.lease_id},{"request",request_},
  {"joints",joints_},{"requests",requests_},{"side_effects",side_effects_},
  {"issued_at",lease_.issued_at},{"valid_until",lease_.valid_until},{"renewal_sequence",renewal_}};
}
bool ResourceAuthority::persist(const std::string &event) {
 if(storage_failed_)return false;
 try{commit_(snapshot(event));return true;}catch(...){storage_failed_=true;phase_=ResourcePhase::QUARANTINED;reason_="RESOURCE_PERSISTENCE_FAILED";return false;}
}
bool ResourceAuthority::record(const std::string &event,const nlohmann::json &details) {
 if(storage_failed_)return false;
 try{auto value=snapshot(event);value["details"]=details;commit_(value);return true;}
 catch(...){storage_failed_=true;phase_=ResourcePhase::QUARANTINED;reason_="RESOURCE_PERSISTENCE_FAILED";return false;}
}
void ResourceAuthority::quarantine(const std::string &reason){phase_=ResourcePhase::QUARANTINED;reason_=reason;persist("quarantined");}
bool ResourceAuthority::clocks(int64_t now,int64_t steady) {
 const bool okay=now>=0&&now<=MAX_TIME-LEASE&&steady>=0&&(last_ros_<0||now>=last_ros_)&&(last_steady_<0||steady>=last_steady_);
 last_ros_=now;last_steady_=steady;if(!okay)quarantine("RESOURCE_CLOCK_RESET");return okay;
}
Acquisition ResourceAuthority::acquire(const std::string &task,const std::string &request,int64_t now,int64_t steady) {
 if(!clocks(now,steady))return {false,false,reason_};
 if(!name(task)||!name(request))return {false,false,"INVALID_TASK_IDENTITY"};
 if(requests_.count({task,request}))return {false,true,"REQUEST_ALREADY_RECORDED"};
 if(phase_!=ResourcePhase::IDLE||storage_failed_)return {false,false,"RESOURCES_BUSY_OR_QUARANTINED"};
 if(requests_.size()>=128||number_==UINT64_MAX)return {false,false,"RESOURCE_HISTORY_EXHAUSTED"};
 requests_.emplace(task,request);request_=request;renewal_=0;side_effects_=false;
 lease_={task,epoch_+"_"+std::to_string(++number_),epoch_,joints_,now,now+LEASE,steady,now};
 phase_=ResourcePhase::RESERVED;reason_="RESERVED";
 if(!persist("reserved"))return {false,false,reason_};
 return {true,false,reason_};
}
std::optional<ResourceGrant> ResourceAuthority::grant(int64_t now,int64_t steady) {
 if(!clocks(now,steady)||storage_failed_||phase_==ResourcePhase::IDLE||phase_==ResourcePhase::STOPPING||phase_==ResourcePhase::QUARANTINED)return {};
 if(now>=lease_.valid_until||steady-lease_.received_steady>=lease_.valid_until-lease_.received_at){quarantine("RESOURCE_LEASE_EXPIRED");return {};}
 return lease_;
}
bool ResourceAuthority::renew(const std::string &lease,const std::string &epoch,uint64_t sequence,int64_t now,int64_t steady) {
 if(!grant(now,steady)||lease!=lease_.lease_id||epoch!=epoch_||sequence<=renewal_)return false;
 renewal_=sequence;lease_.issued_at=lease_.received_at=now;lease_.received_steady=steady;lease_.valid_until=now+LEASE;
 // Lease renewals are volatile. Persisted unresolved ownership already blocks
 // every restart; a heartbeat cannot change the durable handoff decision.
 return true;
}
void ResourceAuthority::submitted(int64_t now,int64_t steady) {
 if(!grant(now,steady)||phase_!=ResourcePhase::RESERVED)throw std::logic_error("RESOURCE_NOT_RESERVED");
 side_effects_=true;phase_=ResourcePhase::EXECUTING;reason_="EXECUTING";
 if(!persist("before_child_submission"))throw std::runtime_error(reason_);
}
void ResourceAuthority::holding(int64_t now,int64_t steady) {
 if(!grant(now,steady)||phase_!=ResourcePhase::EXECUTING)throw std::logic_error("RESOURCE_NOT_EXECUTING");
 phase_=ResourcePhase::HOLDING;reason_="HOLDING";if(!persist("hold_confirmed"))throw std::runtime_error(reason_);
}
void ResourceAuthority::stop(const std::string &reason,int64_t now,int64_t steady) {
 if(!clocks(now,steady)||phase_==ResourcePhase::IDLE||phase_==ResourcePhase::QUARANTINED)return;
 phase_=ResourcePhase::STOPPING;reason_=reason;persist("stop_requested");
}
void ResourceAuthority::uncertain_result(const std::string &reason,int64_t now,int64_t steady) {
 if(!clocks(now,steady)||phase_==ResourcePhase::IDLE)return;
 quarantine(reason);
}
bool ResourceAuthority::release(bool terminal,bool safe,int64_t now,int64_t steady) {
 if(!clocks(now,steady)||storage_failed_||(phase_!=ResourcePhase::STOPPING&&phase_!=ResourcePhase::QUARANTINED))return false;
 if(side_effects_&&(!terminal||!safe))return false;
 // Commit the outcome before the handoff. If either write fails, in-memory and
 // restart state deny new grants; no cancel ACK is a release proof.
 if(!persist("result_release_pending"))return false;
 phase_=ResourcePhase::IDLE;reason_="RELEASED";side_effects_=false;
 return persist("resource_handoff_committed");
}
}
