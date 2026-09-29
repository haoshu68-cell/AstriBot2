#include "astribot_s1_payload_state/ledger.hpp"
#include <algorithm>
#include <stdexcept>
#include <limits>
namespace astribot::payload {
namespace {
void require(bool ok,const char *reason) {if(!ok)throw std::invalid_argument(reason);}
bool valid_name(const std::string &v) {return !v.empty() && v.size()<=256 && v.find('\0')==std::string::npos;}
bool confirmed_status(uint8_t status) {return status==Observation::EMPTY || status==Observation::ATTACHED;}
}
Ledger::Ledger(Config config,Commit commit):config_(std::move(config)),commit_(std::move(commit)) {
  require((config_.environment=="simulation" || config_.environment=="hardware") && valid_name(config_.session) &&
          valid_name(config_.source) && valid_name(config_.ledger_epoch) && !config_.allowed_links.empty() && bool(commit_),"INVALID_LEDGER_CONFIG");
  state_.observation.environment=config_.environment;state_.observation.session_id=config_.session;state_.observation.source_id=config_.source;
  state_.ledger_epoch=config_.ledger_epoch;state_.reason="ATTACHMENT_STATE_UNAVAILABLE";
}
void Ledger::persist(const nlohmann::json &record) {
  try {commit_(record);}catch(...) {storage_failed_=true;throw std::runtime_error("LEDGER_PERSISTENCE_FAILED");}
}
void Ledger::invalidate(const std::string &reason) {
  if(state_.confirmed || state_.reason!=reason)++generation_;
  state_.confirmed=false;state_.reason=reason;scene_ros_=scene_steady_=-1;
}
bool Ledger::clock(int64_t ros_now,int64_t steady_now) {
  if(ros_now<0 || steady_now<0) {invalidate("INVALID_TIME");return false;}
  last_ros_=ros_now;last_steady_=steady_now;
  return true;
}
bool Ledger::fresh(int64_t ros_now,int64_t steady_now) const {
  if(!have_ || clock_latched_ || storage_failed_ || source_rejected_)return false;
  (void)ros_now;(void)steady_now;return true;
}
bool Ledger::observe(const Observation &o,int64_t ros_now,int64_t steady_now,std::optional<Receipt> receipt) {
  clock(ros_now,steady_now);
  if(o.environment!=config_.environment || o.session_id!=config_.session || o.source_id!=config_.source)return false;
  if(storage_failed_)return false;
  // Retired source packets are ignored. They may not displace the current source or renew it.
  if(retired_.count(o.source_epoch))return false;
  try {
    require(valid_name(o.source_epoch) && valid_name(o.transaction_id) && o.sequence>0 && o.revision>0,"INVALID_SOURCE_IDENTITY");
    const bool same_source=have_ && o.source_epoch==state_.observation.source_epoch;
    if(same_source) {
      const auto &old=state_.observation;
      if(o.clock_epoch<old.clock_epoch)return false;
      if(o.clock_epoch==old.clock_epoch && ns(o.observed_at)<ns(old.observed_at))return false;
      if(o.clock_epoch==old.clock_epoch && o.revision<=old.revision && o.sequence<old.sequence)return false;
      if(o==old)return false;  // Exact replay cannot renew or revoke a newer lease.
    }
    const auto observed=ns(o.observed_at),until=ns(o.valid_until);
    require(o.full_inventory,"INCOMPLETE_PHYSICAL_INVENTORY");
    require(o.status<=Observation::TRANSITION,"INVALID_ATTACHMENT_STATUS");
    require(o.status!=Observation::EMPTY || o.objects.empty(),"EMPTY_WITH_PAYLOAD");
    require(o.status!=Observation::ATTACHED || !o.objects.empty(),"ATTACHED_WITHOUT_PAYLOAD");
    const auto geometry=canonical(o.objects,config_.allowed_links);const auto hash=digest(geometry.dump());
    (void)receipt;
    // A structurally valid pre-fault sample cannot renew evidence, but it is
    // not a new fault. Moving the floor to each delayed retry would starve
    // recovery forever even when every sample remains within its lease.
    if(source_rejected_ && same_source && o.clock_epoch==state_.observation.clock_epoch &&
       observed<=recovery_after_)return false;
    if(clock_latched_)require(!same_source || o.clock_epoch>clock_floor_,"CLOCK_EPOCH_NOT_RECONCILED");
    if(same_source) {
      const auto &old=state_.observation;
      if(o.clock_epoch<old.clock_epoch)return false;
      if(o.clock_epoch==old.clock_epoch) {
        if(o.sequence<old.sequence)return false;
        if(o.sequence==old.sequence) {
          require(o.revision==old.revision && o.status==old.status && hash==state_.geometry_digest &&
                  o.observed_at==old.observed_at && o.valid_until==old.valid_until,"CONFLICTING_DUPLICATE");
          return false;
        }
        require(o.revision>=old.revision && observed>=ns(old.observed_at),"NONMONOTONIC_SOURCE_STATE");
        if(o.revision==old.revision)require(hash==state_.geometry_digest && o.status==old.status,"CONTENT_CHANGED_WITHOUT_REVISION");
      }
    }
    if(!same_source || o.clock_epoch!=state_.observation.clock_epoch)recovery_after_=-1;
    const bool changed=!same_source || o.clock_epoch!=state_.observation.clock_epoch || o.revision!=state_.observation.revision;
    if(changed) {
      require(state_.ledger_revision<std::numeric_limits<uint64_t>::max(),"LEDGER_REVISION_EXHAUSTED");
      if(have_ && !same_source)require(retired_.size()<64,"SOURCE_RESTART_BUDGET_EXHAUSTED");
      const auto next=state_.ledger_revision+1;
      persist({{"event","observation"},{"ledger_epoch",config_.ledger_epoch},{"ledger_revision",next},
        {"environment",o.environment},{"session",o.session_id},{"source",o.source_id},{"source_epoch",o.source_epoch},
        {"clock_epoch",o.clock_epoch},{"source_revision",o.revision},{"sequence",o.sequence},{"observed_at_ns",observed},
        {"valid_until_ns",until},{"status",o.status},{"transaction_id",o.transaction_id},{"objects",geometry}});
      if(have_ && !same_source)retired_.insert(state_.observation.source_epoch);
      ++generation_;state_.ledger_revision=next;
      state_.attachment_revision=digest(nlohmann::json::array({config_.session,config_.ledger_epoch,next}).dump());
      state_.confirmed=false;scene_ros_=scene_steady_=-1;
    }
    received_=steady_now;state_.observation=o;state_.geometry_digest=hash;have_=true;clock_latched_=false;source_rejected_=false;
    if(!confirmed_status(o.status))invalidate(o.status==Observation::TRANSITION?"ATTACHMENT_TRANSITION":"ATTACHMENT_UNKNOWN");
    else if(!state_.confirmed)state_.reason="WAITING_FOR_SCENE_RECONCILIATION";
    return true;
  } catch(const std::invalid_argument &e) {source_fault(e.what(),ros_now,steady_now);return false;}
    catch(const std::exception &) {storage_failed_=true;invalidate("LEDGER_PERSISTENCE_FAILED");return false;}
}
void Ledger::source_fault(const std::string &reason,int64_t ros_now,int64_t steady_now) {
  clock(ros_now,steady_now);source_rejected_=true;recovery_after_=std::max(recovery_after_,ns(state_.observation.observed_at));
  const bool changed=state_.confirmed || state_.reason!=reason;invalidate(reason);
  if(changed && !storage_failed_) {
    try {persist({{"event","source_fault"},{"ledger_epoch",state_.ledger_epoch},{"ledger_revision",state_.ledger_revision},
      {"reason",reason},{"recovery_after_ns",recovery_after_}});}catch(...) {storage_failed_=true;invalidate("LEDGER_PERSISTENCE_FAILED");}
  }
}
std::optional<Ticket> Ledger::request(int64_t ros_now,int64_t steady_now) {
  clock(ros_now,steady_now);
  if(!fresh(ros_now,steady_now) || !confirmed_status(state_.observation.status))return std::nullopt;
  return Ticket{generation_,ros_now,steady_now};
}
bool Ledger::reconcile(const Ticket &ticket,const moveit_msgs::msg::PlanningScene &scene,int64_t ros_now,int64_t steady_now) {
  clock(ros_now,steady_now);
  if(ticket.generation!=generation_)return false;
  if(!fresh(ros_now,steady_now) || !confirmed_status(state_.observation.status))return false;
  if(ticket.steady_at>steady_now || steady_now-ticket.steady_at>=kLease)return false;
  try {
    require(!scene.is_diff && !scene.robot_state.is_diff,"FULL_SCENE_READBACK_REQUIRED");
    require(scene_matches(state_.observation.objects,scene.robot_state.attached_collision_objects,config_.allowed_links),"PHYSICAL_SCENE_MISMATCH");
    if(!state_.confirmed)persist({{"event","scene_confirmed"},{"ledger_epoch",state_.ledger_epoch},{"ledger_revision",state_.ledger_revision},
      {"attachment_revision",state_.attachment_revision},{"geometry_digest",state_.geometry_digest},{"source_sequence",state_.observation.sequence},
      {"request_at_ns",ticket.ros_at},{"confirmed_at_ns",ros_now}});
    scene_ros_=ticket.ros_at;scene_steady_=ticket.steady_at;state_.confirmed=true;state_.reason="ATTACHMENT_CONFIRMED";return true;
  } catch(const std::invalid_argument &e) {invalidate(e.what());return false;}
    catch(const std::exception &) {storage_failed_=true;invalidate("LEDGER_PERSISTENCE_FAILED");return false;}
}
State Ledger::state(int64_t ros_now,int64_t steady_now) {
  clock(ros_now,steady_now);
  state_.published_at=stamp(std::max<int64_t>(0,ros_now));
  state_.valid_until=state_.observation.valid_until;
  return state_;
}
} // namespace astribot::payload
