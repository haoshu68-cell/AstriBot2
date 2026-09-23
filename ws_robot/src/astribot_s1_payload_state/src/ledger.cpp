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
  const bool rollback=(last_ros_>=0 && ros_now<last_ros_) || (last_steady_>=0 && steady_now<last_steady_);
  last_ros_=ros_now;last_steady_=steady_now;
  if(rollback) {
    clock_floor_=have_?state_.observation.clock_epoch:0;clock_latched_=true;
    invalidate("CLOCK_RESET");observed_wall_deadline_=-1;
  }
  return !rollback;
}
bool Ledger::fresh(int64_t ros_now,int64_t steady_now) const {
  if(!have_ || clock_latched_ || storage_failed_ || source_rejected_)return false;
  const auto &o=state_.observation;
  return ns(o.observed_at)<=ros_now && ros_now<ns(o.valid_until) &&
         steady_now>=received_ && steady_now<observed_wall_deadline_;
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
      if(o.clock_epoch==old.clock_epoch && o.revision<=old.revision && o.sequence<old.sequence)return false;
      if(o==old)return false;  // Exact replay cannot renew or revoke a newer lease.
    }
    const auto observed=ns(o.observed_at),until=ns(o.valid_until);
    require(o.full_inventory,"INCOMPLETE_PHYSICAL_INVENTORY");
    require(o.status<=Observation::TRANSITION,"INVALID_ATTACHMENT_STATUS");
    require(o.status!=Observation::EMPTY || o.objects.empty(),"EMPTY_WITH_PAYLOAD");
    require(o.status!=Observation::ATTACHED || !o.objects.empty(),"ATTACHED_WITHOUT_PAYLOAD");
    const auto geometry=canonical(o.objects,config_.allowed_links);const auto hash=digest(geometry.dump());
    if(observed>ros_now && same_source && o.clock_epoch==state_.observation.clock_epoch &&
       o.revision==state_.observation.revision && o.status==state_.observation.status &&
       hash==state_.geometry_digest && until>observed && until-observed<=kLease)return false;
    const auto received=receipt.value_or(Receipt{ros_now,steady_now});
    require(received.ros_at>=0 && received.ros_at<=ros_now && received.steady_at>=0 && received.steady_at<=steady_now &&
            observed<=received.ros_at && until>received.ros_at &&
            steady_now-received.steady_at<until-received.ros_at,"ATTACHMENT_QUEUE_EXCEEDED_SOURCE_LEASE");
    require(observed<=ros_now && until>ros_now && until>observed && until-observed<=kLease,"INVALID_SOURCE_LEASE");
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
    // Identical capture time cannot be renewed by a newer sequence/receive time.
    const bool new_capture=!have_ || !same_source || o.clock_epoch!=state_.observation.clock_epoch || observed>ns(state_.observation.observed_at);
    if(new_capture)observed_wall_deadline_=received.steady_at+(until-received.ros_at);
    else observed_wall_deadline_=std::min(observed_wall_deadline_,received.steady_at+(until-received.ros_at));
    received_=steady_now;state_.observation=o;state_.geometry_digest=hash;have_=true;clock_latched_=false;source_rejected_=false;
    if(!confirmed_status(o.status))invalidate(o.status==Observation::TRANSITION?"ATTACHMENT_TRANSITION":"ATTACHMENT_UNKNOWN");
    else if(!state_.confirmed)state_.reason="WAITING_FOR_SCENE_RECONCILIATION";
    return true;
  } catch(const std::invalid_argument &e) {source_fault(e.what(),ros_now,steady_now);return false;}
    catch(const std::exception &) {storage_failed_=true;invalidate("LEDGER_PERSISTENCE_FAILED");return false;}
}
void Ledger::source_fault(const std::string &reason,int64_t ros_now,int64_t steady_now) {
  clock(ros_now,steady_now);source_rejected_=true;recovery_after_=std::max(recovery_after_,ros_now);
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
  if(ticket.ros_at>ros_now || ticket.steady_at>steady_now || ros_now-ticket.ros_at>=kLease || steady_now-ticket.steady_at>=kLease)return false;
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
  if(state_.confirmed && (!fresh(ros_now,steady_now) || scene_ros_<0 || ros_now<scene_ros_ || ros_now-scene_ros_>=kLease ||
       steady_now<scene_steady_ || steady_now-scene_steady_>=kLease))invalidate("ATTACHMENT_EVIDENCE_EXPIRED");
  state_.published_at=stamp(std::max<int64_t>(0,ros_now));state_.valid_until=stamp(0);
  if(state_.confirmed) {
    const auto wall_remaining=std::min(observed_wall_deadline_-steady_now,kLease-(steady_now-scene_steady_));
    state_.valid_until=stamp(std::min({ns(state_.observation.valid_until),scene_ros_+kLease,ros_now+wall_remaining}));
  }
  return state_;
}
} // namespace astribot::payload
