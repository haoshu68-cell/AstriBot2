#include "astribot_s1_payload_state/consumer.hpp"
#include <algorithm>
namespace astribot::payload {
namespace {void require(bool ok,const char *reason) {if(!ok)throw std::invalid_argument(reason);}}
void Consumer::revoke(const std::string &reason) {
  if(value_)capture_floor_=std::max(capture_floor_,ns(value_->observation.observed_at));
  if(value_ || reason_!=reason)++generation_;
  value_.reset();reason_=reason;deadline_=published_deadline_=-1;
}
bool Consumer::clock(int64_t ros_now,int64_t steady_now) {
  const bool okay=ros_now>=0 && steady_now>=0 && (last_ros_<0 || ros_now>=last_ros_) && (last_steady_<0 || steady_now>=last_steady_);
  if(!okay) {capture_floor_=std::max(capture_floor_,last_ros_);revoke("ATTACHMENT_CLOCK_RESET");}
  last_ros_=ros_now;last_steady_=steady_now;return okay;
}
bool Consumer::receive(const State &s,int64_t ros_now,int64_t steady_now) {
  clock(ros_now,steady_now);
  const auto &o=s.observation;
  if(o.environment!=config_.environment || o.session_id!=config_.session || o.source_id!=config_.source ||
      config_.session.empty() || config_.source.empty())return false;
  if(retired_.count(s.ledger_epoch))return false;
  try {
    require(!s.ledger_epoch.empty() && s.ledger_epoch.size()<=256,"ATTACHMENT_INVALID_LEDGER_IDENTITY");
    if(s.ledger_epoch!=previous_epoch_) {
      if(!previous_epoch_.empty()) {
        require(retired_.size()<64,"ATTACHMENT_RESTART_BUDGET_EXHAUSTED");retired_.insert(previous_epoch_);
      }
      revoke("ATTACHMENT_LEDGER_RESTART");previous_epoch_=s.ledger_epoch;previous_ledger_revision_=0;
      previous_source_epoch_.clear();previous_clock_epoch_=0;capture_floor_=-1;
    }
    if(s.ledger_revision<previous_ledger_revision_)return false;
    const bool same_context=o.source_epoch==previous_source_epoch_ && o.clock_epoch==previous_clock_epoch_;
    // Causally older packets cannot revoke or renew a newer valid observation.
    if(value_ && same_context && s.ledger_revision==previous_ledger_revision_ && o.sequence<value_->observation.sequence)return false;
    require(s.ledger_revision>0 && !o.source_epoch.empty() && o.source_epoch.size()<=256 && o.sequence>0 && o.revision>0,"ATTACHMENT_INVALID_IDENTITY");
    require(s.confirmed && o.full_inventory && (o.status==o.EMPTY || o.status==o.ATTACHED),"ATTACHMENT_UNCONFIRMED");
    require((o.status==o.EMPTY)==o.objects.empty(),"ATTACHMENT_STATUS_CONTENT_CONFLICT");
    require(s.attachment_revision==digest(nlohmann::json::array({config_.session,s.ledger_epoch,s.ledger_revision}).dump()),"ATTACHMENT_REVISION_MISMATCH");
    require(s.geometry_digest==digest(canonical(o.objects,config_.allowed_links).dump()),"ATTACHMENT_GEOMETRY_DIGEST_MISMATCH");
    require(same_context || previous_ledger_revision_==0 || s.ledger_revision>previous_ledger_revision_,"ATTACHMENT_CONTEXT_CHANGED_WITHOUT_REVISION");
    const auto observed=ns(o.observed_at),published=ns(s.published_at),until=ns(s.valid_until),source_until=ns(o.valid_until);
    if(observed>ros_now || published>ros_now) {
      if(value_ && s.attachment_revision==value_->attachment_revision && same_context)return false;
      throw std::invalid_argument("ATTACHMENT_FUTURE_CONTEXT");
    }
    if(same_context && value_ && (observed<ns(value_->observation.observed_at) || published<ns(value_->published_at)))return false;
    require(until>ros_now && source_until>ros_now && source_until>observed && source_until-observed<=kLease &&
            until<=source_until && observed<=published && published<until,"ATTACHMENT_INVALID_LEASE");
    require(!same_context || observed>capture_floor_,"ATTACHMENT_NEW_CAPTURE_REQUIRED");
    if(same_context && value_ && s.ledger_revision==previous_ledger_revision_)
      require(o.revision==value_->observation.revision && s.geometry_digest==value_->geometry_digest && o.status==value_->observation.status,
              "ATTACHMENT_CONTENT_CHANGED_WITHOUT_REVISION");
    const bool new_capture=!value_ || !same_context || observed>ns(value_->observation.observed_at);
    const int64_t proposed=steady_now+(until-ros_now);
    deadline_=new_capture?proposed:std::min(deadline_,proposed);
    const bool new_publication=!value_ || !same_context || published>ns(value_->published_at);
    const auto publication_deadline=steady_now+(kLease-(ros_now-published));
    published_deadline_=new_publication?publication_deadline:std::min(published_deadline_,publication_deadline);
    if(!value_ || s.attachment_revision!=value_->attachment_revision)++generation_;
    if(!same_context)capture_floor_=-1;
    previous_revision_=s.attachment_revision;previous_ledger_revision_=s.ledger_revision;
    previous_clock_epoch_=o.clock_epoch;previous_source_epoch_=o.source_epoch;
    value_=s;reason_="ATTACHMENT_CONFIRMED";return true;
  } catch(const std::exception &e) {
    // Negative/transition evidence has a version too. Preserve it before dropping the usable value.
    if(s.ledger_epoch==previous_epoch_) {
      previous_ledger_revision_=std::max(previous_ledger_revision_,s.ledger_revision);
      previous_clock_epoch_=o.clock_epoch;previous_source_epoch_=o.source_epoch;
    }
    const std::string failure=e.what();
    revoke(failure);
    // A valid but pre-fault capture is a recovery retry, not a new fault.
    // Advancing the floor to every retry's receive time would permanently
    // reject a healthy stream with any positive acquisition/transport delay.
    if(failure!="ATTACHMENT_NEW_CAPTURE_REQUIRED")capture_floor_=std::max(capture_floor_,ros_now);
    return false;
  }
}
const State *Consumer::current(int64_t ros_now,int64_t steady_now) {
  clock(ros_now,steady_now);
  if(value_ && (ros_now<ns(value_->observation.observed_at) || ros_now>=ns(value_->valid_until) ||
      steady_now>=deadline_ || steady_now>=published_deadline_)) {
    revoke("ATTACHMENT_EVIDENCE_EXPIRED");capture_floor_=std::max(capture_floor_,ros_now);
  }
  return value_?&*value_:nullptr;
}
} // namespace astribot::payload
