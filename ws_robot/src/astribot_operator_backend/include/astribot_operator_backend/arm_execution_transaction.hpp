#pragma once
#include <chrono>
#include <string>
#include <utility>

namespace astribot_operator_backend {
// Pure transaction core. The adapter owns resource locking, evidence collection,
// durable intent, and actual action dispatch; this type never commands hardware.
class ArmExecutionTransaction {
public:
 using Clock=std::chrono::steady_clock;
 using Time=Clock::time_point;
 enum class State { Ready, Submitted, Running, Canceling, Verifying, Succeeded, Canceled, Failed, Unknown };
 enum class Terminal { Success, Canceled, Failed };
 struct Binding {
   std::string plan_id, scene_revision, map_version, calibration_revision, payload_revision, lease_id;
   bool operator==(const Binding & b)const {
     return plan_id==b.plan_id&&scene_revision==b.scene_revision&&map_version==b.map_version&&
       calibration_revision==b.calibration_revision&&payload_revision==b.payload_revision&&lease_id==b.lease_id;
   }
   bool complete()const{return !plan_id.empty()&&!scene_revision.empty()&&!map_version.empty()&&
     !calibration_revision.empty()&&!payload_revision.empty()&&!lease_id.empty();}
 };
 struct Evidence {
   Binding binding;
   Time observed{};
   bool source_unique=false, lease_valid=false, resource_locked=false, base_held=false;
   bool controller_ready=false, start_matches=false, payload_known=false;
   bool arm_stopped=false, outcome_verified=false;
 };
private:
 Binding binding_;
 Time expires_,last_now_{},deadline_{},terminal_at_{};
 State state_=State::Ready;
 Terminal terminal_=Terminal::Failed;
 std::string reason_="ARM.READY",command_;
 bool consumed_=false,terminal_received_=false;
 bool advance(Time now){
   if(now<last_now_){state_=State::Unknown;reason_="ARM.CLOCK_ROLLBACK";return false;}
   last_now_=now;return true;
 }
 static bool fresh(const Evidence & e,Time now){return e.source_unique&&e.observed<=now&&now-e.observed<std::chrono::seconds(1);}
 bool held(const Evidence & e,Time now)const{return fresh(e,now)&&e.binding==binding_&&e.lease_valid&&
   e.resource_locked&&e.base_held&&e.controller_ready&&e.payload_known;}
public:
 ArmExecutionTransaction(Binding binding,Time created,Time expires):binding_(std::move(binding)),expires_(expires),last_now_(created){
   if(!binding_.complete()||expires<=created){state_=State::Failed;reason_="ARM.INVALID_BINDING";}
 }
 ArmExecutionTransaction(const ArmExecutionTransaction &)=delete;
 ArmExecutionTransaction & operator=(const ArmExecutionTransaction &)=delete;
 State state()const{return state_;}
 const std::string & reason()const{return reason_;}
 bool consumed()const{return consumed_;}
 bool blocksRelease()const{return consumed_&&state_!=State::Succeeded&&state_!=State::Canceled&&state_!=State::Failed;}
 // A true return authorizes exactly one dispatch. The caller must persist the
 // consumed intent before dispatch; persistence failure requires markUnknown().
 bool begin(const std::string & command,const Evidence & e,Time now){
   if(!advance(now)||state_!=State::Ready||consumed_)return false;
   if(command.empty()||now>=expires_||!held(e,now)||!e.start_matches){reason_="ARM.ADMISSION_REJECTED";return false;}
   command_=command;consumed_=true;state_=State::Submitted;reason_="ARM.SUBMITTED";deadline_=now+std::chrono::seconds(5);return true;
 }
 bool accepted(const std::string & command,Time now){
   if(!advance(now)||command!=command_)return false;
   if((state_==State::Submitted||state_==State::Canceling)&&now>=deadline_){markUnknown("ARM.ACK_OR_CANCEL_TIMEOUT");return true;}
   if(state_==State::Unknown&&consumed_)return true;
   if(state_==State::Canceling)return true; // adapter must cancel this late handle
   if(state_!=State::Submitted)return false;
   state_=State::Running;reason_="ARM.RUNNING";deadline_=now+std::chrono::seconds(120);return false;
 }
 bool cancel(Time now){
   if(!advance(now))return false;
   if(state_!=State::Submitted&&state_!=State::Running)return false;
   state_=State::Canceling;reason_="ARM.CANCEL_REQUESTED";deadline_=now+std::chrono::seconds(5);return true;
 }
 void terminal(const std::string & command,Terminal value,Time now){
   if(!advance(now)||command!=command_||terminal_received_)return;
   if(state_!=State::Submitted&&state_!=State::Running&&state_!=State::Canceling)return;
   if(now>=deadline_){markUnknown("ARM.LATE_TERMINAL");return;}
   terminal_received_=true;terminal_=value;terminal_at_=now;state_=State::Verifying;
   reason_="ARM.WAIT_STOP_AND_OUTCOME";deadline_=now+std::chrono::seconds(5);
 }
 // Returns true only when the adapter should request cancellation. An action
 // terminal alone never confirms standstill or physical grasp/place success.
 bool observe(const Evidence & e,Time now){
   if(!advance(now))return false;
   if(state_==State::Verifying){
     if(now>=deadline_){markUnknown("ARM.STOP_OR_OUTCOME_UNCONFIRMED");return false;}
     // Task lease may already be lost. Resource ownership and physical evidence
     // must remain valid; payload revision may change as the action completes.
     if(fresh(e,now)&&e.observed>terminal_at_&&e.resource_locked&&e.base_held&&
        e.binding.plan_id==binding_.plan_id&&e.binding.scene_revision==binding_.scene_revision&&
        e.binding.map_version==binding_.map_version&&e.binding.calibration_revision==binding_.calibration_revision&&
        e.arm_stopped&&e.payload_known&&e.outcome_verified){
       state_=terminal_==Terminal::Success?State::Succeeded:terminal_==Terminal::Canceled?State::Canceled:State::Failed;
       reason_=terminal_==Terminal::Success?"ARM.SUCCEEDED":terminal_==Terminal::Canceled?"ARM.CANCELED":"ARM.FAILED";
     }return false;
   }
   if(state_==State::Canceling){if(now>=deadline_)markUnknown("ARM.CANCEL_UNCONFIRMED");return false;}
   if(state_==State::Submitted||state_==State::Running){
     if(now>=deadline_||!held(e,now))return cancel(now);
   }return false;
 }
 void markUnknown(std::string reason){if(consumed_){state_=State::Unknown;reason_=std::move(reason);}}
};
}
