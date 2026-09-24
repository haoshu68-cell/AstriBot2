#include "astribot_s1_transport_native/payload_command.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>
namespace astribot::transport {
namespace {
void require(bool okay,const char *reason){if(!okay)throw std::runtime_error(reason);}
uint64_t natural(const nlohmann::json &value) {
 require(value.is_number_unsigned()||(value.is_number_integer()&&value.get<int64_t>()>=0),"PAYLOAD_DIAGNOSTIC_INTEGER_INVALID");
 return value.get<uint64_t>();
}
struct Snapshot {
 std::string source_epoch,plugin_epoch;
 uint64_t entity=0,source_clock=0,plugin_clock=0,revision=0,accepted=0,applied=0;
 int64_t capture=0,deadline=0;
 bool attached=false,complete=false;
};
Snapshot snapshot(const std::string &model,const nlohmann::json &d,PayloadCommand::Receipt receipt,int64_t ros,int64_t steady) {
 try {
  const auto capture=natural(d.at("stamp_ns"));
  require(capture>0&&capture<=uint64_t(INT64_MAX)&&receipt.ros>=int64_t(capture)&&ros>=receipt.ros&&ros-int64_t(capture)<300000000&&receipt.steady>=0&&steady>=receipt.steady,"PAYLOAD_DIAGNOSTIC_STALE");
  const int64_t remaining=300000000-(receipt.ros-int64_t(capture));
  require(steady-receipt.steady<remaining&&receipt.steady<=INT64_MAX-remaining,"PAYLOAD_DIAGNOSTIC_STALE");
  uint64_t entity=0;size_t matches=0;
  for(const auto &row:d.at("models"))if(row.at(1).get<std::string>()==model){entity=natural(row.at(0));++matches;}
  require(matches==1&&entity>0,"PAYLOAD_MODEL_IDENTITY_INVALID");
  Snapshot result;result.entity=entity;result.capture=int64_t(capture);result.deadline=receipt.steady+remaining;
  result.source_epoch=d.at("source_epoch").get<std::string>();
  require(!result.source_epoch.empty(),"PAYLOAD_EPOCH_MISSING");
  result.source_clock=natural(d.at("clock_epoch"));result.revision=natural(d.at("revision"));
  require(result.revision>0,"PAYLOAD_REVISION_INVALID");
  const auto reason=d.at("reason").get<std::string>();
  result.complete=reason=="EMPTY_INVENTORY_OBSERVED"||reason=="ATTACHED_INVENTORY_OBSERVED";
  if(reason=="PAYLOAD_TRANSITION_OR_ERROR") {
   require(d.at("execution").is_array()&&d.at("execution").empty(),"PAYLOAD_PENDING_EXECUTION_INVALID");
   return result;
  }
  require(result.complete||reason=="INVENTORY_CHANGED_RECONCILIATION_REQUIRED","PAYLOAD_INVENTORY_FAILED");
  const nlohmann::json *execution=nullptr;matches=0;
  for(const auto &row:d.at("execution"))if(natural(row.at("entity"))==entity){execution=&row;++matches;}
  require(matches==1,"PAYLOAD_EXECUTION_IDENTITY_INVALID");
  const auto &e=*execution;const auto accepted=natural(e.at("accepted")),applied=natural(e.at("applied"));
  require(accepted<=UINT32_MAX&&applied<=accepted,"PAYLOAD_COMMAND_COUNTER_INVALID");
  result.plugin_epoch=e.at("epoch").get<std::string>();require(!result.plugin_epoch.empty(),"PAYLOAD_EPOCH_MISSING");
  result.plugin_clock=natural(e.at("clock_epoch"));result.accepted=accepted;result.applied=applied;
  result.attached=e.at("attached").get<bool>();return result;
 }catch(const nlohmann::json::exception &e){throw std::runtime_error(std::string("PAYLOAD_DIAGNOSTIC_INVALID:")+e.what());}
}
}
PayloadCommand::PayloadCommand(std::string model,bool attach,const nlohmann::json &d,Receipt receipt,int64_t ros,int64_t steady)
 :model_(std::move(model)),attach_(attach) {
 const auto s=snapshot(model_,d,receipt,ros,steady);
 require(s.complete,"PAYLOAD_INVENTORY_UNCONFIRMED");
 require(s.accepted==s.applied,"PAYLOAD_COMMAND_ALREADY_PENDING");
 require(s.attached!=attach_,"PAYLOAD_TRANSITION_ALREADY_APPLIED");
 require(s.accepted<UINT32_MAX,"PAYLOAD_COMMAND_IDS_EXHAUSTED");
 source_epoch_=s.source_epoch;plugin_epoch_=s.plugin_epoch;entity_=s.entity;
 source_clock_=s.source_clock;plugin_clock_=s.plugin_clock;revision_=s.revision;capture_=s.capture;
 latest_capture_=s.capture;deadline_=s.deadline;
 command_=uint32_t(s.accepted+1);
}
bool PayloadCommand::applied(const nlohmann::json &d,Receipt receipt,int64_t ros,int64_t steady) {
 const auto s=snapshot(model_,d,receipt,ros,steady);
 require(s.source_epoch==source_epoch_&&s.source_clock==source_clock_&&s.entity==entity_,"PAYLOAD_COMMAND_CONTEXT_CHANGED");
 require(s.capture>=latest_capture_,"PAYLOAD_CAPTURE_REGRESSED");
 deadline_=s.capture>latest_capture_?s.deadline:std::min(deadline_,s.deadline);latest_capture_=s.capture;
 require(steady<deadline_,"PAYLOAD_DIAGNOSTIC_STALE");
 if(!s.plugin_epoch.empty())require(s.plugin_epoch==plugin_epoch_&&s.plugin_clock==plugin_clock_,"PAYLOAD_COMMAND_CONTEXT_CHANGED");
 if(!s.complete){unconfirmed_capture_=std::max(unconfirmed_capture_,s.capture);return false;}
 if(s.capture<=unconfirmed_capture_)return false;
 require(s.accepted<=command_,"FOREIGN_PAYLOAD_COMMAND");
 if(s.accepted<command_||s.applied<command_||!s.complete)return false;
 require(s.revision>revision_&&s.capture>capture_,"PAYLOAD_NEW_APPLIED_EVIDENCE_REQUIRED");
 require(s.attached==attach_,"PAYLOAD_APPLIED_STATE_CONFLICT");
 return true;
}
}
