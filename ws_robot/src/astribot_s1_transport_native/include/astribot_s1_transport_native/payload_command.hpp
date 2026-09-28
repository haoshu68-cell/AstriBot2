#pragma once
#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>
namespace astribot::transport {
// Binds one simulation-only physical command to the independent full-inventory
// diagnostic. This proves only application of this model's command. Scene,
// ledger/geometry reconciliation and global EMPTY are separate task barriers.
class PayloadCommand {
public:
 struct Receipt {int64_t ros,steady;};
 PayloadCommand(std::string model,bool attach,const nlohmann::json &diagnostic,
   Receipt received,int64_t ros_now,int64_t steady_now);
 uint32_t id() const{return command_;}
 // PAYLOAD_TRANSITION_OR_ERROR has no execution array entries. It cannot
 // confirm application; the physical client must propagate plugin state errors
 // and impose the transaction timeout while waiting for complete inventory.
 bool applied(const nlohmann::json &diagnostic,Receipt received,
   int64_t ros_now,int64_t steady_now);
private:
 std::string model_,source_epoch_,plugin_epoch_;
 uint64_t entity_=0,source_clock_=0,plugin_clock_=0,revision_=0;
 int64_t capture_=0;
 int64_t latest_capture_=0,unconfirmed_capture_=0;
 uint32_t command_=0;
 bool attach_=false;
};
}
