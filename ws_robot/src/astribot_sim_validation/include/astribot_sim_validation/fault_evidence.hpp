#pragma once
#include <cmath>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>

namespace astribot_sim_validation {
// Require distinct source timestamps as well as recent receipt: repeated stale
// packets must not manufacture a stopped interval.
class StabilityWindow {
public:
  void observe(int64_t stamp, double wall, bool acceptable) {
    if (!acceptable || stamp<=0 || stamp<last_) {reset();return;}
    if (stamp==last_) {return;}
    if (last_ && stamp-last_>500000000) {reset();}
    last_=stamp;wall_=wall;if (!first_) {first_=stamp;}++samples_;
  }
  bool ready(double wall) const {
    return samples_>=3 && last_-first_>=500000000 && wall>=wall_ && wall-wall_<=.5;
  }
  void reset() {first_=last_=0;samples_=0;wall_=-1.;}
private:
  int64_t first_{0},last_{0};unsigned samples_{0};double wall_{-1.};
};

inline bool faultStageReady(const nlohmann::json & ledger,const std::string & stage,
                           double speed,double minimum_speed) {
  try {return ledger.at("stage")==stage && ledger.at("object").at("state")=="ATTACHED" &&
    !ledger.at("object").at("attachment").get<std::string>().empty() &&
    std::isfinite(speed) && speed>=minimum_speed;}
  catch (const nlohmann::json::exception &) {return false;}
}
inline bool canceledWithPayload(const nlohmann::json & ledger,const nlohmann::json & before) {
  try {return ledger.at("stage")=="CANCELED" && ledger.at("reason")=="USER_CANCEL" &&
    ledger.at("stop_error")=="" && ledger.at("object").at("state")=="ATTACHED" &&
    ledger.at("object").at("object_id")==before.at("object_id") &&
    ledger.at("object").at("version")==before.at("version") &&
    ledger.at("object").at("attachment")==before.at("attachment");}
  catch (const nlohmann::json::exception &) {return false;}
}
}
