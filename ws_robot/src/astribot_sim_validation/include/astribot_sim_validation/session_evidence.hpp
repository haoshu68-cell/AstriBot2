#pragma once
#include <nlohmann/json.hpp>
#include <string>
namespace astribot_sim_validation {
inline bool geometryAdmission(bool complete,bool attachment_confirmed,int64_t source,int64_t until,int64_t now){
 return complete&&attachment_confirmed&&source>0&&source<=now&&until>now&&now-source<=300000000;
}
inline bool isolationValid(const std::string & domain,const std::string & partition,const std::string & localhost,bool sim_time){
 return domain=="213"&&partition=="astribot_operator_validation_213"&&localhost=="1"&&sim_time;
}
inline bool completedLedger(const nlohmann::json & ledger,bool process_success){
 try {return process_success&&ledger.at("stage")=="SUCCEEDED"&&ledger.at("object").at("state")=="PLACED"&&ledger.at("object").at("attachment")=="";}
 catch(const nlohmann::json::exception &){return false;}
}
inline bool terminalForNewRun(const nlohmann::json & session,const std::string & previous_run){
 try {const auto run=session.at("run").get<std::string>();
   return !run.empty()&&run!=previous_run&&(session.at("state")=="SUCCEEDED"||session.at("state")=="RECOVERY_REQUIRED");
 }catch(const nlohmann::json::exception &){return false;}
}
}
