#include "astribot_sim_validation/session_evidence.hpp"
#include <gtest/gtest.h>
using namespace astribot_sim_validation;using Json=nlohmann::json;
TEST(SessionEvidence, GeometryMustBeCompleteAndSourceFreshBeforeGrasp){
 const int64_t now=1000000000;
 EXPECT_TRUE(geometryAdmission(true,true,now,now+300000000,now));
 EXPECT_FALSE(geometryAdmission(false,true,now,now+300000000,now));
 EXPECT_FALSE(geometryAdmission(true,false,now,now+300000000,now));
 EXPECT_FALSE(geometryAdmission(true,true,now+1,now+300000000,now));
 EXPECT_FALSE(geometryAdmission(true,true,now-300000001,now+300000000,now));
 EXPECT_FALSE(geometryAdmission(true,true,now,now,now));
}
TEST(SessionEvidence, PreviousSuccessCannotCompleteNextTask){
 Json session={{"state","SUCCEEDED"},{"run","previous"}};
 EXPECT_FALSE(terminalForNewRun(session,"previous"));
 session["run"]="next";EXPECT_TRUE(terminalForNewRun(session,"previous"));
 session["state"]="RUNNING";EXPECT_FALSE(terminalForNewRun(session,"previous"));
 session["state"]="RECOVERY_REQUIRED";EXPECT_TRUE(terminalForNewRun(session,"previous"));
 session["run"]="";EXPECT_FALSE(terminalForNewRun(session,"previous"));
 EXPECT_FALSE(terminalForNewRun(Json::object(),"previous"));
}
TEST(SimulationIsolation, RejectsEveryMissingBoundary){
 EXPECT_TRUE(isolationValid("213","astribot_operator_validation_213","1",true));
 EXPECT_FALSE(isolationValid("25","astribot_operator_validation_213","1",true));
 EXPECT_FALSE(isolationValid("213","default","1",true));
 EXPECT_FALSE(isolationValid("213","astribot_operator_validation_213","0",true));
 EXPECT_FALSE(isolationValid("213","astribot_operator_validation_213","1",false));
}
TEST(SessionEvidence, ExitCodeCannotReplacePayloadEvidence){
 Json ledger={{"stage","SUCCEEDED"},{"object",{{"state","PLACED"},{"attachment",""}}}};
 EXPECT_TRUE(completedLedger(ledger,true));EXPECT_FALSE(completedLedger(ledger,false));
 ledger["object"]["state"]="ATTACHED";EXPECT_FALSE(completedLedger(ledger,true));
 ledger["object"]["state"]="PLACED";ledger["object"]["attachment"]="left_tcp";EXPECT_FALSE(completedLedger(ledger,true));
 ledger["object"]["attachment"]="";ledger["stage"]="FAULT";EXPECT_FALSE(completedLedger(ledger,true));
 EXPECT_FALSE(completedLedger(Json::object(),true));
}
