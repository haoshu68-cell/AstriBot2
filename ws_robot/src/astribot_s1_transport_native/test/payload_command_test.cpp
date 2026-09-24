#include <gtest/gtest.h>
#include "astribot_s1_transport_native/payload_command.hpp"
#include <limits>
using namespace astribot::transport;
namespace {
constexpr int64_t SECOND=1000000000;
nlohmann::json inventory(bool attached=false,uint64_t accepted=7,uint64_t applied=7) {
 return {{"stamp_ns",SECOND},{"source_epoch","inventory-boot"},{"clock_epoch",2},
  {"revision",10},{"sequence",20},{"reason",attached?"ATTACHED_INVENTORY_OBSERVED":"EMPTY_INVENTORY_OBSERVED"},
  {"models",{{41,"fixture"},{1,"astribot_s1"}}},
  {"execution",{{{"entity",41},{"epoch","plugin-boot"},{"clock_epoch",3},
    {"accepted",accepted},{"applied",applied},{"attached",attached},{"parent",27}}}}};
}
nlohmann::json applied() {
 auto v=inventory(true,8,8);v["stamp_ns"]=SECOND+100;v["revision"]=11;v["sequence"]=21;return v;
}
}
// A command id reset to 1, accepting a queue ACK as physical application, or
// mixing restarted plugin/source epochs must fail these executor boundaries.
TEST(PayloadCommand, ContinuesActualPluginCounterAndRequiresAppliedInventory) {
 PayloadCommand command("fixture",true,inventory(),{SECOND,SECOND},SECOND,SECOND);
 EXPECT_EQ(command.id(),8u);
 auto pending=applied();pending["reason"]="PAYLOAD_TRANSITION_OR_ERROR";
 pending["execution"]=nlohmann::json::array();
 EXPECT_FALSE(command.applied(pending,{SECOND+100,SECOND+100},SECOND+100,SECOND+100));
 auto changed=pending;changed["source_epoch"]="restarted";
 EXPECT_THROW(command.applied(changed,{SECOND+100,SECOND+100},SECOND+100,SECOND+100),std::runtime_error);
 auto complete=applied();complete["stamp_ns"]=SECOND+101;
 EXPECT_TRUE(command.applied(complete,{SECOND+101,SECOND+101},SECOND+101,SECOND+101));
}
TEST(PayloadCommand, ReconciliationTransitionRequiresANewerCompleteCapture) {
 PayloadCommand command("fixture",true,inventory(),{SECOND,SECOND},SECOND,SECOND);
 auto transition=applied();transition["reason"]="INVENTORY_CHANGED_RECONCILIATION_REQUIRED";
 // Producer preserves the complete execution array but revokes positive
 // inventory for this capture when the command applied between samples.
 EXPECT_FALSE(command.applied(transition,{SECOND+100,SECOND+100},SECOND+100,SECOND+100));
 EXPECT_FALSE(command.applied(applied(),{SECOND+100,SECOND+100},SECOND+100,SECOND+100));
 auto complete=applied();complete["stamp_ns"]=SECOND+101;
 EXPECT_TRUE(command.applied(complete,{SECOND+101,SECOND+101},SECOND+101,SECOND+101));
}
TEST(PayloadCommand, FrozenClockCannotExtendRemainingSourceLease) {
 PayloadCommand command("fixture",true,inventory(),{SECOND,SECOND},SECOND,SECOND);
 auto value=applied();value["stamp_ns"]=1100000000;
 // Capture 1.1, receipt ROS 1.3 / steady 2.0: only 100 ms remain.
 ASSERT_TRUE(command.applied(value,{1300000000,2000000000},1300000000,2000000000));
 EXPECT_THROW(command.applied(value,{1300000000,2000000000},1300000000,2100000000),std::runtime_error);
 PayloadCommand delayed("fixture",true,inventory(),{SECOND,SECOND},SECOND,SECOND);
 // The first processing may itself occur after the receipt's original lease.
 EXPECT_THROW(delayed.applied(value,{1300000000,2000000000},1300000000,2100000000),std::runtime_error);
}
TEST(PayloadCommand, DuplicateCaptureCannotRenewItsOriginalDeadline) {
 PayloadCommand command("fixture",true,inventory(),{SECOND,SECOND},SECOND,SECOND);
 auto value=applied();value["stamp_ns"]=1100000000;
 ASSERT_TRUE(command.applied(value,{1100000000,2000000000},1100000000,2000000000));
 // A repeated publication of the same capture is not a new observation.
 EXPECT_THROW(command.applied(value,{1100000000,2300000000},1100000000,2300000000),std::runtime_error);
}
TEST(PayloadCommand, SourceAndPluginIdentityAreBoundAcrossThePhysicalCommand) {
 PayloadCommand command("fixture",true,inventory(),{SECOND,SECOND},SECOND,SECOND);
 auto value=applied();value["source_epoch"]="restarted";
 EXPECT_THROW(command.applied(value,{SECOND+100,SECOND+100},SECOND+100,SECOND+100),std::runtime_error);
 value=applied();value["execution"][0]["epoch"]="restarted";
 EXPECT_THROW(command.applied(value,{SECOND+100,SECOND+100},SECOND+100,SECOND+100),std::runtime_error);
 value=applied();value["execution"][0]["clock_epoch"]=4;
 EXPECT_THROW(command.applied(value,{SECOND+100,SECOND+100},SECOND+100,SECOND+100),std::runtime_error);
 value=applied();value["models"][0][0]=42;value["execution"][0]["entity"]=42;
 EXPECT_THROW(command.applied(value,{SECOND+100,SECOND+100},SECOND+100,SECOND+100),std::runtime_error);
}
TEST(PayloadCommand, CannotOverridePendingForeignOrExhaustedCommands) {
 EXPECT_THROW(PayloadCommand("fixture",true,inventory(false,8,7),{SECOND,SECOND},SECOND,SECOND),std::runtime_error);
 EXPECT_THROW(PayloadCommand("fixture",true,inventory(false,UINT32_MAX,UINT32_MAX),{SECOND,SECOND},SECOND,SECOND),std::runtime_error);
 EXPECT_THROW(PayloadCommand("missing",true,inventory(),{SECOND,SECOND},SECOND,SECOND),std::runtime_error);
 PayloadCommand command("fixture",true,inventory(),{SECOND,SECOND},SECOND,SECOND);
 auto value=applied();value["execution"][0]["accepted"]=9;
 EXPECT_THROW(command.applied(value,{SECOND+100,SECOND+100},SECOND+100,SECOND+100),std::runtime_error);
 value=applied();value["execution"][0]["attached"]=false;
 EXPECT_THROW(command.applied(value,{SECOND+100,SECOND+100},SECOND+100,SECOND+100),std::runtime_error);
}
TEST(PayloadCommand, FreshSourceAndReceiptAndNewRevisionAreRequired) {
 EXPECT_THROW(PayloadCommand("fixture",true,inventory(),{SECOND+300000000,SECOND},SECOND+300000000,SECOND+300000000),std::runtime_error);
 EXPECT_THROW(PayloadCommand("fixture",true,inventory(),{SECOND,SECOND},SECOND,SECOND+300000000),std::runtime_error);
 PayloadCommand command("fixture",true,inventory(),{SECOND,SECOND},SECOND,SECOND);
 auto value=applied();value["revision"]=10;
 EXPECT_THROW(command.applied(value,{SECOND+100,SECOND+100},SECOND+100,SECOND+100),std::runtime_error);
 value=applied();value["stamp_ns"]=SECOND+101;
 EXPECT_THROW(command.applied(value,{SECOND+100,SECOND+100},SECOND+100,SECOND+100),std::runtime_error);
 value=applied();value["reason"]="PAYLOAD_PARENT_IDENTITY_MISMATCH";
 EXPECT_THROW(command.applied(value,{SECOND+100,SECOND+100},SECOND+100,SECOND+100),std::runtime_error);
}
TEST(PayloadCommand, DetachDoesNotClaimTheWholeInventoryIsEmpty) {
 PayloadCommand command("fixture",false,inventory(true),{SECOND,SECOND},SECOND,SECOND);
 auto value=applied();value["execution"][0]["attached"]=false;
 // Other registered payloads may remain attached. The task must independently
 // reconcile the complete observation/PlanningScene for a global EMPTY claim.
 value["reason"]="ATTACHED_INVENTORY_OBSERVED";
 EXPECT_TRUE(command.applied(value,{SECOND+100,SECOND+100},SECOND+100,SECOND+100));
}
TEST(PayloadCommand, RejectsMalformedExternalCounterAndDuplicateIdentity) {
 auto value=inventory();value["execution"][0]["accepted"]=7.5;
 EXPECT_THROW(PayloadCommand("fixture",true,value,{SECOND,SECOND},SECOND,SECOND),std::runtime_error);
 value=inventory();value["models"].push_back({42,"fixture"});
 EXPECT_THROW(PayloadCommand("fixture",true,value,{SECOND,SECOND},SECOND,SECOND),std::runtime_error);
 value=inventory();value["execution"].push_back(value["execution"][0]);
 EXPECT_THROW(PayloadCommand("fixture",true,value,{SECOND,SECOND},SECOND,SECOND),std::runtime_error);
}
