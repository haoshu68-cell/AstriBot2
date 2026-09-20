#include "astribot_operator_backend/arm_execution_transaction.hpp"
#include <gtest/gtest.h>
using Tx=astribot_operator_backend::ArmExecutionTransaction;
using namespace std::chrono_literals;
class Execution : public ::testing::Test {
protected:
 Tx::Time now=Tx::Time{}+100s;
 Tx::Binding binding{"plan","scene","map","calibration","payload","lease"};
 Tx tx{binding,now,now+30s};
 Tx::Evidence e{binding,now,true,true,true,true,true,true,true,false,false};
 void begin(){ASSERT_TRUE(tx.begin("command",e,now));}
};
TEST_F(Execution, ConsumesPlanBeforeDispatchAndRejectsDuplicate){begin();EXPECT_TRUE(tx.consumed());EXPECT_FALSE(tx.begin("command",e,now));EXPECT_FALSE(tx.begin("different",e,now));EXPECT_TRUE(tx.blocksRelease());}
TEST_F(Execution, WrongCommandCannotFinish){begin();tx.accepted("other",now);EXPECT_EQ(tx.state(),Tx::State::Submitted);tx.terminal("other",Tx::Terminal::Success,now);EXPECT_EQ(tx.state(),Tx::State::Submitted);}
TEST_F(Execution, SuccessNeedsNewStopAndPhysicalOutcome){begin();tx.accepted("command",now);tx.terminal("command",Tx::Terminal::Success,now);EXPECT_EQ(tx.state(),Tx::State::Verifying);e.arm_stopped=e.outcome_verified=true;tx.observe(e,now);EXPECT_EQ(tx.state(),Tx::State::Verifying);e.observed=now+1ms;tx.observe(e,now+1ms);EXPECT_EQ(tx.state(),Tx::State::Succeeded);EXPECT_FALSE(tx.blocksRelease());}
TEST_F(Execution, CancelBeforeGoalAcceptCancelsLateHandle){begin();EXPECT_TRUE(tx.cancel(now));EXPECT_TRUE(tx.accepted("command",now));EXPECT_EQ(tx.state(),Tx::State::Canceling);tx.terminal("command",Tx::Terminal::Canceled,now);e.observed=now+1ms;e.arm_stopped=e.outcome_verified=true;e.lease_valid=false;tx.observe(e,now+1ms);EXPECT_EQ(tx.state(),Tx::State::Canceled);}
TEST_F(Execution, CancelDoesNotOverwriteRacingSuccess){begin();tx.cancel(now);tx.terminal("command",Tx::Terminal::Success,now);e.observed=now+1ms;e.arm_stopped=e.outcome_verified=true;tx.observe(e,now+1ms);EXPECT_EQ(tx.state(),Tx::State::Succeeded);}
TEST_F(Execution, MissingTerminalBlocksReleaseAndLateResultCannotRecover){begin();tx.cancel(now);tx.observe(e,now+5s);EXPECT_EQ(tx.state(),Tx::State::Unknown);tx.terminal("command",Tx::Terminal::Success,now+6s);EXPECT_EQ(tx.state(),Tx::State::Unknown);EXPECT_TRUE(tx.blocksRelease());}
TEST_F(Execution, MissingGraspEvidenceIsUnknown){begin();tx.terminal("command",Tx::Terminal::Success,now);e.observed=now+1ms;e.arm_stopped=true;e.payload_known=false;tx.observe(e,now+1ms);tx.observe(e,now+5s);EXPECT_EQ(tx.state(),Tx::State::Unknown);EXPECT_TRUE(tx.blocksRelease());}
TEST_F(Execution, LeaseLossRequestsCancel){begin();tx.accepted("command",now);e.lease_valid=false;EXPECT_TRUE(tx.observe(e,now));EXPECT_EQ(tx.state(),Tx::State::Canceling);}
TEST_F(Execution, GoalAckTimeoutRequestsCancel){begin();EXPECT_TRUE(tx.observe(e,now+5s));EXPECT_EQ(tx.state(),Tx::State::Canceling);}
TEST_F(Execution, ExecutionDeadlineRequestsCancel){begin();tx.accepted("command",now);e.observed=now+120s;EXPECT_TRUE(tx.observe(e,now+120s));}
TEST_F(Execution, PersistenceFailureCannotRedispatch){begin();tx.markUnknown("ARM.PERSISTENCE_FAILED");EXPECT_FALSE(tx.begin("retry",e,now));EXPECT_TRUE(tx.blocksRelease());}
TEST_F(Execution, ClockRollbackCannotAuthorize){begin();tx.observe(e,now-1ms);EXPECT_EQ(tx.state(),Tx::State::Unknown);}
TEST_F(Execution, DuplicateTerminalCannotChangeOutcome){begin();tx.terminal("command",Tx::Terminal::Canceled,now);tx.terminal("command",Tx::Terminal::Success,now);e.observed=now+1ms;e.arm_stopped=e.outcome_verified=true;tx.observe(e,now+1ms);EXPECT_EQ(tx.state(),Tx::State::Canceled);}
class AdmissionFault : public Execution, public ::testing::WithParamInterface<int> {};
TEST_P(AdmissionFault, NeverDispatches){
 switch(GetParam()){
 case 0:e.source_unique=false;break;case 1:e.lease_valid=false;break;case 2:e.resource_locked=false;break;
 case 3:e.base_held=false;break;case 4:e.controller_ready=false;break;case 5:e.start_matches=false;break;
 case 6:e.payload_known=false;break;case 7:e.observed=now-1s;break;case 8:e.observed=now+1ms;break;
 case 9:e.binding.scene_revision="changed";break;case 10:e.binding.map_version="changed";break;
 case 11:e.binding.calibration_revision="changed";break;case 12:e.binding.payload_revision="changed";break;
 case 13:e.binding.lease_id="changed";break;case 14:e.binding.plan_id="changed";break;case 15:now+=30s;break;
 }
 EXPECT_FALSE(tx.begin("command",e,now));EXPECT_FALSE(tx.consumed());
}
INSTANTIATE_TEST_SUITE_P(MissingOrChangedEvidence,AdmissionFault,::testing::Range(0,16));

TEST_F(Execution, LateGoalAckRequiresCancelAndRecovery){begin();EXPECT_TRUE(tx.accepted("command",now+5s));EXPECT_EQ(tx.state(),Tx::State::Unknown);EXPECT_TRUE(tx.blocksRelease());}
TEST_F(Execution, LateTerminalCannotBypassTimer){begin();tx.cancel(now);tx.terminal("command",Tx::Terminal::Success,now+5s);EXPECT_EQ(tx.state(),Tx::State::Unknown);}
TEST_F(Execution, LostResourceCannotConfirmStop){begin();tx.terminal("command",Tx::Terminal::Success,now);e.observed=now+1ms;e.arm_stopped=e.outcome_verified=true;e.resource_locked=false;tx.observe(e,now+1ms);EXPECT_EQ(tx.state(),Tx::State::Verifying);}
