#include <gtest/gtest.h>
#include "astribot_s1_transport_native/resource_authority.hpp"
using namespace astribot::transport;
namespace {
constexpr int64_t T=1000000000,MS=1000000;
std::set<std::string> joints(){return {"left","right"};}
class AuthorityFixture:public ::testing::Test {
protected:
 std::vector<nlohmann::json> records;
 ResourceAuthority a{"boot",joints(),[this](const auto &v){records.push_back(v);}};
 auto acquire(){return a.acquire("task","request",T,T);}
};
TEST_F(AuthorityFixture, AcquisitionIsExclusiveAndReplayCannotRenew) {
 auto first=acquire();ASSERT_TRUE(first.accepted);auto grant=a.grant(T,T);ASSERT_TRUE(grant);
 EXPECT_FALSE(a.acquire("other","next",T+MS,T+MS).accepted);
 EXPECT_TRUE(a.acquire("task","request",T+MS,T+MS).duplicate);
 EXPECT_EQ(a.grant(T+MS,T+MS)->valid_until,grant->valid_until);
}
TEST_F(AuthorityFixture, ExpiryQuarantinesRatherThanReleasing) {
 ASSERT_TRUE(acquire().accepted);a.submitted(T,T);
 EXPECT_FALSE(a.grant(T+2000*MS,T+2000*MS));EXPECT_EQ(a.phase(),ResourcePhase::QUARANTINED);
 EXPECT_FALSE(a.acquire("other","next",T+2001*MS,T+2001*MS).accepted);
}
TEST_F(AuthorityFixture, UncertainChildResultQuarantinesAndRequiresActualTerminalProof) {
 ASSERT_TRUE(acquire().accepted);a.submitted(T,T);a.stop("CHILD_UNKNOWN",T+MS,T+MS);
 a.uncertain_result("RESOURCE_RECOVERY_REQUIRED",T+2*MS,T+2*MS);
 EXPECT_EQ(a.phase(),ResourcePhase::QUARANTINED);
 EXPECT_FALSE(a.grant(T+3*MS,T+3*MS));EXPECT_FALSE(a.release(false,true,T+3*MS,T+3*MS));
 EXPECT_FALSE(a.acquire("next","next",T+4*MS,T+4*MS).accepted);
}
TEST_F(AuthorityFixture, RenewalCannotReviveGapOrReplay) {
 ASSERT_TRUE(acquire().accepted);auto g=*a.grant(T,T);
 EXPECT_FALSE(a.renew("wrong",g.epoch,1,T+100*MS,T+100*MS));
 ASSERT_TRUE(a.renew(g.lease_id,g.epoch,1,T+100*MS,T+100*MS));
 EXPECT_FALSE(a.renew(g.lease_id,g.epoch,1,T+200*MS,T+200*MS));
 EXPECT_EQ(a.grant(T+200*MS,T+200*MS)->valid_until,T+2100*MS);
 EXPECT_FALSE(a.renew(g.lease_id,g.epoch,2,T+2100*MS,T+2100*MS));
}
TEST_F(AuthorityFixture, CancelAckAndTerminalAloneCannotRelease) {
 ASSERT_TRUE(acquire().accepted);a.submitted(T,T);a.stop("CANCELED",T+MS,T+MS);
 EXPECT_FALSE(a.release(false,true,T+2*MS,T+2*MS));
 EXPECT_FALSE(a.release(true,false,T+3*MS,T+3*MS));
 EXPECT_FALSE(a.acquire("other","next",T+4*MS,T+4*MS).accepted);
 EXPECT_TRUE(a.release(true,true,T+5*MS,T+5*MS));
 EXPECT_TRUE(a.acquire("other","next",T+6*MS,T+6*MS).accepted);
 EXPECT_TRUE(a.acquire("task","request",T+7*MS,T+7*MS).duplicate);
}
TEST_F(AuthorityFixture, PureReservationMayReleaseWithoutInventedMotionEvidence) {
 ASSERT_TRUE(acquire().accepted);a.stop("CANCELED",T+MS,T+MS);
 EXPECT_TRUE(a.release(false,false,T+2*MS,T+2*MS));
}
TEST_F(AuthorityFixture, WallExpiryAndClockResetDoNotRenew) {
 ASSERT_TRUE(acquire().accepted);EXPECT_FALSE(a.grant(T,T+2000*MS));
 ResourceAuthority b("boot2",joints(),[](const auto &){});ASSERT_TRUE(b.acquire("task","r",T,T).accepted);
 EXPECT_FALSE(b.grant(T-1,T+1));EXPECT_EQ(b.phase(),ResourcePhase::QUARANTINED);
}
TEST_F(AuthorityFixture, RestartRetainsUnresolvedResourcesAndRequestIdentity) {
 ASSERT_TRUE(acquire().accepted);a.submitted(T,T);auto saved=records.back();
 ResourceAuthority restarted("boot2",joints(),[](const auto &){},saved);
 EXPECT_EQ(restarted.phase(),ResourcePhase::QUARANTINED);
 EXPECT_FALSE(restarted.acquire("new","next",T,T).accepted);
 EXPECT_FALSE(restarted.grant(T,T));
}
TEST_F(AuthorityFixture, CommittedReleaseAndDuplicateSurviveRestart) {
 ASSERT_TRUE(acquire().accepted);a.stop("CANCELED",T,T);ASSERT_TRUE(a.release(false,false,T,T));
 ResourceAuthority restarted("boot2",joints(),[](const auto &){},records.back());
 EXPECT_TRUE(restarted.acquire("task","request",T+MS,T+MS).duplicate);
 EXPECT_TRUE(restarted.acquire("task","new",T+2*MS,T+2*MS).accepted);
}
TEST(ResourceAuthority, PersistenceFailureNeverGrants) {
 ResourceAuthority a("boot",joints(),[](const auto &){throw std::runtime_error("disk");});
 EXPECT_FALSE(a.acquire("task","r",T,T).accepted);EXPECT_FALSE(a.grant(T,T));
 EXPECT_EQ(a.phase(),ResourcePhase::QUARANTINED);
}
TEST(ResourceAuthority, FailedReleaseCommitKeepsResourcesQuarantined) {
 bool fail=false;ResourceAuthority a("boot",joints(),[&](const auto &){if(fail)throw std::runtime_error("disk");});
 ASSERT_TRUE(a.acquire("task","r",T,T).accepted);a.submitted(T,T);a.stop("CANCELED",T,T);fail=true;
 EXPECT_FALSE(a.release(true,true,T,T));EXPECT_EQ(a.phase(),ResourcePhase::QUARANTINED);
 EXPECT_FALSE(a.acquire("other","r",T,T).accepted);
}
TEST_F(AuthorityFixture, ContinueOperationPreservesLeaseAndDoesNotRenewIt) {
 ASSERT_TRUE(acquire().accepted);a.submitted(T,T);a.holding(T+MS,T+MS);
 const auto original=*a.grant(T+MS,T+MS);
 EXPECT_FALSE(a.continue_from_hold(original.lease_id,original.epoch,"transfer:op:2:PLACE",1,true,true,T+2*MS,T+2*MS));
 EXPECT_FALSE(a.continue_from_hold(original.lease_id,original.epoch,std::string(118,'a')+"_op_2_PLACE",1,true,true,T+2*MS,T+2*MS));
 EXPECT_EQ(a.phase(),ResourcePhase::HOLDING);
 const auto context=std::string(117,'a')+"_op_2_PLACE";
 ASSERT_TRUE(a.continue_from_hold(original.lease_id,original.epoch,context,1,true,true,T+2*MS,T+2*MS));
 EXPECT_EQ(a.phase(),ResourcePhase::RESERVED);
 const auto continued=*a.grant(T+2*MS,T+2*MS);
 EXPECT_EQ(continued.owner_id,original.owner_id);EXPECT_EQ(continued.lease_id,original.lease_id);
 EXPECT_EQ(continued.epoch,original.epoch);EXPECT_EQ(continued.valid_until,original.valid_until);
 EXPECT_EQ(records.back()["event"],"operation_continuation_committed");
 EXPECT_TRUE(records.back()["side_effects"].get<bool>());
 a.submitted(T+3*MS,T+3*MS);a.holding(T+4*MS,T+4*MS);
 EXPECT_FALSE(a.continue_from_hold(original.lease_id,original.epoch,context,1,true,true,T+5*MS,T+5*MS));
 EXPECT_EQ(a.phase(),ResourcePhase::HOLDING);
}
TEST_F(AuthorityFixture, ContinueRequiresCurrentIdentityHoldAndBothEvidenceBoundaries) {
 ASSERT_TRUE(acquire().accepted);const auto g=*a.grant(T,T);
 EXPECT_FALSE(a.continue_from_hold(g.lease_id,g.epoch,"place",1,true,true,T,T));
 a.submitted(T,T);a.holding(T,T);
 EXPECT_FALSE(a.continue_from_hold("wrong",g.epoch,"place",1,true,true,T,T));
 EXPECT_FALSE(a.continue_from_hold(g.lease_id,"wrong","place",1,true,true,T,T));
 EXPECT_FALSE(a.continue_from_hold(g.lease_id,g.epoch,"place",1,false,true,T,T));
 EXPECT_FALSE(a.continue_from_hold(g.lease_id,g.epoch,"place",1,true,false,T,T));
 EXPECT_FALSE(a.continue_from_hold(g.lease_id,g.epoch,"place",2,true,true,T,T));
 a.stop("CANCELED",T,T);
 EXPECT_FALSE(a.continue_from_hold(g.lease_id,g.epoch,"place",1,true,true,T,T));
}
TEST_F(AuthorityFixture, SuccessfulCompletionCommitsThenReleasesWithoutCancellation) {
 ASSERT_TRUE(acquire().accepted);const auto g=*a.grant(T,T);a.submitted(T,T);a.holding(T,T);
 EXPECT_FALSE(a.complete(g.lease_id,g.epoch,false,true,true,T,T));
 EXPECT_FALSE(a.complete(g.lease_id,g.epoch,true,false,true,T,T));
 EXPECT_FALSE(a.complete(g.lease_id,g.epoch,true,true,false,T,T));
 EXPECT_FALSE(a.complete("wrong",g.epoch,true,true,true,T,T));
 ASSERT_TRUE(a.complete(g.lease_id,g.epoch,true,true,true,T,T));
 EXPECT_EQ(records[records.size()-2]["event"],"result_success_release_pending");
 EXPECT_EQ(records[records.size()-2]["phase"],int(ResourcePhase::HOLDING));
 EXPECT_EQ(records.back()["event"],"resource_handoff_committed");
 EXPECT_EQ(a.phase(),ResourcePhase::IDLE);
 for(const auto &record:records)EXPECT_NE(record["event"],"stop_requested");
 EXPECT_FALSE(a.complete(g.lease_id,g.epoch,true,true,true,T,T));
 ResourceAuthority restarted("next_boot",joints(),[](const auto &){},records.back());
 EXPECT_TRUE(restarted.acquire("task","request",T,T).duplicate);
 EXPECT_TRUE(restarted.acquire("other","request",T,T).accepted);
}
TEST_F(AuthorityFixture, CancellationPreventsNormalCompletion) {
 ASSERT_TRUE(acquire().accepted);const auto g=*a.grant(T,T);a.submitted(T,T);a.holding(T,T);
 a.stop("CANCELED",T,T);
 EXPECT_FALSE(a.complete(g.lease_id,g.epoch,true,true,true,T,T));
 EXPECT_EQ(a.phase(),ResourcePhase::STOPPING);
}
TEST(ResourceAuthority, ContinuationPersistenceFailureKeepsOwnershipUnresolved) {
 ResourceAuthority a("boot",joints(),[](const auto &r){
   if(r.at("event")=="operation_continuation_committed")throw std::runtime_error("disk");
 });
 ASSERT_TRUE(a.acquire("task","r",T,T).accepted);const auto g=*a.grant(T,T);a.submitted(T,T);a.holding(T,T);
 EXPECT_FALSE(a.continue_from_hold(g.lease_id,g.epoch,"place",1,true,true,T,T));
 EXPECT_EQ(a.phase(),ResourcePhase::QUARANTINED);EXPECT_FALSE(a.grant(T,T));
}
TEST(ResourceAuthority, NormalCompletionCommitFailuresCannotGrantNewOwner) {
 for(const auto *failed_event:{"result_success_release_pending","resource_handoff_committed"}) {
  std::vector<nlohmann::json> saved;
  ResourceAuthority a("boot",joints(),[&](const auto &r){
    if(r.at("event")==failed_event)throw std::runtime_error("disk");
    saved.push_back(r);
  });
  ASSERT_TRUE(a.acquire("task","r",T,T).accepted);const auto g=*a.grant(T,T);a.submitted(T,T);a.holding(T,T);
  EXPECT_FALSE(a.complete(g.lease_id,g.epoch,true,true,true,T,T));
  EXPECT_EQ(a.phase(),ResourcePhase::QUARANTINED);
  EXPECT_FALSE(a.acquire("other","r",T,T).accepted);
  ResourceAuthority restarted("next_boot",joints(),[](const auto &){},saved.back());
  EXPECT_EQ(restarted.phase(),ResourcePhase::QUARANTINED);
 }
}
}
