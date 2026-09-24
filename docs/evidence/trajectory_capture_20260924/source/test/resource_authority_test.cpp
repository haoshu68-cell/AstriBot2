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
}
