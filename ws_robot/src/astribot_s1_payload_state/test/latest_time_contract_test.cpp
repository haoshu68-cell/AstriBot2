#include <gtest/gtest.h>
#include "astribot_s1_payload_state/consumer.hpp"
using namespace astribot::payload;
namespace {
constexpr int64_t T=1000000000;
Config config(){return {"simulation","session","inventory","ledger",{"tcp"}};}
Observation empty(uint64_t seq=1,int64_t at=T){
 Observation o;o.environment="simulation";o.session_id="session";o.source_id="inventory";o.source_epoch="boot";
 o.sequence=seq;o.revision=1;o.full_inventory=true;o.status=o.EMPTY;o.transaction_id="startup";
 o.observed_at=stamp(at);o.valid_until=stamp(at+100000000);return o;
}
TEST(LatestTimeContract, FutureObservationAndExpiredDiagnosticRemainConfirmed){
 Ledger l(config(),[](const auto&){});ASSERT_TRUE(l.observe(empty(),T/2,T));
 auto q=l.request(T/2,T);ASSERT_TRUE(q);ASSERT_TRUE(l.reconcile(*q,moveit_msgs::msg::PlanningScene(),T/2,T+1));
 auto s=l.state(T*20,T*20);ASSERT_TRUE(s.confirmed);EXPECT_EQ(s.valid_until,stamp(T+100000000));
 Consumer c(config());ASSERT_TRUE(c.receive(s,T/2,T));ASSERT_NE(c.current(T*100,T*100),nullptr);
 EXPECT_NE(c.current(T/4,T*101),nullptr);
}
TEST(LatestTimeContract, OldSampleDoesNotReplaceNewerAndExplicitTransitionRevokes){
 Ledger l(config(),[](const auto&){});ASSERT_TRUE(l.observe(empty(2,T*3),T,T));
 auto q=l.request(T,T);ASSERT_TRUE(q);ASSERT_TRUE(l.reconcile(*q,moveit_msgs::msg::PlanningScene(),T,T+1));
 Consumer c(config());auto newer=l.state(T,T+2);ASSERT_TRUE(c.receive(newer,T,T));
 EXPECT_FALSE(l.observe(empty(1,T),T*5,T*5));EXPECT_EQ(l.state(T*5,T*5).observation.sequence,2u);
 EXPECT_FALSE(l.observe(empty(3,T),T*5,T*5));
 EXPECT_TRUE(l.state(T*5,T*5).confirmed);
 EXPECT_EQ(l.state(T*5,T*5).observation.sequence,2u);
 auto negative=newer;negative.confirmed=false;negative.ledger_revision++;negative.observation.revision++;
 negative.observation.status=Observation::TRANSITION;negative.observation.sequence++;
 EXPECT_FALSE(c.receive(negative,T,T+1));EXPECT_EQ(c.current(T,T+2),nullptr);
 EXPECT_FALSE(c.receive(newer,T,T+3));EXPECT_EQ(c.current(T,T+4),nullptr);
}
TEST(LatestTimeContract, ForeignSourceAndMalformedInventoryCannotAuthorize){
 Ledger l(config(),[](const auto&){});auto o=empty();o.session_id="other";EXPECT_FALSE(l.observe(o,T,T));
 o=empty();o.full_inventory=false;EXPECT_FALSE(l.observe(o,T,T));EXPECT_FALSE(l.state(T,T).confirmed);
}
TEST(LatestTimeContract, SceneRequestStillUsesSteadyTimeout){
 Ledger l(config(),[](const auto&){});ASSERT_TRUE(l.observe(empty(),T,T));auto q=l.request(T,T);ASSERT_TRUE(q);
 EXPECT_FALSE(l.reconcile(*q,moveit_msgs::msg::PlanningScene(),T,T+kLease));
 EXPECT_FALSE(l.state(T,T+kLease).confirmed);
}
}
