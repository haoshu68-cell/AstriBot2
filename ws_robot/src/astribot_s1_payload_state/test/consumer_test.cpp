#include <gtest/gtest.h>
#include "astribot_s1_payload_state/consumer.hpp"
using namespace astribot::payload;
namespace {
constexpr int64_t T=1000000000;
Config cfg() {return {"simulation","session","inventory","ledger",{"tcp"}};}
State state(int64_t at=T) {
  Observation o;o.environment="simulation";o.session_id="session";o.source_id="inventory";o.source_epoch="boot";
  o.sequence=o.revision=1;o.full_inventory=true;o.status=o.EMPTY;o.transaction_id="startup";
  o.observed_at=stamp(at);o.valid_until=stamp(at+kLease);
  Ledger l(cfg(),[](const auto &){});if(!l.observe(o,at,at))throw std::runtime_error("fixture observation");
  auto ticket=l.request(at,at);if(!ticket || !l.reconcile(*ticket,moveit_msgs::msg::PlanningScene(),at,at))throw std::runtime_error("fixture scene");return l.state(at,at);
}
TEST(Consumer, ConfirmedEmptyIsAvailableAndStrictlyExpires) {
  Consumer c(cfg());auto s=state();ASSERT_TRUE(c.receive(s,T,T));ASSERT_NE(c.current(T,T),nullptr);
  EXPECT_NE(c.current(T+kLease-1,T+kLease-1),nullptr);EXPECT_EQ(c.current(T+kLease,T+kLease),nullptr);
}
TEST(Consumer, HeartbeatsCannotRenewSameCaptureWhenRosTimeFrozen) {
  Consumer c(cfg());auto s=state();ASSERT_TRUE(c.receive(s,T,T));
  ASSERT_TRUE(c.receive(s,T,T+kLease/2));EXPECT_EQ(c.current(T,T+kLease),nullptr);
}
TEST(Consumer, InvalidStatusRevokesAndReplayCannotRecover) {
  Consumer c(cfg());auto s=state();ASSERT_TRUE(c.receive(s,T,T));const auto generation=c.generation();
  auto bad=s;bad.confirmed=false;ASSERT_FALSE(c.receive(bad,T+1,T+1));EXPECT_GT(c.generation(),generation);
  EXPECT_FALSE(c.receive(s,T+2,T+2));EXPECT_EQ(c.current(T+2,T+2),nullptr);
  auto fresh=state(T+3);EXPECT_TRUE(c.receive(fresh,T+3,T+3));
}
TEST(Consumer, FutureSameIdentityDoesNotRevokeOrExtendValidCache) {
  Consumer c(cfg());auto s=state();ASSERT_TRUE(c.receive(s,T,T));
  auto future=s;future.published_at=stamp(T+1);EXPECT_FALSE(c.receive(future,T,T+100));EXPECT_NE(c.current(T,T+100),nullptr);
  future.confirmed=false;EXPECT_FALSE(c.receive(future,T,T+101));EXPECT_EQ(c.current(T,T+101),nullptr);
}
TEST(Consumer, SourceWrongOrGeometryDigestWrongCannotAuthorize) {
  Consumer c(cfg());auto s=state();s.observation.session_id="foreign";EXPECT_FALSE(c.receive(s,T,T));
  s=state();s.geometry_digest="forged";EXPECT_FALSE(c.receive(s,T,T));EXPECT_EQ(c.current(T,T),nullptr);
}
TEST(Consumer, LedgerRestartRevokesOldAndRetiredEpochCannotReturn) {
  Consumer c(cfg());auto s=state();ASSERT_TRUE(c.receive(s,T,T));
  auto fresh=state(T+1);fresh.ledger_epoch="new";fresh.attachment_revision=digest(nlohmann::json::array({"session","new",1}).dump());
  ASSERT_TRUE(c.receive(fresh,T+1,T+1));EXPECT_FALSE(c.receive(s,T+2,T+2));
  ASSERT_NE(c.current(T+2,T+2),nullptr);EXPECT_EQ(c.current(T+2,T+2)->ledger_epoch,"new");
}
TEST(Consumer, ClockRollbackCannotResurrectOldState) {
  Consumer c(cfg());auto s=state();ASSERT_TRUE(c.receive(s,T,T));EXPECT_EQ(c.current(T-1,T+1),nullptr);
  EXPECT_FALSE(c.receive(s,T,T+2));
}
TEST(Consumer, EmptyArrayWithoutFullConfirmationIsNotEmptyProof) {
  Consumer c(cfg());auto s=state();s.observation.full_inventory=false;EXPECT_FALSE(c.receive(s,T,T));
  EXPECT_EQ(c.current(T,T),nullptr);
}

TEST(Consumer, RestartUnknownRevokesAndRetiresPreviousLedger) {
  Consumer c(cfg());auto old=state();ASSERT_TRUE(c.receive(old,T,T));
  auto unknown=old;unknown.confirmed=false;unknown.ledger_epoch="restarted";unknown.ledger_revision=0;
  unknown.observation.status=unknown.observation.UNKNOWN;unknown.observation.source_epoch="";
  EXPECT_FALSE(c.receive(unknown,T+1,T+1));EXPECT_EQ(c.current(T+1,T+1),nullptr);
  old.observation.observed_at=stamp(T+2);old.observation.valid_until=stamp(T+2+kLease);
  old.published_at=stamp(T+2);old.valid_until=stamp(T+2+kLease);old.observation.sequence=2;
  EXPECT_FALSE(c.receive(old,T+2,T+2));EXPECT_EQ(c.current(T+2,T+2),nullptr);
}
TEST(Consumer, SourceEpochCannotChangeInsideSameLedgerRevision) {
  Consumer c(cfg());auto s=state();ASSERT_TRUE(c.receive(s,T,T));
  s.observation.source_epoch="other_boot";EXPECT_FALSE(c.receive(s,T+1,T+1));EXPECT_EQ(c.current(T+1,T+1),nullptr);
}

TEST(Consumer, NewerTransitionCannotBeOverwrittenByDelayedOldEmpty) {
  Consumer c(cfg());auto old=state();ASSERT_TRUE(c.receive(old,T,T));
  auto delayed=old;delayed.observation.sequence=2;delayed.observation.observed_at=stamp(T+50);
  delayed.observation.valid_until=stamp(T+50+kLease);delayed.published_at=stamp(T+50);delayed.valid_until=stamp(T+50+kLease);
  auto pending=delayed;pending.confirmed=false;pending.ledger_revision=2;
  pending.observation.status=pending.observation.TRANSITION;pending.observation.revision=2;pending.observation.observed_at=stamp(T+100);
  pending.attachment_revision=digest(nlohmann::json::array({"session","ledger",2}).dump());
  EXPECT_FALSE(c.receive(pending,T+100,T+100));EXPECT_FALSE(c.receive(delayed,T+101,T+101));
  EXPECT_EQ(c.current(T+101,T+101),nullptr);
}

TEST(Consumer, ExpiredOutOfOrderPacketDoesNotRevokeFreshState) {
  Consumer c(cfg());auto old=state();ASSERT_TRUE(c.receive(old,T,T));
  auto fresh=state(T+400000000);fresh.observation.sequence=2;ASSERT_TRUE(c.receive(fresh,T+400000000,T+400000000));
  EXPECT_FALSE(c.receive(old,T+600000000,T+600000000));EXPECT_NE(c.current(T+600000000,T+600000000),nullptr);
}

TEST(Consumer, RecoveryFloorDoesNotChaseValidDelayedCapturesForever) {
  Consumer c(cfg());auto original=state();ASSERT_TRUE(c.receive(original,T,T));
  auto negative=original;negative.confirmed=false;
  ASSERT_FALSE(c.receive(negative,T+100000000,T+100000000));
  auto before_fault=state(T+90000000);before_fault.observation.sequence=2;
  // A valid ledger confirmation may still contain a pre-fault capture. Reject
  // it without turning its arrival into a new physical fault barrier.
  EXPECT_FALSE(c.receive(before_fault,T+150000000,T+150000000));
  EXPECT_EQ(c.current(T+150000000,T+150000000),nullptr);
  auto after_fault=state(T+110000000);after_fault.observation.sequence=3;
  EXPECT_TRUE(c.receive(after_fault,T+160000000,T+160000000));
  EXPECT_NE(c.current(T+160000000,T+160000000),nullptr);
}
}
