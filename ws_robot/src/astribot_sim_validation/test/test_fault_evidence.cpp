#include "astribot_sim_validation/fault_evidence.hpp"
#include <gtest/gtest.h>
using namespace astribot_sim_validation;
using Json=nlohmann::json;
TEST(FaultEvidence, RequiresAttachedStageAndMeasuredMotion) {
  Json l={{"stage","TRANSPORT"},{"object",{{"state","ATTACHED"},{"attachment","tcp"}}}};
  EXPECT_TRUE(faultStageReady(l,"TRANSPORT",.03,.02));
  EXPECT_FALSE(faultStageReady(l,"TRANSPORT",0.,.02));
  EXPECT_FALSE(faultStageReady(l,"TRANSPORT_POSTURE",.03,.02));
  l["object"]["state"]="ATTACH_PENDING";
  EXPECT_FALSE(faultStageReady(l,"TRANSPORT",.03,.02));
}
TEST(FaultEvidence, AckCannotReplaceTerminalAndPayloadFacts) {
  Json object={{"object_id","box"},{"state","ATTACHED"},{"version",3},{"attachment","tcp"}};
  Json l={{"stage","CANCELED"},{"reason","USER_CANCEL"},{"stop_error",""},{"object",object}};
  EXPECT_TRUE(canceledWithPayload(l,object));
  l["stage"]="CANCELING";EXPECT_FALSE(canceledWithPayload(l,object));
  l["stage"]="CANCELED";l["stop_error"]="TIMEOUT";EXPECT_FALSE(canceledWithPayload(l,object));
  l["stop_error"]="";l["object"]["version"]=4;EXPECT_FALSE(canceledWithPayload(l,object));
  l["object"]=object;l["object"]["attachment"]="";EXPECT_FALSE(canceledWithPayload(l,object));
}
TEST(FaultEvidence, FrozenAndGappedStreamsCannotProveStop) {
  StabilityWindow w;
  w.observe(1000000000,1.,true);w.observe(1250000000,1.25,true);w.observe(1500000000,1.5,true);
  EXPECT_TRUE(w.ready(1.5));
  w.observe(1500000000,2.1,true);EXPECT_FALSE(w.ready(2.1));
  w.observe(2200000000,2.2,true);EXPECT_FALSE(w.ready(2.2));
  w.observe(2300000000,2.3,false);EXPECT_FALSE(w.ready(2.3));
  w.observe(2100000000,2.4,true);EXPECT_FALSE(w.ready(2.4));
}

TEST(FaultEvidence, RefreshObservationTimeAfterSpinningCallbacks) {
  StabilityWindow w;
  w.observe(1000000000,1.,true);w.observe(1250000000,1.25,true);
  const double before_service=1.49;
  w.observe(1500000000,1.5,true); // Receipt during a synchronous service spin.
  EXPECT_FALSE(w.ready(before_service));
  EXPECT_TRUE(w.ready(1.51));
  EXPECT_FALSE(w.ready(2.01));
}
