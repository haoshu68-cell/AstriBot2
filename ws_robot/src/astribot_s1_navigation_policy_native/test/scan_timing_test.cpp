#include <gtest/gtest.h>
#if __has_include("astribot_s1_navigation_policy_native/scan_timing.hpp")
#include "astribot_s1_navigation_policy_native/scan_timing.hpp"
using astribot::navigation::ScanTiming;

TEST(ScanTiming, CorrelatesCaptureReceiveTfSelectionAndFinish) {
  ScanTiming trace;
  const auto id=trace.receive(100,140,500,"scan",0);
  trace.tf_check(id,0,-1,145,510);
  trace.tf_check(id,1,1,160,540);
  trace.select(id,165,550);
  trace.finish(id,true,200,620,"");
  auto snapshot=trace.snapshot();
  ASSERT_TRUE(snapshot.successful);
  const auto& r=*snapshot.successful;
  EXPECT_EQ(r.sequence,id);EXPECT_EQ(r.capture_ros_ns,100);
  EXPECT_EQ(r.receive.ros_ns,140);EXPECT_EQ(r.receive.steady_ns,500);
  ASSERT_TRUE(r.first_tf_ready);EXPECT_EQ(r.first_tf_ready->steady_ns,540);
  ASSERT_TRUE(r.selected);EXPECT_EQ(r.selected->steady_ns,550);
  ASSERT_TRUE(r.finished);EXPECT_EQ(r.finished->ros_ns,200);
  EXPECT_EQ(r.tf_checks,2u);EXPECT_EQ(snapshot.waiting->tracking_tf,0);
}

TEST(ScanTiming, DuplicateAndOutOfOrderCapturesKeepDistinctArrivalIdentity) {
  ScanTiming trace;
  auto a=trace.receive(300,320,100,"scan",0);
  auto b=trace.receive(300,330,110,"scan",0);
  auto c=trace.receive(200,340,120,"scan",0);
  EXPECT_LT(a,b);EXPECT_LT(b,c);
  trace.select(c,350,130);trace.finish(c,true,360,140,"");
  ASSERT_TRUE(trace.snapshot().successful);
  EXPECT_EQ(trace.snapshot().successful->capture_ros_ns,200);
}

TEST(ScanTiming, ZeroRosTimeIsARealMilestoneAndClocksAreNeverMixed) {
  ScanTiming trace;auto id=trace.receive(0,0,1000,"scan",0);
  trace.tf_check(id,1,1,0,1020);trace.select(id,0,1030);
  trace.finish(id,true,0,1040,"");auto r=trace.snapshot().successful;
  ASSERT_TRUE(r);ASSERT_TRUE(r->selected);ASSERT_TRUE(r->finished);
  EXPECT_EQ(r->finished->ros_ns,0);EXPECT_EQ(r->finished->steady_ns,1040);
}

TEST(ScanTiming, FailedProcessingDoesNotPretendToRefreshSuccessfulInput) {
  ScanTiming trace;auto a=trace.receive(100,110,1000,"scan",0);
  trace.select(a,120,1010);trace.finish(a,true,150,1050,"");
  auto b=trace.receive(200,210,1100,"scan",0);
  trace.select(b,220,1110);trace.finish(b,false,230,1120,"TF_LOOKUP_FAILED");
  auto s=trace.snapshot();EXPECT_EQ(s.successful->sequence,a);
  EXPECT_EQ(s.finished->sequence,b);EXPECT_FALSE(s.finished->processing_success);
  EXPECT_EQ(s.finished->reason,"TF_LOOKUP_FAILED");
}

TEST(ScanTiming, CapacityIsBoundedAndLostUpdatesAreExplicit) {
  ScanTiming trace;auto old=trace.receive(0,0,0,"scan",0);
  for(int i=1;i<100;++i)trace.receive(i,i,i,"scan",0);
  trace.select(old,200,200);
  auto s=trace.snapshot();EXPECT_EQ(s.retained,ScanTiming::capacity);
  EXPECT_EQ(s.evictions,100-ScanTiming::capacity);EXPECT_EQ(s.missing_updates,1u);
  EXPECT_FALSE(s.selected);
}

TEST(ScanTiming, EpochResetInvalidatesPreviousSelectedAndSuccessfulRecords) {
  ScanTiming trace;auto old=trace.receive(100,100,1000,"scan",0);
  trace.select(old,110,1010);trace.finish(old,true,120,1020,"");
  trace.reset(1);auto s=trace.snapshot();
  EXPECT_EQ(s.clock_epoch,1);EXPECT_FALSE(s.successful);EXPECT_FALSE(s.received);
  trace.finish(old,true,0,1030,"");EXPECT_FALSE(trace.snapshot().successful);
  auto next=trace.receive(0,0,1040,"scan",1);EXPECT_GT(next,old);
}

TEST(ScanTiming, DropReasonsAndFirstTfReadinessAreNotOverwritten) {
  ScanTiming trace;auto a=trace.receive(1,2,3,"scan",0);
  trace.tf_check(a,1,1,4,5);trace.tf_check(a,1,1,6,7);
  trace.drop(a,"superseded",8,9);
  auto s=trace.snapshot();ASSERT_TRUE(s.dropped);
  EXPECT_EQ(s.dropped->first_tf_ready->ros_ns,4);
  EXPECT_EQ(s.dropped->reason,"superseded");EXPECT_EQ(s.drop_counts.at("superseded"),1u);
}

TEST(ScanTiming, DiagnosticTextStorageIsBounded) {
  ScanTiming trace;auto id=trace.receive(1,2,3,std::string(10000,'x'),0);
  trace.finish(id,false,4,5,std::string(10000,'e'));
  EXPECT_LE(trace.snapshot().received->frame_id.size(),256u);
  EXPECT_LE(trace.snapshot().finished->reason.size(),256u);
}

TEST(ScanTiming, TextLimitPreservesCompleteUtf8CodePoints) {
  const std::pair<std::string,std::string> cases[]={
    {std::string(254,'a')+u8"é",std::string(254,'a')+u8"é"},
    {std::string(255,'a')+u8"é",std::string(255,'a')},
    {std::string(253,'a')+u8"帧",std::string(253,'a')+u8"帧"},
    {std::string(254,'a')+u8"帧",std::string(254,'a')},
    {std::string(252,'a')+u8"😀",std::string(252,'a')+u8"😀"},
    {std::string(253,'a')+u8"😀",std::string(253,'a')}};
  for(const auto& [input,expected]:cases) {
    ScanTiming trace;auto id=trace.receive(1,2,3,input,0);
    trace.finish(id,false,4,5,input);const auto snapshot=trace.snapshot();
    EXPECT_EQ(snapshot.received->frame_id,expected);
    EXPECT_EQ(snapshot.finished->reason,expected);
  }
}

TEST(ScanTiming, MalformedUtf8IsReplacedWithoutLosingValidText) {
  const std::pair<std::string,std::string> cases[]={
    {"prefix\x80" "suffix","prefix?suffix"},
    {"\xc0\xaf","??"}, {"\xe0\x80\xaf","???"},
    {"\xed\xa0\x80","???"}, {"\xf4\x90\x80\x80","????"},
    {"\xff\xfe","??"}, {"\xe5\xb8","??"}, {"\xc2" "A","?A"},
    {u8"é帧😀\U0010ffff",u8"é帧😀\U0010ffff"},
    {std::string(10000,static_cast<char>(0xff)),std::string(256,'?')}};
  for(const auto& [input,expected]:cases) {
    ScanTiming trace;auto id=trace.receive(1,2,3,input,0);
    trace.finish(id,false,4,5,input);const auto snapshot=trace.snapshot();
    EXPECT_EQ(snapshot.received->frame_id,expected);
    EXPECT_EQ(snapshot.finished->reason,expected);
  }
}

TEST(ScanTiming, LateOldEpochReceiveCannotRepopulateCurrentDiagnostics) {
  ScanTiming trace;trace.reset(1);
  const auto old=trace.receive(100,101,1000,"old",0);
  trace.select(old,102,1001);trace.finish(old,true,103,1002,"");
  auto snapshot=trace.snapshot();
  EXPECT_EQ(snapshot.clock_epoch,1);EXPECT_EQ(snapshot.retained,0u);
  EXPECT_FALSE(snapshot.received);EXPECT_FALSE(snapshot.selected);
  EXPECT_FALSE(snapshot.finished);EXPECT_FALSE(snapshot.successful);
  EXPECT_EQ(snapshot.drop_counts["obsolete_epoch"],1u);
  EXPECT_EQ(snapshot.missing_updates,2u);
  const auto current=trace.receive(1,2,1003,"current",1);
  EXPECT_GT(current,old);trace.finish(current,true,3,1004,"");
  trace.finish(old,true,104,1005,"");snapshot=trace.snapshot();
  ASSERT_TRUE(snapshot.successful);EXPECT_EQ(snapshot.successful->sequence,current);
  EXPECT_EQ(snapshot.successful->clock_epoch,1);
}

TEST(ScanTiming, DeferralDoesNotPretendToPerformTfOrFinishAFrame) {
  ScanTiming trace;auto id=trace.receive(10,2,3,"scan",0);
  trace.defer(id,"future",4,5);auto r=trace.snapshot().deferred;
  ASSERT_TRUE(r);EXPECT_EQ(r->reason,"future");
  EXPECT_FALSE(r->last_tf_check);EXPECT_FALSE(r->finished);
  ASSERT_TRUE(r->deferred_at);EXPECT_EQ(r->deferred_at->ros_ns,4);
  for(int i=0;i<100;++i)trace.defer(id,std::to_string(i),4,5);
  EXPECT_EQ(trace.snapshot().defer_counts.size(),2u);
}
#else
TEST(ScanTiming, RuntimeTraceIsImplementedInCpp) {
  FAIL() << "Bounded C++ scan timing recorder is missing";
}
#endif
