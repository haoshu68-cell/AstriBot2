#include <gtest/gtest.h>
#include "astribot_s1_transport_native/child_actions.hpp"
using astribot::transport::ChildActions;
TEST(ChildActions, GoalIdentityRetainsEveryByteIncludingLeadingZeros){
 std::array<uint8_t,16> first{},second{};first[0]=1;first[1]=0x23;second[0]=0x12;second[1]=3;
 auto a=astribot::transport::canonical_goal_id(first),b=astribot::transport::canonical_goal_id(second);
 EXPECT_EQ(a.size(),32U);EXPECT_EQ(a.substr(0,4),"0123");EXPECT_EQ(b.substr(0,4),"1203");EXPECT_NE(a,b);
}
TEST(ChildActions, UnknownIsNotATerminalFailure){
 ChildActions c({"arm"});c.submitted("arm");c.response("arm","current");
 EXPECT_THROW(c.result("arm","current",false,false),std::logic_error);
 EXPECT_FALSE(c.all_terminal());EXPECT_FALSE(c.all_successful());
}
TEST(ChildActions, PendingAcceptanceCannotReleaseOrInventCompletion) {
 ChildActions c({"left","right"});c.submitted("left");
 EXPECT_FALSE(c.all_terminal());EXPECT_FALSE(c.all_successful());EXPECT_THROW(c.proof(),std::logic_error);
 c.response("left","uuid");EXPECT_FALSE(c.all_terminal());
 c.result("left","uuid",true);EXPECT_TRUE(c.all_terminal());EXPECT_FALSE(c.all_successful());
}
TEST(ChildActions, RejectedAndCanceledChildrenAreTerminalButNotSuccessful) {
 ChildActions c({"left","right"});c.submitted("left");c.submitted("right");
 c.response("left","");c.response("right","uuid");c.result("right","uuid",false);
 EXPECT_TRUE(c.all_terminal());EXPECT_TRUE(c.failed());EXPECT_FALSE(c.all_successful());
}
TEST(ChildActions, ResultsAreBoundToCurrentGoalsAndExactlyOnce) {
 ChildActions c({"left"});c.submitted("left");c.response("left","new");
 EXPECT_THROW(c.result("left","old",true),std::logic_error);EXPECT_FALSE(c.all_terminal());
 c.result("left","new",true);EXPECT_TRUE(c.all_successful());EXPECT_EQ(c.proof(),"left:new;");
 EXPECT_THROW(c.result("left","new",true),std::logic_error);
 EXPECT_THROW(c.submitted("left"),std::logic_error);
}
