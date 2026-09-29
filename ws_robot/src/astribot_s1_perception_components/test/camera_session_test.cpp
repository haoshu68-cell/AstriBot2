#include <gtest/gtest.h>
#include "astribot_s1_autonomy/camera_session.hpp"
using astribot::vision::CameraSession;
TEST(CameraSession, AcquiresAndRejectsOtherOwner) {
 CameraSession s("boot");auto a=s.request(0,"owner","run","q1","",.5,100,100,100);ASSERT_TRUE(a);auto token=s.token();
 EXPECT_FALSE(s.request(0,"other","run2","q2","",.5,101,101,101));EXPECT_EQ(s.token(),token);
}
TEST(CameraSession, DuplicateAcquireCannotRenewLease) {
 CameraSession s("boot");ASSERT_TRUE(s.request(0,"owner","run","q1","",.5,100,100,100));auto end=s.deadline();
 EXPECT_TRUE(s.request(0,"owner","run","q1","",.5,101,101,101));EXPECT_EQ(s.deadline(),end);
}
TEST(CameraSession, ReleaseAndExpiredTokensCannotReactivate) {
 CameraSession s("boot");ASSERT_TRUE(s.request(0,"o","e","a","",.5,100,100,100));auto token=s.token();
 ASSERT_TRUE(s.request(2,"o","e","b",token,0.,101,101,101));EXPECT_FALSE(s.active());
 EXPECT_FALSE(s.request(1,"o","e","c",token,.5,102,102,102));
 EXPECT_FALSE(s.request(0,"o","e","a","",.5,102,102,102));
}
TEST(CameraSession, WallExpiryAndClockRollbackInvalidate) {
 CameraSession s("boot");ASSERT_TRUE(s.request(0,"o","e","a","",.5,100,100,100));s.tick(100,500000101);EXPECT_FALSE(s.active());
 ASSERT_TRUE(s.request(0,"o","e","b","",.5,200,200,500000102));auto token=s.token();s.tick(150,500000103);EXPECT_FALSE(s.active());
 EXPECT_FALSE(s.request(1,"o","e","c",token,.5,151,151,500000104));
}
TEST(CameraSession, RejectsMissingIdentityInvalidBudgetAndOldRequest) {
 CameraSession s("boot");EXPECT_FALSE(s.request(0,"","e","q","",.5,100,100,100));
 EXPECT_FALSE(s.request(0,"o","e","q","",3.,100,100,100));
 EXPECT_FALSE(s.request(0,"o","e","q","",NAN,100,100,100));
 EXPECT_FALSE(s.request(0,"o","e","q","",.5,1,1000000000,100));
}
TEST(CameraSession, CorrectRenewExtendsAndDifferentBootHasDifferentToken) {
 CameraSession s("boot"),other("other");ASSERT_TRUE(s.request(0,"o","e","a","",.5,100,100,100));auto token=s.token();
 EXPECT_TRUE(s.request(1,"o","e","b",token,.5,101,101,101));EXPECT_EQ(s.deadline(),500000101);
 ASSERT_TRUE(other.request(0,"o","e","a","",.5,100,100,100));EXPECT_NE(s.token(),other.token());
}
TEST(CameraSession, ClockAheadRequestCanRetryWithItsOriginalIdentity) {
 CameraSession s("boot");
 EXPECT_FALSE(s.request(0,"o","e","a","",.5,1001000000,1000000000,1000000000));
 EXPECT_EQ(s.reason(),"REQUEST_CLOCK_AHEAD");EXPECT_FALSE(s.active());
 EXPECT_TRUE(s.request(0,"o","e","a","",.5,1001000000,1001000000,1001000000));
 const auto deadline=s.deadline();
 EXPECT_TRUE(s.request(0,"o","e","a","",.5,1001000000,1002000000,1002000000));
 EXPECT_EQ(s.deadline(),deadline);
}
