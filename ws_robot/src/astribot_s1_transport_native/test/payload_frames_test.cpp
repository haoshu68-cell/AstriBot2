#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/clock.hpp>
#include "astribot_s1_transport_native/payload_frames.hpp"
using namespace astribot::transport;
namespace {
const char *xml=R"(<robot name="test"><link name="root"/><link name="base"/><link name="wrist"/><link name="tcp"/>
<joint name="root_base" type="fixed"><parent link="root"/><child link="base"/><origin xyz="0 0 .1"/></joint>
<joint name="arm" type="continuous"><parent link="base"/><child link="wrist"/><axis xyz="0 0 1"/></joint>
<joint name="tool" type="fixed"><parent link="wrist"/><child link="tcp"/><origin xyz="0 -.15 0"/></joint></robot>)";
ignition::msgs::Pose_V poses(int64_t stamp,double z) {
 ignition::msgs::Pose_V value;value.mutable_header()->mutable_stamp()->set_sec(stamp/1000000000);
 value.mutable_header()->mutable_stamp()->set_nsec(stamp%1000000000);
 auto model=value.add_pose();model->set_name("robot");model->set_id(136);model->mutable_position()->set_z(z);model->mutable_orientation()->set_w(1.);
 auto local=value.add_pose();local->set_name("base");local->set_id(137);local->mutable_orientation()->set_w(1.);return value;
}
}
TEST(PayloadFrames, UsesTopLevelModelAndActualFixedChainWithInterpolation) {
 Eigen::Isometry3d model_root=Eigen::Isometry3d::Identity();model_root.translation().x()=.2;
 PayloadFrames frames(xml,"base","tcp","robot",136,model_root);
 EXPECT_EQ(frames.known_links(),(std::set<std::string>{"root","base","wrist","tcp"}));
 EXPECT_EQ(frames.parent_link(),"wrist");EXPECT_DOUBLE_EQ(frames.parent_from_tcp().translation().y(),-.15);
 frames.observe(poses(1000000000,.13),{1100000000,2000000000});
 frames.observe(poses(1100000000,.33),{1100000000,2000000001});
 const auto result=frames.world_from_base(1050000000,1100000000,2050000000);
 ASSERT_TRUE(result);EXPECT_NEAR(result->translation().x(),.2,1e-12);EXPECT_NEAR(result->translation().z(),.33,1e-12);
 EXPECT_FALSE(frames.world_from_base(1150000000,1150000000,2050000000));
}
TEST(PayloadFrames, WrongEntityAndDuplicateRenewalCannotSupplyWorldTruth) {
 PayloadFrames frames(xml,"base","tcp","robot",136,Eigen::Isometry3d::Identity());
 auto wrong=poses(1000000000,.13);wrong.mutable_pose(0)->set_id(999);
 EXPECT_THROW(frames.observe(wrong,{1000000000,2000000000}),std::runtime_error);
 frames.observe(poses(1000000000,.13),{1100000000,2000000000});
 frames.observe(poses(1000000000,.13),{1100000000,2150000000});
 EXPECT_THROW(frames.world_from_base(1000000000,1100000000,2200000000),std::runtime_error);
}

TEST(PayloadWorldInbox, RebindingDropsQueuedAndInFlightPreviousEpoch) {
 PayloadWorldInbox inbox;
 const auto unbound=inbox.ticket();
 inbox.receive(poses(1000000000,.13),{1000000000,2000000000},unbound);
 EXPECT_TRUE(inbox.take().empty());
 inbox.bind(1000000000);
 // A callback begun before the first binding cannot seed the new epoch,
 // even when its source capture is newer than the binding time.
 inbox.receive(poses(1100000000,.13),{1100000000,2000000000},unbound);
 EXPECT_TRUE(inbox.take().empty());
 const auto previous=inbox.ticket();
 inbox.receive(poses(1100000000,.13),{1100000000,2000000000},previous);
 auto first=inbox.take();ASSERT_EQ(first.size(),1u);
 EXPECT_EQ(first.front().first.header().stamp().nsec(),100000000);
 inbox.receive(poses(1100000000,.13),{1100000000,2000000000},previous);
 // Rebind while the old capture still satisfies the original 300 ms lease.
 inbox.bind(1150000000);
 EXPECT_TRUE(inbox.take().empty());
 inbox.receive(poses(1200000000,.13),{1200000000,2050000000},previous);
 EXPECT_TRUE(inbox.take().empty());
 // A queued transport packet delivered after binding is still too old.
 inbox.receive(poses(1100000000,.13),{1200000000,2050000000},inbox.ticket());
 EXPECT_TRUE(inbox.take().empty());
 inbox.receive(poses(1200000000,.33),{1200000000,2050000000},inbox.ticket());
 auto values=inbox.take();ASSERT_EQ(values.size(),1u);
 EXPECT_EQ(values.front().second.steady,2050000000);
 EXPECT_DOUBLE_EQ(values.front().first.pose(0).position().z(),.33);
}

TEST(PayloadFrames, SameExecutorTfPollingUsesOriginalCaptureWithoutTimeoutWrapper) {
 auto clock=std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);tf2_ros::Buffer buffer(clock);
 ASSERT_FALSE(buffer.isUsingDedicatedThread()); // TransformListener spin_thread=false configuration.
 geometry_msgs::msg::TransformStamped transform;transform.header.frame_id="base";transform.child_frame_id="wrist";
 transform.header.stamp.sec=10;transform.transform.translation.x=.1;transform.transform.rotation.w=1.;
 ASSERT_TRUE(buffer.setTransform(transform,"fixture",false));
 // Reproduce the previous Humble API defect even though this capture exists.
 EXPECT_FALSE(buffer.canTransform("base","wrist",rclcpp::Time(10000000000LL,RCL_ROS_TIME),rclcpp::Duration::from_nanoseconds(0)));
 const tf2::TimePoint capture{std::chrono::nanoseconds(10000000000LL)};
 EXPECT_TRUE(buffer.canTransform("base","wrist",capture));
 EXPECT_DOUBLE_EQ(buffer.lookupTransform("base","wrist",capture).transform.translation.x,.1);
 EXPECT_FALSE(buffer.canTransform("base","missing",capture));
 EXPECT_FALSE(buffer.canTransform("base","wrist",capture+std::chrono::nanoseconds(1)));
}
