#include <gtest/gtest.h>
#include <cstring>
#include <thread>
#define main rgbd_node_main
#include "../src/rgbd_object_pose_node.cpp"
#undef main

using namespace std::chrono_literals;
using Detection = astribot_perception_msgs::msg::Detection2D;
using Health = astribot_perception_msgs::msg::CameraHealth;
using Observation = astribot_perception_msgs::msg::ObjectPoseObservation;

class VisionContract : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    sut = std::make_shared<RgbdObjectPoseNode>();
    probe = std::make_shared<rclcpp::Node>("vision_contract_probe");
    depth_pub = probe->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", rclcpp::SensorDataQoS());
    info_pub = probe->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", rclcpp::SensorDataQoS());
    health_pub = probe->create_publisher<Health>("/perception/camera_health/head_rgbd", rclcpp::QoS(1).transient_local());
    det_pub = probe->create_publisher<Detection>("/perception/detections", 10);
    out = probe->create_subscription<Observation>("/perception/object_pose", 10,
      [this](Observation::ConstSharedPtr msg) { observations.push_back(*msg); });
    executor.add_node(probe); executor.add_node(sut);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (det_pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) pump(10);
    ASSERT_GT(det_pub->get_subscription_count(), 0u);
    const auto now = probe->now();
    depth.header.stamp = now; depth.header.frame_id = "camera_optical";
    depth.width = depth.height = 10; depth.encoding = "32FC1"; depth.step = 40;
    depth.data.resize(400); set_depth(1.0f);
    info.header = depth.header; info.width = info.height = 10;
    info.k = {100.,0.,4.5,0.,100.,4.5,0.,0.,1.};
    health.header = depth.header; health.frame_id = "camera_optical";
    health.camera_id = "head_rgbd"; health.source_epoch = "epoch1";
    health.calibration_revision = 1; health.valid = true;
    health.capture_stamp = now; health.valid_until = now + rclcpp::Duration::from_seconds(.25);
    det.header = depth.header; det.detection_id = "d1"; det.object_id = "o1";
    det.source_camera_id = "head_rgbd"; det.model_name = "fixture"; det.model_revision = "test1";
    det.source_epoch="epoch1"; det.calibration_revision = 1; det.confidence = .9;
    det.x_offset = det.y_offset = 2; det.width = det.height = 6;
  }
  void TearDown() override { executor.remove_node(sut); executor.remove_node(probe); }
  void pump(int ms=20) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    do { executor.spin_some(); std::this_thread::sleep_for(1ms); } while(std::chrono::steady_clock::now() < end);
  }
  void set_depth(float z) { for(size_t i=0;i<100;++i) std::memcpy(depth.data.data()+4*i,&z,4); }
  void inputs() { depth_pub->publish(depth); info_pub->publish(info); health_pub->publish(health); pump(); }
  void send() { inputs(); det_pub->publish(det); pump(35); }
  std::shared_ptr<RgbdObjectPoseNode> sut;
  rclcpp::Node::SharedPtr probe;
  rclcpp::executors::SingleThreadedExecutor executor;
  sensor_msgs::msg::Image depth; sensor_msgs::msg::CameraInfo info; Health health; Detection det;
  std::vector<Observation> observations;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
  rclcpp::Publisher<Health>::SharedPtr health_pub;
  rclcpp::Publisher<Detection>::SharedPtr det_pub;
  rclcpp::Subscription<Observation>::SharedPtr out;
};
TEST_F(VisionContract, PositiveAlignedDepth) { send(); ASSERT_EQ(observations.size(),1u); EXPECT_NEAR(observations[0].pose.pose.position.z,1.,1e-6); }
TEST_F(VisionContract, RejectsFrameMismatch) { det.header.frame_id="wrong_camera"; send(); EXPECT_TRUE(observations.empty()); }
TEST_F(VisionContract, RejectsUnsynchronisedDepth) { depth.header.stamp=probe->now()-rclcpp::Duration::from_seconds(.10); send(); EXPECT_TRUE(observations.empty()); }
TEST_F(VisionContract, RejectsExpiredHealth) { health.valid_until=probe->now()-rclcpp::Duration::from_seconds(.01); send(); EXPECT_TRUE(observations.empty()); }
TEST_F(VisionContract, RejectsUnknownCalibration) { det.calibration_revision=0; send(); EXPECT_TRUE(observations.empty()); }
TEST_F(VisionContract, RejectsInvalidMaskRatherThanBoxFallback) { det.mask.width=3; det.mask.height=3; det.mask.step=3; det.mask.encoding="rgb8"; det.mask.data.resize(9,255); send(); EXPECT_TRUE(observations.empty()); }
TEST_F(VisionContract, RejectsNaNConfidence) { det.confidence=std::numeric_limits<double>::quiet_NaN(); send(); EXPECT_TRUE(observations.empty()); }
TEST_F(VisionContract, RejectsStaleIntrinsics) { info.header.stamp=probe->now()-rclcpp::Duration::from_seconds(5); send(); EXPECT_TRUE(observations.empty()); }
TEST_F(VisionContract, PreservesCaptureDeadline) { send(); ASSERT_EQ(observations.size(),1u); EXPECT_LE(rclcpp::Time(observations[0].valid_until).nanoseconds(),rclcpp::Time(health.valid_until).nanoseconds()); }
TEST_F(VisionContract, MatchesHistoryRatherThanLatestDepth) {
  inputs(); set_depth(2.f); depth.header.stamp=probe->now()+rclcpp::Duration::from_seconds(.01);
  info.header=depth.header; inputs(); det_pub->publish(det); pump(30);
  ASSERT_EQ(observations.size(),1u); EXPECT_NEAR(observations[0].pose.pose.position.z,1.,1e-6);
}
TEST_F(VisionContract, PositionMustNotClaimOrientation) { send(); ASSERT_EQ(observations.size(),1u); EXPECT_TRUE(observations[0].position_valid); EXPECT_FALSE(observations[0].orientation_valid); EXPECT_GE(observations[0].pose.covariance[35],1e6); }
TEST_F(VisionContract, RejectsTruncatedDepthBuffer) { depth.data.resize(20);send();EXPECT_TRUE(observations.empty()); }
TEST_F(VisionContract, RejectsBigEndianDepth) { depth.is_bigendian=true;send();EXPECT_TRUE(observations.empty()); }
TEST_F(VisionContract, RejectsEpochMismatch) { det.source_epoch="previous_session";send();EXPECT_TRUE(observations.empty()); }
TEST_F(VisionContract, MillimetreDepthUsesMeters) {
  depth.encoding="16UC1";depth.step=20;depth.data.resize(200);
  for(size_t i=0;i<100;++i) {uint16_t z=1250;std::memcpy(depth.data.data()+2*i,&z,2);}
  send();ASSERT_EQ(observations.size(),1u);EXPECT_NEAR(observations[0].pose.pose.position.z,1.25,1e-6);
}
TEST_F(VisionContract, MaskControlsVisibleSupportCentroid) {
  det.mask.width=det.mask.height=6;det.mask.step=6;det.mask.encoding="mono8";det.mask.data.resize(36,0);
  for(size_t y=0;y<6;++y) for(size_t x=0;x<2;++x) det.mask.data[y*6+x]=255;
  send();ASSERT_EQ(observations.size(),1u);EXPECT_NEAR(observations[0].pose.pose.position.x,-.02,1e-6);
}
TEST_F(VisionContract, RejectsDegenerateIntrinsicsWithoutNaNObservation) { info.k[0]=1e-310;send();EXPECT_TRUE(observations.empty()); }
TEST_F(VisionContract, RejectsCovarianceOverflow) { info.k[0]=1e-160;send();EXPECT_TRUE(observations.empty()); }
TEST_F(VisionContract, PausedClockAndLostHealthRejectsCachedObservation) {
  inputs();auto * clock=sut->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,rclcpp::Time(det.header.stamp).nanoseconds()+50000000),RCL_RET_OK);
  pump(550);det_pub->publish(det);pump(30);EXPECT_TRUE(observations.empty());
}
TEST_F(VisionContract, PausedClockDoesNotPreserveDepthDespiteHealthHeartbeat) {
  inputs();auto * clock=sut->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,rclcpp::Time(det.header.stamp).nanoseconds()+50000000),RCL_RET_OK);
  for(int i=0;i<6;++i) {health_pub->publish(health);pump(45);}
  det_pub->publish(det);pump(30);EXPECT_TRUE(observations.empty());
}
TEST_F(VisionContract, SameRevisionCannotChangeIntrinsics) {
  inputs();info.k[0]=200;info.header.stamp=probe->now();depth.header.stamp=info.header.stamp;det.header.stamp=info.header.stamp;
  send();EXPECT_TRUE(observations.empty());
}
TEST_F(VisionContract, InvalidFirstIntrinsicsMustNotPoisonRecovery) {
  info.k[0]=0;inputs();info.k[0]=100;info.header.stamp=probe->now();depth.header.stamp=info.header.stamp;det.header.stamp=info.header.stamp;
  send();ASSERT_EQ(observations.size(),1u);EXPECT_TRUE(observations[0].position_valid);
}
TEST_F(VisionContract, ZeroStampIntrinsicsMustNotPoisonRecovery) {
  const auto original=info;info.header.stamp=builtin_interfaces::msg::Time{};info.k[0]=200;inputs();info=original;
  send();ASSERT_EQ(observations.size(),1u);EXPECT_TRUE(observations[0].position_valid);
}
