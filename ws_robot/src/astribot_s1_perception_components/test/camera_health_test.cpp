#include <gtest/gtest.h>
#include <thread>
#include <astribot_perception_msgs/msg/camera_session_state.hpp>
#define main camera_health_main
#include "../src/camera_health_node.cpp"
#undef main
using namespace std::chrono_literals;
using Health=astribot_perception_msgs::msg::CameraHealth;
class HealthContract : public ::testing::Test {
protected:
  static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void start(int calibration=1,bool activation=false,double timer_period=.01) {
    rclcpp::NodeOptions opts;opts.parameter_overrides({
      rclcpp::Parameter("color_topic","/health_test/color"),rclcpp::Parameter("depth_topic","/health_test/depth"),
      rclcpp::Parameter("info_topic","/health_test/info"),rclcpp::Parameter("calibration_revision",calibration),
      rclcpp::Parameter("timer_period_sec",timer_period),
      rclcpp::Parameter("activation_topic",activation?"/health_test/session":"")});
    sut=std::make_shared<CameraHealthNode>(opts);probe=std::make_shared<rclcpp::Node>("health_probe");
    color_pub=probe->create_publisher<sensor_msgs::msg::Image>("/health_test/color",rclcpp::SensorDataQoS());
    depth_pub=probe->create_publisher<sensor_msgs::msg::Image>("/health_test/depth",rclcpp::SensorDataQoS());
    info_pub=probe->create_publisher<sensor_msgs::msg::CameraInfo>("/health_test/info",rclcpp::SensorDataQoS());
    sub=probe->create_subscription<Health>("/perception/camera_health",10,[this](Health::ConstSharedPtr p){health=p;});
    exec.add_node(sut);exec.add_node(probe);pump(30);
    image.header.frame_id="camera";image.width=image.height=10;image.encoding="rgb8";image.step=30;image.data.resize(300);
    depth=image;depth.encoding="16UC1";depth.step=20;depth.data.resize(200);
    info.header=image.header;info.width=info.height=10;info.k={100.,0.,4.5,0.,100.,4.5,0.,0.,1.};
  }
  void pump(int ms) {const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);do{exec.spin_some();std::this_thread::sleep_for(1ms);}while(std::chrono::steady_clock::now()<end);}
  void send() {
    const auto t=probe->now();
    for(int i=0;i<2;++i) {
      image.header.stamp=depth.header.stamp=info.header.stamp=t-rclcpp::Duration::from_seconds(i==0?.1:0.);
      color_pub->publish(image);depth_pub->publish(depth);info_pub->publish(info);pump(10);
    }
    pump(10);
  }
  void TearDown() override {if(sut) exec.remove_node(sut);if(probe)exec.remove_node(probe);}
  std::shared_ptr<CameraHealthNode> sut;rclcpp::Node::SharedPtr probe;rclcpp::executors::SingleThreadedExecutor exec;
  sensor_msgs::msg::Image image,depth;sensor_msgs::msg::CameraInfo info;Health::ConstSharedPtr health;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub,depth_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;rclcpp::Subscription<Health>::SharedPtr sub;
};
TEST_F(HealthContract, ValidAlignedStreams) { start();send();ASSERT_TRUE(health);EXPECT_TRUE(health->valid); }
TEST_F(HealthContract, RejectsInvalidIntrinsics) { start();info.k[0]=0;send();ASSERT_TRUE(health);EXPECT_FALSE(health->valid); }
TEST_F(HealthContract, RejectsMalformedDepth) { start();depth.data.resize(1);send();ASSERT_TRUE(health);EXPECT_FALSE(health->valid); }
TEST_F(HealthContract, RejectsUnknownCalibration) { start(0);send();ASSERT_TRUE(health);EXPECT_FALSE(health->valid); }
TEST_F(HealthContract, PausedClockKeepsLatestCameraObservation) {
  start();send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  auto * clock=sut->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,rclcpp::Time(image.header.stamp).nanoseconds()+40000000),RCL_RET_OK);
  pump(300);ASSERT_TRUE(health);EXPECT_TRUE(health->valid);
}
TEST_F(HealthContract, SameRevisionCannotChangeIntrinsics) {
  start();send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  info.k[0]=200;send();ASSERT_TRUE(health);EXPECT_FALSE(health->valid);
}
TEST_F(HealthContract, InFlightRgbDoesNotInvalidateLastMatchedSnapshot) {
  start();send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  const auto matched=health->capture_stamp;
  pump(70);
  const auto next=probe->now();
  image.header.stamp=depth.header.stamp=info.header.stamp=next;
  color_pub->publish(image);pump(15);
  ASSERT_TRUE(health);EXPECT_TRUE(health->valid)<<health->reason_code;
  EXPECT_EQ(health->capture_stamp,matched);
  EXPECT_GT(health->sync_skew_sec,0.0);
  depth_pub->publish(depth);pump(15);
  ASSERT_TRUE(health);EXPECT_TRUE(health->valid)<<health->reason_code;
  EXPECT_EQ(rclcpp::Time(health->capture_stamp).nanoseconds(),next.nanoseconds());
  info_pub->publish(info);pump(15);
  ASSERT_TRUE(health);EXPECT_TRUE(health->valid)<<health->reason_code;
  EXPECT_EQ(rclcpp::Time(health->capture_stamp).nanoseconds(),next.nanoseconds());
}
TEST_F(HealthContract, DelayedDepthMatchesBufferedRgbAndInfo) {
  start();send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  pump(70);
  const auto first=probe->now();
  image.header.stamp=info.header.stamp=first;
  color_pub->publish(image);info_pub->publish(info);pump(10);
  pump(90);
  const auto second=probe->now();
  image.header.stamp=info.header.stamp=second;
  color_pub->publish(image);info_pub->publish(info);pump(5);
  depth.header.stamp=first;depth_pub->publish(depth);pump(15);
  ASSERT_TRUE(health);EXPECT_TRUE(health->valid)<<health->reason_code;
  EXPECT_EQ(rclcpp::Time(health->capture_stamp).nanoseconds(),first.nanoseconds());
  depth.header.stamp=second;depth_pub->publish(depth);pump(15);
  ASSERT_TRUE(health);EXPECT_TRUE(health->valid)<<health->reason_code;
  EXPECT_EQ(rclcpp::Time(health->capture_stamp).nanoseconds(),second.nanoseconds());
}
TEST_F(HealthContract, NearestDepthKeepsItsOriginalCaptureStamp) {
  start();send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  const auto matched=health->capture_stamp,expires=health->valid_until;
  for(int i=0;i<5;++i) {
    pump(60);image.header.stamp=info.header.stamp=probe->now();
    color_pub->publish(image);info_pub->publish(info);pump(5);
  }
  ASSERT_TRUE(health);EXPECT_TRUE(health->valid);
  EXPECT_EQ(health->reason_code,"CAMERA_READY");
  EXPECT_EQ(health->capture_stamp,matched);EXPECT_EQ(health->valid_until,expires);
}
TEST_F(HealthContract, PersistentSkewUsesNearestAvailableFrames) {
  start();const auto t=probe->now();
  for(int i=0;i<2;++i) {
    image.header.stamp=t-rclcpp::Duration::from_seconds(i==0?.18:.08);
    depth.header.stamp=info.header.stamp=t-rclcpp::Duration::from_seconds(i==0?.13:.03);
    color_pub->publish(image);depth_pub->publish(depth);info_pub->publish(info);pump(15);
  }
  ASSERT_TRUE(health);EXPECT_TRUE(health->valid);EXPECT_GT(health->sync_skew_sec,.03);
}
TEST_F(HealthContract, MalformedDepthCannotHideBehindMatchedSnapshot) {
  start();send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  depth.header.stamp=probe->now();depth.data.resize(1);depth_pub->publish(depth);pump(20);
  ASSERT_TRUE(health);EXPECT_FALSE(health->valid);
}
TEST_F(HealthContract, FrameMismatchCannotHideBehindMatchedSnapshot) {
  start();send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  image.header.stamp=probe->now();image.header.frame_id="wrong_camera";
  color_pub->publish(image);pump(20);
  ASSERT_TRUE(health);EXPECT_FALSE(health->valid);EXPECT_EQ(health->reason_code,"CAMERA_FRAME_MISMATCH");
}
TEST_F(HealthContract, LowRateIsDiagnosticOnly) {
  start();send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  pump(150);const auto next=probe->now();
  image.header.stamp=depth.header.stamp=info.header.stamp=next;
  color_pub->publish(image);depth_pub->publish(depth);info_pub->publish(info);pump(20);
  ASSERT_TRUE(health);EXPECT_TRUE(health->valid);EXPECT_LT(health->frequency_hz,10.);
}
TEST_F(HealthContract, ClockRollbackDoesNotChangeSourceEpoch) {
  start();send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  const auto epoch=health->source_epoch;
  auto * clock=sut->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,rclcpp::Time(image.header.stamp).nanoseconds()-100000000),RCL_RET_OK);
  pump(20);ASSERT_TRUE(health);EXPECT_TRUE(health->valid);EXPECT_EQ(health->source_epoch,epoch);
}

TEST_F(HealthContract, MissingTaskSessionRejectsFreshImages) {
 start(1,true);send();ASSERT_TRUE(health);EXPECT_FALSE(health->valid);EXPECT_EQ(health->state,"INACTIVE");
}
TEST_F(HealthContract, TaskSessionReleaseImmediatelyRevokesHealth) {
 start(1,true);using S=astribot_perception_msgs::msg::CameraSessionState;
 auto pub=probe->create_publisher<S>("/health_test/session",rclcpp::QoS(1).transient_local());
 S state;state.camera_id="camera";state.owner_id="owner";state.execution_id="task";state.session_token="session1";state.active=true;
 state.activated_at=probe->now()-rclcpp::Duration::from_seconds(.2);state.header.stamp=probe->now();state.valid_until=probe->now()+rclcpp::Duration::from_seconds(2.);pub->publish(state);pump(20);send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
 state.active=false;state.header.stamp=probe->now();pub->publish(state);pump(20);ASSERT_TRUE(health);EXPECT_FALSE(health->valid);EXPECT_EQ(health->state,"INACTIVE");
}
TEST_F(HealthContract, LatestActiveSessionDoesNotExpireFromObservationAge) {
 start(1,true);using S=astribot_perception_msgs::msg::CameraSessionState;
 auto pub=probe->create_publisher<S>("/health_test/session",rclcpp::QoS(1).transient_local());
 S state;state.camera_id="camera";state.owner_id="owner";state.execution_id="task";state.session_token="session1";state.active=true;
 state.activated_at=probe->now()-rclcpp::Duration::from_seconds(.2);state.header.stamp=probe->now();state.valid_until=probe->now()+rclcpp::Duration::from_seconds(2.);pub->publish(state);pump(20);send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
 pump(300);send();ASSERT_TRUE(health);EXPECT_TRUE(health->valid);EXPECT_EQ(health->state,"OK");
}

TEST_F(HealthContract, NewSessionCannotAcceptBufferedOldImages) {
 start(1,true);using S=astribot_perception_msgs::msg::CameraSessionState;
 auto pub=probe->create_publisher<S>("/health_test/session",rclcpp::QoS(1).transient_local());
 const auto t=probe->now();S state;state.camera_id="camera";state.owner_id="owner";
 state.execution_id="task";state.session_token="new_session";state.active=true;
 state.header.stamp=t;state.activated_at=t;state.valid_until=t+rclcpp::Duration::from_seconds(2.);
 pub->publish(state);pump(15);
 for(double age:{.15,.05}) {
   image.header.stamp=depth.header.stamp=info.header.stamp=t-rclcpp::Duration::from_seconds(age);
   color_pub->publish(image);depth_pub->publish(depth);info_pub->publish(info);pump(10);
 }
 pump(15);ASSERT_TRUE(health);EXPECT_FALSE(health->valid);
}
TEST_F(HealthContract, ObservationLifetimeCannotOutliveItsCameraSession) {
 start(1,true);using S=astribot_perception_msgs::msg::CameraSessionState;
 auto pub=probe->create_publisher<S>("/health_test/session",rclcpp::QoS(1).transient_local());
 const auto t=probe->now();S state;state.camera_id="camera";state.owner_id="owner";
 state.execution_id="task";state.session_token="short_session";state.active=true;
 state.header.stamp=t;state.activated_at=t-rclcpp::Duration::from_seconds(.2);
 state.valid_until=t+rclcpp::Duration::from_seconds(.15);
 pub->publish(state);pump(15);send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
 EXPECT_LE(rclcpp::Time(health->valid_until).nanoseconds(),rclcpp::Time(state.valid_until).nanoseconds());
}
TEST_F(HealthContract, FutureHeartbeatAndExplicitReleaseAreApplied) {
 start(1,true);using S=astribot_perception_msgs::msg::CameraSessionState;
 auto pub=probe->create_publisher<S>("/health_test/session",rclcpp::QoS(1).transient_local());
 S state;state.camera_id="camera";state.owner_id="owner";state.execution_id="task";
 state.session_token="same_session";state.active=true;state.header.stamp=probe->now();
 state.activated_at=probe->now()-rclcpp::Duration::from_seconds(.2);
 state.valid_until=probe->now()+rclcpp::Duration::from_seconds(2.);
 pub->publish(state);pump(15);send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
 auto * clock=sut->get_clock()->get_clock_handle();const auto t=probe->now();
 ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
 ASSERT_EQ(rcl_set_ros_time_override(clock,t.nanoseconds()),RCL_RET_OK);
 state.header.stamp=t+rclcpp::Duration::from_seconds(.001);pub->publish(state);pump(20);
 ASSERT_TRUE(health);EXPECT_TRUE(health->valid);
 // A release is applied immediately even when its header is ahead of /clock.
 state.active=false;pub->publish(state);pump(20);EXPECT_FALSE(health->valid);
}

TEST_F(HealthContract, DelayedAndDuplicateObservationsKeepStampAndEpoch) {
  start();send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  const auto epoch=health->source_epoch;
  const auto capture=health->capture_stamp,expires=health->valid_until;
  pump(300);ASSERT_TRUE(health->valid);
  EXPECT_EQ(health->source_epoch,epoch);
  EXPECT_EQ(health->capture_stamp,capture);EXPECT_EQ(health->valid_until,expires);
  color_pub->publish(image);depth_pub->publish(depth);info_pub->publish(info);pump(25);
  EXPECT_TRUE(health->valid);EXPECT_EQ(health->source_epoch,epoch);
  EXPECT_EQ(health->capture_stamp,capture);EXPECT_EQ(health->valid_until,expires);
  send();ASSERT_TRUE(health->valid)<<health->reason_code;
  EXPECT_EQ(health->source_epoch,epoch);
  EXPECT_GT(rclcpp::Time(health->capture_stamp).nanoseconds(),rclcpp::Time(capture).nanoseconds());
}

TEST_F(HealthContract, CorrectedColorAloneCannotReviveOldMatchedSnapshot) {
  start();send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  const auto epoch=health->source_epoch;
  const auto capture=health->capture_stamp;
  image.header.stamp=probe->now();image.header.frame_id="wrong_camera";
  color_pub->publish(image);pump(15);ASSERT_FALSE(health->valid);
  const auto broken_epoch=health->source_epoch;EXPECT_NE(broken_epoch,epoch);
  image.header.stamp=depth.header.stamp=info.header.stamp=probe->now();image.header.frame_id="camera";
  color_pub->publish(image);pump(15);
  EXPECT_FALSE(health->valid);EXPECT_EQ(health->capture_stamp,capture);
  EXPECT_EQ(health->source_epoch,broken_epoch);
  depth_pub->publish(depth);info_pub->publish(info);pump(20);
  ASSERT_TRUE(health->valid)<<health->reason_code;EXPECT_EQ(health->source_epoch,broken_epoch);
  EXPECT_EQ(health->capture_stamp,image.header.stamp);
}

TEST_F(HealthContract, HeartbeatRecoveryRequiresFramesAfterReauthorization) {
  start(1,true);using S=astribot_perception_msgs::msg::CameraSessionState;
  auto pub=probe->create_publisher<S>("/health_test/session",rclcpp::QoS(1).transient_local());
  S state;state.camera_id="camera";state.owner_id="owner";state.execution_id="task";
  state.session_token="same_session";state.active=true;
  state.activated_at=probe->now()-rclcpp::Duration::from_seconds(.2);
  state.header.stamp=probe->now();state.valid_until=probe->now()+rclcpp::Duration::from_seconds(2.);
  pub->publish(state);pump(15);send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  const auto epoch=health->source_epoch;
  state.active=false;state.header.stamp=probe->now();
  pub->publish(state);pump(15);ASSERT_FALSE(health->valid);
  const auto broken_epoch=health->source_epoch;EXPECT_NE(broken_epoch,epoch);
  // Images arriving while the lease is unavailable cannot authorize recovery.
  send();ASSERT_FALSE(health->valid);
  state.active=true;state.header.stamp=probe->now();state.activated_at=state.header.stamp;
  state.valid_until=probe->now()+rclcpp::Duration::from_seconds(2.);
  pub->publish(state);pump(15);EXPECT_FALSE(health->valid);
  EXPECT_NE(health->source_epoch,broken_epoch);
  const auto resumed_epoch=health->source_epoch;
  pump(110);send();ASSERT_TRUE(health->valid)<<health->reason_code;EXPECT_EQ(health->source_epoch,resumed_epoch);
}

TEST_F(HealthContract, ReusedSessionTokenNeverReusesPriorEpoch) {
  start(1,true);using S=astribot_perception_msgs::msg::CameraSessionState;
  auto pub=probe->create_publisher<S>("/health_test/session",rclcpp::QoS(1).transient_local());
  S state;state.camera_id="camera";state.owner_id="owner";state.execution_id="task";
  state.session_token="same_session";state.active=true;
  state.activated_at=probe->now()-rclcpp::Duration::from_seconds(.2);
  state.header.stamp=probe->now();state.valid_until=probe->now()+rclcpp::Duration::from_seconds(2.);
  pub->publish(state);pump(15);send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  const auto epoch=health->source_epoch;
  state.active=false;state.header.stamp=probe->now();pub->publish(state);pump(15);ASSERT_FALSE(health->valid);
  state.active=true;state.header.stamp=probe->now();pub->publish(state);pump(15);send();
  ASSERT_TRUE(health->valid)<<health->reason_code;
  EXPECT_NE(health->source_epoch,epoch);EXPECT_LE(health->source_epoch.size(),128u);
  EXPECT_NE(health->source_epoch.find(state.session_token),std::string::npos);
}

TEST_F(HealthContract, ChangedExecutionWithSameTokenRevokesOldFrames) {
  start(1,true);using S=astribot_perception_msgs::msg::CameraSessionState;
  auto pub=probe->create_publisher<S>("/health_test/session",rclcpp::QoS(1).transient_local());
  S state;state.camera_id="camera";state.owner_id="owner";state.execution_id="task";
  state.session_token="same_session";state.active=true;
  state.activated_at=probe->now()-rclcpp::Duration::from_seconds(.2);
  state.header.stamp=probe->now();state.valid_until=probe->now()+rclcpp::Duration::from_seconds(2.);
  pub->publish(state);pump(15);send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  const auto epoch=health->source_epoch;
  state.execution_id="next_task";state.header.stamp=probe->now();pub->publish(state);pump(15);
  EXPECT_FALSE(health->valid);EXPECT_NE(health->source_epoch,epoch);
  send();ASSERT_TRUE(health->valid)<<health->reason_code;
  EXPECT_NE(health->source_epoch,epoch);
}

TEST_F(HealthContract, DelayedFramesCapturedBeforeReauthorizationStayInvalid) {
  start(1,true);using S=astribot_perception_msgs::msg::CameraSessionState;
  auto pub=probe->create_publisher<S>("/health_test/session",rclcpp::QoS(1).transient_local());
  S state;state.camera_id="camera";state.owner_id="owner";state.execution_id="task";
  state.session_token="same_session";state.active=true;
  state.activated_at=probe->now()-rclcpp::Duration::from_seconds(.2);
  state.header.stamp=probe->now();state.valid_until=probe->now()+rclcpp::Duration::from_seconds(2.);
  pub->publish(state);pump(15);send();ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  state.active=false;state.header.stamp=probe->now();
  pub->publish(state);pump(15);ASSERT_FALSE(health->valid);
  image.header.stamp=depth.header.stamp=info.header.stamp=probe->now();
  pump(15);
  state.active=true;state.header.stamp=probe->now();state.activated_at=state.header.stamp;
  state.valid_until=probe->now()+rclcpp::Duration::from_seconds(2.);
  pub->publish(state);pump(15);
  color_pub->publish(image);depth_pub->publish(depth);info_pub->publish(info);pump(15);
  EXPECT_FALSE(health->valid);
  EXPECT_LT(rclcpp::Time(health->capture_stamp).nanoseconds(),rclcpp::Time(state.header.stamp).nanoseconds());
  EXPECT_LT(health->age_sec,.25); // Rejection is a recovery boundary, not expiry.
  pump(110);send();ASSERT_TRUE(health->valid)<<health->reason_code;
}

TEST_F(HealthContract, MalformedCallbackBreaksEpochEvenBetweenHealthPublications) {
  // Health is published on session updates here; the long timer allows the
  // malformed callback and repair to happen without an intervening health tick.
  start(1,true,1.);using S=astribot_perception_msgs::msg::CameraSessionState;
  auto pub=probe->create_publisher<S>("/health_test/session",rclcpp::QoS(1).transient_local());
  S state;state.camera_id="camera";state.owner_id="owner";state.execution_id="task";
  state.session_token="same_session";state.active=true;
  state.activated_at=probe->now()-rclcpp::Duration::from_seconds(.2);
  state.header.stamp=probe->now();state.valid_until=probe->now()+rclcpp::Duration::from_seconds(2.);
  pub->publish(state);pump(10);send();
  state.header.stamp=probe->now();pub->publish(state);pump(10);
  ASSERT_TRUE(health);ASSERT_TRUE(health->valid);
  const auto epoch=health->source_epoch;const auto sequence=health->sequence;
  depth.header.stamp=probe->now();depth.data.resize(1);depth_pub->publish(depth);pump(10);
  depth.data.resize(200);send();send();
  ASSERT_EQ(health->sequence,sequence); // No invalid publication was needed.
  state.header.stamp=probe->now();pub->publish(state);pump(10);
  ASSERT_TRUE(health->valid)<<health->reason_code;
  EXPECT_NE(health->source_epoch,epoch);
}

TEST_F(HealthContract, LatestContractFutureAndOldCaptureRemainUsable) {
 start();const auto t=probe->now()+rclcpp::Duration::from_seconds(20.);
 image.header.stamp=depth.header.stamp=t;info.header.stamp=t-rclcpp::Duration::from_seconds(3.);
 color_pub->publish(image);depth_pub->publish(depth);info_pub->publish(info);pump(30);
 ASSERT_TRUE(health);ASSERT_TRUE(health->valid)<<health->reason_code;const auto capture=health->capture_stamp;
 auto *clock=sut->get_clock()->get_clock_handle();ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
 ASSERT_EQ(rcl_set_ros_time_override(clock,t.nanoseconds()+100000000000LL),RCL_RET_OK);
 pump(300);ASSERT_TRUE(health->valid)<<health->reason_code;EXPECT_EQ(health->capture_stamp,capture);
 image.header.stamp=depth.header.stamp=info.header.stamp=t-rclcpp::Duration::from_seconds(5.);
 color_pub->publish(image);depth_pub->publish(depth);info_pub->publish(info);pump(30);
 EXPECT_EQ(health->capture_stamp,capture);EXPECT_TRUE(health->valid);
}
