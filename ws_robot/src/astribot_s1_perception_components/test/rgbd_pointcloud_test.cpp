#include <gtest/gtest.h>
#include <thread>
#include <cstring>
#define main rgbd_pointcloud_main
#include "../src/rgbd_pointcloud_node.cpp"
#undef main
using namespace std::chrono_literals;
class CloudContract : public ::testing::Test {
protected:
  struct DelayedProjector:astribot::vision::DepthProjector {
    explicit DelayedProjector(int ms):delay(ms),cpu(astribot::vision::make_depth_projector("cpu")){}
    void project(const astribot::vision::DepthView& v,const astribot::vision::ProjectionConfig& c,float* out)override {
      std::this_thread::sleep_for(std::chrono::milliseconds(delay));cpu->project(v,c,out);
    }
    int delay;std::unique_ptr<astribot::vision::DepthProjector>cpu;
  };
  void delayed(int milliseconds, bool require_session=false) {
    exec.remove_node(sut);sut.reset();
    rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("depth_topic","/cloud_test/depth"),
      rclcpp::Parameter("camera_info_topic","/cloud_test/info"),rclcpp::Parameter("output_topic","/cloud_test/cloud"),rclcpp::Parameter("decimation",1),
      rclcpp::Parameter("activation_topic",require_session?"/cloud_test/session":"")});
    sut=std::make_shared<astribot_s1_autonomy::RgbdPointcloudNode>(options,std::make_unique<DelayedProjector>(milliseconds));
    exec.add_node(sut);pump(70);
  }
  static void SetUpTestSuite() { rclcpp::init(0,nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("depth_topic","/cloud_test/depth"),
      rclcpp::Parameter("camera_info_topic","/cloud_test/info"),
      rclcpp::Parameter("output_topic","/cloud_test/cloud"),rclcpp::Parameter("decimation",1)});
    sut=std::make_shared<astribot_s1_autonomy::RgbdPointcloudNode>(options);
    probe=std::make_shared<rclcpp::Node>("cloud_probe");
    dp=probe->create_publisher<sensor_msgs::msg::Image>("/cloud_test/depth",rclcpp::SensorDataQoS());
    ip=probe->create_publisher<sensor_msgs::msg::CameraInfo>("/cloud_test/info",rclcpp::SensorDataQoS());
    sub=probe->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_test/cloud",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr p){cloud=p;++count;});
    exec.add_node(sut);exec.add_node(probe);pump(60);
    image.header.frame_id="optical";image.header.stamp=probe->now();
    image.width=2;image.height=1;image.step=4;image.encoding="16UC1";image.data={232,3,208,7};
    info.header=image.header;info.width=2;info.height=1;info.k={100.,0.,0.,0.,100.,0.,0.,0.,1.};
  }
  void pump(int ms){auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms); do{exec.spin_some();std::this_thread::sleep_for(1ms);}while(std::chrono::steady_clock::now()<end);}
  void send(bool depth_first=false){if(depth_first){dp->publish(image);pump(15);ip->publish(info);}else{ip->publish(info);pump(15);dp->publish(image);}pump(30);}
  void TearDown()override{exec.remove_node(sut);exec.remove_node(probe);}
  std::shared_ptr<astribot_s1_autonomy::RgbdPointcloudNode> sut;
  rclcpp::Node::SharedPtr probe;rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dp;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ip;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  sensor_msgs::msg::Image image;sensor_msgs::msg::CameraInfo info;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;int count=0;
};
TEST_F(CloudContract, ConvertsMillimeters){send();ASSERT_TRUE(cloud);sensor_msgs::PointCloud2ConstIterator<float> z(*cloud,"z");EXPECT_FLOAT_EQ(*z,1.f);++z;EXPECT_FLOAT_EQ(*z,2.f);EXPECT_EQ(cloud->header,image.header);}
TEST_F(CloudContract, MatchesDepthBeforeInfo){send(true);ASSERT_TRUE(cloud);}
TEST_F(CloudContract, RejectsDecimationOutsideRangeBeforeIntegerConversion) {
  for(const int64_t value : {int64_t{0}, int64_t{65}, int64_t{4294967297}}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("decimation", value)});
    EXPECT_THROW(std::make_shared<astribot_s1_autonomy::RgbdPointcloudNode>(options),
                 std::invalid_argument) << "decimation=" << value;
  }
}
TEST_F(CloudContract, UsesNearestInfoStamp){info.header.stamp=probe->now()-rclcpp::Duration::from_seconds(.1);send();EXPECT_EQ(count,1);}
TEST_F(CloudContract, AcceptsDelayedPair){image.header.stamp=info.header.stamp=probe->now()-rclcpp::Duration::from_seconds(2);send();EXPECT_EQ(count,1);}
TEST_F(CloudContract, RejectsWrongFrame){info.header.frame_id="other";send();EXPECT_EQ(count,0);}
TEST_F(CloudContract, RejectsWrongDimensions){info.width=3;send();EXPECT_EQ(count,0);}
TEST_F(CloudContract, RejectsInvalidStride){image.step=1;send();EXPECT_EQ(count,0);}
TEST_F(CloudContract, RejectsNonfiniteIntrinsics){info.k[0]=NAN;send();EXPECT_EQ(count,0);}
TEST_F(CloudContract, RejectsUnsupportedEncoding){image.encoding="rgb8";send();EXPECT_EQ(count,0);}
TEST_F(CloudContract, RejectsDuplicatePair){send();ASSERT_EQ(count,1);send();EXPECT_EQ(count,1);}
TEST_F(CloudContract, AcceptsFuturePair){image.header.stamp=info.header.stamp=probe->now()+rclcpp::Duration::from_seconds(2);send();EXPECT_EQ(count,1);}
TEST_F(CloudContract, RejectsTruncatedPayload){image.data.resize(1);send();EXPECT_EQ(count,0);}
TEST_F(CloudContract, ConvertsFloatMeters){image.encoding="32FC1";image.step=8;image.data.resize(8);float values[2]={1.f,2.f};std::memcpy(image.data.data(),values,8);send();ASSERT_TRUE(cloud);sensor_msgs::PointCloud2ConstIterator<float> z(*cloud,"z");EXPECT_FLOAT_EQ(*z,1.f);++z;EXPECT_FLOAT_EQ(*z,2.f);}
TEST_F(CloudContract, ConvertsBigEndianMillimeters){image.is_bigendian=true;image.data={3,232,7,208};send();ASSERT_TRUE(cloud);sensor_msgs::PointCloud2ConstIterator<float> z(*cloud,"z");EXPECT_FLOAT_EQ(*z,1.f);++z;EXPECT_FLOAT_EQ(*z,2.f);}
TEST_F(CloudContract, FreshPairRecoversAfterMismatch){info.header.frame_id="other";send();ASSERT_EQ(count,0);info.header.frame_id=image.header.frame_id;image.header.stamp=info.header.stamp=probe->now();send(true);ASSERT_EQ(count,1);}
TEST_F(CloudContract, ClockRollbackKeepsLatestSourceOrder) {
  ASSERT_EQ(rcl_enable_ros_time_override(sut->get_clock()->get_clock_handle()), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(sut->get_clock()->get_clock_handle(), 100000000000LL), RCL_RET_OK);
  image.header.stamp=info.header.stamp=rclcpp::Time(100000000000LL);
  dp->publish(image);pump(15);
  ASSERT_EQ(rcl_set_ros_time_override(sut->get_clock()->get_clock_handle(), 50000000000LL), RCL_RET_OK);
  ip->publish(info);pump(30);ASSERT_EQ(count,1);
  const auto latest=cloud->header.stamp;
  image.header.stamp=info.header.stamp=rclcpp::Time(50000000000LL);
  send();ASSERT_EQ(count,1);EXPECT_EQ(cloud->header.stamp,latest);
}
TEST_F(CloudContract, FrozenClockDoesNotExpirePartial) {
  ASSERT_EQ(rcl_enable_ros_time_override(sut->get_clock()->get_clock_handle()), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(sut->get_clock()->get_clock_handle(), 100000000000LL), RCL_RET_OK);
  image.header.stamp=info.header.stamp=rclcpp::Time(100000000000LL);
  ip->publish(info);pump(300);dp->publish(image);pump(30);EXPECT_EQ(count,1);
}
TEST_F(CloudContract, BoundedCacheSelectsLatestDepthWithNearestInfo) {
  ASSERT_EQ(rcl_enable_ros_time_override(sut->get_clock()->get_clock_handle()), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(sut->get_clock()->get_clock_handle(), 100000000000LL), RCL_RET_OK);
  info.header.stamp=rclcpp::Time(99900000000LL);
  for(int i=0;i<5;++i){image.header.stamp=rclcpp::Time(99900000000LL+i*10000000LL);dp->publish(image);pump(10);}
  ip->publish(info);pump(30);ASSERT_EQ(count,1);EXPECT_EQ(cloud->header.stamp,image.header.stamp);
  EXPECT_GE(sut->statistics().evicted,1u);EXPECT_LE(sut->statistics().cache_peak,8u);
}
TEST_F(CloudContract, SlowComputeDoesNotBlockPairingAndReplacesPending) {
  delayed(200);
  for(int i=0;i<3;++i){image.header.stamp=info.header.stamp=probe->now();send();}
  const auto s=sut->statistics();EXPECT_EQ(s.paired,3u);EXPECT_GE(s.replaced,1u);
  EXPECT_EQ(s.processed,0u);EXPECT_LE(s.cache_peak,8u);EXPECT_EQ(s.pending_peak,1u);
  pump(400);
}
TEST_F(CloudContract, DelayedComputePublishesLatestValidResult) {
  delayed(300);image.header.stamp=info.header.stamp=probe->now();send();pump(350);
  EXPECT_EQ(count,1);EXPECT_EQ(sut->statistics().expired,0u);
}
TEST_F(CloudContract, ClockRollbackDoesNotInvalidateInFlightResult) {
  delayed(120);
  ASSERT_EQ(rcl_enable_ros_time_override(sut->get_clock()->get_clock_handle()),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(sut->get_clock()->get_clock_handle(),100000000000LL),RCL_RET_OK);
  image.header.stamp=info.header.stamp=rclcpp::Time(100000000000LL);send();
  const auto latest=image.header.stamp;
  ASSERT_EQ(rcl_set_ros_time_override(sut->get_clock()->get_clock_handle(),50000000000LL),RCL_RET_OK);
  image.header.stamp=info.header.stamp=rclcpp::Time(50000000000LL);send();pump(220);
  ASSERT_TRUE(cloud);EXPECT_EQ(cloud->header.stamp,latest);EXPECT_EQ(count,1);
  EXPECT_EQ(sut->statistics().invalidated,0u);
}
TEST_F(CloudContract, CalibrationChangeInvalidatesInFlightResult) {
  delayed(100);image.header.stamp=info.header.stamp=probe->now();send();
  info.k[0]=200.;image.header.stamp=info.header.stamp=probe->now();send();pump(220);
  ASSERT_TRUE(cloud);EXPECT_EQ(count,1);EXPECT_EQ(cloud->header.stamp,image.header.stamp);
  EXPECT_GE(sut->statistics().invalidated,1u);
}
TEST_F(CloudContract, MalformedTimestampsAreRejectedWithoutThrowing) {
  image.header.stamp.sec=-1;info.header.stamp=image.header.stamp;
  EXPECT_NO_THROW(send());EXPECT_EQ(count,0);
  image.header.stamp=probe->now();--image.header.stamp.sec;image.header.stamp.nanosec+=1000000000u;info.header.stamp=image.header.stamp;
  EXPECT_NO_THROW(send());EXPECT_EQ(count,0);
  image.header.stamp=info.header.stamp=probe->now();send();EXPECT_EQ(count,1);
}
TEST_F(CloudContract, SessionReleaseInvalidatesInFlightResult) {
  delayed(120,true);
  auto session_pub=probe->create_publisher<astribot_perception_msgs::msg::CameraSessionState>(
    "/cloud_test/session",rclcpp::QoS(1).reliable().transient_local());pump(40);
  astribot_perception_msgs::msg::CameraSessionState s;
  s.active=true;s.session_token="one";s.owner_id="task";s.execution_id="execution";
  s.header.stamp=s.activated_at=probe->now();s.valid_until=probe->now()+rclcpp::Duration::from_seconds(1.);
  session_pub->publish(s);pump(10);image.header.stamp=info.header.stamp=probe->now();send();
  s.active=false;s.header.stamp=probe->now();session_pub->publish(s);pump(160);
  EXPECT_EQ(count,0);EXPECT_GE(sut->statistics().invalidated,1u);
}
TEST_F(CloudContract, SessionTokenChangeInvalidatesInFlightResult) {
  delayed(120,true);
  auto session_pub=probe->create_publisher<astribot_perception_msgs::msg::CameraSessionState>(
    "/cloud_test/session",rclcpp::QoS(1).reliable().transient_local());pump(40);
  astribot_perception_msgs::msg::CameraSessionState s;
  s.active=true;s.session_token="one";s.owner_id="task";s.execution_id="execution";
  s.header.stamp=s.activated_at=probe->now();s.valid_until=probe->now()+rclcpp::Duration::from_seconds(1.);
  session_pub->publish(s);pump(10);image.header.stamp=info.header.stamp=probe->now();send();
  s.session_token="two";s.header.stamp=s.activated_at=probe->now();session_pub->publish(s);pump(160);
  EXPECT_EQ(count,0);EXPECT_GE(sut->statistics().invalidated,1u);
}
TEST_F(CloudContract, MalformedSessionTimesCannotCrashWorkerOrAllowCloud) {
  delayed(0,true);
  auto session_pub=probe->create_publisher<astribot_perception_msgs::msg::CameraSessionState>(
    "/cloud_test/session",rclcpp::QoS(1).reliable().transient_local());pump(40);
  for(int field=0;field<3;++field){
    astribot_perception_msgs::msg::CameraSessionState s;
    s.active=true;s.session_token="one";s.owner_id="task";s.execution_id="execution";
    s.header.stamp=s.activated_at=probe->now();s.valid_until=probe->now()+rclcpp::Duration::from_seconds(1.);
    if(field==0)s.header.stamp.sec=-1;
    if(field==1)s.activated_at.sec=-1;
    if(field==2)s.valid_until.sec=-1;
    session_pub->publish(s);EXPECT_NO_THROW(pump(10));
    image.header.stamp=info.header.stamp=probe->now();EXPECT_NO_THROW(send());EXPECT_EQ(count,0);
  }
}
TEST_F(CloudContract, SessionObservationDeadlineDoesNotSuppressResult) {
  delayed(120,true);
  auto session_pub=probe->create_publisher<astribot_perception_msgs::msg::CameraSessionState>(
    "/cloud_test/session",rclcpp::QoS(1).reliable().transient_local());pump(40);
  astribot_perception_msgs::msg::CameraSessionState s;
  s.active=true;s.session_token="one";s.owner_id="task";s.execution_id="execution";
  s.header.stamp=s.activated_at=probe->now();s.valid_until=probe->now()+rclcpp::Duration::from_seconds(.08);
  session_pub->publish(s);pump(10);image.header.stamp=info.header.stamp=probe->now();send();pump(160);
  EXPECT_EQ(count,1);
}

TEST_F(CloudContract, LatestContractFutureDepthUsesNearestInfoAndRejectsOldOverwrite) {
 const auto t=probe->now()+rclcpp::Duration::from_seconds(30.);image.header.stamp=t;
 info.header.stamp=t-rclcpp::Duration::from_seconds(2.);send();ASSERT_EQ(count,1);
 EXPECT_EQ(cloud->header.stamp,image.header.stamp);const auto capture=cloud->header.stamp;
 image.header.stamp=info.header.stamp=t-rclcpp::Duration::from_seconds(10.);send();EXPECT_EQ(count,1);
 EXPECT_EQ(cloud->header.stamp,capture);
}
TEST_F(CloudContract, LatestContractDelayedProcessingDoesNotExpireValidInput) {
 delayed(300);image.header.stamp=info.header.stamp=probe->now()-rclcpp::Duration::from_seconds(5.);
 send();pump(340);ASSERT_EQ(count,1);EXPECT_EQ(cloud->header.stamp,image.header.stamp);
}
