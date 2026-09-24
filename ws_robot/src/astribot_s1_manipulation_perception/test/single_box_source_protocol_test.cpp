#include "single_box_fixture.hpp"
#include <cstdlib>
#include <thread>
using namespace std::chrono_literals;

class SingleBoxProtocol : public SingleBoxRequest {
protected:
  static void SetUpTestSuite() {
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"100");
    ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"),"1");rclcpp::init(0,nullptr);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
  rclcpp::Node::SharedPtr node,probe;
  rclcpp::executors::SingleThreadedExecutor exec;
  std::unique_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<pp::SingleBoxRequestSource> source;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
  rclcpp::Publisher<astribot_perception_msgs::msg::CameraHealth>::SharedPtr camera_pub;
  rclcpp::Publisher<astribot_perception_msgs::msg::ProjectionHealth>::SharedPtr projection_pub;
  void SetUp() override {
    SingleBoxRequest::SetUp();
    node=std::make_shared<rclcpp::Node>("m3_single_box_source_test");
    probe=std::make_shared<rclcpp::Node>("m3_single_box_fixture_publishers");
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
    set_time(10100000000LL);
    buffer=std::make_unique<tf2_ros::Buffer>(node->get_clock());
    ASSERT_TRUE(buffer->setTransform(tf,"synthetic_capture_fixture",true));
    fixture.station_frame="astribot_torso_base";
    pp::SingleBoxTopics topics;topics.color="/m3_capture_test/color";topics.info="/m3_capture_test/info";
    topics.points="/m3_capture_test/points";topics.camera_health="/m3_capture_test/camera";
    topics.projection_health="/m3_capture_test/projection";
    source=std::make_unique<pp::SingleBoxRequestSource>(node,*buffer,topics);
    auto sensor=rclcpp::SensorDataQoS().keep_last(4);
    rgb_pub=probe->create_publisher<sensor_msgs::msg::Image>(topics.color,sensor);
    info_pub=probe->create_publisher<sensor_msgs::msg::CameraInfo>(topics.info,sensor);
    cloud_pub=probe->create_publisher<sensor_msgs::msg::PointCloud2>(topics.points,sensor);
    camera_pub=probe->create_publisher<astribot_perception_msgs::msg::CameraHealth>(topics.camera_health,rclcpp::QoS(10).reliable());
    projection_pub=probe->create_publisher<astribot_perception_msgs::msg::ProjectionHealth>(topics.projection_health,rclcpp::QoS(1).reliable().transient_local());
    exec.add_node(node);exec.add_node(probe);
    const auto until=std::chrono::steady_clock::now()+2s;
    while(std::chrono::steady_clock::now()<until&&
      (rgb_pub->get_subscription_count()!=1||info_pub->get_subscription_count()!=1||
       cloud_pub->get_subscription_count()!=1||camera_pub->get_subscription_count()!=1||projection_pub->get_subscription_count()!=1))spin(2ms);
    ASSERT_EQ(rgb_pub->get_subscription_count(),1u);ASSERT_EQ(cloud_pub->get_subscription_count(),1u);
    ASSERT_EQ(info_pub->get_subscription_count(),1u);ASSERT_EQ(camera_pub->get_subscription_count(),1u);
    ASSERT_EQ(projection_pub->get_subscription_count(),1u);
  }
  void TearDown() override {
    exec.remove_node(node);exec.remove_node(probe);source.reset();buffer.reset();node.reset();probe.reset();
  }
  void set_time(int64_t t) {ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),t),RCL_RET_OK);}
  void spin(std::chrono::milliseconds duration) {
    const auto until=std::chrono::steady_clock::now()+duration;
    do {exec.spin_some();std::this_thread::sleep_for(1ms);}while(std::chrono::steady_clock::now()<until);
  }
  void health() {++frame.projection.sequence;camera_pub->publish(frame.camera);projection_pub->publish(frame.projection);spin(10ms);}
  void frames() {rgb_pub->publish(*rgb);info_pub->publish(*info);cloud_pub->publish(*cloud);spin(10ms);}
  void prime() {health();frames();}
  pp::Request capture() {return source->capture(task,fixture,[this]{return source->bind_context(task.context);});}
  std::string capture_error() {
    try {(void)capture();return "ACCEPTED";}catch(const std::exception& e){return e.what();}
  }
  void stamps(int64_t t) {
    rgb->header.stamp=info->header.stamp=cloud->header.stamp=rclcpp::Time(t);
    frame.camera.header.stamp=frame.camera.capture_stamp=rclcpp::Time(t);
    frame.camera.valid_until=rclcpp::Time(t+250000000LL);
    frame.projection.header.stamp=frame.projection.capture_stamp=rclcpp::Time(t);
    frame.projection.valid_until=rclcpp::Time(t+250000000LL);
  }
};

TEST_F(SingleBoxProtocol, RealSubscriptionsProduceBoundRequest) {
  prime();auto result=capture();EXPECT_EQ(result.object_cloud.width,2500u);
  EXPECT_EQ(result.object_cloud.header,rgb->header);EXPECT_EQ(result.capture_camera_info,*info);
  EXPECT_EQ(result.context.source_epoch,frame.camera.source_epoch);
  EXPECT_EQ(result.context.processing_epoch,frame.projection.processing_epoch);
  EXPECT_EQ(result.capture_station_from_camera.header.frame_id,"astribot_torso_base");
  EXPECT_LT(result.result_deadline_steady,std::chrono::steady_clock::now()+5s);
}
TEST_F(SingleBoxProtocol, CalibrationChangeLatchesUntilAuthoritativeRevisionChanges) {
  prime();ASSERT_NO_THROW(capture());
  info->header.stamp.nanosec=20000000;info->k[0]=101;info_pub->publish(*info);spin(10ms);
  EXPECT_EQ(capture_error(),"CAMERA_INFO_CHANGED_WITHOUT_REVISION");
  task.context.calibration_revision=frame.camera.calibration_revision=5;
  stamps(10100000000LL);health();frames();
  auto context=source->bind_context(task.context);
  EXPECT_EQ(context.calibration_revision,5u);EXPECT_EQ(context.camera_info_revision,pp::camera_info_revision(*info));
}
TEST_F(SingleBoxProtocol, ProjectionEpochCannotReuseCachedOrRepeatedFrame) {
  prime();ASSERT_NO_THROW(capture());
  frame.projection.processing_epoch="projector_boot:2";health();
  frames();EXPECT_NE(capture_error(),"ACCEPTED");
  stamps(10050000000LL);health();frames();
  EXPECT_EQ(capture().context.processing_epoch,"projector_boot:2");
}
TEST_F(SingleBoxProtocol, LateReceiptPauseAndDuplicateMessagesDoNotRenewLease) {
  set_time(10200000000LL);prime();
  spin(100ms); // delivery was already 200 ms late; ROS clock remains frozen
  health();frames();
  EXPECT_EQ(capture_error(),"CAMERA_INFO_STALE");
}
TEST_F(SingleBoxProtocol, RollbackRequiresNewOwnerClockEpochAndFreshFrame) {
  prime();ASSERT_NO_THROW(capture());set_time(9100000000LL);
  EXPECT_NE(capture_error(),"ACCEPTED");
  frame.camera.source_epoch="camera_after_rollback";frame.projection.processing_epoch="projector_after_rollback";
  frame.projection.epoch_first_capture_stamp.sec=9;stamps(9100000000LL);health();frames();
  EXPECT_EQ(capture_error(),"CLOCK_EPOCH_NOT_ADVANCED");
  task.context.clock_epoch=3;EXPECT_EQ(capture().context.clock_epoch,3u);
}
TEST_F(SingleBoxProtocol, InitialZeroClockEpochIsValidButRollbackStillRequiresChange) {
  task.context.clock_epoch=0;prime();
  ASSERT_EQ(capture().context.clock_epoch,0u);
  set_time(9100000000LL);EXPECT_NE(capture_error(),"ACCEPTED");
  frame.camera.source_epoch="camera_after_rollback";
  frame.projection.processing_epoch="projector_after_rollback";
  frame.projection.epoch_first_capture_stamp.sec=9;
  stamps(9100000000LL);health();frames();
  EXPECT_EQ(capture_error(),"CLOCK_EPOCH_NOT_ADVANCED");
  task.context.clock_epoch=1;EXPECT_EQ(capture().context.clock_epoch,1u);
}
TEST_F(SingleBoxProtocol, LaterClientCannotResetFrozenOriginalDeadline) {
  prime();auto request=capture();request.result_deadline_steady=std::chrono::steady_clock::now()-1ms;
  auto client_node=std::make_shared<rclcpp::Node>("m3_expired_request_client");
  ASSERT_EQ(rcl_enable_ros_time_override(client_node->get_clock()->get_clock_handle()),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(client_node->get_clock()->get_clock_handle(),10100000000LL),RCL_RET_OK);
  pp::PickPlanningClient client(client_node); // No Action peers or goal dispatch.
  auto result=client.plan(request,[&]{return request.context;},[]{return false;});
  EXPECT_FALSE(result.success);EXPECT_EQ(result.reason,"ORIGINAL_DEADLINE_EXPIRED");EXPECT_TRUE(result.terminal_confirmed);
}
