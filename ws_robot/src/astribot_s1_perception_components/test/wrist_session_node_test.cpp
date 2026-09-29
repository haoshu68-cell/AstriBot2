#include <gtest/gtest.h>
#include <thread>
#define main wrist_session_main
#include "../src/wrist_camera_session_node.cpp"
#undef main

using Service = astribot_perception_msgs::srv::SetCameraSession;
using Image = sensor_msgs::msg::Image;
using State = astribot_perception_msgs::msg::CameraSessionState;

class WristSessionNodeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("raw_prefix", "/test_wrist/raw"),
      rclcpp::Parameter("active_prefix", "/test_wrist/active")});
    sut = std::make_shared<WristCameraSession>(opts);
    probe = std::make_shared<rclcpp::Node>("session_node_probe");
    auto * clock = sut->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    set_time(10000000000LL);
    publisher = probe->create_publisher<Image>("/test_wrist/raw/image", rclcpp::SensorDataQoS());
    subscriber = probe->create_subscription<Image>("/test_wrist/active/image", rclcpp::SensorDataQoS(),
      [this](Image::ConstSharedPtr msg) {images.push_back(*msg);});
    status_sub = probe->create_subscription<State>("/perception/camera_session/left_wrist_rgbd",
      rclcpp::QoS(1).transient_local(), [this](State::ConstSharedPtr msg) {state = *msg;});
    client = probe->create_client<Service>("/perception/camera_session/left_wrist_rgbd/set");
    executor.add_node(sut);executor.add_node(probe);pump(60);
  }
  void TearDown() override {executor.remove_node(sut);executor.remove_node(probe);}
  void pump(int milliseconds) {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    do {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    while (std::chrono::steady_clock::now() < end);
  }
  void set_time(int64_t ns) {
    ASSERT_EQ(rcl_set_ros_time_override(sut->get_clock()->get_clock_handle(), ns), RCL_RET_OK);
  }
  Service::Response::SharedPtr request(uint8_t operation, const std::string & token="", double lease=2.) {
    auto req=std::make_shared<Service::Request>();req->header.stamp=sut->now();
    req->camera_id="left_wrist_rgbd";req->owner_id="task";req->execution_id="execution";
    req->request_id=std::to_string(++sequence);req->operation=operation;
    req->session_token=token;req->lease_sec=lease;
    auto future=client->async_send_request(req);
    for(int i=0;i<100 && future.wait_for(std::chrono::seconds(0))!=std::future_status::ready;++i) pump(5);
    if(future.wait_for(std::chrono::seconds(0))!=std::future_status::ready) throw std::runtime_error("service timeout");
    return future.get();
  }
  void send(int64_t stamp) {
    Image msg;msg.header.stamp=rclcpp::Time(stamp);msg.header.frame_id="optical";
    msg.encoding="mono8";msg.width=msg.height=msg.step=1;msg.data={42};publisher->publish(msg);pump(20);
  }
  std::shared_ptr<WristCameraSession> sut;
  rclcpp::Node::SharedPtr probe;rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Publisher<Image>::SharedPtr publisher;rclcpp::Subscription<Image>::SharedPtr subscriber;
  rclcpp::Subscription<State>::SharedPtr status_sub;rclcpp::Client<Service>::SharedPtr client;
  State state;std::vector<Image> images;int sequence=0;
};

TEST_F(WristSessionNodeTest, InactiveAndReleasedSessionsDoNotForward) {
  send(10000000000LL);EXPECT_TRUE(images.empty());
  auto acquired=request(0);ASSERT_TRUE(acquired->accepted);pump(40);
  send(10000000000LL);ASSERT_EQ(images.size(),1u);
  EXPECT_EQ(images[0].header.frame_id,"optical");EXPECT_EQ(images[0].data[0],42);
  ASSERT_TRUE(request(2,acquired->state.session_token)->accepted);
  send(10000000000LL);EXPECT_EQ(images.size(),1u);
  pump(60);EXPECT_FALSE(state.active);
}
TEST_F(WristSessionNodeTest, RejectsOldFutureAndPreviousGenerationFrames) {
  auto acquired=request(0);ASSERT_TRUE(acquired->accepted);pump(40);
  send(9999999999LL);send(10000000001LL);EXPECT_TRUE(images.empty());
  set_time(10300000000LL);send(10000000000LL);EXPECT_TRUE(images.empty());
  send(10300000000LL);ASSERT_EQ(images.size(),1u);
  ASSERT_TRUE(request(2,acquired->state.session_token)->accepted);
  set_time(10400000000LL);ASSERT_TRUE(request(0)->accepted);pump(40);
  send(10300000000LL);EXPECT_EQ(images.size(),1u);
}
TEST_F(WristSessionNodeTest, WallExpiryWorksWhileRosClockIsPaused) {
  ASSERT_TRUE(request(0,"",.1)->accepted);pump(160);
  send(10000000000LL);EXPECT_TRUE(images.empty());EXPECT_FALSE(state.active);
}
TEST_F(WristSessionNodeTest, ImageBeforeMatchingClockTickWaitsThenForwards) {
  ASSERT_TRUE(request(0)->accepted);pump(40);
  send(10001000000LL);EXPECT_TRUE(images.empty());
  set_time(10001000000LL);pump(60);
  ASSERT_EQ(images.size(),1u);
  EXPECT_EQ(rclcpp::Time(images[0].header.stamp).nanoseconds(),10001000000LL);
}
TEST_F(WristSessionNodeTest, DeferredImageExpiresOnWallClock) {
  ASSERT_TRUE(request(0)->accepted);pump(40);
  send(10001000000LL);pump(260);
  set_time(10001000000LL);pump(60);EXPECT_TRUE(images.empty());
}
TEST_F(WristSessionNodeTest, ReleaseClearsDeferredFrames) {
  auto acquired=request(0);ASSERT_TRUE(acquired->accepted);pump(40);
  send(10001000000LL);EXPECT_TRUE(images.empty());
  ASSERT_TRUE(request(2,acquired->state.session_token)->accepted);
  set_time(10001000000LL);ASSERT_TRUE(request(0)->accepted);pump(60);
  EXPECT_TRUE(images.empty());
}
TEST_F(WristSessionNodeTest, RollbackRevokesSessionAndOldToken) {
  auto acquired=request(0);ASSERT_TRUE(acquired->accepted);pump(40);
  set_time(9000000000LL);send(9000000000LL);pump(60);
  EXPECT_FALSE(state.active);EXPECT_TRUE(images.empty());
  EXPECT_FALSE(request(1,acquired->state.session_token)->accepted);
}
TEST_F(WristSessionNodeTest, NavigationCameraCannotBeDisabledByWristGate) {
  rclcpp::NodeOptions opts;opts.parameter_overrides({rclcpp::Parameter("camera_id","head_rgbd")});
  EXPECT_THROW(std::make_shared<WristCameraSession>(opts),std::invalid_argument);
}
