#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "astribot_s1_perception_components/vision_validation.hpp"

// Exercise the production producer and consumer through their ROS topics.
// Keeping the nodes in this test process lets the test control both ROS clocks
// without publishing /clock into another session.
#define main health_epoch_producer_main
#include "../src/camera_health_node.cpp"
#undef main
#define main health_epoch_consumer_main
#include "../src/detection_gate_node.cpp"
#undef main

using namespace std::chrono_literals;

class HealthEpochFlow : public ::testing::Test
{
protected:
  using Health = astribot_perception_msgs::msg::CameraHealth;
  using Detection = astribot_perception_msgs::msg::Detection2D;

  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    rclcpp::NodeOptions health_options;
    health_options.parameter_overrides({
      rclcpp::Parameter("camera_id", "epoch_flow_camera"),
      rclcpp::Parameter("frame_id", "epoch_flow_optical"),
      rclcpp::Parameter("color_topic", "/health_epoch_flow/color"),
      rclcpp::Parameter("depth_topic", "/health_epoch_flow/depth"),
      rclcpp::Parameter("info_topic", "/health_epoch_flow/info"),
      rclcpp::Parameter("health_topic", "/health_epoch_flow/health"),
      rclcpp::Parameter("calibration_revision", 1),
      rclcpp::Parameter("max_age_sec", 0.25),
      rclcpp::Parameter("timer_period_sec", 0.005)});
    health_node_ = std::make_shared<CameraHealthNode>(health_options);

    rclcpp::NodeOptions gate_options;
    gate_options.parameter_overrides({
      rclcpp::Parameter("camera_id", "epoch_flow_camera"),
      rclcpp::Parameter("health_topic", "/health_epoch_flow/health"),
      rclcpp::Parameter("input_topic", "/health_epoch_flow/detections"),
      rclcpp::Parameter("output_topic", "/health_epoch_flow/accepted"),
      rclcpp::Parameter("max_detection_age_sec", 0.25),
      rclcpp::Parameter("max_health_age_sec", 0.25)});
    gate_ = std::make_shared<DetectionGate>(gate_options);
    probe_ = std::make_shared<rclcpp::Node>("health_epoch_flow_probe");

    for (const auto & node : std::vector<rclcpp::Node::SharedPtr>{health_node_, gate_, probe_}) {
      ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    }
    set_time(100000000000LL);

    color_pub_ = probe_->create_publisher<sensor_msgs::msg::Image>(
      "/health_epoch_flow/color", rclcpp::SensorDataQoS());
    depth_pub_ = probe_->create_publisher<sensor_msgs::msg::Image>(
      "/health_epoch_flow/depth", rclcpp::SensorDataQoS());
    info_pub_ = probe_->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/health_epoch_flow/info", rclcpp::SensorDataQoS());
    detection_pub_ = probe_->create_publisher<Detection>("/health_epoch_flow/detections", 10);
    health_sub_ = probe_->create_subscription<Health>(
      "/health_epoch_flow/health", rclcpp::QoS(10).reliable().transient_local(),
      [this](Health::ConstSharedPtr message) {health_ = message;});
    accepted_sub_ = probe_->create_subscription<Detection>(
      "/health_epoch_flow/accepted", 10,
      [this](Detection::ConstSharedPtr message) {accepted_.push_back(*message);});
    executor_.add_node(health_node_);
    executor_.add_node(gate_);
    executor_.add_node(probe_);

    // Discovery time is outside the measured, sub-250 ms fault/recovery flow.
    ASSERT_TRUE(wait_until([this]() {
      return color_pub_->get_subscription_count() == 1 &&
             depth_pub_->get_subscription_count() == 1 &&
             info_pub_->get_subscription_count() == 1 &&
             detection_pub_->get_subscription_count() == 1 &&
             health_node_->count_subscribers("/health_epoch_flow/health") == 2 &&
             accepted_sub_->get_publisher_count() == 1;
    }, 2s));

    color_.header.frame_id = "epoch_flow_optical";
    color_.width = color_.height = 10;
    color_.encoding = "rgb8";
    color_.step = 30;
    color_.data.resize(300);
    depth_ = color_;
    depth_.encoding = "16UC1";
    depth_.step = 20;
    depth_.data.resize(200);
    info_.header.frame_id = color_.header.frame_id;
    info_.width = info_.height = 10;
    info_.k = {100., 0., 4.5, 0., 100., 4.5, 0., 0., 1.};
  }

  void TearDown() override
  {
    if (health_node_) {executor_.remove_node(health_node_);}
    if (gate_) {executor_.remove_node(gate_);}
    if (probe_) {executor_.remove_node(probe_);}
  }

  void set_time(int64_t stamp_ns)
  {
    for (const auto & node : std::vector<rclcpp::Node::SharedPtr>{health_node_, gate_, probe_}) {
      ASSERT_EQ(
        rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), stamp_ns), RCL_RET_OK);
    }
  }

  bool wait_until(const std::function<bool()> & predicate, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
      executor_.spin_some();
      if (predicate()) {return true;}
      std::this_thread::sleep_for(1ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return predicate();
  }

  void pump(std::chrono::milliseconds duration)
  {
    (void)wait_until([]() {return false;}, duration);
  }

  void publish_triple(int64_t stamp_ns)
  {
    color_.header.stamp = depth_.header.stamp = info_.header.stamp =
      rclcpp::Time(stamp_ns, RCL_ROS_TIME);
    color_pub_->publish(color_);
    depth_pub_->publish(depth_);
    info_pub_->publish(info_);
  }

  std::shared_ptr<CameraHealthNode> health_node_;
  std::shared_ptr<DetectionGate> gate_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_, depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<Detection>::SharedPtr detection_pub_;
  rclcpp::Subscription<Health>::SharedPtr health_sub_;
  rclcpp::Subscription<Detection>::SharedPtr accepted_sub_;
  sensor_msgs::msg::Image color_, depth_;
  sensor_msgs::msg::CameraInfo info_;
  Health::ConstSharedPtr health_;
  std::vector<Detection> accepted_;
};

TEST_F(HealthEpochFlow, ShortFaultRecoveryRejectsOldEpochWhileDetectionIsStillFresh)
{
  // This fails if the producer reuses its pre-fault epoch, or the consumer
  // stops comparing epochs. No synthetic CameraHealth is published by the test.
  const auto flow_started = std::chrono::steady_clock::now();
  set_time(100100000000LL);
  publish_triple(100100000000LL);
  pump(5ms);
  set_time(100150000000LL);
  publish_triple(100150000000LL);
  ASSERT_TRUE(wait_until([this]() {return health_ && health_->valid;}, 40ms));
  const auto old_epoch = health_->source_epoch;

  Detection detection;
  detection.header.stamp = probe_->now();
  detection.header.frame_id = "epoch_flow_optical";
  detection.source_camera_id = "epoch_flow_camera";
  detection.source_epoch = old_epoch;
  detection.detection_id = detection.object_id = "same_fresh_observation";
  detection.model_name = "fixture";
  detection.model_revision = "v1";
  detection.calibration_revision = 1;
  detection.confidence = 0.9;
  detection.width = detection.height = 10;
  detection_pub_->publish(detection);
  ASSERT_TRUE(wait_until([this]() {return accepted_.size() == 1;}, 40ms));
  ASSERT_EQ(accepted_.front().source_epoch, old_epoch);
  accepted_.clear();

  set_time(100170000000LL);
  color_.header.stamp = probe_->now();
  color_.header.frame_id = "wrong_optical_frame";
  color_pub_->publish(color_);
  ASSERT_TRUE(wait_until([this]() {
    return health_ && !health_->valid && health_->reason_code == "CAMERA_FRAME_MISMATCH";
  }, 40ms));
  const auto invalid_epoch = health_->source_epoch;
  EXPECT_NE(invalid_epoch, old_epoch);

  // A complete frame captured after the fault watermark authorizes recovery.
  set_time(100190000000LL);
  color_.header.frame_id = "epoch_flow_optical";
  publish_triple(100190000000LL);
  ASSERT_TRUE(wait_until([this]() {return health_ && health_->valid;}, 40ms));
  const auto recovered_epoch = health_->source_epoch;
  ASSERT_NE(recovered_epoch, old_epoch);
  EXPECT_EQ(recovered_epoch, invalid_epoch);
  EXPECT_EQ(rclcpp::Time(health_->capture_stamp).nanoseconds(), 100190000000LL);

  // Check the production rejection reason against the actual recovered health,
  // rather than interpreting silence on the output topic as proof of rejection.
  // Camera identity is unchanged, so CAMERA_SOURCE_MISMATCH means epoch mismatch.
  EXPECT_EQ((probe_->now() - rclcpp::Time(detection.header.stamp)).nanoseconds(), 40000000LL);
  EXPECT_LT(probe_->now(), rclcpp::Time(health_->valid_until));
  ASSERT_EQ(detection.source_camera_id, health_->camera_id);
  ASSERT_EQ(
    astribot::vision::detection_error(
      detection, health_.get(), probe_->now().nanoseconds(), 0.25, 0.25, 0.25),
    "CAMERA_SOURCE_MISMATCH");

  // Changing only the epoch clears the contract error. This counterfactual
  // checks that the old timestamp and all other metadata are still admissible;
  // it is not published as a new observation.
  auto current_detection = detection;
  current_detection.source_epoch = recovered_epoch;
  ASSERT_TRUE(astribot::vision::detection_error(
      current_detection, health_.get(), probe_->now().nanoseconds(), 0.25, 0.25, 0.25).empty());

  // The positive ROS control is a genuinely newer capture from the recovered
  // source. The producer's capture stamp is carried through the real gate.
  current_detection.header.stamp = health_->capture_stamp;
  ASSERT_GT(
    rclcpp::Time(current_detection.header.stamp).nanoseconds(),
    rclcpp::Time(detection.header.stamp).nanoseconds());
  detection_pub_->publish(detection);
  detection_pub_->publish(current_detection);
  ASSERT_TRUE(wait_until([this]() {return !accepted_.empty();}, 40ms));
  pump(10ms);

  ASSERT_EQ(accepted_.size(), 1u);
  EXPECT_EQ(accepted_.front().source_epoch, recovered_epoch);
  EXPECT_EQ(accepted_.front().header.stamp, current_detection.header.stamp);
  EXPECT_TRUE(health_->valid) << health_->reason_code;
  EXPECT_LT(
    std::chrono::duration<double>(std::chrono::steady_clock::now() - flow_started).count(),
    0.25) << "Flow exceeded the camera freshness budget; this run cannot exclude wall expiry";
}
