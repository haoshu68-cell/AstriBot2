#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <thread>
#include <unistd.h>
#define main camera_postprocess_main
#include "../src/camera_calibration_postprocess_node.cpp"
#undef main

using namespace std::chrono_literals;

class CameraPostprocessFrame : public ::testing::Test {
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void check(const std::string & output_frame) {
    const auto profile = std::filesystem::temp_directory_path() /
      ("astribot_camera_frame_" + std::to_string(getpid()) + ".yaml");
    {std::ofstream file(profile); file << "width: 4\nheight: 3\nintrinsics:\n  color: [10, 0, 1.5, 0, 10, 1, 0, 0, 1]\ndistortion: [0, 0, 0, 0, 0]\n";}
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("profile", profile.string()),
      rclcpp::Parameter("input_image", "/frame_test/color_in"),
      rclcpp::Parameter("output_image", "/frame_test/color_out"),
      rclcpp::Parameter("input_depth", "/frame_test/depth_in"),
      rclcpp::Parameter("output_depth", "/frame_test/depth_out"),
      rclcpp::Parameter("input_info", "/frame_test/info_in"),
      rclcpp::Parameter("output_info", "/frame_test/info_out"),
      rclcpp::Parameter("output_frame", output_frame)});
    auto sut = std::make_shared<astribot_s1_autonomy::CameraCalibrationPostprocess>(options);
    std::filesystem::remove(profile);
    auto probe = std::make_shared<rclcpp::Node>("camera_frame_probe");
    const auto qos = rclcpp::SensorDataQoS();
    auto color_pub = probe->create_publisher<sensor_msgs::msg::Image>("/frame_test/color_in", qos);
    auto depth_pub = probe->create_publisher<sensor_msgs::msg::Image>("/frame_test/depth_in", qos);
    auto info_pub = probe->create_publisher<sensor_msgs::msg::CameraInfo>("/frame_test/info_in", qos);
    sensor_msgs::msg::Image::ConstSharedPtr color_out, depth_out;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_out;
    auto color_sub = probe->create_subscription<sensor_msgs::msg::Image>("/frame_test/color_out", qos,
      [&](sensor_msgs::msg::Image::ConstSharedPtr msg){color_out=msg;});
    auto depth_sub = probe->create_subscription<sensor_msgs::msg::Image>("/frame_test/depth_out", qos,
      [&](sensor_msgs::msg::Image::ConstSharedPtr msg){depth_out=msg;});
    auto info_sub = probe->create_subscription<sensor_msgs::msg::CameraInfo>("/frame_test/info_out", qos,
      [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg){info_out=msg;});
    sensor_msgs::msg::Image color;
    color.header.frame_id = "robot/link/sensor"; color.header.stamp.sec = 12;
    color.width=4; color.height=3; color.encoding="rgb8"; color.step=12; color.data.assign(36, 17);
    auto depth=color; depth.encoding="16UC1"; depth.step=8; depth.data.assign(24, 3);
    sensor_msgs::msg::CameraInfo info; info.header=color.header;
    rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(sut); executor.add_node(probe);
    const auto deadline=std::chrono::steady_clock::now()+2s;
    while ((!color_out || !depth_out || !info_out) && std::chrono::steady_clock::now()<deadline) {
      color_pub->publish(color); depth_pub->publish(depth); info_pub->publish(info);
      executor.spin_some(); std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(color_out); ASSERT_TRUE(depth_out); ASSERT_TRUE(info_out);
    const auto expected=output_frame.empty()?color.header.frame_id:output_frame;
    EXPECT_EQ(color_out->header.frame_id, expected);
    EXPECT_EQ(depth_out->header.frame_id, expected);
    EXPECT_EQ(info_out->header.frame_id, expected);
    EXPECT_EQ(color_out->header.stamp, color.header.stamp);
    EXPECT_EQ(depth_out->header.stamp, depth.header.stamp);
    EXPECT_EQ(info_out->header.stamp, info.header.stamp);
    EXPECT_EQ(color_out->data, color.data); EXPECT_EQ(depth_out->data, depth.data);
  }
};
TEST_F(CameraPostprocessFrame, DeclaredOpticalFrameAppliesToAllStreams) {check("head_rgbd_camera_optical_frame");}
TEST_F(CameraPostprocessFrame, UnconfiguredFramePreservesInput) {check("");}
