// Copyright 2026 Astribot.
// C++ replacement for the simulation camera calibration post-process node.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>

namespace astribot_s1_autonomy
{

class CameraCalibrationPostprocess final : public rclcpp::Node
{
public:
  explicit CameraCalibrationPostprocess(const rclcpp::NodeOptions & options)
  : Node("camera_calibration_postprocess", options)
  {
    const std::string profile = declare_parameter<std::string>("profile", "");
    input_image_topic_ = declare_parameter<std::string>("input_image", "");
    output_image_topic_ = declare_parameter<std::string>("output_image", "");
    input_info_topic_ = declare_parameter<std::string>("input_info", "");
    output_info_topic_ = declare_parameter<std::string>("output_info", "");
    input_depth_topic_ = declare_parameter<std::string>("input_depth", "");
    output_depth_topic_ = declare_parameter<std::string>("output_depth", "");
    output_frame_ = declare_parameter<std::string>("output_frame", "");

    if (profile.empty() || input_image_topic_.empty() || output_image_topic_.empty() ||
      input_info_topic_.empty() || output_info_topic_.empty())
    {
      throw std::invalid_argument(
              "profile, input_image/output_image and input_info/output_info are required");
    }
    loadProfile(profile);

    const auto qos = rclcpp::SensorDataQoS();
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(output_info_topic_, qos);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(output_image_topic_, qos);
    if (!input_depth_topic_.empty() && !output_depth_topic_.empty()) {
      depth_pub_ = create_publisher<sensor_msgs::msg::Image>(output_depth_topic_, qos);
    }

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      input_info_topic_, qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {publishInfo(*msg);});
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_image_topic_, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {remapAndPublish(*msg, false);});
    if (depth_pub_) {
      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        input_depth_topic_, qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {remapAndPublish(*msg, true);});
    }

    RCLCPP_INFO(
      get_logger(), "loaded %s (%dx%d), C++ remap active: image=%s depth=%s",
      profile.c_str(), width_, height_, output_image_topic_.c_str(),
      depth_pub_ ? output_depth_topic_.c_str() : "disabled");
  }

private:
  static std::vector<double> readVector(const YAML::Node & node, const char * key)
  {
    const auto value = node[key];
    if (!value || !value.IsSequence()) {
      throw std::invalid_argument(std::string("camera profile missing sequence: ") + key);
    }
    return value.as<std::vector<double>>();
  }

  void loadProfile(const std::string & path)
  {
    const YAML::Node cfg = YAML::LoadFile(path);
    if (!cfg["width"] || !cfg["height"] || !cfg["intrinsics"] ||
      !cfg["intrinsics"]["color"])
    {
      throw std::invalid_argument("camera profile missing width/height/intrinsics.color");
    }
    width_ = cfg["width"].as<int>();
    height_ = cfg["height"].as<int>();
    if (width_ <= 0 || height_ <= 0) {
      throw std::invalid_argument("camera profile width/height must be positive");
    }

    const auto k = readVector(cfg["intrinsics"], "color");
    if (k.size() != 9U || k[0] <= 0.0 || k[4] <= 0.0) {
      throw std::invalid_argument("camera profile intrinsics.color must contain a valid 3x3 K");
    }
    intrinsics_ = k;
    distortion_ = cfg["distortion"] ? readVector(cfg, "distortion") : std::vector<double>(5, 0.0);
    if (distortion_.size() != 5U) {
      throw std::invalid_argument("camera profile distortion must contain five values");
    }
    if (std::all_of(distortion_.begin(), distortion_.end(), [](double v) {return std::abs(v - 1.0) < 1e-12;})) {
      std::fill(distortion_.begin(), distortion_.end(), 0.0);
    }

    const cv::Mat camera_matrix(3, 3, CV_64F, intrinsics_.data());
    const cv::Mat distortion(1, 5, CV_64F, distortion_.data());
    cv::initUndistortRectifyMap(
      camera_matrix, distortion, cv::Mat(), camera_matrix,
      cv::Size(width_, height_), CV_32FC1, map_x_, map_y_);
  }

  void publishInfo(const sensor_msgs::msg::CameraInfo & input)
  {
    auto output = input;
    if (!output_frame_.empty()) {
      output.header.frame_id = output_frame_;
    }
    output.width = static_cast<uint32_t>(width_);
    output.height = static_cast<uint32_t>(height_);
    for (std::size_t i = 0; i < intrinsics_.size(); ++i) {
      output.k[i] = intrinsics_[i];
    }
    output.d.assign(distortion_.begin(), distortion_.end());
    output.distortion_model = "plumb_bob";
    info_pub_->publish(std::move(output));
  }

  void remapAndPublish(const sensor_msgs::msg::Image & input, bool depth)
  {
    try {
      const auto image = cv_bridge::toCvCopy(input, input.encoding);
      if (!image || image->image.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "empty %s image on %s", depth ? "depth" : "color", input.header.frame_id.c_str());
        return;
      }
      cv::Mat output;
      cv::remap(
        image->image, output, map_x_, map_y_,
        depth ? cv::INTER_NEAREST : cv::INTER_LINEAR, cv::BORDER_CONSTANT);
      cv_bridge::CvImage converted(input.header, input.encoding, output);
      if (!output_frame_.empty()) {
        converted.header.frame_id = output_frame_;
      }
      const auto message = converted.toImageMsg();
      if (depth) {
        depth_pub_->publish(std::move(*message));
      } else {
        image_pub_->publish(std::move(*message));
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "failed to remap %s image: %s", depth ? "depth" : "color", error.what());
    }
  }

  int width_{0};
  int height_{0};
  std::vector<double> intrinsics_;
  std::vector<double> distortion_;
  cv::Mat map_x_;
  cv::Mat map_y_;

  std::string input_image_topic_;
  std::string output_image_topic_;
  std::string input_info_topic_;
  std::string output_info_topic_;
  std::string input_depth_topic_;
  std::string output_depth_topic_;
  std::string output_frame_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
};

}  // namespace astribot_s1_autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<astribot_s1_autonomy::CameraCalibrationPostprocess>(
      rclcpp::NodeOptions());
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    fprintf(stderr, "camera_calibration_postprocess: %s\n", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
