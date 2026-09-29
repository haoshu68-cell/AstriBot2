#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "astribot_perception_msgs/msg/camera_health.hpp"
#include "astribot_perception_msgs/msg/detection2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "astribot_s1_perception_components/vision_validation.hpp"

class DetectionGate final : public rclcpp::Node
{
public:
  explicit DetectionGate(const rclcpp::NodeOptions & options=rclcpp::NodeOptions())
  : Node("detection_gate",options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/perception/detections");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/perception/valid_detections");
    health_topic_ = declare_parameter<std::string>(
      "health_topic", "/perception/camera_health/head_rgbd");
    camera_id_ = declare_parameter<std::string>("camera_id", "head_rgbd");
    min_confidence_ = declare_parameter<double>("min_confidence", 0.25);
    max_detection_age_sec_ = declare_parameter<double>("max_detection_age_sec", 0.30);
    max_health_age_sec_ = declare_parameter<double>("max_health_age_sec", 0.50);
    require_camera_health_ = declare_parameter<bool>("require_camera_health", true);

    if (!require_camera_health_ || camera_id_.empty() || !std::isfinite(min_confidence_) ||
        min_confidence_<0 || min_confidence_>1 || !std::isfinite(max_detection_age_sec_) ||
        max_detection_age_sec_<=0 || !std::isfinite(max_health_age_sec_) || max_health_age_sec_<=0)
      throw std::invalid_argument("invalid detection gate thresholds");
    publisher_ = create_publisher<astribot_perception_msgs::msg::Detection2D>(
      output_topic_, rclcpp::QoS(10).reliable());
    detection_subscription_ = create_subscription<astribot_perception_msgs::msg::Detection2D>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&DetectionGate::on_detection, this, std::placeholders::_1));
    health_subscription_ = create_subscription<astribot_perception_msgs::msg::CameraHealth>(
      health_topic_, rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&DetectionGate::on_health, this, std::placeholders::_1));
  }

private:
  void check_clock() {
    const auto t=now().nanoseconds();
    if(t<last_clock_) health_.reset();
    last_clock_=t;
  }
  static rclcpp::Time stamp_time(const builtin_interfaces::msg::Time & stamp)
  {
    return rclcpp::Time(stamp, RCL_ROS_TIME);
  }

  void on_health(const astribot_perception_msgs::msg::CameraHealth::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    check_clock();
    health_ = message;
    health_received_=std::chrono::steady_clock::now();
  }

  void reject(const std::string & detection_id, const char * reason)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Detection %s rejected by C++ gate: %s", detection_id.c_str(), reason);
  }

  void on_detection(const astribot_perception_msgs::msg::Detection2D::ConstSharedPtr message)
  {
    if (!message) {
      return;
    }
    astribot_perception_msgs::msg::CameraHealth::ConstSharedPtr health;
    { std::lock_guard<std::mutex> lock(mutex_); check_clock();
      if(std::chrono::duration<double>(std::chrono::steady_clock::now()-health_received_).count()>max_health_age_sec_) {
        reject(message->detection_id,"CAMERA_HEALTH_RECEIVE_TIMEOUT");return;
      }
      health=health_;
    }
    const auto error=astribot::vision::detection_error(*message,health.get(),
      now().nanoseconds(),max_detection_age_sec_,max_health_age_sec_,min_confidence_);
    if (!error.empty()) { reject(message->detection_id,error.c_str()); return; }
    if (message->source_camera_id!=camera_id_) { reject(message->detection_id,"CAMERA_SOURCE_MISMATCH"); return; }
    publisher_->publish(*message);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string health_topic_;
  std::string camera_id_;
  double min_confidence_{0.25};
  double max_detection_age_sec_{0.30};
  double max_health_age_sec_{0.50};
  bool require_camera_health_{true};
  std::mutex mutex_;
  int64_t last_clock_{0};
  std::chrono::steady_clock::time_point health_received_;
  astribot_perception_msgs::msg::CameraHealth::ConstSharedPtr health_;
  rclcpp::Subscription<astribot_perception_msgs::msg::Detection2D>::SharedPtr detection_subscription_;
  rclcpp::Subscription<astribot_perception_msgs::msg::CameraHealth>::SharedPtr health_subscription_;
  rclcpp::Publisher<astribot_perception_msgs::msg::Detection2D>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionGate>());
  rclcpp::shutdown();
  return 0;
}
