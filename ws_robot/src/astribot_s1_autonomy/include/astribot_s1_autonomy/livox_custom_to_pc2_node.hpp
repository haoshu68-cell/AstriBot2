// Copyright 2026 Astribot.
#ifndef ASTRIBOT_S1_AUTONOMY__LIVOX_CUSTOM_TO_PC2_NODE_HPP_
#define ASTRIBOT_S1_AUTONOMY__LIVOX_CUSTOM_TO_PC2_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "astribot_s1_autonomy/livox_custom_convert.hpp"

namespace astribot_s1_autonomy
{

/// 一路输入的运行时状态。
struct ConvertChannel
{
  std::string in_topic;
  std::string out_topic;
  LivoxConvertConfig cfg;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
  std::uint64_t frames{0};
  std::uint64_t total_pts{0};
  std::uint64_t kept_pts{0};
  std::uint64_t dropped_null{0};
  std::uint64_t dropped_other{0};
  /// 最近一帧的接收时刻（秒，稳态时钟）。用于判断这一路是不是停了。
  double last_rx_sec{-1.0};
};

class LivoxCustomToPc2Node : public rclcpp::Node
{
public:
  explicit LivoxCustomToPc2Node(const rclcpp::NodeOptions & options);

private:
  /// 声明并读取一路的参数。prefix 形如 "front"。
  ConvertChannel declare_channel(const std::string & prefix,
                                 const std::string & default_in,
                                 const std::string & default_out,
                                 double default_null_x,
                                 double default_null_y,
                                 double default_null_z);

  void on_cloud(std::size_t idx,
                const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);

  void report();

  std::vector<ConvertChannel> channels_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  double report_period_sec_{10.0};
  /// 一路多久没数据就告警（秒）。
  double stale_warn_sec_{2.0};

  std::vector<LivoxPoint> in_buf_;
  std::vector<ConvertedPoint> out_buf_;
};

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__LIVOX_CUSTOM_TO_PC2_NODE_HPP_
