// Copyright 2026 Astribot.
//
// Livox CustomMsg → PointCloud2 转换节点（ROS 层）。
// 几何/判定逻辑全在 livox_custom_convert.hpp，本文件只负责订阅、装配、发布。
//
// ════════════════ 它在链路里的位置 ════════════════
//   厂商驱动(xfer_format=1, CustomMsg)
//        /livox/lidar_front ─┐
//        /livox/lidar_back  ─┴─▶ 【本节点】 ─▶ /livox/lidar_front_pc2
//                                              /livox/lidar_back_pc2
//        ─▶ livox_preprocess_node ×2 ─▶ livox_fusion_node
//        ─▶ pointcloud_slice_scan_node ─▶ /scan ─▶ nav2 obstacle_layer
//
// SLAM 继续直接吃 CustomMsg，不受本节点影响 —— 这是刻意的：
// mid360.yaml 的 lidar_type=0 需要 CustomMsg，改驱动的 xfer_format 会打断 SLAM。
//
// ════════════════ 两路的无效点位置不同 ════════════════
// 驱动把外参写进了雷达设备，所以无回波的零点也被外参变换过：
// front 那一路堆在 (0,0,0)，back 那一路堆在外参平移量 (0.001,-0.496,0.084)。
// 因此 null_xyz 是**每路各自配**的参数，不能共用一份。
// 详细危害说明见 livox_custom_convert.hpp 的文件头。
//
// ════════════════ frame_id ════════════════
// 实机两路点云的 header.frame_id 都是 `livox_frame`（驱动 launch 里是一个全局值），
// 而且这并非疏漏：外参已在设备内应用，back 的点本来就在 front 系里。
// 本节点默认**原样透传** frame_id，不改写。URDF 里已补
// `livox_mid360_left → livox_frame` 恒等边，所以下游用 TF 摆放点云是通的。
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
  // 累计统计，用于周期上报
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

  // 复用缓冲，避免每帧分配两万点
  std::vector<LivoxPoint> in_buf_;
  std::vector<ConvertedPoint> out_buf_;
};

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__LIVOX_CUSTOM_TO_PC2_NODE_HPP_
