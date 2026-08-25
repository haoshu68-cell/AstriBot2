// Copyright 2026 Astribot.
//
// 感知模块节点：3D 点云多层高度切片投影 → 2D LaserScan。
//
// 职责边界（有意做得很薄）：
//   本类只负责「取数据 / 查 TF / 发话题 / 处理异常」，
//   真正的算法在 slice_projector.hpp（切片投影融合）和 self_filter.hpp（自身点剔除）里。
//
// 线程模型（对应「消息处理避免阻塞回调线程，耗时算法放到独立工作线程」）：
//   订阅回调只做「校验 + 存入单槽缓冲 + 唤醒工作线程」，永不做重活；
//   工作线程取最新一帧做 PCL 滤波/切片/投影。单槽缓冲意味着处理不过来时
//   自动丢弃中间帧、永远处理最新数据，这对导航避障比「排队处理旧帧」更合理。
#ifndef ASTRIBOT_S1_AUTONOMY__POINTCLOUD_SLICE_SCAN_NODE_HPP_
#define ASTRIBOT_S1_AUTONOMY__POINTCLOUD_SLICE_SCAN_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "astribot_s1_autonomy/self_filter.hpp"
#include "astribot_s1_autonomy/slice_projector.hpp"

namespace astribot_s1_autonomy
{

/// 一条「连杆链」的自身点剔除配置。链上相邻两个 frame 之间生成一个胶囊体。
struct SelfFilterChainConfig
{
  std::string name;
  /// 按顺序排列的 TF frame 名。至少 2 个才能连成线段；
  /// 只给 1 个时退化成以该 frame 原点为心的球。
  std::vector<std::string> frames;
  double radius{0.0};
};

class PointcloudSliceScanNode : public rclcpp::Node
{
public:
  explicit PointcloudSliceScanNode(const rclcpp::NodeOptions & options);
  ~PointcloudSliceScanNode() override;

  // 禁用拷贝/移动：本类持有工作线程和 ROS 句柄。
  PointcloudSliceScanNode(const PointcloudSliceScanNode &) = delete;
  PointcloudSliceScanNode & operator=(const PointcloudSliceScanNode &) = delete;
  PointcloudSliceScanNode(PointcloudSliceScanNode &&) = delete;
  PointcloudSliceScanNode & operator=(PointcloudSliceScanNode &&) = delete;

private:
  /// 「输入点云无效时」的行为，对应需求里的两种可选处理。
  enum class InvalidInputPolicy
  {
    kHoldLast,   ///< 继续重发上一帧有效 scan（时间戳刷新），保持 costmap 不闪
    kStopOutput  ///< 干脆不发，让下游按「传感器超时」处理
  };

  // ---------------- 参数 ----------------
  void declareParameters();
  /// 从参数服务器读取全部参数到成员变量。返回 false 表示参数非法。
  bool loadParameters(std::string & error);
  /// 动态调参回调。只接受能安全热更新的参数；非法值一律拒绝并说明原因。
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters);

  // ---------------- 数据流 ----------------
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  /// 工作线程主循环。
  void workerLoop();
  /// 处理一帧点云。返回 false 表示本帧被丢弃（异常已在内部打过日志）。
  bool processCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  /// 输入看门狗：长时间收不到点云时按策略处理。
  void watchdogCallback();

  // ---------------- TF ----------------
  /// 查询 cloud_frame → base_frame。失败返回 false（不抛异常）。
  bool lookupCloudTransform(
    const std::string & cloud_frame,
    const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & out);
  /// 用当前 TF 把各连杆链解析成 base_frame 下的胶囊体。
  /// 拿不到某个 frame 时跳过对应胶囊并计数，不影响其余部分。
  std::vector<FilterCapsule> resolveSelfFilterCapsules(const rclcpp::Time & stamp);

  // ---------------- 输出 ----------------
  void publishScan(const ProjectionResult & result, const rclcpp::Time & stamp);
  /// 重发上一帧（仅刷新时间戳），用于 kHoldLast 策略。
  void publishFilteredCloud(
    const std::vector<SlicePoint> & kept, const rclcpp::Time & stamp);
  void republishLastScan();
  void publishSliceMarkers(
    const std::vector<SlicePoint> & kept_points,
    const std::vector<SlicePoint> & self_points,
    const std::vector<FilterCapsule> & capsules,
    const rclcpp::Time & stamp);

  // ---------------- 成员 ----------------
  // 参数
  std::string input_cloud_topic_;
  std::string output_scan_topic_;
  std::string marker_topic_;
  std::string base_frame_;
  double tf_timeout_sec_{0.0};
  double max_cloud_age_sec_{0.0};
  double tf_time_tolerance_sec_{0.0};
  bool enable_voxel_filter_{true};
  double voxel_leaf_size_{0.0};
  bool enable_outlier_filter_{true};
  int outlier_mean_k_{0};
  double outlier_stddev_mul_{0.0};
  int min_valid_points_{0};
  double watchdog_period_sec_{0.0};
  double input_timeout_sec_{0.0};
  bool publish_markers_{true};
  int marker_point_stride_{1};
  InvalidInputPolicy invalid_input_policy_{InvalidInputPolicy::kHoldLast};

  std::vector<SelfFilterChainConfig> self_filter_chains_;

  // 算法核心
  SliceProjector projector_;
  SelfFilter self_filter_;
  /// projector_ / self_filter_ / 参数成员的保护锁（动态调参与工作线程并发访问）。
  std::mutex config_mutex_;

  // ROS 句柄
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  // 剔除自身点后的点云。让给 SLAM 出单层 /scan 的 pointcloud_to_laserscan
  // 复用同一套自滤，而不是各自实现一遍（自滤一分叉就会一条链干净一条脏）。
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // 单槽缓冲 + 工作线程
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::atomic<bool> running_{false};
  std::thread worker_thread_;

  // 状态
  sensor_msgs::msg::LaserScan::SharedPtr last_valid_scan_;
  std::mutex last_scan_mutex_;
  rclcpp::Time last_input_time_;
  /// 节点构造完成的时刻。用于「从未收到过输入」的告警判据。
  rclcpp::Time node_start_time_;
  std::atomic<uint64_t> dropped_frame_count_{0U};
  std::atomic<uint64_t> processed_frame_count_{0U};
  /// 动态调参脏标记：参数回调只置位，工作线程在下一帧统一重载，
  /// 保证配置切换是原子的（不会出现半套新参数半套旧参数）。
  std::atomic<bool> config_dirty_{false};
  bool input_timeout_warned_{false};
};

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__POINTCLOUD_SLICE_SCAN_NODE_HPP_
