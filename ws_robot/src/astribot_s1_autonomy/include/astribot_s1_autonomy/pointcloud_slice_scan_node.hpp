// Copyright 2026 Astribot.
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

  void declareParameters();
  /// 从参数服务器读取全部参数到成员变量。返回 false 表示参数非法。
  bool loadParameters(std::string & error);
  /// 动态调参回调。只接受能安全热更新的参数；非法值一律拒绝并说明原因。
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters);

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  /// 工作线程主循环。
  void workerLoop();
  /// 处理一帧点云。返回 false 表示本帧被丢弃（异常已在内部打过日志）。
  bool processCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  /// 输入看门狗：长时间收不到点云时按策略处理。
  void watchdogCallback();

  /// 查询 cloud_frame → base_frame。失败返回 false（不抛异常）。
  bool lookupCloudTransform(
    const std::string & cloud_frame,
    const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & out);
  /// 用当前 TF 把各连杆链解析成 base_frame 下的胶囊体。
  /// 拿不到某个 frame 时跳过对应胶囊并计数，不影响其余部分。
  std::vector<FilterCapsule> resolveSelfFilterCapsules(const rclcpp::Time & stamp);

  void publishScan(const ProjectionResult & result, const rclcpp::Time & stamp);
  /// 重发上一帧（仅刷新时间戳），用于 kHoldLast 策略。
  void publishFilteredCloud(
    const std::vector<SlicePoint> & kept, const rclcpp::Time & stamp);
  void republishLastScan();
  /// 无效帧的**统一**出口。所有"这一帧不能当有效帧发"的分支都要走这里，
  /// 而不是各自调 republishLastScan() —— 分散调用就没法给 hold_last 上限计数，
  /// 而没有上限的 hold_last 会让陈旧对所有时效性判据隐身（见 hold_last_max_frames_）。
  ///
  /// \param reason 供日志用的原因短语。
  /// \return 恒为 false，方便调用处直接 `return handleInvalidFrame(...)`。
  bool handleInvalidFrame(const char * reason);
  void publishSliceMarkers(
    const std::vector<SlicePoint> & kept_points,
    const std::vector<SlicePoint> & self_points,
    const std::vector<FilterCapsule> & capsules,
    const rclcpp::Time & stamp);

  std::string input_cloud_topic_;
  std::string output_scan_topic_;
  std::string marker_topic_;
  std::string base_frame_;
  double tf_timeout_sec_{0.0};
  /// **单帧**所有连杆 TF 查询的总等待预算(s)。
  double tf_total_budget_sec_{0.0};
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
  /// hold_last 最多连续重发多少帧，超过就转成停止输出。
  int hold_last_max_frames_{0};
  /// 已连续重发的帧数。归零时机：任意一帧成功发布。
  int hold_last_streak_{0};

  std::vector<SelfFilterChainConfig> self_filter_chains_;

  SliceProjector projector_;
  SelfFilter self_filter_;
  /// projector_ / self_filter_ / 参数成员的保护锁（动态调参与工作线程并发访问）。
  std::mutex config_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::atomic<bool> running_{false};
  std::thread worker_thread_;

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
