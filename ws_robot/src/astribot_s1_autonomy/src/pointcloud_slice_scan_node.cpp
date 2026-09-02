// Copyright 2026 Astribot.
#include "astribot_s1_autonomy/pointcloud_slice_scan_node.hpp"

// cpplint 把 Eigen/PCL 的头当作 "C system header"，要求排在 C++ 标准库之前，
// 因此这里刻意先放它们，再放 <algorithm> 等标准库头。
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "tf2/exceptions.h"

namespace astribot_s1_autonomy
{

namespace
{
/// 日志限流周期(ms)。TF 抖动/点云空帧这类异常往往连续发生，
/// 不限流会把日志刷爆、反而看不到关键信息。
constexpr int kLogThrottleMs = 2000;
/// 允许配置的切片层数上限，纯防御性数值（防止 YAML 写出上千层）。
constexpr std::size_t kMaxSliceCount = 32U;
/// 需求明确禁止单层切片，这里是硬性下限。
constexpr std::size_t kMinSliceCount = 2U;

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
}  // namespace

PointcloudSliceScanNode::PointcloudSliceScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_slice_scan_node", options),
  last_input_time_(0, 0, RCL_ROS_TIME),
  node_start_time_(0, 0, RCL_ROS_TIME)
{
  declareParameters();

  std::string error;
  if (!loadParameters(error)) {
    // 参数非法时不崩溃：报 ERROR 并保持节点存活，
    // 让使用者可以用 ros2 param set 改完再热更新（对应「参数读取失败不允许崩溃」）。
    RCLCPP_ERROR(
      get_logger(),
      "参数校验失败，节点将保持空转直到参数被修正: %s", error.c_str());
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // !!! 实测踩坑（非常隐蔽，务必保留这两行的组合）!!!
  // 本节点在**自己的工作线程**里带 timeout 调 lookupTransform。
  // 如果 listener 不开独立 spin 线程、且不告诉 buffer「有专线在灌数据」，
  // tf2 会直接报 "Do not call canTransform or lookupTransform with a timeout
  // unless you are using another thread for populating data"，并且**每次带超时的
  // 查询都必然失败**。
  // 症状极具误导性：/tf_static 里的静态变换（雷达→本体）照样查得到，
  // 于是点云投影一切正常、scan 有输出；但 robot_state_publisher 发在 /tf 上的
  // 动态变换（双臂 27 个连杆）100% 查不到，导致自身点剔除形同虚设——
  // 机械臂在 costmap 里变成幽灵障碍物，而日志只有一条不起眼的 WARN。
  //   spin_thread=true         : listener 自带线程和执行器，独立灌 buffer
  //   setUsingDedicatedThread  : 告知 buffer 确有专线，超时语义才生效
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  tf_buffer_->setUsingDedicatedThread(true);

  const std::string filtered_topic = get_parameter("filtered_cloud_topic").as_string();
  if (!filtered_topic.empty()) {
    // QoS 与点云输入一致(BEST_EFFORT)：这是高频传感器数据，可靠传输只会堆积延迟。
    filtered_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      filtered_topic, rclcpp::SensorDataQoS());
  }
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
    output_scan_topic_, rclcpp::SensorDataQoS());
  if (publish_markers_) {
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(1));
  }

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {cloudCallback(msg);});

  running_.store(true);
  worker_thread_ = std::thread([this]() {workerLoop();});

  node_start_time_ = now();
  if (watchdog_period_sec_ > 0.0) {
    watchdog_timer_ = create_wall_timer(
      std::chrono::duration<double>(watchdog_period_sec_),
      [this]() {watchdogCallback();});
  }

  param_callback_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {return onParameterChange(params);});

  RCLCPP_INFO(
    get_logger(),
    "感知节点已启动: 输入=%s 输出=%s 基坐标系=%s 切片层数=%zu 自身剔除链=%zu",
    input_cloud_topic_.c_str(), output_scan_topic_.c_str(), base_frame_.c_str(),
    projector_.params().slices.size(), self_filter_chains_.size());
}

PointcloudSliceScanNode::~PointcloudSliceScanNode()
{
  // 先停线程再让成员析构，避免工作线程访问已销毁对象。
  running_.store(false);
  queue_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void PointcloudSliceScanNode::declareParameters()
{
  auto describe = [](const std::string & text) {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = text;
      return d;
    };

  declare_parameter<std::string>("input_cloud_topic", "/lidar/points", describe("输入点云话题"));
  declare_parameter<std::string>(
    "output_scan_topic", "/scan_from_cloud", describe("输出 LaserScan 话题"));
  declare_parameter<std::string>("marker_topic", "~/debug_markers", describe("调试 Marker 话题"));
  declare_parameter<std::string>(
    "base_frame", "astribot_torso_base",
    describe("投影所在的本体坐标系。**本机器人没有 base_link** —— "
             "默认值原为 base_link，yaml 静默失效时会回落到它并让 TF 查询全失败。"));

  declare_parameter<double>("tf_timeout_sec", 0.05, describe("TF 查询超时(s)"));
  declare_parameter<double>("max_cloud_age_sec", 0.30, describe("点云时间戳与当前时间的最大偏差(s)，超出则丢帧"));
  declare_parameter<double>("tf_time_tolerance_sec", 0.10, describe("点云时间戳与 TF 时间戳的最大偏差(s)，超出则丢帧"));

  declare_parameter<bool>("enable_voxel_filter", true, describe("是否启用体素降采样"));
  declare_parameter<double>("voxel_leaf_size", 0.03, describe("体素栅格边长(m)"));
  declare_parameter<bool>("enable_outlier_filter", true, describe("是否启用统计离群点滤波"));
  declare_parameter<int>("outlier_mean_k", 12, describe("统计离群点滤波的近邻数 K"));
  declare_parameter<double>("outlier_stddev_mul", 2.0, describe("统计离群点滤波的标准差倍数阈值"));

  declare_parameter<int>("min_valid_points", 20, describe("预处理后至少需要的点数，不足视为无效帧"));
  declare_parameter<double>("watchdog_period_sec", 0.5, describe("输入看门狗检查周期(s)，<=0 关闭"));
  declare_parameter<double>("input_timeout_sec", 1.0, describe("多久收不到点云算超时(s)"));
  declare_parameter<std::string>(
    "invalid_input_policy", "hold_last",
    describe("输入无效时的策略: hold_last=重发上一帧, stop_output=停止输出"));

  declare_parameter<std::string>(
    "filtered_cloud_topic", "~/cloud_self_filtered",
    describe(
      "剔除自身点后的点云话题。空字符串=不发布。"
      "存在的意义是让**其它**消费者(如给 SLAM 出单层 /scan 的 "
      "pointcloud_to_laserscan)复用同一套自身点剔除，而不是各自再实现一遍 —— "
      "自滤实现一分叉就会出现'一条链干净另一条脏'的情况，实测吃过这个亏"));
  declare_parameter<bool>("publish_markers", true, describe("是否发布调试 Marker"));
  declare_parameter<int>("marker_point_stride", 3, describe("Marker 点抽稀步长，1=全部发布"));

  // ---- 扫描几何 ----
  declare_parameter<double>("scan.angle_min", -M_PI, describe("扫描起始角(rad)"));
  declare_parameter<double>("scan.angle_max", M_PI, describe("扫描终止角(rad)"));
  declare_parameter<double>("scan.angle_increment", M_PI / 360.0, describe("角度分辨率(rad)"));
  declare_parameter<double>("scan.range_min", 0.15, describe("最小有效距离(m)"));
  declare_parameter<double>("scan.range_max", 12.0, describe("最大有效距离(m)"));
  declare_parameter<double>("scan.scan_time", 0.1, describe("填入 LaserScan.scan_time 的值(s)"));
  declare_parameter<std::string>(
    "scan.no_return_mode", "range_max",
    describe(
      "某方向无障碍时填什么: range_max=填最大距离(需求规定的默认行为), "
      "infinity=填 inf(与既有 pointcloud_to_laserscan 的 use_inf:=true 行为一致)"));

  // ---- 多层切片 ----
  // ROS2 参数不支持「字典数组」，因此用「名字列表 + 名字前缀嵌套键」的惯用法，
  // 这样每一项依然是独立参数，可被 ros2 param set 动态修改。
  declare_parameter<std::vector<std::string>>(
    "slice_names", std::vector<std::string>{"ground_near", "low", "mid", "high"},
    describe("切片层名列表，顺序即融合顺序；至少 2 层"));

  // ---- 自身点剔除 ----
  declare_parameter<bool>("self_filter.footprint.enabled", true, describe("是否启用底盘足迹圆柱剔除"));
  declare_parameter<double>("self_filter.footprint.radius", 0.45, describe("底盘足迹半径(m)"));
  declare_parameter<double>("self_filter.footprint.z_min", -1.0, describe("足迹圆柱下界(m, base_frame)"));
  declare_parameter<double>("self_filter.footprint.z_max", 0.30, describe("足迹圆柱上界(m, base_frame)"));
  declare_parameter<std::vector<std::string>>(
    "self_filter.chain_names", std::vector<std::string>{},
    describe("连杆链名列表，每条链按相邻 frame 生成胶囊体"));
}

bool PointcloudSliceScanNode::loadParameters(std::string & error)
{
  std::lock_guard<std::mutex> lock(config_mutex_);

  input_cloud_topic_ = get_parameter("input_cloud_topic").as_string();
  output_scan_topic_ = get_parameter("output_scan_topic").as_string();
  marker_topic_ = get_parameter("marker_topic").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  if (base_frame_.empty()) {
    error = "base_frame 不能为空";
    return false;
  }

  tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
  max_cloud_age_sec_ = get_parameter("max_cloud_age_sec").as_double();
  tf_time_tolerance_sec_ = get_parameter("tf_time_tolerance_sec").as_double();
  if (tf_timeout_sec_ < 0.0 || max_cloud_age_sec_ <= 0.0 || tf_time_tolerance_sec_ <= 0.0) {
    error = "tf_timeout_sec 需 >=0，max_cloud_age_sec / tf_time_tolerance_sec 需 >0";
    return false;
  }

  enable_voxel_filter_ = get_parameter("enable_voxel_filter").as_bool();
  voxel_leaf_size_ = get_parameter("voxel_leaf_size").as_double();
  if (enable_voxel_filter_ && !(voxel_leaf_size_ > 0.0)) {
    error = "启用体素滤波时 voxel_leaf_size 必须为正";
    return false;
  }
  enable_outlier_filter_ = get_parameter("enable_outlier_filter").as_bool();
  outlier_mean_k_ = static_cast<int>(get_parameter("outlier_mean_k").as_int());
  outlier_stddev_mul_ = get_parameter("outlier_stddev_mul").as_double();
  if (enable_outlier_filter_ && (outlier_mean_k_ < 2 || !(outlier_stddev_mul_ > 0.0))) {
    error = "启用离群点滤波时要求 outlier_mean_k>=2 且 outlier_stddev_mul>0";
    return false;
  }

  min_valid_points_ = static_cast<int>(get_parameter("min_valid_points").as_int());
  if (min_valid_points_ < 0) {
    error = "min_valid_points 不能为负";
    return false;
  }
  watchdog_period_sec_ = get_parameter("watchdog_period_sec").as_double();
  input_timeout_sec_ = get_parameter("input_timeout_sec").as_double();
  if (input_timeout_sec_ <= 0.0) {
    error = "input_timeout_sec 必须为正";
    return false;
  }

  const std::string policy = get_parameter("invalid_input_policy").as_string();
  if (policy == "hold_last") {
    invalid_input_policy_ = InvalidInputPolicy::kHoldLast;
  } else if (policy == "stop_output") {
    invalid_input_policy_ = InvalidInputPolicy::kStopOutput;
  } else {
    error = "invalid_input_policy 只能是 hold_last 或 stop_output，当前=" + policy;
    return false;
  }

  publish_markers_ = get_parameter("publish_markers").as_bool();
  marker_point_stride_ = static_cast<int>(get_parameter("marker_point_stride").as_int());
  if (marker_point_stride_ < 1) {
    error = "marker_point_stride 必须 >=1";
    return false;
  }

  // ---- 切片层：先读名字列表，再按需声明并读取每层的嵌套参数 ----
  const std::vector<std::string> slice_names =
    get_parameter("slice_names").as_string_array();
  if (slice_names.size() < kMinSliceCount) {
    error = "切片层至少需要 " + std::to_string(kMinSliceCount) +
      " 层(需求明确禁止单层切片)，当前=" + std::to_string(slice_names.size());
    return false;
  }
  if (slice_names.size() > kMaxSliceCount) {
    error = "切片层数超过上限 " + std::to_string(kMaxSliceCount);
    return false;
  }

  SliceProjector::Params proj_params;
  proj_params.angle_min = get_parameter("scan.angle_min").as_double();
  proj_params.angle_max = get_parameter("scan.angle_max").as_double();
  proj_params.angle_increment = get_parameter("scan.angle_increment").as_double();
  proj_params.range_min = get_parameter("scan.range_min").as_double();
  proj_params.range_max = get_parameter("scan.range_max").as_double();
  // 需求规定：切片后无有效障碍物点时，距离填最大探测距离。
  // 同时提供 infinity 模式，便于和既有 pointcloud_to_laserscan(use_inf:=true) 对齐行为。
  const std::string no_return_mode = get_parameter("scan.no_return_mode").as_string();
  if (no_return_mode == "range_max") {
    proj_params.no_return_value = static_cast<float>(proj_params.range_max);
  } else if (no_return_mode == "infinity") {
    proj_params.no_return_value = std::numeric_limits<float>::infinity();
  } else {
    error = "scan.no_return_mode 只能是 range_max 或 infinity，当前=" + no_return_mode;
    return false;
  }

  for (const std::string & name : slice_names) {
    const std::string prefix = "slices." + name + ".";
    // 这些嵌套参数在首次见到层名时才声明，因此支持「改 slice_names 加新层」。
    if (!has_parameter(prefix + "z_min")) {
      declare_parameter<double>(prefix + "z_min", 0.0);
      declare_parameter<double>(prefix + "z_max", 0.0);
      declare_parameter<int>(prefix + "min_points", 1);
      declare_parameter<double>(prefix + "max_range", proj_params.range_max);
      declare_parameter<bool>(prefix + "enabled", true);
    }
    SliceConfig cfg;
    cfg.name = name;
    cfg.z_min = get_parameter(prefix + "z_min").as_double();
    cfg.z_max = get_parameter(prefix + "z_max").as_double();
    cfg.min_points = static_cast<int>(get_parameter(prefix + "min_points").as_int());
    cfg.max_range = get_parameter(prefix + "max_range").as_double();
    cfg.enabled = get_parameter(prefix + "enabled").as_bool();
    proj_params.slices.push_back(cfg);
  }

  std::string proj_error;
  if (!projector_.configure(proj_params, proj_error)) {
    error = "切片投影器参数非法: " + proj_error;
    return false;
  }

  // ---- 自身点剔除 ----
  FootprintCylinder footprint;
  footprint.enabled = get_parameter("self_filter.footprint.enabled").as_bool();
  footprint.radius = get_parameter("self_filter.footprint.radius").as_double();
  footprint.z_min = get_parameter("self_filter.footprint.z_min").as_double();
  footprint.z_max = get_parameter("self_filter.footprint.z_max").as_double();
  if (footprint.enabled &&
    (!(footprint.radius > 0.0) || !(footprint.z_max > footprint.z_min)))
  {
    error = "足迹圆柱要求 radius>0 且 z_max>z_min";
    return false;
  }
  self_filter_.setFootprint(footprint);

  self_filter_chains_.clear();
  const std::vector<std::string> chain_names =
    get_parameter("self_filter.chain_names").as_string_array();
  for (const std::string & name : chain_names) {
    const std::string prefix = "self_filter.chains." + name + ".";
    if (!has_parameter(prefix + "radius")) {
      declare_parameter<double>(prefix + "radius", 0.0);
      declare_parameter<std::vector<std::string>>(prefix + "frames", std::vector<std::string>{});
    }
    SelfFilterChainConfig chain;
    chain.name = name;
    chain.radius = get_parameter(prefix + "radius").as_double();
    chain.frames = get_parameter(prefix + "frames").as_string_array();
    if (!(chain.radius > 0.0)) {
      error = "连杆链 '" + name + "' 的 radius 必须为正";
      return false;
    }
    if (chain.frames.empty()) {
      error = "连杆链 '" + name + "' 的 frames 不能为空";
      return false;
    }
    self_filter_chains_.push_back(std::move(chain));
  }

  error.clear();
  return true;
}

rcl_interfaces::msg::SetParametersResult PointcloudSliceScanNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // 话题名/线程相关参数在运行期改动需要重建订阅发布，风险高且收益低，
  // 这里明确拒绝，并提示改 YAML 后重启（对应「异常场景输出关键上下文」）。
  static const std::vector<std::string> kRestartRequired = {
    "input_cloud_topic", "output_scan_topic", "marker_topic", "watchdog_period_sec"};
  for (const rclcpp::Parameter & p : parameters) {
    if (std::find(kRestartRequired.begin(), kRestartRequired.end(), p.get_name()) !=
      kRestartRequired.end())
    {
      result.successful = false;
      result.reason = "参数 '" + p.get_name() + "' 不支持热更新，请改 YAML 后重启节点";
      return result;
    }
  }

  // 逐个「预演」：先校验能否接受，真正生效放在下一帧由工作线程统一重载。
  // 为什么不在这里直接改成员：本回调运行在执行器线程上，而工作线程可能正在
  // 用这些参数跑 PCL 滤波；就地修改会撕裂配置（半套新半套旧）。
  // ROS2 的语义是本回调返回 successful=true 之后，新值才写入参数服务器，
  // 所以这里只置脏标记，工作线程下一帧调用 loadParameters() 一次性原子换上。
  for (const rclcpp::Parameter & p : parameters) {
    if (p.get_name() == "invalid_input_policy") {
      const std::string v = p.as_string();
      if (v != "hold_last" && v != "stop_output") {
        result.successful = false;
        result.reason = "invalid_input_policy 只能是 hold_last / stop_output";
        return result;
      }
    }
  }

  config_dirty_.store(true);
  result.reason = "已接受，将在下一帧生效";
  return result;
}

void PointcloudSliceScanNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  // ---- 消息判空 ----
  if (!msg) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs, "收到空的点云消息指针，已丢弃");
    return;
  }

  last_input_time_ = now();
  input_timeout_warned_ = false;

  // ---- 空点云：不能拿去生成脏 scan ----
  if (msg->width == 0U || msg->height == 0U || msg->data.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "收到空点云(width=%u height=%u)，按策略处理，不生成脏 scan",
      msg->width, msg->height);
    if (invalid_input_policy_ == InvalidInputPolicy::kHoldLast) {
      republishLastScan();
    }
    return;
  }

  // ---- 时间戳校验：与当前时间偏差过大的帧直接丢 ----
  const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
  const double age = (now() - stamp).seconds();
  if (std::fabs(age) > max_cloud_age_sec_) {
    dropped_frame_count_.fetch_add(1U);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "点云时间戳偏差 %.3fs 超过上限 %.3fs，丢弃该帧(累计丢弃 %" PRIu64 " 帧)",
      age, max_cloud_age_sec_, static_cast<uint64_t>(dropped_frame_count_.load()));
    return;
  }

  // ---- 存入单槽缓冲，唤醒工作线程 ----
  // 回调到此结束，绝不在这里做 PCL 运算。
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    pending_cloud_ = msg;
  }
  queue_cv_.notify_one();
}

void PointcloudSliceScanNode::workerLoop()
{
  while (running_.load()) {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(
        lock, [this]() {return !running_.load() || pending_cloud_ != nullptr;});
      if (!running_.load()) {
        return;
      }
      cloud = pending_cloud_;
      pending_cloud_.reset();
    }
    if (!cloud) {
      continue;
    }

    // 动态调参在此处统一落地：整套参数一次性重载，避免出现半新半旧的中间状态。
    // 重载失败（新值组合非法）时保留旧配置继续工作，只报 ERROR，不中断数据流。
    if (config_dirty_.exchange(false)) {
      std::string error;
      if (loadParameters(error)) {
        RCLCPP_INFO(get_logger(), "参数已热更新并生效");
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "参数热更新失败，继续使用上一套有效配置: %s", error.c_str());
      }
    }

    // 处理过程中的所有异常都在 processCloud 内部消化，
    // 这里再兜一层，保证工作线程绝不会因为异常而退出。
    try {
      (void)processCloud(cloud);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleMs,
        "处理点云时发生未预期异常，已忽略本帧: %s", e.what());
    }
  }
}

bool PointcloudSliceScanNode::lookupCloudTransform(
  const std::string & cloud_frame,
  const rclcpp::Time & stamp,
  geometry_msgs::msg::TransformStamped & out)
{
  if (!tf_buffer_) {
    return false;
  }
  try {
    out = tf_buffer_->lookupTransform(
      base_frame_, cloud_frame, stamp,
      tf2::durationFromSec(tf_timeout_sec_));
  } catch (const tf2::TransformException & e) {
    // TF 未就绪/超时/外推失败都走这里：丢弃当前帧、等 TF 恢复，绝不抛出崩溃。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "TF 查询失败(%s -> %s)，丢弃当前帧: %s",
      cloud_frame.c_str(), base_frame_.c_str(), e.what());
    return false;
  }

  // 点云时间戳与 TF 时间戳偏差校验：偏差过大说明两者时间错位，
  // 继续用下去会把「机器人过去的姿态」和「现在的点云」混在一起，
  // 在 base_frame 里生成位置错误的障碍物。
  const rclcpp::Time tf_stamp(out.header.stamp, RCL_ROS_TIME);
  const double delta = std::fabs((tf_stamp - stamp).seconds());
  if (delta > tf_time_tolerance_sec_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "点云时间戳与 TF 时间戳偏差 %.3fs 超过上限 %.3fs，丢弃该帧",
      delta, tf_time_tolerance_sec_);
    return false;
  }
  return true;
}

std::vector<FilterCapsule> PointcloudSliceScanNode::resolveSelfFilterCapsules(
  const rclcpp::Time & stamp)
{
  std::vector<FilterCapsule> capsules;
  if (!tf_buffer_) {
    return capsules;
  }

  std::size_t missing = 0U;
  for (const SelfFilterChainConfig & chain : self_filter_chains_) {
    // 先把链上每个 frame 在 base_frame 下的原点位置查出来。
    // 查不到的 frame 标成无效，后面连线时跳过——机械臂某个连杆的 TF
    // 偶发丢失时，其余连杆仍然正常剔除。
    std::vector<std::array<float, 3>> origins;
    std::vector<bool> valid;
    origins.reserve(chain.frames.size());
    valid.reserve(chain.frames.size());

    for (const std::string & frame : chain.frames) {
      std::array<float, 3> p{{0.0F, 0.0F, 0.0F}};
      bool ok = false;
      try {
        const geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
          base_frame_, frame, stamp, tf2::durationFromSec(tf_timeout_sec_));
        p[0] = static_cast<float>(tf.transform.translation.x);
        p[1] = static_cast<float>(tf.transform.translation.y);
        p[2] = static_cast<float>(tf.transform.translation.z);
        ok = true;
      } catch (const tf2::TransformException &) {
        ++missing;
      }
      origins.push_back(p);
      valid.push_back(ok);
    }

    if (chain.frames.size() == 1U) {
      // 单 frame 链退化成球（线段两端重合，pointSegmentDistanceSquared 会自动处理）。
      if (valid.front()) {
        FilterCapsule c;
        c.name = chain.name;
        c.x0 = c.x1 = origins[0][0];
        c.y0 = c.y1 = origins[0][1];
        c.z0 = c.z1 = origins[0][2];
        c.radius = static_cast<float>(chain.radius);
        capsules.push_back(std::move(c));
      }
      continue;
    }

    for (std::size_t i = 0; i + 1U < chain.frames.size(); ++i) {
      if (!valid[i] || !valid[i + 1U]) {
        continue;
      }
      FilterCapsule c;
      c.name = chain.name + "[" + std::to_string(i) + "]";
      c.x0 = origins[i][0];
      c.y0 = origins[i][1];
      c.z0 = origins[i][2];
      c.x1 = origins[i + 1U][0];
      c.y1 = origins[i + 1U][1];
      c.z1 = origins[i + 1U][2];
      c.radius = static_cast<float>(chain.radius);
      capsules.push_back(std::move(c));
    }
  }

  if (missing > 0U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "本帧有 %zu 个连杆 TF 查询失败，对应胶囊体已跳过(其余部分仍生效)", missing);
  }
  return capsules;
}

bool PointcloudSliceScanNode::processCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  if (!msg) {
    return false;
  }
  const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);

  // ---- 1) ROS 消息 → PCL ----
  auto cloud_in = std::make_shared<PointCloudXYZ>();
  pcl::fromROSMsg(*msg, *cloud_in);
  if (cloud_in->empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs,
      "点云转换后为空(可能字段不含 x/y/z)，丢弃该帧");
    return false;
  }

  // ---- 2) 体素降采样 ----
  auto cloud_work = cloud_in;
  if (enable_voxel_filter_) {
    auto downsampled = std::make_shared<PointCloudXYZ>();
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud_work);
    const auto leaf = static_cast<float>(voxel_leaf_size_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*downsampled);
    if (!downsampled->empty()) {
      cloud_work = downsampled;
    }
  }

  // ---- 3) 统计离群点滤波 ----
  // 放在降采样之后：SOR 要建 kd-tree，先降采样能显著降低耗时。
  if (enable_outlier_filter_ &&
    cloud_work->size() > static_cast<std::size_t>(outlier_mean_k_))
  {
    auto denoised = std::make_shared<PointCloudXYZ>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_work);
    sor.setMeanK(outlier_mean_k_);
    sor.setStddevMulThresh(outlier_stddev_mul_);
    sor.filter(*denoised);
    if (!denoised->empty()) {
      cloud_work = denoised;
    }
  }

  // ---- 4) 变换到 base_frame ----
  geometry_msgs::msg::TransformStamped tf_msg;
  if (!lookupCloudTransform(msg->header.frame_id, stamp, tf_msg)) {
    if (invalid_input_policy_ == InvalidInputPolicy::kHoldLast) {
      republishLastScan();
    }
    dropped_frame_count_.fetch_add(1U);
    return false;
  }

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  {
    const auto & q = tf_msg.transform.rotation;
    const auto & t = tf_msg.transform.translation;
    const Eigen::Quaternionf quat(
      static_cast<float>(q.w), static_cast<float>(q.x),
      static_cast<float>(q.y), static_cast<float>(q.z));
    transform.linear() = quat.normalized().toRotationMatrix();
    transform.translation() << static_cast<float>(t.x),
      static_cast<float>(t.y), static_cast<float>(t.z);
  }
  auto cloud_base = std::make_shared<PointCloudXYZ>();
  pcl::transformPointCloud(*cloud_work, *cloud_base, transform);

  // ---- 5) 自身点剔除（TF 实时驱动）----
  std::vector<SlicePoint> kept;
  std::vector<SlicePoint> self_points;
  std::vector<FilterCapsule> capsules;
  ProjectionResult result;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    capsules = resolveSelfFilterCapsules(stamp);
    self_filter_.setCapsules(capsules);

    kept.reserve(cloud_base->size());
    for (const pcl::PointXYZ & p : cloud_base->points) {
      SlicePoint sp;
      sp.x = p.x;
      sp.y = p.y;
      sp.z = p.z;
      if (self_filter_.isSelfPoint(sp)) {
        if (publish_markers_) {
          self_points.push_back(sp);
        }
        continue;
      }
      kept.push_back(sp);
    }

    if (static_cast<int>(kept.size()) < min_valid_points_) {
      // 注意这里和「切片后无障碍物」是两种不同情况：
      // 点数过少说明这一帧本身不可信（雷达遮挡/剔除过度），不能当成「前方无障碍」。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleMs,
        "剔除自身点后仅剩 %zu 点(阈值 %d)，本帧视为无效", kept.size(), min_valid_points_);
      if (invalid_input_policy_ == InvalidInputPolicy::kHoldLast) {
        republishLastScan();
      }
      return false;
    }

    // ---- 6) 多层切片投影融合 ----
    projector_.project(kept, result);
  }

  // ---- 7) 输出 ----
  publishScan(result, stamp);
  publishFilteredCloud(kept, stamp);
  if (publish_markers_) {
    publishSliceMarkers(kept, self_points, capsules, stamp);
  }

  const uint64_t count = processed_frame_count_.fetch_add(1U) + 1U;
  // 正常帧只做低频 INFO，避免刷屏；关键统计量都带上便于现场判断。
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "已处理 %" PRIu64 " 帧: 输入 %zu 点 → 保留 %zu 点(自身剔除 %zu) → 占用角度桶 %zu/%zu",
    count, cloud_in->size(), kept.size(),
    self_points.size(), result.occupied_bucket_count, projector_.bucket_count());
  return true;
}

void PointcloudSliceScanNode::publishScan(
  const ProjectionResult & result, const rclcpp::Time & stamp)
{
  if (!scan_pub_) {
    return;
  }
  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  const SliceProjector::Params & p = projector_.params();

  scan->header.stamp = stamp;
  scan->header.frame_id = base_frame_;
  scan->angle_min = static_cast<float>(p.angle_min);
  scan->angle_max = static_cast<float>(p.angle_max);
  scan->angle_increment = static_cast<float>(p.angle_increment);
  scan->range_min = static_cast<float>(p.range_min);
  scan->range_max = static_cast<float>(p.range_max);
  scan->scan_time = static_cast<float>(get_parameter("scan.scan_time").as_double());
  scan->time_increment = 0.0F;
  scan->ranges = result.ranges;
  // intensities 留空：本 scan 是点云投影产物，没有真实回波强度，
  // 填假值会误导下游算法。

  scan_pub_->publish(*scan);
  {
    std::lock_guard<std::mutex> lock(last_scan_mutex_);
    last_valid_scan_ = scan;
  }
}

void PointcloudSliceScanNode::publishFilteredCloud(
  const std::vector<SlicePoint> & kept, const rclcpp::Time & stamp)
{
  if (!filtered_cloud_pub_) {
    return;
  }
  // 只搬 xyz：下游（pointcloud_to_laserscan）只用 xyz，多带强度/时间戳字段
  // 除了增加带宽没有别的作用。
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = base_frame_;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(kept.size());
  msg.is_dense = true;
  msg.is_bigendian = false;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
  for (const SlicePoint & p : kept) {
    *it_x = static_cast<float>(p.x);
    *it_y = static_cast<float>(p.y);
    *it_z = static_cast<float>(p.z);
    ++it_x;
    ++it_y;
    ++it_z;
  }
  filtered_cloud_pub_->publish(msg);
}

void PointcloudSliceScanNode::republishLastScan()
{
  if (!scan_pub_) {
    return;
  }
  sensor_msgs::msg::LaserScan::SharedPtr copy;
  {
    std::lock_guard<std::mutex> lock(last_scan_mutex_);
    if (!last_valid_scan_) {
      return;   // 还没有过有效帧，无可重发
    }
    copy = std::make_shared<sensor_msgs::msg::LaserScan>(*last_valid_scan_);
  }
  // 只刷新时间戳，内容保持上一帧。这样 costmap 不会因为「传感器超时」
  // 把已知障碍物清空，同时下游若关心时效性也能从日志看到我们在保持旧值。
  copy->header.stamp = now();
  scan_pub_->publish(*copy);
}

void PointcloudSliceScanNode::watchdogCallback()
{
  if (last_input_time_.nanoseconds() == 0) {
    // 从未收到过任何点云。这种情况必须告警——它恰恰是最难查的故障
    // （上游雷达节点没起来、话题名写错、QoS 不匹配、domain 不同），
    // 而如果这里保持沉默，节点会一声不响地什么都不干，日志里只有一行启动信息。
    // 用节点存活时长做判据，超过一个输入超时周期还没有数据就周期性提醒。
    const double alive = (now() - node_start_time_).seconds();
    if (alive > input_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleMs * 3,
        "启动后 %.1fs 从未收到任何点云。请依次检查："
        "① 上游话题是否在发 (ros2 topic hz %s)；"
        "② 话题名/命名空间是否一致；③ QoS 是否匹配(本节点用 SensorDataQoS)；"
        "④ ROS_DOMAIN_ID 是否一致；⑤ use_sim_time 是否与仿真一致",
        alive, input_cloud_topic_.c_str());
    }
    return;
  }
  const double idle = (now() - last_input_time_).seconds();
  if (idle < input_timeout_sec_) {
    return;
  }
  if (!input_timeout_warned_) {
    RCLCPP_WARN(
      get_logger(),
      "已 %.2fs 未收到点云(超时阈值 %.2fs)，策略=%s",
      idle, input_timeout_sec_,
      invalid_input_policy_ == InvalidInputPolicy::kHoldLast ? "重发上一帧" : "停止输出");
    input_timeout_warned_ = true;
  }
  if (invalid_input_policy_ == InvalidInputPolicy::kHoldLast) {
    republishLastScan();
  }
}

void PointcloudSliceScanNode::publishSliceMarkers(
  const std::vector<SlicePoint> & kept_points,
  const std::vector<SlicePoint> & self_points,
  const std::vector<FilterCapsule> & capsules,
  const rclcpp::Time & stamp)
{
  if (!marker_pub_) {
    return;
  }
  auto array = std::make_shared<visualization_msgs::msg::MarkerArray>();
  const SliceProjector::Params & p = projector_.params();
  const auto stride = static_cast<std::size_t>(marker_point_stride_);

  // 为每层分配一个固定色，便于在 RViz 里一眼分辨高度层。
  // 色相按层序均匀铺开（HSV→RGB 的简化版本，只取饱和度/亮度拉满的环）。
  auto sliceColor = [](std::size_t idx, std::size_t total) {
      std_msgs::msg::ColorRGBA c;
      c.a = 1.0F;
      const float h = (total > 0U) ? (static_cast<float>(idx) / static_cast<float>(total)) : 0.0F;
      const float x = h * 6.0F;
      const int sector = static_cast<int>(x) % 6;
      const float f = x - std::floor(x);
      switch (sector) {
        case 0: c.r = 1.0F; c.g = f; c.b = 0.0F; break;
        case 1: c.r = 1.0F - f; c.g = 1.0F; c.b = 0.0F; break;
        case 2: c.r = 0.0F; c.g = 1.0F; c.b = f; break;
        case 3: c.r = 0.0F; c.g = 1.0F - f; c.b = 1.0F; break;
        case 4: c.r = f; c.g = 0.0F; c.b = 1.0F; break;
        default: c.r = 1.0F; c.g = 0.0F; c.b = 1.0F - f; break;
      }
      return c;
    };

  int marker_id = 0;
  // ---- 每个切片层一个 POINTS marker ----
  for (std::size_t s = 0; s < p.slices.size(); ++s) {
    const SliceConfig & cfg = p.slices[s];
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = base_frame_;
    m.ns = "slice_" + cfg.name;
    m.id = marker_id++;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.03;
    m.scale.y = 0.03;
    m.color = sliceColor(s, p.slices.size());
    // 未启用的层用半透明显示，一眼能看出「这层没参与融合」。
    if (!cfg.enabled) {
      m.color.a = 0.25F;
    }
    m.pose.orientation.w = 1.0;
    for (std::size_t i = 0; i < kept_points.size(); i += stride) {
      const SlicePoint & sp = kept_points[i];
      if (static_cast<double>(sp.z) < cfg.z_min || static_cast<double>(sp.z) >= cfg.z_max) {
        continue;
      }
      geometry_msgs::msg::Point gp;
      gp.x = static_cast<double>(sp.x);
      gp.y = static_cast<double>(sp.y);
      gp.z = static_cast<double>(sp.z);
      m.points.push_back(gp);
    }
    array->markers.push_back(std::move(m));
  }

  // ---- 被判为「自身」的点，灰色显示 ----
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = base_frame_;
    m.ns = "self_filtered";
    m.id = marker_id++;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.03;
    m.scale.y = 0.03;
    m.color.r = 0.6F;
    m.color.g = 0.6F;
    m.color.b = 0.6F;
    m.color.a = 0.6F;
    m.pose.orientation.w = 1.0;
    for (std::size_t i = 0; i < self_points.size(); i += stride) {
      geometry_msgs::msg::Point gp;
      gp.x = static_cast<double>(self_points[i].x);
      gp.y = static_cast<double>(self_points[i].y);
      gp.z = static_cast<double>(self_points[i].z);
      m.points.push_back(gp);
    }
    array->markers.push_back(std::move(m));
  }

  // ---- 剔除胶囊体的轴线，用粗线表示（线宽≈直径）----
  for (const FilterCapsule & c : capsules) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = base_frame_;
    m.ns = "self_filter_capsule";
    m.id = marker_id++;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = static_cast<double>(c.radius) * 2.0;
    m.color.r = 1.0F;
    m.color.g = 0.4F;
    m.color.b = 0.0F;
    m.color.a = 0.35F;
    m.pose.orientation.w = 1.0;
    geometry_msgs::msg::Point a;
    a.x = static_cast<double>(c.x0);
    a.y = static_cast<double>(c.y0);
    a.z = static_cast<double>(c.z0);
    geometry_msgs::msg::Point b;
    b.x = static_cast<double>(c.x1);
    b.y = static_cast<double>(c.y1);
    b.z = static_cast<double>(c.z1);
    m.points.push_back(a);
    m.points.push_back(b);
    array->markers.push_back(std::move(m));
  }

  marker_pub_->publish(*array);
}

}  // namespace astribot_s1_autonomy

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(astribot_s1_autonomy::PointcloudSliceScanNode)
