// Copyright 2026 Astribot.
//
// livox_custom_to_pc2_node.hpp 的实现。
#include "astribot_s1_autonomy/livox_custom_to_pc2_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "sensor_msgs/msg/point_field.hpp"

namespace astribot_s1_autonomy
{

namespace
{
/// 装配 PointCloud2 的 fields。字段顺序与 ConvertedPoint 的内存布局无关 ——
/// 这里显式写 offset，不依赖结构体 padding。
///
/// 字段命名沿用社区惯例，好让现成工具直接吃：
///   x/y/z/intensity  —— pcl_conversions、pointcloud_to_laserscan 都认
///   ring             —— velodyne/ouster 驱动的惯例（uint16）
///   time             —— 相对本帧 header.stamp 的秒偏移（float32），供去畸变
/// 刻意**不叫** `t`：ouster 用 `t` 且是 uint32 纳秒，语义不同，同名会骗人。
struct FieldLayout
{
  static constexpr std::uint32_t kOffX = 0;
  static constexpr std::uint32_t kOffY = 4;
  static constexpr std::uint32_t kOffZ = 8;
  static constexpr std::uint32_t kOffIntensity = 12;
  static constexpr std::uint32_t kOffTime = 16;
  static constexpr std::uint32_t kOffRing = 20;
  static constexpr std::uint32_t kOffTag = 22;
  static constexpr std::uint32_t kPointStep = 24;   // 22 向上对齐到 4 的倍数
};

void fill_fields(sensor_msgs::msg::PointCloud2 * msg)
{
  using PF = sensor_msgs::msg::PointField;
  auto add = [msg](const char * name, std::uint32_t offset, std::uint8_t dtype) {
      PF f;
      f.name = name;
      f.offset = offset;
      f.datatype = dtype;
      f.count = 1;
      msg->fields.push_back(f);
    };
  msg->fields.clear();
  add("x", FieldLayout::kOffX, PF::FLOAT32);
  add("y", FieldLayout::kOffY, PF::FLOAT32);
  add("z", FieldLayout::kOffZ, PF::FLOAT32);
  add("intensity", FieldLayout::kOffIntensity, PF::FLOAT32);
  add("time", FieldLayout::kOffTime, PF::FLOAT32);
  add("ring", FieldLayout::kOffRing, PF::UINT16);
  add("tag", FieldLayout::kOffTag, PF::UINT8);
}

template<typename T>
inline void put(std::uint8_t * base, std::uint32_t offset, T value)
{
  std::memcpy(base + offset, &value, sizeof(T));
}
}  // namespace

LivoxCustomToPc2Node::LivoxCustomToPc2Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("livox_custom_to_pc2", options)
{
  report_period_sec_ = declare_parameter<double>("report_period_sec", 10.0);
  stale_warn_sec_ = declare_parameter<double>("stale_warn_sec", 2.0);

  // 两路的无效点默认值直接取自厂商 MID360_config.json：
  //   lidar[0] IP .12 = front，外参全零        → 无效点在 (0,0,0)
  //   lidar[1] IP .13 = back，外参 (1,-496,84)mm → 无效点在 (0.001,-0.496,0.084)
  // 这两组默认值就是实机实测到的假点团位置（front 33.4%、back 34.0%）。
  channels_.push_back(declare_channel(
      "front", "/livox/lidar_front", "/livox/lidar_front_pc2",
      0.0, 0.0, 0.0));
  channels_.push_back(declare_channel(
      "back", "/livox/lidar_back", "/livox/lidar_back_pc2",
      0.001, -0.496, 0.084));

  // 传感器数据用 BEST_EFFORT + 小深度：这是点云的惯例，也与驱动的发布 QoS 匹配。
  // 深度给 5 而不是 1：转换有计算量，留一点缓冲避免抖动时直接丢帧。
  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(5);

  for (std::size_t i = 0; i < channels_.size(); ++i) {
    ConvertChannel & ch = channels_[i];
    ch.pub = create_publisher<sensor_msgs::msg::PointCloud2>(ch.out_topic, qos);
    ch.sub = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      ch.in_topic, qos,
      [this, i](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
        this->on_cloud(i, msg);
      });
    RCLCPP_INFO(get_logger(), "第 %zu 路：%s → %s，无效点 (%.4f, %.4f, %.4f) r=%.3f",
                i, ch.in_topic.c_str(), ch.out_topic.c_str(),
                ch.cfg.null_x, ch.cfg.null_y, ch.cfg.null_z, ch.cfg.null_radius);
  }

  if (report_period_sec_ > 0.0) {
    report_timer_ = create_wall_timer(
      std::chrono::duration<double>(report_period_sec_),
      [this]() {this->report();});
  }

  in_buf_.reserve(30000);
  out_buf_.reserve(30000);

  RCLCPP_INFO(get_logger(),
              "CustomMsg→PointCloud2 转换就绪。\n"
              "  为什么需要它：厂商驱动以 xfer_format=1 发 CustomMsg（SLAM 需要），\n"
              "  而感知链（livox_fusion → slice_scan → /scan）订阅 PointCloud2。\n"
              "  /scan 是两个 costmap 的 obstacle_layer 唯一数据源，断了就没有动态避障。\n"
              "  为什么是 C++：实测 rclpy 反序列化两万点的 CustomMsg 跟不上 10Hz×2 路，\n"
              "  只能收到 20~40%% 且静默丢帧。");
}

ConvertChannel LivoxCustomToPc2Node::declare_channel(
  const std::string & prefix, const std::string & default_in,
  const std::string & default_out,
  double default_null_x, double default_null_y, double default_null_z)
{
  ConvertChannel ch;
  ch.in_topic = declare_parameter<std::string>(prefix + ".input_topic", default_in);
  ch.out_topic = declare_parameter<std::string>(prefix + ".output_topic", default_out);
  ch.cfg.null_x = static_cast<float>(
    declare_parameter<double>(prefix + ".null_point.x", default_null_x));
  ch.cfg.null_y = static_cast<float>(
    declare_parameter<double>(prefix + ".null_point.y", default_null_y));
  ch.cfg.null_z = static_cast<float>(
    declare_parameter<double>(prefix + ".null_point.z", default_null_z));
  ch.cfg.null_radius = static_cast<float>(
    declare_parameter<double>(prefix + ".null_point.radius", 0.005));
  ch.cfg.min_range = static_cast<float>(
    declare_parameter<double>(prefix + ".min_range", 0.0));
  ch.cfg.max_range = static_cast<float>(
    declare_parameter<double>(prefix + ".max_range", 0.0));
  ch.cfg.max_spatial_noise = static_cast<std::uint8_t>(
    declare_parameter<int>(prefix + ".max_spatial_noise", 4));
  // 参数不合法就在构造期抛，不要等到第一帧到了才发现。
  ch.cfg.validate();
  return ch;
}

void LivoxCustomToPc2Node::on_cloud(
  std::size_t idx, const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
  ConvertChannel & ch = channels_[idx];
  ch.last_rx_sec =
    std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();

  in_buf_.clear();
  in_buf_.reserve(msg->points.size());
  for (const auto & p : msg->points) {
    LivoxPoint q;
    q.offset_time = p.offset_time;
    q.x = p.x;
    q.y = p.y;
    q.z = p.z;
    q.reflectivity = p.reflectivity;
    q.tag = p.tag;
    q.line = p.line;
    in_buf_.push_back(q);
  }

  const std::uint64_t stamp_ns =
    static_cast<std::uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
    static_cast<std::uint64_t>(msg->header.stamp.nanosec);

  const LivoxConvertStats st =
    convert_frame(in_buf_, msg->timebase, stamp_ns, ch.cfg, &out_buf_);

  ++ch.frames;
  ch.total_pts += st.total;
  ch.kept_pts += st.kept;
  ch.dropped_null += st.dropped_null;
  ch.dropped_other += st.dropped_range + st.dropped_noise + st.dropped_nonfinite;

  sensor_msgs::msg::PointCloud2 out;
  // frame_id 原样透传：实机是 livox_frame，URDF 里已有对应的恒等边。
  out.header = msg->header;
  out.height = 1;
  out.width = static_cast<std::uint32_t>(out_buf_.size());
  fill_fields(&out);
  out.is_bigendian = false;
  out.point_step = FieldLayout::kPointStep;
  out.row_step = out.point_step * out.width;
  out.is_dense = true;   // 无效点与 NaN 都已剔除，这里可以如实声明
  out.data.resize(static_cast<std::size_t>(out.row_step));

  std::uint8_t * base = out.data.data();
  for (std::size_t i = 0; i < out_buf_.size(); ++i) {
    std::uint8_t * p = base + i * FieldLayout::kPointStep;
    const ConvertedPoint & c = out_buf_[i];
    put<float>(p, FieldLayout::kOffX, c.x);
    put<float>(p, FieldLayout::kOffY, c.y);
    put<float>(p, FieldLayout::kOffZ, c.z);
    put<float>(p, FieldLayout::kOffIntensity, c.intensity);
    put<float>(p, FieldLayout::kOffTime, c.time);
    put<std::uint16_t>(p, FieldLayout::kOffRing, c.ring);
    put<std::uint8_t>(p, FieldLayout::kOffTag, c.tag);
  }

  ch.pub->publish(std::move(out));
}

void LivoxCustomToPc2Node::report()
{
  const double now =
    std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();

  for (const ConvertChannel & ch : channels_) {
    if (ch.frames == 0U) {
      // 一帧都没收到与「收过但停了」是两种故障，指向的地方不同。
      RCLCPP_WARN(get_logger(),
                  "%s：一帧都没收到。查驱动是否在发、以及 xfer_format 是否为 1"
                  "（CustomMsg）。",
                  ch.in_topic.c_str());
      continue;
    }
    const double age = now - ch.last_rx_sec;
    const double keep_pct =
      ch.total_pts == 0U ? 0.0 :
      100.0 * static_cast<double>(ch.kept_pts) / static_cast<double>(ch.total_pts);
    const double null_pct =
      ch.total_pts == 0U ? 0.0 :
      100.0 * static_cast<double>(ch.dropped_null) / static_cast<double>(ch.total_pts);

    RCLCPP_INFO(get_logger(),
                "%s：帧=%lu 点=%lu 保留=%.1f%% 无效点剔除=%.1f%% 其他剔除=%lu 龄期=%.2fs",
                ch.in_topic.c_str(),
                static_cast<unsigned long>(ch.frames),
                static_cast<unsigned long>(ch.total_pts),
                keep_pct, null_pct,
                static_cast<unsigned long>(ch.dropped_other), age);

    if (age > stale_warn_sec_) {
      RCLCPP_WARN(get_logger(),
                  "%s 已 %.2fs 没有新帧（上限 %.2fs）—— 这一路停了，"
                  "上面那些计数是历史值，不是当前速率。",
                  ch.in_topic.c_str(), age, stale_warn_sec_);
    }
    // 无效点比例是个强信号：实测 front≈33%、back≈34%。
    // 明显偏离说明 null_point 配错了（比如两路配串），那会让假点团漏进 costmap。
    if (null_pct < 5.0) {
      RCLCPP_WARN(get_logger(),
                  "%s 的无效点剔除只有 %.1f%%，实机实测应在 30%% 上下。"
                  "检查 %s.null_point 是否配成了这一路真实的外参平移量 —— "
                  "配错的后果是假点团进入 costmap 变成障碍物。",
                  ch.in_topic.c_str(), null_pct, ch.in_topic.c_str());
    }
  }
}

}  // namespace astribot_s1_autonomy
