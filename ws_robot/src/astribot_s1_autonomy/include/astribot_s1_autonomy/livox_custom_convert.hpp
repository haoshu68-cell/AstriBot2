// Copyright 2026 Astribot.
//
// Livox CustomMsg → PointCloud2 的**纯逻辑**核心：无效点判定与字段装配。
// 不 include rclcpp、不 include 任何消息类型，好让判定规则脱离 ROS 单测。
//
// ════════════════ 为什么需要这一层转换 ════════════════
// 实机厂商驱动以 xfer_format=1 发 `livox_ros_driver2/msg/CustomMsg`，
// 而 SLAM 需要这个格式（mid360.yaml 的 lidar_type=0 要 CustomMsg），不能改。
// 但我们的感知链（livox_fusion_node → pointcloud_slice_scan_node）订阅的是
// `sensor_msgs/PointCloud2`。两个 costmap 的 obstacle_layer 唯一数据源是
// `/scan`，而 `/scan` 由切片链产出 —— 所以这条链断着就等于**没有动态避障**。
//
// 早先记录说这是「xfer_format 1-vs-2 互斥、无解」。那个结论是错的：
// 不是无解，是中间缺一个转换环节。本文件就是那一环的判定部分。
//
// ════════════════ 为什么必须是 C++ 而不是 Python ════════════════
// 实测（同一台机器）：rclpy 反序列化 MID360 的 CustomMsg（每帧约两万点）
// 一帧要几十毫秒，订阅两路时只能收到 20~40%，而且 BEST_EFFORT 悄悄丢、
// 不报任何错。同一探针改成 raw=True 只取字节就能满速收到 10Hz。
// 也就是说 Python 版转换节点会**结构性地跟不上**（需要 20 帧/s 的反序列化预算，
// 实测只有约 7.7 帧/s），并且它丢帧的方式是静默的。所以这里用 C++。
//
// ════════════════ 无效点（假点团）是本文件的重点 ════════════════
// 雷达无回波时会输出一个零点。而驱动通过 SetLivoxLidarInstallAttitude 把外参
// **写进了雷达设备**，于是那个零点也被外参一起变换，落在外参平移处：
//
//   /livox/lidar_front  外参恒等      → 无效点堆在 (0, 0, 0)          实测占 33.4%
//   /livox/lidar_back   外参 y=-496mm → 无效点堆在 (0.001,-0.496,0.084) 实测占 34.0%
//
// 关键危害：back 那一团距原点 0.496m，**大于** livox_preprocess_node 的
// range_min=0.35，所以现有的球面距离门限**滤不掉它**。它落在机器人足迹内，
// 一旦自滤没接上就直接变成障碍物 —— 本项目已经栽过一次同类问题
// （夹爪不在自滤链里 → 机器人把指尖当障碍 → 探索 0 次派发）。
//
// 所以无效点位置是**每路可配的参数**，默认 (0,0,0)；对已在设备内应用了外参的
// 那一路，必须配成该外参的平移量。判定用一个小半径的球，而不是「距原点很近」。
#ifndef ASTRIBOT_S1_AUTONOMY__LIVOX_CUSTOM_CONVERT_HPP_
#define ASTRIBOT_S1_AUTONOMY__LIVOX_CUSTOM_CONVERT_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace astribot_s1_autonomy
{

/// 配置被拒绝。宁可起不来，也不要静默地按一组错参数跑。
class LivoxConvertConfigError : public std::invalid_argument
{
public:
  explicit LivoxConvertConfigError(const std::string & what)
  : std::invalid_argument(what) {}
};

/// 一个输入点（与 CustomPoint 同构，但不依赖消息类型）。
struct LivoxPoint
{
  std::uint32_t offset_time{0};   ///< 相对帧起始的纳秒偏移
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::uint8_t reflectivity{0};
  std::uint8_t tag{0};
  std::uint8_t line{0};
};

/// 转换配置。
struct LivoxConvertConfig
{
  /// 无效点（无回波）在本路点云里的堆积坐标，单位米。
  /// 默认 (0,0,0)；对已在设备内应用外参的那一路，须配成外参平移量。
  float null_x{0.0F};
  float null_y{0.0F};
  float null_z{0.0F};
  /// 判定为无效点的球半径（米）。实测那一团是**精确**同一坐标，
  /// 5mm 足够；给一点余量以防固件做浮点换算。
  float null_radius{0.005F};
  /// 球面距离下限（米）。0 表示不启用。留给「明确知道这一路装在何处」的场合，
  /// 默认 0 —— 距离过滤是 livox_preprocess_node 的职责，这里不重复做，
  /// 免得两处门限各改一个、叠乘出一个没人预期的结果。
  float min_range{0.0F};
  /// 球面距离上限（米）。0 表示不启用。
  float max_range{0.0F};
  /// 丢弃 tag 的空间置信度位（bit0-1）为「噪点」的点。
  /// Livox tag 的 bit0-1：0=置信度高，1/2/3 依次变差。阈值 4 表示不丢弃。
  std::uint8_t max_spatial_noise{4};

  /// 校验。不合法就抛，调用方（节点层）在构造时就会失败。
  void validate() const;
};

/// 一次转换的统计量。故障时先看这几个数。
struct LivoxConvertStats
{
  std::size_t total{0};        ///< 输入点数
  std::size_t kept{0};         ///< 保留点数
  std::size_t dropped_null{0}; ///< 因落在无效点球内被丢
  std::size_t dropped_range{0};///< 因球面距离越界被丢
  std::size_t dropped_noise{0};///< 因 tag 噪点位被丢
  std::size_t dropped_nonfinite{0}; ///< 因 NaN/Inf 被丢

  /// 保留比例。total==0 时返回 0，不做除零。
  double kept_ratio() const;
};

/// 输出的一个点。字段顺序与 PointCloud2 的 fields 装配保持一致。
struct ConvertedPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};  ///< 取自 reflectivity（0~255），保留为 float 便于下游
  float time{0.0F};       ///< 相对 header.stamp 的秒偏移，供后续逐点去畸变
  std::uint16_t ring{0};  ///< 取自 line（激光线号）
  std::uint8_t tag{0};    ///< 原样带出，便于事后诊断

  bool operator==(const ConvertedPoint & other) const;
};

/// 判定单点是否应当丢弃；返回 true 表示丢弃，并把原因累加进 stats。
bool should_drop(const LivoxPoint & p, const LivoxConvertConfig & cfg,
                 LivoxConvertStats * stats);

/// 转换一帧。timebase 与 header_stamp_ns 都是纳秒；
/// 输出点的 time 字段 = (timebase + offset_time - header_stamp_ns) / 1e9。
///
/// 为什么不直接用 offset_time：CustomMsg 的 offset_time 是相对 `timebase` 的，
/// 而 timebase 与 header.stamp 未必相等（驱动可能用到达时刻打戳）。下游做
/// 去畸变时需要的是「相对本帧 header.stamp 的偏移」，所以这里换算好再输出。
LivoxConvertStats convert_frame(const std::vector<LivoxPoint> & in,
                                std::uint64_t timebase,
                                std::uint64_t header_stamp_ns,
                                const LivoxConvertConfig & cfg,
                                std::vector<ConvertedPoint> * out);

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__LIVOX_CUSTOM_CONVERT_HPP_
