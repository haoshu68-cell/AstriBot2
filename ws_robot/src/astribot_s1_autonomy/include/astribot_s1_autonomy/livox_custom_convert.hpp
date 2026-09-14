// Copyright 2026 Astribot.
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
