// Copyright 2026 Astribot.
#ifndef ASTRIBOT_S1_AUTONOMY__SLICE_PROJECTOR_HPP_
#define ASTRIBOT_S1_AUTONOMY__SLICE_PROJECTOR_HPP_

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace astribot_s1_autonomy
{

/// 单个点（已变换到 base_frame 下）。刻意用最小结构，避免和 PCL 类型耦合。
struct SlicePoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

/// 一个高度切片层的配置。全部字段来自 YAML，禁止硬编码。
struct SliceConfig
{
  /// 层名，仅用于日志和 Marker 命名空间。
  std::string name;
  /// 该层在 base_frame 下的高度区间 [z_min, z_max)，单位 m。
  double z_min{0.0};
  double z_max{0.0};
  /// 该层某个角度桶内至少要有多少个点，才认为这个方向真的存在障碍物。
  /// 用于压制地面反光点、雪花噪点这类「孤点」——越贴近地面的层建议给得越大。
  int min_points{1};
  /// 该层的信任距离上限(m)。贴地层在底盘俯仰时容易把远处地面误判成障碍，
  /// 因此可以只信任近距离；高层（检测悬空物）可以放到全量程。
  double max_range{0.0};
  /// 该层是否参与融合。留着 false 的层依然会发 Marker，便于对比调参。
  bool enabled{true};
};

/// 投影融合的结果。
struct ProjectionResult
{
  /// 融合后每个角度桶的距离，长度 == bucket_count()。
  /// 无障碍的桶填 no_return_value（见 Params）。
  std::vector<float> ranges;
  /// 每层各自的桶距离，供 RViz 分层可视化 / 调参对比用。
  /// 外层下标与 Params::slices 一一对应。
  std::vector<std::vector<float>> per_slice_ranges;
  /// 每层实际落在有效高度区间内的点数，供日志和调参用。
  std::vector<std::size_t> per_slice_point_counts;
  /// 落入任何一层、且最终贡献了有效距离的桶数量。
  std::size_t occupied_bucket_count{0};
  /// 因为超出 [range_min, range_max] 而被丢弃的点数。
  std::size_t out_of_range_point_count{0};
};

/// 多层切片投影器。
class SliceProjector
{
public:
  struct Params
  {
    /// 扫描角度范围与分辨率，单位 rad。由 YAML 提供。
    double angle_min{0.0};
    double angle_max{0.0};
    double angle_increment{0.0};
    /// 有效距离区间，单位 m。
    double range_min{0.0};
    double range_max{0.0};
    /// 「该方向无障碍」时填入的值。按需求文档，默认填 range_max。
    float no_return_value{0.0F};
    /// 各切片层配置，至少 2 层（单层会被节点层拒绝）。
    std::vector<SliceConfig> slices;
  };

  SliceProjector() = default;

  /// 设置参数并预分配缓冲区。返回 false 表示参数自身不自洽（调用方应拒绝启动/拒绝本次更新）。
  /// error 会被填成人类可读的原因，便于直接打到 RCLCPP_ERROR。
  bool configure(const Params & params, std::string & error);

  /// 角度桶总数。configure 成功后才有意义。
  std::size_t bucket_count() const {return bucket_count_;}

  const Params & params() const {return params_;}

  /// 执行投影融合。points 必须已经变换到 base_frame。
  /// 本函数不分配新的大块内存（复用内部缓冲区），可在高频回调链路中反复调用。
  void project(const std::vector<SlicePoint> & points, ProjectionResult & result);

private:
  /// 把角度映射到桶下标；返回 false 表示该角度不在 [angle_min, angle_max] 内。
  bool angleToBucket(double angle, std::size_t & bucket) const;

  Params params_;
  std::size_t bucket_count_{0};
  bool configured_{false};

  /// 内部复用缓冲：[slice][bucket]
  std::vector<std::vector<float>> slice_min_range_;
  std::vector<std::vector<int>> slice_hit_count_;
};

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__SLICE_PROJECTOR_HPP_
