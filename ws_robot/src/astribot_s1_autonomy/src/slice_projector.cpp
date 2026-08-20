// Copyright 2026 Astribot.
#include "astribot_s1_autonomy/slice_projector.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace astribot_s1_autonomy
{

namespace
{
/// 角度桶数量的上限保护。按 0.1° 分辨率算 360° 也只有 3600 个桶，
/// 给到 100000 足够宽松；纯粹是为了防止 YAML 里把 angle_increment 写成 0 附近的
/// 极小值时分配出天量内存（对应「循环逻辑增加最大迭代次数保护」）。
constexpr std::size_t kMaxBucketCount = 100000U;
}  // namespace

bool SliceProjector::configure(const Params & params, std::string & error)
{
  configured_ = false;
  std::ostringstream oss;

  if (!(params.angle_increment > 0.0)) {
    oss << "angle_increment 必须为正，当前=" << params.angle_increment;
    error = oss.str();
    return false;
  }
  if (!(params.angle_max > params.angle_min)) {
    oss << "要求 angle_max > angle_min，当前 min=" << params.angle_min
        << " max=" << params.angle_max;
    error = oss.str();
    return false;
  }
  if (!(params.range_min >= 0.0) || !(params.range_max > params.range_min)) {
    oss << "要求 0 <= range_min < range_max，当前 min=" << params.range_min
        << " max=" << params.range_max;
    error = oss.str();
    return false;
  }

  const double span = params.angle_max - params.angle_min;
  const double raw_count = std::floor(span / params.angle_increment) + 1.0;
  if (!(raw_count >= 2.0) || raw_count > static_cast<double>(kMaxBucketCount)) {
    oss << "由 angle_min/max/increment 推出的桶数=" << raw_count
        << " 超出合理范围 [2, " << kMaxBucketCount << "]";
    error = oss.str();
    return false;
  }

  if (params.slices.empty()) {
    error = "切片层列表为空";
    return false;
  }

  // 逐层校验。注意这里只校验「层自身是否自洽」，
  // 层数是否 >= 2 由节点层判定（那是需求约束，不是算法约束）。
  for (std::size_t i = 0; i < params.slices.size(); ++i) {
    const SliceConfig & s = params.slices[i];
    if (!(s.z_max > s.z_min)) {
      oss << "切片层[" << i << "]" << (s.name.empty() ? "" : (" '" + s.name + "'"))
          << " 要求 z_max > z_min，当前 z_min=" << s.z_min << " z_max=" << s.z_max;
      error = oss.str();
      return false;
    }
    if (s.min_points < 1) {
      oss << "切片层[" << i << "] min_points 必须 >= 1，当前=" << s.min_points;
      error = oss.str();
      return false;
    }
    if (!(s.max_range > 0.0)) {
      oss << "切片层[" << i << "] max_range 必须为正，当前=" << s.max_range;
      error = oss.str();
      return false;
    }
  }

  params_ = params;
  bucket_count_ = static_cast<std::size_t>(raw_count);

  slice_min_range_.assign(params_.slices.size(), std::vector<float>(bucket_count_, 0.0F));
  slice_hit_count_.assign(params_.slices.size(), std::vector<int>(bucket_count_, 0));

  configured_ = true;
  error.clear();
  return true;
}

bool SliceProjector::angleToBucket(double angle, std::size_t & bucket) const
{
  if (angle < params_.angle_min || angle > params_.angle_max) {
    return false;
  }
  const double idx = std::round((angle - params_.angle_min) / params_.angle_increment);
  if (idx < 0.0) {
    return false;
  }
  const std::size_t candidate = static_cast<std::size_t>(idx);
  if (candidate >= bucket_count_) {
    // 浮点舍入可能把最后一个角度顶出去一格，夹回最后一个桶而不是丢弃，
    // 避免 360° 扫描在接缝处出现一个恒定空洞。
    bucket = bucket_count_ - 1U;
    return true;
  }
  bucket = candidate;
  return true;
}

void SliceProjector::project(const std::vector<SlicePoint> & points, ProjectionResult & result)
{
  const std::size_t slice_count = params_.slices.size();

  result.ranges.assign(bucket_count_, params_.no_return_value);
  result.per_slice_ranges.assign(slice_count, std::vector<float>());
  result.per_slice_point_counts.assign(slice_count, 0U);
  result.occupied_bucket_count = 0U;
  result.out_of_range_point_count = 0U;

  if (!configured_) {
    // 未配置就调用属于调用方的编程错误；这里保持「不崩溃」，返回全空扫描。
    return;
  }

  // 复位内部缓冲。用 infinity 作为「本层本桶还没有观测」的哨兵值。
  constexpr float kInf = std::numeric_limits<float>::infinity();
  for (std::size_t s = 0; s < slice_count; ++s) {
    std::fill(slice_min_range_[s].begin(), slice_min_range_[s].end(), kInf);
    std::fill(slice_hit_count_[s].begin(), slice_hit_count_[s].end(), 0);
  }

  // ---- 第一遍：逐点分桶，按层累计「最近距离」和「证据数」 ----
  for (const SlicePoint & p : points) {
    // 非有限值（NaN/Inf）在 Livox 原始数据里是常态，必须显式挡掉，
    // 否则 atan2/比较会把脏值传播进 scan。
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }

    const double range = std::hypot(static_cast<double>(p.x), static_cast<double>(p.y));
    if (range < params_.range_min || range > params_.range_max) {
      ++result.out_of_range_point_count;
      continue;
    }

    std::size_t bucket = 0U;
    const double angle = std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));
    if (!angleToBucket(angle, bucket)) {
      continue;
    }

    const float range_f = static_cast<float>(range);
    for (std::size_t s = 0; s < slice_count; ++s) {
      const SliceConfig & cfg = params_.slices[s];
      // 高度区间取左闭右开，保证相邻层不会把同一个点重复计两次。
      if (static_cast<double>(p.z) < cfg.z_min || static_cast<double>(p.z) >= cfg.z_max) {
        continue;
      }
      ++result.per_slice_point_counts[s];

      // 未启用的层照样统计点数（便于对比调参），但不参与距离累计。
      if (!cfg.enabled) {
        continue;
      }
      if (range > cfg.max_range) {
        continue;
      }
      slice_min_range_[s][bucket] = std::min(slice_min_range_[s][bucket], range_f);
      ++slice_hit_count_[s][bucket];
    }
  }

  // ---- 第二遍：逐层做证据数判决，再跨层取最近距离做融合 ----
  for (std::size_t s = 0; s < slice_count; ++s) {
    const SliceConfig & cfg = params_.slices[s];
    std::vector<float> & per_slice = result.per_slice_ranges[s];
    per_slice.assign(bucket_count_, params_.no_return_value);

    if (!cfg.enabled) {
      continue;
    }
    for (std::size_t b = 0; b < bucket_count_; ++b) {
      if (slice_hit_count_[s][b] < cfg.min_points) {
        continue;   // 证据不足，判为噪点
      }
      const float r = slice_min_range_[s][b];
      if (!std::isfinite(r)) {
        continue;
      }
      per_slice[b] = r;
      if (r < result.ranges[b]) {
        result.ranges[b] = r;
      }
    }
  }

  for (std::size_t b = 0; b < bucket_count_; ++b) {
    if (result.ranges[b] < params_.no_return_value) {
      ++result.occupied_bucket_count;
    }
  }
}

}  // namespace astribot_s1_autonomy
