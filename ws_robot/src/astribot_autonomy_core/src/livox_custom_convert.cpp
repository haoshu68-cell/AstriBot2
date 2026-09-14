// Copyright 2026 Astribot.
#include "astribot_s1_autonomy/livox_custom_convert.hpp"

#include <cmath>
#include <sstream>

namespace astribot_s1_autonomy
{

namespace
{
/// Livox tag 的 bit0-1 是空间置信度（0 最好）。
inline std::uint8_t spatial_noise_bits(std::uint8_t tag)
{
  return static_cast<std::uint8_t>(tag & 0x03U);
}
}  // namespace

void LivoxConvertConfig::validate() const
{
  std::ostringstream oss;
  if (!std::isfinite(null_x) || !std::isfinite(null_y) || !std::isfinite(null_z)) {
    oss << "null_xyz 含非有限数 (" << null_x << ", " << null_y << ", " << null_z << ")";
    throw LivoxConvertConfigError(oss.str());
  }
  if (!(null_radius >= 0.0F) || !std::isfinite(null_radius)) {
    oss << "null_radius=" << null_radius << " 必须是非负有限数（0 表示不做无效点剔除）";
    throw LivoxConvertConfigError(oss.str());
  }
  if (!std::isfinite(min_range) || min_range < 0.0F) {
    oss << "min_range=" << min_range << " 必须是非负有限数";
    throw LivoxConvertConfigError(oss.str());
  }
  if (!std::isfinite(max_range) || max_range < 0.0F) {
    oss << "max_range=" << max_range << " 必须是非负有限数";
    throw LivoxConvertConfigError(oss.str());
  }
  if (max_range > 0.0F && min_range > 0.0F && max_range <= min_range) {
    oss << "max_range=" << max_range << " 必须大于 min_range=" << min_range
        << "；否则每一帧都会被清空，而表现是「雷达好像没数据」，"
           "排查方向会被带到驱动那边去";
    throw LivoxConvertConfigError(oss.str());
  }
  if (max_spatial_noise > 4U) {
    oss << "max_spatial_noise=" << static_cast<int>(max_spatial_noise)
        << " 超出范围；Livox tag 的 bit0-1 只有 0~3，阈值 4 表示不丢弃";
    throw LivoxConvertConfigError(oss.str());
  }
}

double LivoxConvertStats::kept_ratio() const
{
  if (total == 0U) {
    return 0.0;
  }
  return static_cast<double>(kept) / static_cast<double>(total);
}

bool ConvertedPoint::operator==(const ConvertedPoint & o) const
{
  return x == o.x && y == o.y && z == o.z &&
         intensity == o.intensity && time == o.time &&
         ring == o.ring && tag == o.tag;
}

bool should_drop(const LivoxPoint & p, const LivoxConvertConfig & cfg,
                 LivoxConvertStats * stats)
{
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
    if (stats != nullptr) {
      ++stats->dropped_nonfinite;
    }
    return true;
  }

  if (cfg.null_radius > 0.0F) {
    const float dx = p.x - cfg.null_x;
    const float dy = p.y - cfg.null_y;
    const float dz = p.z - cfg.null_z;
    if (dx * dx + dy * dy + dz * dz <= cfg.null_radius * cfg.null_radius) {
      if (stats != nullptr) {
        ++stats->dropped_null;
      }
      return true;
    }
  }

  if (cfg.min_range > 0.0F || cfg.max_range > 0.0F) {
    const float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
    if (cfg.min_range > 0.0F && r2 < cfg.min_range * cfg.min_range) {
      if (stats != nullptr) {
        ++stats->dropped_range;
      }
      return true;
    }
    if (cfg.max_range > 0.0F && r2 > cfg.max_range * cfg.max_range) {
      if (stats != nullptr) {
        ++stats->dropped_range;
      }
      return true;
    }
  }

  if (cfg.max_spatial_noise <= 3U &&
      spatial_noise_bits(p.tag) >= cfg.max_spatial_noise)
  {
    if (stats != nullptr) {
      ++stats->dropped_noise;
    }
    return true;
  }

  return false;
}

LivoxConvertStats convert_frame(const std::vector<LivoxPoint> & in,
                                std::uint64_t timebase,
                                std::uint64_t header_stamp_ns,
                                const LivoxConvertConfig & cfg,
                                std::vector<ConvertedPoint> * out)
{
  cfg.validate();
  LivoxConvertStats stats;
  stats.total = in.size();
  if (out == nullptr) {
    return stats;
  }
  out->clear();
  out->reserve(in.size());

  for (const LivoxPoint & p : in) {
    if (should_drop(p, cfg, &stats)) {
      continue;
    }
    ConvertedPoint q;
    q.x = p.x;
    q.y = p.y;
    q.z = p.z;
    q.intensity = static_cast<float>(p.reflectivity);
    const std::int64_t abs_ns =
      static_cast<std::int64_t>(timebase) + static_cast<std::int64_t>(p.offset_time);
    const std::int64_t rel_ns = abs_ns - static_cast<std::int64_t>(header_stamp_ns);
    q.time = static_cast<float>(static_cast<double>(rel_ns) * 1e-9);
    q.ring = static_cast<std::uint16_t>(p.line);
    q.tag = p.tag;
    out->push_back(q);
  }

  stats.kept = out->size();
  return stats;
}

}  // namespace astribot_s1_autonomy
