// Copyright 2026 Astribot.
//
// livox_custom_convert.hpp 的实现。纯逻辑，不碰 ROS。
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
  // 顺序有意为之：先 NaN，再无效点，再距离，最后噪点位。
  // 这样统计量能指向「最根本」的那个原因，而不是恰好先命中的那个。
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
    // timebase 与 header_stamp 谁大都有可能（驱动用到达时刻打戳时 header 会晚于
    // timebase），所以这里必须允许负数。
    //
    // 精确一点说明为什么用 int64 而不是「uint64 相减再转 int64」：后者其实也对
    // ——补码下 uint64 的模 2^64 回绕再转回 int64 恰好还原出正确的负值
    //（实测两种写法给出同一个 -999999995）。真正会出事的是**把无符号差值直接
    // 转成浮点**，例如 `double t = (timebase + offset - header) * 1e-9;`，
    // 那会得到约 1.8e10 秒（约 585 年），而且不报错、下游只会看到「这一点来自
    // 遥远未来」。用 int64 全程有符号是为了让这个陷阱压根没有出现的机会，
    // 而不是因为无符号相减本身错。
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
