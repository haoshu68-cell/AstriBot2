// Copyright 2026 Astribot.
#include "astribot_s1_autonomy/self_filter.hpp"

#include <algorithm>
#include <cmath>

namespace astribot_s1_autonomy
{

float pointSegmentDistanceSquared(
  float px, float py, float pz,
  float ax, float ay, float az,
  float bx, float by, float bz)
{
  // 记 ab 为线段向量、ap 为点相对起点的向量，
  // t = clamp(dot(ap, ab) / |ab|^2, 0, 1) 就是点在线段上的最近投影参数。
  // clamp 把投影落在线段外的情况自动收到端点，于是同一个公式同时覆盖
  // 「侧面圆柱」和「两端半球」，这也是用胶囊体的实现便利之处。
  const float abx = bx - ax;
  const float aby = by - ay;
  const float abz = bz - az;
  const float apx = px - ax;
  const float apy = py - ay;
  const float apz = pz - az;

  const float ab_len_sq = (abx * abx) + (aby * aby) + (abz * abz);

  float t = 0.0F;
  // 线段退化成一个点时 ab_len_sq==0，直接取 t=0 退化为点到点距离，避免除零。
  constexpr float kDegenerateEpsSq = 1e-12F;
  if (ab_len_sq > kDegenerateEpsSq) {
    t = ((apx * abx) + (apy * aby) + (apz * abz)) / ab_len_sq;
    t = std::clamp(t, 0.0F, 1.0F);
  }

  const float dx = apx - (t * abx);
  const float dy = apy - (t * aby);
  const float dz = apz - (t * abz);
  return (dx * dx) + (dy * dy) + (dz * dz);
}

bool SelfFilter::isSelfPoint(const SlicePoint & p) const
{
  // ---- 1) 底盘足迹圆柱（跟随 base_frame，底盘倾斜时一起倾斜）----
  if (footprint_.enabled) {
    const double z = static_cast<double>(p.z);
    if (z >= footprint_.z_min && z <= footprint_.z_max) {
      const double r_sq = (static_cast<double>(p.x) * static_cast<double>(p.x)) +
        (static_cast<double>(p.y) * static_cast<double>(p.y));
      if (r_sq <= footprint_.radius * footprint_.radius) {
        return true;
      }
    }
  }

  // ---- 2) 双臂/躯干连杆胶囊体（每帧由 TF 刷新）----
  for (const FilterCapsule & c : capsules_) {
    if (!(c.radius > 0.0F)) {
      continue;
    }
    const float d_sq = pointSegmentDistanceSquared(
      p.x, p.y, p.z,
      c.x0, c.y0, c.z0,
      c.x1, c.y1, c.z1);
    if (d_sq <= (c.radius * c.radius)) {
      return true;
    }
  }

  return false;
}

}  // namespace astribot_s1_autonomy
