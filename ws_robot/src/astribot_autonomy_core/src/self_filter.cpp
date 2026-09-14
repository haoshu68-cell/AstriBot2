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
  const float abx = bx - ax;
  const float aby = by - ay;
  const float abz = bz - az;
  const float apx = px - ax;
  const float apy = py - ay;
  const float apz = pz - az;

  const float ab_len_sq = (abx * abx) + (aby * aby) + (abz * abz);

  float t = 0.0F;
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
