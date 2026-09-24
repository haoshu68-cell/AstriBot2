#include "astribot_s1_manipulation_perception/inference_contract.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
namespace astribot::inference {
std::vector<float> decode_cloud(const sensor_msgs::msg::PointCloud2 &c,
                                size_t min_points, size_t max_points) {
  const uint64_t count = uint64_t(c.width) * c.height;
  if (c.is_bigendian || count < min_points || count > max_points ||
      c.point_step < 12 || c.point_step > 256 ||
      uint64_t(c.row_step) < uint64_t(c.width) * c.point_step ||
      uint64_t(c.row_step) * c.height != c.data.size() ||
      c.data.size() > 64000000)
    throw std::runtime_error("INVALID_CLOUD_LAYOUT");
  std::array<int, 3> offsets{-1, -1, -1};
  for (const auto &f : c.fields)
    for (size_t a = 0; a < 3; ++a)
      if (f.name == std::string(1, "xyz"[a])) {
        if (offsets[a] != -1 ||
            f.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
            f.count != 1 || uint64_t(f.offset) + 4 > c.point_step)
          throw std::runtime_error("INVALID_CLOUD_FIELDS");
        offsets[a] = static_cast<int>(f.offset);
      }
  for (size_t a = 0; a < 3; ++a) {
    if (offsets[a] < 0)
      throw std::runtime_error("MISSING_XYZ");
    for (size_t b = 0; b < a; ++b)
      if (std::abs(offsets[a] - offsets[b]) < 4)
        throw std::runtime_error("OVERLAPPING_XYZ");
  }
  std::vector<float> points;
  points.reserve(count * 3);
  for (uint32_t row = 0; row < c.height; ++row)
    for (uint32_t col = 0; col < c.width; ++col) {
      float p[3];
      const auto *src = c.data.data() + uint64_t(row) * c.row_step +
                        uint64_t(col) * c.point_step;
      for (size_t a = 0; a < 3; ++a) {
        std::memcpy(&p[a], src + offsets[a], 4);
        if (!std::isfinite(p[a]) || std::abs(p[a]) > 10.f)
          throw std::runtime_error("INVALID_CLOUD_POINT");
      }
      if (p[2] <= 0)
        throw std::runtime_error("NONOPTICAL_CLOUD");
      points.insert(points.end(), p, p + 3);
    }
  return points;
}
bool valid_rotation(const double *r) {
  for (size_t i = 0; i < 9; ++i)
    if (!std::isfinite(r[i]))
      return false;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      double d = 0;
      for (int k = 0; k < 3; ++k)
        d += r[k * 3 + i] * r[k * 3 + j];
      if (std::abs(d - (i == j ? 1. : 0.)) > 1e-3)
        return false;
    }
  double det = r[0] * (r[4] * r[8] - r[5] * r[7]) -
               r[1] * (r[3] * r[8] - r[5] * r[6]) +
               r[2] * (r[3] * r[7] - r[4] * r[6]);
  return std::abs(det - 1.) < 1e-3;
}
bool valid_window(int64_t sample, int64_t now, int64_t until, double max_age,
                  double max_validity) {
  return sample > 0 && now >= sample &&
         double(now - sample) * 1e-9 <= max_age && until > now &&
         double(until - sample) * 1e-9 <= max_validity;
}
} // namespace astribot::inference
