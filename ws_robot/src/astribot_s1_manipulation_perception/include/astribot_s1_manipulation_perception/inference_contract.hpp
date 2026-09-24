#pragma once
#include <cstdint>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdexcept>
#include <vector>
namespace astribot::inference {
// Strict XYZ camera-optical snapshot: malformed/nonfinite data is not silently
// dropped.
std::vector<float> decode_cloud(const sensor_msgs::msg::PointCloud2 &,
                                size_t min_points = 64,
                                size_t max_points = 200000);
bool valid_rotation(const double *row_major);
bool valid_window(int64_t sample_ns, int64_t now_ns, int64_t until_ns,
                  double max_age, double max_validity);
} // namespace astribot::inference
