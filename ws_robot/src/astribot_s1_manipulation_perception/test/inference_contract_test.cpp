#include "astribot_s1_manipulation_perception/inference_contract.hpp"
#include <cstring>
#include <gtest/gtest.h>
#include <limits>
using namespace astribot::inference;
sensor_msgs::msg::PointCloud2 cloud() {
  sensor_msgs::msg::PointCloud2 c;
  c.header.frame_id = "camera";
  c.header.stamp.sec = 10;
  c.width = 4;
  c.height = 2;
  c.point_step = 16;
  c.row_step = 80;
  c.data.resize(160);
  for (int i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = std::string(1, "xyz"[i]);
    f.offset = i * 4;
    f.datatype = f.FLOAT32;
    f.count = 1;
    c.fields.push_back(f);
  }
  for (unsigned y = 0; y < 2; ++y)
    for (unsigned x = 0; x < 4; ++x) {
      float p[3] = {.1f, .2f, 1.f};
      std::memcpy(c.data.data() + 80 * y + 16 * x, p, 12);
    }
  return c;
}
TEST(CloudContract, ReadsOrganizedCloudWithRowPadding) {
  auto c = cloud();
  auto v = decode_cloud(c, 1);
  ASSERT_EQ(v.size(), 24u);
  EXPECT_FLOAT_EQ(v[23], 1.f);
}
TEST(CloudContract, RejectsTruncatedRow) {
  auto c = cloud();
  c.data.pop_back();
  EXPECT_THROW(decode_cloud(c, 1), std::runtime_error);
}
TEST(CloudContract, RejectsOverlappingFields) {
  auto c = cloud();
  c.fields[1].offset = 0;
  EXPECT_THROW(decode_cloud(c, 1), std::runtime_error);
}
TEST(CloudContract, RejectsEndianAndNan) {
  auto c = cloud();
  c.is_bigendian = true;
  EXPECT_THROW(decode_cloud(c, 1), std::runtime_error);
  c.is_bigendian = false;
  float n = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(c.data.data(), &n, 4);
  EXPECT_THROW(decode_cloud(c, 1), std::runtime_error);
}
TEST(CloudContract, RejectsMissingAndWrongType) {
  auto c = cloud();
  c.fields.pop_back();
  EXPECT_THROW(decode_cloud(c, 1), std::runtime_error);
  c = cloud();
  c.fields[0].datatype = 8;
  EXPECT_THROW(decode_cloud(c, 1), std::runtime_error);
}
TEST(CloudContract, RejectsHugeDimensionsAndTooFewPoints) {
  auto c = cloud();
  c.width = 0xffffffff;
  c.height = 0xffffffff;
  EXPECT_THROW(decode_cloud(c, 1), std::runtime_error);
  EXPECT_THROW(decode_cloud(cloud(), 100), std::runtime_error);
}
TEST(RotationContract, RejectsReflectionsAndNonfinite) {
  double r[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  EXPECT_TRUE(valid_rotation(r));
  r[8] = -1;
  EXPECT_FALSE(valid_rotation(r));
  r[8] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(valid_rotation(r));
}
TEST(RotationContract, RejectsScaledOrSkewedMatrix) {
  double r[9] = {1, 0, .02, 0, 1, 0, 0, 0, 1};
  EXPECT_FALSE(valid_rotation(r));
}
TEST(SnapshotContract, RequiresFreshNonfutureStampAndBoundedExpiry) {
  EXPECT_TRUE(valid_window(10e9, 10.2e9, 12e9, .5, 5));
  EXPECT_FALSE(valid_window(10e9, 11e9, 12e9, .5, 5));
  EXPECT_FALSE(valid_window(11e9, 10e9, 12e9, .5, 5));
  EXPECT_FALSE(valid_window(10e9, 10.2e9, 20e9, .5, 5));
  EXPECT_FALSE(valid_window(0, 10e9, 12e9, .5, 5));
}
