// Copyright 2026 Astribot.
//
// 多层切片投影融合的单元测试。
// 重点验证「为什么必须多层」——单层切片会漏掉哪些障碍物，多层为什么不会。
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "astribot_s1_autonomy/slice_projector.hpp"

namespace astribot_s1_autonomy
{
namespace
{

/// 造一份贴近本项目实际的 4 层配置（z 值为 astribot_torso_base 系，地面≈-0.129）。
SliceProjector::Params makeParams()
{
  SliceProjector::Params p;
  p.angle_min = -M_PI;
  p.angle_max = M_PI;
  p.angle_increment = M_PI / 180.0;   // 1°，测试里用粗一点便于数桶
  p.range_min = 0.35;
  p.range_max = 20.0;
  p.no_return_value = 20.0F;

  const struct
  {
    const char * name;
    double z_min;
    double z_max;
    int min_points;
    double max_range;
  } defs[] = {
    {"low_obstacle", -0.08, 0.12, 1, 6.0},
    {"main_nav", 0.12, 0.55, 1, 20.0},
    {"torso_high", 0.55, 1.05, 1, 12.0},
    {"overhead", 1.05, 1.50, 1, 8.0},
  };
  for (const auto & d : defs) {
    SliceConfig s;
    s.name = d.name;
    s.z_min = d.z_min;
    s.z_max = d.z_max;
    s.min_points = d.min_points;
    s.max_range = d.max_range;
    s.enabled = true;
    p.slices.push_back(s);
  }
  return p;
}

SlicePoint pt(float x, float y, float z)
{
  SlicePoint p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

/// 正前方（+x 轴）对应的桶下标。
std::size_t frontBucket(const SliceProjector & proj)
{
  const auto & p = proj.params();
  return static_cast<std::size_t>(std::round((0.0 - p.angle_min) / p.angle_increment));
}

// ---------------------------------------------------------------------------
// 参数校验：非法配置必须被拒绝而不是带着跑
// ---------------------------------------------------------------------------
TEST(SliceProjectorConfig, RejectsNonPositiveAngleIncrement)
{
  SliceProjector proj;
  SliceProjector::Params p = makeParams();
  p.angle_increment = 0.0;
  std::string err;
  EXPECT_FALSE(proj.configure(p, err));
  EXPECT_FALSE(err.empty());
}

TEST(SliceProjectorConfig, RejectsInvertedRange)
{
  SliceProjector proj;
  SliceProjector::Params p = makeParams();
  p.range_min = 5.0;
  p.range_max = 1.0;
  std::string err;
  EXPECT_FALSE(proj.configure(p, err));
}

TEST(SliceProjectorConfig, RejectsInvertedSliceHeights)
{
  SliceProjector proj;
  SliceProjector::Params p = makeParams();
  p.slices[1].z_min = 1.0;
  p.slices[1].z_max = 0.5;
  std::string err;
  EXPECT_FALSE(proj.configure(p, err));
}

TEST(SliceProjectorConfig, AcceptsRealisticConfig)
{
  SliceProjector proj;
  std::string err;
  ASSERT_TRUE(proj.configure(makeParams(), err)) << err;
  // ±π、1° 分辨率 → 361 个桶
  EXPECT_EQ(proj.bucket_count(), 361U);
}

// ---------------------------------------------------------------------------
// 核心：多层融合 vs 单层切片
// ---------------------------------------------------------------------------
TEST(SliceProjectorFusion, LowPalletWouldBeMissedBySingleMidSliceButIsCaught)
{
  // 场景：正前方 1.5m 处有一个 20cm 高的托盘（z≈-0.02，即离地约 11cm），
  // 正前方 5m 处有一面墙（z=0.3，主导航层高度）。
  //
  // 传统做法只切 [0.05, 0.6] 这一层（本项目既有 pointcloud_to_laserscan 的配置），
  // 在 torso_base 系里就是只看 z∈[0.05,0.6]，托盘的 -0.02 完全在窗口之外
  // ⇒ 单层切片只会报告 5m 处有墙，底盘会直接撞上 1.5m 处的托盘。
  //
  // 多层切片里 low_obstacle 层覆盖 [-0.08, 0.12)，托盘被抓到，
  // 跨层取 min 之后正前方距离应是 1.5m 而不是 5m。
  SliceProjector proj;
  std::string err;
  ASSERT_TRUE(proj.configure(makeParams(), err)) << err;

  std::vector<SlicePoint> cloud;
  cloud.push_back(pt(1.5F, 0.0F, -0.02F));   // 低矮托盘
  cloud.push_back(pt(5.0F, 0.0F, 0.30F));    // 远处墙面

  ProjectionResult r;
  proj.project(cloud, r);

  const std::size_t b = frontBucket(proj);
  ASSERT_LT(b, r.ranges.size());
  EXPECT_NEAR(r.ranges[b], 1.5F, 1e-3F) << "融合后应取最近的托盘距离";

  // 分层结果自查：低层看到 1.5m，主导航层看到 5m
  EXPECT_NEAR(r.per_slice_ranges[0][b], 1.5F, 1e-3F);
  EXPECT_NEAR(r.per_slice_ranges[1][b], 5.0F, 1e-3F);
}

TEST(SliceProjectorFusion, OverheadBeamIsCaptured)
{
  // 悬空横梁：离地 1.3m（z=1.17），底盘能过但躯干/头会撞，必须进 scan。
  SliceProjector proj;
  std::string err;
  ASSERT_TRUE(proj.configure(makeParams(), err)) << err;

  std::vector<SlicePoint> cloud;
  cloud.push_back(pt(2.0F, 0.0F, 1.17F));

  ProjectionResult r;
  proj.project(cloud, r);
  EXPECT_NEAR(r.ranges[frontBucket(proj)], 2.0F, 1e-3F);
  EXPECT_EQ(r.per_slice_point_counts[3], 1U) << "应落在 overhead 层";
}

TEST(SliceProjectorFusion, GroundPointsAreExcluded)
{
  // 地面点 z=-0.13（地面约 -0.129）低于最低层下界 -0.08 ⇒ 必须被排除，
  // 否则整个 scan 会被地面糊成一圈假障碍。
  SliceProjector proj;
  std::string err;
  ASSERT_TRUE(proj.configure(makeParams(), err)) << err;

  std::vector<SlicePoint> cloud;
  for (int i = 0; i < 50; ++i) {
    cloud.push_back(pt(1.0F + (0.05F * static_cast<float>(i)), 0.0F, -0.13F));
  }

  ProjectionResult r;
  proj.project(cloud, r);
  EXPECT_EQ(r.occupied_bucket_count, 0U) << "地面点不应产生任何障碍";
  EXPECT_NEAR(r.ranges[frontBucket(proj)], 20.0F, 1e-3F);
}

TEST(SliceProjectorFusion, TooHighPointsAreExcluded)
{
  // 屋顶/货架顶（z=2.5）高于最高层上界 1.50 ⇒ 排除
  SliceProjector proj;
  std::string err;
  ASSERT_TRUE(proj.configure(makeParams(), err)) << err;

  std::vector<SlicePoint> cloud;
  cloud.push_back(pt(3.0F, 0.0F, 2.5F));
  ProjectionResult r;
  proj.project(cloud, r);
  EXPECT_EQ(r.occupied_bucket_count, 0U);
}

TEST(SliceProjectorFusion, EmptyResultFillsRangeMax)
{
  // 需求规定：切片后无有效障碍物点时，全部距离填最大探测距离。
  SliceProjector proj;
  std::string err;
  ASSERT_TRUE(proj.configure(makeParams(), err)) << err;

  ProjectionResult r;
  proj.project({}, r);
  ASSERT_EQ(r.ranges.size(), proj.bucket_count());
  for (const float v : r.ranges) {
    EXPECT_NEAR(v, 20.0F, 1e-6F);
  }
  EXPECT_EQ(r.occupied_bucket_count, 0U);
}

// ---------------------------------------------------------------------------
// 抗噪：min_points 与 max_range
// ---------------------------------------------------------------------------
TEST(SliceProjectorNoise, SinglePointRejectedWhenMinPointsIsThree)
{
  SliceProjector proj;
  SliceProjector::Params p = makeParams();
  p.slices[0].min_points = 3;          // 贴地层要求 3 个点才认账
  std::string err;
  ASSERT_TRUE(proj.configure(p, err)) << err;

  // 只给 1 个孤点 → 判为噪声
  ProjectionResult r;
  proj.project({pt(1.5F, 0.0F, -0.02F)}, r);
  EXPECT_EQ(r.occupied_bucket_count, 0U) << "证据不足应被压制";

  // 同一方向给 3 个点 → 认账
  std::vector<SlicePoint> three;
  three.push_back(pt(1.50F, 0.0F, -0.02F));
  three.push_back(pt(1.51F, 0.0F, -0.03F));
  three.push_back(pt(1.52F, 0.0F, -0.04F));
  proj.project(three, r);
  EXPECT_EQ(r.occupied_bucket_count, 1U) << "证据充足应认账";
}

TEST(SliceProjectorNoise, PerSliceMaxRangeLimitsTrustDistance)
{
  // 贴地层 max_range=6.0：8m 处的贴地点（很可能是俯仰造成的地面误判）应被忽略；
  // 但同样距离上主导航层高度的点仍然有效。
  SliceProjector proj;
  std::string err;
  ASSERT_TRUE(proj.configure(makeParams(), err)) << err;

  ProjectionResult r;
  proj.project({pt(8.0F, 0.0F, -0.02F)}, r);
  EXPECT_EQ(r.occupied_bucket_count, 0U) << "超出贴地层信任距离，应忽略";

  proj.project({pt(8.0F, 0.0F, 0.30F)}, r);
  EXPECT_EQ(r.occupied_bucket_count, 1U) << "主导航层信任到 20m，应保留";
}

TEST(SliceProjectorNoise, DisabledSliceDoesNotContributeButStillCounts)
{
  SliceProjector proj;
  SliceProjector::Params p = makeParams();
  p.slices[0].enabled = false;         // 关掉贴地层
  std::string err;
  ASSERT_TRUE(proj.configure(p, err)) << err;

  ProjectionResult r;
  proj.project({pt(1.5F, 0.0F, -0.02F)}, r);
  EXPECT_EQ(r.occupied_bucket_count, 0U) << "关掉的层不参与融合";
  EXPECT_EQ(r.per_slice_point_counts[0], 1U) << "但仍统计点数，便于对比调参";
}

// ---------------------------------------------------------------------------
// 距离门限与脏数据
// ---------------------------------------------------------------------------
TEST(SliceProjectorRobustness, OutOfRangePointsAreCounted)
{
  SliceProjector proj;
  std::string err;
  ASSERT_TRUE(proj.configure(makeParams(), err)) << err;

  std::vector<SlicePoint> cloud;
  cloud.push_back(pt(0.10F, 0.0F, 0.3F));    // 近于 range_min
  cloud.push_back(pt(25.0F, 0.0F, 0.3F));    // 远于 range_max
  ProjectionResult r;
  proj.project(cloud, r);
  EXPECT_EQ(r.out_of_range_point_count, 2U);
  EXPECT_EQ(r.occupied_bucket_count, 0U);
}

TEST(SliceProjectorRobustness, NonFinitePointsDoNotCorruptOutput)
{
  // Livox 原始数据里 NaN/Inf 是常态，必须挡掉且不污染任何桶。
  SliceProjector proj;
  std::string err;
  ASSERT_TRUE(proj.configure(makeParams(), err)) << err;

  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<SlicePoint> cloud;
  cloud.push_back(pt(nan, 0.0F, 0.3F));
  cloud.push_back(pt(1.0F, nan, 0.3F));
  cloud.push_back(pt(1.0F, 0.0F, nan));
  cloud.push_back(pt(inf, 0.0F, 0.3F));
  cloud.push_back(pt(2.0F, 0.0F, 0.3F));     // 唯一有效点

  ProjectionResult r;
  proj.project(cloud, r);
  EXPECT_EQ(r.occupied_bucket_count, 1U);
  EXPECT_NEAR(r.ranges[frontBucket(proj)], 2.0F, 1e-3F);
  for (const float v : r.ranges) {
    EXPECT_TRUE(std::isfinite(v)) << "输出里不允许出现 NaN/Inf";
  }
}

TEST(SliceProjectorRobustness, ProjectBeforeConfigureReturnsEmptyScanNotCrash)
{
  // 未配置就调用属于调用方错误，但绝不能崩。
  SliceProjector proj;
  ProjectionResult r;
  proj.project({pt(1.0F, 0.0F, 0.3F)}, r);
  EXPECT_TRUE(r.ranges.empty());
  EXPECT_EQ(r.occupied_bucket_count, 0U);
}

TEST(SliceProjectorGeometry, AnglesMapToExpectedBuckets)
{
  SliceProjector proj;
  std::string err;
  ASSERT_TRUE(proj.configure(makeParams(), err)) << err;

  // 正左方（+y）应落在 +90° 的桶
  ProjectionResult r;
  proj.project({pt(0.0F, 2.0F, 0.3F)}, r);
  const auto left = static_cast<std::size_t>(
    std::round((M_PI_2 - proj.params().angle_min) / proj.params().angle_increment));
  EXPECT_NEAR(r.ranges[left], 2.0F, 1e-3F);

  // 正后方（-x）
  proj.project({pt(-3.0F, 0.0F, 0.3F)}, r);
  const auto back = static_cast<std::size_t>(
    std::round((M_PI - proj.params().angle_min) / proj.params().angle_increment));
  ASSERT_LT(back, r.ranges.size());
  EXPECT_NEAR(r.ranges[back], 3.0F, 1e-3F);
}

}  // namespace
}  // namespace astribot_s1_autonomy
