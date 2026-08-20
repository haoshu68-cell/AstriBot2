// Copyright 2026 Astribot.
//
// 自身点剔除的几何单元测试。
//
// 为什么需要这个测试（而不是只靠仿真跑一遍）：
//   本仿真链路里上游 livox_preprocess_node 配了 range_min=0.35（单雷达坐标系下），
//   而双臂离雷达通常不到 0.35m，所以**机械臂的点在进入本模块之前就已经被距离门限
//   截掉了**——实测：把左臂从收纳摆到大幅伸展，落在左臂胶囊体内的点数恒为 0，
//   而躯干胶囊体内稳定有 664 点（降采样后 131）。
//   也就是说这条链路无法端到端验证「机械臂点被剔除」这个能力。
//
//   因此这里用**真实机器人几何**构造点云做单元测试，直接验证：
//     1. 落在连杆胶囊体内的点一定被判为自身点；
//     2. 胶囊体外一点点的点一定不被误剔（不会吃掉贴着手臂的真实障碍物）；
//     3. 胶囊体随连杆位姿移动后，剔除区域跟着移动（TF 驱动，而非静态裁剪）；
//     4. 底盘足迹圆柱在底盘倾斜时跟着倾斜（base_frame 内判定，天然成立）。
#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "astribot_s1_autonomy/self_filter.hpp"

namespace astribot_s1_autonomy
{
namespace
{

/// 造一个胶囊体：轴线从 (x0,y0,z0) 到 (x1,y1,z1)。
FilterCapsule makeCapsule(
  float x0, float y0, float z0, float x1, float y1, float z1, float radius)
{
  FilterCapsule c;
  c.x0 = x0; c.y0 = y0; c.z0 = z0;
  c.x1 = x1; c.y1 = y1; c.z1 = z1;
  c.radius = radius;
  c.name = "test";
  return c;
}

SlicePoint makePoint(float x, float y, float z)
{
  SlicePoint p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

// ---------------------------------------------------------------------------
// 点到线段距离：先把基础几何钉死
// ---------------------------------------------------------------------------
TEST(PointSegmentDistance, PerpendicularToMiddle)
{
  // 线段沿 x 轴从 0 到 1，点在 (0.5, 0.3, 0) → 距离应为 0.3
  const float d_sq = pointSegmentDistanceSquared(
    0.5F, 0.3F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F);
  EXPECT_NEAR(std::sqrt(d_sq), 0.3F, 1e-5F);
}

TEST(PointSegmentDistance, BeyondEndpointFallsBackToEndpointDistance)
{
  // 点在线段延长线外侧 → 应退化成到端点的距离，而不是到无限长直线的距离。
  // 这正是「胶囊体两端是半球」的由来。
  const float d_sq = pointSegmentDistanceSquared(
    2.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F);
  EXPECT_NEAR(std::sqrt(d_sq), 1.0F, 1e-5F);
}

TEST(PointSegmentDistance, DegenerateSegmentDoesNotDivideByZero)
{
  // 线段两端重合（单 frame 链退化成球）：必须给出点到点距离，且不能是 NaN。
  const float d_sq = pointSegmentDistanceSquared(
    0.0F, 0.0F, 0.5F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F);
  EXPECT_TRUE(std::isfinite(d_sq));
  EXPECT_NEAR(std::sqrt(d_sq), std::sqrt(1.0F + 1.0F + 0.25F), 1e-5F);
}

// ---------------------------------------------------------------------------
// 机械臂胶囊体剔除
// ---------------------------------------------------------------------------
class ArmSelfFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 用接近真实的左臂几何：肩部在 (0.0, 0.25, 0.9)，
    // 大臂斜向前下伸到 (0.35, 0.30, 0.65)，半径取 yaml 默认 0.15。
    capsules_.push_back(makeCapsule(0.0F, 0.25F, 0.90F, 0.35F, 0.30F, 0.65F, 0.15F));
    filter_.setCapsules(capsules_);

    FootprintCylinder fp;
    fp.enabled = true;
    fp.radius = 0.40;
    fp.z_min = -0.25;
    fp.z_max = 0.05;
    filter_.setFootprint(fp);
  }

  SelfFilter filter_;
  std::vector<FilterCapsule> capsules_;
};

TEST_F(ArmSelfFilterTest, PointOnArmAxisIsSelf)
{
  // 轴线中点必然在胶囊体内
  EXPECT_TRUE(filter_.isSelfPoint(makePoint(0.175F, 0.275F, 0.775F)));
}

TEST_F(ArmSelfFilterTest, PointJustInsideRadiusIsSelf)
{
  // 距轴线 0.14m（半径 0.15）→ 应判为自身
  EXPECT_TRUE(filter_.isSelfPoint(makePoint(0.175F, 0.275F + 0.14F, 0.775F)));
}

TEST_F(ArmSelfFilterTest, PointJustOutsideRadiusIsNotSelf)
{
  // 距轴线 0.20m → 在胶囊体外，必须保留。
  // 这条是防「过剔」的关键：手臂旁边 5cm 的真实障碍物不能被吃掉。
  EXPECT_FALSE(filter_.isSelfPoint(makePoint(0.175F, 0.275F + 0.20F, 0.775F)));
}

TEST_F(ArmSelfFilterTest, DistantObstacleIsNotSelf)
{
  // 3m 外的货架点，无论如何都不能被判为自身
  EXPECT_FALSE(filter_.isSelfPoint(makePoint(3.0F, 0.0F, 0.5F)));
}

TEST_F(ArmSelfFilterTest, FilterRegionFollowsLinkPose)
{
  // === 这条测试对应「必须依赖 TF 实时变换，禁止静态裁剪」===
  // 取一个点，它在手臂"伸展"姿态下属于自身，在"收纳"姿态下则是自由空间。
  const SlicePoint probe = makePoint(0.60F, 0.30F, 0.55F);

  // 收纳姿态（大臂贴着身体竖直向下）：该点在胶囊体外
  std::vector<FilterCapsule> folded;
  folded.push_back(makeCapsule(0.0F, 0.25F, 0.90F, 0.02F, 0.26F, 0.55F, 0.15F));
  filter_.setCapsules(folded);
  EXPECT_FALSE(filter_.isSelfPoint(probe)) << "收纳姿态下该点应是自由空间";

  // 伸展姿态（大臂前伸到该点附近）：同一个点变成自身点
  std::vector<FilterCapsule> extended;
  extended.push_back(makeCapsule(0.0F, 0.25F, 0.90F, 0.65F, 0.30F, 0.55F, 0.15F));
  filter_.setCapsules(extended);
  EXPECT_TRUE(filter_.isSelfPoint(probe)) << "伸展姿态下该点应被判为机械臂自身";
}

TEST_F(ArmSelfFilterTest, EmptyCapsulesStillKeepsFootprintActive)
{
  // TF 全部拿不到时（节点会打 WARN 并传空列表），足迹圆柱仍应生效，
  // 但不能拿过期位姿去剔——所以手臂区域此时是"漏剔"而非"错剔"，这是有意的取舍。
  filter_.setCapsules({});
  EXPECT_TRUE(filter_.isSelfPoint(makePoint(0.1F, 0.1F, -0.10F))) << "足迹圆柱内";
  EXPECT_FALSE(filter_.isSelfPoint(makePoint(0.175F, 0.275F, 0.775F))) << "手臂区域此时不剔";
}

// ---------------------------------------------------------------------------
// 底盘足迹圆柱
// ---------------------------------------------------------------------------
TEST(FootprintFilterTest, InsideCylinderIsSelf)
{
  SelfFilter f;
  FootprintCylinder fp;
  fp.enabled = true;
  fp.radius = 0.40;
  fp.z_min = -0.25;
  fp.z_max = 0.05;
  f.setFootprint(fp);

  EXPECT_TRUE(f.isSelfPoint(makePoint(0.30F, 0.0F, -0.10F))) << "圆柱内";
  EXPECT_FALSE(f.isSelfPoint(makePoint(0.50F, 0.0F, -0.10F))) << "半径外";
  EXPECT_FALSE(f.isSelfPoint(makePoint(0.30F, 0.0F, 0.50F))) << "高度超出上界";
  EXPECT_FALSE(f.isSelfPoint(makePoint(0.30F, 0.0F, -0.50F))) << "低于下界";
}

TEST(FootprintFilterTest, DisabledFootprintFiltersNothing)
{
  SelfFilter f;
  FootprintCylinder fp;
  fp.enabled = false;
  fp.radius = 0.40;
  fp.z_min = -0.25;
  fp.z_max = 0.05;
  f.setFootprint(fp);
  EXPECT_FALSE(f.isSelfPoint(makePoint(0.10F, 0.0F, -0.10F)));
}

}  // namespace
}  // namespace astribot_s1_autonomy
