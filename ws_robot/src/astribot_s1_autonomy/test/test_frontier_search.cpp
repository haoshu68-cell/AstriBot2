// Copyright 2026 Astribot.
//
// 前沿搜索的单元测试。用手工构造的小地图把每条异常规则都钉死：
// 目标点不落在障碍物内、不输出过近目标、被包围的前沿判为无效、
// 空地图不崩、采样数随面积自适应、历史访问惩罚生效。
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "astribot_s1_autonomy/frontier_search.hpp"

namespace astribot_s1_autonomy
{
namespace
{

constexpr int8_t kUnknown = -1;
constexpr int8_t kFree = 0;
constexpr int8_t kOccupied = 100;

/// 造一张 w×h 的地图，默认全未知，分辨率 0.05（与本项目 SLAM 一致）。
GridMap makeMap(unsigned int w, unsigned int h, int8_t fill = kUnknown)
{
  GridMap m;
  m.width = w;
  m.height = h;
  m.resolution = 0.05;
  m.origin_x = 0.0;
  m.origin_y = 0.0;
  m.data.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), fill);
  return m;
}

/// 把矩形区域 [x0,x1]×[y0,y1] 填成某个值。
void fillRect(GridMap & m, unsigned int x0, unsigned int y0,
  unsigned int x1, unsigned int y1, int8_t v)
{
  for (unsigned int y = y0; y <= y1 && y < m.height; ++y) {
    for (unsigned int x = x0; x <= x1 && x < m.width; ++x) {
      m.data[m.index(x, y)] = v;
    }
  }
}

/// 一套宽松的默认参数：测试里主要验证逻辑，不想被过严的阈值挡住。
FrontierSearchParams makeParams()
{
  FrontierSearchParams p;
  p.occupied_threshold = 65;
  p.free_threshold = 25;
  p.obstacle_inflation_radius = 0.0;    // 默认不膨胀，个别用例单独打开
  p.min_obstacle_cluster_cells = 0;
  p.use_eight_connectivity = true;
  p.min_frontier_cells = 1;
  p.gain_window_radius = 0.5;
  p.adaptive_sample_gain = 0.5;
  p.min_samples_per_cluster = 1;
  p.max_samples_per_cluster = 8;
  p.required_clearance_radius = 0.0;
  p.min_goal_distance = 0.0;
  p.max_goal_distance = 0.0;
  p.weight_distance = 1.0;
  p.weight_gain = 1.0;
  p.weight_visit_penalty = 1.0;
  p.visit_penalty_radius = 0.5;
  p.random_seed = 42U;
  return p;
}

// ---------------------------------------------------------------------------
// 异常输入：不能崩
// ---------------------------------------------------------------------------
TEST(FrontierSearchRobustness, EmptyMapProducesNoGoal)
{
  FrontierSearch s;
  std::string err;
  ASSERT_TRUE(s.configure(makeParams(), err)) << err;

  GridMap empty;
  FrontierSearch::Result r;
  s.search(empty, 0.0, 0.0, {}, r);
  EXPECT_EQ(r.best_candidate_index, -1);
  EXPECT_FALSE(r.summary.empty());
}

TEST(FrontierSearchRobustness, InconsistentDataLengthIsRejected)
{
  FrontierSearch s;
  std::string err;
  ASSERT_TRUE(s.configure(makeParams(), err)) << err;

  GridMap m = makeMap(10, 10);
  m.data.resize(50);          // 故意和 width*height 不一致
  FrontierSearch::Result r;
  s.search(m, 0.0, 0.0, {}, r);
  EXPECT_EQ(r.best_candidate_index, -1);
}

TEST(FrontierSearchRobustness, SearchBeforeConfigureDoesNotCrash)
{
  FrontierSearch s;
  GridMap m = makeMap(10, 10, kFree);
  FrontierSearch::Result r;
  s.search(m, 0.0, 0.0, {}, r);
  EXPECT_EQ(r.best_candidate_index, -1);
}

TEST(FrontierSearchConfig, RejectsInvertedThresholds)
{
  FrontierSearch s;
  FrontierSearchParams p = makeParams();
  p.free_threshold = 80;
  p.occupied_threshold = 20;
  std::string err;
  EXPECT_FALSE(s.configure(p, err));
  EXPECT_FALSE(err.empty());
}

// ---------------------------------------------------------------------------
// 前沿提取
// ---------------------------------------------------------------------------
TEST(FrontierExtraction, FullyExploredMapHasNoFrontier)
{
  // 全空闲、无未知格 ⇒ 没有前沿 ⇒ 上层应判定探索完成
  FrontierSearch s;
  std::string err;
  ASSERT_TRUE(s.configure(makeParams(), err)) << err;

  GridMap m = makeMap(40, 40, kFree);
  FrontierSearch::Result r;
  s.search(m, 1.0, 1.0, {}, r);
  EXPECT_EQ(r.raw_frontier_cell_count, 0U);
  EXPECT_EQ(r.best_candidate_index, -1);
}

TEST(FrontierExtraction, FreeUnknownBoundaryBecomesFrontier)
{
  // 左半空闲、右半未知 ⇒ 中间竖直交界线应被识别为前沿
  FrontierSearch s;
  std::string err;
  ASSERT_TRUE(s.configure(makeParams(), err)) << err;

  GridMap m = makeMap(40, 40, kUnknown);
  fillRect(m, 0, 0, 19, 39, kFree);
  FrontierSearch::Result r;
  s.search(m, 0.25, 1.0, {}, r);

  EXPECT_GT(r.raw_frontier_cell_count, 0U);
  ASSERT_FALSE(r.clusters.empty());
  // 前沿应集中在 x≈19 格（世界坐标 ≈ 0.975m）
  EXPECT_NEAR(r.clusters.front().centroid_x, 0.975, 0.10);
}

TEST(FrontierExtraction, GoalIsNeverInsideObstacle)
{
  // === 对应「生成的目标点落在障碍物内部：直接丢弃」===
  // 空闲区和未知区之间夹一道墙：墙后的未知区不该产生可用目标。
  FrontierSearch s;
  FrontierSearchParams p = makeParams();
  p.obstacle_inflation_radius = 0.10;
  std::string err;
  ASSERT_TRUE(s.configure(p, err)) << err;

  GridMap m = makeMap(40, 40, kUnknown);
  fillRect(m, 0, 0, 18, 39, kFree);
  fillRect(m, 19, 0, 20, 39, kOccupied);    // 竖直墙
  FrontierSearch::Result r;
  s.search(m, 0.25, 1.0, {}, r);

  // 所有被采纳的候选点都不能落在占据格上
  for (const GoalCandidate & c : r.candidates) {
    if (!c.valid) {
      continue;
    }
    unsigned int mx = 0U;
    unsigned int my = 0U;
    ASSERT_TRUE(m.worldToMap(c.x, c.y, mx, my));
    EXPECT_LT(m.data[m.index(mx, my)], static_cast<int8_t>(p.occupied_threshold))
      << "目标点落在了障碍物内: (" << c.x << ", " << c.y << ")";
  }
}

TEST(FrontierExtraction, EnclosedFrontierIsRejectedAsUnreachable)
{
  // === 对应「过滤被障碍物包围的无效前沿块」===
  // 布局：左边一块空闲区(机器人在此) | 实墙 | 右边一块被墙封住的空闲区+未知区。
  // 右边那块空闲-未知交界虽然几何上是前沿，但机器人过不去 ⇒ 必须判无效。
  FrontierSearch s;
  std::string err;
  ASSERT_TRUE(s.configure(makeParams(), err)) << err;

  GridMap m = makeMap(60, 30, kUnknown);
  fillRect(m, 0, 0, 20, 29, kFree);         // 机器人所在区域
  fillRect(m, 21, 0, 23, 29, kOccupied);    // 完全封死的墙
  fillRect(m, 24, 10, 30, 20, kFree);       // 墙后的独立空闲腔
  // 墙后空闲腔右侧仍是未知 ⇒ 会产生前沿格

  FrontierSearch::Result r;
  s.search(m, 0.5, 0.75, {}, r);

  bool found_unreachable_reject = false;
  for (const FrontierCluster & c : r.clusters) {
    // 墙后那块（x > 1.2m）应被判为不可达
    if (c.centroid_x > 1.2 && !c.accepted) {
      found_unreachable_reject = true;
      EXPECT_EQ(c.reject_reason, "被障碍物包围/不可达");
    }
  }
  EXPECT_TRUE(found_unreachable_reject) << "墙后的独立前沿块应被判为不可达";
}

// ---------------------------------------------------------------------------
// 采样与过滤
// ---------------------------------------------------------------------------
TEST(FrontierSampling, TooCloseGoalsAreFiltered)
{
  // === 对应「目标点距离机器人过近：过滤」===
  FrontierSearch s;
  FrontierSearchParams p = makeParams();
  p.min_goal_distance = 1.0;             // 1m 内的目标一律不要
  std::string err;
  ASSERT_TRUE(s.configure(p, err)) << err;

  GridMap m = makeMap(40, 40, kUnknown);
  fillRect(m, 0, 0, 19, 39, kFree);
  FrontierSearch::Result r;
  // 机器人就贴在前沿边上（x≈0.95）
  s.search(m, 0.95, 1.0, {}, r);

  for (const GoalCandidate & c : r.candidates) {
    if (c.valid) {
      EXPECT_GE(c.distance, 1.0) << "输出了过近的目标";
    }
  }
}

TEST(FrontierSampling, SampleCountScalesWithFrontierArea)
{
  // === 对应「大面积前沿多采样，小面积前沿少采样」===
  //
  // 注意构造方式：不能简单地"在未知场里放一块小的空闲矩形"来制造小前沿，
  // 因为空闲矩形的**整条周长**都是空闲-未知交界，前沿反而很长。
  // 正确做法是用占据墙把空闲区围起来，只留一个可控宽度的开口，
  // 开口宽度才真正决定前沿长度。
  //
  // 同时把 max_samples_per_cluster 放大到 50，避免两种情况都撞上限
  // （撞上限就看不出自适应效果了——这正是本测试第一版写错的地方）。
  FrontierSearch s;
  FrontierSearchParams p = makeParams();
  p.adaptive_sample_gain = 0.2;
  p.min_samples_per_cluster = 1;
  p.max_samples_per_cluster = 50;
  std::string err;
  ASSERT_TRUE(s.configure(p, err)) << err;

  // ---- 小前沿：3 格宽的开口 ----
  GridMap small = makeMap(60, 60, kUnknown);
  fillRect(small, 1, 1, 40, 58, kFree);        // 空闲走廊
  fillRect(small, 41, 1, 41, 58, kOccupied);   // 右侧封墙
  fillRect(small, 41, 29, 41, 31, kFree);      // 只开 3 格的口子
  FrontierSearch::Result rs;
  s.search(small, 0.5, 1.5, {}, rs);

  // ---- 大前沿：30 格宽的开口 ----
  GridMap big = makeMap(60, 60, kUnknown);
  fillRect(big, 1, 1, 40, 58, kFree);
  fillRect(big, 41, 1, 41, 58, kOccupied);
  fillRect(big, 41, 15, 41, 44, kFree);        // 开 30 格
  FrontierSearch::Result rb;
  s.search(big, 0.5, 1.5, {}, rb);

  // 先确认两者的前沿规模确实拉开了差距，否则本测试没有意义
  ASSERT_GT(rb.raw_frontier_cell_count, rs.raw_frontier_cell_count)
    << "构造失败：大开口的前沿格数应明显多于小开口";
  // 也确认没有撞到采样上限，否则比较会失真
  ASSERT_LT(rb.candidates.size(), 50U) << "撞到采样上限，测试失去意义";

  EXPECT_GT(rb.candidates.size(), rs.candidates.size())
    << "大面积前沿应该采出更多候选点 (小=" << rs.candidates.size()
    << " 前沿格=" << rs.raw_frontier_cell_count
    << " / 大=" << rb.candidates.size()
    << " 前沿格=" << rb.raw_frontier_cell_count << ")";
}

TEST(FrontierSampling, CandidatesDoNotAllCollapseToOnePoint)
{
  // 采样应沿前沿铺开，而不是全部扎堆在质心。
  FrontierSearch s;
  std::string err;
  ASSERT_TRUE(s.configure(makeParams(), err)) << err;

  GridMap m = makeMap(40, 60, kUnknown);
  fillRect(m, 0, 0, 19, 59, kFree);       // 一条很长的前沿
  FrontierSearch::Result r;
  s.search(m, 0.25, 1.5, {}, r);

  ASSERT_GE(r.candidates.size(), 2U);
  double min_y = 1e9;
  double max_y = -1e9;
  for (const GoalCandidate & c : r.candidates) {
    min_y = std::min(min_y, c.y);
    max_y = std::max(max_y, c.y);
  }
  EXPECT_GT(max_y - min_y, 0.3) << "候选点在前沿方向上应有明显跨度，不能扎堆";
}

TEST(FrontierSampling, ClearanceRequirementRejectsTightSpots)
{
  // 净空不足的候选点应被拒（机器人开不进去的缝隙）
  FrontierSearch s;
  FrontierSearchParams p = makeParams();
  p.required_clearance_radius = 0.30;    // 需要 30cm 净空
  p.obstacle_inflation_radius = 0.05;
  std::string err;
  ASSERT_TRUE(s.configure(p, err)) << err;

  // 一条只有 2 格(10cm)宽的窄缝通向未知区
  GridMap m = makeMap(40, 40, kUnknown);
  fillRect(m, 0, 0, 10, 39, kFree);
  fillRect(m, 11, 0, 13, 18, kOccupied);
  fillRect(m, 11, 19, 13, 20, kFree);      // 窄缝
  fillRect(m, 11, 21, 13, 39, kOccupied);

  FrontierSearch::Result r;
  s.search(m, 0.25, 1.0, {}, r);
  for (const GoalCandidate & c : r.candidates) {
    if (c.valid) {
      // 通过的必须真的有净空，不能是窄缝里的点
      EXPECT_LT(c.x, 0.60) << "窄缝内的点不该通过净空校验";
    }
  }
}

// ---------------------------------------------------------------------------
// 代价函数与历史惩罚
// ---------------------------------------------------------------------------
TEST(FrontierCost, VisitHistoryPushesGoalAway)
{
  // === 对应「历史访问惩罚」与「防止原地震荡」===
  FrontierSearch s;
  FrontierSearchParams p = makeParams();
  p.weight_distance = 1.0;
  p.weight_gain = 0.0;                 // 关掉增益项，单独观察惩罚效果
  p.weight_visit_penalty = 50.0;       // 惩罚给很大，效果明确
  p.visit_penalty_radius = 0.5;
  std::string err;
  ASSERT_TRUE(s.configure(p, err)) << err;

  GridMap m = makeMap(40, 60, kUnknown);
  fillRect(m, 0, 0, 19, 59, kFree);

  // 先跑一次，拿到无惩罚时的最优目标
  FrontierSearch::Result r1;
  s.search(m, 0.25, 1.5, {}, r1);
  ASSERT_GE(r1.best_candidate_index, 0);
  const GoalCandidate first = r1.candidates[static_cast<std::size_t>(r1.best_candidate_index)];

  // 把该点记入历史，再跑一次：最优目标应该换到别处
  std::vector<VisitRecord> history;
  VisitRecord rec;
  rec.x = first.x;
  rec.y = first.y;
  rec.count = 3U;
  history.push_back(rec);

  FrontierSearch::Result r2;
  s.search(m, 0.25, 1.5, history, r2);
  ASSERT_GE(r2.best_candidate_index, 0);
  const GoalCandidate second = r2.candidates[static_cast<std::size_t>(r2.best_candidate_index)];

  const double moved = std::hypot(second.x - first.x, second.y - first.y);
  EXPECT_GT(moved, 0.2) << "带上历史惩罚后应该换一个目标，而不是原地重复";
}

TEST(FrontierCost, HigherGainWinsWhenDistanceEqual)
{
  // 增益权重应真的起作用：两块前沿距离相当时，未知区更大的那块胜出。
  FrontierSearch s;
  FrontierSearchParams p = makeParams();
  p.weight_distance = 0.1;
  p.weight_gain = 100.0;
  p.weight_visit_penalty = 0.0;
  p.gain_window_radius = 0.6;
  std::string err;
  ASSERT_TRUE(s.configure(p, err)) << err;

  // 上方开一个大口(通向大片未知)，下方开一个小口(通向被夹住的小片未知)
  GridMap m = makeMap(60, 60, kUnknown);
  fillRect(m, 0, 0, 25, 59, kFree);
  // 下方小口两侧用空闲把未知挤小，从而降低该处的未知增益
  fillRect(m, 26, 0, 40, 8, kFree);
  fillRect(m, 26, 12, 40, 20, kFree);

  FrontierSearch::Result r;
  s.search(m, 0.5, 1.5, {}, r);
  ASSERT_GE(r.best_candidate_index, 0);
  const GoalCandidate best = r.candidates[static_cast<std::size_t>(r.best_candidate_index)];
  // 最优点的归一化增益应当接近最大值
  EXPECT_GT(best.gain_normalized, 0.5) << "应优先选增益大的前沿";
}

TEST(FrontierGeometry, GoalYawFacesFrontierCentroid)
{
  // 目标朝向应指向前沿块质心，让机器人到位后雷达正对未知区。
  FrontierSearch s;
  std::string err;
  ASSERT_TRUE(s.configure(makeParams(), err)) << err;

  GridMap m = makeMap(40, 40, kUnknown);
  fillRect(m, 0, 0, 19, 39, kFree);
  FrontierSearch::Result r;
  s.search(m, 0.25, 1.0, {}, r);
  ASSERT_GE(r.best_candidate_index, 0);

  const GoalCandidate best = r.candidates[static_cast<std::size_t>(r.best_candidate_index)];
  const FrontierCluster & cluster = r.clusters[best.cluster_index];
  const double expected = std::atan2(cluster.centroid_y - best.y, cluster.centroid_x - best.x);
  // 质心与目标点重合时会退化为「由机器人看向目标」，这里跨度足够不会退化
  EXPECT_NEAR(best.yaw, expected, 1e-6);
}

// ---------------------------------------------------------------------------
// 预处理
// ---------------------------------------------------------------------------
TEST(FrontierPreprocess, SmallObstacleSpecklesAreRemoved)
{
  // === 对应「消除噪声小斑块」===
  // 一个孤立的单格占据点应被当成噪声抹掉，不该在它周围留下膨胀阴影，
  // 从而不影响前沿判定。
  FrontierSearch s;
  FrontierSearchParams p = makeParams();
  p.min_obstacle_cluster_cells = 4;      // 少于 4 格视为噪声
  p.obstacle_inflation_radius = 0.10;
  std::string err;
  ASSERT_TRUE(s.configure(p, err)) << err;

  GridMap clean = makeMap(40, 40, kUnknown);
  fillRect(clean, 0, 0, 19, 39, kFree);
  FrontierSearch::Result r_clean;
  s.search(clean, 0.25, 1.0, {}, r_clean);

  GridMap speckled = clean;
  speckled.data[speckled.index(10, 20)] = kOccupied;   // 单格噪声
  FrontierSearch::Result r_speckled;
  s.search(speckled, 0.25, 1.0, {}, r_speckled);

  // 噪声被抹掉后，前沿格数应与干净地图一致
  EXPECT_EQ(r_speckled.raw_frontier_cell_count, r_clean.raw_frontier_cell_count)
    << "单格噪声斑块不应影响前沿提取";
}

TEST(FrontierPreprocess, InflationPreventsWallHuggingGoals)
{
  // 膨胀半径应把贴墙的前沿格排除掉，避免选出机器人挤不进去的目标。
  FrontierSearch s;
  FrontierSearchParams p = makeParams();
  p.obstacle_inflation_radius = 0.20;   // 4 格
  std::string err;
  ASSERT_TRUE(s.configure(p, err)) << err;

  GridMap m = makeMap(40, 40, kUnknown);
  fillRect(m, 0, 0, 30, 39, kFree);
  fillRect(m, 15, 0, 15, 39, kOccupied);   // 中间一道墙

  FrontierSearch::Result r;
  s.search(m, 0.25, 1.0, {}, r);
  // 所有有效候选都应离墙(x≈0.775m)至少一个膨胀半径
  for (const GoalCandidate & c : r.candidates) {
    if (c.valid) {
      EXPECT_GT(std::fabs(c.x - 0.775), 0.15) << "候选点贴墙太近";
    }
  }
}

}  // namespace
}  // namespace astribot_s1_autonomy
