// Copyright 2026 Astribot.
//
// escape_logic 的离线测试。
//
// 本文件的第一职责是**守住安全红线**：脱困逻辑只允许穿膨胀带，
// 绝不允许穿真实障碍。红线用「穷举哨兵」守，不是用单点用例守 ——
// 单点用例只能证明某一种输入是对的，穷举能证明**没有任何输入组合**
// 能让红线失效。

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "astribot_s1_autonomy/escape_logic.hpp"

namespace astribot_s1_autonomy
{
namespace
{

/// 造一张全空闲的三态图。分辨率 0.05 与本项目 costmap 一致。
GridMap makeFreeGrid(unsigned int w = 40U, unsigned int h = 40U, double res = 0.05)
{
  GridMap g;
  g.width = w;
  g.height = h;
  g.resolution = res;
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.data.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0);
  return g;
}

void setCell(GridMap & g, unsigned int mx, unsigned int my, int value)
{
  g.data[g.index(mx, my)] = static_cast<int8_t>(value);
}

/// 世界坐标处置值。
void setAt(GridMap & g, double wx, double wy, int value)
{
  unsigned int mx = 0U;
  unsigned int my = 0U;
  ASSERT_TRUE(g.worldToMap(wx, wy, mx, my));
  setCell(g, mx, my, value);
}

CellReading reading(int costmap_tri, int map_tri)
{
  CellReading r;
  r.costmap_valid = true;
  r.costmap_tri = costmap_tri;
  r.map_valid = true;
  r.map_tri = map_tri;
  return r;
}

}  // namespace

// =====================================================================
// 🔴 红线哨兵组：这几条一旦失败，脱困逻辑就变成了危险动作
// =====================================================================

TEST(RedLineSentinel, PhysicallyOccupiedNeverYieldsEscape)
{
  // 穷举：/map 判占据时，**任何** costmap 值 × 任何失败次数，
  // 都不允许返回 kEscape。
  EscapeTriggerConfig cfg;
  for (int cost = -1; cost <= 100; ++cost) {
    for (int fails = 0; fails <= 50; fails += 5) {
      std::string why;
      const auto v = evaluateEscapeTrigger(reading(cost, 100), fails, cfg, why);
      EXPECT_EQ(v, EscapeVerdict::kBlockedPhysically)
        << "cost=" << cost << " fails=" << fails << " why=" << why;
      EXPECT_NE(v, EscapeVerdict::kEscape) << "红线被突破！cost=" << cost;
      EXPECT_FALSE(why.empty()) << "红线命中必须给出可读原因";
    }
  }
}

TEST(RedLineSentinel, PhysicallyUnknownNeverYieldsEscape)
{
  // 未知 != 可通行。穷举 costmap 侧所有取值。
  EscapeTriggerConfig cfg;
  for (int cost = -1; cost <= 100; ++cost) {
    for (int fails = 0; fails <= 50; fails += 10) {
      std::string why;
      const auto v = evaluateEscapeTrigger(reading(cost, -1), fails, cfg, why);
      EXPECT_NE(v, EscapeVerdict::kEscape)
        << "未知格被当成可通行！cost=" << cost << " fails=" << fails;
    }
  }
}

TEST(RedLineSentinel, MissingDataNeverYieldsEscape)
{
  EscapeTriggerConfig cfg;
  const int fails = 999;
  for (int cost = -1; cost <= 100; ++cost) {
    CellReading no_cost = reading(cost, 0);
    no_cost.costmap_valid = false;
    CellReading no_map = reading(cost, 0);
    no_map.map_valid = false;
    std::string why;
    EXPECT_EQ(evaluateEscapeTrigger(no_cost, fails, cfg, why), EscapeVerdict::kDataInsufficient);
    EXPECT_EQ(evaluateEscapeTrigger(no_map, fails, cfg, why), EscapeVerdict::kDataInsufficient);
  }
}

TEST(RedLineSentinel, SegmentThroughOccupiedIsRejected)
{
  // 一道薄墙（单格宽）也必须被拦下 —— 这正是「目标合法、起点合法、
  // 中间是墙」那个失败模式。
  GridMap map = makeFreeGrid();
  EscapeSearchConfig cfg;
  const PlanarPoint a{0.10, 0.50};
  const PlanarPoint b{0.90, 0.50};
  ASSERT_TRUE(segmentPhysicallyClear(a, b, map, cfg));

  setAt(map, 0.50, 0.50, 100);      // 中间放一格墙
  EXPECT_FALSE(segmentPhysicallyClear(a, b, map, cfg))
    << "单格薄墙没被拦下 —— 直驱会撞上去";
}

TEST(RedLineSentinel, SegmentThroughUnknownIsRejected)
{
  GridMap map = makeFreeGrid();
  EscapeSearchConfig cfg;
  setAt(map, 0.50, 0.50, -1);
  EXPECT_FALSE(segmentPhysicallyClear({0.10, 0.50}, {0.90, 0.50}, map, cfg));
}

TEST(RedLineSentinel, SegmentLeavingMapIsRejected)
{
  // 出图按不可通行处理。「无数据当成安全」是本项目已踩过的一类错误。
  GridMap map = makeFreeGrid(20U, 20U, 0.05);      // 只有 1m x 1m
  EscapeSearchConfig cfg;
  EXPECT_FALSE(segmentPhysicallyClear({0.50, 0.50}, {5.00, 0.50}, map, cfg));
}

TEST(RedLineSentinel, EmptyMapIsNotTraversable)
{
  GridMap empty;
  EscapeSearchConfig cfg;
  EXPECT_FALSE(segmentPhysicallyClear({0.0, 0.0}, {0.1, 0.0}, empty, cfg));
}

// =====================================================================
// 触发判定
// =====================================================================

TEST(Trigger, InflationLethalButPhysicallyFreeYieldsEscape)
{
  EscapeTriggerConfig cfg;
  cfg.trigger_failures = 5;
  std::string why;
  // 这就是实测那 25652 次失败的场景：costmap 致命(>=253→100)、/map 空闲。
  EXPECT_EQ(evaluateEscapeTrigger(reading(100, 0), 5, cfg, why), EscapeVerdict::kEscape);
  EXPECT_NE(why.find("膨胀带"), std::string::npos) << "原因里应点明是膨胀导致";
}

TEST(Trigger, PlannerNonLethalMeansGoalSideProblem)
{
  // 起点合法 ⇒ 失败原因在目标侧，应该换候选点而不是脱困。
  EscapeTriggerConfig cfg;
  std::string why;
  EXPECT_EQ(evaluateEscapeTrigger(reading(0, 0), 999, cfg, why), EscapeVerdict::kNone);
  EXPECT_NE(why.find("目标侧"), std::string::npos);
}

TEST(Trigger, BelowFailureThresholdDoesNotFire)
{
  EscapeTriggerConfig cfg;
  cfg.trigger_failures = 5;
  std::string why;
  for (int f = 0; f < 5; ++f) {
    EXPECT_EQ(evaluateEscapeTrigger(reading(100, 0), f, cfg, why), EscapeVerdict::kNone)
      << "failures=" << f;
  }
  EXPECT_EQ(evaluateEscapeTrigger(reading(100, 0), 5, cfg, why), EscapeVerdict::kEscape);
}

TEST(Trigger, VerdictAlwaysHasReason)
{
  // 禁止静默：任何结论都必须给出可读原因。
  EscapeTriggerConfig cfg;
  for (int cost = -1; cost <= 100; ++cost) {
    for (int m = -1; m <= 100; ++m) {
      std::string why;
      (void)evaluateEscapeTrigger(reading(cost, m), 7, cfg, why);
      EXPECT_FALSE(why.empty()) << "cost=" << cost << " map=" << m;
    }
  }
}

TEST(Trigger, ToStringCoversAllVerdicts)
{
  EXPECT_STREQ(toString(EscapeVerdict::kNone), "NONE");
  EXPECT_STREQ(toString(EscapeVerdict::kEscape), "ESCAPE");
  EXPECT_STREQ(toString(EscapeVerdict::kBlockedPhysically), "BLOCKED_PHYSICALLY");
  EXPECT_STREQ(toString(EscapeVerdict::kDataInsufficient), "DATA_INSUFFICIENT");
}

// =====================================================================
// 一级方案：来路脱困
// =====================================================================

TEST(Breadcrumb, EmptyTrailFallsThrough)
{
  GridMap cost = makeFreeGrid();
  GridMap map = makeFreeGrid();
  EscapeSearchConfig cfg;
  PlanarPoint t;
  std::string why;
  EXPECT_FALSE(pickBreadcrumbTarget({}, {0.5, 0.5}, cost, map, cfg, t, why));
  EXPECT_NE(why.find("为空"), std::string::npos);
}

TEST(Breadcrumb, PicksNewestUsablePoint)
{
  GridMap cost = makeFreeGrid();
  GridMap map = makeFreeGrid();
  EscapeSearchConfig cfg;
  const PlanarPoint robot{1.00, 0.50};
  // 机器人所在与近处都在膨胀带里；只有 0.60 处出带。
  setAt(cost, 1.00, 0.50, 100);
  setAt(cost, 0.90, 0.50, 100);
  setAt(cost, 0.80, 0.50, 100);

  std::vector<Breadcrumb> trail{
    {{0.60, 0.50}, 1.0},      // 更旧、可用
    {{0.80, 0.50}, 2.0},      // 更新、但仍在带里
    {{0.90, 0.50}, 3.0},      // 最新、仍在带里
  };
  PlanarPoint t;
  std::string why;
  ASSERT_TRUE(pickBreadcrumbTarget(trail, robot, cost, map, cfg, t, why));
  EXPECT_NEAR(t.x, 0.60, 1e-9);
  EXPECT_NEAR(t.y, 0.50, 1e-9);
}

TEST(Breadcrumb, RejectsPointBehindAWall)
{
  // 来路点本身合法，但直线被墙挡住 —— 必须拒绝（直驱没有规划器）。
  GridMap cost = makeFreeGrid();
  GridMap map = makeFreeGrid();
  EscapeSearchConfig cfg;
  const PlanarPoint robot{1.00, 0.50};
  setAt(cost, 1.00, 0.50, 100);
  setAt(map, 0.75, 0.50, 100);      // 中间一道墙

  std::vector<Breadcrumb> trail{{{0.50, 0.50}, 1.0}};
  PlanarPoint t;
  std::string why;
  EXPECT_FALSE(pickBreadcrumbTarget(trail, robot, cost, map, cfg, t, why));
  EXPECT_NE(why.find("直线被挡"), std::string::npos) << why;
}

TEST(Breadcrumb, RejectsPointBeyondSearchRadius)
{
  GridMap cost = makeFreeGrid(200U, 200U, 0.05);
  GridMap map = makeFreeGrid(200U, 200U, 0.05);
  EscapeSearchConfig cfg;
  cfg.search_radius_m = 0.30;
  const PlanarPoint robot{5.00, 5.00};
  setAt(cost, 5.00, 5.00, 100);

  std::vector<Breadcrumb> trail{{{2.00, 5.00}, 1.0}};      // 3m 远，超出 0.30
  PlanarPoint t;
  std::string why;
  EXPECT_FALSE(pickBreadcrumbTarget(trail, robot, cost, map, cfg, t, why));
}

// =====================================================================
// 二级方案：最近可规划格搜索
// =====================================================================

TEST(NearestPlannable, FindsCellOutsideInflationBand)
{
  GridMap cost = makeFreeGrid();
  GridMap map = makeFreeGrid();
  EscapeSearchConfig cfg;
  const PlanarPoint robot{0.50, 0.50};
  // 机器人周围 3 格半径全是膨胀带，之外自由。
  for (int dx = -3; dx <= 3; ++dx) {
    for (int dy = -3; dy <= 3; ++dy) {
      setAt(cost, 0.50 + (dx * 0.05), 0.50 + (dy * 0.05), 100);
    }
  }
  PlanarPoint t;
  bool relaxed = true;
  std::string why;
  ASSERT_TRUE(findNearestPlannableCell(robot, 0.0, cost, map, cfg, t, relaxed, why));
  // 目标必须真的出带
  int tri = 0;
  unsigned int mx = 0U;
  unsigned int my = 0U;
  ASSERT_TRUE(cost.worldToMap(t.x, t.y, mx, my));
  tri = static_cast<int>(cost.data[cost.index(mx, my)]);
  EXPECT_FALSE(isPlannerLethal(tri, cfg.thresholds)) << "选出的目标仍在膨胀带里";
}

TEST(NearestPlannable, HonorsHeadingConstraintFirst)
{
  GridMap cost = makeFreeGrid(80U, 80U, 0.05);
  GridMap map = makeFreeGrid(80U, 80U, 0.05);
  EscapeSearchConfig cfg;
  cfg.heading_tol_rad = 0.5;      // 收紧到 ±28.6°
  const PlanarPoint robot{2.00, 2.00};
  // 机器人周围一圈膨胀带
  for (int dx = -2; dx <= 2; ++dx) {
    for (int dy = -2; dy <= 2; ++dy) {
      setAt(cost, 2.00 + (dx * 0.05), 2.00 + (dy * 0.05), 100);
    }
  }
  PlanarPoint t;
  bool relaxed = true;
  std::string why;
  // 参考朝向 +x：选出的目标应该在 +x 一侧
  ASSERT_TRUE(findNearestPlannableCell(robot, 0.0, cost, map, cfg, t, relaxed, why));
  EXPECT_FALSE(relaxed) << "本该在趟1(方向约束内)就找到：" << why;
  const double bearing = std::atan2(t.y - robot.y, t.x - robot.x);
  EXPECT_LE(std::fabs(bearing), cfg.heading_tol_rad + 1e-6)
    << "选出的目标方位 " << bearing << " 超出方向约束";
}

TEST(NearestPlannable, RelaxesConstraintOnlyWhenNoSolution)
{
  GridMap cost = makeFreeGrid(80U, 80U, 0.05);
  GridMap map = makeFreeGrid(80U, 80U, 0.05);
  EscapeSearchConfig cfg;
  cfg.heading_tol_rad = 0.3;
  const PlanarPoint robot{2.00, 2.00};
  // +x 整个半平面都设成膨胀带，逼迫只能往 -x 逃
  for (unsigned int mx = 0U; mx < cost.width; ++mx) {
    for (unsigned int my = 0U; my < cost.height; ++my) {
      if (cost.worldX(mx) >= robot.x - 0.01) {
        setCell(cost, mx, my, 100);
      }
    }
  }
  PlanarPoint t;
  bool relaxed = false;
  std::string why;
  ASSERT_TRUE(findNearestPlannableCell(robot, 0.0, cost, map, cfg, t, relaxed, why));
  EXPECT_TRUE(relaxed) << "趟1 应无解、趟2 才有解：" << why;
  EXPECT_NE(why.find("放开方向约束"), std::string::npos) << "放开约束必须在原因里可见";
  EXPECT_LT(t.x, robot.x) << "只能往 -x 逃";
}

TEST(NearestPlannable, NoSolutionWhenFullySurrounded)
{
  GridMap cost = makeFreeGrid(40U, 40U, 0.05);
  GridMap map = makeFreeGrid(40U, 40U, 0.05);
  EscapeSearchConfig cfg;
  cfg.search_radius_m = 0.40;
  // 整张图都是膨胀带
  for (auto & v : cost.data) {
    v = 100;
  }
  PlanarPoint t;
  bool relaxed = false;
  std::string why;
  EXPECT_FALSE(findNearestPlannableCell({1.00, 1.00}, 0.0, cost, map, cfg, t, relaxed, why));
  EXPECT_NE(why.find("无解"), std::string::npos) << why;
}

// 无解时**必须说出是哪一条否决的**，否则线上只能靠读代码猜。
//
// 这条测试挡住的具体历史错误：2026-09-02 在线验证里本函数连续 4 次返回
// 「半径 1.5m 内没有任何可用格子」，5 个否决条件全部沉默地 continue，
// 完全无法定位。我据此做的第一版推断还是错的（怀疑 require_target_map_free，
// 被另一条日志的「物理不可站=0」直接否证）。
// 判据取「诊断串里必须同时含扫描总数与全部 6 类计数的字样」，
// 而不是只检查非空 —— 少一类就会在那一类上永远瞎。
TEST(NearestPlannable, FailureReasonIsItemized)
{
  GridMap cost = makeFreeGrid(40U, 40U, 0.05);
  GridMap map = makeFreeGrid(40U, 40U, 0.05);
  EscapeSearchConfig cfg;
  cfg.search_radius_m = 0.40;
  for (auto & v : cost.data) {
    v = 100;                       // 整张图都是膨胀带 ⇒ 必然无解
  }
  PlanarPoint t;
  bool relaxed = false;
  std::string why;
  ASSERT_FALSE(findNearestPlannableCell({1.00, 1.00}, 0.0, cost, map, cfg, t, relaxed, why));

  for (const char * key : {"扫", "超距", "方向约束", "膨胀带/未知",
      "物理占据/未知", "非明确空闲", "直线被挡"})
  {
    EXPECT_NE(why.find(key), std::string::npos)
      << "诊断串缺少「" << key << "」这一类计数，该类否决将无法定位。实际: " << why;
  }
  // 本场景是「全图膨胀带」，所以膨胀带那一类必须真的记到了非零。
  EXPECT_EQ(why.find("膨胀带/未知 0、"), std::string::npos)
    << "全图膨胀带却记成 0，计数没接上。实际: " << why;
}

TEST(NearestPlannable, RobotOutsideCostmapIsRejected)
{
  GridMap cost = makeFreeGrid(20U, 20U, 0.05);
  GridMap map = makeFreeGrid(20U, 20U, 0.05);
  EscapeSearchConfig cfg;
  PlanarPoint t;
  bool relaxed = false;
  std::string why;
  EXPECT_FALSE(findNearestPlannableCell({99.0, 99.0}, 0.0, cost, map, cfg, t, relaxed, why));
}

TEST(NearestPlannable, NeverSelectsPhysicallyOccupiedTarget)
{
  // 🔴 红线：即使 costmap 说某格非致命，只要 /map 说它占据，就不能选。
  // 构造：costmap 全自由（除机器人格），/map 全占据（除机器人格）。
  GridMap cost = makeFreeGrid(40U, 40U, 0.05);
  GridMap map = makeFreeGrid(40U, 40U, 0.05);
  EscapeSearchConfig cfg;
  const PlanarPoint robot{1.00, 1.00};
  setAt(cost, 1.00, 1.00, 100);
  for (auto & v : map.data) {
    v = 100;
  }
  setAt(map, 1.00, 1.00, 0);      // 机器人自己那格物理自由
  PlanarPoint t;
  bool relaxed = false;
  std::string why;
  EXPECT_FALSE(findNearestPlannableCell(robot, 0.0, cost, map, cfg, t, relaxed, why))
    << "选了一个 /map 判占据的目标！";
}

// =====================================================================
// 速度指令
// =====================================================================

TEST(Velocity, ObeysLinearLimit)
{
  EscapeLimits lim;
  const auto cmd = escapeVelocity({0.0, 0.0}, 0.0, {10.0, 0.0}, lim);
  EXPECT_LE(std::hypot(cmd.vx, cmd.vy), lim.max_linear + 1e-9);
  EXPECT_FALSE(cmd.arrived);
}

TEST(Velocity, ObeysAngularLimit)
{
  EscapeLimits lim;
  // 目标在正后方 → 航向误差 pi，wz 必须被限幅
  const auto cmd = escapeVelocity({0.0, 0.0}, 0.0, {-1.0, 0.0}, lim);
  EXPECT_LE(std::fabs(cmd.wz), lim.max_angular + 1e-9);
}

TEST(Velocity, HolonomicSidewaysMotionWithoutTurningFirst)
{
  // 全向底盘：目标在正左方时应该直接侧移，vy 拿到全部速度。
  EscapeLimits lim;
  const auto cmd = escapeVelocity({0.0, 0.0}, 0.0, {0.0, 1.0}, lim);
  EXPECT_NEAR(cmd.vx, 0.0, 1e-6);
  EXPECT_NEAR(cmd.vy, lim.max_linear, 1e-6);
}

TEST(Velocity, BodyFrameRotationIsApplied)
{
  // 车头朝 +y（yaw=pi/2），目标在世界 +x → 车体系里是右侧（vy 为负）。
  EscapeLimits lim;
  const auto cmd = escapeVelocity({0.0, 0.0}, M_PI / 2.0, {1.0, 0.0}, lim);
  EXPECT_NEAR(cmd.vx, 0.0, 1e-6);
  EXPECT_NEAR(cmd.vy, -lim.max_linear, 1e-6);
}

TEST(Velocity, ZeroWhenArrived)
{
  EscapeLimits lim;
  const auto cmd = escapeVelocity({0.0, 0.0}, 0.0, {0.01, 0.0}, lim);
  EXPECT_TRUE(cmd.arrived);
  EXPECT_NEAR(cmd.vx, 0.0, 1e-12);
  EXPECT_NEAR(cmd.vy, 0.0, 1e-12);
  EXPECT_NEAR(cmd.wz, 0.0, 1e-12);
}

TEST(Velocity, NoAngularJitterWithinTolerance)
{
  EscapeLimits lim;
  // 航向误差小于 align_tol_rad → wz 必须为 0，否则在带里原地抖
  const auto cmd = escapeVelocity({0.0, 0.0}, 0.0, {1.0, 0.05}, lim);
  EXPECT_NEAR(cmd.wz, 0.0, 1e-12);
}

// =====================================================================
// 出带判据
// =====================================================================

TEST(Cleared, RequiresConsecutiveTicks)
{
  EXPECT_FALSE(escapeCleared(0, 5));
  EXPECT_FALSE(escapeCleared(4, 5));
  EXPECT_TRUE(escapeCleared(5, 5));
  EXPECT_TRUE(escapeCleared(6, 5));
}

TEST(Cleared, NonPositiveNeedNeverClears)
{
  // need<=0 是配置错误。按「永不判出带」处理 ——
  // 若按「立刻出带」处理，一个配错的 0 会让出带判据彻底失效。
  EXPECT_FALSE(escapeCleared(0, 0));
  EXPECT_FALSE(escapeCleared(999, 0));
  EXPECT_FALSE(escapeCleared(999, -1));
}

// =====================================================================
// 🔴 红线哨兵组之二：足迹扫掠
//
// 最初的实现只查中心线，等于把机器人当质点。中心线离墙 0.05m 时
// 中心线全程「空闲」而轮子已经压在墙上。下面几条守住这个缺口。
// =====================================================================

TEST(RedLineSentinel, FootprintSweepRejectsWallBesideCenterLine)
{
  GridMap map = makeFreeGrid(80U, 80U, 0.05);      // 4m x 4m
  EscapeSearchConfig cfg;
  const PlanarPoint a{0.20, 1.00};
  const PlanarPoint b{1.00, 1.00};
  ASSERT_TRUE(segmentPhysicallyClear(a, b, map, cfg));

  // 墙**不在**中心线上，横向偏 0.20m < 足迹半径 0.386m。
  setAt(map, 0.60, 1.20, 100);
  EXPECT_FALSE(segmentPhysicallyClear(a, b, map, cfg))
    << "中心线旁 0.20m 的墙没被拦下 —— 中心线干净但轮子会撞上去";
}

TEST(RedLineSentinel, FootprintSweepExhaustiveOverLateralOffset)
{
  // 穷举横向偏移。留出边界附近的一段不判 —— 那一段的结论取决于
  // 「格中心距离 vs 格边距离」这类栅格离散细节，不是本层的语义。
  EscapeSearchConfig cfg;
  const double radius = cfg.footprint_radius_m;
  ASSERT_GT(radius, 0.0);

  for (int i = 0; i <= 70; ++i) {
    const double offset = 0.01 * static_cast<double>(i);
    GridMap map = makeFreeGrid(120U, 120U, 0.05);      // 6m x 6m
    const PlanarPoint a{0.50, 2.00};
    const PlanarPoint b{1.50, 2.00};
    setAt(map, 1.00, 2.00 + offset, 100);

    const bool clear = segmentPhysicallyClear(a, b, map, cfg);
    if (offset <= radius - 0.05) {
      EXPECT_FALSE(clear) << "横向偏移 " << offset << "m（< 足迹半径 " << radius
                          << "m）的墙必须拦下";
    } else if (offset >= radius + 0.10) {
      EXPECT_TRUE(clear) << "横向偏移 " << offset << "m 已在足迹之外，"
                         << "拦下它会让脱困在恰好需要它的窄处无解";
    }
  }
}

TEST(RedLineSentinel, FootprintAnnulusUnknownDoesNotBlock)
{
  // 这是一条**判断**，不是疏漏：环带里的未知不拦。
  // 探索期地图边缘天然被未知包围，若环带的未知也拦，脱困会恰好在
  // 最需要它的时候（贴着未知边界）永远无解。用户定的红线原话是
  // 「不能穿过 obstacle 层**占据**栅格」，未知不在其中。
  GridMap map = makeFreeGrid(80U, 80U, 0.05);
  EscapeSearchConfig cfg;
  setAt(map, 0.60, 1.20, -1);      // 中心线旁的未知
  EXPECT_TRUE(segmentPhysicallyClear({0.20, 1.00}, {1.00, 1.00}, map, cfg));
}

TEST(RedLineSentinel, CenterLineUnknownAlwaysBlocks)
{
  // 与上一条成对：中心线上的未知**必须**拦。
  // 中心线上是未知 = 要把车开进从没看见过的地方。
  GridMap map = makeFreeGrid(80U, 80U, 0.05);
  EscapeSearchConfig cfg;
  setAt(map, 0.60, 1.00, -1);      // 正好在中心线上
  EXPECT_FALSE(segmentPhysicallyClear({0.20, 1.00}, {1.00, 1.00}, map, cfg));
}

TEST(RedLineSentinel, FootprintCellCapFailsClosed)
{
  // 超出防御上限时必须判**不可通行**。判成可通行等于「算不过来就放行」，
  // 那是最危险的一种失败方向。
  GridMap map = makeFreeGrid(80U, 80U, 0.05);
  EscapeSearchConfig cfg;
  cfg.max_footprint_cells = 4U;      // 故意设到不可能满足
  EXPECT_FALSE(segmentPhysicallyClear({0.20, 1.00}, {1.00, 1.00}, map, cfg));
}

TEST(FootprintSweep, ZeroRadiusRestoresCenterLineOnlyBehaviour)
{
  // 回归开关：仓库规范是不删旧逻辑。radius=0 必须还原成只查中心线，
  // 否则「新旧对比」这件事根本做不了。
  GridMap map = makeFreeGrid(80U, 80U, 0.05);
  EscapeSearchConfig cfg;
  cfg.footprint_radius_m = 0.0;
  setAt(map, 0.60, 1.20, 100);      // 旁边的墙，旧行为查不到
  EXPECT_TRUE(segmentPhysicallyClear({0.20, 1.00}, {1.00, 1.00}, map, cfg));
}

TEST(FootprintSweep, WallBeyondSegmentEndsDoesNotBlock)
{
  // 夹到 [0,1] 的意义：墙在线段**延长线**上、且离两端都超过足迹半径时，
  // 不该被算成挡路。不夹的话（按直线算距离）它会被误拦。
  GridMap map = makeFreeGrid(120U, 120U, 0.05);
  EscapeSearchConfig cfg;
  setAt(map, 2.00, 1.00, 100);      // 线段在 x∈[0.20,1.00]，墙在 x=2.00
  EXPECT_TRUE(segmentPhysicallyClear({0.20, 1.00}, {1.00, 1.00}, map, cfg));
}

TEST(FootprintSweep, RealMapSizedSweepStaysCheap)
{
  // 复杂度守卫。本项目已有一次「40 个小样本用例全绿、真实规模一跑就卡死」
  // 的记录（O(位姿 x 占据格)）。足迹扫掠若写成「沿线段每点扫一个圆盘」，
  // 同一格会被重复查上百次，量级从每段 ~1300 格涨到 ~95000 格。
  //
  // 这里按真实规模压一遍：仓库地图 289x420 @0.05，脱困搜索半径内
  // 候选格约 2800 个，每个候选都要查一条线段。
  GridMap map = makeFreeGrid(289U, 420U, 0.05);
  EscapeSearchConfig cfg;
  // 撒一些墙，避免「全空闲」让 isPhysicallyOccupied 的短路把开销测低了。
  // 间距必须 >> 2 x 足迹半径(0.772m)，否则半径内处处有墙，每条线段都在
  // 头几格就早退 —— 那样测出来的是早退开销，不是扫掠开销。这里取 3m。
  for (unsigned int my = 0U; my < map.height; my += 60U) {
    for (unsigned int mx = 0U; mx < map.width; mx += 60U) {
      setCell(map, mx, my, 100);
    }
  }

  const PlanarPoint robot{7.00, 10.00};
  const auto t0 = std::chrono::steady_clock::now();
  std::size_t clear_count = 0U;
  const int kCandidates = 2800;
  for (int i = 0; i < kCandidates; ++i) {
    // 半径 1.5m 内绕一圈的候选点，长度覆盖到上限。
    const double ang = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(kCandidates);
    const PlanarPoint cand{robot.x + (1.5 * std::cos(ang)), robot.y + (1.5 * std::sin(ang))};
    if (segmentPhysicallyClear(robot, cand, map, cfg)) {
      ++clear_count;
    }
  }
  const double elapsed_sec =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  // 判据是「一个 2Hz 的 tick 装得下」，留足余量。
  EXPECT_LT(elapsed_sec, 2.0)
    << "真实规模足迹扫掠耗时 " << elapsed_sec << "s，一个 2Hz tick 装不下；"
    << "检查是否退化成了「沿线段逐点扫圆盘」";
  // 顺带确认这轮确实在做事（不是被某个早退分支全部短路掉了）。
  EXPECT_GT(clear_count, 0U) << "2800 个候选一个都不通 —— 测试夹具本身有问题";
  EXPECT_LT(clear_count, static_cast<std::size_t>(kCandidates))
    << "2800 个候选全通 —— 撒的墙没起作用，这轮没真正压到扫掠";
}

}  // namespace astribot_s1_autonomy

