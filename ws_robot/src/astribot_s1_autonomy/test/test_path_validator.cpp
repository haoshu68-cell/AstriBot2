// Copyright 2026 Astribot.
//
// 未知区域禁行校验的单元测试。这些规则一旦漏检，现场表现是机器人一头开进
// 没扫过的区域，事后极难复盘，所以每条都必须能离线钉死。
//
// 重点是 SegmentCrossingUnknownIsRejected：路径顶点全合法、但顶点之间的
// 插值路段穿过未知格。只查顶点的实现会完全放过这种情况。
#include <cmath>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "astribot_s1_autonomy/path_validator.hpp"

namespace astribot_s1_autonomy
{
namespace
{

constexpr int8_t kUnknown = -1;
constexpr int8_t kFree = 0;
constexpr int8_t kOccupied = 100;

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

void fillRect(
  GridMap & m, unsigned int x0, unsigned int y0,
  unsigned int x1, unsigned int y1, int8_t v)
{
  for (unsigned int y = y0; y <= y1 && y < m.height; ++y) {
    for (unsigned int x = x0; x <= x1 && x < m.width; ++x) {
      m.data[m.index(x, y)] = v;
    }
  }
}

/// 默认参数：净空半径故意调小(0.10)，让「净空」和「路径」两条规则能分开测。
PathValidator makeValidator(double clearance = 0.10, double step = 0.0)
{
  PathValidatorParams p;
  p.goal_clearance_radius = clearance;
  // 未知净空默认给一格，保持既有用例语义不变（它们测的是占据/路径规则）。
  p.goal_unknown_clearance_radius = 0.05;
  p.path_sample_step = step;
  p.path_endpoint_tolerance = 0.5;
  PathValidator v;
  std::string err;
  EXPECT_TRUE(v.configure(p, err)) << err;
  return v;
}

std::vector<PlanarPoint> makePath(const std::vector<std::pair<double, double>> & pts)
{
  std::vector<PlanarPoint> out;
  out.reserve(pts.size());
  for (const auto & p : pts) {
    out.push_back(PlanarPoint{p.first, p.second});
  }
  return out;
}

// ------------------------- isKnownFree -------------------------

TEST(PathValidator, UnknownAndOccupiedAndOutOfBoundsAreNotFree) {
  GridMap m = makeMap(40, 40, kFree);
  m.data[m.index(10, 10)] = kUnknown;
  m.data[m.index(11, 10)] = kOccupied;
  const PathValidator v = makeValidator();

  EXPECT_TRUE(v.isKnownFree(m, m.worldX(5), m.worldY(5)));
  EXPECT_FALSE(v.isKnownFree(m, m.worldX(10), m.worldY(10)));
  EXPECT_FALSE(v.isKnownFree(m, m.worldX(11), m.worldY(10)));
  // 越界必须是 false，而不是读到越界内存后「碰巧」返回 true。
  EXPECT_FALSE(v.isKnownFree(m, -5.0, -5.0));
  EXPECT_FALSE(v.isKnownFree(m, 100.0, 100.0));
}

TEST(PathValidator, UnconfiguredValidatorRejectsEverything) {
  // 参数没加载成功时必须一律拒绝，而不是用默认值蒙着放行。
  const GridMap m = makeMap(20, 20, kFree);
  const PathValidator v;
  EXPECT_FALSE(v.isKnownFree(m, m.worldX(5), m.worldY(5)));
  EXPECT_FALSE(v.validateGoal(m, m.worldX(5), m.worldY(5)).valid);
}

// ------------------------- 校验1：目标点 -------------------------

TEST(PathValidator, GoalOnUnknownCellIsRejected) {
  GridMap m = makeMap(40, 40, kFree);
  m.data[m.index(20, 20)] = kUnknown;
  const PathValidator v = makeValidator();
  const ValidationResult r = v.validateGoal(m, m.worldX(20), m.worldY(20));
  EXPECT_FALSE(r.valid);
  EXPECT_FALSE(r.reason.empty());
}

TEST(PathValidator, GoalAdjacentToUnknownIsRejectedByClearance) {
  // 目标格自身空闲，但紧邻未知区。这里测的是【未知】净空半径这一条。
  GridMap m = makeMap(60, 60, kFree);
  fillRect(m, 24, 0, 24, 59, kUnknown);          // 一整列未知
  PathValidatorParams p;
  p.goal_clearance_radius = 0.42;                // 占据净空按机器人外接半径
  p.goal_unknown_clearance_radius = 0.20;        // 未知净空 0.20m = 4 格
  PathValidator v;
  std::string err;
  ASSERT_TRUE(v.configure(p, err)) << err;

  // 距未知列 1 格(0.05m) —— 未知净空不足，必须拒。
  EXPECT_FALSE(v.validateGoal(m, m.worldX(23), m.worldY(30)).valid);
  // 距未知列 6 格(0.30m > 0.20m) —— 通过。
  EXPECT_TRUE(v.validateGoal(m, m.worldX(18), m.worldY(30)).valid);
}

TEST(PathValidator, FrontierGoalSurvivesLargeOccupiedClearance) {
  // ============ 回归测试：自我否决的探索目标 ============
  // 这是仿真里真实发生过的 bug：占据净空和未知净空原本是同一个 0.42m 参数。
  // 前沿点的定义就是「空闲格且邻域含未知格」，所以「0.42m 内无未知格」这条要求
  // 会让**每一个**前沿点都不合法 —— 探索在构造上无法启动，
  // 现场表现是「有 249 个前沿格，却一个候选都留不下，永远停在 PAUSED」。
  //
  // 正确行为：紧贴未知区的目标点必须能通过，只要它周围没有障碍物。
  GridMap m = makeMap(80, 80, kFree);
  fillRect(m, 40, 0, 79, 79, kUnknown);          // 右半边全未知，x=40 是前沿

  PathValidatorParams p;
  p.goal_clearance_radius = 0.42;                // 大占据净空（真实机器人尺寸）
  p.goal_unknown_clearance_radius = 0.10;        // 小未知净空（2 格）
  PathValidator v;
  std::string err;
  ASSERT_TRUE(v.configure(p, err)) << err;

  // 距未知区 3 格(0.15m > 0.10m)、周围无障碍：这是典型的前沿目标，必须通过。
  const ValidationResult ok = v.validateGoal(m, m.worldX(36), m.worldY(40));
  EXPECT_TRUE(ok.valid) << "前沿目标被自己的未知净空规则否决了: " << ok.reason;

  // 但真正的障碍物仍然要按 0.42m 拦住。
  GridMap m2 = makeMap(80, 80, kFree);
  fillRect(m2, 40, 0, 79, 79, kUnknown);
  fillRect(m2, 30, 38, 31, 42, kOccupied);       // 距 x=36 约 5~6 格(0.25~0.30m)
  EXPECT_FALSE(v.validateGoal(m2, m2.worldX(36), m2.worldY(40)).valid);
}

TEST(PathValidator, RealFrontierCellItselfMustPassValidateGoal) {
  // ============ 回归测试：上面那条用例的漏网之鱼 ============
  // 上面 FrontierGoalSurvivesLargeOccupiedClearance 取的目标点距未知区 **3 格**，
  // 而**真正的前沿格**按定义距未知区只有 **1 格**（自身空闲 + 8 邻域含未知）。
  // 于是那条用例在 goal_unknown_clearance_radius=0.10 下能过，
  // 真实地图上每一个前沿候选却全被否 —— 绿的测试套件放过了这个 bug。
  //
  // 实测证据（仿真活地图 277x414）：合法前沿候选 6784 个，通过 0 个，
  // 6775 个死在「未知净空半径(0.1m)内有未知格」；本参数设 0 后全部通过。
  //
  // 本用例直接把不变量写死：**紧邻未知区一格的前沿格本身必须能当目标点**，
  // 否则探索在构造上不可能启动。任何 >= 1 格的未知净空半径都违反它。
  GridMap m = makeMap(80, 80, kFree);
  fillRect(m, 40, 0, 79, 79, kUnknown);          // x>=40 未知，x=39 是真前沿格

  PathValidatorParams p;
  p.goal_clearance_radius = 0.42;
  p.goal_unknown_clearance_radius = 0.0;         // 唯一能容纳真前沿格的取值
  PathValidator v;
  std::string err;
  ASSERT_TRUE(v.configure(p, err)) << err;

  // x=39 正交紧邻未知格(d=1 格)，x=38 对角紧邻(d_sq=2)。两者都是真前沿格。
  const ValidationResult orth = v.validateGoal(m, m.worldX(39), m.worldY(40));
  EXPECT_TRUE(orth.valid) << "正交紧邻未知区的前沿格被否决: " << orth.reason;
  const ValidationResult diag = v.validateGoal(m, m.worldX(38), m.worldY(40));
  EXPECT_TRUE(diag.valid) << "次邻未知区的前沿格被否决: " << diag.reason;

  // 反过来钉住「未知区禁行」没被削弱：目标格自己是未知，仍然必须拒。
  EXPECT_FALSE(v.validateGoal(m, m.worldX(45), m.worldY(40)).valid);

  // 也钉住半径一旦 >= 1 格就会重现这个 bug —— 这是本 bug 的机制本身。
  PathValidatorParams bad = p;
  bad.goal_unknown_clearance_radius = 0.05;      // 1 格
  PathValidator vb;
  ASSERT_TRUE(vb.configure(bad, err)) << err;
  EXPECT_FALSE(vb.validateGoal(m, m.worldX(39), m.worldY(40)).valid)
    << "未知净空半径 1 格竟然放过了紧邻未知区的前沿格，说明净空判据的格算术变了，"
       "请重新核对 d_sq <= (r/resolution)^2 的边界";
}

TEST(PathValidator, UnknownClearanceLargerThanOccupiedIsRejectedAtConfigure) {
  // 两个半径写反会让所有前沿点不合法。宁可拒绝配置，也不要静默变成
  // 一个「永远找不到目标」的节点 —— 那种故障现场极难定位。
  PathValidatorParams p;
  p.goal_clearance_radius = 0.10;
  p.goal_unknown_clearance_radius = 0.42;
  PathValidator v;
  std::string err;
  EXPECT_FALSE(v.configure(p, err));
  EXPECT_FALSE(err.empty());
}

TEST(PathValidator, GoalNearOccupiedIsRejectedByClearance) {
  GridMap m = makeMap(60, 60, kFree);
  fillRect(m, 30, 28, 32, 32, kOccupied);
  const PathValidator v = makeValidator(0.20);
  EXPECT_FALSE(v.validateGoal(m, m.worldX(28), m.worldY(30)).valid);
  EXPECT_TRUE(v.validateGoal(m, m.worldX(20), m.worldY(30)).valid);
}

TEST(PathValidator, GoalOutsideMapIsRejected) {
  const GridMap m = makeMap(40, 40, kFree);
  const PathValidator v = makeValidator();
  EXPECT_FALSE(v.validateGoal(m, -1.0, -1.0).valid);
}

TEST(PathValidator, EmptyMapIsRejectedWithoutCrashing) {
  const GridMap empty;
  const PathValidator v = makeValidator();
  EXPECT_FALSE(v.validateGoal(empty, 0.0, 0.0).valid);
  EXPECT_FALSE(v.validatePath(empty, makePath({{0.0, 0.0}, {1.0, 0.0}}), {1.0, 0.0}).valid);
}

// ------------------------- 校验2：路径 -------------------------

TEST(PathValidator, FullyFreePathIsAccepted) {
  const GridMap m = makeMap(80, 80, kFree);
  const PathValidator v = makeValidator();
  const auto path = makePath({{0.5, 0.5}, {1.0, 0.5}, {1.5, 0.5}});
  const ValidationResult r = v.validatePath(m, path, {1.5, 0.5});
  EXPECT_TRUE(r.valid) << r.reason;
  EXPECT_GT(r.samples_checked, path.size());     // 证明确实做了段内插值，而不是只查顶点
}

TEST(PathValidator, EmptyAndSinglePointPathsAreRejected) {
  const GridMap m = makeMap(40, 40, kFree);
  const PathValidator v = makeValidator();
  EXPECT_FALSE(v.validatePath(m, {}, {0.5, 0.5}).valid);
  EXPECT_FALSE(v.validatePath(m, makePath({{0.5, 0.5}}), {0.5, 0.5}).valid);
}

TEST(PathValidator, TruncatedPathIsRejected) {
  // 规划器把不可达目标「尽力靠近」后返回半截路径，不能算成功。
  const GridMap m = makeMap(80, 80, kFree);
  const PathValidator v = makeValidator();
  const auto path = makePath({{0.5, 0.5}, {1.0, 0.5}});
  EXPECT_FALSE(v.validatePath(m, path, {3.0, 0.5}).valid);   // 终点差 2m >> 0.5m 容差
}

TEST(PathValidator, PathVertexOnUnknownIsRejected) {
  GridMap m = makeMap(80, 80, kFree);
  fillRect(m, 20, 8, 20, 12, kUnknown);
  const PathValidator v = makeValidator();
  const auto path = makePath({{0.5, 0.5}, {m.worldX(20), m.worldY(10)}, {1.5, 0.5}});
  const ValidationResult r = v.validatePath(m, path, {1.5, 0.5});
  EXPECT_FALSE(r.valid);
  EXPECT_GE(r.bad_segment_index, 0);
}

TEST(PathValidator, SegmentCrossingUnknownIsRejected) {
  // ===================== 本文件最关键的一条 =====================
  // 路径只有两个顶点，两个顶点所在格都合法；未知区是一条竖直窄带，
  // 夹在两顶点正中间。只检查顶点的实现会判定「路径合法」并放行，
  // 机器人随后直接穿过未知带。
  GridMap m = makeMap(80, 40, kFree);
  fillRect(m, 39, 0, 41, 39, kUnknown);          // x=39..41 三列未知(0.15m 宽)

  const PathValidator v = makeValidator(0.05);   // 净空调到 1 格，避免干扰本条判定
  const double y = m.worldY(20);
  const auto path = makePath({{m.worldX(10), y}, {m.worldX(70), y}});

  // 先自证前提：两个顶点自身都是合法的已知空闲格。
  ASSERT_TRUE(v.isKnownFree(m, m.worldX(10), y));
  ASSERT_TRUE(v.isKnownFree(m, m.worldX(70), y));

  const ValidationResult r = v.validatePath(m, path, {m.worldX(70), y});
  EXPECT_FALSE(r.valid) << "段内插值采样失效：未知窄带被放过了";
  EXPECT_EQ(r.bad_segment_index, 0);
  // 首个非法点应落在未知带附近(x≈39..41 格 ⇒ 1.95..2.10m)。
  EXPECT_NEAR(r.first_bad_point.x, m.worldX(40), 0.15);
}

TEST(PathValidator, SegmentCrossingOccupiedIsRejected) {
  GridMap m = makeMap(80, 40, kFree);
  fillRect(m, 39, 0, 41, 39, kOccupied);
  const PathValidator v = makeValidator(0.05);
  const double y = m.worldY(20);
  const auto path = makePath({{m.worldX(10), y}, {m.worldX(70), y}});
  EXPECT_FALSE(v.validatePath(m, path, {m.worldX(70), y}).valid);
}

TEST(PathValidator, SampleStepIsAtMostHalfResolution) {
  // 自动步长必须 <= resolution/2，否则单格宽的未知缝隙可能被跨过去。
  const GridMap m = makeMap(80, 40, kFree);
  const PathValidator v = makeValidator(0.05);
  const double y = m.worldY(20);
  const double x0 = m.worldX(10);
  const double x1 = m.worldX(70);
  const ValidationResult r = v.validatePath(m, makePath({{x0, y}, {x1, y}}), {x1, y});
  ASSERT_TRUE(r.valid) << r.reason;
  const double length = x1 - x0;
  const double expected_min = length / (m.resolution / 2.0);
  EXPECT_GE(static_cast<double>(r.samples_checked), expected_min);
}

TEST(PathValidator, MaxSamplesCapsWorkWithoutFalseAccept) {
  // 采样上限是防御性保护，但绝不能因为「点数超限」就把路径判成合法。
  PathValidatorParams p;
  p.goal_clearance_radius = 0.05;
  p.goal_unknown_clearance_radius = 0.05;   // 不能大于占据净空，否则 configure 拒绝
  p.max_samples = 10U;
  PathValidator v;
  std::string err;
  ASSERT_TRUE(v.configure(p, err)) << err;

  const GridMap m = makeMap(400, 40, kFree);
  const double y = m.worldY(20);
  const auto path = makePath({{m.worldX(5), y}, {m.worldX(395), y}});
  EXPECT_FALSE(v.validatePath(m, path, {m.worldX(395), y}).valid);
}

// ============ 跟踪期几何辅助（「未到位不换路径」判据的算法核心）============
//
// 这一组测试守的是这个已实测的缺陷：跟踪期无条件周期重规划，
// 36 个目标下发了 393 次 FollowPath，平均每个目标换 10.9 条路径、
// 每条只被跟踪约 1.5s 就被下一条抢占。修法是「只有当前路径失效才换」，
// 而失效判据完全建立在下面这三个函数上，所以它们必须逐条钉死。

TEST(TrackingHelpers, NearestIndexPicksClosestVertex) {
  const std::vector<PlanarPoint> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};
  EXPECT_EQ(nearestPathIndex(path, {0.1, 0.0}), 0U);
  EXPECT_EQ(nearestPathIndex(path, {1.4, 0.2}), 1U);
  EXPECT_EQ(nearestPathIndex(path, {2.6, -0.1}), 3U);
  EXPECT_EQ(nearestPathIndex(path, {99.0, 99.0}), 3U);   // 远处也要落在末点，不是越界
}

TEST(TrackingHelpers, DeviationOfEmptyPathIsNegativeNotZero) {
  // !!! 这是本组最关键的一条 !!!
  // 空路径返回 0 会让「根本没有路径」被读成「完美贴合路径」，
  // 于是永远不触发重规划 —— 与「没收到 scan 当成前方无障碍」是同一类错误。
  EXPECT_LT(pathDeviation({}, {0.0, 0.0}), 0.0);
}

TEST(TrackingHelpers, DeviationMeasuresDistanceToNearestVertex) {
  const std::vector<PlanarPoint> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  EXPECT_NEAR(pathDeviation(path, {1.0, 0.5}), 0.5, 1e-9);
  EXPECT_NEAR(pathDeviation(path, {1.0, 0.0}), 0.0, 1e-9);
}

TEST(TrackingHelpers, RemainingPathStartsAtNearestVertexAndKeepsEndpoint) {
  const std::vector<PlanarPoint> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};
  const auto rest = remainingPath(path, {1.9, 0.1});
  ASSERT_EQ(rest.size(), 2U);
  EXPECT_NEAR(rest.front().x, 2.0, 1e-9);
  // 终点必须保留：validatePath 要靠它做「规划终点没被截断」检查。
  EXPECT_NEAR(rest.back().x, 3.0, 1e-9);
}

TEST(TrackingHelpers, RemainingPathOfEmptyIsEmpty) {
  EXPECT_TRUE(remainingPath({}, {0.0, 0.0}).empty());
}

TEST(TrackingHelpers, RemainingPathNearGoalShrinksToSinglePoint) {
  // 快到终点时剩余段只剩 1 个点。调用方必须把这种情况当成「快到了」，
  // 而不是丢给 validatePath —— 后者对 <2 点的路径一律判不合法（这是对的），
  // 若直接拿它的结论去决定要不要换路径，每个目标的收尾阶段都会被打断一次。
  const std::vector<PlanarPoint> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  const auto rest = remainingPath(path, {2.01, 0.0});
  EXPECT_EQ(rest.size(), 1U);

  const PathValidator v = makeValidator(0.05);
  const GridMap m = makeMap(80, 40, kFree);
  EXPECT_FALSE(v.validatePath(m, rest, {2.0, 0.0}).valid);
}

TEST(TrackingHelpers, RemainingPathIgnoresObstacleBehindRobot) {
  // 只校验剩余段的理由：身后新观测到的障碍与「还能不能继续往前跟」无关。
  // 拿整条路径去校验会因为走过的那一段变成障碍而反复误判需要重规划。
  GridMap m = makeMap(120, 40, kFree);
  const double y = m.worldY(20);
  const std::vector<PlanarPoint> path{
    {m.worldX(10), y}, {m.worldX(40), y}, {m.worldX(70), y}, {m.worldX(110), y}};
  // 在第一段上（机器人身后）放一堵墙
  fillRect(m, 20, 0, 22, 39, kOccupied);

  const PathValidator v = makeValidator(0.05);
  const PlanarPoint goal{m.worldX(110), y};
  const PlanarPoint robot{m.worldX(70), y};

  EXPECT_FALSE(v.validatePath(m, path, goal).valid);          // 整条：被身后的墙否掉
  const auto rest = remainingPath(path, robot);
  EXPECT_TRUE(v.validatePath(m, rest, goal).valid);           // 剩余段：仍然可通行
}

TEST(TrackingHelpers, RemainingPathStillCatchesObstacleAhead) {
  // 反向对照：前方新出现障碍时，剩余段校验必须报不合法 ——
  // 否则「未失效不换路径」就变成了「永远不换路径」，机器人会往障碍里开。
  GridMap m = makeMap(120, 40, kFree);
  const double y = m.worldY(20);
  const std::vector<PlanarPoint> path{
    {m.worldX(10), y}, {m.worldX(40), y}, {m.worldX(70), y}, {m.worldX(110), y}};
  fillRect(m, 90, 0, 92, 39, kOccupied);                      // 障碍在机器人前方

  const PathValidator v = makeValidator(0.05);
  const auto rest = remainingPath(path, {m.worldX(70), y});
  EXPECT_FALSE(v.validatePath(m, rest, {m.worldX(110), y}).valid);
}

}  // namespace
}  // namespace astribot_s1_autonomy
