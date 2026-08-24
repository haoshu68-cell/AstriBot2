// Copyright 2026 Astribot.
//
// costmap_adapter 单元测试。
//
// 除了常规的边界/映射校验，这里刻意放了一组**回归哨兵**
// （TwoGridDeadlock_*）：把「探索一个目标都发不出去」那个真实缺陷的成因
// 用最小构造复现出来 —— 同一条路径，在代价地图派生的栅格上合法，
// 在 SLAM 原始 /map 上被判「穿越未知」。
// 谁要是哪天把校验数据源改回 /map，这两个测试会立刻红。
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "astribot_s1_autonomy/costmap_adapter.hpp"
#include "astribot_s1_autonomy/path_validator.hpp"

namespace astribot_s1_autonomy
{
namespace
{

/// 造一张全自由的代价地图原始字节。
std::vector<uint8_t> freeCostmapBytes(unsigned int w, unsigned int h)
{
  return std::vector<uint8_t>(static_cast<std::size_t>(w) * h, Nav2CostValues::kFreeSpace);
}

PathValidatorParams defaultValidatorParams()
{
  PathValidatorParams p;
  p.occupied_threshold = 65;
  p.free_threshold = 25;
  p.goal_clearance_radius = 0.42;
  p.goal_unknown_clearance_radius = 0.0;
  p.path_sample_step = 0.0;
  p.path_endpoint_tolerance = 0.5;
  p.max_samples = 200000U;
  return p;
}

}  // namespace

// ======================= 参数自检 =======================

TEST(CostmapAdapterParams, AcceptsNav2Defaults)
{
  CostmapAdapterParams p;
  std::string error;
  EXPECT_TRUE(validateCostmapAdapterParams(p, error)) << error;
  EXPECT_EQ(p.unknown_cost, 255);
  EXPECT_EQ(p.lethal_cost_threshold, 253);
}

TEST(CostmapAdapterParams, RejectsZeroLethalThreshold)
{
  // 阈值 0 会把 FREE_SPACE 也判成致命障碍，整张图全是墙。
  // 这种配置错了以后现场毫无线索，必须在启动时就拦住。
  CostmapAdapterParams p;
  p.lethal_cost_threshold = 0;
  std::string error;
  EXPECT_FALSE(validateCostmapAdapterParams(p, error));
  EXPECT_NE(error.find("lethal_cost_threshold"), std::string::npos);
}

TEST(CostmapAdapterParams, RejectsOutOfRangeValues)
{
  std::string error;
  CostmapAdapterParams too_big;
  too_big.lethal_cost_threshold = 256;
  EXPECT_FALSE(validateCostmapAdapterParams(too_big, error));

  CostmapAdapterParams negative_unknown;
  negative_unknown.unknown_cost = -1;
  EXPECT_FALSE(validateCostmapAdapterParams(negative_unknown, error));
}

TEST(CostmapAdapterParams, RejectsLethalThresholdAboveUnknown)
{
  // 阈值高于未知值 → 没有任何代价值会被判成致命障碍，等于关掉了碰撞校验。
  CostmapAdapterParams p;
  p.unknown_cost = 200;
  p.lethal_cost_threshold = 253;
  std::string error;
  EXPECT_FALSE(validateCostmapAdapterParams(p, error));
  EXPECT_NE(error.find("致命障碍"), std::string::npos);
}

// ======================= 转换的边界条件 =======================

TEST(CostmapAdapter, RejectsDataLengthMismatch)
{
  // 长度不一致必须拒绝转换：放过去就是越界读。
  GridMap out;
  std::string error;
  std::vector<uint8_t> data(10U, 0U);
  EXPECT_FALSE(
    costmapToGridMap(4U, 4U, 0.05, 0.0, 0.0, data, CostmapAdapterParams{}, out, error));
  EXPECT_NE(error.find("不一致"), std::string::npos);
  EXPECT_TRUE(out.empty());
}

TEST(CostmapAdapter, RejectsZeroSizeAndBadResolution)
{
  GridMap out;
  std::string error;
  EXPECT_FALSE(
    costmapToGridMap(0U, 4U, 0.05, 0.0, 0.0, {}, CostmapAdapterParams{}, out, error));

  std::vector<uint8_t> data = freeCostmapBytes(4U, 4U);
  EXPECT_FALSE(
    costmapToGridMap(4U, 4U, 0.0, 0.0, 0.0, data, CostmapAdapterParams{}, out, error));
  EXPECT_NE(error.find("分辨率"), std::string::npos);

  EXPECT_FALSE(
    costmapToGridMap(4U, 4U, -0.05, 0.0, 0.0, data, CostmapAdapterParams{}, out, error));
}

TEST(CostmapAdapter, RejectsBadParamsBeforeTouchingData)
{
  // 参数非法时不应该开始转换，也不应该崩在越界读上。
  CostmapAdapterParams bad;
  bad.lethal_cost_threshold = 0;
  GridMap out;
  std::string error;
  std::vector<uint8_t> data = freeCostmapBytes(4U, 4U);
  EXPECT_FALSE(costmapToGridMap(4U, 4U, 0.05, 0.0, 0.0, data, bad, out, error));
  EXPECT_NE(error.find("参数非法"), std::string::npos);
}

// ======================= 三态映射 =======================

TEST(CostmapAdapter, MapsNav2CostSemanticsToThreeStates)
{
  // 逐一钉死 nav2 代价语义的映射结果。
  // 1~252 是膨胀梯度，**必须**映射成空闲——它是代价偏好而不是不可通行，
  // 判成障碍会让所有贴墙路径被否决，探索重新锁死（详见头文件说明）。
  std::vector<uint8_t> data = {
    Nav2CostValues::kNoInformation,             // 255 未知
    Nav2CostValues::kLethalObstacle,            // 254 障碍本体
    Nav2CostValues::kInscribedInflatedObstacle,  // 253 足迹必碰
    252U,                                       // 膨胀梯度上端
    128U,                                       // 膨胀梯度中段
    1U,                                         // 膨胀梯度下端
    Nav2CostValues::kFreeSpace,                 // 0 自由
    Nav2CostValues::kFreeSpace,
  };
  GridMap out;
  std::string error;
  ASSERT_TRUE(
    costmapToGridMap(4U, 2U, 0.05, 0.0, 0.0, data, CostmapAdapterParams{}, out, error)) << error;
  ASSERT_TRUE(out.consistent());

  EXPECT_EQ(out.data[0], static_cast<int8_t>(-1));
  EXPECT_EQ(out.data[1], static_cast<int8_t>(100));
  EXPECT_EQ(out.data[2], static_cast<int8_t>(100));
  EXPECT_EQ(out.data[3], static_cast<int8_t>(0));
  EXPECT_EQ(out.data[4], static_cast<int8_t>(0));
  EXPECT_EQ(out.data[5], static_cast<int8_t>(0));
  EXPECT_EQ(out.data[6], static_cast<int8_t>(0));
}

TEST(CostmapAdapter, UnknownWinsWhenThresholdsOverlap)
{
  // 两个阈值配成相等时，255 既满足「等于未知值」又满足「>= 致命下界」。
  // 必须判未知：把未知误判成墙会让前沿方向全变成障碍，
  // 现场表现是「明明有路却说被挡住」，比反过来更难排查。
  CostmapAdapterParams p;
  p.unknown_cost = 255;
  p.lethal_cost_threshold = 255;
  GridMap out;
  std::string error;
  std::vector<uint8_t> data = {Nav2CostValues::kNoInformation, 254U};
  ASSERT_TRUE(costmapToGridMap(2U, 1U, 0.05, 0.0, 0.0, data, p, out, error)) << error;
  EXPECT_EQ(out.data[0], static_cast<int8_t>(-1));
  // 254 < 阈值 255，落到空闲分支。这是该配置的必然结果，
  // 也正是为什么默认阈值取 253 而不是 255。
  EXPECT_EQ(out.data[1], static_cast<int8_t>(0));
}

TEST(CostmapAdapter, PreservesGeometryAndRowMajorOrder)
{
  // 几何信息（尺寸/分辨率/原点）必须原样带过去：
  // 代价地图的 origin 与 /map 的并不相同，串了就会整体偏移，
  // 而偏移后的校验依然「有结果」，不会报错，极难发现。
  std::vector<uint8_t> data = freeCostmapBytes(5U, 3U);
  data[static_cast<std::size_t>(2U) * 5U + 3U] = Nav2CostValues::kLethalObstacle;  // (mx=3,my=2)

  GridMap out;
  std::string error;
  ASSERT_TRUE(
    costmapToGridMap(5U, 3U, 0.05, -1.25, 2.5, data, CostmapAdapterParams{}, out, error)) << error;

  EXPECT_EQ(out.width, 5U);
  EXPECT_EQ(out.height, 3U);
  EXPECT_DOUBLE_EQ(out.resolution, 0.05);
  EXPECT_DOUBLE_EQ(out.origin_x, -1.25);
  EXPECT_DOUBLE_EQ(out.origin_y, 2.5);
  EXPECT_EQ(out.data[out.index(3U, 2U)], static_cast<int8_t>(100));
  EXPECT_EQ(out.data[out.index(2U, 2U)], static_cast<int8_t>(0));

  // 原点带上以后世界坐标反查也必须对得上。
  unsigned int mx = 0U;
  unsigned int my = 0U;
  ASSERT_TRUE(out.worldToMap(out.worldX(3U), out.worldY(2U), mx, my));
  EXPECT_EQ(mx, 3U);
  EXPECT_EQ(my, 2U);
}

TEST(CostmapAdapter, OutputIsThresholdCaliberIndependent)
{
  // 输出用 0/100 两个极值，目的是让这一层不必知道 PathValidator 的阈值。
  // 这里用两套差异很大的阈值验证：结论必须一致，否则两处参数就会漂移。
  std::vector<uint8_t> data = {
    Nav2CostValues::kFreeSpace, 252U, Nav2CostValues::kLethalObstacle,
    Nav2CostValues::kNoInformation};
  GridMap grid;
  std::string error;
  ASSERT_TRUE(
    costmapToGridMap(4U, 1U, 0.05, 0.0, 0.0, data, CostmapAdapterParams{}, grid, error)) << error;

  for (const auto & thresholds : std::vector<std::pair<int, int>>{{65, 25}, {99, 1}}) {
    PathValidatorParams vp = defaultValidatorParams();
    vp.occupied_threshold = thresholds.first;
    vp.free_threshold = thresholds.second;
    PathValidator validator;
    std::string cfg_error;
    ASSERT_TRUE(validator.configure(vp, cfg_error)) << cfg_error;

    EXPECT_TRUE(validator.isKnownFree(grid, grid.worldX(0U), grid.worldY(0U)));
    EXPECT_TRUE(validator.isKnownFree(grid, grid.worldX(1U), grid.worldY(0U)));
    EXPECT_FALSE(validator.isKnownFree(grid, grid.worldX(2U), grid.worldY(0U)));
    EXPECT_FALSE(validator.isKnownFree(grid, grid.worldX(3U), grid.worldY(0U)));
  }
}

// ======================= 统计 =======================

TEST(CostmapAdapter, SummarizeCountsThreeStates)
{
  std::vector<uint8_t> data = {
    Nav2CostValues::kNoInformation, Nav2CostValues::kNoInformation,
    Nav2CostValues::kLethalObstacle, Nav2CostValues::kFreeSpace};
  GridMap grid;
  std::string error;
  ASSERT_TRUE(
    costmapToGridMap(4U, 1U, 0.05, 0.0, 0.0, data, CostmapAdapterParams{}, grid, error)) << error;

  const CostmapGridStats stats = summarizeGrid(grid);
  EXPECT_EQ(stats.total, 4U);
  EXPECT_EQ(stats.unknown, 2U);
  EXPECT_EQ(stats.lethal, 1U);
  EXPECT_EQ(stats.free, 1U);
  EXPECT_DOUBLE_EQ(stats.unknownRatio(), 0.5);
  EXPECT_DOUBLE_EQ(stats.lethalRatio(), 0.25);
}

TEST(CostmapAdapter, SummarizeOnInconsistentGridReturnsZeros)
{
  GridMap broken;
  broken.width = 4U;
  broken.height = 4U;
  broken.resolution = 0.05;
  broken.data.resize(3U);
  const CostmapGridStats stats = summarizeGrid(broken);
  EXPECT_EQ(stats.total, 0U);
  EXPECT_DOUBLE_EQ(stats.unknownRatio(), 0.0);
}

// =============== 回归哨兵：两张图判据互相锁死 ===============
//
// 真实缺陷复现（实测 631/631 次候选否决、dispatched 恒为 0）：
//   · planner_server 在 global_costmap 上规划。它的 obstacle_layer 配了
//     clearing:True，会沿每条扫描射线把栅格刷成 FREE_SPACE，覆盖 NO_INFORMATION。
//   · 而 slam_toolbox(Karto) 丢弃 inf 读数，同一片区域在 /map 里仍是未知。
//   · 于是规划器合法输出的路径，拿 /map 去校验必然被判「穿越未知」。
// 实测非法点最近出现在机器人正前方 0.05m（路径第 3 个采样点），无路可活。
//
// 下面两个测试把这个差异做成最小构造：**同一条路径、同一套校验参数**，
// 只换栅格数据源，结论相反。
namespace
{

constexpr unsigned int kGridW = 40U;   // 40 * 0.05 = 2.0m
constexpr unsigned int kGridH = 40U;
constexpr double kRes = 0.05;

/// SLAM 原始 /map 风格：起点附近一小段就是未知区。
/// 对应实测现象——机器人正前方 0.05m 处 /map 即为 -1。
GridMap slamStyleGridWithUnknownBandAhead()
{
  GridMap g;
  g.width = kGridW;
  g.height = kGridH;
  g.resolution = kRes;
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.data.assign(static_cast<std::size_t>(kGridW) * kGridH, static_cast<int8_t>(0));
  // x ∈ [0.15, 0.60) 整列判未知。
  for (unsigned int my = 0U; my < kGridH; ++my) {
    for (unsigned int mx = 3U; mx < 12U; ++mx) {
      g.data[g.index(mx, my)] = static_cast<int8_t>(-1);
    }
  }
  return g;
}

/// 一条沿 y=0.10 从 x=0.10 走到 x=1.00 的直线路径，顶点间距 0.10m。
/// 顶点间距刻意大于分辨率，用来同时验证「必须段内插值采样」这条约束。
std::vector<PlanarPoint> straightPathAlongX()
{
  std::vector<PlanarPoint> path;
  for (int i = 1; i <= 10; ++i) {
    path.push_back(PlanarPoint{static_cast<double>(i) * 0.10, 0.10});
  }
  return path;
}

}  // namespace

TEST(TwoGridDeadlock, SlamMapRejectsThePathThePlannerLegallyProduced)
{
  // 这是缺陷现场：用 /map 校验，路径在起步阶段就被判穿越未知。
  PathValidator validator;
  std::string cfg_error;
  ASSERT_TRUE(validator.configure(defaultValidatorParams(), cfg_error)) << cfg_error;

  const GridMap slam_map = slamStyleGridWithUnknownBandAhead();
  const std::vector<PlanarPoint> path = straightPathAlongX();
  const ValidationResult r =
    validator.validatePath(slam_map, path, PlanarPoint{1.00, 0.10});

  EXPECT_FALSE(r.valid);
  EXPECT_NE(r.reason.find("穿过未知栅格"), std::string::npos) << r.reason;
  // 非法点必须落在未知带里，且在路径极早期——与实测的「第 3 个采样点」同性质。
  EXPECT_GE(r.first_bad_point.x, 0.15);
  EXPECT_LT(r.first_bad_point.x, 0.60);
  EXPECT_LE(r.samples_checked, 10U);
}

TEST(TwoGridDeadlock, CostmapDerivedGridAcceptsTheSamePath)
{
  // 正解：改用规划器所用的那张代价地图（obstacle_layer 已把该区域清成自由），
  // 同一条路径、同一套参数，校验通过。矛盾在构造上消失。
  GridMap costmap_grid;
  std::string error;
  ASSERT_TRUE(
    costmapToGridMap(
      kGridW, kGridH, kRes, 0.0, 0.0, freeCostmapBytes(kGridW, kGridH),
      CostmapAdapterParams{}, costmap_grid, error)) << error;

  PathValidator validator;
  std::string cfg_error;
  ASSERT_TRUE(validator.configure(defaultValidatorParams(), cfg_error)) << cfg_error;

  const std::vector<PlanarPoint> path = straightPathAlongX();
  const ValidationResult r =
    validator.validatePath(costmap_grid, path, PlanarPoint{1.00, 0.10});

  EXPECT_TRUE(r.valid) << r.reason;
  // 段内插值确实生效了：10 个顶点、跨度 0.9m、步长 0.025m → 远多于 10 个采样点。
  EXPECT_GT(r.samples_checked, 30U);
}

TEST(TwoGridDeadlock, CostmapGridStillRejectsRealLethalObstacles)
{
  // 换数据源不等于放宽安全性：代价地图里的致命障碍照样拦得住。
  std::vector<uint8_t> bytes = freeCostmapBytes(kGridW, kGridH);
  // 在 x≈0.50、y≈0.10 处放一片致命障碍，正压在路径上。
  for (unsigned int my = 0U; my < 5U; ++my) {
    for (unsigned int mx = 9U; mx < 12U; ++mx) {
      bytes[static_cast<std::size_t>(my) * kGridW + mx] = Nav2CostValues::kLethalObstacle;
    }
  }
  GridMap grid;
  std::string error;
  ASSERT_TRUE(
    costmapToGridMap(kGridW, kGridH, kRes, 0.0, 0.0, bytes, CostmapAdapterParams{}, grid, error))
    << error;

  PathValidator validator;
  std::string cfg_error;
  ASSERT_TRUE(validator.configure(defaultValidatorParams(), cfg_error)) << cfg_error;

  const ValidationResult r =
    validator.validatePath(grid, straightPathAlongX(), PlanarPoint{1.00, 0.10});
  EXPECT_FALSE(r.valid);
  EXPECT_NE(r.reason.find("非空闲栅格"), std::string::npos) << r.reason;
}

TEST(TwoGridDeadlock, EndpointTruncationCheckSurvivesTheDataSourceChange)
{
  // 终点截断检查是**刻意保留**的一条：它拦的是「规划器把不可达目标尽力靠近后
  // 返回半截路径、action 仍报 SUCCEEDED」这一类，与查哪张图无关。
  // 换数据源时很容易连它一起丢掉，所以单独钉一个测试。
  GridMap grid;
  std::string error;
  ASSERT_TRUE(
    costmapToGridMap(
      kGridW, kGridH, kRes, 0.0, 0.0, freeCostmapBytes(kGridW, kGridH),
      CostmapAdapterParams{}, grid, error)) << error;

  PathValidator validator;
  std::string cfg_error;
  ASSERT_TRUE(validator.configure(defaultValidatorParams(), cfg_error)) << cfg_error;

  // 路径走到 1.00 就停了，但请求的目标在 1.90 —— 差 0.9m，超过上限 0.5m。
  const ValidationResult r =
    validator.validatePath(grid, straightPathAlongX(), PlanarPoint{1.90, 0.10});
  EXPECT_FALSE(r.valid);
  EXPECT_NE(r.reason.find("截断"), std::string::npos) << r.reason;
}

// =============== 回归哨兵：净空半径的口径必须跟着数据源改 ===============
//
// 这是「两张图混用」的第二处实例，实测代价是 691/697 = 99.1% 的目标复检否决。
// goal_clearance_radius=0.42 是按 /map（占据格 = 原始障碍本体）标定的；
// 而 costmap 里被判占据的是代价 >=253 的格，其语义已经是
// 「机器人中心在此则足迹必然碰撞」——膨胀余量算过了，再套邻域就是重复计算。
namespace
{

/// 造一张代价地图：左侧一竖条是 nav2 膨胀后的致命区（>=253），其余自由。
/// 致命区右边界在 x = 0.30，模拟「真实障碍在 x≈-0.09、内切半径 0.388 膨胀到 0.30」。
GridMap costmapWithInflatedLethalBand()
{
  std::vector<uint8_t> bytes = freeCostmapBytes(kGridW, kGridH);
  for (unsigned int my = 0U; my < kGridH; ++my) {
    for (unsigned int mx = 0U; mx < 6U; ++mx) {   // x ∈ [0, 0.30)
      bytes[static_cast<std::size_t>(my) * kGridW + mx] =
        Nav2CostValues::kInscribedInflatedObstacle;
    }
  }
  GridMap grid;
  std::string error;
  const bool ok = costmapToGridMap(
    kGridW, kGridH, kRes, 0.0, 0.0, bytes, CostmapAdapterParams{}, grid, error);
  EXPECT_TRUE(ok) << error;
  return grid;
}

}  // namespace

TEST(ClearanceCaliber, NonZeroRadiusDoubleCountsTheFootprintOnCostmap)
{
  // 目标点 (0.35, 0.50) 位于致命区之外 —— 在 costmap 口径下机器人停这里不碰撞，
  // 是个完全合法的贴墙前沿。但 0.42m 净空半径会把它否掉。
  PathValidatorParams vp = defaultValidatorParams();
  vp.goal_clearance_radius = 0.42;   // 按 /map 标定的旧值
  PathValidator validator;
  std::string cfg_error;
  ASSERT_TRUE(validator.configure(vp, cfg_error)) << cfg_error;

  const GridMap grid = costmapWithInflatedLethalBand();
  const ValidationResult r = validator.validateGoal(grid, 0.35, 0.50);
  EXPECT_FALSE(r.valid);
  EXPECT_NE(r.reason.find("净空半径"), std::string::npos) << r.reason;
}

TEST(ClearanceCaliber, ZeroRadiusAcceptsTheLegalWallHuggingGoal)
{
  // 同一个目标点，净空半径改 0 → 通过。
  //
  // 注意：0 是精确的**碰撞**判据，但**不是**推荐的生产取值。
  // 实测 0.0 会让目标压在致命区边界上，控制器在自己的收敛容差球内反复碰撞代价，
  // 结果 3 个目标全部导航超时（见下面 ControllerToleranceIsTheRightCaliber）。
  PathValidatorParams vp = defaultValidatorParams();
  vp.goal_clearance_radius = 0.0;
  PathValidator validator;
  std::string cfg_error;
  ASSERT_TRUE(validator.configure(vp, cfg_error)) << cfg_error;

  const GridMap grid = costmapWithInflatedLethalBand();
  const ValidationResult r = validator.validateGoal(grid, 0.35, 0.50);
  EXPECT_TRUE(r.valid) << r.reason;
}

TEST(ClearanceCaliber, ControllerToleranceIsTheRightCaliber)
{
  // 生产取值 = nav2 controller 的 xy_goal_tolerance(本项目 0.25)。
  // 语义：以目标为心、控制器容差为半径的球内，足迹处处不碰撞
  // —— 目标既合法(不像 0.42 那样重复计足迹)，又真的收敛得进去(不像 0.0 那样超时)。
  PathValidatorParams vp = defaultValidatorParams();
  vp.goal_clearance_radius = 0.25;
  PathValidator validator;
  std::string cfg_error;
  ASSERT_TRUE(validator.configure(vp, cfg_error)) << cfg_error;

  const GridMap grid = costmapWithInflatedLethalBand();   // 致命区 x ∈ [0, 0.30)

  // 距致命区仅 0.05m —— 控制器容差球会伸进致命区，必须否掉。
  const ValidationResult too_close = validator.validateGoal(grid, 0.35, 0.50);
  EXPECT_FALSE(too_close.valid) << too_close.reason;

  // 距致命区 0.30m > 0.25m —— 容差球整体在安全区内，放行。
  const ValidationResult ok = validator.validateGoal(grid, 0.60, 0.50);
  EXPECT_TRUE(ok.valid) << ok.reason;

  // 而同一个点在旧的 0.42m 口径下会被误否 —— 这就是「重复计足迹」的代价。
  PathValidatorParams old_vp = defaultValidatorParams();
  old_vp.goal_clearance_radius = 0.42;
  PathValidator old_validator;
  ASSERT_TRUE(old_validator.configure(old_vp, cfg_error)) << cfg_error;
  EXPECT_FALSE(old_validator.validateGoal(grid, 0.60, 0.50).valid);
}

TEST(ClearanceCaliber, ZeroRadiusStillRejectsGoalsInsideTheLethalRegion)
{
  // 关键：半径改 0 **不等于**放弃碰撞校验。
  // 目标落在致命区内（足迹必然碰撞）时，单格检查照样拦住。
  PathValidatorParams vp = defaultValidatorParams();
  vp.goal_clearance_radius = 0.0;
  PathValidator validator;
  std::string cfg_error;
  ASSERT_TRUE(validator.configure(vp, cfg_error)) << cfg_error;

  const GridMap grid = costmapWithInflatedLethalBand();
  const ValidationResult r = validator.validateGoal(grid, 0.15, 0.50);   // x<0.30，在致命区里
  EXPECT_FALSE(r.valid);
  EXPECT_NE(r.reason.find("不是空闲"), std::string::npos) << r.reason;
}

TEST(ClearanceCaliber, ZeroRadiusStillRejectsUnknownGoalCell)
{
  // 未知格也照样拦得住——「不得把未知栅格作为航点」这条没有被削弱。
  std::vector<uint8_t> bytes = freeCostmapBytes(kGridW, kGridH);
  bytes[static_cast<std::size_t>(10U) * kGridW + 7U] = Nav2CostValues::kNoInformation;
  GridMap grid;
  std::string error;
  ASSERT_TRUE(
    costmapToGridMap(kGridW, kGridH, kRes, 0.0, 0.0, bytes, CostmapAdapterParams{}, grid, error))
    << error;

  PathValidatorParams vp = defaultValidatorParams();
  vp.goal_clearance_radius = 0.0;
  PathValidator validator;
  std::string cfg_error;
  ASSERT_TRUE(validator.configure(vp, cfg_error)) << cfg_error;

  const ValidationResult r = validator.validateGoal(grid, grid.worldX(7U), grid.worldY(10U));
  EXPECT_FALSE(r.valid);
  EXPECT_NE(r.reason.find("未知栅格"), std::string::npos) << r.reason;
}

}  // namespace astribot_s1_autonomy
