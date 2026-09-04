// Copyright 2026 Astribot
//
// narrow_math 的离线测试。
//
// 第一职责是守住两条不能错的判据：
//   🔴 真障碍(254)绝不允许贴 —— 用穷举哨兵守
//   🔴 中心格致命时本层必须拒绝接管 —— 那是物理放不进去，不是窄通道
// 第二职责是守住那条**决定能不能过去**的几何：八边形有利朝向每 45° 复现。

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "astribot_s1_path_tracking/narrow_math.hpp"

namespace astribot_s1_path_tracking
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kOct = kPi / 4.0;      // 正八边形有利朝向周期
}  // namespace

// =====================================================================
// 🔴 红线哨兵
// =====================================================================

TEST(NarrowRedLine, LethalAtFavorableOrientationNeverYieldsNarrow)
{
  // **最有利朝向下**仍压真障碍(254) 时，任何 中心格代价 x 当前朝向代价 x
  // 有无路径，都必须判 kPhysicallyBlocked，绝不能返回 kNarrow。
  NarrowTriggerConfig cfg;
  for (int center = 0; center <= 255; ++center) {
    for (int have = 0; have <= 1; ++have) {
      for (double fav : {254.0, 254.5, 255.0, 300.0}) {
        for (double fp : {0.0, 253.0, 254.0}) {
          const auto v = evaluateNarrowTrigger(center, fp, fav, have != 0, false, false, cfg);
          EXPECT_EQ(v, NarrowVerdict::kPhysicallyBlocked)
            << "center=" << center << " fp=" << fp << " fav=" << fav
            << " have_path=" << have;
          EXPECT_NE(v, NarrowVerdict::kNarrow) << "红线被突破！";
        }
      }
    }
  }
}

TEST(NarrowRedLine, UnfavorableYawAloneIsNotPhysicallyBlocked)
{
  // 这是本层目标域的核心语义，也是一条实测教训：
  // 正八边形顶点比边中点多伸出 0.034m，通道宽 0.772~0.840m 时
  // 顶点朝墙压 254、边朝墙过得去。
  //
  // 「当前朝向压 254 但最有利朝向干净」必须判 kNarrow（接管并先转朝向），
  // 不能判 kPhysicallyBlocked —— 否则本层在自己唯一存在的理由上把自己否掉。
  // 修复前实测：中心代价 0 -> 218 -> 229 -> 致命，车被一路推进膨胀带深处。
  NarrowTriggerConfig cfg;
  for (double fav : {0.0, 100.0, 252.0, 253.0}) {
    const auto v = evaluateNarrowTrigger(0.0, 254.0, fav, true, false, false, cfg);
    EXPECT_EQ(v, NarrowVerdict::kNarrow)
      << "当前朝向压 254、有利朝向=" << fav << " 被误判成物理堵死";
  }
}

TEST(NarrowRedLine, CenterLethalStillWinsOverUnfavorableYaw)
{
  // 中心格致命时，即使有利朝向干净也不接管 —— 中心致命是「物理放不进去」，
  // 转朝向解决不了（判据是单个中心格，与朝向无关）。
  NarrowTriggerConfig cfg;
  EXPECT_EQ(
    evaluateNarrowTrigger(253.0, 254.0, 0.0, true, false, false, cfg), NarrowVerdict::kCenterLethal);
  EXPECT_EQ(
    evaluateNarrowTrigger(255.0, 253.0, 100.0, true, false, false, cfg), NarrowVerdict::kCenterLethal);
}

TEST(NarrowRedLine, CenterLethalIsNotOurJob)
{
  // 中心格致命 ⇒ clearance<0.388 ⇒ 物理放不进去 ⇒ 本层必须拒绝接管。
  // 穷举足迹代价（254 以下、即"未压到障碍本体"的全部取值）。
  NarrowTriggerConfig cfg;
  for (int fp = 0; fp < 254; ++fp) {
    const auto v = evaluateNarrowTrigger(253.0, fp, 0.0, true, false, false, cfg);
    EXPECT_EQ(v, NarrowVerdict::kCenterLethal) << "fp=" << fp;
    EXPECT_NE(v, NarrowVerdict::kNarrow)
      << "在物理放不进去的地方接管了贴边通行！fp=" << fp;
  }
}

TEST(NarrowRedLine, ScanNeverSelectsLethalOffset)
{
  // 扫描时压到 254 的偏移必须被丢弃。构造：只有一侧可用。
  auto cost = [](double x, double /*y*/, double /*yaw*/) {
      // x > 0.02 的一侧是真障碍
      return x > 0.02 ? 254.0 : 200.0;
    };
  const auto r = scanLateral({0.0, 0.0}, 0.0, kPi / 2.0, 0.10, 0.01, cost);
  // 通道方向 +y ⇒ 横向是 -x 方向；正偏移对应 x<0（可用侧）
  ASSERT_TRUE(r.valid);
  const double chosen_x = -std::sin(kPi / 2.0) * r.best_offset_m;
  EXPECT_LE(chosen_x, 0.02) << "选中了真障碍那一侧";
  EXPECT_LT(cost(chosen_x, 0.0, 0.0), NarrowCostValues::kLethal);
}

TEST(NarrowRedLine, ScanInvalidWhenBothSidesLethal)
{
  auto cost = [](double, double, double) {return 254.0;};
  const auto r = scanLateral({0.0, 0.0}, 0.0, 0.0, 0.10, 0.01, cost);
  EXPECT_FALSE(r.valid);
  EXPECT_EQ(r.usable_count, 0U);
}

// =====================================================================
// 触发判定
// =====================================================================

TEST(NarrowTrigger, TargetBandYieldsNarrow)
{
  // 目标域：中心格可站(<253) + 足迹**多边形**压到真障碍(>=254)。
  // ⚠️ 这里刻意用 254 而不是 253。默认阈值已改成 254，理由是 253 在
  //    多边形查询里会重复计底盘半宽（推导见 narrow_math.hpp 的 NarrowCostValues）。
  NarrowTriggerConfig cfg;
  EXPECT_EQ(evaluateNarrowTrigger(137.0, 254.0, 0.0, true, false, false, cfg), NarrowVerdict::kNarrow);
  EXPECT_EQ(evaluateNarrowTrigger(0.0, 254.0, 0.0, true, false, false, cfg), NarrowVerdict::kNarrow);
}

TEST(NarrowTrigger, InflationBandOnFootprintIsNotBlocked)
{
  // 🔴 本条是这次改动的核心回归测试：足迹多边形读到 253 **不算过不去**。
  //
  // 253 的含义是"这一格离障碍不超过内切半径"，而内切半径就是底盘半宽。
  // 足迹多边形已经把底盘尺寸表达了一遍，再要求它躲开 253 带就是算两遍。
  // 上一版默认阈值 253 让本机每一条 0.65m 级通道都恒判"过不去"
  //（实测 909 次），而实际读数 51 次 253 / 仅 2 次 254 —— 从未真的撞上。
  NarrowTriggerConfig cfg;
  for (double center : {0.0, 100.0, 137.0, 252.0}) {
    EXPECT_EQ(evaluateNarrowTrigger(center, 253.0, 0.0, true, false, false, cfg), NarrowVerdict::kNone)
      << "足迹多边形读到 253 被当成过不去 ⇒ 重复计底盘半宽（center=" << center << "）";
  }
}

TEST(NarrowTrigger, WideAreaYieldsNone)
{
  NarrowTriggerConfig cfg;
  EXPECT_EQ(evaluateNarrowTrigger(0.0, 0.0, 0.0, true, false, false, cfg), NarrowVerdict::kNone);
  EXPECT_EQ(evaluateNarrowTrigger(137.0, 252.0, 0.0, true, false, false, cfg), NarrowVerdict::kNone);
}

TEST(NarrowTrigger, CenterThresholdStaysAtInscribed)
{
  // 与上一条相反的一半：**中心格**读到 253 必须算致命。
  // 那里膨胀带正好代表底盘尺寸，正是 SmacPlanner2D 判起点用的判据。
  // 两个阈值方向相反，任何"统一成一个数"的改法都会破坏其中一半。
  NarrowTriggerConfig cfg;
  EXPECT_DOUBLE_EQ(cfg.center_lethal_threshold, NarrowCostValues::kInscribedInflated);
  EXPECT_DOUBLE_EQ(cfg.footprint_lethal_threshold, NarrowCostValues::kLethal);
  EXPECT_EQ(evaluateNarrowTrigger(253.0, 0.0, 0.0, true, false, false, cfg), NarrowVerdict::kCenterLethal);
}

TEST(NarrowTrigger, NoPathMeansNoTakeover)
{
  // 本层以全局路径为核心约束。无路径不接管 —— 那种情况是 ESCAPE 的事。
  NarrowTriggerConfig cfg;
  EXPECT_EQ(evaluateNarrowTrigger(137.0, 254.0, 0.0, false, false, false, cfg), NarrowVerdict::kNoPath);
}

TEST(NarrowTrigger, ToStringCoversAll)
{
  EXPECT_STREQ(toString(NarrowVerdict::kNone), "NONE");
  EXPECT_STREQ(toString(NarrowVerdict::kNarrow), "NARROW");
  EXPECT_STREQ(toString(NarrowVerdict::kCenterLethal), "CENTER_LETHAL");
  EXPECT_STREQ(toString(NarrowVerdict::kPhysicallyBlocked), "PHYSICALLY_BLOCKED");
  EXPECT_STREQ(toString(NarrowVerdict::kNoPath), "NO_PATH");
}

// =====================================================================
// 有利朝向（决定能不能过去的那条几何）
// =====================================================================

TEST(FavorableYaw, OctagonRecursEvery45Degrees)
{
  // 八边形每 45° 复现，所以 0/45/90/135/180... 全都是零误差。
  for (int k = -8; k <= 8; ++k) {
    const double yaw = k * kOct;
    EXPECT_NEAR(favorableYawError(yaw, 0.0, kOct), 0.0, 1e-9) << "k=" << k;
  }
}

TEST(FavorableYaw, PicksNearestFavorableNotCorridorHeading)
{
  // 车体 40°、通道 0°：最近的有利朝向是 45°，所以误差应是 -5°，
  // **不是** -40°（那是"对齐通道方向"的错误做法，会绕远路）。
  const double err = favorableYawError(40.0 * kPi / 180.0, 0.0, kOct);
  EXPECT_NEAR(err, -5.0 * kPi / 180.0, 1e-6);
  EXPECT_LT(std::fabs(err), 10.0 * kPi / 180.0) << "转多了：应该就近对齐";
}

TEST(FavorableYaw, BoundedByHalfPeriod)
{
  // 误差必须落在 ±period/2 内，任何输入都不例外。
  for (int deg = -360; deg <= 360; deg += 3) {
    const double e = favorableYawError(deg * kPi / 180.0, 0.7, kOct);
    EXPECT_LE(std::fabs(e), (kOct / 2.0) + 1e-9) << "deg=" << deg;
  }
}

TEST(FavorableYaw, InvalidPeriodMeansNoCorrection)
{
  EXPECT_NEAR(favorableYawError(1.0, 0.0, 0.0), 0.0, 1e-12);
  EXPECT_NEAR(favorableYawError(1.0, 0.0, -1.0), 0.0, 1e-12);
}

// =====================================================================
// 横向扫描
// =====================================================================

TEST(LateralScan, FindsMinimumCostOffset)
{
  // 代价在横向 +0.03 处最低（通道方向 +x ⇒ 横向 +y）
  auto cost = [](double /*x*/, double y, double /*yaw*/) {
      return 250.0 + std::fabs(y - 0.03) * 100.0;
    };
  const auto r = scanLateral({0.0, 0.0}, 0.0, 0.0, 0.10, 0.01, cost);
  ASSERT_TRUE(r.valid);
  EXPECT_FALSE(r.saturated);
  EXPECT_NEAR(r.best_offset_m, 0.03, 0.011) << "没找到最低代价横向位置";
}

TEST(LateralScan, DetectsSaturationAndRefusesToChase)
{
  // 完全饱和（全 253）：必须报 saturated 且**不横移** ——
  // 这正是实测踩过的坑：在饱和区把 [253] 当成有梯度的量去追。
  auto cost = [](double, double, double) {return 253.0;};
  const auto r = scanLateral({0.0, 0.0}, 0.0, 0.0, 0.10, 0.01, cost);
  ASSERT_TRUE(r.valid);
  EXPECT_TRUE(r.saturated);
  EXPECT_NEAR(r.best_offset_m, 0.0, 1e-12) << "饱和时不该横移";
}

TEST(LateralScan, PrefersSmallerOffsetOnTie)
{
  // 两个等代价位置时取横移小的：无谓横移在窄通道里是纯风险。
  auto cost = [](double /*x*/, double y, double /*yaw*/) {
      return (std::fabs(std::fabs(y) - 0.05) < 1e-9) ? 100.0 : 200.0;
    };
  const auto r = scanLateral({0.0, 0.0}, 0.0, 0.0, 0.10, 0.01, cost);
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(std::fabs(r.best_offset_m), 0.05, 0.011);
}

TEST(LateralScan, StepFinerThanGridResolution)
{
  // 本档目标区间只有 0.032m(0.42-0.388)，栅格是 0.05m。
  // 用 0.01 步长必须能在 0.032 内产生多个采样点，否则本层没有分辨力。
  auto cost = [](double, double, double) {return 253.0;};
  const auto r = scanLateral({0.0, 0.0}, 0.0, 0.0, 0.032, 0.01, cost);
  ASSERT_TRUE(r.valid);
  EXPECT_GE(r.usable_count, 5U) << "0.032m 内采样点太少，分辨不出这一档";
}

TEST(LateralScan, RejectsBadArguments)
{
  auto cost = [](double, double, double) {return 0.0;};
  EXPECT_FALSE(scanLateral({0, 0}, 0, 0, -1.0, 0.01, cost).valid);
  EXPECT_FALSE(scanLateral({0, 0}, 0, 0, 0.1, 0.0, cost).valid);
  EXPECT_FALSE(scanLateral({0, 0}, 0, 0, 0.1, 0.01, nullptr).valid);
}

// =====================================================================
// 速度律
// =====================================================================

TEST(NarrowVel, HoldsForwardMotionUntilYawAligned)
{
  // 朝向没对齐时**不许前进** —— 这是本层最关键的一条。
  NarrowLimits lim;
  const auto cmd = narrowVelocity(0.0, 0.0, 0.0, 0.30, lim);   // 误差 0.30 > gate 0.12
  EXPECT_TRUE(cmd.holding_for_yaw);
  EXPECT_NEAR(cmd.vx, 0.0, 1e-12);
  EXPECT_NEAR(cmd.vy, 0.0, 1e-12);
  EXPECT_GT(std::fabs(cmd.wz), 0.0) << "应该在转";
}

TEST(NarrowVel, MovesOnceYawWithinGate)
{
  NarrowLimits lim;
  const auto cmd = narrowVelocity(0.0, 0.0, 0.0, 0.05, lim);
  EXPECT_FALSE(cmd.holding_for_yaw);
  EXPECT_NEAR(cmd.vx, lim.v_along, 1e-9);
}

TEST(NarrowVel, ObeysAllLimits)
{
  NarrowLimits lim;
  // 巨大的横向偏移与朝向误差，全部必须被限幅
  const auto cmd = narrowVelocity(0.0, 0.0, 100.0, 0.05, lim);
  EXPECT_LE(std::fabs(cmd.wz), lim.wz_max + 1e-9);
  // 车体系 vy 就是横向修正，必须被 v_lateral_max 限住
  EXPECT_LE(std::fabs(cmd.vy), lim.v_lateral_max + 1e-9);
}

TEST(NarrowVel, WzOpposesYawError)
{
  NarrowLimits lim;
  EXPECT_LT(narrowVelocity(0.0, 0.0, 0.0, 0.30, lim).wz, 0.0) << "正误差应负向修正";
  EXPECT_GT(narrowVelocity(0.0, 0.0, 0.0, -0.30, lim).wz, 0.0) << "负误差应正向修正";
}

TEST(NarrowVel, BodyFrameConversionIsApplied)
{
  // 车头朝 +y(yaw=pi/2)，通道方向 +x ⇒ 车体系里前进方向是 -y
  NarrowLimits lim;
  const auto cmd = narrowVelocity(kPi / 2.0, 0.0, 0.0, 0.0, lim);
  EXPECT_NEAR(cmd.vx, 0.0, 1e-9);
  EXPECT_NEAR(cmd.vy, -lim.v_along, 1e-9);
}

// =====================================================================
// 通道方向估计 / 退出判据
// =====================================================================

TEST(CorridorHeading, EstimatesFromLookahead)
{
  std::vector<PlanarPoint> path;
  for (int i = 0; i <= 20; ++i) {
    path.push_back({i * 0.05, 0.0});
  }
  double h = -99.0;
  ASSERT_TRUE(corridorHeadingFromPath(path, {0.0, 0.0}, 0.30, h));
  EXPECT_NEAR(h, 0.0, 1e-9);
}

TEST(CorridorHeading, FailsExplicitlyOnDegeneratePath)
{
  // 失败时不返回 0 冒充成功 —— 调用方必须能区分。
  double h = -99.0;
  EXPECT_FALSE(corridorHeadingFromPath({}, {0, 0}, 0.3, h));
  EXPECT_FALSE(corridorHeadingFromPath({{0, 0}}, {0, 0}, 0.3, h));
  std::vector<PlanarPoint> same{{1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0}};
  EXPECT_FALSE(corridorHeadingFromPath(same, {1.0, 1.0}, 0.3, h));
  EXPECT_DOUBLE_EQ(h, -99.0) << "失败时不得改写输出";
}

TEST(NarrowClearedRule, RequiresConsecutiveTicks)
{
  EXPECT_FALSE(narrowCleared(0, 3));
  EXPECT_FALSE(narrowCleared(2, 3));
  EXPECT_TRUE(narrowCleared(3, 3));
}

TEST(NarrowClearedRule, NonPositiveNeedNeverClears)
{
  // need<=0 按「永不脱离」处理：若按「立刻脱离」，接管会一拍就退出，
  // 等于本层从未生效且不报错。
  EXPECT_FALSE(narrowCleared(999, 0));
  EXPECT_FALSE(narrowCleared(999, -1));
}

// =====================================================================
// 连续失败计数与放弃判据
//
// 这两条是**跑机实测抓出来的真实缺陷**，不是假想用例：
// 13 次接管里 10 次正常穿过，却仍触发 2 次「判定该路径不可行」，
// 因为计数把成功穿越也算进了上限。
// =====================================================================

TEST(FailureCount, SuccessfulTraversalResetsTheCount)
{
  int n = 0;
  // 连续三次穿过：计数必须一直是 0，绝不能累积到放弃上限。
  for (int i = 0; i < 3; ++i) {
    updateNarrowFailureCount(false, n);          // 进入接管
    EXPECT_EQ(n, 1) << "进入接管应先按失败计一次";
    updateNarrowFailureCount(true, n);           // 穿过去了
    EXPECT_EQ(n, 0) << "穿过去必须清零 —— 否则沿途多个窄处的长路径会被误判不可行";
  }
  EXPECT_FALSE(shouldGiveUpNarrow(n, 3, false));
}

TEST(FailureCount, LongPathThroughManyNarrowSpotsIsNeverRejected)
{
  // 回归本尊：一条沿途有 8 个窄处、每个都成功穿过的路径。
  // 修复前这条在第 4 个窄处就被判「不可行」。
  int n = 0;
  for (int spot = 0; spot < 8; ++spot) {
    updateNarrowFailureCount(false, n);
    ASSERT_FALSE(shouldGiveUpNarrow(n, 3, false))
      << "第 " << (spot + 1) << " 个窄处就放弃了 —— 长路径经过多个窄处是常态";
    updateNarrowFailureCount(true, n);
  }
}

TEST(FailureCount, ConsecutiveFailuresDoReachTheLimit)
{
  // 反向：连续失败（从不清零）必须真的触发放弃，否则红线「不能无限循环脱困」失守。
  int n = 0;
  updateNarrowFailureCount(false, n);
  EXPECT_FALSE(shouldGiveUpNarrow(n, 3, false));
  updateNarrowFailureCount(false, n);
  EXPECT_FALSE(shouldGiveUpNarrow(n, 3, false));
  updateNarrowFailureCount(false, n);
  EXPECT_TRUE(shouldGiveUpNarrow(n, 3, false)) << "连续 3 次未穿过必须放弃";
}

TEST(FailureCount, GiveUpFiresExactlyOnce)
{
  // already_gave_up 之后必须恒为 false。实测未加这道闸时，
  // 每次放弃都以 20Hz 刷了 15 条同样的 WARN。
  EXPECT_TRUE(shouldGiveUpNarrow(5, 3, false));
  EXPECT_FALSE(shouldGiveUpNarrow(5, 3, true));
  EXPECT_FALSE(shouldGiveUpNarrow(999, 1, true));
}

TEST(FailureCount, IllegalMaxAttemptsFailsTowardGivingUp)
{
  // max_attempts < 1 是配置错误。必须按「立刻放弃」处理 ——
  // 按「永不放弃」处理会让红线彻底失守（无限贴边）。
  EXPECT_TRUE(shouldGiveUpNarrow(0, 0, false));
  EXPECT_TRUE(shouldGiveUpNarrow(0, -1, false));
}

TEST(FailureCount, NegativeCountIsSanitized)
{
  int n = -5;
  updateNarrowFailureCount(false, n);
  EXPECT_EQ(n, 1) << "负值应被归零后再自增，而不是继续往上爬到 -4";
}

// =====================================================================
// 卡住判据：按进展判，不按时长判
//
// 这一组守的是一个**实测缺陷**：narrow_timeout 25s x v_along 0.10m/s
// = 给窄通道设了 2.5 米长度上限。一条 2.5m 的健康通行被连砍 3 次，
// 期间机器人全程 0.10m/s 前进、朝向误差 ±0.008rad、离路径最远 0.112m。
// =====================================================================

TEST(Stall, HealthyLongCorridorIsNeverJudgedStalled)
{
  // 回归本尊：0.10m/s 匀速走 40 秒（4 米），远超旧的 2.5m 上限。
  // 一次都不许判卡住。剩余弧长从 4.0 单调减到 0。
  NarrowProgressState st;
  const double v = 0.10;
  const double total = 4.0;
  for (int i = 0; i <= 400; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    const double remaining = total - (v * t);
    EXPECT_FALSE(narrowStalled(remaining, t, 0.05, 6.0, st))
      << "t=" << t << "s 剩余=" << remaining << "m 被误判为原地蹭 —— "
      << "这正是「走得久」被当成「卡住」的那个缺陷";
  }
  EXPECT_NEAR(st.best_remaining_m, 0.0, 0.06);
}

TEST(Stall, TrulyStuckIsDetected)
{
  // 反向：弧长完全不涨（原地蹭）必须在 stall_timeout 之后判出。
  // 这才是「无限循环脱困」的真实特征。
  NarrowProgressState st;
  ASSERT_FALSE(narrowStalled(1.0, 0.0, 0.05, 6.0, st));      // 初始化
  EXPECT_FALSE(narrowStalled(1.0, 3.0, 0.05, 6.0, st));      // 3s 未超
  EXPECT_FALSE(narrowStalled(1.0, 6.0, 0.05, 6.0, st));      // 6s 恰好不超
  EXPECT_TRUE(narrowStalled(1.0, 6.01, 0.05, 6.0, st)) << "原地蹭 6s 后必须判卡住";
}

TEST(Stall, NoiseSizedJitterDoesNotCountAsProgress)
{
  // 真正的噪声是**围绕定值抖动**，不是每拍稳定累加。
  // （本测试最初写成「每拍 +1mm」，那其实是 0.01m/s 的真实蠕行、
  //  6s 窗口内累计 0.06m > min_gain 0.05m，本就该算进展 —— 是测试写错了。）
  NarrowProgressState st;
  ASSERT_FALSE(narrowStalled(1.0, 0.0, 0.05, 6.0, st));
  bool stalled = false;
  for (int i = 1; i <= 100; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    // ±2mm 抖动，均值不变
    const double jitter = ((i % 2) == 0) ? 0.002 : -0.002;
    stalled = narrowStalled(1.0 + jitter, t, 0.05, 6.0, st);
  }
  EXPECT_TRUE(stalled) << "毫米级抖动被当成了进展 —— 卡住判据形同虚设";
  EXPECT_NEAR(st.best_remaining_m, 1.0, 0.003) << "best 不该被抖动拉低";
}

TEST(Stall, ThresholdImpliesMinimumTolerableSpeed)
{
  // 判据 min_gain/stall_timeout 隐含一个「最低容忍速度」：
  //   0.05m / 6.0s = 0.0083 m/s
  // 比这更慢的推进会被判卡住。这是一个**设计阈值**，写成测试是为了
  // 让它显式可见 —— 改任一参数都会改变这个隐含速度。
  const double min_gain = 0.05;
  const double window = 6.0;
  const double implied = min_gain / window;
  EXPECT_NEAR(implied, 0.00833, 1e-4);

  // 略快于隐含速度：不判卡住
  NarrowProgressState fast;
  ASSERT_FALSE(narrowStalled(100.0, 0.0, min_gain, window, fast));
  bool fast_stalled = false;
  for (int i = 1; i <= 200; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    fast_stalled = narrowStalled(100.0 - (implied * 1.5 * t), t, min_gain, window, fast);
  }
  EXPECT_FALSE(fast_stalled);

  // 明显慢于隐含速度：判卡住
  NarrowProgressState slow;
  ASSERT_FALSE(narrowStalled(100.0, 0.0, min_gain, window, slow));
  bool slow_stalled = false;
  for (int i = 1; i <= 200; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    slow_stalled = narrowStalled(100.0 - (implied * 0.3 * t), t, min_gain, window, slow);
  }
  EXPECT_TRUE(slow_stalled);
}

TEST(Stall, BackwardProgressDoesNotRefreshTheTimer)
{
  // 被推着往后退不能算进展：后退 ⇒ 剩余弧长**变大** ⇒ best 不该被刷新。
  NarrowProgressState st;
  ASSERT_FALSE(narrowStalled(1.0, 0.0, 0.05, 6.0, st));
  EXPECT_FALSE(narrowStalled(2.0, 3.0, 0.05, 6.0, st));
  EXPECT_TRUE(narrowStalled(2.5, 6.5, 0.05, 6.0, st)) << "一路后退却没判卡住";
  EXPECT_DOUBLE_EQ(st.best_remaining_m, 1.0) << "best 只减不增";
}

TEST(Stall, IllegalTimeoutFailsTowardDetectingStall)
{
  // <=0 是配置错误。必须按「立刻判卡住」处理 ——
  // 按「永不判」处理会让「不能无限循环脱困」这条红线彻底失守。
  NarrowProgressState st;
  ASSERT_FALSE(narrowStalled(1.0, 0.0, 0.05, 0.0, st));      // 初始化那一拍
  EXPECT_TRUE(narrowStalled(1.0, 0.1, 0.05, 0.0, st));
  NarrowProgressState st2;
  ASSERT_FALSE(narrowStalled(1.0, 0.0, 0.05, -1.0, st2));
  EXPECT_TRUE(narrowStalled(1.0, 0.1, 0.05, -1.0, st2));
}

TEST(ArcProgress, MeasuresAlongPathNotStraightLine)
{
  // 弧长进度必须沿路径累计。用直线距离会被贴边横移污染。
  const std::vector<PlanarPoint> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};
  double p = 0.0;
  ASSERT_TRUE(pathArcProgress(path, {0.0, 0.0}, p));
  EXPECT_NEAR(p, 0.0, 1e-9);
  ASSERT_TRUE(pathArcProgress(path, {1.5, 0.0}, p));
  EXPECT_NEAR(p, 1.5, 1e-9);
  ASSERT_TRUE(pathArcProgress(path, {3.0, 0.0}, p));
  EXPECT_NEAR(p, 3.0, 1e-9);
}

TEST(ArcProgress, LateralOffsetDoesNotChangeProgress)
{
  // 这是选弧长而不是直线距离的全部理由：横移 0.2m 不产生假进展。
  const std::vector<PlanarPoint> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  double on_axis = 0.0;
  double offset = 0.0;
  ASSERT_TRUE(pathArcProgress(path, {1.0, 0.0}, on_axis));
  ASSERT_TRUE(pathArcProgress(path, {1.0, 0.2}, offset));
  EXPECT_NEAR(on_axis, offset, 1e-9) << "横移改变了弧长进度 —— 贴边会产生假进展";
}

TEST(ArcProgress, DegeneratePathReportsFailure)
{
  // 退化路径**不返回 0 冒充成功**：调用方必须能区分。
  double p = -1.0;
  EXPECT_FALSE(pathArcProgress({}, {0.0, 0.0}, p));
  EXPECT_FALSE(pathArcProgress({{1.0, 1.0}}, {0.0, 0.0}, p));
}

// =====================================================================
// 剩余弧长判据的回归：v4 五轮里两个实测误杀事件，各一条
//
// 两次事件里机器人**都在被指令以 0.10m/s 前进**（日志 指令=(0.076,0.065)
// 与 (0.099,-0.011)），却被判"原地蹭"。根因是旧判据用"从路径起点累计的
// 弧长"，而 replan_policy: on_invalid 会换路径、把这个量的原点挪到机器人
// 脚下。改用剩余弧长后必须不再复现。
// =====================================================================

TEST(RemainingArc, ReplanDoesNotResetTheMetric)
{
  // 事件 A 复现：进度涨到 1.5037m 后换了路径。
  // 旧判据下新路径从 ≈0 起算、永远超不过高水位 1.55 → 6s 后误杀。
  // 剩余弧长对"同一目标的重规划"不敏感，所以必须继续判为有进展。
  const std::vector<PlanarPoint> before{{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}};
  const std::vector<PlanarPoint> after{{1.5, 0.0}, {2.5, 0.0}, {4.0, 0.0}};  // 重规划，同终点

  double rem_before = 0.0;
  double rem_after = 0.0;
  const PlanarPoint robot{1.5, 0.0};
  ASSERT_TRUE(pathRemainingArc(before, robot, rem_before));
  ASSERT_TRUE(pathRemainingArc(after, robot, rem_after));
  EXPECT_NEAR(rem_before, rem_after, 1e-9)
    << "换路径让剩余弧长跳了 —— 那它和旧的累计弧长一样不能用";

  // 对照：旧的累计弧长在同一场景下**确实**会跳（这是缺陷的证明）
  double arc_before = 0.0;
  double arc_after = 0.0;
  ASSERT_TRUE(pathArcProgress(before, robot, arc_before));
  ASSERT_TRUE(pathArcProgress(after, robot, arc_after));
  EXPECT_GT(std::fabs(arc_before - arc_after), 1.0)
    << "累计弧长本该跳变；不跳说明这个对照用例没有复现出缺陷";
}

TEST(RemainingArc, MovingRobotIsNeverJudgedStalledAcrossAReplan)
{
  // 端到端：机器人 0.10m/s 前进，中途换一次路径（同终点），全程不许判卡住。
  NarrowProgressState st;
  PlanarPoint last_end{};
  std::size_t last_size = 0U;
  bool ever_stalled = false;
  for (int i = 0; i <= 300; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    const double x = 0.10 * t;                       // 沿 x 前进
    // 第 150 拍换路径：起点挪到机器人脚下，终点不变
    const std::vector<PlanarPoint> path = (i < 150)
      ? std::vector<PlanarPoint>{{0.0, 0.0}, {15.0, 0.0}, {30.0, 0.0}}
      : std::vector<PlanarPoint>{{x, 0.0}, {20.0, 0.0}, {30.0, 0.0}};
    if (pathWasReplaced(path, last_end, last_size)) {
      st = NarrowProgressState{};
    }
    double rem = 0.0;
    ASSERT_TRUE(pathRemainingArc(path, PlanarPoint{x, 0.0}, rem));
    if (narrowStalled(rem, t, 0.05, 6.0, st)) {
      ever_stalled = true;
    }
  }
  EXPECT_FALSE(ever_stalled) << "一个持续前进的机器人跨越一次重规划就被误杀";
}

TEST(RemainingArc, GoalSwitchWhileMovingIsNotAStall)
{
  // 这一条才真正咬住"不重开窗口"的后果：机器人一直在前进，但中途**换了目标**
  // （新目标更远 ⇒ 剩余弧长整体跳大）。不重开窗口时，跳大之后永远超不过
  // 旧高水位，6s 后必然误杀 —— 而机器人从头到尾都在动。
  //
  // 探索协调器就是这么跑的：抵达一个前沿点后立刻派下一个更远的点。
  NarrowProgressState st;
  PlanarPoint last_end{};
  std::size_t last_size = 0U;
  bool ever_stalled = false;
  for (int i = 0; i <= 300; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    const double x = 0.10 * t;
    // 第 100 拍换到一个远得多的目标
    const std::vector<PlanarPoint> path = (i < 100)
      ? std::vector<PlanarPoint>{{0.0, 0.0}, {5.0, 0.0}}
      : std::vector<PlanarPoint>{{0.0, 0.0}, {50.0, 0.0}};
    if (pathWasReplaced(path, last_end, last_size)) {
      st = NarrowProgressState{};
    }
    double rem = 0.0;
    ASSERT_TRUE(pathRemainingArc(path, PlanarPoint{x, 0.0}, rem));
    if (narrowStalled(rem, t, 0.05, 6.0, st)) {
      ever_stalled = true;
    }
  }
  EXPECT_FALSE(ever_stalled)
    << "换了更远的目标之后，一个持续前进的机器人被判成原地蹭";
}

TEST(RemainingArc, StillDetectsARobotThatReallyStopped)
{
  // 反向：路径不变、机器人不动 ⇒ 必须照常判卡住。修完不能把红线弄丢。
  const std::vector<PlanarPoint> path{{0.0, 0.0}, {5.0, 0.0}};
  NarrowProgressState st;
  PlanarPoint last_end{};
  std::size_t last_size = 0U;
  bool stalled = false;
  for (int i = 0; i <= 100; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    (void)pathWasReplaced(path, last_end, last_size);
    double rem = 0.0;
    ASSERT_TRUE(pathRemainingArc(path, PlanarPoint{1.0, 0.0}, rem));  // 位置恒定
    stalled = narrowStalled(rem, t, 0.05, 6.0, st);
  }
  EXPECT_TRUE(stalled) << "真的停住了却没判出来 —— 红线失守";
}

TEST(RemainingArc, GoalChangeReopensTheWindow)
{
  // 换**目标**（终点变远）⇒ 剩余弧长整体跳大 ⇒ 必须重开窗口，
  // 否则永远超不过旧高水位 → 误杀。
  PlanarPoint last_end{};
  std::size_t last_size = 0U;
  const std::vector<PlanarPoint> near_goal{{0.0, 0.0}, {2.0, 0.0}};
  const std::vector<PlanarPoint> far_goal{{0.0, 0.0}, {20.0, 0.0}};
  EXPECT_FALSE(pathWasReplaced(near_goal, last_end, last_size)) << "首次不算换";
  EXPECT_TRUE(pathWasReplaced(far_goal, last_end, last_size)) << "终点挪远必须识别为换了";
}

TEST(RemainingArc, VertexJitterIsNotTreatedAsAReplan)
{
  // 重规划带来的亚栅格顶点抖动**不能**算换路径 ——
  // 每拍都重开窗口等于把卡住判据整个关掉。
  PlanarPoint last_end{};
  std::size_t last_size = 0U;
  const std::vector<PlanarPoint> a{{0.0, 0.0}, {5.0, 0.0}};
  ASSERT_FALSE(pathWasReplaced(a, last_end, last_size));
  const std::vector<PlanarPoint> b{{0.0, 0.0}, {5.02, 0.01}};   // 终点抖 2cm
  EXPECT_FALSE(pathWasReplaced(b, last_end, last_size))
    << "2cm 抖动被当成换路径 —— 卡住判据会被每拍重开而失效";
}

TEST(RemainingArc, LateralOffsetDoesNotFakeProgress)
{
  // 贴边通行会横移。横移不得让剩余弧长变化（否则横着挪就能刷进展）。
  const std::vector<PlanarPoint> path{{0.0, 0.0}, {10.0, 0.0}};
  double on_path = 0.0;
  double off_path = 0.0;
  ASSERT_TRUE(pathRemainingArc(path, PlanarPoint{3.0, 0.0}, on_path));
  ASSERT_TRUE(pathRemainingArc(path, PlanarPoint{3.0, 0.25}, off_path));
  EXPECT_NEAR(on_path, off_path, 1e-9) << "横移刷出了假进展";
}

TEST(RemainingArc, DegeneratePathReportsFailure)
{
  double rem = 0.0;
  EXPECT_FALSE(pathRemainingArc({}, PlanarPoint{0.0, 0.0}, rem));
  EXPECT_FALSE(pathRemainingArc({{1.0, 1.0}}, PlanarPoint{0.0, 0.0}, rem));
}

TEST(RemainingArc, IsZeroAtTheGoalAndFullAtTheStart)
{
  const std::vector<PlanarPoint> path{{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};  // 全长 7
  double rem = 0.0;
  ASSERT_TRUE(pathRemainingArc(path, PlanarPoint{0.0, 0.0}, rem));
  EXPECT_NEAR(rem, 7.0, 1e-9);
  ASSERT_TRUE(pathRemainingArc(path, PlanarPoint{3.0, 4.0}, rem));
  EXPECT_NEAR(rem, 0.0, 1e-9);
}

TEST(RemainingArc, DecreasesMonotonicallyThroughASharpBend)
{
  // 折返/急弯是旧判据停止增长的地方（v4 实测通道方向从 -0.13 跳到 -2.90）。
  // 剩余弧长在同样的路径上必须单调减。
  const std::vector<PlanarPoint> path{{0.0, 0.0}, {2.0, 0.0}, {2.0, 0.1}, {0.0, 0.1}};
  double prev = 1e9;
  for (int i = 0; i <= 20; ++i) {
    const double s = 0.1 * static_cast<double>(i);   // 沿第一段走
    double rem = 0.0;
    ASSERT_TRUE(pathRemainingArc(path, PlanarPoint{s, 0.0}, rem));
    EXPECT_LT(rem, prev + 1e-9) << "s=" << s << " 处剩余弧长变大了";
    prev = rem;
  }
}

// =====================================================================
// 朝向闸门：两处判据必须是同一个函数
// =====================================================================

TEST(YawGate, HoldsExactlyOutsideTheGate)
{
  EXPECT_FALSE(yawGateHolding(0.0, 0.12));
  EXPECT_FALSE(yawGateHolding(0.12, 0.12)) << "恰好等于门限不算超出";
  EXPECT_TRUE(yawGateHolding(0.121, 0.12));
  EXPECT_TRUE(yawGateHolding(-0.121, 0.12)) << "闸门必须对称";
}

TEST(YawGate, NarrowVelocityAgreesWithTheSharedPredicate)
{
  // 防漂：narrowVelocity 里的 holding_for_yaw 必须与 yawGateHolding 完全一致。
  // 这两处曾各写一遍 fabs(yaw_error) > gate，靠注释约定"逐字一致"。
  NarrowLimits lim;
  lim.yaw_gate_rad = 0.12;
  lim.wz_max = 0.2;
  lim.kp_yaw = 1.5;
  lim.v_along = 0.10;
  lim.v_lateral_max = 0.05;
  lim.kp_lateral = 1.0;
  for (int i = -40; i <= 40; ++i) {
    const double ye = 0.01 * static_cast<double>(i);
    const NarrowCommand c = narrowVelocity(0.0, 0.0, 0.0, ye, lim);
    EXPECT_EQ(c.holding_for_yaw, yawGateHolding(ye, lim.yaw_gate_rad))
      << "yaw_error=" << ye << " 两处判据不一致";
    if (c.holding_for_yaw) {
      EXPECT_DOUBLE_EQ(c.vx, 0.0);
      EXPECT_DOUBLE_EQ(c.vy, 0.0);
    }
  }
}

// =====================================================================
// 进入前的朝向预对齐
// =====================================================================

namespace
{
/// 一条沿 +x 的直线路径，点距 0.05m（与栅格同量级，接近真实规划输出）。
std::vector<PlanarPoint> straightPathX(double length_m)
{
  std::vector<PlanarPoint> p;
  for (double s = 0.0; s <= length_m + 1e-9; s += 0.05) {
    p.push_back(PlanarPoint{s, 0.0});
  }
  return p;
}

/// 造一个「只有把八边形平边正对通道壁才过得去」的窄处：
/// x >= gate_x 处，足迹代价取决于朝向与 45° 同余类的接近程度。
/// 朝向落在同余类附近(误差 < tol) ⇒ 200（可通行）；否则 254（足迹压到障碍本体）。
/// ⚠️ 用 254 而不是 253：这些 cost_fn 模拟的是 footprintCostAtPose（多边形查询），
///    而多边形查询的"过不去"就是 254。用 253 会把重复计底盘半宽的旧语义写进测试。
FootprintCostFn gateNeedsFavorableYaw(double gate_x, double tol)
{
  return [gate_x, tol](double x, double /*y*/, double yaw) {
      if (x < gate_x) {
        return 0.0;                        // 闸门之前一律开阔
      }
      const double err = favorableYawError(yaw, 0.0, kOct);
      return (std::fabs(err) < tol) ? 200.0 : 254.0;
    };
}

PrealignConfig defaultPrealignCfg()
{
  PrealignConfig cfg;
  cfg.preview_m = 1.00;
  cfg.sample_step_m = 0.10;
  cfg.tangent_lookahead_m = 0.40;
  cfg.favorable_period_rad = kOct;
  cfg.footprint_lethal_threshold = NarrowCostValues::kLethal;
  return cfg;
}
}  // namespace

TEST(Prealign, DetectsNarrowPassableOnlyWhenAligned)
{
  // 机器人朝向 22.5°(=半周期)，正是最不利朝向：与任何 45° 同余类都差 22.5°。
  const double bad_yaw = kOct * 0.5;
  const PrealignPreview r = previewFavorableAlignment(
    straightPathX(3.0), PlanarPoint{0.0, 0.0}, bad_yaw,
    defaultPrealignCfg(), gateNeedsFavorableYaw(0.5, 0.05));

  ASSERT_EQ(r.verdict, PrealignVerdict::kNeeded)
    << "当前朝向过不去、转正过得去 ⇒ 必须判需要预对齐";
  // 目标朝向必须落在 45° 同余类上（本例最近的是 0 或 45°）。
  EXPECT_LT(std::fabs(favorableYawError(r.target_yaw, 0.0, kOct)), 1e-9)
    << "target_yaw=" << r.target_yaw << " 不在有利同余类上";
  // 触发点应当在闸门附近，而不是贴着机器人或跑到前视尽头。
  EXPECT_GE(r.at_distance_m, 0.5);
  EXPECT_LE(r.at_distance_m, 0.65);
}

TEST(Prealign, ClearAheadReportsNonZeroSamples)
{
  // 🔴 这条防的是「扫了 0 个点也报开阔」这种静默假阴性。
  const PrealignPreview r = previewFavorableAlignment(
    straightPathX(3.0), PlanarPoint{0.0, 0.0}, 0.3, defaultPrealignCfg(),
    [](double, double, double) {return 0.0;});

  EXPECT_EQ(r.verdict, PrealignVerdict::kClearAhead);
  EXPECT_GT(r.samples, 0U) << "报开阔却一个点都没扫 ⇒ 与故障无法区分";
  EXPECT_EQ(r.skipped_no_tangent, 0U);
}

TEST(Prealign, TrulyNarrowIsNotOurJob)
{
  // 两个朝向都过不去 ⇒ 该交红线/ESCAPE，不能报 kNeeded 让机器人白转一场。
  const PrealignPreview r = previewFavorableAlignment(
    straightPathX(3.0), PlanarPoint{0.0, 0.0}, 0.0, defaultPrealignCfg(),
    [](double x, double, double) {return x < 0.5 ? 0.0 : 254.0;});

  EXPECT_EQ(r.verdict, PrealignVerdict::kBlockedEvenFavorable)
    << "必须与 kClearAhead 区分开，否则'预对齐从不触发'看起来像'一路开阔'";
  EXPECT_NE(r.verdict, PrealignVerdict::kNeeded);
}

TEST(Prealign, PathTooShortIsNotClearAhead)
{
  std::vector<PlanarPoint> tiny{{0.0, 0.0}, {0.01, 0.0}};
  const PrealignPreview r = previewFavorableAlignment(
    tiny, PlanarPoint{0.0, 0.0}, 0.0, defaultPrealignCfg(),
    [](double, double, double) {return 0.0;});
  EXPECT_EQ(r.verdict, PrealignVerdict::kPathTooShort)
    << "缺数据不等于开阔";
}

TEST(Prealign, RejectsBadConfig)
{
  const auto free_fn = [](double, double, double) {return 0.0;};
  const auto path = straightPathX(3.0);
  const PlanarPoint origin{0.0, 0.0};

  PrealignConfig bad = defaultPrealignCfg();
  bad.sample_step_m = 0.0;
  EXPECT_EQ(
    previewFavorableAlignment(path, origin, 0.0, bad, free_fn).verdict,
    PrealignVerdict::kInvalidConfig);

  bad = defaultPrealignCfg();
  bad.sample_step_m = 2.0;              // 步长 > 前视距离
  EXPECT_EQ(
    previewFavorableAlignment(path, origin, 0.0, bad, free_fn).verdict,
    PrealignVerdict::kInvalidConfig);

  bad = defaultPrealignCfg();
  bad.favorable_period_rad = -1.0;
  EXPECT_EQ(
    previewFavorableAlignment(path, origin, 0.0, bad, free_fn).verdict,
    PrealignVerdict::kInvalidConfig);
}

TEST(Prealign, TargetIsNearestFavorableNotCorridorHeading)
{
  // 通道方向 200°(3.49rad)。八边形每 45° 复现，所以不该绕远去凑 200°，
  // 转角必须 <= 半周期 22.5°。
  const double corridor = 3.4907;                  // 200 度
  const double robot_yaw = corridor + kOct * 0.5;  // 偏离半周期
  auto path = straightPathX(3.0);
  // 把路径旋到 200° 方向，保证切向就是 corridor。
  for (auto & p : path) {
    const double s = p.x;
    p.x = s * std::cos(corridor);
    p.y = s * std::sin(corridor);
  }
  const PrealignPreview r = previewFavorableAlignment(
    path, PlanarPoint{0.0, 0.0}, robot_yaw, defaultPrealignCfg(),
    [corridor](double x, double y, double yaw) {
      const double s = x * std::cos(corridor) + y * std::sin(corridor);
      if (s < 0.5) {return 0.0;}
      return std::fabs(favorableYawError(yaw, corridor, kOct)) < 0.05 ? 200.0 : 254.0;
    });

  ASSERT_EQ(r.verdict, PrealignVerdict::kNeeded);
  EXPECT_LE(std::fabs(normalizeAngle(r.target_yaw - robot_yaw)), kOct * 0.5 + 1e-6)
    << "转角超过半周期 ⇒ 绕远了";
}

TEST(Prealign, SweepBlockedByHardObstacleRefusesRotation)
{
  // 🔴 安全：旋转扫掠过程中撞真障碍(254) ⇒ 必须拒绝，不许硬转。
  double worst = 0.0;
  const bool ok = sweepClearForRotation(
    PlanarPoint{0.0, 0.0}, 0.0, kOct, 0.05, NarrowCostValues::kLethal,
    [](double, double, double yaw) {
      return (std::fabs(yaw - kOct * 0.5) < 0.06) ? 254.0 : 0.0;   // 中途一段是真障碍
    }, worst);

  EXPECT_FALSE(ok) << "扫掠途中有真障碍却放行旋转";
  EXPECT_GE(worst, 254.0) << "worst_cost 必须填，失败时也要能看出多糟";
}

TEST(Prealign, SweepClearWhenWholeArcIsFree)
{
  double worst = -1.0;
  EXPECT_TRUE(
    sweepClearForRotation(
      PlanarPoint{0.0, 0.0}, 0.0, kOct, 0.05, NarrowCostValues::kLethal,
      [](double, double, double) {return 100.0;}, worst));
  EXPECT_DOUBLE_EQ(worst, 100.0);
}

TEST(Prealign, SweepTakesShortestDirection)
{
  // from=-170°, to=+170° ⇒ 最近方向是跨 ±180°（20°），不是绕 340°。
  // 若走了长边，必然扫到 0° 附近那个"障碍"。
  const double from = -2.9671;    // -170 度
  const double to = 2.9671;       // +170 度
  double worst = 0.0;
  const bool ok = sweepClearForRotation(
    PlanarPoint{0.0, 0.0}, from, to, 0.05, NarrowCostValues::kLethal,
    [](double, double, double yaw) {
      return (std::fabs(yaw) < 1.0) ? 254.0 : 0.0;    // 0° 附近是障碍
    }, worst);
  EXPECT_TRUE(ok) << "走了长边（绕过 0°），worst=" << worst;
}

TEST(Prealign, SweepRejectsBadStep)
{
  double worst = 0.0;
  EXPECT_FALSE(
    sweepClearForRotation(
      PlanarPoint{0.0, 0.0}, 0.0, 1.0, 0.0, NarrowCostValues::kLethal,
      [](double, double, double) {return 0.0;}, worst))
    << "step<=0 必须拒绝旋转（失败偏安全侧），不能当成放行";
}

// =====================================================================
// 窄通道内临时缩小足迹（八边形 -> 正方形）的策略判定
// =====================================================================

namespace
{
/// 造一个「宽度介于两种足迹之间」的窄处：
/// x >= gate_x 处，默认(大)足迹任何朝向都过不去，小足迹只在有利朝向下过得去。
/// 这正是实测里占 12/18 的那一档（足迹代价=253，有利朝向也=253）。
FootprintCostFn bigAlwaysBlocked(double gate_x)
{
  return [gate_x](double x, double, double) {
      return x < gate_x ? 0.0 : 254.0;
    };
}
FootprintCostFn smallNeedsFavorableYaw(double gate_x, double tol)
{
  return [gate_x, tol](double x, double, double yaw) {
      if (x < gate_x) {return 0.0;}
      return std::fabs(favorableYawError(yaw, 0.0, kOct)) < tol ? 180.0 : 254.0;
    };
}
}  // namespace

TEST(Strategy, ShrinkOnlyWhenBigFootprintCannotPassEvenAligned)
{
  // 这是本功能存在的唯一理由：大足迹转正也过不去、小足迹转正过得去。
  const PrealignPreview ignored = previewFavorableAlignment(
    straightPathX(3.0), PlanarPoint{0.0, 0.0}, kOct * 0.5, defaultPrealignCfg(),
    bigAlwaysBlocked(0.5));
  ASSERT_EQ(ignored.verdict, PrealignVerdict::kBlockedEvenFavorable)
    << "前提检查：只给大足迹时这一档必须是「转正也过不去」";

  const StrategyPreview r = previewNarrowStrategy(
    straightPathX(3.0), PlanarPoint{0.0, 0.0}, kOct * 0.5, defaultPrealignCfg(),
    bigAlwaysBlocked(0.5), smallNeedsFavorableYaw(0.5, 0.05));

  ASSERT_EQ(r.strategy, NarrowStrategy::kAlignThenShrink);
  EXPECT_LT(std::fabs(favorableYawError(r.target_yaw, 0.0, kOct)), 1e-9)
    << "target_yaw 必须落在有利同余类上";
}

TEST(Strategy, NoShrinkWhenAligningAloneIsEnough)
{
  // 🔴 不无谓缩小足迹：转正就够时必须只预对齐。
  // 缩足迹会把代价地图的朝向盲区从 0.035 放大到 0.133，没必要就不付这个代价。
  const StrategyPreview r = previewNarrowStrategy(
    straightPathX(3.0), PlanarPoint{0.0, 0.0}, kOct * 0.5, defaultPrealignCfg(),
    gateNeedsFavorableYaw(0.5, 0.05),          // 大足迹转正就能过
    smallNeedsFavorableYaw(0.5, 0.05));

  EXPECT_EQ(r.strategy, NarrowStrategy::kAlignOnly);
  EXPECT_NE(r.strategy, NarrowStrategy::kAlignThenShrink);
}

TEST(Strategy, BothBlockedIsNotOurJob)
{
  const StrategyPreview r = previewNarrowStrategy(
    straightPathX(3.0), PlanarPoint{0.0, 0.0}, 0.0, defaultPrealignCfg(),
    bigAlwaysBlocked(0.5), bigAlwaysBlocked(0.5));   // 小足迹也一样堵
  EXPECT_EQ(r.strategy, NarrowStrategy::kBlocked)
    << "两种足迹都不行 ⇒ 交红线/ESCAPE，不能报要缩足迹让机器人白转一场";
}

TEST(Strategy, ClearAheadReportsNonZeroSamples)
{
  const StrategyPreview r = previewNarrowStrategy(
    straightPathX(3.0), PlanarPoint{0.0, 0.0}, 0.3, defaultPrealignCfg(),
    [](double, double, double) {return 0.0;},
    [](double, double, double) {return 0.0;});
  EXPECT_EQ(r.strategy, NarrowStrategy::kNone);
  EXPECT_GT(r.samples, 0U) << "报开阔却一个点都没扫 ⇒ 与故障无法区分";
}

TEST(Strategy, EmptyNarrowCostFnNeverAsksToShrink)
{
  // 一键回退语义：不提供小足迹取值函数时，绝不可能要求缩足迹。
  const StrategyPreview r = previewNarrowStrategy(
    straightPathX(3.0), PlanarPoint{0.0, 0.0}, 0.0, defaultPrealignCfg(),
    bigAlwaysBlocked(0.5), FootprintCostFn{});
  EXPECT_EQ(r.strategy, NarrowStrategy::kBlocked);
  EXPECT_NE(r.strategy, NarrowStrategy::kAlignThenShrink);
}

TEST(Strategy, PathTooShortIsNotClearAhead)
{
  std::vector<PlanarPoint> tiny{{0.0, 0.0}, {0.01, 0.0}};
  const StrategyPreview r = previewNarrowStrategy(
    tiny, PlanarPoint{0.0, 0.0}, 0.0, defaultPrealignCfg(),
    [](double, double, double) {return 0.0;},
    [](double, double, double) {return 0.0;});
  EXPECT_EQ(r.strategy, NarrowStrategy::kPathTooShort) << "缺数据不等于开阔";
}

TEST(Strategy, RejectsBadConfig)
{
  PrealignConfig bad = defaultPrealignCfg();
  bad.sample_step_m = 0.0;
  EXPECT_EQ(
    previewNarrowStrategy(
      straightPathX(3.0), PlanarPoint{0.0, 0.0}, 0.0, bad,
      [](double, double, double) {return 0.0;},
      [](double, double, double) {return 0.0;}).strategy,
    NarrowStrategy::kInvalidConfig);
}

// =====================================================================
// 缩足迹的**切出**判据（迟滞）
//
// 这一组测的不是新函数，而是控制器 evaluatePrealign 里那段切出探针的
// **判据本身**：
//     exit_cfg = 切入配置; exit_cfg.preview_m = exit_preview_m;
//     ep = previewNarrowStrategy(path, robot, yaw, exit_cfg, cost_big, {});
//     clear = (ep.strategy == kNone || ep.strategy == kPathTooShort);
//
// 为什么值得单独测：第一轮 A/B 实测 123 次切入 / 121 次复原 / 中位驻留 0.15s，
// 而 0.15s 恰好 = narrow_clear_ticks(3) / controller_frequency(20Hz)。
// 病因不在参数，而在**旧切出判据只看当前位姿**，而当前位姿的大足迹代价
// 必然低于阈值（否则窄通道接管层早接管了、根本走不到预对齐）——
// 于是切出判据在切入那一瞬间就已成立。下面 OldCriterionFiresAtOnce
// 就是把这个失效模式钉住的回归测试。
// =====================================================================

namespace
{
/// 一段**有限长**的窄处（门洞）：x ∈ [x0, x1) 内大足迹任何朝向都过不去。
/// 比半平面更贴近真实 —— 只有有限长的门洞才可能"走过去之后就清了"，
/// 而"能不能清"正是切出判据的全部内容。
FootprintCostFn blockedBand(double x0, double x1)
{
  return [x0, x1](double x, double, double) {
      return (x >= x0 && x < x1) ? 254.0 : 0.0;
    };
}

/// 与控制器逐字一致的切出判据。写成函数而不是在每个用例里重复，
/// 是为了让"测的判据"与"跑的判据"只有一份定义。
bool exitClear(
  const std::vector<PlanarPoint> & path, const PlanarPoint & robot, double yaw,
  double exit_preview_m, const FootprintCostFn & cost_big)
{
  PrealignConfig cfg = defaultPrealignCfg();
  cfg.preview_m = exit_preview_m;
  const StrategyPreview ep = previewNarrowStrategy(
    path, robot, yaw, cfg, cost_big, FootprintCostFn{});
  return ep.strategy == NarrowStrategy::kNone ||
         ep.strategy == NarrowStrategy::kPathTooShort;
}
}  // namespace

TEST(SquareExit, OldCriterionFiresAtOnce)
{
  // 🔴 回归测试：把 123 次切换那个失效模式直接钉住。
  // 门洞在前方 1.10m 处，机器人当前位姿完全开阔。
  const auto cost_big = blockedBand(1.10, 1.30);
  const auto path = straightPathX(3.0);

  // 旧判据（只看当前位姿）：立刻判"装得下" ⇒ 切入那一拍就要复原。
  EXPECT_LT(cost_big(0.0, 0.0, 0.0), NarrowCostValues::kLethal)
    << "前提：当前位姿必然不致命 —— 否则窄通道接管层早接管、走不到预对齐";

  // 新判据（前视 1.20m 窗口）：门洞还在窗口里 ⇒ 不许复原。
  EXPECT_FALSE(exitClear(path, PlanarPoint{0.0, 0.0}, 0.0, 1.20, cost_big))
    << "切出判据必须看前视窗口；只看当前位姿就是 0.15s 抖动的病根";
}

TEST(SquareExit, LongerExitWindowCreatesRealHysteresisBand)
{
  // 迟滞的定义：存在一段位置，切入判据说"开阔"而切出判据说"还没清"。
  // 那段就是迟滞带，宽度 = exit_preview - entry_preview = 0.20m = 4 个栅格。
  // （对比：八边形各向异性 0.032m < 一个栅格 0.05m，代价地图表达不出来，
  //   所以纯预对齐在这张图上测不出效果。）
  const auto cost_big = blockedBand(1.10, 1.30);
  const auto path = straightPathX(3.0);
  const PlanarPoint robot{0.0, 0.0};

  PrealignConfig entry = defaultPrealignCfg();      // preview 1.00m
  const StrategyPreview in = previewNarrowStrategy(
    path, robot, 0.0, entry, cost_big, FootprintCostFn{});
  EXPECT_EQ(in.strategy, NarrowStrategy::kNone)
    << "1.10m 处的门洞落在 1.00m 切入窗口之外，切入侧看不见它";

  EXPECT_FALSE(exitClear(path, robot, 0.0, 1.20, cost_big))
    << "1.20m 切出窗口看得见它 ⇒ 已生效的小足迹在这一段必须保持";
}

TEST(SquareExit, ExitWindowShorterThanEntryGuaranteesThrash)
{
  // 反过来配（切出窗口比切入窗口短）就**保证**振荡 —— 所以控制器有启动守卫
  // 直接拒绝这种配置。这个用例说明那条守卫拦的是真问题，不是洁癖。
  const auto cost_big = blockedBand(0.80, 1.00);
  const auto path = straightPathX(3.0);
  const PlanarPoint robot{0.0, 0.0};

  PrealignConfig entry = defaultPrealignCfg();      // 1.00m：看得见门洞
  EXPECT_EQ(
    previewNarrowStrategy(path, robot, 0.0, entry, cost_big, FootprintCostFn{}).strategy,
    NarrowStrategy::kBlocked);
  // 切出窗口 0.60m：看不见门洞 ⇒ 判"清了" ⇒ 与切入判据同时成立 ⇒ 切入-复原-切入…
  EXPECT_TRUE(exitClear(path, robot, 0.0, 0.60, cost_big))
    << "两个判据同时成立即为自激；启动守卫必须拒绝 exit < entry";
}

TEST(SquareExit, ClearsOnceGateIsBehind)
{
  // 必须真的会清 —— 否则"迟滞"只是把锁存换了个名字，每次都走硬超时。
  const auto cost_big = blockedBand(0.30, 0.50);
  const auto path = straightPathX(3.0);
  EXPECT_TRUE(exitClear(path, PlanarPoint{1.00, 0.0}, 0.0, 1.20, cost_big))
    << "门洞已在身后、前视窗口内全开阔 ⇒ 必须判清，否则只能等硬超时";
}

TEST(SquareExit, NeedingRotationIsNotClear)
{
  // 🔴 刻意的保守选择：大足迹"转个朝向才过得去"(kAlignOnly)**不算**装得下。
  // 此刻朝向锁在 square_locked_yaw_ 上，按 kAlignOnly 复原可能直接落进 253 带。
  const auto path = straightPathX(3.0);
  const auto need_yaw = gateNeedsFavorableYaw(0.50, 0.05);
  const double bad_yaw = kOct * 0.5;                 // 最不利朝向

  PrealignConfig cfg = defaultPrealignCfg();
  cfg.preview_m = 1.20;
  EXPECT_EQ(
    previewNarrowStrategy(path, PlanarPoint{0.0, 0.0}, bad_yaw, cfg, need_yaw,
      FootprintCostFn{}).strategy,
    NarrowStrategy::kAlignOnly) << "前提检查：这一档确实是「转正就能过」";
  EXPECT_FALSE(exitClear(path, PlanarPoint{0.0, 0.0}, bad_yaw, 1.20, need_yaw))
    << "kAlignOnly 不算装得下 —— 复原后可能立刻落在膨胀带里";
}

TEST(SquareExit, ProbeCanNeverAskToShrink)
{
  // 切出探针故意把第二个 cost_fn 传空：它只问"大足迹过不过得去"。
  // 若哪天有人手滑把小足迹的取值函数传进去，kAlignThenShrink 会被
  // 当成"不清"永远卡着 —— 这个用例把"传空"这个约定钉住。
  const auto cost_big = blockedBand(0.50, 0.70);
  const StrategyPreview ep = previewNarrowStrategy(
    straightPathX(3.0), PlanarPoint{0.0, 0.0}, 0.0, defaultPrealignCfg(),
    cost_big, FootprintCostFn{});
  EXPECT_NE(ep.strategy, NarrowStrategy::kAlignThenShrink);
  EXPECT_EQ(ep.strategy, NarrowStrategy::kBlocked);
}

// =====================================================================
// 🔴 重复计底盘半宽：用一条**已知宽度**的真通道把它量出来
//
// 这一组不是逻辑测试，是**算术回归**。上一版把足迹多边形的阈值设成 253，
// 于是本仿真环境里每一条 0.65m 级通道都被判"连小足迹转正也过不去"
//（实测 909 次），而真实读数是 51 次 253 / 仅 2 次 254 —— 从未真的撞上。
//
// 下面用带膨胀的一维代价场复现整条因果链，并把两个阈值下的结论都测出来。
// =====================================================================

namespace
{
constexpr double kRes = 0.05;          // 与 local/global costmap 的 resolution 一致

/// 造一条沿 x 延伸、宽 width_m 的走廊（墙在 y = ±width/2），并按 nav2 的
/// InflationLayer::computeCost 逐字生成代价：
///   到墙距离 == 0                    -> 254
///   到墙距离 <= inscribed_radius     -> 253
///   否则                              -> 指数衰减(<=252)
/// inscribed_radius 是**代价地图当前足迹**的内切半径（含 footprint_padding）。
FootprintCostFn corridorCost(double width_m, double inscribed_radius, double lateral_half)
{
  const double half = width_m * 0.5;
  return [half, inscribed_radius, lateral_half](double, double y, double) {
      // 足迹多边形查询 = 取外轮廓上代价最大的点。走廊里最糟的两点就是
      // 侧向最外那两点 y ± lateral_half。
      double worst = 0.0;
      for (const double s : {-1.0, 1.0}) {
        const double edge = y + s * lateral_half;
        const double d = half - std::fabs(edge);       // 到最近的墙的距离
        double c;
        if (d <= 0.0) {
          c = 254.0;                                   // 轮廓已经在墙里
        } else if (d <= inscribed_radius) {
          c = 253.0;                                   // 膨胀内切带
        } else {
          c = 252.0 * std::exp(-3.0 * (d - inscribed_radius));
        }
        worst = std::max(worst, c);
      }
      return worst;
    };
}
}  // namespace

TEST(CorridorArithmetic, Inflation253MakesEveryRealCorridorLookBlocked)
{
  // 本机实测数据（nav2 自己的 calculateMinAndMaxDistances + padFootprint）：
  //   八边形 侧向半宽 0.3894  内切(含 padding 0.01) 0.3992
  //   正方形 侧向半宽 0.3100  内切(含 padding 0.01) 0.3200
  const double kOctLateral = 0.3894, kOctInscribed = 0.3992;
  const double kSqLateral = 0.3100, kSqInscribed = 0.3200;

  // 用户给的场景事实：本仿真环境**没有低于 0.65m 的通道**。
  const double kNarrowest = 0.65;

  // ---- ① 正方形在 0.65m 通道里，几何上确实过得去 ----
  EXPECT_LT(kSqLateral, kNarrowest * 0.5)
    << "正方形侧向半宽 " << kSqLateral << " 必须 < 通道半宽 " << kNarrowest * 0.5;
  const auto sq = corridorCost(kNarrowest, kSqInscribed, kSqLateral);
  EXPECT_LT(sq(0.0, 0.0, 0.0), NarrowCostValues::kLethal)
    << "阈值 254 下：正方形居中时不压障碍本体 ⇒ 过得去（正确结论）";

  // ---- ② 但同一个位姿，253 阈值判它"过不去" ----
  EXPECT_GE(sq(0.0, 0.0, 0.0), NarrowCostValues::kInscribedInflated)
    << "轮廓落在膨胀内切带里 —— 这就是 909 次假阴性的来源";

  // ---- ③ 八边形在同一通道里是**真的**过不去（轮廓已进墙）----
  const auto oct = corridorCost(kNarrowest, kOctInscribed, kOctLateral);
  EXPECT_GE(oct(0.0, 0.0, 0.0), NarrowCostValues::kLethal)
    << "八边形侧向半宽 " << kOctLateral << " > 通道半宽 " << kNarrowest * 0.5
    << " ⇒ 轮廓进墙，254 阈值下正确判过不去";

  // ⇒ 换到 254 阈值后，"缩足迹"这个策略在本图上**才有区分度**：
  //    八边形 254(过不去) / 正方形 <254(过得去) = kAlignThenShrink 成立。
  EXPECT_LT(sq(0.0, 0.0, 0.0), oct(0.0, 0.0, 0.0))
    << "两种足迹必须能被区分开，否则缩足迹永远不会被选中";
}

TEST(CorridorArithmetic, The253ThresholdNeedsFourTimesHalfWidth)
{
  // 把重复计的量级算出来：轮廓要躲开 253 带，需要
  //     通道半宽 > 侧向半宽 + 内切半径 ≈ 2 * 侧向半宽
  // ⇒ 通道宽 > 4 * 侧向半宽。这就是"窄于约 1.58m 恒为 253"的成因。
  const double lat = 0.3894, ins = 0.3992;
  const double need = 2.0 * (lat + ins);
  EXPECT_NEAR(need, 1.5772, 1e-3);
  // 早先记下的实测症状是"窄于 1.62m 恒为 253"，两者差 0.043m < 一个栅格 0.05m。
  EXPECT_LT(std::fabs(need - 1.62), kRes)
    << "推导值与当初实测的 1.62m 必须在一个栅格之内，否则成因还没找对";

  // 逐个宽度扫一遍，确认 253 阈值下确实一路"过不去"直到 1.58m
  for (double w = 0.65; w < need; w += 0.05) {
    const auto c = corridorCost(w, ins, lat);
    EXPECT_GE(c(0.0, 0.0, 0.0), NarrowCostValues::kInscribedInflated)
      << "宽度 " << w << "m 在 253 阈值下应当被判过不去（这正是问题所在）";
  }
  const auto wide = corridorCost(need + 0.10, ins, lat);
  EXPECT_LT(wide(0.0, 0.0, 0.0), NarrowCostValues::kInscribedInflated)
    << "宽到 " << need + 0.10 << "m 才终于脱离 253 —— 判据的可用域只剩这些";
}

TEST(CorridorArithmetic, PlannerCenterCheckIsWhatActuallyBlocks)
{
  // 真正让 SmacPlanner2D 报 "Starting point in lethal space" 的是
  // **中心格**判据：中心格代价 >= 253 ⟺ 中心到墙距离 <= 内切半径。
  // 这一处用 253 是**对的** —— 那里膨胀带正好代表底盘尺寸。
  const double kNarrowest = 0.65, half = kNarrowest * 0.5;
  EXPECT_LT(half, 0.3992) << "八边形内切 0.3992 > 通道半宽 ⇒ 中心即 253，规划器拒绝";
  EXPECT_GT(half, 0.3200) << "正方形内切 0.3200 < 通道半宽 ⇒ 中心非 253，规划器可规划";
  // 余量只有 5mm = 0.1 个栅格。这是真实的紧张度，不是笔误 ——
  // 想放宽只有两条路：footprint_padding 0.01->0（余量 25mm）或
  // resolution 0.05->0.025（让 25mm 变成 1 个栅格）。
  EXPECT_NEAR(half - 0.3200, 0.005, 1e-9);
  EXPECT_LT(half - 0.3200, kRes) << "余量小于一个栅格 —— 这一点必须写在结论里";
}

// =====================================================================
// 侧向半宽 与 「按周期取模是否保几何」
//
// 这一组是 2026-09-03 那场事故的直接回归：当时共享一个 pi/4 周期
// （八边形的对称周期）去给**两种**足迹算有利朝向，结果正方形被算到
// 对角朝墙（0.4384，比八边形的 0.4200 还宽），"缩足迹"反而放大了足迹。
// =====================================================================

namespace
{
std::vector<PlanarPoint> octagonFp()
{
  return {{0.42, 0.0}, {0.297, 0.297}, {0.0, 0.42}, {-0.297, 0.297},
    {-0.42, 0.0}, {-0.297, -0.297}, {0.0, -0.42}, {0.297, -0.297}};
}
std::vector<PlanarPoint> squareFp()
{
  return {{0.31, 0.31}, {-0.31, 0.31}, {-0.31, -0.31}, {0.31, -0.31}};
}
/// 保几何校验的容差(m)。按物理取：比栅格 0.05m 细 50 倍，
/// 又容得下 yaml 三位小数的舍入(0.0214mm)。与生产守卫用同一个值。
constexpr double kPeriodTolM = 1e-3;
}  // namespace

TEST(LateralExtent, MatchesHandComputedValues)
{
  // delta=0 ⇒ 面朝通道方向，半宽就是 max|y|。
  EXPECT_NEAR(lateralHalfExtent(squareFp(), 0.0), 0.31, 1e-9);
  EXPECT_NEAR(lateralHalfExtent(octagonFp(), 0.0), 0.42, 1e-9);
  // delta=45° ⇒ 正方形对角朝墙 = 0.31*sqrt(2)
  EXPECT_NEAR(lateralHalfExtent(squareFp(), M_PI / 4.0), 0.31 * std::sqrt(2.0), 1e-9);
  // 八边形 45° ⇒ 对角顶点朝墙 = 0.297*sqrt(2) = 0.4200214...
  // ⚠️ 不是 0.42：yaml 里的八边形**不是正八边形**，对角顶点比轴向顶点外伸
  //    0.0214mm。这个量物理上可忽略（栅格的 1/2333），但写测试时按 0.42 断言
  //    就会失败 —— 第一版就是这么失败的，是我手算的值错了，不是实现错了。
  EXPECT_NEAR(
    lateralHalfExtent(octagonFp(), M_PI / 4.0), 0.297 * std::sqrt(2.0), 1e-9);
}

TEST(LateralExtent, SquareIsWiderThanOctagonAtFortyFiveDegrees)
{
  // 🔴 事故本体：pi/4 同余类里存在一个姿态，正方形比八边形还宽。
  const double sq = lateralHalfExtent(squareFp(), M_PI / 4.0);
  const double oc = lateralHalfExtent(octagonFp(), M_PI / 4.0);
  EXPECT_GT(sq, oc)
    << "正方形对角(" << sq << ") 必须确实宽于八边形(" << oc
    << ") —— 这就是共享 pi/4 周期会把机器人转到最坏姿态的原因";
  // 而在 0°（路径方向本身）正方形是真的更窄，这才是我们要的姿态。
  EXPECT_LT(lateralHalfExtent(squareFp(), 0.0), lateralHalfExtent(octagonFp(), 0.0));
}

TEST(LateralExtent, HalfPiPreservesGeometryForBothFootprints)
{
  // 🔴 这条决定了「按 pi/2 取模」这个等价合不合法。不成立就不许取模。
  //
  // ⚠️ 容差必须按**物理**取，不能取 1e-9。第一版取 1e-9 时八边形也判 false ——
  //    因为 yaml 的八边形不是正八边形（对角 0.297*sqrt2 比轴向 0.42 多 0.0214mm）。
  //    那是 yaml 三位小数的舍入，不是形状差异。1e-9 把舍入当差异 ⇒ 误报。
  //    取 1mm：比栅格 50mm 细 50 倍，又容得下舍入；而要抓的真实差异
  //   （正方形 pi/4：|0.4384-0.3100| = 128mm）比它大 128 倍，抓得很稳。
  EXPECT_TRUE(periodPreservesLateralExtent(squareFp(), M_PI / 2.0, kPeriodTolM))
    << "正方形在 90° 下必须保几何，否则 pi/2 同余类不成立";
  EXPECT_TRUE(periodPreservesLateralExtent(octagonFp(), M_PI / 2.0, kPeriodTolM))
    << "八边形在 90° 下必须保几何";
}

TEST(LateralExtent, QuarterPiDoesNotPreserveGeometryForSquare)
{
  // 反面：pi/4 对正方形**不保几何** —— 这正是启动守卫必须拦住的配置。
  EXPECT_FALSE(periodPreservesLateralExtent(squareFp(), M_PI / 4.0, kPeriodTolM))
    << "pi/4 对正方形保几何？那 0.3100 与 0.4384 就该相等 —— 说明校验函数是坏的";
  // 八边形在 1mm 容差下 pi/4 是保几何的，所以旧配置对**八边形单独**成立 ——
  // 错的是把同一个周期用到正方形上。这个对比正是"必须逐个足迹校验"的理由。
  EXPECT_TRUE(periodPreservesLateralExtent(octagonFp(), M_PI / 4.0, kPeriodTolM));
}

TEST(LateralExtent, RejectsBadArguments)
{
  EXPECT_FALSE(periodPreservesLateralExtent(squareFp(), 0.0, 1e-9));
  EXPECT_FALSE(periodPreservesLateralExtent(squareFp(), M_PI / 2.0, 0.0));
  EXPECT_FALSE(periodPreservesLateralExtent({{0.0, 0.0}, {1.0, 0.0}}, M_PI / 2.0, 1e-9));
  EXPECT_DOUBLE_EQ(lateralHalfExtent({{0.0, 0.0}, {1.0, 0.0}}, 0.0), 0.0)
    << "退化多边形必须返回 0，让「更窄」断言失败而不是放行";
}

TEST(LateralExtent, NarrowestCorridorEachFootprintCanPass)
{
  // 把结论落成数：沿路径方向对齐后，各自能过的最窄通道（不含 padding）。
  const double sq = 2.0 * lateralHalfExtent(squareFp(), 0.0);
  const double oc = 2.0 * lateralHalfExtent(octagonFp(), 0.0);
  EXPECT_NEAR(sq, 0.620, 1e-9);
  EXPECT_NEAR(oc, 0.840, 1e-9);
  // 本仿真环境最窄 0.65m：正方形过得去、八边形过不去 —— 缩足迹的意义就在这。
  EXPECT_LT(sq, 0.65);
  EXPECT_GT(oc, 0.65);
}

// =====================================================================
// 启动守卫的判据本身（守卫用的就是这两个函数，这里逐条反向验证）
// =====================================================================

TEST(SquareGuards, RejectsPeriodThatDoesNotPreserveLateralExtent)
{
  // 生产配置：pi/2 对两种足迹都成立 -> 放行
  EXPECT_TRUE(periodPreservesLateralExtent(octagonFp(), M_PI / 2.0, kPeriodTolM));
  EXPECT_TRUE(periodPreservesLateralExtent(squareFp(), M_PI / 2.0, kPeriodTolM));
  // 事故配置：pi/4 对正方形不成立 -> 守卫必须拒绝启动
  EXPECT_FALSE(periodPreservesLateralExtent(squareFp(), M_PI / 4.0, kPeriodTolM));
  // 其它常见误配也必须被拒
  for (double bad : {M_PI / 3.0, M_PI / 6.0, M_PI / 8.0, 1.0, 0.5}) {
    EXPECT_FALSE(periodPreservesLateralExtent(squareFp(), bad, kPeriodTolM))
      << "period=" << bad << " 对正方形竟然保几何？";
  }
}

TEST(SquareGuards, RejectsFootprintThatIsWiderAfterSwitch)
{
  // 守卫 8 / 运行期断言的判据：对齐后(delta=0)小足迹侧向半宽必须更小。
  const double lat_big = lateralHalfExtent(octagonFp(), 0.0);

  // ① 生产的正方形：0.31 < 0.42 -> 放行
  EXPECT_LT(lateralHalfExtent(squareFp(), 0.0), lat_big);

  // ② 把同一个正方形**转 45° 摆**（对角朝前）：侧向 0.4384 > 0.42 -> 必须拒绝。
  //    这是事故的等价构造 —— 面积一样、内切半径一样，只是摆放角度不同，
  //    而"内切半径"这个口径完全看不出区别。
  const double r = 0.31 * std::sqrt(2.0);
  const std::vector<PlanarPoint> diamond{{r, 0.0}, {0.0, r}, {-r, 0.0}, {0.0, -r}};
  EXPECT_GT(lateralHalfExtent(diamond, 0.0), lat_big)
    << "45° 摆放的同一正方形必须被判成更宽";
  // 内切半径口径下两者相同 —— 证明只看内切半径抓不到这件事
  EXPECT_NEAR(
    lateralHalfExtent(squareFp(), M_PI / 4.0), lateralHalfExtent(diamond, 0.0), 1e-9);
}

TEST(SquareGuards, DegenerateFootprintNeverPasses)
{
  // 退化足迹返回 0，于是"更窄"断言 (0 < lat_big) 会**通过** —— 危险。
  // 所以生产代码在 size<3 时是直接抛异常，不走比较。这条把那个前提钉住：
  // 一旦有人把守卫改成"只比数值"，这个用例会提醒他 0 是个陷阱。
  EXPECT_DOUBLE_EQ(lateralHalfExtent({{0.0, 0.0}, {1.0, 0.0}}, 0.0), 0.0);
  EXPECT_LT(lateralHalfExtent({{0.0, 0.0}, {1.0, 0.0}}, 0.0), lateralHalfExtent(octagonFp(), 0.0))
    << "退化足迹在纯数值比较下会被判成『更窄』—— 所以必须先查点数再比";
  EXPECT_FALSE(periodPreservesLateralExtent({{0.0, 0.0}, {1.0, 0.0}}, M_PI / 2.0, kPeriodTolM))
    << "退化足迹的周期校验必须 fail-safe 返回 false";
}

}  // namespace astribot_s1_path_tracking

// ==================== 朝向层迟滞：切入闸门 vs 交回阈值 ====================
// 实测缺陷：切入与交回复用同一个 yaw_gate_rad(0.120) ⇒ 没有迟滞 ⇒ 在闸门上自激。
//   误差 0.121 > 0.120 -> 接管纯旋转、平移置零 -> 转到 0.1199 -> 交回内层
//   -> 内层一往前走 -> 又超 0.120
// 30s 硬超时窗口内「重新对齐」累计 749 次、机器人一步未前进，最后被硬超时踢回
// 八边形。现象是「能进不能出」，而每条日志单独看都完全合理。
// 实测误差分布 min=0.120 p50=0.131 p95=0.215 max=0.215 —— 全部紧贴闸门。
TEST(YawHysteresis, ResumeMustBeStrictlyTighterThanGate)
{
  astribot_s1_path_tracking::NarrowLimits lim;
  EXPECT_GT(lim.yaw_gate_rad, lim.yaw_resume_rad)
    << "交回阈值必须严格小于切入闸门；相等就是没有迟滞，保证自激";
  EXPECT_GT(lim.yaw_resume_rad, 0.0);
}

TEST(YawHysteresis, BandCoversMeasuredJitter)
{
  // 迟滞带必须覆盖实测抖动，否则等于没加。实测超出量 p95 = 0.215-0.120 = 0.095rad，
  // 这里要求带宽至少是抖动**中位**超出量(0.131-0.120=0.011)的若干倍。
  astribot_s1_path_tracking::NarrowLimits lim;
  const double band = lim.yaw_gate_rad - lim.yaw_resume_rad;
  EXPECT_GE(band, 0.011 * 4.0)
    << "迟滞带 " << band << " 太窄，覆盖不住实测抖动 -> 仍会自激";
}

TEST(YawHysteresis, ResumeStaysWellBelowPassabilityNeed)
{
  // 交回阈值不能收得过狠：那会让机器人在通道里追求过高的对齐精度而走不动。
  // 0.06rad = 3.44°，远小于"朝向不对就过不去"所要求的精度。
  astribot_s1_path_tracking::NarrowLimits lim;
  EXPECT_LE(lim.yaw_resume_rad, 0.10);
}

TEST(YawHysteresis, GateAloneWouldOscillateOnMeasuredSamples)
{
  // 反向断言：用实测那 5 个紧贴闸门的样本，证明"单阈值"会在每个样本上都翻转，
  // 而"双阈值"不会。这条存在的意义是防止有人把 yaw_resume 改回等于 gate 后
  // 测试仍然全绿。
  const astribot_s1_path_tracking::NarrowLimits lim;
  const double measured[] = {0.120, 0.121, 0.124, 0.131, 0.135};
  int single_flips = 0;
  int dual_flips = 0;
  bool latched = false;
  for (double e : measured) {
    // 单阈值：每拍独立判断，超了就接管、不超就交回
    if (e > lim.yaw_gate_rad) { ++single_flips; }
    // 双阈值：一旦接管，必须降到 resume 以下才交回
    if (!latched && e > lim.yaw_gate_rad) { latched = true; ++dual_flips; }
    else if (latched && e <= lim.yaw_resume_rad) { latched = false; ++dual_flips; }
  }
  EXPECT_GE(single_flips, 4) << "单阈值在这组实测样本上反复接管，即自激";
  EXPECT_LE(dual_flips, 1) << "双阈值只应接管一次，之后保持接管直到真的转好";
}

// ============ 小足迹生效期间不得早退（否则落回零梯度的 MPPI） ============
// evaluateNarrow 开头有条"便宜早退"：中心格与足迹代价都低于阈值就放行。
// 缩足迹成功之后 footprint_cost 正好从 254 掉到 253 —— 那**就是**"装得下了"
// 的含义 —— 于是早退命中、贴边通行层不接管、控制权落回内层 MPPI。
// 而小足迹生效的区间按定义就是"窄到八边形过不去"，那里代价场是饱和的：
//     零梯度阈值 W <= 4 x 侧向半宽 = 4 x 0.32 = 1.28m
// 用武之地是 0.64 < W <= 0.86m，远小于 1.28m。实测（0.65~0.86m 通道）
// costmap_raw 剖面 22 点里 253 占 15 个(68%)，交回 MPPI 后累计行程 1.706m /
// 净位移 0.488m、|vx| 中位 0.0134、近零帧 46.4%，11 次切换里 4 次撞满 30s 超时。
//
// 这组测试把"早退条件"当纯函数复算一遍，钉住 squareActive 这个例外。
namespace
{
/// 与 evaluateNarrow 里那条早退**逐字同构**的判据（含 squareActive 例外）。
bool wouldEarlyReturn(
  double center_cost, double footprint_cost, bool square_active,
  const astribot_s1_path_tracking::NarrowTriggerConfig & cfg)
{
  return !square_active &&
         center_cost < cfg.center_lethal_threshold &&
         footprint_cost < cfg.footprint_lethal_threshold;
}
}  // namespace

TEST(SquareNoEarlyReturn, ShrunkFootprintCostIs253AndWouldTriggerEarlyReturn)
{
  // 前提复核：缩足迹成功后的典型读数(中心 253 以下、足迹 253)确实会命中早退。
  // 这一条证明例外**是必需的**，不是多余的防御。
  astribot_s1_path_tracking::NarrowTriggerConfig cfg;
  EXPECT_TRUE(wouldEarlyReturn(200.0, 253.0, /*square_active=*/ false, cfg))
    << "足迹 253 < 254 阈值 ⇒ 不加例外就会早退 ⇒ 落回零梯度的 MPPI";
}

TEST(SquareNoEarlyReturn, SquareActiveSuppressesEarlyReturn)
{
  astribot_s1_path_tracking::NarrowTriggerConfig cfg;
  EXPECT_FALSE(wouldEarlyReturn(200.0, 253.0, /*square_active=*/ true, cfg))
    << "小足迹生效期间必须继续由贴边通行层接管，不能早退";
}

TEST(SquareNoEarlyReturn, OpenSpaceStillEarlyReturnsWhenSquareInactive)
{
  // 例外不能把开阔处的早退也一起关掉 —— 那会让本层在整条路上都接管，
  // 白付路径变换/横向扫描的开销，并且把 MPPI 完全排除在正常路段之外。
  astribot_s1_path_tracking::NarrowTriggerConfig cfg;
  EXPECT_TRUE(wouldEarlyReturn(0.0, 0.0, /*square_active=*/ false, cfg));
}

TEST(SquareNoEarlyReturn, LethalStillNoEarlyReturnRegardlessOfSquare)
{
  // 足迹真致命(254)时两种状态都不该早退。
  astribot_s1_path_tracking::NarrowTriggerConfig cfg;
  EXPECT_FALSE(wouldEarlyReturn(200.0, 254.0, false, cfg));
  EXPECT_FALSE(wouldEarlyReturn(200.0, 254.0, true, cfg));
}

TEST(SquareNoEarlyReturn, ZeroGradientThresholdArithmetic)
{
  // 钉住那条算术：正方形侧向半宽 0.32 ⇒ 零梯度阈值 1.28m ⇒ 覆盖整个用武之地
  // (0.64, 0.86]。这解释了"为什么缩足迹之后仍然需要贴边通行层沿路径走"。
  const std::vector<astribot_s1_path_tracking::PlanarPoint> sq =
    astribot_s1_path_tracking::squareFp();
  const double lateral = astribot_s1_path_tracking::lateralHalfExtent(sq, 0.0);
  const double zero_grad_w = 4.0 * lateral;
  EXPECT_NEAR(lateral, 0.31, 1e-9);
  EXPECT_NEAR(zero_grad_w, 1.24, 1e-9);
  // 用武之地上界 0.86m 必须仍然小于零梯度阈值 —— 即缩足迹**没有**消除零梯度
  EXPECT_LT(2.0 * 0.4300, zero_grad_w)
    << "若 0.86 >= 零梯度阈值，则缩足迹本身就解决了梯度问题，本例外可以删除";
}

// ==========================================================================
// 「代价场饱和 + 内层无进展」第二入口
//
// 加这条入口的实测依据（2026-09-03，本图 y=-5.82、x=2.00→4.25 共 2.25m 通道）：
//     足迹多边形代价  恒 253，**一次 254 都没有**
//     中心格代价      63 → 216（峰值），从不 >= 253
//     净宽            1.05~1.80m（八边形只要 0.86m，本来就过得去）
// ⇒ 原有 ①②④ 三条判据全不成立 ⇒ 一次都不接管（日志侧：5 条腿触发 0 次）。
// 而机器人确实过不去：停在 x≈2.02~2.14（5 次实测），81 次 Failed to make progress。
// ==========================================================================
namespace astribot_s1_path_tracking
{

TEST(SaturationEntry, MeasuredCorridorProfileGetsNoTakeoverWithoutTheSecondEntry)
{
  // 这一条是**反向断言**：把实测剖面喂进去，不给 saturated_stalled，
  // 结果必须是 kNone —— 这正是"策略一次都没生效"的机制本身。
  const NarrowTriggerConfig cfg;
  const double measured_center[] = {63.0, 80.0, 89.0, 100.0, 150.0, 186.0, 201.0, 216.0};
  for (const double c : measured_center) {
    EXPECT_EQ(
      evaluateNarrowTrigger(c, 253.0, 253.0, true, false, false, cfg), NarrowVerdict::kNone)
      << "中心代价 " << c << " 配足迹 253：无第二入口时必须是 kNone（这就是那个 bug）";
  }
}

TEST(SaturationEntry, SameProfileTakesOverOnceStallIsObserved)
{
  const NarrowTriggerConfig cfg;
  const double measured_center[] = {63.0, 80.0, 89.0, 100.0, 150.0, 186.0, 201.0, 216.0};
  for (const double c : measured_center) {
    EXPECT_EQ(
      evaluateNarrowTrigger(c, 253.0, 253.0, true, true, false, cfg), NarrowVerdict::kNarrow)
      << "中心代价 " << c << " 配足迹 253 且实测无进展：必须接管";
  }
}

TEST(SaturationEntry, StallDoesNotOverrideAnyRedLine)
{
  const NarrowTriggerConfig cfg;
  // 红线：最有利朝向下压真障碍 —— 无进展也不许接管硬挤。
  EXPECT_EQ(
    evaluateNarrowTrigger(100.0, 253.0, 254.0, true, true, false, cfg),
    NarrowVerdict::kPhysicallyBlocked);
  // 中心格致命 —— 归 ESCAPE，不是本层。
  EXPECT_EQ(
    evaluateNarrowTrigger(253.0, 253.0, 100.0, true, true, false, cfg),
    NarrowVerdict::kCenterLethal);
  // 无路径 —— 本层以路径为核心约束，无进展也不接管。
  EXPECT_EQ(
    evaluateNarrowTrigger(100.0, 253.0, 100.0, false, true, false, cfg),
    NarrowVerdict::kNoPath);
}

TEST(SaturationEntry, UnsaturatedStallIsNotThisLayersBusiness)
{
  // 没饱和还走不动，是别的原因（内层参数、动力学、被真障碍挡住）。
  // 本层不能因为"车没动"就接管 —— 那会变成万能兜底，掩盖真实故障。
  const NarrowTriggerConfig cfg;
  for (const double fp : {0.0, 100.0, 200.0, 252.0}) {
    EXPECT_EQ(
      evaluateNarrowTrigger(100.0, fp, fp, true, true, false, cfg), NarrowVerdict::kNone)
      << "足迹 " << fp << " 未达饱和阈值，无进展也不接管";
  }
}

TEST(SaturationEntry, SaturationThresholdMustStayBelowLethal)
{
  // 阈值配成 254 时这条入口与 ④ 完全重合 = 等于没加。
  // 控制器启动守卫会拒绝这种配置，这里锁住那条算术前提。
  const NarrowTriggerConfig cfg;
  EXPECT_LT(cfg.saturation_threshold, cfg.footprint_lethal_threshold);
  EXPECT_DOUBLE_EQ(cfg.saturation_threshold, NarrowCostValues::kInscribedInflated);
  EXPECT_DOUBLE_EQ(cfg.footprint_lethal_threshold, NarrowCostValues::kLethal);
}

TEST(SaturationStall, WindowNeedsBothSaturatedAndFollowing)
{
  const NarrowTriggerConfig cfg;
  NarrowStallState st;
  // 不饱和 ⇒ 窗口清空
  EXPECT_FALSE(updateSaturationStall(false, true, 0.0, 0.0, 0.0, cfg, st));
  EXPECT_FALSE(st.has_anchor);
  // 不在 FOLLOW（原地对齐）⇒ 窗口清空，原地转不算卡住
  EXPECT_FALSE(updateSaturationStall(true, false, 0.0, 0.0, 0.0, cfg, st));
  EXPECT_FALSE(st.has_anchor);
  // 两者都真 ⇒ 开窗
  EXPECT_FALSE(updateSaturationStall(true, true, 0.0, 0.0, 0.0, cfg, st));
  EXPECT_TRUE(st.has_anchor);
}

TEST(SaturationStall, FiresOnlyAfterTheFullWindow)
{
  const NarrowTriggerConfig cfg;    // 3.0s / 0.05m
  NarrowStallState st;
  EXPECT_FALSE(updateSaturationStall(true, true, 0.0, 0.0, 100.0, cfg, st));
  // 窗口内一动不动，但还没到 3s
  EXPECT_FALSE(updateSaturationStall(true, true, 0.0, 0.0, 101.0, cfg, st));
  EXPECT_FALSE(updateSaturationStall(true, true, 0.0, 0.0, 102.9, cfg, st));
  // 到点
  EXPECT_TRUE(updateSaturationStall(true, true, 0.0, 0.0, 103.0, cfg, st));
}

TEST(SaturationStall, ProgressReopensTheWindow)
{
  const NarrowTriggerConfig cfg;
  NarrowStallState st;
  // ⚠️ 推进量必须由 cfg 推出来，不能写死数字。原来写死 0.06（"> 0.05"），
  // 门槛从 0.05 改到 0.15 之后这条测试就在测一件不成立的事（0.06 不再算推进），
  // 而失败信息指向 anchor_sec、完全看不出真因。
  const double progressed = cfg.saturation_min_move_m * 1.2;
  updateSaturationStall(true, true, 0.0, 0.0, 100.0, cfg, st);
  // 差一点到窗口尾时推进了 > 门槛 ⇒ 内层还在工作，窗口重开
  EXPECT_FALSE(
    updateSaturationStall(
      true, true, progressed, 0.0, 100.0 + cfg.saturation_stall_sec - 0.1, cfg, st));
  EXPECT_DOUBLE_EQ(st.anchor_sec, 100.0 + cfg.saturation_stall_sec - 0.1);
  // 从新锚点起再等不足一个窗口还不够
  EXPECT_FALSE(
    updateSaturationStall(
      true, true, progressed, 0.0, 100.0 + 2 * cfg.saturation_stall_sec - 1.1, cfg, st));
  // 满一个窗口才算卡住
  EXPECT_TRUE(
    updateSaturationStall(
      true, true, progressed, 0.0, 100.0 + 2 * cfg.saturation_stall_sec - 0.1, cfg, st));
}

TEST(SaturationStall, WindowIsClearedNotPausedOnLeavingSaturation)
{
  // 「走一段-停一段」不能累计成假卡住：脱离饱和带必须清空而不是暂停。
  const NarrowTriggerConfig cfg;
  NarrowStallState st;
  updateSaturationStall(true, true, 0.0, 0.0, 100.0, cfg, st);
  EXPECT_FALSE(updateSaturationStall(true, true, 0.0, 0.0, 102.5, cfg, st));
  // 中间出了一拍饱和带
  EXPECT_FALSE(updateSaturationStall(false, true, 0.0, 0.0, 102.6, cfg, st));
  // 回到饱和带：从这一刻重新起算，不能拿 100.0 当锚点直接判卡住
  EXPECT_FALSE(updateSaturationStall(true, true, 0.0, 0.0, 102.7, cfg, st));
  EXPECT_DOUBLE_EQ(st.anchor_sec, 102.7);
  EXPECT_FALSE(updateSaturationStall(true, true, 0.0, 0.0, 104.9, cfg, st));
  EXPECT_TRUE(updateSaturationStall(true, true, 0.0, 0.0, 105.7, cfg, st));
}

TEST(SaturationStall, ClockGoingBackwardsIsNotALongStall)
{
  const NarrowTriggerConfig cfg;
  NarrowStallState st;
  updateSaturationStall(true, true, 0.0, 0.0, 500.0, cfg, st);
  // sim time 重置 ⇒ now < anchor。不能算成负时长，也不能算成"卡了很久"。
  EXPECT_FALSE(updateSaturationStall(true, true, 0.0, 0.0, 10.0, cfg, st));
  EXPECT_DOUBLE_EQ(st.anchor_sec, 10.0);
}

TEST(SaturationStall, WindowExceedsOneReplanPeriod)
{
  // 实测内层重规划周期 1.05s。窗口必须明显大于它，否则换路径那一拍的
  // 停顿会被误判成卡住。
  const NarrowTriggerConfig cfg;
  EXPECT_GT(cfg.saturation_stall_sec, 2.0 * 1.05);
}

TEST(SaturationStall, MinMoveSitsBetweenMeasuredCreepAndMeasuredHealthyTravel)
{
  // 🔴 这条测试原来断言的是「>= 一个栅格(0.05m)」，理由是"低于此与定位噪声
  // 不可分"。理由本身没错，但它是**下界**、而当作了取值依据 —— 实测把 0.05
  // 直接否掉了：MPPI 在膨胀坡脚是蠕行不是不动，任意 3s 窗口位移 0.0937m
  // 已经超过 0.05m，窗口被一次次重开，第二入口接管 0 次。
  // 「有没有动」是错的问题，对的问题是「推进得比本层还慢吗」。
  const NarrowTriggerConfig cfg;
  const double window_sec = cfg.saturation_stall_sec;      // 3.0s
  const double v_along = 0.10;                             // narrow_v_along 生产值

  const double measured_creep_m = 0.0937;    // 实测：任意 3s 窗口内最大位移
  const double measured_healthy_m = 0.1022 * window_sec;   // phase5 成功段 0.1022m/s
  const double layer_reach_m = v_along * window_sec;       // 本层自己能走多远

  // 必须能判出蠕行 —— 否则这条入口在实测现场根本进不去。
  EXPECT_GT(cfg.saturation_min_move_m, measured_creep_m)
    << "门槛没盖住实测蠕行 0.0937m，第二入口会像 phase7 那样接管 0 次";
  // 必须不误判健康通行 —— 否则会把本来走得好的段也抢过来。
  EXPECT_LT(cfg.saturation_min_move_m, measured_healthy_m)
    << "门槛超过实测健康通行 " << measured_healthy_m << "m，会误接管";
  // 必须 < 本层自己可达，否则连本层都达不到、等于恒判无进展（启动守卫同款）。
  EXPECT_LT(cfg.saturation_min_move_m, layer_reach_m);
  // 仍然要在定位噪声之上（原来那条理由作为下界依然成立）。
  EXPECT_GE(cfg.saturation_min_move_m, 0.05);
  // 两侧余量都要有量级冗余，不能刚好卡在实测值上。
  EXPECT_GT(cfg.saturation_min_move_m / measured_creep_m, 1.5);
  EXPECT_GT(measured_healthy_m / cfg.saturation_min_move_m, 1.5);
}

TEST(SaturationStall, MeasuredCreepTraceWouldNeverHaveTriggeredAtOneGrid)
{
  // 用实测蠕行轨迹直接回放：0.0061 m/s 净速率、每拍 0.1s。
  // 旧门槛 0.05m 下窗口会被反复重开 ⇒ 永不触发（= phase7 的 0 次接管）；
  // 新门槛 0.15m 下同一条轨迹必须触发。
  const double dt = 0.1;
  const double creep_speed = 0.0937 / 3.0;   // 实测 3s 窗口最大位移折算的速率

  auto replay = [&](double min_move) {
      NarrowTriggerConfig cfg;
      cfg.saturation_stall_sec = 3.0;
      cfg.saturation_min_move_m = min_move;
      NarrowStallState st;
      bool ever_stalled = false;
      for (int i = 0; i < 200; ++i) {      // 20 秒
        const double t = i * dt;
        const double x = creep_speed * t;
        if (updateSaturationStall(true, true, x, 0.0, t, cfg, st)) {ever_stalled = true;}
      }
      return ever_stalled;
    };

  EXPECT_FALSE(replay(0.05)) << "旧门槛竟然触发了，实测前提(0 次接管)有误";
  EXPECT_TRUE(replay(0.15)) << "新门槛在实测蠕行轨迹上仍不触发，改了等于没改";
}

// =====================================================================
// 第二入口的最短驻留。这一组测的是 2026-09-03 第二轮实测暴露的振荡：
// 只把脱离阈值从 254 改成 253（迟滞同阈值）不够，还必须同**变量**。
// =====================================================================

TEST(SaturationDwell, MeasuredEngagementDurationsAreAllTooShortToEverAlign)
{
  // 2026-09-03 第二轮实测的 18 次接管，逐次配对「接管时长 / 起手朝向误差」。
  // 闸门 0.12rad、wz 上限 0.20rad/s ⇒ 转进闸门需要 (|err| - gate)/wz 秒。
  const double gate = 0.12;
  const double wz = 0.20;
  struct Engagement { double dur_sec; double yaw_err_rad; };
  const std::vector<Engagement> measured{
    {0.60, 0.443}, {0.90, 0.458}, {0.80, 0.444}, {0.55, 0.423}, {0.75, 0.432},
    {2.79, 0.421}, {0.95, 0.686}, {1.80, 0.774}, {27.10, 0.496}, {0.45, 0.733},
    {1.70, 0.736}, {0.90, 0.515}, {16.70, 0.105}, {0.60, 0.526}, {0.45, 0.513},
    {0.35, 0.508}, {17.20, 0.619}, {15.52, 0.178}};

  int too_short = 0;
  for (const auto & e : measured) {
    const double need = (e.yaw_err_rad - gate) / wz;
    if (e.dur_sec < need) {++too_short;}
  }
  // 实测就是 13/18 次在**还没转正**时被代价纹理赶了出去。这不是调参问题：
  // 那 13 次一拍前进指令都没发出过，出去后 MPPI 转回自己的朝向、再卡 3s、
  // 再从同样的 ~0.44rad 重来 —— 出口判据必须先给足转正时间。
  EXPECT_EQ(too_short, 13);

  // 而 4.0s 的默认驻留覆盖了这 18 次里**每一次**的转正需求。
  for (const auto & e : measured) {
    EXPECT_LE((e.yaw_err_rad - gate) / wz, 4.0)
      << "朝向误差 " << e.yaw_err_rad << "rad 转正要 "
      << (e.yaw_err_rad - gate) / wz << "s，超过了默认驻留 4.0s";
  }
}

TEST(SaturationDwell, DefaultDwellCoversTheArithmeticAlignBound)
{
  const double bound = saturationAlignBoundSec(M_PI / 2.0, 0.20);
  EXPECT_NEAR(bound, 3.927, 1e-3);       // (1.5708/2)/0.20
  // 生产默认值（见 three_phase_controller.hpp）必须覆盖这个下界。
  EXPECT_GE(4.0, bound);
  // 且必须短于卡住判据 6s，否则驻留期自己会撞上「原地蹭」被判死。
  EXPECT_LT(4.0, 6.0);
}

TEST(SaturationDwell, AlignBoundScalesWithBothInputsAndSurvivesZeroWz)
{
  EXPECT_GT(saturationAlignBoundSec(M_PI / 2.0, 0.10),
    saturationAlignBoundSec(M_PI / 2.0, 0.20));      // 转得慢 -> 要更久
  EXPECT_GT(saturationAlignBoundSec(M_PI, 0.20),
    saturationAlignBoundSec(M_PI / 2.0, 0.20));      // 周期大 -> 要更久
  EXPECT_TRUE(std::isfinite(saturationAlignBoundSec(M_PI / 2.0, 0.0)));  // 不许 inf/nan
}

TEST(SaturationDwell, HoldsOnlyWhileEngagedViaSaturationAndInsideTheWindow)
{
  // 窗口内、由第二入口进来 ⇒ 保持（代价掉下去也不许退）。
  EXPECT_TRUE(saturationDwellHolding(true, true, 1.0, 4.0));
  // 窗口过了 ⇒ 交还给代价判据。
  EXPECT_FALSE(saturationDwellHolding(true, true, 4.0, 4.0));
  EXPECT_FALSE(saturationDwellHolding(true, true, 9.9, 4.0));
  // 不是第二入口进来的（即 254 那条正常入口）⇒ 本规则不介入，
  // 否则会把原来就能正常脱离的接管一律拖长 4s。
  EXPECT_FALSE(saturationDwellHolding(true, false, 1.0, 4.0));
  // 根本没接管 ⇒ 不介入（早退路径必须照常放行开阔地）。
  EXPECT_FALSE(saturationDwellHolding(false, true, 1.0, 4.0));
}

TEST(SaturationDwell, ClockGoingBackwardsReleasesRatherThanLatchesForever)
{
  // 负的已接管时长只可能来自时钟回跳。若按「< dwell 即保持」处理，
  // 回跳越大保持越久 —— 而回跳量无上界，等于永久锁死接管。
  EXPECT_FALSE(saturationDwellHolding(true, true, -0.5, 4.0));
  EXPECT_FALSE(saturationDwellHolding(true, true, -1e6, 4.0));
}

TEST(SaturationDwell, ZeroDwellIsTheOneKeyRollbackToTheOscillatingBehaviour)
{
  // 把 dwell 设 0 应当完全等价于「没有这条规则」，供一键回退 A/B。
  EXPECT_FALSE(saturationDwellHolding(true, true, 0.0, 0.0));
}

// 出口判据必须同时问「拍数」和「驻留」。这个用例存在的理由是实测抓到的偏差：
// 早退路径问了驻留，而 kNone 那条 disengageNarrow 没问 ⇒ banner 打印
// 「最短驻留=4.0s」而真实接管只活了 0.45s，随后交回 MPPI 96s 没有进展。
TEST(NarrowDisengage, DwellVetoesEvenWhenClearTicksAreSatisfied)
{
  // 拍数够了但驻留还没满 ⇒ 不许退出。这就是原来漏掉的那一项。
  EXPECT_FALSE(narrowShouldDisengage(3, 3, /*dwell_holding=*/ true));
  EXPECT_FALSE(narrowShouldDisengage(999, 3, true));
  // 驻留满了、拍数够了 ⇒ 退出。
  EXPECT_TRUE(narrowShouldDisengage(3, 3, false));
  // 驻留满了但拍数不够 ⇒ 仍不退（消抖照旧生效）。
  EXPECT_FALSE(narrowShouldDisengage(2, 3, false));
  // need 非法时 narrowCleared 按「永不判脱离」处理，这里必须继承该语义，
  // 否则配置写错会让接管一拍就退出且不报错。
  EXPECT_FALSE(narrowShouldDisengage(999, 0, false));
  EXPECT_FALSE(narrowShouldDisengage(999, -1, false));
}

TEST(NarrowDisengage, DwellDominatesClearTicksByArithmeticNotByTuning)
{
  // 算术：驻留 4.0s @20Hz = 80 拍，而 narrow_clear_ticks 默认 3 拍。
  // 80 >> 3 ⇒ 在驻留窗口内，拍数判据**必然**早已满足。
  // 也就是说漏问驻留不是"偶尔"漏掉，而是每次接管都会被拍数判据抢先退出。
  const double dwell_sec = 4.0;
  const double hz = 20.0;
  const int dwell_ticks = static_cast<int>(dwell_sec * hz);
  const int clear_ticks = 3;
  EXPECT_GT(dwell_ticks, clear_ticks);
  // 逐拍走一遍：驻留窗口内每一拍都不许退出。
  for (int tick = 0; tick <= dwell_ticks; ++tick) {
    const double engaged_sec = tick / hz;
    const bool holding = saturationDwellHolding(true, true, engaged_sec, dwell_sec);
    const bool disengage = narrowShouldDisengage(clear_ticks, clear_ticks, holding);
    if (engaged_sec < dwell_sec) {
      EXPECT_FALSE(disengage) << "tick=" << tick << " engaged_sec=" << engaged_sec;
    } else {
      EXPECT_TRUE(disengage) << "tick=" << tick << " engaged_sec=" << engaged_sec;
    }
  }
  // 驻留下界不能小于转正所需时间，否则必然在还没转正时被赶出去
  // （实测就是这样：接管 0.45s，而下界 3.93s）。
  EXPECT_GE(dwell_sec, saturationAlignBoundSec(M_PI / 2.0, 0.20));
}

}  // namespace astribot_s1_path_tracking
namespace astribot_s1_path_tracking
{

// =====================================================================
// 例外五：入口判据不得复用为每拍的驾驶判据
//
// 实测（51.3s 一次接管、1024 帧 /cmd_vel）：中位 |wz|=0.0287 而本层饱和值
// 应为 0.20；|wz| 落在 0.19~0.21 的帧仅 12.1%；105 帧 |vx|>0.10、32 帧
// |wz|>0.21 —— 都超过本层硬上限，只能来自内层 MPPI。即"已接管"标志亮着，
// 方向盘 88% 的时间在 MPPI 手里；累计转角 3.728rad vs 净转角 1.311rad。
// =====================================================================
TEST(SaturationEngagedDrive, LayerCannotRevokeItsOwnAuthorityByMakingProgress)
{
  NarrowTriggerConfig cfg;
  const double fp = cfg.saturation_threshold;      // 253：这一类通道的恒定读数

  // 未接管 + 内层无进展 ⇒ 允许进入（第二入口的本意）
  EXPECT_EQ(
    evaluateNarrowTrigger(0.0, fp, fp, true, /*stalled=*/ true, /*engaged=*/ false, cfg),
    NarrowVerdict::kNarrow);

  // 🔴 关键：已接管 + 内层"有进展"（因为**本层自己在走**）⇒ 仍须 kNarrow。
  //    修复前这里返回 kNone，方向盘每 1.5s 就被交回一次。
  EXPECT_EQ(
    evaluateNarrowTrigger(0.0, fp, fp, true, /*stalled=*/ false, /*engaged=*/ true, cfg),
    NarrowVerdict::kNarrow)
    << "已接管期间问 saturated_stalled 等于让本层用自己的进展吊销自己的授权";

  // 未接管 + 有进展 ⇒ 不该进入（入口判据本身没被削弱）
  EXPECT_EQ(
    evaluateNarrowTrigger(0.0, fp, fp, true, /*stalled=*/ false, /*engaged=*/ false, cfg),
    NarrowVerdict::kNone);
}

TEST(SaturationEngagedDrive, SelfRevokeDeadlineIsShorterThanAnyAlignment)
{
  // 算术证明这个 bug 无法靠调参消除：
  //   本层推进到 min_move 所需时间 = min_move / v_along
  //   而转正到有利朝向所需时间 = (favorable_period/2) / wz_max
  // 前者 < 后者 ⇒ 授权必在转正之前就被自己吊销。
  NarrowTriggerConfig cfg;
  const double v_along = 0.10;      // narrow_v_along
  const double wz_max = 0.20;       // narrow_wz_max
  const double favorable_period = 1.5707963267948966;

  const double self_revoke_sec = cfg.saturation_min_move_m / v_along;
  const double align_sec = saturationAlignBoundSec(favorable_period, wz_max);

  EXPECT_NEAR(self_revoke_sec, 1.5, 1e-9);
  EXPECT_NEAR(align_sec, 3.927, 1e-3);
  EXPECT_LT(self_revoke_sec, align_sec)
    << "自我吊销 " << self_revoke_sec << "s < 转正所需 " << align_sec
    << "s ⇒ 只要每拍还问 saturated_stalled，本层就永远转不正。"
       "把 min_move 调大到 " << align_sec * v_along
    << "m 以上才能躲过，但那等于要求本层比它自己更慢 —— 所以只能改结构。";
}
}  // namespace astribot_s1_path_tracking
