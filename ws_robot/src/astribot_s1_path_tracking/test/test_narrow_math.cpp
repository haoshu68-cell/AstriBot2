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
          const auto v = evaluateNarrowTrigger(center, fp, fav, have != 0, cfg);
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
    const auto v = evaluateNarrowTrigger(0.0, 254.0, fav, true, cfg);
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
    evaluateNarrowTrigger(253.0, 254.0, 0.0, true, cfg), NarrowVerdict::kCenterLethal);
  EXPECT_EQ(
    evaluateNarrowTrigger(255.0, 253.0, 100.0, true, cfg), NarrowVerdict::kCenterLethal);
}

TEST(NarrowRedLine, CenterLethalIsNotOurJob)
{
  // 中心格致命 ⇒ clearance<0.388 ⇒ 物理放不进去 ⇒ 本层必须拒绝接管。
  // 穷举足迹代价（253 以下的真障碍之外的全部取值）。
  NarrowTriggerConfig cfg;
  for (int fp = 0; fp < 254; ++fp) {
    const auto v = evaluateNarrowTrigger(253.0, fp, 0.0, true, cfg);
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
  // 目标域：中心格可站(<253) + 足迹碰致命带(>=253)
  NarrowTriggerConfig cfg;
  EXPECT_EQ(evaluateNarrowTrigger(137.0, 253.0, 0.0, true, cfg), NarrowVerdict::kNarrow);
  EXPECT_EQ(evaluateNarrowTrigger(0.0, 253.0, 0.0, true, cfg), NarrowVerdict::kNarrow);
}

TEST(NarrowTrigger, WideAreaYieldsNone)
{
  NarrowTriggerConfig cfg;
  EXPECT_EQ(evaluateNarrowTrigger(0.0, 0.0, 0.0, true, cfg), NarrowVerdict::kNone);
  EXPECT_EQ(evaluateNarrowTrigger(137.0, 252.0, 0.0, true, cfg), NarrowVerdict::kNone);
}

TEST(NarrowTrigger, NoPathMeansNoTakeover)
{
  // 本层以全局路径为核心约束。无路径不接管 —— 那种情况是 ESCAPE 的事。
  NarrowTriggerConfig cfg;
  EXPECT_EQ(evaluateNarrowTrigger(137.0, 253.0, 0.0, false, cfg), NarrowVerdict::kNoPath);
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

}  // namespace astribot_s1_path_tracking
