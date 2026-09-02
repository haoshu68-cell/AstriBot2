// Copyright 2026 Astribot
//
// 三段式跟踪纯函数层的测试。
//
// 这里刻意包含几条**哨兵**：
//   · align_goal_enabled=false 时绝不产生终点旋转（探索场景的硬要求）
//   · 容差取自调用方传入，函数内部不自带阈值
//   · 路径退化时明确失败，而不是返回一个看起来正常的角度
// 这三条一旦被人「顺手优化」掉，在线表现分别是「探索每个点都多转一次」、
// 「两处阈值悄悄漂开」、「机器人朝 +x 乱走」，都很难从日志反推。

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "astribot_s1_path_tracking/align_math.hpp"

using astribot_s1_path_tracking::Phase;
using astribot_s1_path_tracking::PlanarPoint;
using astribot_s1_path_tracking::advancePhase;
using astribot_s1_path_tracking::alignAngularVelocity;
using astribot_s1_path_tracking::needsStartAlign;
using astribot_s1_path_tracking::normalizeAngle;
using astribot_s1_path_tracking::pathStartHeading;
using astribot_s1_path_tracking::shortestAngularDiff;
using astribot_s1_path_tracking::toString;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;
}  // namespace

// ---------------- 角度归一化与最短转向 ----------------

TEST(NormalizeAngle, KeepsInsideRange)
{
  EXPECT_NEAR(normalizeAngle(0.0), 0.0, kEps);
  EXPECT_NEAR(normalizeAngle(1.0), 1.0, kEps);
  EXPECT_NEAR(normalizeAngle(-1.0), -1.0, kEps);
}

TEST(NormalizeAngle, WrapsBeyondPi)
{
  EXPECT_NEAR(normalizeAngle(3.0 * kPi), kPi, 1e-6);
  EXPECT_NEAR(normalizeAngle(2.0 * kPi + 0.5), 0.5, 1e-9);
  EXPECT_NEAR(normalizeAngle(-2.0 * kPi - 0.5), -0.5, 1e-9);
}

TEST(NormalizeAngle, HugeInputDoesNotHang)
{
  // 手写 while(a>pi) a-=2pi 在 1e9 量级会转很久；这里必须立即返回。
  const double v = normalizeAngle(1.0e9);
  EXPECT_LE(std::fabs(v), kPi + 1e-6);
}

TEST(ShortestAngularDiff, TakesShortWayAroundPi)
{
  // 从 +170° 到 -170°：近路是 +20°，不是 -340°。
  const double from = 170.0 * kPi / 180.0;
  const double to = -170.0 * kPi / 180.0;
  EXPECT_NEAR(shortestAngularDiff(from, to), 20.0 * kPi / 180.0, 1e-9);
}

TEST(ShortestAngularDiff, SignEncodesDirection)
{
  EXPECT_GT(shortestAngularDiff(0.0, 0.5), 0.0);
  EXPECT_LT(shortestAngularDiff(0.0, -0.5), 0.0);
}

TEST(ShortestAngularDiff, ZeroWhenEqual)
{
  EXPECT_NEAR(shortestAngularDiff(1.234, 1.234), 0.0, kEps);
}

// ---------------- 路径起始朝向 ----------------

TEST(PathStartHeading, UsesLookaheadNotAdjacentPoint)
{
  // 前两点几乎重合且方向偏（模拟规划器输出的密集顶点 + 噪声），
  // 真实走向是 +x。取相邻点会得到约 +45°，取前视应得到接近 0。
  std::vector<PlanarPoint> path{
    {0.0, 0.0}, {0.001, 0.001}, {0.20, 0.0}, {0.60, 0.0}, {1.00, 0.0}};
  double h = -99.0;
  ASSERT_TRUE(pathStartHeading(path, 0.5, h));
  EXPECT_NEAR(h, 0.0, 0.05);
}

TEST(PathStartHeading, FallsBackToEndpointWhenPathShorterThanLookahead)
{
  std::vector<PlanarPoint> path{{0.0, 0.0}, {0.0, 0.10}};
  double h = 0.0;
  ASSERT_TRUE(pathStartHeading(path, 5.0, h));
  EXPECT_NEAR(h, kPi / 2.0, 1e-6);
}

TEST(PathStartHeading, RejectsTooFewPoints)
{
  std::vector<PlanarPoint> one{{0.0, 0.0}};
  double h = 0.0;
  EXPECT_FALSE(pathStartHeading(one, 0.5, h));
  EXPECT_FALSE(pathStartHeading({}, 0.5, h));
}

TEST(PathStartHeading, RejectsDegeneratePathInsteadOfReturningZero)
{
  // 哨兵：所有顶点重合时没有方向可言。若实现改成返回 atan2(0,0)=0，
  // 机器人会一律先转到朝 +x，而日志上完全看不出异常。
  std::vector<PlanarPoint> same{{1.0, 2.0}, {1.0, 2.0}, {1.0, 2.0}};
  double h = 12345.0;
  EXPECT_FALSE(pathStartHeading(same, 0.5, h));
  EXPECT_DOUBLE_EQ(h, 12345.0);  // 失败时不得改写出参
}

TEST(PathStartHeading, RejectsNonPositiveLookahead)
{
  std::vector<PlanarPoint> path{{0.0, 0.0}, {1.0, 0.0}};
  double h = 0.0;
  EXPECT_FALSE(pathStartHeading(path, 0.0, h));
  EXPECT_FALSE(pathStartHeading(path, -1.0, h));
}

// ---------------- 起步对齐触发 ----------------

TEST(NeedsStartAlign, TriggersOnlyBeyondMinAngle)
{
  EXPECT_TRUE(needsStartAlign(1.0, 0.5));
  EXPECT_TRUE(needsStartAlign(-1.0, 0.5));
  EXPECT_FALSE(needsStartAlign(0.4, 0.5));
  EXPECT_FALSE(needsStartAlign(-0.4, 0.5));
}

TEST(NeedsStartAlign, BoundaryIsNotTriggering)
{
  EXPECT_FALSE(needsStartAlign(0.5, 0.5));
}

// ---------------- 原地旋转角速度 ----------------

TEST(AlignAngularVelocity, ZeroInsideTolerance)
{
  EXPECT_NEAR(alignAngularVelocity(0.05, 1.0, 1.0, 0.1, 0.10), 0.0, kEps);
  EXPECT_NEAR(alignAngularVelocity(-0.05, 1.0, 1.0, 0.1, 0.10), 0.0, kEps);
}

TEST(AlignAngularVelocity, SignFollowsError)
{
  EXPECT_GT(alignAngularVelocity(0.8, 1.0, 1.0, 0.05, 0.1), 0.0);
  EXPECT_LT(alignAngularVelocity(-0.8, 1.0, 1.0, 0.05, 0.1), 0.0);
}

TEST(AlignAngularVelocity, ClampsToMax)
{
  EXPECT_NEAR(alignAngularVelocity(3.0, 2.0, 0.6, 0.05, 0.1), 0.6, kEps);
  EXPECT_NEAR(alignAngularVelocity(-3.0, 2.0, 0.6, 0.05, 0.1), -0.6, kEps);
}

TEST(AlignAngularVelocity, AppliesFloorSoItActuallyMoves)
{
  // 比例律算出 0.02，低于底盘能驱动的下限；必须抬到 floor。
  const double v = alignAngularVelocity(0.02, 1.0, 1.0, 0.15, 0.01);
  EXPECT_NEAR(v, 0.15, kEps);
}

TEST(AlignAngularVelocity, FloorNeverExceedsMax)
{
  // 哨兵：floor 与 max 冲突时，max 必须赢 —— 否则会下发超过底盘上限的指令。
  const double v = alignAngularVelocity(0.5, 1.0, 0.2, 0.9, 0.01);
  EXPECT_NEAR(v, 0.2, kEps);
}

TEST(AlignAngularVelocity, ProportionalInMidRange)
{
  const double v = alignAngularVelocity(0.4, 0.5, 1.0, 0.0, 0.01);
  EXPECT_NEAR(v, 0.2, kEps);
}

TEST(AlignAngularVelocity, TaperDownNearTargetLimitsOvershoot)
{
  // 实测原地旋转发零速后仍余转 0.006(慢) ~ 0.033(快) rad，
  // 所以靠近目标时输出必须变小，不能一路满速冲到容差边界。
  const double far_v = std::fabs(alignAngularVelocity(1.2, 0.5, 1.0, 0.0, 0.02));
  const double near_v = std::fabs(alignAngularVelocity(0.1, 0.5, 1.0, 0.0, 0.02));
  EXPECT_LT(near_v, far_v);
}

// ---------------- 相位推进 ----------------

TEST(AdvancePhase, StartAlignHoldsUntilAligned)
{
  EXPECT_EQ(
    advancePhase(Phase::kAlignStart, 1.0, 0.0, 5.0, 0.18, 0.05, 0.20, true),
    Phase::kAlignStart);
}

TEST(AdvancePhase, StartAlignReleasesWhenWithinTolerance)
{
  EXPECT_EQ(
    advancePhase(Phase::kAlignStart, 0.03, 0.0, 5.0, 0.18, 0.05, 0.20, true),
    Phase::kFollow);
}

TEST(AdvancePhase, StartAlignSkippedWhenErrorBelowMinAngle)
{
  // 误差 0.1 < min 0.20：不值得原地转，直接跟踪。
  EXPECT_EQ(
    advancePhase(Phase::kAlignStart, 0.1, 0.0, 5.0, 0.18, 0.05, 0.20, true),
    Phase::kFollow);
}

TEST(AdvancePhase, FollowHoldsWhileFarFromGoal)
{
  EXPECT_EQ(
    advancePhase(Phase::kFollow, 0.0, 1.0, 2.0, 0.18, 0.05, 0.20, true),
    Phase::kFollow);
}

TEST(AdvancePhase, FollowEntersGoalAlignWhenYawOff)
{
  EXPECT_EQ(
    advancePhase(Phase::kFollow, 0.0, 1.0, 0.10, 0.18, 0.05, 0.20, true),
    Phase::kAlignGoal);
}

TEST(AdvancePhase, FollowGoesDoneWhenYawAlreadyGood)
{
  EXPECT_EQ(
    advancePhase(Phase::kFollow, 0.0, 0.01, 0.10, 0.18, 0.05, 0.20, true),
    Phase::kDone);
}

TEST(AdvancePhase, ExplorationNeverRotatesAtGoal)
{
  // 哨兵（探索场景的硬要求）：align_goal_enabled=false 时，
  // 无论终点姿态误差多大，都不得进入 kAlignGoal。
  for (const double err : {0.0, 0.5, 1.5, 3.0, -0.5, -3.0}) {
    EXPECT_EQ(
      advancePhase(Phase::kFollow, 0.0, err, 0.10, 0.18, 0.05, 0.20, false),
      Phase::kDone) << "goal_error=" << err;
  }
}

TEST(AdvancePhase, GoalAlignCollapsesIfDisabledMidFlight)
{
  // 运行期把开关关掉（改参数）时，已经在 kAlignGoal 的也必须立刻收敛，
  // 不能继续原地转。
  EXPECT_EQ(
    advancePhase(Phase::kAlignGoal, 0.0, 2.0, 0.05, 0.18, 0.05, 0.20, false),
    Phase::kDone);
}

TEST(AdvancePhase, GoalAlignHoldsUntilYawWithinTolerance)
{
  EXPECT_EQ(
    advancePhase(Phase::kAlignGoal, 0.0, 0.5, 0.05, 0.18, 0.05, 0.20, true),
    Phase::kAlignGoal);
  EXPECT_EQ(
    advancePhase(Phase::kAlignGoal, 0.0, 0.02, 0.05, 0.18, 0.05, 0.20, true),
    Phase::kDone);
}

TEST(AdvancePhase, DoneIsTerminal)
{
  EXPECT_EQ(
    advancePhase(Phase::kDone, 3.0, 3.0, 9.0, 0.18, 0.05, 0.20, true),
    Phase::kDone);
}

TEST(AdvancePhase, ToleranceComesFromCaller)
{
  // 哨兵：位置容差必须是传入的那个（nav2 GoalChecker 给的），
  // 函数内部不得自带一份。同样的距离，容差不同结论就该不同。
  EXPECT_EQ(
    advancePhase(Phase::kFollow, 0.0, 1.0, 0.15, 0.10, 0.05, 0.20, true),
    Phase::kFollow);  // 0.15 > 0.10 -> 继续跟踪
  EXPECT_EQ(
    advancePhase(Phase::kFollow, 0.0, 1.0, 0.15, 0.25, 0.05, 0.20, true),
    Phase::kAlignGoal);  // 0.15 <= 0.25 -> 位置已到
}

// ---------------- 同一目标判定（1Hz 重规划抖动的修复）----------------

TEST(IsSameGoal, SameEndpointWithinEpsilon)
{
  EXPECT_TRUE(astribot_s1_path_tracking::isSameGoal({1.0, 2.0}, {1.05, 2.05}, 0.25));
}

TEST(IsSameGoal, DifferentEndpointBeyondEpsilon)
{
  EXPECT_FALSE(astribot_s1_path_tracking::isSameGoal({1.0, 2.0}, {1.0, 3.0}, 0.25));
}

TEST(IsSameGoal, BoundaryIsSame)
{
  EXPECT_TRUE(astribot_s1_path_tracking::isSameGoal({0.0, 0.0}, {0.25, 0.0}, 0.25));
}

TEST(IsSameGoal, InvalidEpsilonIsConservativelyDifferent)
{
  // 哨兵：阈值非法时必须判「不同目标」。判成"同一目标"会跳过起步对齐，
  // 而那正是需求 3(a) 要的行为 —— 静默跳过比多转一次危险得多。
  EXPECT_FALSE(astribot_s1_path_tracking::isSameGoal({1.0, 1.0}, {1.0, 1.0}, 0.0));
  EXPECT_FALSE(astribot_s1_path_tracking::isSameGoal({1.0, 1.0}, {1.0, 1.0}, -1.0));
}

TEST(PhaseName, AllPhasesHaveNames)
{
  EXPECT_STREQ(toString(Phase::kAlignStart), "ALIGN_START");
  EXPECT_STREQ(toString(Phase::kFollow), "FOLLOW");
  EXPECT_STREQ(toString(Phase::kAlignGoal), "ALIGN_GOAL");
  EXPECT_STREQ(toString(Phase::kDone), "DONE");
}
// ============ 相位计时器重置（曾经永久锁死整套导航的那条不变式）============
//
// 实测缺陷：enterPhase 里"相位值没变就 return"，于是上一个目标在 ALIGN_START
// 段被中止、新目标又要求进 ALIGN_START 时，计时器继承旧值 ——
// 新目标第一拍就判超时(日志读到 1108.297s > 15s)、抛异常、abort，
// 之后每个目标都瞬间失败且永不恢复。恢复探索后实测 12 个目标 12 个失败、
// 0 次进度停滞（机器人根本没开始动）。

TEST(PhaseTimer, PhaseChangeAlwaysRestarts) {
  // 相位真的变了，无论是不是新目标都要重置
  EXPECT_TRUE(shouldRestartPhaseTimer(Phase::kAlignStart, Phase::kFollow, false));
  EXPECT_TRUE(shouldRestartPhaseTimer(Phase::kFollow, Phase::kAlignGoal, false));
  EXPECT_TRUE(shouldRestartPhaseTimer(Phase::kAlignStart, Phase::kFollow, true));
}

TEST(PhaseTimer, NewGoalRestartsEvenWhenPhaseUnchanged) {
  // !!! 这条就是那个 bug 的哨兵 !!!
  // 新目标要求进的相位与当前相位相同（上一个目标死在 ALIGN_START 里），
  // 必须重置 —— 否则新目标继承旧计时器，第一拍即超时。
  EXPECT_TRUE(shouldRestartPhaseTimer(Phase::kAlignStart, Phase::kAlignStart, true));
  EXPECT_TRUE(shouldRestartPhaseTimer(Phase::kFollow, Phase::kFollow, true));
  EXPECT_TRUE(shouldRestartPhaseTimer(Phase::kAlignGoal, Phase::kAlignGoal, true));
}

TEST(PhaseTimer, SameGoalReplanDoesNotRestart) {
  // 反向对照：同一目标的周期性重规划**不能**重置，
  // 否则对齐段永远等不到超时，align_timeout 保护形同虚设。
  EXPECT_FALSE(shouldRestartPhaseTimer(Phase::kAlignStart, Phase::kAlignStart, false));
  EXPECT_FALSE(shouldRestartPhaseTimer(Phase::kFollow, Phase::kFollow, false));
}

TEST(PhaseTimer, DoneIsNotSpecialCased) {
  // kDone 也走同一套规则，不要给它开后门
  EXPECT_FALSE(shouldRestartPhaseTimer(Phase::kDone, Phase::kDone, false));
  EXPECT_TRUE(shouldRestartPhaseTimer(Phase::kDone, Phase::kDone, true));
  EXPECT_TRUE(shouldRestartPhaseTimer(Phase::kDone, Phase::kAlignStart, false));
}


