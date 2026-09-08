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
#include <limits>
#include <vector>

#include "astribot_s1_path_tracking/align_math.hpp"

using astribot_s1_path_tracking::Phase;
using astribot_s1_path_tracking::PlanarPoint;
using astribot_s1_path_tracking::advancePhase;
using astribot_s1_path_tracking::alignAngularVelocity;
using astribot_s1_path_tracking::approachSpeedCap;
// 只吃 bool/double，没有本命名空间的实参 => ADL 找不到它，必须显式 using。
// （上面那批之所以不用，是因为参数里带 Phase/PlanarPoint，ADL 自动生效。）
using astribot_s1_path_tracking::isFreshFollowAttempt;
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

// 这个用例原来叫 DoneIsTerminal，断言 dist=9.0(远远超出容差 0.18) 时仍留在
// kDone —— 它把一个真 bug 锁死成了"规范"。实测后果：远端目标那一腿在 t+45.5s
// 打出 "ALIGN_GOAL -> DONE"，随后 135s 内 /cmd_vel 非零线速度 **0 帧**、
// max‖v‖=0.0000、净位移 0.048m，nav2 从未打出 "Reached the goal!"，整段跑到
// 180s 被主动取消。所以断言反过来写：kDone 必须可撤回。
TEST(AdvancePhase, DoneIsRevocableWhenPathEndMovesAway)
{
  // 路径末端跑远了（1Hz 重规划换路径、恢复行为把机器人转走都会造成这个）
  // -> 必须回到 kFollow 继续开，不能停在 kDone 出零速。
  EXPECT_EQ(
    advancePhase(Phase::kDone, 3.0, 3.0, 9.0, 0.18, 0.05, 0.20, true),
    Phase::kFollow);
  // 刚超出容差一点也要退出：退出阈值不许比 GoalChecker 的 xy 容差宽，
  // 否则 (xy_tol, exit_tol] 就是一段死区，本层停车而 GoalChecker 不认账。
  EXPECT_EQ(
    advancePhase(Phase::kDone, 0.0, 0.0, 0.1801, 0.18, 0.05, 0.20, true),
    Phase::kFollow);
  // 仍在容差内则保持 kDone（出零速，把到位裁决交给 GoalChecker）。
  EXPECT_EQ(
    advancePhase(Phase::kDone, 3.0, 3.0, 0.18, 0.18, 0.05, 0.20, true),
    Phase::kDone);
  EXPECT_EQ(
    advancePhase(Phase::kDone, 3.0, 3.0, 0.0, 0.18, 0.05, 0.20, true),
    Phase::kDone);
  // 退出判据与 kFollow 的进入判据必须是同一个不等式的两侧（死区为空集）:
  // 对同一个 dist，两个相位的裁决必须一致地"要不要继续开"。
  for (double d : {0.0, 0.05, 0.1799, 0.18, 0.1801, 0.5, 9.0}) {
    const bool follow_keeps_driving =
      advancePhase(Phase::kFollow, 0.0, 0.0, d, 0.18, 0.05, 0.20, true) == Phase::kFollow;
    const bool done_resumes_driving =
      advancePhase(Phase::kDone, 0.0, 0.0, d, 0.18, 0.05, 0.20, true) == Phase::kFollow;
    EXPECT_EQ(follow_keeps_driving, done_resumes_driving) << "dist=" << d;
  }
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

// ============ 「同一目标被重新下发」= 新一次尝试（2026-09-04 实机根因）============
//
// 上面那三条只覆盖了「新目标」。实机死在另一半：**同一个目标**被协调器
// 重新下发（上一次 FollowPath 已 abort），setPlan 判出 same_goal 就提前
// return，enterPhase 压根没被调用，计时器继续沿用旧值。
// 实测：734.0s 最后一次真·新目标之后，同一目标 (1.68, 8.43) 每 1.5s 重下发，
// 计时器单调爬到 110.708s，每条新路径第一拍即超时 -> patience exceeded ×43
// -> abort ×48 -> 上游 3 连败 -> PAUSED -> 自动恢复 3 次全在 3s 内再死。
//
// 判据只能看时间空档：action 存续期间 nav2 会以 controller_frequency 持续
// tick，action 一结束 tick 就停。

TEST(FreshAttempt, FirstEverSetPlanIsFresh) {
  // 从没 tick 过 = 第一次下发。idle_gap 此时无意义，给什么都不该改变结论。
  EXPECT_TRUE(isFreshFollowAttempt(false, 0.0, 0.5));
  EXPECT_TRUE(isFreshFollowAttempt(false, 999.0, 0.5));
  EXPECT_TRUE(isFreshFollowAttempt(false, -1.0, 0.5));
}

TEST(FreshAttempt, GapLongerThanThresholdIsFresh) {
  // !!! 这条是实机那个锁死的哨兵 !!!
  // 上一个 action 已 abort，tick 停了；协调器隔 1.5s 重下发同一目标。
  // 1.5s > 0.5s => 必须判为新尝试，给一份完整的 15s 对齐预算。
  EXPECT_TRUE(isFreshFollowAttempt(true, 1.5, 0.5));
  EXPECT_TRUE(isFreshFollowAttempt(true, 0.51, 0.5));
  // 实机那次 PAUSED 冷却是 10.2s，更不该被误判成"还在同一个 action 里"
  EXPECT_TRUE(isFreshFollowAttempt(true, 10.2, 0.5));
}

TEST(FreshAttempt, TickGapWithinActionIsNotFresh) {
  // 反向对照：action 存续期间的周期重规划。20Hz 下相邻 tick 间隔 0.05s，
  // 抖动到 0.2s 也仍在同一个 action 里 —— 判成新尝试就会让 align_timeout
  // 永远等不到触发，等于把"原地转不动"这道保护关掉。
  EXPECT_FALSE(isFreshFollowAttempt(true, 0.05, 0.5));
  EXPECT_FALSE(isFreshFollowAttempt(true, 0.2, 0.5));
  EXPECT_FALSE(isFreshFollowAttempt(true, 0.5, 0.5));   // 恰好等于阈值：不算空档
}

TEST(FreshAttempt, IllegalThresholdFailsClosed) {
  // 阈值非法时按保守方向（不重置）。宁可少重置一次 —— 对齐段仍会在 15s 后
  // 正常超时；反过来把每次重规划都当新尝试则是把保护彻底关掉。
  EXPECT_FALSE(isFreshFollowAttempt(true, 100.0, 0.0));
  EXPECT_FALSE(isFreshFollowAttempt(true, 100.0, -1.0));
  // 但"从没 tick 过"优先于阈值检查：那确实是第一次下发。
  EXPECT_TRUE(isFreshFollowAttempt(false, 0.0, 0.0));
}

TEST(FreshAttempt, NanGapFailsClosed) {
  // NaN 的任何比较都是 false，于是自然落到"不是新尝试"，与上面同向。
  // 显式钉住：时钟跳变造出 NaN 时不许把保护关掉。
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isFreshFollowAttempt(true, nan, 0.5));
}

// ============ 接近段线性限速（提升到位精度）============
//
// 主因是 **0.55s 未建模死时间**，不是路径跟不准：
//   run11 18 轮实测 cross_track_p95 = 0.048m，而 arrival_error_xy p50 = 0.121m、
//   overshoot_radial p50 = 0.146m —— 误差集中在终段。
//   vel_track_best_lag_s p50 = 0.55 / max 0.64，vel_track_gain = 0.977
//   ⇒ 底盘最终跟得上，只是慢半拍。终段过冲 ≈ v_接近 × τ。
// nav2 MPPI 没有 dead-time 参数，改代价权重动不了这个乘积，只能压小 v_接近。

namespace
{
// 生产取值，与 nav2_params_mppi.yaml 的三段式实例一致。
// 这里刻意抄一份**是为了让单测能独立表达算术**；yaml 与实现不许漂移这件事
// 由 astribot_s1_navigation/test/test_config_consistency.py 从 yaml 直接读值来守。
constexpr double kApproachDist = 1.50;
constexpr double kApproachVMin = 0.05;
constexpr double kNominal = 1.0;        // inner.vx_max
constexpr double kLagSec = 0.55;        // vel_track_best_lag_s p50（实测）
constexpr double kXyTol = 0.18;         // general_goal_checker.xy_goal_tolerance
}  // namespace

TEST(ApproachSpeedCap, NoLimitOutsideConvergenceZone) {
  // 收敛区之外必须逐字返回内层的速度 —— 限速只影响最后 1.5m，
  // 不许把整段行程拖慢（反向守卫：duration_s / traveled_m 不能恶化）。
  EXPECT_DOUBLE_EQ(kNominal, approachSpeedCap(5.0, kApproachDist, kApproachVMin, kNominal));
  EXPECT_DOUBLE_EQ(kNominal, approachSpeedCap(1.6, kApproachDist, kApproachVMin, kNominal));
  // 边界：d == D 时还不限速（区间是 [0, D)）
  EXPECT_DOUBLE_EQ(kNominal, approachSpeedCap(kApproachDist, kApproachDist, kApproachVMin, kNominal));
}

TEST(ApproachSpeedCap, LinearInsideZone) {
  EXPECT_DOUBLE_EQ(0.5, approachSpeedCap(0.75, kApproachDist, kApproachVMin, kNominal));
  EXPECT_DOUBLE_EQ(0.2, approachSpeedCap(0.30, kApproachDist, kApproachVMin, kNominal));
  EXPECT_DOUBLE_EQ(0.12, approachSpeedCap(0.18, kApproachDist, kApproachVMin, kNominal));
}

TEST(ApproachSpeedCap, FloorKeepsChassisMoving) {
  // 线性律在 d < D·v_min/v0 = 0.075m 处算出的值小于 v_min，被下限抬起来。
  // 下限存在的理由：实测底盘 0.02 m/s 就能平动（无静摩擦地板），
  // 0.05 有 2.5 倍余量；没有下限则终段会算出小到驱动不了底盘的值而卡死。
  EXPECT_DOUBLE_EQ(kApproachVMin, approachSpeedCap(0.05, kApproachDist, kApproachVMin, kNominal));
  EXPECT_DOUBLE_EQ(kApproachVMin, approachSpeedCap(0.0, kApproachDist, kApproachVMin, kNominal));
  EXPECT_DOUBLE_EQ(kApproachVMin, approachSpeedCap(-0.1, kApproachDist, kApproachVMin, kNominal));
}

TEST(ApproachSpeedCap, ResidualOvershootMeetsAccuracyTarget) {
  // 算术判据一：终段残留过冲 = v_min·τ，必须落在 0.05m 目标内。
  EXPECT_LT(kApproachVMin * kLagSec, 0.05);      // 0.05×0.55 = 0.0275

  // 算术判据二（更强）：nav2 判到位那一拍（d = xy_tol）的滑行量必须**小于容差本身**，
  // 否则机器人会从"刚判到位"的位置一路冲过目标点。
  const double v_at_tol = approachSpeedCap(kXyTol, kApproachDist, kApproachVMin, kNominal);
  EXPECT_LT(v_at_tol * kLagSec, kXyTol);         // 0.12×0.55 = 0.066 < 0.18

  // 反证：不限速时同一拍的滑行量 1.0×0.55 = 0.55m，是容差的 3 倍 ——
  // 这就是 run11 overshoot p50=0.146 的来源方向（实际接近速度约 0.27 m/s）。
  EXPECT_GT(kNominal * kLagSec, kXyTol);
}

TEST(ApproachSpeedCap, ImpliedDecelWithinMeasuredChassisLimit) {
  // 线性律的隐含最大减速在 d=D 处 = v0²/D。必须 ≤ 实测有效减速 0.45~0.8 m/s²
  // （由滑行量反推 v²/2d：0.069m@0.25m/s ⇒ 0.45，0.100m@0.40m/s ⇒ 0.8）。
  // 否则又变成"规划了做不到的刹车"，限速本身就成了新的模型失配。
  EXPECT_LE(kNominal * kNominal / kApproachDist, 0.8);   // 1.0/1.5 = 0.67
  // 哨兵：若有人把 D 收到 1.0，隐含减速 1.0 m/s² 已超出底盘能力 —— 本条会失败。
}

TEST(ApproachSpeedCap, MonotoneNonDecreasingInDistance) {
  // 单调性保证不会出现"越靠近反而允许更快"的反向段。
  double prev = 0.0;
  for (double d = 0.0; d <= 2.0; d += 0.01) {
    const double v = approachSpeedCap(d, kApproachDist, kApproachVMin, kNominal);
    EXPECT_GE(v, prev - 1e-12) << "d=" << d;
    prev = v;
  }
}

TEST(ApproachSpeedCap, NeverExceedsNominal) {
  // 内层本来就在慢速走（窄通道邻域、setSpeedLimit 生效等）时，
  // v_min 不许把它抬快 —— 下限只是"能动"，不是"至少这么快"。
  EXPECT_DOUBLE_EQ(0.03, approachSpeedCap(0.02, kApproachDist, kApproachVMin, 0.03));
  EXPECT_DOUBLE_EQ(0.03, approachSpeedCap(1.0, kApproachDist, kApproachVMin, 0.03));
}

TEST(ApproachSpeedCap, IllegalParamsDegradeToNoLimit) {
  // 参数非法时退化成"不限速"= 今天的行为，而不是限到 v_min ——
  // 后者会因为一个坏参数把机器人静默限到蠕行，比不限速危险得多。
  // 合法性由 configure() 的启动期 throw 负责（拒绝启动，不静默回退）。
  EXPECT_DOUBLE_EQ(kNominal, approachSpeedCap(0.1, 0.0, kApproachVMin, kNominal));
  EXPECT_DOUBLE_EQ(kNominal, approachSpeedCap(0.1, -1.5, kApproachVMin, kNominal));
  EXPECT_DOUBLE_EQ(kNominal, approachSpeedCap(0.1, kApproachDist, -0.05, kNominal));
  // nominal <= 0：内层已经在发零速，没有可限的东西
  EXPECT_DOUBLE_EQ(0.0, approachSpeedCap(0.1, kApproachDist, kApproachVMin, 0.0));
}

TEST(ApproachSpeedCap, NanDistanceDegradesToNoLimit) {
  // NaN 的比较全为 false，自然落到"不限速"这一支，与非法参数同向。
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(kNominal, approachSpeedCap(nan, kApproachDist, kApproachVMin, kNominal));
}



