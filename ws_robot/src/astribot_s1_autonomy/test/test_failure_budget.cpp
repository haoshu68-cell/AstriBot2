// Copyright 2026 Astribot
//
// 这组测试的重点不是"计数器会不会加一"，而是**闭环里预算会不会被吃掉**。
// 原来的 bug 用单点断言测不出来：每一次赋值单独看都是对的，
// 错的是"清零条件属于哪个循环"。所以下面的核心用例都是**跑循环**。

#include <gtest/gtest.h>

#include "astribot_s1_autonomy/failure_budget.hpp"

using astribot_s1_autonomy::ExplorationFailureBudget;

namespace
{

/// 实测那个死循环的形状：GEN_NEXT_POINT 采到候选 -> VALIDATING 校验全废 -> 重采样。
/// 每一轮采样都**成功**（这正是关键：采样成功是这个循环的常态，不是例外）。
/// @return 跑到"预算用尽"用了几轮；0 表示跑满 limit 轮仍未用尽（即死循环）
int roundsUntilEscalation(ExplorationFailureBudget & b, int limit)
{
  for (int i = 1; i <= limit; ++i) {
    b.onCandidatesSampled();                    // 采到候选（每轮都成功）
    if (b.onAllCandidatesInvalid()) {           // 校验全废
      return i;
    }
  }
  return 0;
}

}  // namespace

// ===================================================================
// 核心回归：这一条就是那个 400s 空转
// ===================================================================

TEST(ValidationBudget, SampleSuccessMustNotRefillValidationBudget)
{
  // 这是回归本体。共用计数器的那一版在这里会返回 0（永不升级）。
  ExplorationFailureBudget b;
  b.max_validation_failures = 4;
  EXPECT_EQ(roundsUntilEscalation(b, 1000), 4)
    << "采样成功清掉了校验预算 —— 就是实测 837/837 全是 1/4、空转 400s 的根因";
}

TEST(ValidationBudget, CounterActuallyClimbsAcrossRounds)
{
  // 直接盯住实测里"永远是 1"的那个读数。
  ExplorationFailureBudget b;
  b.max_validation_failures = 100;              // 调高，观察爬升而不触发升级
  const int expected[] = {1, 2, 3, 4, 5};
  for (int want : expected) {
    b.onCandidatesSampled();
    EXPECT_FALSE(b.onAllCandidatesInvalid());
    EXPECT_EQ(b.validation_failures, want)
      << "实测这里 837 次全是 1，说明每轮都被清零了";
  }
}

TEST(ValidationBudget, EscalationIsReachableForEveryLegalLimit)
{
  // 穷举合法上限：每一个都必须可达，不能只有某几个值恰好能走通。
  for (int limit = 1; limit <= 32; ++limit) {
    ExplorationFailureBudget b;
    b.max_validation_failures = limit;
    EXPECT_EQ(roundsUntilEscalation(b, 1000), limit) << "上限 " << limit << " 不可达";
  }
}

// ===================================================================
// 清零条件的归属：只有"走出循环"才能清
// ===================================================================

TEST(ValidationBudget, OnlyDispatchClearsIt)
{
  ExplorationFailureBudget b;
  b.max_validation_failures = 4;
  b.onCandidatesSampled();
  EXPECT_FALSE(b.onAllCandidatesInvalid());
  b.onCandidatesSampled();
  EXPECT_FALSE(b.onAllCandidatesInvalid());
  EXPECT_EQ(b.validation_failures, 2);

  b.onGoalDispatched();                         // 真的走出了循环
  EXPECT_EQ(b.validation_failures, 0);
}

TEST(ValidationBudget, DispatchAfterRecoveryGivesFullBudgetAgain)
{
  // 下发成功之后重新陷入循环，必须还有完整的 4 轮预算 —— 不能"一次用尽终身用尽"。
  ExplorationFailureBudget b;
  b.max_validation_failures = 4;
  EXPECT_EQ(roundsUntilEscalation(b, 1000), 4);
  b.onGoalDispatched();
  EXPECT_EQ(roundsUntilEscalation(b, 1000), 4);
}

TEST(ValidationBudget, IsNotClearedByRepeatedSamplingAlone)
{
  // 连续采样很多次都不该动校验预算（哪怕一次校验失败都没发生过之后又发生）。
  ExplorationFailureBudget b;
  b.max_validation_failures = 4;
  b.onCandidatesSampled();
  EXPECT_FALSE(b.onAllCandidatesInvalid());
  for (int i = 0; i < 50; ++i) {
    b.onCandidatesSampled();
  }
  EXPECT_EQ(b.validation_failures, 1) << "采样本身不得回填校验预算";
}

// ===================================================================
// 两个预算必须互相独立（共用就是原来的 bug）
// ===================================================================

TEST(BudgetIndependence, SampleFailuresDoNotConsumeValidationBudget)
{
  ExplorationFailureBudget b;
  b.max_sample_failures = 4;
  b.max_validation_failures = 4;
  for (int i = 0; i < 3; ++i) {
    (void)b.onNoCandidateSampled();
  }
  EXPECT_EQ(b.sample_failures, 3);
  EXPECT_EQ(b.validation_failures, 0);
}

TEST(BudgetIndependence, ValidationFailuresDoNotConsumeSampleBudget)
{
  ExplorationFailureBudget b;
  b.max_sample_failures = 4;
  b.max_validation_failures = 100;
  for (int i = 0; i < 10; ++i) {
    b.onCandidatesSampled();
    (void)b.onAllCandidatesInvalid();
  }
  EXPECT_EQ(b.validation_failures, 10);
  EXPECT_EQ(b.sample_failures, 0);
}

TEST(BudgetIndependence, DispatchDoesNotRefillSampleBudget)
{
  // 下发成功不代表"以后采得到候选"，所以不该清采样预算。
  ExplorationFailureBudget b;
  b.max_sample_failures = 4;
  (void)b.onNoCandidateSampled();
  (void)b.onNoCandidateSampled();
  b.onGoalDispatched();
  EXPECT_EQ(b.sample_failures, 2);
}

TEST(BudgetIndependence, EachBudgetEscalatesOnItsOwnSchedule)
{
  // 采样预算 2、校验预算 5：两者必须各按自己的上限升级，互不干扰。
  ExplorationFailureBudget b;
  b.max_sample_failures = 2;
  b.max_validation_failures = 5;

  b.onCandidatesSampled();
  EXPECT_FALSE(b.onAllCandidatesInvalid());     // validate=1
  EXPECT_FALSE(b.onNoCandidateSampled());       // sample=1
  EXPECT_TRUE(b.onNoCandidateSampled());        // sample=2 -> 升级
  EXPECT_EQ(b.validation_failures, 1) << "采样侧升级不该影响校验预算";
}

// ===================================================================
// 采样预算原有行为不能被我改坏
// ===================================================================

TEST(SampleBudget, SampledSuccessStillClearsIt)
{
  ExplorationFailureBudget b;
  b.max_sample_failures = 4;
  (void)b.onNoCandidateSampled();
  (void)b.onNoCandidateSampled();
  EXPECT_EQ(b.sample_failures, 2);
  b.onCandidatesSampled();
  EXPECT_EQ(b.sample_failures, 0);
}

TEST(SampleBudget, EscalatesExactlyAtTheLimit)
{
  ExplorationFailureBudget b;
  b.max_sample_failures = 3;
  EXPECT_FALSE(b.onNoCandidateSampled());
  EXPECT_FALSE(b.onNoCandidateSampled());
  EXPECT_TRUE(b.onNoCandidateSampled());
}

// ===================================================================
// 全量重置
// ===================================================================

TEST(ResetAll, ClearsBoth)
{
  ExplorationFailureBudget b;
  b.max_sample_failures = 100;
  b.max_validation_failures = 100;
  (void)b.onNoCandidateSampled();
  b.onCandidatesSampled();
  (void)b.onAllCandidatesInvalid();
  (void)b.onAllCandidatesInvalid();
  b.resetAll();
  EXPECT_EQ(b.sample_failures, 0);
  EXPECT_EQ(b.validation_failures, 0);
}

// ===================================================================
// 边界：上限为 1 时必须一次就升级（别退化成"永不升级"）
// ===================================================================

TEST(Boundary, LimitOneEscalatesImmediately)
{
  ExplorationFailureBudget b;
  b.max_validation_failures = 1;
  b.max_sample_failures = 1;
  b.onCandidatesSampled();
  EXPECT_TRUE(b.onAllCandidatesInvalid());

  ExplorationFailureBudget c;
  c.max_sample_failures = 1;
  EXPECT_TRUE(c.onNoCandidateSampled());
}

TEST(Boundary, NonPositiveLimitEscalatesRatherThanLoopingForever)
{
  // 参数校验拦的是 <1，理论上到不了这里；但万一到了，
  // 必须**倒向升级**而不是倒向死循环 —— 死循环没有任何日志、无法排查。
  ExplorationFailureBudget b;
  b.max_validation_failures = 0;
  b.max_sample_failures = 0;
  b.onCandidatesSampled();
  EXPECT_TRUE(b.onAllCandidatesInvalid()) << "非法上限必须失败向升级，不能永不升级";
  EXPECT_TRUE(b.onNoCandidateSampled());

  ExplorationFailureBudget c;
  c.max_validation_failures = -5;
  c.onCandidatesSampled();
  EXPECT_TRUE(c.onAllCandidatesInvalid());
}

// ===================================================================
// 到顶夹紧：读数不得超过分母，但夹紧**不能**破坏升级可达性
// ===================================================================

TEST(Clamp, CounterNeverExceedsItsLimit)
{
  // 实测日志出现过 `连续采样失败 5/4`。夹紧后读数必须停在 4。
  ExplorationFailureBudget b;
  b.max_sample_failures = 4;
  b.max_validation_failures = 4;
  for (int i = 0; i < 20; ++i) {
    (void)b.onNoCandidateSampled();
    b.onCandidatesSampled();                  // 采样成功清采样预算，但…
    (void)b.onAllCandidatesInvalid();
    EXPECT_LE(b.sample_failures, 4);
    EXPECT_LE(b.validation_failures, 4);
  }
}

TEST(Clamp, StillReportsEscalationOnEveryCallOnceAtTheCap)
{
  // 夹紧之后**每一次**调用都必须继续返回 true —— 否则到顶后升级只触发一次，
  // 之后就静默了，等于把上限变成"一次性"。这正是 D 要防的那类静默。
  ExplorationFailureBudget b;
  b.max_validation_failures = 3;
  b.onCandidatesSampled();
  EXPECT_FALSE(b.onAllCandidatesInvalid());
  b.onCandidatesSampled();
  EXPECT_FALSE(b.onAllCandidatesInvalid());
  b.onCandidatesSampled();
  EXPECT_TRUE(b.onAllCandidatesInvalid());    // 到顶
  for (int i = 0; i < 10; ++i) {
    b.onCandidatesSampled();
    EXPECT_TRUE(b.onAllCandidatesInvalid()) << "到顶后第 " << i << " 次不再上报升级";
    EXPECT_EQ(b.validation_failures, 3);
  }
}

TEST(Clamp, DispatchStillClearsAClampedCounter)
{
  ExplorationFailureBudget b;
  b.max_validation_failures = 2;
  b.onCandidatesSampled();
  (void)b.onAllCandidatesInvalid();
  b.onCandidatesSampled();
  (void)b.onAllCandidatesInvalid();
  EXPECT_EQ(b.validation_failures, 2);
  b.onGoalDispatched();
  EXPECT_EQ(b.validation_failures, 0) << "夹紧后仍必须能被下发成功清掉";
  EXPECT_EQ(roundsUntilEscalation(b, 100), 2) << "清零后预算必须完整回来";
}

// ===================================================================
// 长跑：预算不能随轮次漂移
// ===================================================================

TEST(LongRun, BudgetIsStableAcrossManyDispatchCycles)
{
  // 模拟 200 个"陷入循环 -> 升级 -> 恢复下发"的周期，每次都必须正好 4 轮。
  ExplorationFailureBudget b;
  b.max_validation_failures = 4;
  for (int cycle = 0; cycle < 200; ++cycle) {
    ASSERT_EQ(roundsUntilEscalation(b, 100), 4) << "第 " << cycle << " 个周期预算漂了";
    b.onGoalDispatched();
  }
}
