// Copyright 2026 Astribot
//
// scan 时效性守护策略的边界测试。
//
// 这两个函数每一条分支都对应一次实测事故或一次已犯过的错误，不是补覆盖率：
//   · tfLookupWait      —— 2026-09-01 急停后 36×50ms 串联放大成 1.8s
//   · holdLastDecision  —— 无上限的 hold_last 让陈旧对所有时效性判据隐身

#include <gtest/gtest.h>

#include "astribot_s1_autonomy/scan_guard_policy.hpp"

using astribot_s1_autonomy::HoldLastAction;
using astribot_s1_autonomy::holdLastDecision;
using astribot_s1_autonomy::tfLookupWait;

// ───────────────────────── tfLookupWait ─────────────────────────

TEST(TfLookupWait, HealthyPathIsUnchangedByBudget)
{
  // 最重要的一条：健康态行为必须与"加预算之前"完全一致。
  // 健康态每次查询立即返回，elapsed 几乎不涨，所以恒返回单次超时。
  EXPECT_DOUBLE_EQ(0.05, tfLookupWait(0.05, 0.06, 0.0));
  EXPECT_DOUBLE_EQ(0.05, tfLookupWait(0.05, 0.06, 0.0001));
}

TEST(TfLookupWait, FirstFailureIsAllowedToWaitFull)
{
  // 预算 0.06 > 单次 0.05：第一个失败的连杆允许等满一次。
  // 这保留了"某个连杆偶发丢失时其余仍正常剔除"的原有语义。
  EXPECT_DOUBLE_EQ(0.05, tfLookupWait(0.05, 0.06, 0.0));
}

TEST(TfLookupWait, SecondFailureIsClampedByRemainingBudget)
{
  // 第一个连杆等满 0.05 之后，剩余预算只有 0.01。
  EXPECT_DOUBLE_EQ(0.01, tfLookupWait(0.05, 0.06, 0.05));
}

TEST(TfLookupWait, ExhaustedBudgetReturnsZeroNotNegative)
{
  // 返回 0 的语义是"仍然查、但不等"，不是"跳过不查"。
  // 返回负数会被 tf2::durationFromSec 变成负超时，行为未定义。
  EXPECT_DOUBLE_EQ(0.0, tfLookupWait(0.05, 0.06, 0.06));
  EXPECT_DOUBLE_EQ(0.0, tfLookupWait(0.05, 0.06, 5.0));
  EXPECT_GE(tfLookupWait(0.05, 0.06, 1000.0), 0.0);
}

TEST(TfLookupWait, TheActualIncidentIsBounded)
{
  // 复现事故的算术：36 个连杆全部查不到。
  // 旧行为 = 36 × 0.05 = 1.80s（实测自滤级增量 1.91s，其中 94.2% 是纯等待）
  // 新行为 = 总等待不超过预算
  const double per = 0.05, budget = 0.06;
  double elapsed = 0.0;
  for (int i = 0; i < 36; ++i) {
    elapsed += tfLookupWait(per, budget, elapsed);   // 失败即等满允许值
  }
  EXPECT_LE(elapsed, budget + 1e-9) << "36 个连杆的总等待必须被预算封住";
  EXPECT_LT(elapsed, 0.10) << "必须远小于雷达帧周期 100ms，否则仍会掉帧";
  // 并且确实比旧行为小一个量级以上
  EXPECT_LT(elapsed, 36 * per / 10.0);
}

TEST(TfLookupWait, ZeroBudgetMeansCacheOnly)
{
  // 预算 0 是合法配置：所有查询都不等，只吃 tf 缓存。
  EXPECT_DOUBLE_EQ(0.0, tfLookupWait(0.05, 0.0, 0.0));
}

// ─────────────────────── holdLastDecision ───────────────────────

TEST(HoldLastDecision, StopOutputPolicyNeverRepublishes)
{
  EXPECT_EQ(HoldLastAction::kStopOutput, holdLastDecision(false, 0, 5));
  EXPECT_EQ(HoldLastAction::kStopOutput, holdLastDecision(false, 99, 5));
}

TEST(HoldLastDecision, TransientHiccupIsHeld)
{
  // 单帧/几帧抖动仍然保持上一帧 —— 这是 hold_last 存在的正当理由
  // （别让 costmap 因为一次传感器抖动把已知障碍物清空）。
  for (int streak = 0; streak < 5; ++streak) {
    EXPECT_EQ(HoldLastAction::kRepublish, holdLastDecision(true, streak, 5))
      << "streak=" << streak;
  }
}

TEST(HoldLastDecision, TripsExactlyAtTheLimitNotAfter)
{
  // 边界必须是 >=，不是 >。写成 > 会多重发一帧，
  // 而"多一帧"在 10Hz 下就是多 100ms 的隐身时间。
  EXPECT_EQ(HoldLastAction::kRepublish, holdLastDecision(true, 4, 5));
  EXPECT_EQ(HoldLastAction::kStopOutput, holdLastDecision(true, 5, 5));
  EXPECT_EQ(HoldLastAction::kStopOutput, holdLastDecision(true, 6, 5));
}

TEST(HoldLastDecision, NonPositiveLimitMeansUnlimited)
{
  // <=0 = 不限制。保留这个配置是为了兼容，但它会让陈旧永久隐身。
  EXPECT_EQ(HoldLastAction::kRepublish, holdLastDecision(true, 100000, 0));
  EXPECT_EQ(HoldLastAction::kRepublish, holdLastDecision(true, 100000, -1));
}

TEST(HoldLastDecision, LimitOfOneAllowsExactlyOneRepublish)
{
  EXPECT_EQ(HoldLastAction::kRepublish, holdLastDecision(true, 0, 1));
  EXPECT_EQ(HoldLastAction::kStopOutput, holdLastDecision(true, 1, 1));
}

TEST(HoldLastDecision, MaxBlindTimeIsBounded)
{
  // 把"最长隐身时间"算出来并钉住：上限 5 帧 @10Hz = 0.5s。
  // 它必须大于真实抖动(1~2 帧)，又必须远小于会造成危险的秒级陈旧。
  const int limit = 5;
  const double frame_period = 0.1;
  int republished = 0;
  for (int streak = 0; streak < 100; ++streak) {
    if (holdLastDecision(true, streak, limit) == HoldLastAction::kRepublish) {
      ++republished;
    } else {
      break;
    }
  }
  EXPECT_EQ(limit, republished);
  EXPECT_DOUBLE_EQ(0.5, republished * frame_period);
}
