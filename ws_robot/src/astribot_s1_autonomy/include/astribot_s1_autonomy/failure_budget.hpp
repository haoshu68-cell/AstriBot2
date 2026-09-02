// Copyright 2026 Astribot
//
// 探索协调器的**失败预算**。从节点里抽出来单独成一个可测单元，原因是
// 这里出过一个只靠读单点赋值绝对看不出来的 bug：
//
//   共用一个计数器时，「采到候选点就清零」这条清零规则，同时清掉了
//   「候选全废」的预算。而「采样 -> 校验全废 -> 重采样」这个循环里，
//   采样**每轮都成功**，于是预算每轮都被清一次，上限结构上永不可达。
//   六轮实测 837 次全部打印 `1/4`，一次没到 2；某轮据此空转 400s、
//   同一候选点被丢弃 760 次、机器人一动不动，而且整条
//   PAUSED -> 自动恢复 -> 脱困 的恢复阶梯全部不可达
//   （升级条件永不成立，连一行 WARN 都不会打，日志上看不出任何异常）。
//
// 所以这里的核心不变式只有一条，并且由单测钉住：
//
//   **每个预算的清零条件，必须是"真的走出了它所限制的那个循环"，
//     而不是"那个循环里某一步成功了"。**
//
// 对应关系：
//   - 采样预算(sample)   限制「反复采不到候选」 -> 采到候选即走出 -> 采样成功清零
//   - 校验预算(validate) 限制「反复校验全废」   -> 目标发出去才算走出 -> 下发成功清零
//
// 注意 validate 预算**不能**在采样成功时清零（那正是原来的 bug），
// 也不能在"某个候选通过校验"时清零 —— 通过校验之后下发仍可能失败
// （动作服务器未就绪、并发下发被拒），那时并没有走出循环。

#ifndef ASTRIBOT_S1_AUTONOMY__FAILURE_BUDGET_HPP_
#define ASTRIBOT_S1_AUTONOMY__FAILURE_BUDGET_HPP_

namespace astribot_s1_autonomy
{

/// 探索协调器的两个连续失败预算。纯数据 + 纯逻辑，不依赖 rclcpp。
struct ExplorationFailureBudget
{
  // ---- 上限（来自参数）----
  int max_sample_failures{4};
  int max_validation_failures{4};

  // ---- 当前连续失败数 ----
  int sample_failures{0};
  int validation_failures{0};

  /// 采到了候选点。**只**清采样预算。
  ///
  /// 刻意不碰 validation_failures：这个函数在「采样->校验全废->重采样」
  /// 循环里每轮都会被调到，碰了就等于把校验预算废掉。
  void onCandidatesSampled();

  /// 本轮一个合法候选都没采到。
  /// @return true 表示预算已用尽，调用方该升级（自举或转 PAUSED）
  [[nodiscard]] bool onNoCandidateSampled();

  /// 本轮采到了候选，但逐个校验后全部不合法。
  /// @return true 表示预算已用尽，调用方该转 PAUSED
  [[nodiscard]] bool onAllCandidatesInvalid();

  /// 目标**真的下发成功**了（动作 goal 已发出）。清校验预算。
  ///
  /// 这是走出「校验全废」循环的唯一标志，所以只有它能清 validation_failures。
  void onGoalDispatched();

  /// 全量重置：自动恢复 / 人工 resume 用。
  void resetAll();
};

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__FAILURE_BUDGET_HPP_
