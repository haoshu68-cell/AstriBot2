// Copyright 2026 Astribot

#ifndef ASTRIBOT_S1_AUTONOMY__FAILURE_BUDGET_HPP_
#define ASTRIBOT_S1_AUTONOMY__FAILURE_BUDGET_HPP_

namespace astribot_s1_autonomy
{

/// 探索协调器的两个连续失败预算。纯数据 + 纯逻辑，不依赖 rclcpp。
struct ExplorationFailureBudget
{
  int max_sample_failures{4};
  int max_validation_failures{4};

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
