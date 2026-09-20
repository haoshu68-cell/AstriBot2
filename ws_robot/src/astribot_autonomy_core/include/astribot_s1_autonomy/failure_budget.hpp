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

  /// 采到候选时只清采样预算；保留校验预算，避免重采样循环绕过失败上限。
  void onCandidatesSampled();

  /// 本轮一个合法候选都没采到。
  /// @return true 表示预算已用尽，调用方该转 PAUSED
  [[nodiscard]] bool onNoCandidateSampled();

  /// 本轮采到了候选，但逐个校验后全部不合法。
  /// @return true 表示预算已用尽，调用方该转 PAUSED
  [[nodiscard]] bool onAllCandidatesInvalid();

  /// 动作 goal 已发出时清校验预算。
  void onGoalDispatched();

  /// 全量重置：自动恢复 / 人工 resume 用。
  void resetAll();
};

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__FAILURE_BUDGET_HPP_
