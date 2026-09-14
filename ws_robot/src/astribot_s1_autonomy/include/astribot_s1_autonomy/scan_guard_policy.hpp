// Copyright 2026 Astribot
#ifndef ASTRIBOT_S1_AUTONOMY__SCAN_GUARD_POLICY_HPP_
#define ASTRIBOT_S1_AUTONOMY__SCAN_GUARD_POLICY_HPP_

#include <algorithm>

namespace astribot_s1_autonomy
{

/// TF 查询预算与 hold_last 上限这两处**纯算术**从节点里抽出来单独放，理由与

/// 算出"本次 TF 查询允许等多久"。
/// \param per_lookup_timeout 单次查询超时(s)，即 tf_timeout_sec。
/// \param total_budget       本帧所有查询的总预算(s)，即 tf_total_budget_sec。
/// \param elapsed            本帧到目前为止已经等掉的时间(s)。
inline double tfLookupWait(
  double per_lookup_timeout, double total_budget, double elapsed)
{
  const double remain = total_budget - elapsed;
  if (remain <= 0.0) {
    return 0.0;
  }
  return std::min(per_lookup_timeout, remain);
}

/// 无效帧时该怎么办。
enum class HoldLastAction
{
  kRepublish,    ///< 重发上一帧(仅刷新时间戳)
  kStopOutput,   ///< 不发 —— 让下游的超时判据能真正生效
};

/// \param hold_last_enabled 策略是否为 hold_last。
/// \param streak            **连续**重发了多少帧(成功发布一帧就归零)。
/// \param max_frames        连续重发上限；<=0 表示不限制。
inline HoldLastAction holdLastDecision(
  bool hold_last_enabled, int streak, int max_frames)
{
  if (!hold_last_enabled) {
    return HoldLastAction::kStopOutput;
  }
  if (max_frames > 0 && streak >= max_frames) {
    return HoldLastAction::kStopOutput;
  }
  return HoldLastAction::kRepublish;
}

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__SCAN_GUARD_POLICY_HPP_
