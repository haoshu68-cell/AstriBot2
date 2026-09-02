// Copyright 2026 Astribot
#ifndef ASTRIBOT_S1_AUTONOMY__SCAN_GUARD_POLICY_HPP_
#define ASTRIBOT_S1_AUTONOMY__SCAN_GUARD_POLICY_HPP_

#include <algorithm>

namespace astribot_s1_autonomy
{

/// TF 查询预算与 hold_last 上限这两处**纯算术**从节点里抽出来单独放，理由与
/// livox_custom_convert 一样：它们是"错了不会崩、只会静默变成错行为"的那类逻辑，
/// 必须能在没有 ROS、没有机器人的情况下逐个边界测。
///
/// 这两条策略解决的都是同一次实测事故(2026-09-01 21:26 急停)暴露的问题，
/// 背景见 pointcloud_slice_scan_node.hpp 里 tf_total_budget_sec_ /
/// hold_last_max_frames_ 的注释。

/// 算出"本次 TF 查询允许等多久"。
///
/// \param per_lookup_timeout 单次查询超时(s)，即 tf_timeout_sec。
/// \param total_budget       本帧所有查询的总预算(s)，即 tf_total_budget_sec。
/// \param elapsed            本帧到目前为止已经等掉的时间(s)。
/// \return 允许等待的秒数，恒 >= 0。
///
/// 语义要点：预算耗尽返回 0 表示"仍然查、但不等" —— 命中 tf 缓存的连杆照样成功。
/// 所以健康态(每次查询立即返回、elapsed 几乎不涨)下本函数恒返回
/// per_lookup_timeout，行为与加预算之前**完全一致**。
inline double tfLookupWait(
  double per_lookup_timeout, double total_budget, double elapsed)
{
  const double remain = total_budget - elapsed;
  if (remain <= 0.0) {
    return 0.0;
  }
  // 这里**不需要**再套一层 std::max(0.0, ...)：上面已经保证 remain > 0，
  // 而 per_lookup_timeout 由节点侧校验过 >= 0，所以 min 的结果必然 >= 0。
  // （变异测试实证：加上那层 max 是等价变异，12 条测试一条都察觉不到 ——
  //   说明它是死代码，留着只会让人以为下界保护在这一行。）
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
///
/// 为什么 streak 必须是"连续"而不是"累计"：累计计数下，偶发抖动累积到上限后
/// 会永久跳闸，之后每次真实抖动都变成停止输出。本项目在重试上限那条上已经
/// 犯过同一个错(把成功也计入，误杀了完全走得通的长路径)。
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
