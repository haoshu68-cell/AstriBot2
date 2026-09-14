// Copyright 2026 Astribot

#ifndef ASTRIBOT_S1_MANIPULATION__SINGULARITY_MONITOR_HPP_
#define ASTRIBOT_S1_MANIPULATION__SINGULARITY_MONITOR_HPP_

#include <string>
#include <vector>

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>

namespace astribot_s1_manipulation
{

/// 奇异检测阈值，全部来自 yaml。
struct SingularityParams
{
  /// 总开关。关掉后 check() 直接返回"非奇异"，但仍然填充奇异值供观察。
  bool enabled{true};

  /// 最小奇异值下限。低于此值判奇异。
  /// 量纲：末端线速度/关节角速度，即 m/rad（旋转行为 rad/rad）。
  /// 取值参考：本机臂展约 0.8m，完全伸直时 σ_min 会掉到 1e-3 量级，
  /// 正常工作构型在 0.05~0.3 之间，所以 0.02 能挡住真奇异又不误杀。
  double min_singular_value{0.02};

  /// 条件数上限。超过此值判奇异（数值病态）。
  double max_condition_number{80.0};

  /// σ_max 本身的下限。σ_max 都接近 0 说明整个雅可比退化（例如把
  /// 不成链的组传进来导致空矩阵），这种情况直接判奇异而不是去算条件数
  /// （会除零）。这是数值稳定性保护，不是业务阈值。
  double degenerate_jacobian_epsilon{1e-9};

  /// 允许轨迹**开头一段**处于奇异构型（"逃离奇异"豁免）。
  bool allow_singular_start{true};
};

/// 单个构型的奇异检测结果。即使判定为非奇异也填满数值，便于观察轨迹上的
/// 奇异值变化趋势（demo 节点会打印全轨迹的最小 σ_min）。
struct SingularityReport
{
  bool singular{false};
  bool valid{false};            ///< 雅可比是否成功算出（false 时其余字段无意义）
  double min_singular_value{0.0};
  double max_singular_value{0.0};
  double condition_number{0.0};
  std::string reason;           ///< 判定依据，用于日志与错误上报
};

/// 奇异点检测器。无状态（除阈值外），线程安全：check() 是 const，
/// 且不修改传入的 RobotState 之外的任何东西。
class SingularityMonitor
{
public:
  SingularityMonitor() = default;

  /// 校验并保存阈值。阈值非法（负数、min >= max 等）时返回 false 并填 error。
  bool configure(const SingularityParams & params, std::string & error);

  bool isConfigured() const noexcept
  {
    return configured_;
  }

  const SingularityParams & params() const noexcept
  {
    return params_;
  }

  /// 检测单个构型。
  /// @param state 待检构型。必须已 update()（内部会自己调一次以保证 FK 最新）。
  /// @param jmg   规划组。必须是链（getJacobian 的前提），双臂组要分别传左右。
  /// @param tip_link_name 求雅可比的参考 link（TCP）。必须属于 jmg。
  /// 任何前置条件不满足都返回 valid=false 的报告，不抛异常、不崩溃。
  SingularityReport check(
    const moveit::core::RobotState & state,
    const moveit::core::JointModelGroup * jmg,
    const std::string & tip_link_name) const;

  /// 检测一串构型，返回最差的那个（σ_min 最小者）。
  /// 用于整条轨迹的奇异校验：任一采样点奇异，整条轨迹就不可用。
  /// @param worst_index 输出最差点的下标；states 为空时不写。
  SingularityReport checkStates(
    const std::vector<moveit::core::RobotState> & states,
    const moveit::core::JointModelGroup * jmg,
    const std::string & tip_link_name,
    std::size_t * worst_index = nullptr) const;

private:
  SingularityParams params_;
  bool configured_{false};
};

}  // namespace astribot_s1_manipulation

#endif  // ASTRIBOT_S1_MANIPULATION__SINGULARITY_MONITOR_HPP_
