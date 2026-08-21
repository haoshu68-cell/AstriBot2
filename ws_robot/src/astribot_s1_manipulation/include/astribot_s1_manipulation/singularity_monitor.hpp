// Copyright 2026 Astribot
//
// 奇异点检测：用雅可比矩阵的奇异值判断构型是否接近奇异。
//
// 数学含义
// --------
// 雅可比 J(q) 把关节速度映射到末端速度：v = J(q) * q_dot
// 对 J 做 SVD：J = U * S * V^T，S = diag(σ_1 >= σ_2 >= ... >= σ_m)
//
//   · σ_min -> 0 意味着存在某个末端速度方向，需要**无穷大**的关节速度才能实现
//     （即该方向瞬时失去可控性）。这就是奇异构型。
//   · 条件数 κ = σ_max / σ_min 衡量"各方向能力的悬殊程度"。κ 很大时，
//     即使 σ_min 还没到 0，逆解在数值上也已经病态：末端一个小位移误差
//     会被放大成巨大的关节速度指令，真机上表现为突然抖动/超速报警。
//
// 本机器人是 7-DOF 冗余臂，J 是 6x7。奇异并非"J 不可逆"（它本来就不是方阵），
// 而是"J 丢秩"，即 rank(J) < 6 —— 用 σ_min 判定正是这个意思。
// 典型奇异构型：肘部完全伸直（joint_4 = 0，SRDF 的 ready 姿态刻意给了 1.0 避开）、
// 腕部两轴共线。
//
// 为什么两条判据都要
// ------------------
// 只看 σ_min：不同任务尺度下 σ_min 的绝对量级不可比（末端离基座远时整体偏大）。
// 只看 κ：J 整体缩放时 κ 不变，会漏掉"整体能力都很弱"的构型。
// 两条同时用，任一条触发即判奇异 —— 阈值都从 yaml 读，不写死。

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
