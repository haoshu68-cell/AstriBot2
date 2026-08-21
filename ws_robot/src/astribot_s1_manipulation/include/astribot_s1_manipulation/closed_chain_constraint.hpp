// Copyright 2026 Astribot
//
// 双臂闭链约束。
//
// 物理场景
// --------
// 两条手臂共同夹持同一个刚体（一块板、一个箱子）。物体是刚体，两个夹持点
// 在物体上的位置固定，因此两个末端(TCP)之间的**相对位姿在整个搬运过程中恒定**。
// 这就形成了一个运动学闭链：
//
//     torso_end_effector ─┬─ arm_left  ─→ TCP_L ─┐
//                         └─ arm_right ─→ TCP_R ─┴─ 同一个被夹持刚体
//
// 数学表达
// --------
// 设 T_L(q_L)、T_R(q_R) 为左右 TCP 在同一参考系（机器人根系）下的位姿，
// 闭链约束即：
//
//     T_rel = T_L^{-1} · T_R = const                                  (1)
//
// 等价地，约束残差函数为
//
//     E(q_L, q_R) = T_L(q_L)^{-1} · T_R(q_R) · T_rel^{-1}  =  I        (2)
//
// 残差分两个**量纲不同**的分量，必须分别设阈值，不能加成一个标量：
//     e_p = || translation(E) ||                     单位 m
//     e_r = | angle-axis(rotation(E)) |              单位 rad
// 把 0.005m 和 0.02rad 相加是没有物理意义的。
//
// 为什么不是"两条独立单臂规划"
// ----------------------------
// 独立规划两条臂，(1) 式不会被满足：两臂各走各的最优路径，相对位姿一路漂移，
// 夹持的物体会被拉扯/掉落。任务明确禁止这种做法。
//
// 本实现的做法：leader-follower + IK 投影
// --------------------------------------
// 1. 选一条臂做 leader，用 OMPL 在它自己的 7 维关节空间里规划（自由度低，成功率高）。
// 2. 对 leader 轨迹的**每一个** waypoint：
//      正解得到 T_L → 由 (1) 式算出 follower 必须到达的位姿 T_R = T_L · T_rel
//      → 对 follower 组求 IK 得到 q_R。
// 3. 逐点校验残差 (2)、碰撞、奇异；任一点不过 → 整条丢弃，重规划。
//
// 这样得到的轨迹在**每一个采样点**上都严格满足闭链约束（残差可量化输出），
// 而不是"两条独立轨迹凑在一起"。相比在 14 维空间里做约束流形采样
// (ompl::base::Constraint + ConstrainedPlanningStateSpace)，本方案的优势是
// 确定性强、失败原因可定位（能明确指出是第几个点 IK 失败/残差多大），
// 代价是 leader 的路径不是双臂整体最优。

#ifndef ASTRIBOT_S1_MANIPULATION__CLOSED_CHAIN_CONSTRAINT_HPP_
#define ASTRIBOT_S1_MANIPULATION__CLOSED_CHAIN_CONSTRAINT_HPP_

#include <array>
#include <string>

#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace astribot_s1_manipulation
{

struct ClosedChainParams
{
  /// leader 臂的规划组名（在它的关节空间里跑 OMPL）。
  std::string leader_group{"arm_left"};
  /// follower 臂的规划组名（逐点 IK 跟随）。
  std::string follower_group{"arm_right"};

  /// 两个 TCP 的 link 名。本机器人没有夹爪，默认用 tool_link
  /// （实测这两个 link 无碰撞几何，是纯坐标系，正合适做 TCP）。
  std::string leader_tcp_link{"astribot_arm_left_tool_link"};
  std::string follower_tcp_link{"astribot_arm_right_tool_link"};

  /// T_rel 的来源。
  /// true  = 从"当前机器人状态"捕获（先把两臂摆到夹持位姿，再启动规划）。
  ///         这是推荐做法：夹持几何由实际摆位决定，不需要任何人去量尺寸。
  /// false = 用下面的 relative_* 显式指定。
  bool capture_from_current_state{true};

  /// 显式指定的 T_rel 平移分量 (x, y, z)，单位 m。
  /// 仅在 capture_from_current_state=false 时使用。
  std::array<double, 3> relative_translation{{0.0, 0.0, 0.0}};
  /// 显式指定的 T_rel 旋转四元数 (x, y, z, w)。内部会归一化。
  std::array<double, 4> relative_rotation_xyzw{{0.0, 0.0, 0.0, 1.0}};

  /// 位置残差阈值，单位 m。默认 5mm：对夹持刚体来说，5mm 的相对错位
  /// 已经足以在物体上产生明显内力（取决于夹具刚度），再大就该拒绝了。
  double position_tolerance{0.005};
  /// 姿态残差阈值，单位 rad。默认 0.02rad ≈ 1.15°。
  double orientation_tolerance{0.02};

  /// follower 单次 IK 的超时（秒）与尝试次数。
  /// 逐点 IK 的总开销 = 点数 × attempts × timeout，所以 timeout 要小、
  /// 靠 attempts 换成功率（详见 moveit_config/config/kinematics.yaml 的说明）。
  double ik_timeout{0.01};
  int ik_attempts{3};
};

/// 闭链残差。位置与姿态分开，量纲不同不可混加。
struct ConstraintResidual
{
  bool valid{false};              ///< 是否成功算出（FK 失败时为 false）
  double position_error{0.0};     ///< m
  double orientation_error{0.0};  ///< rad
  bool within_tolerance{false};
  std::string reason;
};

/// 双臂闭链约束求解器。
///
/// 生命周期：configure() 之后持有 RobotModel 的 const 指针（引用计数，不会悬空）。
/// 线程安全：computeResidual 是 const 且不改内部状态；projectFollower 会修改
/// 传入的 RobotState（这是它的输出），但不改本对象。
class ClosedChainConstraint
{
public:
  ClosedChainConstraint() = default;

  /// 校验参数并绑定机器人模型。
  /// 会检查两个组存在、两个 TCP link 存在、两个组都是链（IK 前提）、
  /// 阈值为正。任一不满足返回 false 并填 error。
  bool configure(
    const ClosedChainParams & params,
    const moveit::core::RobotModelConstPtr & robot_model,
    std::string & error);

  bool isConfigured() const noexcept
  {
    return configured_;
  }

  /// T_rel 是否已确定（显式指定时 configure 后即确定；
  /// 捕获模式下必须先调 captureRelativePose）。
  bool hasRelativePose() const noexcept
  {
    return relative_pose_known_;
  }

  /// 从当前状态捕获 T_rel = T_L^{-1} · T_R。
  /// 调用前请确保 state 就是"两臂已夹住物体"的构型。
  bool captureRelativePose(const moveit::core::RobotState & state, std::string & error);

  /// 直接设定 T_rel（供测试或上层从别处算好后注入）。
  void setRelativePose(const Eigen::Isometry3d & relative_pose) noexcept;

  const Eigen::Isometry3d & relativePose() const noexcept
  {
    return relative_pose_;
  }

  const ClosedChainParams & params() const noexcept
  {
    return params_;
  }

  /// 计算给定构型下的闭链残差，即 (2) 式的偏差量。
  /// 不修改 state（内部拷贝做 FK）。
  ConstraintResidual computeResidual(const moveit::core::RobotState & state) const;

  /// 由 leader 当前构型算出 follower 应到位姿并求 IK，把结果写回 state。
  ///
  /// @param[in,out] state 输入时 leader 关节值必须已设好；
  ///                成功时其 follower 关节值被改写为 IK 解。
  ///                **失败时 state 可能已被 setFromIK 部分修改**，
  ///                所以调用方若需回滚，请自己先备份（dual_arm_planner 就是这么做的）。
  /// @param[out] residual 投影后的实际残差（IK 有数值误差，不会精确为 0）。
  /// @param[out] error 失败原因。
  /// @return true 仅当 IK 成功**且**残差在阈值内。
  bool projectFollower(
    moveit::core::RobotState & state,
    ConstraintResidual & residual,
    std::string & error) const;

  /// 目标位姿计算：T_R = T_L · T_rel。单独暴露便于测试和诊断。
  /// @return false 表示 FK 失败或 T_rel 未确定。
  bool computeFollowerTarget(
    const moveit::core::RobotState & state,
    Eigen::Isometry3d & follower_target,
    std::string & error) const;

private:
  ClosedChainParams params_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup * leader_group_{nullptr};
  const moveit::core::JointModelGroup * follower_group_{nullptr};
  Eigen::Isometry3d relative_pose_{Eigen::Isometry3d::Identity()};
  bool relative_pose_known_{false};
  bool configured_{false};
};

}  // namespace astribot_s1_manipulation

#endif  // ASTRIBOT_S1_MANIPULATION__CLOSED_CHAIN_CONSTRAINT_HPP_
