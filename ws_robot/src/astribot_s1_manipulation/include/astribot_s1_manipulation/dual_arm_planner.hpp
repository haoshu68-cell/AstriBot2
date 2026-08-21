// Copyright 2026 Astribot
//
// 双臂规划编排器：对外的总接口。
//
// 职责边界
// --------
// 本类**不实现**规划算法，它把下面这些已有能力串成完整流程：
//   OMPL 规划(经 move_group)  ->  闭链投影  ->  碰撞/奇异校验  ->  时间参数化/节拍优化
// 每一步失败都转成错误码返回，绝不抛异常、绝不返回非法轨迹。
//
// 为什么用 MoveGroupInterface 而不是 PlanningPipeline
// -------------------------------------------------
// MoveGroupInterface 是 Humble 的稳定官方接口，且自带轨迹执行
// （经 move_group 的 controller manager 下发到 ros2_control）。
// 用 PlanningPipeline 在进程内规划虽然少一次 IPC，但执行链要自己搭
// action client，且拿不到 move_group 维护的实时 PlanningScene。
// 任务要求"启动 move_group"并"Gazebo 里下发执行"，MoveGroupInterface 更贴合。
//
// 单臂 vs 双臂闭链的关键区别
// ------------------------
// 单臂：直接交给 OMPL，出来的轨迹只需校验碰撞/奇异/限位。
// 双臂闭链：**不能**直接对 dual_arm(14维) 调 OMPL —— 那样两条臂各走各的，
//   相对位姿一路漂移，闭链约束不成立（任务明确禁止）。必须：
//   leader 单臂规划 -> 逐点投影出 follower -> 逐点校验 -> 合并成 14 维轨迹。
//   详见 closed_chain_constraint.hpp 的数学说明。

#ifndef ASTRIBOT_S1_MANIPULATION__DUAL_ARM_PLANNER_HPP_
#define ASTRIBOT_S1_MANIPULATION__DUAL_ARM_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "astribot_s1_manipulation/closed_chain_constraint.hpp"
#include "astribot_s1_manipulation/collision_validator.hpp"
#include "astribot_s1_manipulation/error_codes.hpp"
#include "astribot_s1_manipulation/singularity_monitor.hpp"
#include "astribot_s1_manipulation/trajectory_metrics.hpp"
#include "astribot_s1_manipulation/trajectory_time_optimizer.hpp"

namespace astribot_s1_manipulation
{

/// 规划器总配置。全部来自 yaml，代码里无魔数。
struct DualArmPlannerParams
{
  /// 双臂协同组名。SRDF 里同时定义了 dual_arm(14 DOF) 和
  /// dual_arm_with_torso(18 DOF)，切换只改这里，不改代码。
  std::string dual_arm_group{"dual_arm"};

  /// OMPL 规划器 id。必须是 ompl_planning.yaml 里定义过的配置名，
  /// 例如 RRTstarConfig / BITstarConfig / InformedRRTstarConfig。
  /// 注意：写了 MoveIt 不认识的名字时会**静默回退**到组默认规划器，
  /// 所以规划后要核对日志里打印的实际规划器名。
  std::string planner_id{"RRTstarConfig"};

  double allowed_planning_time{5.0};
  int planning_attempts{4};

  /// 单次规划-校验循环失败后的最大重试次数。
  /// 有上限是硬要求：任务禁止无限重试死循环。
  int max_replan_attempts{3};

  /// 闭链投影时，是否对 leader 轨迹做加密插值再投影。
  /// OMPL 的原始路径点很稀（可能只有 5~10 个点），点之间的中间构型
  /// 没有被校验过 —— 闭链场景下这是危险的，因为两点之间 follower 的
  /// IK 解可能跳变。加密后逐点投影才能保证"整条轨迹"满足约束。
  bool densify_leader_path{true};
  /// 加密后相邻点的最大关节角变化(rad)。给太小会让 IK 调用次数爆炸。
  double densify_max_joint_step{0.05};
  /// 加密后的点数上限，防止病态输入导致内存爆掉。
  int densify_max_waypoints{400};

  ClosedChainParams closed_chain;
  SingularityParams singularity;
  CollisionParams collision;
  TimeOptimizerParams time_optimizer;
  MetricsParams metrics;
};

/// 单臂规划请求。三种目标形式互斥，按 named_target -> joint_target -> pose_target
/// 的优先级取第一个非空者。
struct SingleArmPlanRequest
{
  std::string group;                              ///< arm_left / arm_right / torso
  std::string named_target;                       ///< SRDF 里的 group_state 名，如 "ready"
  std::vector<double> joint_target;               ///< 关节目标，维度必须等于组自由度
  bool use_pose_target{false};
  geometry_msgs::msg::PoseStamped pose_target;    ///< 笛卡尔目标（对 TCP link）
  std::string tcp_link;                           ///< pose 目标作用于哪个 link；空则用组默认 tip
  std::string planner_id;                         ///< 空则用 DualArmPlannerParams::planner_id
};

/// 双臂闭链规划请求。
/// leader 的目标同单臂；follower 不给目标 —— 它由闭链约束唯一决定。
struct ClosedChainPlanRequest
{
  std::string leader_named_target;
  std::vector<double> leader_joint_target;
  bool use_pose_target{false};
  geometry_msgs::msg::PoseStamped leader_pose_target;
  std::string planner_id;
  /// true 时用当前状态捕获 T_rel（推荐：先把两臂摆到夹持位姿再规划）。
  /// false 时用 params.closed_chain 里显式给定的值。
  bool capture_relative_pose_now{true};
};

/// 规划结果。失败时 trajectory 为空（任务要求"返回空轨迹"）。
struct PlanResult
{
  PlanErrorCode code{PlanErrorCode::kInvalidInput};

  /// 关节空间轨迹。失败时为空。
  moveit_msgs::msg::RobotTrajectory trajectory;

  /// 笛卡尔空间轨迹：每个 waypoint 的 TCP 位姿（正解得到）。
  /// 双臂时按 leader 的 TCP 输出；follower 的另存在 follower_cartesian_path。
  std::vector<geometry_msgs::msg::PoseStamped> cartesian_path;
  std::vector<geometry_msgs::msg::PoseStamped> follower_cartesian_path;

  TrajectoryMetrics final_metrics;
  OptimizationResult optimization;

  /// 闭链场景下全轨迹里最差的残差（位置/姿态各取最大值）。
  ConstraintResidual worst_residual;
  /// 全轨迹里最小的雅可比最小奇异值（越小越接近奇异）。
  SingularityReport worst_singularity;

  int attempts_used{0};
  /// 闭链投影时 IK 失败的点数（成功时为 0）。诊断 IK 配置的关键指标。
  int ik_failure_count{0};

  std::string message;

  bool succeeded() const noexcept
  {
    return code == PlanErrorCode::kSuccess && !trajectory.joint_trajectory.points.empty();
  }
};

/// 双臂规划编排器。
///
/// 生命周期：必须先 initialize()（内部会建 PlanningSceneMonitor 并等待场景就绪），
/// 之后才能调 plan* 接口。析构时自动停止监视器。
class DualArmPlanner
{
public:
  /// @param node 必须非空且已 spin（内部要用它的 action client 与参数）。
  explicit DualArmPlanner(const rclcpp::Node::SharedPtr & node);
  ~DualArmPlanner();

  DualArmPlanner(const DualArmPlanner &) = delete;
  DualArmPlanner & operator=(const DualArmPlanner &) = delete;

  /// 初始化：加载机器人模型、启动场景监视、配置各校验模块。
  /// @return false 时 error 里是具体原因（模型加载失败、组不存在、参数非法等）。
  ///         调用方应据此优雅退出，不要继续调 plan*。
  bool initialize(const DualArmPlannerParams & params, std::string & error);

  bool isInitialized() const noexcept
  {
    return initialized_;
  }

  const DualArmPlannerParams & params() const noexcept
  {
    return params_;
  }

  /// 单臂规划：输出关节轨迹 + 笛卡尔轨迹。
  PlanResult planSingleArm(const SingleArmPlanRequest & request);

  /// 双臂协同闭链规划。
  PlanResult planClosedChain(const ClosedChainPlanRequest & request);

  /// 执行一条已规划的轨迹（下发到 ros2_control 的 JointTrajectoryController）。
  /// @param group_name 轨迹所属组。
  /// @return kSuccess 表示 move_group 报告执行成功。
  PlanErrorCode executeTrajectory(
    const std::string & group_name,
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    std::string & message);

  /// 取当前机器人状态的一份拷贝。供上层在规划前捕获 T_rel。
  /// @return false 表示场景监视器还没拿到 /joint_states。
  bool getCurrentState(moveit::core::RobotStatePtr & state, std::string & error) const;

  const moveit::core::RobotModelConstPtr & getRobotModel() const noexcept
  {
    return robot_model_;
  }

  ClosedChainConstraint & closedChainConstraint() noexcept
  {
    return closed_chain_;
  }

private:
  /// 实现细节全部放 cpp，头文件只暴露对外 API。
  class Impl;
  std::unique_ptr<Impl> impl_;

  rclcpp::Node::SharedPtr node_;
  DualArmPlannerParams params_;
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;

  ClosedChainConstraint closed_chain_;
  SingularityMonitor singularity_monitor_;
  CollisionValidator collision_validator_;
  TrajectoryTimeOptimizer time_optimizer_;

  bool initialized_{false};
};

}  // namespace astribot_s1_manipulation

#endif  // ASTRIBOT_S1_MANIPULATION__DUAL_ARM_PLANNER_HPP_
