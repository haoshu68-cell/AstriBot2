// Copyright 2026 Astribot

#include "astribot_s1_manipulation/dual_arm_planner.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace astribot_s1_manipulation
{

namespace
{
constexpr const char * kLoggerName = "astribot_s1_manipulation.dual_arm_planner";


/// PlanningSceneMonitor 等待场景就绪的超时。超过这个时间还没收到
/// /joint_states 就认为环境没起来，让调用方优雅退出而不是死等。
constexpr double kSceneWaitTimeoutSec = 10.0;

/// 两个状态在指定组上的**最大**单关节偏差(rad)。
///
/// 刻意用无穷范数而不是 RobotState::distance()（那是各关节距离之和）：
/// "起点是否已在目标上"这种判定要的是"每一个关节都到位"，
/// 用求和的话 14 个关节各差一点会被累加成一个大数，阈值没法定；
/// 反过来一个关节差很多、其余为 0 时求和值也可能落在阈值内，会误判。
double maxJointDeviation(
  const moveit::core::RobotState & lhs,
  const moveit::core::RobotState & rhs,
  const moveit::core::JointModelGroup * jmg)
{
  if (jmg == nullptr) {
    // 调用方已经校验过组存在；这里返回无穷大保证任何阈值都判为"不相等"，
    // 也就是宁可多规划一次，绝不误报"已经到位"。
    return std::numeric_limits<double>::infinity();
  }
  double worst = 0.0;
  for (const moveit::core::JointModel * joint : jmg->getActiveJointModels()) {
    if (joint == nullptr) {
      continue;
    }
    const double * a = lhs.getJointPositions(joint);
    const double * b = rhs.getJointPositions(joint);
    if (a == nullptr || b == nullptr) {
      return std::numeric_limits<double>::infinity();
    }
    for (std::size_t i = 0; i < joint->getVariableCount(); ++i) {
      worst = std::max(worst, std::abs(a[i] - b[i]));
    }
  }
  return worst;
}
}  // namespace

// ===========================================================================
// Impl：所有实现细节。头文件只暴露对外 API。
// ===========================================================================
class DualArmPlanner::Impl
{
public:
  Impl(const rclcpp::Node::SharedPtr & node, DualArmPlanner * owner)
  : node_(node), owner_(owner) {}

  /// 按需创建并缓存 MoveGroupInterface。
  /// 构造 MoveGroupInterface 会与 move_group 节点通信（拉取参数、建 action client），
  /// 开销不小，所以按组缓存复用。
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> getMoveGroup(
    const std::string & group_name, std::string & error)
  {
    error.clear();
    if (group_name.empty()) {
      error = "group name is empty";
      return nullptr;
    }
    const auto it = move_groups_.find(group_name);
    if (it != move_groups_.end()) {
      return it->second;
    }
    try {
      auto mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_, group_name);
      move_groups_.emplace(group_name, mgi);
      return mgi;
    } catch (const std::exception & e) {
      // MoveGroupInterface 构造失败最常见的原因是 move_group 节点没起来，
      // 或者 SRDF 里没有这个组。两者都要让调用方明确知道。
      error = "failed to create MoveGroupInterface for group '" + group_name +
        "': " + e.what();
      return nullptr;
    }
  }

  /// 把 RobotTrajectory 的每个 waypoint 拆成独立的 RobotState 列表，
  /// 供逐点碰撞/奇异校验使用。
  static std::vector<moveit::core::RobotState> extractStates(
    const robot_trajectory::RobotTrajectory & trajectory)
  {
    std::vector<moveit::core::RobotState> states;
    const std::size_t count = trajectory.getWayPointCount();
    states.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      states.push_back(trajectory.getWayPoint(i));
    }
    return states;
  }

  /// 对关节轨迹做线性加密插值。
  ///
  /// 为什么必须加密：OMPL 返回的路径点可能非常稀（几个点），点与点之间的
  /// 中间构型**从未被校验**。单臂场景下 MoveIt 的 state validity checker 在
  /// 规划期已按 longest_valid_segment_fraction 插值查过碰撞，问题不大；
  /// 但闭链场景下 follower 的 IK 是我们自己在 waypoint 上算的，
  /// 两个稀疏点之间 follower 的解可能跳变（IK 多解），必须加密后逐点投影。
  ///
  /// @return 加密后的构型列表（含首尾）。输入不足两点时原样返回。
  std::vector<moveit::core::RobotState> densify(
    const robot_trajectory::RobotTrajectory & trajectory,
    const moveit::core::JointModelGroup * group,
    double max_joint_step,
    int max_waypoints) const
  {
    std::vector<moveit::core::RobotState> out;
    const std::size_t count = trajectory.getWayPointCount();
    if (count == 0U) {
      return out;
    }
    if (count == 1U || group == nullptr || !(max_joint_step > 0.0)) {
      return extractStates(trajectory);
    }

    const std::vector<std::string> & names = group->getVariableNames();
    out.push_back(trajectory.getWayPoint(0));

    for (std::size_t i = 0; i + 1U < count; ++i) {
      const moveit::core::RobotState & a = trajectory.getWayPoint(i);
      const moveit::core::RobotState & b = trajectory.getWayPoint(i + 1U);

      // 该段里变化最大的那个关节决定需要插几份。
      double max_delta = 0.0;
      for (const std::string & name : names) {
        const double delta =
          std::abs(b.getVariablePosition(name) - a.getVariablePosition(name));
        max_delta = std::max(max_delta, delta);
      }
      int steps = 1;
      if (max_delta > max_joint_step) {
        steps = static_cast<int>(std::ceil(max_delta / max_joint_step));
      }
      // 全局点数上限保护：病态输入（例如关节值跳变几十弧度）不能把内存打爆。
      if (max_waypoints > 0 &&
        static_cast<int>(out.size()) + steps > max_waypoints)
      {
        steps = std::max(1, max_waypoints - static_cast<int>(out.size()));
      }

      for (int s = 1; s <= steps; ++s) {
        const double t = static_cast<double>(s) / static_cast<double>(steps);
        moveit::core::RobotState interpolated(a);
        // 只插值该组的变量，其余关节（躯干/头/轮）保持 a 的取值不变。
        for (const std::string & name : names) {
          const double va = a.getVariablePosition(name);
          const double vb = b.getVariablePosition(name);
          interpolated.setVariablePosition(name, va + t * (vb - va));
        }
        interpolated.update();
        out.push_back(interpolated);
      }

      if (max_waypoints > 0 && static_cast<int>(out.size()) >= max_waypoints) {
        // 达到上限就停止加密，但必须把终点补上，否则轨迹到不了目标。
        if (out.size() > 1U) {
          out.back() = trajectory.getWayPoint(count - 1U);
        }
        break;
      }
    }
    return out;
  }

  /// 对一串构型做正解，输出某个 link 的位姿序列（笛卡尔空间轨迹）。
  static std::vector<geometry_msgs::msg::PoseStamped> computeCartesianPath(
    const std::vector<moveit::core::RobotState> & states,
    const std::string & link_name,
    const std::string & frame_id)
  {
    std::vector<geometry_msgs::msg::PoseStamped> path;
    path.reserve(states.size());
    for (const moveit::core::RobotState & state : states) {
      moveit::core::RobotState local(state);
      local.updateLinkTransforms();
      const Eigen::Isometry3d & pose = local.getGlobalLinkTransform(link_name);
      geometry_msgs::msg::PoseStamped stamped;
      stamped.header.frame_id = frame_id;
      stamped.pose = tf2::toMsg(pose);
      path.push_back(stamped);
    }
    return path;
  }

  /// 由构型列表构造 RobotTrajectory（时间戳留给参数化器填）。
  static robot_trajectory::RobotTrajectory buildTrajectory(
    const moveit::core::RobotModelConstPtr & model,
    const std::string & group_name,
    const std::vector<moveit::core::RobotState> & states)
  {
    robot_trajectory::RobotTrajectory trajectory(model, group_name);
    for (const moveit::core::RobotState & state : states) {
      // dt 给 0：真实时间戳由 IPTP/TOTG 计算。
      trajectory.addSuffixWayPoint(state, 0.0);
    }
    return trajectory;
  }

  rclcpp::Node::SharedPtr node_;
  DualArmPlanner * owner_{nullptr};
  std::map<std::string, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>
  move_groups_;
};

// ===========================================================================
// 构造 / 析构
// ===========================================================================
DualArmPlanner::DualArmPlanner(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  // node 判空放在 initialize 里报错，构造函数不抛异常。
  if (node_) {
    impl_ = std::make_unique<Impl>(node_, this);
  }
}

DualArmPlanner::~DualArmPlanner()
{
  // PlanningSceneMonitor 内部起了订阅线程，必须显式停掉再析构，
  // 否则回调可能在成员已销毁后触发（悬空引用）。
  if (scene_monitor_) {
    scene_monitor_->stopSceneMonitor();
    scene_monitor_->stopStateMonitor();
    scene_monitor_->stopWorldGeometryMonitor();
  }
}

// ===========================================================================
// 初始化
// ===========================================================================
bool DualArmPlanner::initialize(const DualArmPlannerParams & params, std::string & error)
{
  error.clear();
  initialized_ = false;

  if (!node_) {
    error = "node pointer is null";
    return false;
  }
  if (!impl_) {
    error = "internal implementation not constructed";
    return false;
  }

  // ---- 机器人模型 ----
  // 任何加载失败都必须让节点优雅退出（任务要求：URDF/SRDF 加载失败不产生段错误）。
  try {
    robot_model_loader::RobotModelLoader::Options options;
    options.robot_description_ = "robot_description";
    auto loader = std::make_shared<robot_model_loader::RobotModelLoader>(node_, options);
    robot_model_ = loader->getModel();
  } catch (const std::exception & e) {
    error = std::string("exception while loading robot model: ") + e.what();
    return false;
  }
  if (!robot_model_) {
    error = "failed to load robot model from 'robot_description'; "
      "is robot_state_publisher / move_group running with the parameter set?";
    return false;
  }

  // ---- 组存在性校验（早失败，错误信息明确）----
  if (robot_model_->getJointModelGroup(params.dual_arm_group) == nullptr) {
    error = "dual arm group '" + params.dual_arm_group + "' not found in SRDF";
    return false;
  }
  if (robot_model_->getJointModelGroup(params.closed_chain.leader_group) == nullptr) {
    error = "leader group '" + params.closed_chain.leader_group + "' not found in SRDF";
    return false;
  }
  if (robot_model_->getJointModelGroup(params.closed_chain.follower_group) == nullptr) {
    error = "follower group '" + params.closed_chain.follower_group + "' not found in SRDF";
    return false;
  }

  // ---- 场景监视器：提供实时机器人状态与环境物体 ----
  try {
    scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description");
  } catch (const std::exception & e) {
    error = std::string("exception while creating PlanningSceneMonitor: ") + e.what();
    return false;
  }
  if (!scene_monitor_->getPlanningScene()) {
    error = "PlanningSceneMonitor has no planning scene";
    return false;
  }
  scene_monitor_->startStateMonitor();
  scene_monitor_->startSceneMonitor();
  scene_monitor_->startWorldGeometryMonitor();

  // 等 /joint_states 到齐。拿不到当前状态就没法做 FK/IK，也没法捕获 T_rel。
  if (!scene_monitor_->getStateMonitor()->waitForCompleteState(kSceneWaitTimeoutSec)) {
    // 这里用 WARN 而不是直接失败：某些场景下部分关节（如轮子）不在
    // /joint_states 里也能正常规划手臂。但必须让用户看到这条。
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName),
      "did not receive a complete robot state within %.1fs; "
      "planning may fail if the arm joints are missing from /joint_states",
      kSceneWaitTimeoutSec);
  }

  // ---- 各校验模块 ----
  if (!singularity_monitor_.configure(params.singularity, error)) {
    error = "singularity monitor configuration failed: " + error;
    return false;
  }
  {
    planning_scene_monitor::LockedPlanningSceneRO locked_scene(scene_monitor_);
    // 传给 CollisionValidator 的是场景的 diff 快照。注意：这份快照不会随
    // 后续世界变化自动更新，所以每次规划时会重新取（见 planSingleArm）。
    if (!collision_validator_.configure(
        planning_scene::PlanningScene::clone(locked_scene), params.collision, error))
    {
      error = "collision validator configuration failed: " + error;
      return false;
    }
  }
  if (!time_optimizer_.configure(params.time_optimizer, error)) {
    error = "time optimizer configuration failed: " + error;
    return false;
  }
  if (!closed_chain_.configure(params.closed_chain, robot_model_, error)) {
    error = "closed chain constraint configuration failed: " + error;
    return false;
  }

  if (params.max_replan_attempts < 1) {
    error = "max_replan_attempts must be >= 1";
    return false;
  }
  if (params.allowed_planning_time <= 0.0) {
    error = "allowed_planning_time must be > 0";
    return false;
  }
  if (params.planning_attempts < 1) {
    error = "planning_attempts must be >= 1";
    return false;
  }

  params_ = params;

  initialized_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "initialized: dual_arm_group='%s' planner='%s' leader='%s' follower='%s' "
    "leader_tcp='%s' follower_tcp='%s' max_replan=%d",
    params_.dual_arm_group.c_str(), params_.planner_id.c_str(),
    params_.closed_chain.leader_group.c_str(), params_.closed_chain.follower_group.c_str(),
    params_.closed_chain.leader_tcp_link.c_str(),
    params_.closed_chain.follower_tcp_link.c_str(),
    params_.max_replan_attempts);
  return true;
}

bool DualArmPlanner::getCurrentState(
  moveit::core::RobotStatePtr & state, std::string & error) const
{
  error.clear();
  if (!initialized_) {
    error = "DualArmPlanner not initialized";
    return false;
  }
  if (!scene_monitor_) {
    error = "planning scene monitor is null";
    return false;
  }
  const auto state_monitor = scene_monitor_->getStateMonitor();
  if (!state_monitor) {
    error = "current state monitor is null";
    return false;
  }
  moveit::core::RobotStatePtr current = state_monitor->getCurrentState();
  if (!current) {
    error = "current state unavailable (no /joint_states received?)";
    return false;
  }
  state = std::make_shared<moveit::core::RobotState>(*current);
  state->update();
  return true;
}

// ===========================================================================
// 单臂规划
// ===========================================================================
PlanResult DualArmPlanner::planSingleArm(const SingleArmPlanRequest & request)
{
  PlanResult result;

  if (!initialized_) {
    result.code = PlanErrorCode::kNotConfigured;
    result.message = "DualArmPlanner not initialized";
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
    return result;
  }
  if (request.group.empty()) {
    result.code = PlanErrorCode::kInvalidInput;
    result.message = "request.group is empty";
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
    return result;
  }

  const moveit::core::JointModelGroup * jmg =
    robot_model_->getJointModelGroup(request.group);
  if (jmg == nullptr) {
    result.code = PlanErrorCode::kPlanningGroupNotFound;
    result.message = "planning group '" + request.group + "' not found in SRDF";
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
    return result;
  }

  // 关节目标的维度必须与组自由度一致，否则 setJointValueTarget 会静默失败
  // 或者设成部分关节 —— 这种错误在运行时极难定位，所以在入口就拦住。
  if (!request.use_pose_target && request.named_target.empty() &&
    !request.joint_target.empty() &&
    request.joint_target.size() != jmg->getVariableCount())
  {
    std::ostringstream oss;
    oss << "joint_target size " << request.joint_target.size()
        << " does not match group '" << request.group << "' DOF "
        << jmg->getVariableCount();
    result.code = PlanErrorCode::kInvalidInput;
    result.message = oss.str();
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
    return result;
  }
  if (!request.use_pose_target && request.named_target.empty() &&
    request.joint_target.empty())
  {
    result.code = PlanErrorCode::kInvalidInput;
    result.message = "no target specified (named_target / joint_target / pose_target all empty)";
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
    return result;
  }

  std::string error;
  auto move_group = impl_->getMoveGroup(request.group, error);
  if (!move_group) {
    result.code = PlanErrorCode::kRobotModelUnavailable;
    result.message = error;
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
    return result;
  }

  const std::string planner_id =
    request.planner_id.empty() ? params_.planner_id : request.planner_id;

  // 每次规划前刷新碰撞校验用的场景快照：环境物体可能已经变了。
  {
    planning_scene_monitor::LockedPlanningSceneRO locked_scene(scene_monitor_);
    std::string cfg_error;
    if (!collision_validator_.configure(
        planning_scene::PlanningScene::clone(locked_scene), params_.collision, cfg_error))
    {
      result.code = PlanErrorCode::kRobotModelUnavailable;
      result.message = "failed to refresh collision scene: " + cfg_error;
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
      return result;
    }
  }

  const std::string tcp_link = request.tcp_link.empty() ?
    (request.group == params_.closed_chain.leader_group ?
    params_.closed_chain.leader_tcp_link :
    params_.closed_chain.follower_tcp_link) :
    request.tcp_link;

  // 重试循环：采样式规划器有随机性，同一请求重试可能拿到不同（且合法）的解。
  // 次数有上限，绝不无限重试。
  for (int attempt = 1; attempt <= params_.max_replan_attempts; ++attempt) {
    result.attempts_used = attempt;

    try {
      move_group->setPlannerId(planner_id);
      move_group->setPlanningTime(params_.allowed_planning_time);
      move_group->setNumPlanningAttempts(params_.planning_attempts);
      move_group->setStartStateToCurrentState();
      move_group->clearPoseTargets();

      bool target_ok = true;
      if (!request.named_target.empty()) {
        target_ok = move_group->setNamedTarget(request.named_target);
        if (!target_ok) {
          result.code = PlanErrorCode::kInvalidInput;
          result.message = "named target '" + request.named_target +
            "' not found in SRDF for group '" + request.group + "'";
          RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
          return result;   // 名字错了，重试无意义
        }
      } else if (request.use_pose_target) {
        target_ok = move_group->setPoseTarget(request.pose_target, tcp_link);
        if (!target_ok) {
          result.code = PlanErrorCode::kInvalidInput;
          result.message = "failed to set pose target for link '" + tcp_link + "'";
          RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
          return result;
        }
      } else {
        target_ok = move_group->setJointValueTarget(request.joint_target);
        if (!target_ok) {
          // setJointValueTarget 返回 false 通常意味着目标超出关节限位。
          result.code = PlanErrorCode::kInvalidInput;
          result.message = "joint target rejected (out of joint limits?)";
          RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
          return result;
        }
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const moveit::core::MoveItErrorCode plan_code = move_group->plan(plan);
      if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(
          rclcpp::get_logger(kLoggerName),
          "attempt %d/%d: OMPL planning failed for group '%s' with planner '%s' "
          "(MoveItErrorCode=%d)",
          attempt, params_.max_replan_attempts, request.group.c_str(),
          planner_id.c_str(), plan_code.val);
        result.code = PlanErrorCode::kPlannerFailed;
        result.message = "planner returned no solution";
        continue;
      }

      // ---- 转成 RobotTrajectory 做校验与时间优化 ----
      robot_trajectory::RobotTrajectory trajectory(robot_model_, jmg);
      moveit::core::RobotStatePtr reference_state;
      if (!getCurrentState(reference_state, error)) {
        result.code = PlanErrorCode::kRobotModelUnavailable;
        result.message = error;
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
        return result;
      }
      trajectory.setRobotTrajectoryMsg(*reference_state, plan.trajectory_);
      if (trajectory.getWayPointCount() == 0U) {
        result.code = PlanErrorCode::kPlannerFailed;
        result.message = "planner returned an empty trajectory";
        RCLCPP_WARN(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
        continue;
      }

      const std::vector<moveit::core::RobotState> states = impl_->extractStates(trajectory);

      // ---- 碰撞校验（自碰撞 + 臂-底盘 + 环境）----
      std::size_t bad_index = 0;
      const CollisionReport collision =
        collision_validator_.checkStates(states, request.group, &bad_index);
      if (collision.collision) {
        RCLCPP_WARN(
          rclcpp::get_logger(kLoggerName),
          "attempt %d/%d: collision at waypoint %zu/%zu: %s -> discarding solution",
          attempt, params_.max_replan_attempts, bad_index, states.size(),
          collision.reason.c_str());
        result.code = collision.toErrorCode();
        result.message = collision.reason;
        continue;
      }

      // ---- 奇异点校验 ----
      // 只对是链的组做：dual_arm 不是链，雅可比无定义（这里单臂一定是链）。
      if (jmg->isChain()) {
        std::size_t worst_index = 0;
        const SingularityReport singularity =
          singularity_monitor_.checkStates(states, jmg, tcp_link, &worst_index);
        result.worst_singularity = singularity;
        if (!singularity.valid) {
          RCLCPP_WARN(
            rclcpp::get_logger(kLoggerName),
            "attempt %d/%d: singularity check invalid: %s",
            attempt, params_.max_replan_attempts, singularity.reason.c_str());
          result.code = PlanErrorCode::kSingularConfiguration;
          result.message = singularity.reason;
          continue;
        }
        if (singularity.singular) {
          RCLCPP_WARN(
            rclcpp::get_logger(kLoggerName),
            "attempt %d/%d: singular configuration at waypoint %zu/%zu (%s) "
            "-> discarding solution, replanning",
            attempt, params_.max_replan_attempts, worst_index, states.size(),
            singularity.reason.c_str());
          result.code = PlanErrorCode::kSingularConfiguration;
          result.message = singularity.reason;
          continue;
        }
      }

      // ---- 时间参数化 + 节拍优化（内含超限回退）----
      OptimizationResult optimization;
      const PlanErrorCode opt_code = time_optimizer_.optimize(trajectory, optimization);
      result.optimization = optimization;
      if (opt_code != PlanErrorCode::kSuccess) {
        // 超限或参数化失败：不输出任何轨迹。这类失败重试无用（同一条路径
        // 再参数化一次结果一样），所以直接返回。
        result.code = opt_code;
        result.message = optimization.note;
        RCLCPP_ERROR(
          rclcpp::get_logger(kLoggerName),
          "time parameterization/optimization failed: %s", optimization.note.c_str());
        return result;
      }

      // ---- 输出 ----
      MetricsParams metrics_params = params_.metrics;
      result.final_metrics = evaluateTrajectory(trajectory, metrics_params);
      if (!result.final_metrics.isLegal()) {
        // 双保险：optimize() 已经复核过，这里再确认一次最终交付物。
        result.code = PlanErrorCode::kJointLimitViolation;
        result.message = "final trajectory is illegal: " + result.final_metrics.summary;
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
        return result;
      }

      trajectory.getRobotTrajectoryMsg(result.trajectory);
      result.cartesian_path = impl_->computeCartesianPath(
        impl_->extractStates(trajectory), tcp_link, robot_model_->getModelFrame());
      result.code = PlanErrorCode::kSuccess;
      result.message = "single-arm plan succeeded with planner '" + planner_id + "'";

      RCLCPP_INFO(
        rclcpp::get_logger(kLoggerName),
        "single-arm plan OK: group='%s' planner='%s' attempt=%d %s",
        request.group.c_str(), planner_id.c_str(), attempt,
        result.final_metrics.summary.c_str());
      return result;
    } catch (const std::exception & e) {
      // 任何第三方库异常都在这里兜住，转成错误码。
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "attempt %d/%d: exception during single-arm planning: %s",
        attempt, params_.max_replan_attempts, e.what());
      result.code = PlanErrorCode::kExceptionCaught;
      result.message = e.what();
      continue;
    }
  }

  // 重试用尽。保留最后一次的失败原因，同时标明是重试耗尽。
  const std::string last_reason = result.message;
  result.code = PlanErrorCode::kRetriesExhausted;
  result.message = "all " + std::to_string(params_.max_replan_attempts) +
    " attempts failed; last reason: " + last_reason;
  result.trajectory = moveit_msgs::msg::RobotTrajectory();  // 明确返回空轨迹
  RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
  return result;
}

// ===========================================================================
// 双臂协同闭链规划
//
// 这是任务的核心场景。再强调一次为什么不能直接对 dual_arm(14维) 调 OMPL：
// 那样两条臂在各自的子空间里独立探索，相对位姿 T_rel 一路漂移，
// 被夹持的物体会被拉扯 —— 闭链约束根本不成立。任务明确禁止这种做法。
//
// 本实现：leader 单臂规划 -> 加密 -> 逐点 IK 投影出 follower -> 逐点校验。
// 得到的轨迹在**每一个采样点**上都满足 T_L^{-1}·T_R = T_rel（残差可量化）。
// ===========================================================================
PlanResult DualArmPlanner::planClosedChain(const ClosedChainPlanRequest & request)
{
  PlanResult result;

  if (!initialized_) {
    result.code = PlanErrorCode::kNotConfigured;
    result.message = "DualArmPlanner not initialized";
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
    return result;
  }

  const std::string & leader_group_name = params_.closed_chain.leader_group;
  const std::string & follower_group_name = params_.closed_chain.follower_group;
  const moveit::core::JointModelGroup * leader_jmg =
    robot_model_->getJointModelGroup(leader_group_name);
  const moveit::core::JointModelGroup * follower_jmg =
    robot_model_->getJointModelGroup(follower_group_name);
  const moveit::core::JointModelGroup * dual_jmg =
    robot_model_->getJointModelGroup(params_.dual_arm_group);
  if (leader_jmg == nullptr || follower_jmg == nullptr || dual_jmg == nullptr) {
    result.code = PlanErrorCode::kPlanningGroupNotFound;
    result.message = "leader/follower/dual_arm group missing in SRDF";
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
    return result;
  }

  std::string error;

  // ---- 第一步：确定闭链相对位姿 T_rel ----
  // 捕获模式要求"当前两臂已经夹住物体"。这是推荐做法：夹持几何由实际摆位
  // 决定，不需要任何人去量尺寸（也就不存在硬编码闭链几何的问题）。
  if (request.capture_relative_pose_now) {
    moveit::core::RobotStatePtr current_state;
    if (!getCurrentState(current_state, error)) {
      result.code = PlanErrorCode::kRobotModelUnavailable;
      result.message = "cannot capture T_rel: " + error;
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
      return result;
    }
    if (!closed_chain_.captureRelativePose(*current_state, error)) {
      result.code = PlanErrorCode::kInvalidInput;
      result.message = "failed to capture T_rel: " + error;
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
      return result;
    }
  }
  if (!closed_chain_.hasRelativePose()) {
    result.code = PlanErrorCode::kInvalidInput;
    result.message = "closed-chain T_rel is unknown; either set "
      "capture_relative_pose_now=true or provide explicit relative_* values in yaml";
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
    return result;
  }

  // ---- 刷新碰撞场景快照 ----
  {
    planning_scene_monitor::LockedPlanningSceneRO locked_scene(scene_monitor_);
    std::string cfg_error;
    if (!collision_validator_.configure(
        planning_scene::PlanningScene::clone(locked_scene), params_.collision, cfg_error))
    {
      result.code = PlanErrorCode::kRobotModelUnavailable;
      result.message = "failed to refresh collision scene: " + cfg_error;
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
      return result;
    }
  }

  auto leader_move_group = impl_->getMoveGroup(leader_group_name, error);
  if (!leader_move_group) {
    result.code = PlanErrorCode::kRobotModelUnavailable;
    result.message = error;
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
    return result;
  }

  const std::string planner_id =
    request.planner_id.empty() ? params_.planner_id : request.planner_id;

  for (int attempt = 1; attempt <= params_.max_replan_attempts; ++attempt) {
    result.attempts_used = attempt;
    result.ik_failure_count = 0;

    try {
      // ---- 第二步：leader 单臂规划（7 维，成功率远高于 14 维）----
      leader_move_group->setPlannerId(planner_id);
      leader_move_group->setPlanningTime(params_.allowed_planning_time);
      leader_move_group->setNumPlanningAttempts(params_.planning_attempts);
      leader_move_group->setStartStateToCurrentState();
      leader_move_group->clearPoseTargets();

      if (!request.leader_named_target.empty()) {
        if (!leader_move_group->setNamedTarget(request.leader_named_target)) {
          result.code = PlanErrorCode::kInvalidInput;
          result.message = "leader named target '" + request.leader_named_target +
            "' not found in SRDF";
          RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
          return result;
        }
      } else if (request.use_pose_target) {
        if (!leader_move_group->setPoseTarget(
            request.leader_pose_target, params_.closed_chain.leader_tcp_link))
        {
          result.code = PlanErrorCode::kInvalidInput;
          result.message = "failed to set leader pose target";
          RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
          return result;
        }
      } else if (!request.leader_joint_target.empty()) {
        if (request.leader_joint_target.size() != leader_jmg->getVariableCount()) {
          std::ostringstream oss;
          oss << "leader_joint_target size " << request.leader_joint_target.size()
              << " != leader group DOF " << leader_jmg->getVariableCount();
          result.code = PlanErrorCode::kInvalidInput;
          result.message = oss.str();
          RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
          return result;
        }
        if (!leader_move_group->setJointValueTarget(request.leader_joint_target)) {
          result.code = PlanErrorCode::kInvalidInput;
          result.message = "leader joint target rejected (out of limits?)";
          RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
          return result;
        }
      } else {
        result.code = PlanErrorCode::kInvalidInput;
        result.message = "no leader target specified";
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
        return result;
      }

      moveit::planning_interface::MoveGroupInterface::Plan leader_plan;
      const moveit::core::MoveItErrorCode plan_code = leader_move_group->plan(leader_plan);
      if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(
          rclcpp::get_logger(kLoggerName),
          "attempt %d/%d: leader planning failed (MoveItErrorCode=%d)",
          attempt, params_.max_replan_attempts, plan_code.val);
        result.code = PlanErrorCode::kPlannerFailed;
        result.message = "leader planning returned no solution";
        continue;
      }

      moveit::core::RobotStatePtr reference_state;
      if (!getCurrentState(reference_state, error)) {
        result.code = PlanErrorCode::kRobotModelUnavailable;
        result.message = error;
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
        return result;
      }

      robot_trajectory::RobotTrajectory leader_trajectory(robot_model_, leader_jmg);
      leader_trajectory.setRobotTrajectoryMsg(*reference_state, leader_plan.trajectory_);
      if (leader_trajectory.getWayPointCount() == 0U) {
        result.code = PlanErrorCode::kPlannerFailed;
        result.message = "leader trajectory is empty";
        RCLCPP_WARN(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
        continue;
      }

      // ---- 第三步：加密 leader 路径 ----
      // 不加密的话，两个稀疏点之间 follower 的 IK 解可能跳变（IK 多解），
      // 中间构型从未被校验 —— 闭链场景下这是不能接受的。
      std::vector<moveit::core::RobotState> leader_states;
      if (params_.densify_leader_path) {
        leader_states = impl_->densify(
          leader_trajectory, leader_jmg,
          params_.densify_max_joint_step, params_.densify_max_waypoints);
      } else {
        leader_states = impl_->extractStates(leader_trajectory);
      }
      if (leader_states.size() < 2U) {
        // 到这里只有两种可能，必须分开处理，否则会把"已经到位"误报成
        // "规划器坏了"并白重试（Gazebo 实测踩过，见 error_codes.hpp 的
        // kAlreadyAtGoal 说明）：
        //   ① 起点就在目标上 —— OMPL 返回 2 个相同状态、代价 0.00 的退化路径，
        //      加密后自然凑不出 2 个有效点。这是确定性结论，重试毫无意义。
        //   ② 别的原因导致路径退化 —— 那才是真的规划失败，值得重试。
        if (maxJointDeviation(
            leader_trajectory.getWayPoint(0U), leader_trajectory.getLastWayPoint(),
            leader_jmg) <= params_.already_at_goal_tolerance_rad)
        {
          result.code = PlanErrorCode::kAlreadyAtGoal;
          result.message = "leader group '" + leader_jmg->getName() +
            "' is already at the requested target (max joint deviation <= " +
            std::to_string(params_.already_at_goal_tolerance_rad) +
            " rad); no closed-chain motion is needed";
          RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
          // 刻意不输出轨迹：没有动作可执行，给一条零长度轨迹只会让调用方
          // 误以为"执行完了这段就到位了"。轨迹为空 + 明确状态码更不容易用错。
          return result;
        }
        result.code = PlanErrorCode::kPlannerFailed;
        result.message = "leader path has fewer than 2 waypoints after densification";
        RCLCPP_WARN(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
        continue;
      }

      // ---- 第四步：逐点投影 follower + 逐点校验 ----
      std::vector<moveit::core::RobotState> dual_states;
      dual_states.reserve(leader_states.size());

      // 用上一个点的 follower 解作为下一个点 IK 的种子：相邻点的解应该连续，
      // 从邻近种子出发能大幅提高成功率，也避免 IK 在多解之间跳变
      // （跳变会让 follower 关节出现瞬时大位移，时间参数化后必然超速）。
      moveit::core::RobotState working_state(*reference_state);

      double worst_position_error = 0.0;
      double worst_orientation_error = 0.0;
      bool projection_failed = false;
      std::string projection_error;
      std::size_t failed_index = 0;

      for (std::size_t i = 0; i < leader_states.size(); ++i) {
        // 把 leader 的关节值搬进工作状态，follower 关节保持上一次的解（IK 种子）。
        std::vector<double> leader_positions;
        leader_states[i].copyJointGroupPositions(leader_jmg, leader_positions);
        working_state.setJointGroupPositions(leader_jmg, leader_positions);
        working_state.update();

        // 备份 follower 关节：projectFollower 失败时 state 可能已被
        // setFromIK 部分修改，需要回滚才能让下一个点的种子保持有效。
        std::vector<double> follower_backup;
        working_state.copyJointGroupPositions(follower_jmg, follower_backup);

        ConstraintResidual residual;
        std::string project_error;
        if (!closed_chain_.projectFollower(working_state, residual, project_error)) {
          working_state.setJointGroupPositions(follower_jmg, follower_backup);
          working_state.update();
          ++result.ik_failure_count;
          projection_failed = true;
          projection_error = project_error;
          failed_index = i;
          break;
        }

        worst_position_error = std::max(worst_position_error, residual.position_error);
        worst_orientation_error = std::max(worst_orientation_error, residual.orientation_error);

        dual_states.push_back(working_state);
      }

      if (projection_failed) {
        RCLCPP_WARN(
          rclcpp::get_logger(kLoggerName),
          "attempt %d/%d: closed-chain projection failed at waypoint %zu/%zu: %s",
          attempt, params_.max_replan_attempts, failed_index, leader_states.size(),
          projection_error.c_str());
        // 区分 IK 无解与残差超限：两者的处置不同（前者调 IK 参数/换目标，
        // 后者调闭链阈值/换 leader 路径），错误码必须分开。
        result.code = projection_error.find("IK failed") != std::string::npos ?
          PlanErrorCode::kIkFailed : PlanErrorCode::kClosedChainResidualTooLarge;
        result.message = projection_error;
        continue;
      }

      result.worst_residual.valid = true;
      result.worst_residual.position_error = worst_position_error;
      result.worst_residual.orientation_error = worst_orientation_error;
      result.worst_residual.within_tolerance =
        worst_position_error <= params_.closed_chain.position_tolerance &&
        worst_orientation_error <= params_.closed_chain.orientation_tolerance;
      {
        std::ostringstream oss;
        oss << "worst over trajectory: e_p=" << worst_position_error << "m, e_r="
            << worst_orientation_error << "rad";
        result.worst_residual.reason = oss.str();
      }

      // ---- 第五步：合并成 14 维 dual_arm 轨迹 ----
      robot_trajectory::RobotTrajectory dual_trajectory =
        impl_->buildTrajectory(robot_model_, params_.dual_arm_group, dual_states);

      // ---- 第六步：碰撞校验（含双臂互撞）----
      std::size_t bad_index = 0;
      const CollisionReport collision =
        collision_validator_.checkStates(dual_states, params_.dual_arm_group, &bad_index);
      if (collision.collision) {
        RCLCPP_WARN(
          rclcpp::get_logger(kLoggerName),
          "attempt %d/%d: collision at waypoint %zu/%zu: %s -> replanning",
          attempt, params_.max_replan_attempts, bad_index, dual_states.size(),
          collision.reason.c_str());
        result.code = collision.toErrorCode();
        result.message = collision.reason;
        continue;
      }

      // ---- 第七步：奇异点校验（两条臂分别做）----
      // dual_arm 组不是运动链，雅可比无定义，必须拆开检查 —— 这也是
      // SingularityMonitor 里显式拦住非链组的原因。
      bool singular = false;
      for (const auto & arm : {
        std::make_pair(leader_jmg, params_.closed_chain.leader_tcp_link),
        std::make_pair(follower_jmg, params_.closed_chain.follower_tcp_link)
      })
      {
        std::size_t worst_index = 0;
        const SingularityReport report =
          singularity_monitor_.checkStates(dual_states, arm.first, arm.second, &worst_index);
        if (!report.valid || report.singular) {
          RCLCPP_WARN(
            rclcpp::get_logger(kLoggerName),
            "attempt %d/%d: group '%s' singular/invalid at waypoint %zu/%zu: %s",
            attempt, params_.max_replan_attempts, arm.first->getName().c_str(),
            worst_index, dual_states.size(), report.reason.c_str());
          result.code = PlanErrorCode::kSingularConfiguration;
          result.message = arm.first->getName() + ": " + report.reason;
          result.worst_singularity = report;
          singular = true;
          break;
        }
        // 记录两条臂里更差的那个，供上层观察闭链构型的奇异余量。
        if (!result.worst_singularity.valid ||
          report.min_singular_value < result.worst_singularity.min_singular_value)
        {
          result.worst_singularity = report;
        }
      }
      if (singular) {
        continue;
      }

      // ---- 第八步：时间参数化 + 节拍优化 ----
      OptimizationResult optimization;
      const PlanErrorCode opt_code = time_optimizer_.optimize(dual_trajectory, optimization);
      result.optimization = optimization;
      if (opt_code != PlanErrorCode::kSuccess) {
        result.code = opt_code;
        result.message = optimization.note;
        RCLCPP_ERROR(
          rclcpp::get_logger(kLoggerName),
          "closed-chain time parameterization failed: %s", optimization.note.c_str());
        return result;
      }

      MetricsParams metrics_params = params_.metrics;
      result.final_metrics = evaluateTrajectory(dual_trajectory, metrics_params);
      if (!result.final_metrics.isLegal()) {
        result.code = PlanErrorCode::kJointLimitViolation;
        result.message = "final closed-chain trajectory is illegal: " +
          result.final_metrics.summary;
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
        return result;
      }

      // 时间参数化（尤其 TOTG 的重采样）会改变 waypoint 集合，
      // 所以闭链残差必须在**最终轨迹**上再复核一遍 —— 重采样插出来的新点
      // 未必满足闭链约束。这一步不能省，否则交付的轨迹可能在插值点上破坏闭链。
      const std::vector<moveit::core::RobotState> final_states =
        impl_->extractStates(dual_trajectory);
      double final_worst_p = 0.0;
      double final_worst_r = 0.0;
      for (const moveit::core::RobotState & state : final_states) {
        const ConstraintResidual residual = closed_chain_.computeResidual(state);
        if (!residual.valid) {
          result.code = PlanErrorCode::kClosedChainResidualTooLarge;
          result.message = "residual recheck failed after time parameterization: " +
            residual.reason;
          RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
          return result;
        }
        final_worst_p = std::max(final_worst_p, residual.position_error);
        final_worst_r = std::max(final_worst_r, residual.orientation_error);
      }
      result.worst_residual.position_error = final_worst_p;
      result.worst_residual.orientation_error = final_worst_r;
      result.worst_residual.within_tolerance =
        final_worst_p <= params_.closed_chain.position_tolerance &&
        final_worst_r <= params_.closed_chain.orientation_tolerance;
      if (!result.worst_residual.within_tolerance) {
        RCLCPP_WARN(
          rclcpp::get_logger(kLoggerName),
          "attempt %d/%d: closed-chain violated after time parameterization "
          "(e_p=%.5fm tol=%.5f, e_r=%.5frad tol=%.5f) -> replanning",
          attempt, params_.max_replan_attempts,
          final_worst_p, params_.closed_chain.position_tolerance,
          final_worst_r, params_.closed_chain.orientation_tolerance);
        result.code = PlanErrorCode::kClosedChainResidualTooLarge;
        result.message = "closed-chain residual out of tolerance on final trajectory";
        continue;
      }

      // ---- 第九步：输出 ----
      dual_trajectory.getRobotTrajectoryMsg(result.trajectory);
      result.cartesian_path = impl_->computeCartesianPath(
        final_states, params_.closed_chain.leader_tcp_link, robot_model_->getModelFrame());
      result.follower_cartesian_path = impl_->computeCartesianPath(
        final_states, params_.closed_chain.follower_tcp_link, robot_model_->getModelFrame());
      result.code = PlanErrorCode::kSuccess;
      result.message = "closed-chain plan succeeded with planner '" + planner_id + "'";

      RCLCPP_INFO(
        rclcpp::get_logger(kLoggerName),
        "closed-chain plan OK: attempt=%d waypoints=%zu %s | worst residual e_p=%.5fm "
        "e_r=%.5frad | worst sigma_min=%.5f",
        attempt, final_states.size(), result.final_metrics.summary.c_str(),
        final_worst_p, final_worst_r, result.worst_singularity.min_singular_value);
      return result;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "attempt %d/%d: exception during closed-chain planning: %s",
        attempt, params_.max_replan_attempts, e.what());
      result.code = PlanErrorCode::kExceptionCaught;
      result.message = e.what();
      continue;
    }
  }

  const std::string last_reason = result.message;
  result.code = PlanErrorCode::kRetriesExhausted;
  result.message = "all " + std::to_string(params_.max_replan_attempts) +
    " closed-chain attempts failed; last reason: " + last_reason;
  result.trajectory = moveit_msgs::msg::RobotTrajectory();
  RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", result.message.c_str());
  return result;
}

// ===========================================================================
// 轨迹执行
// ===========================================================================
PlanErrorCode DualArmPlanner::executeTrajectory(
  const std::string & group_name,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  std::string & message)
{
  message.clear();

  if (!initialized_) {
    message = "DualArmPlanner not initialized";
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", message.c_str());
    return PlanErrorCode::kNotConfigured;
  }
  if (trajectory.joint_trajectory.points.empty()) {
    // 空轨迹不下发。规划失败时返回的就是空轨迹，这里是最后一道防线：
    // 绝不把空轨迹当成"什么都不用做"而报成功。
    message = "refusing to execute an empty trajectory";
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", message.c_str());
    return PlanErrorCode::kInvalidInput;
  }

  std::string error;
  auto move_group = impl_->getMoveGroup(group_name, error);
  if (!move_group) {
    message = error;
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", message.c_str());
    return PlanErrorCode::kRobotModelUnavailable;
  }

  try {
    // 用 MoveGroupInterface::execute()（阻塞，内部走 /execute_trajectory action）。
    //
    // 曾经怀疑过这个调用本身有并发问题：实测它返回 MoveItErrorCode=-7
    // (CONTROL_FAILED)，同时日志刷
    //   [ERROR] [<node>.rclcpp_action]: unknown goal response, ignoring...
    // 而 move_group 那边却是 "Execution completed: SUCCEEDED"。
    // 真实原因与本调用无关：当时域内泄漏了 7 个 move_group 进程，
    // 7 组同名 action server 互相抢答。根治办法在
    // launch/planning_demo.launch.py 里（demo 节点退出即关闭整个 launch），
    // 那里有完整的排查记录。单一 move_group 下本调用工作正常。
    //
    // 失败时把 MoveItErrorCode 原样打出来：-7 是 CONTROL_FAILED，
    // -1 是 FAILURE，值本身是定位的第一手线索。
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    const moveit::core::MoveItErrorCode code = move_group->execute(plan);
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      std::ostringstream oss;
      oss << "trajectory execution failed with MoveItErrorCode=" << code.val;
      message = oss.str();
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", message.c_str());
      return PlanErrorCode::kPlannerFailed;
    }
    message = "trajectory executed successfully on group '" + group_name + "'";
    RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "%s", message.c_str());
    return PlanErrorCode::kSuccess;
  } catch (const std::exception & e) {
    message = std::string("exception during execution: ") + e.what();
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", message.c_str());
    return PlanErrorCode::kExceptionCaught;
  }
}

}  // namespace astribot_s1_manipulation
