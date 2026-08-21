// Copyright 2026 Astribot
//
// 规划验证 demo 节点。
//
// 它做四件事，对应任务成功判定里的四条：
//   1. single_arm_*       单臂规划，输出关节轨迹 + 笛卡尔轨迹
//   2. closed_chain       双臂协同闭链规划，输出残差与奇异余量
//   3. planner_comparison 同一任务分别用 RRT*/BIT*/Informed RRT* 跑，对比节拍
//                         —— 这是"三个规划器都真的可用"的自证
//   4. 全程打印量化指标（总时长、各关节最大速度、优化前后对比）
//
// 所有参数从 yaml 读，节点内不写死任何数值。

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "astribot_s1_manipulation/dual_arm_planner.hpp"

namespace astribot_s1_manipulation
{

namespace
{
constexpr const char * kLoggerName = "planning_demo";
}  // namespace

/// 从 ROS 参数装载 DualArmPlannerParams。
/// 每个参数都 declare 后再 get，缺项时用结构体里的默认值兜底。
class ParameterLoader
{
public:
  explicit ParameterLoader(const rclcpp::Node::SharedPtr & node)
  : node_(node) {}

  template<typename T>
  T get(const std::string & name, const T & fallback)
  {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<T>(name, fallback);
    }
    T value = fallback;
    if (!node_->get_parameter(name, value)) {
      RCLCPP_WARN(
        node_->get_logger(), "parameter '%s' unreadable, using fallback", name.c_str());
      return fallback;
    }
    return value;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

DualArmPlannerParams loadParams(const rclcpp::Node::SharedPtr & node)
{
  ParameterLoader loader(node);
  DualArmPlannerParams p;

  p.dual_arm_group = loader.get<std::string>("dual_arm_group", p.dual_arm_group);
  p.planner_id = loader.get<std::string>("planner_id", p.planner_id);
  p.allowed_planning_time =
    loader.get<double>("allowed_planning_time", p.allowed_planning_time);
  p.planning_attempts =
    static_cast<int>(loader.get<int64_t>("planning_attempts", p.planning_attempts));
  p.max_replan_attempts =
    static_cast<int>(loader.get<int64_t>("max_replan_attempts", p.max_replan_attempts));
  p.densify_leader_path = loader.get<bool>("densify_leader_path", p.densify_leader_path);
  p.densify_max_joint_step =
    loader.get<double>("densify_max_joint_step", p.densify_max_joint_step);
  p.densify_max_waypoints = static_cast<int>(
    loader.get<int64_t>("densify_max_waypoints", p.densify_max_waypoints));
  p.already_at_goal_tolerance_rad = loader.get<double>(
    "already_at_goal_tolerance_rad", p.already_at_goal_tolerance_rad);

  // ---- 闭链 ----
  auto & cc = p.closed_chain;
  cc.leader_group = loader.get<std::string>("closed_chain.leader_group", cc.leader_group);
  cc.follower_group =
    loader.get<std::string>("closed_chain.follower_group", cc.follower_group);
  cc.leader_tcp_link =
    loader.get<std::string>("closed_chain.leader_tcp_link", cc.leader_tcp_link);
  cc.follower_tcp_link =
    loader.get<std::string>("closed_chain.follower_tcp_link", cc.follower_tcp_link);
  cc.capture_from_current_state =
    loader.get<bool>("closed_chain.capture_from_current_state", cc.capture_from_current_state);
  cc.position_tolerance =
    loader.get<double>("closed_chain.position_tolerance", cc.position_tolerance);
  cc.orientation_tolerance =
    loader.get<double>("closed_chain.orientation_tolerance", cc.orientation_tolerance);
  cc.ik_timeout = loader.get<double>("closed_chain.ik_timeout", cc.ik_timeout);
  cc.ik_attempts =
    static_cast<int>(loader.get<int64_t>("closed_chain.ik_attempts", cc.ik_attempts));

  const std::vector<double> translation = loader.get<std::vector<double>>(
    "closed_chain.relative_translation",
    std::vector<double>{cc.relative_translation[0], cc.relative_translation[1],
      cc.relative_translation[2]});
  if (translation.size() == 3U) {
    cc.relative_translation = {{translation[0], translation[1], translation[2]}};
  } else {
    RCLCPP_WARN(
      node->get_logger(),
      "closed_chain.relative_translation must have 3 elements, got %zu; using default",
      translation.size());
  }
  const std::vector<double> rotation = loader.get<std::vector<double>>(
    "closed_chain.relative_rotation_xyzw",
    std::vector<double>{cc.relative_rotation_xyzw[0], cc.relative_rotation_xyzw[1],
      cc.relative_rotation_xyzw[2], cc.relative_rotation_xyzw[3]});
  if (rotation.size() == 4U) {
    cc.relative_rotation_xyzw = {{rotation[0], rotation[1], rotation[2], rotation[3]}};
  } else {
    RCLCPP_WARN(
      node->get_logger(),
      "closed_chain.relative_rotation_xyzw must have 4 elements, got %zu; using default",
      rotation.size());
  }

  // ---- 奇异 ----
  auto & sg = p.singularity;
  sg.enabled = loader.get<bool>("singularity.enabled", sg.enabled);
  sg.min_singular_value =
    loader.get<double>("singularity.min_singular_value", sg.min_singular_value);
  sg.max_condition_number =
    loader.get<double>("singularity.max_condition_number", sg.max_condition_number);
  sg.degenerate_jacobian_epsilon = loader.get<double>(
    "singularity.degenerate_jacobian_epsilon", sg.degenerate_jacobian_epsilon);
  sg.allow_singular_start =
    loader.get<bool>("singularity.allow_singular_start", sg.allow_singular_start);

  // ---- 碰撞 ----
  auto & col = p.collision;
  col.check_self_collision =
    loader.get<bool>("collision.check_self_collision", col.check_self_collision);
  col.check_environment_collision =
    loader.get<bool>("collision.check_environment_collision", col.check_environment_collision);
  col.collect_all_contacts =
    loader.get<bool>("collision.collect_all_contacts", col.collect_all_contacts);
  col.max_contacts = static_cast<std::size_t>(
    loader.get<int64_t>("collision.max_contacts", static_cast<int64_t>(col.max_contacts)));

  // ---- 时间优化 ----
  auto & to = p.time_optimizer;
  to.enable_optimization =
    loader.get<bool>("time_optimizer.enable_optimization", to.enable_optimization);
  to.baseline_velocity_scaling = loader.get<double>(
    "time_optimizer.baseline_velocity_scaling", to.baseline_velocity_scaling);
  to.baseline_acceleration_scaling = loader.get<double>(
    "time_optimizer.baseline_acceleration_scaling", to.baseline_acceleration_scaling);
  to.optimized_velocity_scaling = loader.get<double>(
    "time_optimizer.optimized_velocity_scaling", to.optimized_velocity_scaling);
  to.optimized_acceleration_scaling = loader.get<double>(
    "time_optimizer.optimized_acceleration_scaling", to.optimized_acceleration_scaling);
  to.totg_path_tolerance =
    loader.get<double>("time_optimizer.totg_path_tolerance", to.totg_path_tolerance);
  to.totg_resample_dt =
    loader.get<double>("time_optimizer.totg_resample_dt", to.totg_resample_dt);
  to.totg_min_angle_change =
    loader.get<double>("time_optimizer.totg_min_angle_change", to.totg_min_angle_change);
  to.enable_ruckig_smoothing =
    loader.get<bool>("time_optimizer.enable_ruckig_smoothing", to.enable_ruckig_smoothing);
  to.limit_tolerance_ratio =
    loader.get<double>("time_optimizer.limit_tolerance_ratio", to.limit_tolerance_ratio);

  p.metrics.limit_tolerance_ratio =
    loader.get<double>("metrics.limit_tolerance_ratio", p.metrics.limit_tolerance_ratio);
  p.metrics.allow_finite_difference =
    loader.get<bool>("metrics.allow_finite_difference", p.metrics.allow_finite_difference);

  return p;
}

/// 打印一次规划结果的全部量化指标。
void reportResult(
  const rclcpp::Logger & logger, const std::string & scenario, const PlanResult & result)
{
  RCLCPP_INFO(logger, "---------- 场景 [%s] 结果 ----------", scenario.c_str());
  RCLCPP_INFO(
    logger, "错误码: %s (%s), 尝试次数: %d, 消息: %s",
    toString(result.code),
    result.succeeded() ? "成功" : (result.noActionNeeded() ? "无需动作" : "失败"),
    result.attempts_used, result.message.c_str());

  if (result.noActionNeeded()) {
    // 不走下面的失败分支：这不是故障，没什么要"定位"的。
    RCLCPP_INFO(logger, "无轨迹输出，因为当前构型已经满足目标，无需运动");
    return;
  }

  if (!result.succeeded()) {
    // 失败时也要把已知信息打全，便于定位。
    if (result.ik_failure_count > 0) {
      RCLCPP_WARN(logger, "follower IK 失败点数: %d", result.ik_failure_count);
    }
    RCLCPP_WARN(logger, "轨迹为空（按设计，失败时不输出任何轨迹）");
    return;
  }

  RCLCPP_INFO(
    logger, "关节轨迹: %zu 点, 关节数 %zu",
    result.trajectory.joint_trajectory.points.size(),
    result.trajectory.joint_trajectory.joint_names.size());
  RCLCPP_INFO(logger, "笛卡尔轨迹(leader TCP): %zu 个位姿", result.cartesian_path.size());
  if (!result.follower_cartesian_path.empty()) {
    RCLCPP_INFO(
      logger, "笛卡尔轨迹(follower TCP): %zu 个位姿",
      result.follower_cartesian_path.size());
  }

  // ---- 节拍指标 ----
  RCLCPP_INFO(logger, "节拍指标: %s", result.final_metrics.summary.c_str());
  RCLCPP_INFO(
    logger, "  总运动时长: %.3f s", result.final_metrics.duration);
  RCLCPP_INFO(
    logger, "  最大关节速度: %.4f rad/s (%s)",
    result.final_metrics.max_joint_velocity,
    result.final_metrics.max_velocity_joint.empty() ?
    "-" : result.final_metrics.max_velocity_joint.c_str());
  RCLCPP_INFO(
    logger, "  最大关节加速度: %.4f rad/s^2 (%s)",
    result.final_metrics.max_joint_acceleration,
    result.final_metrics.max_acceleration_joint.empty() ?
    "-" : result.final_metrics.max_acceleration_joint.c_str());
  RCLCPP_INFO(
    logger, "  速度利用率峰值: %.1f%% (越接近 100%% 说明节拍越紧)",
    result.final_metrics.peak_velocity_utilization);
  RCLCPP_INFO(
    logger, "  轨迹合法性: %s", result.final_metrics.isLegal() ? "合法" : "超限");

  // ---- 优化前后对比：任务要求"优化后节拍相比原始规划有缩短" ----
  RCLCPP_INFO(logger, "优化: %s", result.optimization.note.c_str());
  if (result.optimization.optimized_accepted) {
    RCLCPP_INFO(
      logger, "  已采纳 TOTG 优化，节拍缩短 %.1f%%",
      100.0 * result.optimization.duration_reduction_ratio);
  } else if (result.optimization.fell_back) {
    RCLCPP_WARN(logger, "  已回退到 baseline（优化结果不合法或未更快）");
  }

  // ---- 闭链残差 ----
  if (result.worst_residual.valid) {
    RCLCPP_INFO(
      logger, "闭链残差(全轨迹最差): 位置 %.6f m, 姿态 %.6f rad -> %s",
      result.worst_residual.position_error, result.worst_residual.orientation_error,
      result.worst_residual.within_tolerance ? "满足约束" : "超出阈值");
  }

  // ---- 奇异余量 ----
  if (result.worst_singularity.valid) {
    RCLCPP_INFO(
      logger, "奇异余量(全轨迹最差): sigma_min=%.6f, 条件数=%.2f",
      result.worst_singularity.min_singular_value,
      result.worst_singularity.condition_number);
  }
  if (result.ik_failure_count > 0) {
    RCLCPP_INFO(logger, "follower IK 失败点数: %d", result.ik_failure_count);
  }
}

}  // namespace astribot_s1_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "planning_demo_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const rclcpp::Logger logger = node->get_logger();

  // MoveGroupInterface 与 PlanningSceneMonitor 都需要节点被持续 spin，
  // 否则 action 结果和 /joint_states 回调都进不来（会表现为"规划永远超时"）。
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  int exit_code = 0;
  {
    using namespace astribot_s1_manipulation;  // NOLINT(build/namespaces)

    const DualArmPlannerParams params = loadParams(node);

    DualArmPlanner planner(node);
    std::string error;
    if (!planner.initialize(params, error)) {
      // URDF/SRDF 加载失败、规划器初始化失败 -> 优雅退出，明确日志，不产生段错误。
      RCLCPP_ERROR(logger, "规划器初始化失败，节点退出: %s", error.c_str());
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
      rclcpp::shutdown();
      return 1;
    }

    ParameterLoader loader(node);
    const std::vector<std::string> scenarios = loader.get<std::vector<std::string>>(
      "demo.scenarios", std::vector<std::string>{"single_arm_named"});
    const std::string single_arm_group =
      loader.get<std::string>("demo.single_arm_group", std::string("arm_left"));
    const std::string single_arm_named =
      loader.get<std::string>("demo.single_arm_named_target", std::string("ready"));
    const std::vector<double> single_arm_joints = loader.get<std::vector<double>>(
      "demo.single_arm_joint_target", std::vector<double>{});
    const std::vector<double> leader_joints = loader.get<std::vector<double>>(
      "demo.closed_chain_leader_joint_target", std::vector<double>{});
    const std::vector<std::string> comparison_planners =
      loader.get<std::vector<std::string>>(
      "demo.comparison_planners",
      std::vector<std::string>{"RRTstarConfig", "BITstarConfig", "InformedRRTstarConfig"});
    const bool execute = loader.get<bool>("demo.execute_trajectory", false);
    const bool move_to_ready = loader.get<bool>("demo.move_to_ready_first", true);

    RCLCPP_INFO(logger, "==================================================");
    RCLCPP_INFO(logger, "Astribot S1 双臂运动规划 demo");
    RCLCPP_INFO(logger, "  双臂组: %s", params.dual_arm_group.c_str());
    RCLCPP_INFO(logger, "  默认规划器: %s", params.planner_id.c_str());
    RCLCPP_INFO(
      logger, "  闭链: leader=%s(%s) follower=%s(%s)",
      params.closed_chain.leader_group.c_str(), params.closed_chain.leader_tcp_link.c_str(),
      params.closed_chain.follower_group.c_str(),
      params.closed_chain.follower_tcp_link.c_str());
    RCLCPP_INFO(logger, "  轨迹下发执行: %s", execute ? "开" : "关");
    RCLCPP_INFO(logger, "==================================================");

    // 先摆到 ready：ready 姿态刻意让肘部离开完全伸直的奇异构型
    // （joint_4 = 1.0），从奇异构型起步会让第一个点就被判奇异。
    if (move_to_ready) {
      for (const std::string & arm :
        {params.closed_chain.leader_group, params.closed_chain.follower_group})
      {
        SingleArmPlanRequest request;
        request.group = arm;
        request.named_target = "ready";
        const PlanResult result = planner.planSingleArm(request);
        RCLCPP_INFO(
          logger, "预备动作 [%s -> ready]: %s", arm.c_str(), toString(result.code));
        if (result.succeeded() && execute) {
          std::string message;
          planner.executeTrajectory(arm, result.trajectory, message);
        }
      }
    }

    for (const std::string & scenario : scenarios) {
      if (scenario == "single_arm_named") {
        SingleArmPlanRequest request;
        request.group = single_arm_group;
        request.named_target = single_arm_named;
        const PlanResult result = planner.planSingleArm(request);
        reportResult(logger, scenario, result);
        if (result.succeeded() && execute) {
          std::string message;
          planner.executeTrajectory(single_arm_group, result.trajectory, message);
        }
        // kAlreadyAtGoal 不是失败：没有轨迹可执行，但状态本身就是想要的结果。
        if (!result.succeeded() && !result.noActionNeeded()) {
          exit_code = 1;
        }
      } else if (scenario == "single_arm_joint") {
        if (single_arm_joints.empty()) {
          RCLCPP_WARN(logger, "跳过 single_arm_joint: demo.single_arm_joint_target 未配置");
          continue;
        }
        SingleArmPlanRequest request;
        request.group = single_arm_group;
        request.joint_target = single_arm_joints;
        const PlanResult result = planner.planSingleArm(request);
        reportResult(logger, scenario, result);
        if (result.succeeded() && execute) {
          std::string message;
          planner.executeTrajectory(single_arm_group, result.trajectory, message);
        }
        // kAlreadyAtGoal 不是失败：没有轨迹可执行，但状态本身就是想要的结果。
        if (!result.succeeded() && !result.noActionNeeded()) {
          exit_code = 1;
        }
      } else if (scenario == "closed_chain") {
        if (leader_joints.empty()) {
          RCLCPP_WARN(
            logger, "跳过 closed_chain: demo.closed_chain_leader_joint_target 未配置");
          continue;
        }
        ClosedChainPlanRequest request;
        request.leader_joint_target = leader_joints;
        request.capture_relative_pose_now = params.closed_chain.capture_from_current_state;
        const PlanResult result = planner.planClosedChain(request);
        reportResult(logger, scenario, result);
        if (result.succeeded() && execute) {
          std::string message;
          planner.executeTrajectory(params.dual_arm_group, result.trajectory, message);
        }
        // kAlreadyAtGoal 不是失败：没有轨迹可执行，但状态本身就是想要的结果。
        if (!result.succeeded() && !result.noActionNeeded()) {
          exit_code = 1;
        }
      } else if (scenario == "planner_comparison") {
        // 同一个任务分别用三个规划器跑，对比节拍与成功率。
        // 这是"三个规划器都真的可用"的自证：如果某个规划器没被注册，
        // MoveIt 会静默回退到默认规划器，那么它的节拍会与默认规划器一致 ——
        // 所以要连同 move_group 日志里的实际规划器名一起看。
        //
        // 目标优先用关节目标：机器人启动时往往已经在 named target(ready) 上，
        // 那样规划出来的是"1 个点、时长 0"的退化轨迹，节拍对比毫无意义。
        // 关节目标保证一定有真实运动可比。
        RCLCPP_INFO(logger, "---------- 规划器对比 ----------");
        if (single_arm_joints.empty()) {
          RCLCPP_WARN(
            logger,
            "demo.single_arm_joint_target 未配置，对比将退化为 named target；"
            "若机器人已在该姿态上，结果会是 1 点 0 时长的退化轨迹");
        }
        for (const std::string & planner_id : comparison_planners) {
          SingleArmPlanRequest request;
          request.group = single_arm_group;
          if (!single_arm_joints.empty()) {
            request.joint_target = single_arm_joints;
          } else {
            request.named_target = single_arm_named;
          }
          request.planner_id = planner_id;
          const PlanResult result = planner.planSingleArm(request);
          if (result.succeeded()) {
            RCLCPP_INFO(
              logger,
              "  %-24s 成功 | 时长 %.3fs | 最大速度 %.4f rad/s | %zu 点 | 尝试 %d 次 | "
              "速度利用率 %.1f%% | 最差 sigma_min %.4f",
              planner_id.c_str(), result.final_metrics.duration,
              result.final_metrics.max_joint_velocity,
              result.trajectory.joint_trajectory.points.size(), result.attempts_used,
              result.final_metrics.peak_velocity_utilization,
              result.worst_singularity.min_singular_value);
            if (result.trajectory.joint_trajectory.points.size() <= 1U) {
              RCLCPP_WARN(
                logger,
                "  ^ 只有 %zu 个轨迹点：起点与目标几乎相同，这条对比没有意义",
                result.trajectory.joint_trajectory.points.size());
            }
          } else {
            RCLCPP_WARN(
              logger, "  %-24s 失败 | %s | %s",
              planner_id.c_str(), toString(result.code), result.message.c_str());
            exit_code = 1;
          }
        }
      } else {
        RCLCPP_WARN(logger, "未知场景 '%s'，已跳过", scenario.c_str());
      }
    }

    RCLCPP_INFO(logger, "demo 全部场景执行完毕，退出码 %d", exit_code);
  }

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return exit_code;
}
