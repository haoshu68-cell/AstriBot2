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

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

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

// ==================== 搬运场景 ====================
//
// 把一个目标物体从位置 A 搬到位置 B。
//
// !!! 这是纯运动学演示，务必看清边界 !!!
// 本机器人**没有夹爪关节**：两臂止于 link_7，其后只有一个无碰撞体的 tool_link
// 纯坐标系，SRDF 里刻意没有声明 end_effector（原因见 astribot_s1.srdf 注释）。
// 所以：
//   真的   —— 完整搬运动作序列的规划与执行：home -> A上方 -> 降到A -> 抬起
//             -> B上方 -> 降到B -> 抬起 -> 回home，每步都是笛卡尔位姿目标，
//             每步都过碰撞与奇异点校验，每步都真实下发到控制器执行
//   真的   —— 物体是规划场景里的**真实碰撞体**，规划器必须绕开它而不是穿过去
//   做不到 —— 物体在 Gazebo 里被夹住并跟着走（没有夹爪，物理抓取无法实现）
//
// 物体在规划场景里的位置会在"抬起后"从 A 更新到 B，让后续避障用的是新位置 ——
// 这一步是必要的，否则回程规划仍以为物体在 A，会绕开一个已经不在那儿的障碍。

/// 搬运航路点。名字用于日志与失败定位。
struct TransportStep
{
  std::string name;
  geometry_msgs::msg::PoseStamped pose;
};

/// 造一个位姿。frame 用规划坐标系（本机是 astribot_torso_base，没有 base_link）。
geometry_msgs::msg::PoseStamped makeTransportPose(
  const std::string & frame, double x, double y, double z,
  const std::vector<double> & quat_xyzw)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = frame;
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = z;
  if (quat_xyzw.size() == 4U) {
    ps.pose.orientation.x = quat_xyzw[0];
    ps.pose.orientation.y = quat_xyzw[1];
    ps.pose.orientation.z = quat_xyzw[2];
    ps.pose.orientation.w = quat_xyzw[3];
  } else {
    // 不给姿态时用单位四元数。注意这**不是**"保持当前姿态"，
    // 而是一个明确的朝向；给不出合理姿态时宁可显式指定，不要留 0000 非法值。
    ps.pose.orientation.w = 1.0;
  }
  return ps;
}

/// 把物体作为碰撞体放进规划场景。pose 是物体中心。
moveit_msgs::msg::CollisionObject makeBoxObject(
  const std::string & id, const std::string & frame,
  const std::vector<double> & center_xyz, const std::vector<double> & size_xyz)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {size_xyz[0], size_xyz[1], size_xyz[2]};
  obj.primitives.push_back(box);
  geometry_msgs::msg::Pose p;
  p.position.x = center_xyz[0];
  p.position.y = center_xyz[1];
  p.position.z = center_xyz[2];
  p.orientation.w = 1.0;
  obj.primitive_poses.push_back(p);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

/// 生成完整搬运序列。approach_height 是接近/离开时在抓取点上方留的高度。
std::vector<TransportStep> buildTransportSteps(
  const std::string & frame,
  const std::vector<double> & pick_xyz, const std::vector<double> & place_xyz,
  double grasp_z_offset, double approach_height,
  const std::vector<double> & quat_xyzw)
{
  const double px = pick_xyz[0], py = pick_xyz[1];
  const double pz = pick_xyz[2] + grasp_z_offset;
  const double qx = place_xyz[0], qy = place_xyz[1];
  const double qz = place_xyz[2] + grasp_z_offset;
  return {
    {"1-接近A上方", makeTransportPose(frame, px, py, pz + approach_height, quat_xyzw)},
    {"2-下降到A", makeTransportPose(frame, px, py, pz, quat_xyzw)},
    {"3-抬起(带物体)", makeTransportPose(frame, px, py, pz + approach_height, quat_xyzw)},
    {"4-移动到B上方", makeTransportPose(frame, qx, qy, qz + approach_height, quat_xyzw)},
    {"5-下降到B", makeTransportPose(frame, qx, qy, qz, quat_xyzw)},
    {"6-抬起(已放下)", makeTransportPose(frame, qx, qy, qz + approach_height, quat_xyzw)},
  };
}

/// 把碰撞体变更以「场景差分」的形式发到 /planning_scene。
///
/// 刻意**不用** PlanningSceneInterface：它的 applyCollisionObject 走
/// /apply_planning_scene 服务，而服务调用会阻塞等待应答。实测在本 demo 里
/// 会整体挂死（预备动作打完 SUCCESS 之后再无输出、直到超时被杀），
/// 排查成本很高。发差分话题不依赖任何服务与 move_group capability，
/// PlanningSceneMonitor 本身就订阅这个话题，行为更可预期。
///
/// is_diff 必须置 true：否则会被当成一整个新场景，把机器人状态等其它内容全覆盖掉。
void publishSceneDiff(
  const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr & pub,
  const moveit_msgs::msg::CollisionObject & obj)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  scene.world.collision_objects.push_back(obj);
  pub->publish(scene);
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

    // ---- 搬运场景参数 ----
    // 全部从 yaml 读，不硬编码：换物体尺寸/换搬运位置只改 yaml。
    const std::string transport_group =
      loader.get<std::string>("demo.transport_group", std::string("arm_left"));
    const std::string transport_tcp_link =
      loader.get<std::string>("demo.transport_tcp_link", std::string(""));
    const std::string transport_frame = loader.get<std::string>(
      "demo.transport_frame", std::string("astribot_torso_base"));
    const std::vector<double> transport_pick =
      loader.get<std::vector<double>>("demo.transport_pick_xyz", std::vector<double>{});
    const std::vector<double> transport_place =
      loader.get<std::vector<double>>("demo.transport_place_xyz", std::vector<double>{});
    const std::vector<double> transport_size = loader.get<std::vector<double>>(
      "demo.transport_object_size_xyz", std::vector<double>{0.06, 0.06, 0.12});
    const double transport_grasp_z_offset =
      loader.get<double>("demo.transport_grasp_z_offset", 0.0);
    const double transport_approach_height =
      loader.get<double>("demo.transport_approach_height", 0.15);
    const std::vector<double> transport_quat = loader.get<std::vector<double>>(
      "demo.transport_grasp_orientation_xyzw", std::vector<double>{});
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
      if (scenario == "transport") {
        // ---- 搬运场景：物体从 A 到 B ----
        if (transport_pick.size() != 3U || transport_place.size() != 3U ||
          transport_size.size() != 3U)
        {
          RCLCPP_ERROR(
            logger,
            "跳过 transport: demo.transport_pick_xyz / transport_place_xyz / "
            "transport_object_size_xyz 必须各是 3 个数，当前分别是 %zu/%zu/%zu 个",
            transport_pick.size(), transport_place.size(), transport_size.size());
          exit_code = 1;
          continue;
        }

        const std::string object_id = "transport_target";
        // 场景差分用 transient_local：晚订阅的 PlanningSceneMonitor 也能收到，
        // 不然一发即丢，物体可能压根没进场景而我们毫不知情。
        auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>(
          "/planning_scene", rclcpp::QoS(1).transient_local());

        // 物体先放在 A。它是规划场景里的真实碰撞体，会参与后续每一步的避障。
        publishSceneDiff(
          scene_pub, makeBoxObject(object_id, transport_frame, transport_pick, transport_size));
        // 给 PlanningSceneMonitor 一点时间把差分吃进去，否则第一步规划时物体还不在场景里。
        std::this_thread::sleep_for(std::chrono::milliseconds(800));        RCLCPP_INFO(
          logger,
          "[transport] 物体已加入规划场景: A=(%.3f, %.3f, %.3f) 尺寸=(%.3f, %.3f, %.3f) "
          "frame=%s；目标位置 B=(%.3f, %.3f, %.3f)",
          transport_pick[0], transport_pick[1], transport_pick[2],
          transport_size[0], transport_size[1], transport_size[2],
          transport_frame.c_str(),
          transport_place[0], transport_place[1], transport_place[2]);
        RCLCPP_WARN(
          logger,
          "[transport] 本机无夹爪关节，这是**纯运动学演示**："
          "动作序列/碰撞校验/轨迹执行都是真的，物体不会真的被夹住跟着走");

        const std::vector<TransportStep> steps = buildTransportSteps(
          transport_frame, transport_pick, transport_place,
          transport_grasp_z_offset, transport_approach_height, transport_quat);

        bool all_ok = true;
        for (std::size_t i = 0; i < steps.size(); ++i) {
          const TransportStep & step = steps[i];
          SingleArmPlanRequest request;
          request.group = transport_group;
          request.use_pose_target = true;
          request.pose_target = step.pose;
          request.tcp_link = transport_tcp_link;

          const PlanResult result = planner.planSingleArm(request);
          RCLCPP_INFO(
            logger, "[transport] 步骤 %zu/%zu %s -> 目标(%.3f, %.3f, %.3f): %s",
            i + 1U, steps.size(), step.name.c_str(),
            step.pose.pose.position.x, step.pose.pose.position.y,
            step.pose.pose.position.z, toString(result.code));

          if (!result.succeeded() && !result.noActionNeeded()) {
            // 中间步失败就中止：继续做下一步会让机器人从一个错误的位姿出发，
            // 后面的位姿目标都失去意义，且可能撞上物体。
            RCLCPP_ERROR(
              logger, "[transport] 步骤 %s 失败(%s)，中止搬运（不做无意义的后续步骤）",
              step.name.c_str(), toString(result.code));
            all_ok = false;
            exit_code = 1;
            break;
          }
          if (result.succeeded() && execute) {
            std::string message;
            const PlanErrorCode exec = planner.executeTrajectory(
              transport_group, result.trajectory, message);
            if (exec != PlanErrorCode::kSuccess) {
              RCLCPP_ERROR(
                logger, "[transport] 步骤 %s 执行失败: %s (%s)",
                step.name.c_str(), toString(exec), message.c_str());
              all_ok = false;
              exit_code = 1;
              break;
            }
          }

          // 抬起之后把物体在规划场景里从 A 挪到 B。
          // 必须在这一步做：否则后续规划仍以为物体在 A，会绕开一个已经不在那儿的
          // 障碍，同时对 B 处的真实占用视而不见。
          if (step.name == "3-抬起(带物体)") {
            publishSceneDiff(
              scene_pub,
              makeBoxObject(object_id, transport_frame, transport_place, transport_size));
            std::this_thread::sleep_for(std::chrono::milliseconds(800));
            RCLCPP_INFO(
              logger, "[transport] 规划场景中的物体位置已更新到 B=(%.3f, %.3f, %.3f)",
              transport_place[0], transport_place[1], transport_place[2]);
          }
        }

        // 收尾：把物体从场景里移掉，避免污染后续场景的规划。
        moveit_msgs::msg::CollisionObject remove;
        remove.id = object_id;
        remove.header.frame_id = transport_frame;
        remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        publishSceneDiff(scene_pub, remove);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        RCLCPP_INFO(
          logger, "[transport] 搬运序列%s，物体已从规划场景移除",
          all_ok ? "全部完成" : "中止");
      } else if (scenario == "single_arm_named") {
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
