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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

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

  auto & ex = p.execution;
  ex.settle_timeout = loader.get<double>("execution.settle_timeout", ex.settle_timeout);
  ex.settle_poll_interval =
    loader.get<double>("execution.settle_poll_interval", ex.settle_poll_interval);
  ex.settle_velocity_threshold =
    loader.get<double>("execution.settle_velocity_threshold", ex.settle_velocity_threshold);
  ex.settle_position_epsilon =
    loader.get<double>("execution.settle_position_epsilon", ex.settle_position_epsilon);
  ex.settle_stable_samples = static_cast<int>(
    loader.get<int64_t>("execution.settle_stable_samples", ex.settle_stable_samples));

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

/// 量出「与 TCP 位置重合的那些连杆」的碰撞体外接半径。
///
/// 为什么需要这个函数（第2步"下降到A"失败的真正原因）：
/// 本机 TCP 是个**纯坐标系**（tool_link 无碰撞几何），很容易误以为"TCP 贴着物体
/// 顶面也没关系"。但 URDF 实测：
///   astribot_arm_left_tool_joint  fixed  link_7 -> tool_link   xyz = 0 0 0
///   astribot_arm_left_joint_7     revolute link_6 -> link_7    xyz = 0 0 0
///   link_7 collision: sphere r=0.05 @ (0.006, 0, 0)
///   link_6 collision: cylinder r=0.04 l=0.045 @ (0, 0, 0)
/// 三个坐标系原点**完全重合** —— 也就是说 TCP 正坐在一个半径 5cm 的腕部球心上。
/// TCP 离物体顶面只有 0.03m 时，腕部球已经嵌进物体 0.02m，目标状态必然碰撞，
/// 规划器怎么重试都无解，只会报无信息量的 RETRIES_EXHAUSTED。
///
/// 半径从 RobotModel（即 URDF）现算，不写死数字：换夹具、改 URDF 后自动跟着变。
/// 沿父链上溯的终止条件是"关节原点有平移"——有平移就说明那个连杆不再与 TCP 重合，
/// 它的碰撞体不该算进这个半径里（纯旋转的 joint_7 不算平移，仍要计入）。
double tcpCoincidentCollisionRadius(
  const moveit::core::RobotModelConstPtr & model, const std::string & tcp_link,
  std::string & detail)
{
  detail.clear();
  if (!model) {
    return 0.0;
  }
  // 判定"原点重合"的平移阈值。取 1e-6m：URDF 里写 0 就是精确 0，
  // 这个阈值只用来吸收浮点表示误差，不是工程容差。
  constexpr double kCoincidentEpsilon = 1.0e-6;

  double radius = 0.0;
  const moveit::core::LinkModel * link = model->getLinkModel(tcp_link);
  if (link == nullptr) {
    detail = "找不到 TCP link '" + tcp_link + "'";
    return 0.0;
  }
  while (link != nullptr) {
    if (!link->getShapes().empty()) {
      // getShapeExtentsAtOrigin 是该连杆所有碰撞体的 AABB 尺寸，
      // 其中心在 getCenteredBoundingBoxOffset()。外接半径取
      // "中心偏移量 + 半个最长边"，对球/圆柱都是安全上界。
      const double half_extent = 0.5 * link->getShapeExtentsAtOrigin().maxCoeff();
      const double offset = link->getCenteredBoundingBoxOffset().norm();
      const double link_radius = offset + half_extent;
      if (link_radius > radius) {
        radius = link_radius;
      }
      detail += (detail.empty() ? "" : ", ") + link->getName() + "=" +
        std::to_string(link_radius);
    }
    const moveit::core::LinkModel * parent = link->getParentLinkModel();
    if (parent == nullptr ||
      link->getJointOriginTransform().translation().norm() > kCoincidentEpsilon)
    {
      break;
    }
    link = parent;
  }
  return radius;
}


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

/// 单个候选 TCP 位姿的体检结论。
struct ProbeVerdict
{
  bool ik_ok{false};
  bool collision{false};
  bool singular{false};
  double sigma_min{0.0};
  double condition_number{0.0};
  std::string note;

  bool good() const noexcept
  {
    return ik_ok && !collision && !singular;
  }
};

/// 对一个候选 TCP 位姿做「IK + 碰撞 + 奇异」三项体检。
///
/// 为什么必须有这个函数（两轮实测教训）：
///   第一轮 —— 用 /compute_ik 扫可达域时物体**不在**场景里，扫出来的"可达点"
///             在物体入场之后照样碰撞，白扫。
///   第二轮 —— 改用一个只看 IK 的外部 python 探针，它对 sigma_min 一无所知，
///             于是选出的点被本工程自己的奇异监视器否掉（实测 sigma_min=0.0161
///             < 阈值 0.02），探针说"可以"、真跑说"不行" —— 两边判据不是一套。
/// 结论：探针必须与真跑**共用同一套判据**。所以它写在这里，直接复用
/// CollisionValidator / SingularityMonitor 和同一份 yaml 参数，不再另起一套标准。
///
/// 局限（要如实知道）：这里的碰撞场景只含机器人自身 + 我们放进去的那个物体，
/// 不含 Gazebo 里的环境。对本 demo 够用（物体是唯一环境障碍），
/// 但不能当成"真跑一定不碰"的证明。
ProbeVerdict probeTcpPose(
  const moveit::core::RobotState & seed,
  const moveit::core::JointModelGroup * jmg,
  const std::string & tcp_link,
  const geometry_msgs::msg::PoseStamped & pose,
  double ik_timeout,
  const CollisionValidator & collision,
  const SingularityMonitor & singularity)
{
  ProbeVerdict verdict;
  if (jmg == nullptr) {
    verdict.note = "规划组为空";
    return verdict;
  }

  moveit::core::RobotState state(seed);
  // 位姿给的是 transport_frame，IK 要的是模型坐标系。用 getFrameTransform 换算，
  // 不假设两者相同（本机恰好都是 astribot_torso_base，但换模型就未必）。
  Eigen::Isometry3d target;
  tf2::fromMsg(pose.pose, target);
  const Eigen::Isometry3d frame_in_model = state.getFrameTransform(pose.header.frame_id);
  const Eigen::Isometry3d target_in_model = frame_in_model * target;

  if (!state.setFromIK(jmg, target_in_model, tcp_link, ik_timeout)) {
    verdict.note = "IK 无解";
    return verdict;
  }
  verdict.ik_ok = true;
  state.update();

  const CollisionReport col = collision.check(state, jmg->getName());
  verdict.collision = col.collision;
  if (col.collision) {
    verdict.note = "碰撞: " + col.reason;
  }

  const SingularityReport sing = singularity.check(state, jmg, tcp_link);
  if (sing.valid) {
    verdict.sigma_min = sing.min_singular_value;
    verdict.condition_number = sing.condition_number;
  }
  verdict.singular = sing.singular;
  if (sing.singular) {
    verdict.note += (verdict.note.empty() ? "" : "; ") + std::string("奇异: ") + sing.reason;
  }
  return verdict;
}

// ============================================================================
// mobile_transport 场景用的导航与 TF 辅助
//
// 为什么全部用「阻塞 + future.wait_for()」而不是 spin_until_future_complete：
// 本节点已经由 main() 里的后台线程在 spin 执行器了。再调
// spin_until_future_complete 会有两个执行器抢同一个节点，行为不可预期。
// future.wait_for() 只是等，回调仍在 spin 线程上跑完并把 future 置就绪 ——
// 这才是"节点已被别人 spin"时的正确等待方式。
// ============================================================================

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;

/// 导航一段的结果。刻意不用异常：任务要求规划/执行接口返回明确状态码。
struct NavOutcome
{
  bool succeeded{false};
  std::string reason;
  double elapsed_sec{0.0};
};

/// 造一个 map 系目标位姿。
geometry_msgs::msg::PoseStamped makeNavGoal(
  const std::string & frame, double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = frame;
  // stamp 刻意留 0（= "用最新可用的变换"）。
  // 实测踩坑：仿真下用墙钟 now() 打时间戳，planner_server 会报
  //   Could not transform the start or goal pose in the costmap frame
  // 因为 tf 里全是 sim time，墙钟戳落在未来。
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.orientation.z = std::sin(0.5 * yaw);
  ps.pose.orientation.w = std::cos(0.5 * yaw);
  return ps;
}

/// 用 ComputePathToPose 预检目标可达性。
///
/// 为什么必须预检（实测教训）：不预检时，"目标本来就不可达"会表现成
/// NavigateToPose 长时间挣扎后 ABORTED，很容易被误读成"局部规划器走不动"，
/// 于是跑去调 MPPI 参数 —— 方向完全错。先问全局规划器一句最便宜。
bool verifyGoalReachable(
  const rclcpp::Node::SharedPtr & node, const std::string & action_name,
  const geometry_msgs::msg::PoseStamped & goal, double timeout_sec, std::string & why)
{
  why.clear();
  auto client = rclcpp_action::create_client<ComputePathToPose>(node, action_name);
  const auto wait = std::chrono::duration<double>(std::min(timeout_sec, 10.0));
  if (!client->wait_for_action_server(
      std::chrono::duration_cast<std::chrono::nanoseconds>(wait)))
  {
    why = "全局规划动作 " + action_name + " 未就绪（nav2 起了吗？）";
    return false;
  }

  ComputePathToPose::Goal request;
  request.goal = goal;
  request.use_start = false;   // 用机器人当前位姿作起点

  auto goal_future = client->async_send_goal(request);
  const auto budget = std::chrono::duration<double>(timeout_sec);
  if (goal_future.wait_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(budget)) !=
    std::future_status::ready)
  {
    why = "全局规划目标下发超时";
    return false;
  }
  auto handle = goal_future.get();
  if (!handle) {
    why = "全局规划目标被拒绝";
    return false;
  }
  auto result_future = client->async_get_result(handle);
  if (result_future.wait_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(budget)) !=
    std::future_status::ready)
  {
    why = "全局规划结果等待超时";
    return false;
  }
  const auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
    why = "全局规划未成功（目标可能落在障碍/未知区，或与当前位置不连通）";
    return false;
  }
  if (result.result->path.poses.size() < 2U) {
    why = "全局路径只有 " + std::to_string(result.result->path.poses.size()) + " 个点";
    return false;
  }
  return true;
}

/// 阻塞跑完一次 NavigateToPose。
NavOutcome runNavigation(
  const rclcpp::Node::SharedPtr & node, const std::string & action_name,
  const geometry_msgs::msg::PoseStamped & goal, double timeout_sec)
{
  NavOutcome outcome;
  const auto begin = std::chrono::steady_clock::now();
  const auto stamp_elapsed = [&outcome, begin]() {
      outcome.elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    };

  auto client = rclcpp_action::create_client<NavigateToPose>(node, action_name);
  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    outcome.reason = "导航动作 " + action_name + " 未就绪（nav2 起了吗？）";
    stamp_elapsed();
    return outcome;
  }

  NavigateToPose::Goal request;
  request.pose = goal;

  auto goal_future = client->async_send_goal(request);
  if (goal_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    outcome.reason = "导航目标下发超时";
    stamp_elapsed();
    return outcome;
  }
  auto handle = goal_future.get();
  if (!handle) {
    outcome.reason = "导航目标被 bt_navigator 拒绝";
    stamp_elapsed();
    return outcome;
  }

  auto result_future = client->async_get_result(handle);
  const auto budget = std::chrono::duration<double>(timeout_sec);
  if (result_future.wait_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(budget)) !=
    std::future_status::ready)
  {
    // 超时必须主动取消，否则机器人会在我们已经放弃之后继续往目标开 ——
    // 而上层以为流程已经中止，接下来的机械臂动作就发生在一个意料之外的位置。
    client->async_cancel_goal(handle);
    outcome.reason = "导航超过 " + std::to_string(static_cast<int>(timeout_sec)) +
      "s 未结束，已发取消";
    stamp_elapsed();
    return outcome;
  }

  const auto result = result_future.get();
  stamp_elapsed();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      outcome.succeeded = true;
      outcome.reason = "SUCCEEDED";
      return outcome;
    case rclcpp_action::ResultCode::ABORTED:
      // 状态码语义容易记错：4=SUCCEEDED / 5=CANCELED / 6=ABORTED。
      // ABORTED 是 nav2 主动放弃（恢复行为也用尽了），不是我们取消的。
      outcome.reason = "ABORTED（nav2 主动放弃，恢复行为已用尽）";
      return outcome;
    case rclcpp_action::ResultCode::CANCELED:
      outcome.reason = "CANCELED";
      return outcome;
    default:
      outcome.reason = "UNKNOWN 结果码";
      return outcome;
  }
}

/// 查 map -> 体系 的变换，把一个体系坐标点换算到 map 系。
///
/// 刻意用 TimePointZero + 重试循环，**不用**带 timeout 的 lookupTransform：
/// 本函数在主线程（非执行器线程）里调，带 timeout 的版本对**动态 tf** 会失败，
/// 而静态 tf 却正常 —— 于是看着像没问题，实际拿不到 map->odom->base 这条链。
bool bodyPointToMap(
  tf2_ros::Buffer & buffer, const std::string & map_frame, const std::string & body_frame,
  const std::vector<double> & body_xyz, std::array<double, 3> & out, std::string & why)
{
  why.clear();
  if (body_xyz.size() != 3U) {
    why = "体系坐标不是 3 个数";
    return false;
  }
  constexpr int kRetries = 40;                       // 40 × 50ms = 2s
  for (int i = 0; i < kRetries; ++i) {
    try {
      const geometry_msgs::msg::TransformStamped tf =
        buffer.lookupTransform(map_frame, body_frame, tf2::TimePointZero);
      Eigen::Isometry3d body_in_map;
      body_in_map = tf2::transformToEigen(tf);
      const Eigen::Vector3d p =
        body_in_map * Eigen::Vector3d(body_xyz[0], body_xyz[1], body_xyz[2]);
      out = {p.x(), p.y(), p.z()};
      return true;
    } catch (const tf2::TransformException & e) {
      why = e.what();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  return false;
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
    // 腕部碰撞体与物体顶面之间要留的净空。它**只是余量**，
    // 腕部半径本身由 URDF 现算，不在这个数里。
    const double transport_clearance_margin =
      loader.get<double>("demo.transport_clearance_margin", 0.02);
    // transport_probe 场景的扫描网格（TCP 抓取点坐标，不是物体中心）。
    const std::vector<double> probe_x_list = loader.get<std::vector<double>>(
      "demo.transport_probe_x_list", std::vector<double>{});
    const std::vector<double> probe_y_list = loader.get<std::vector<double>>(
      "demo.transport_probe_y_list", std::vector<double>{});
    const std::vector<double> probe_z_list = loader.get<std::vector<double>>(
      "demo.transport_probe_z_list", std::vector<double>{});
    // mobile_transport（搬运 -> 导航 -> 搬运）的导航段参数。
    const std::string mobile_map_frame = loader.get<std::string>(
      "demo.mobile_transport_map_frame", std::string("map"));
    const std::string mobile_nav_action = loader.get<std::string>(
      "demo.mobile_transport_nav_action", std::string("/navigate_to_pose"));
    const std::string mobile_plan_action = loader.get<std::string>(
      "demo.mobile_transport_plan_action", std::string("/compute_path_to_pose"));
    const std::vector<double> mobile_nav_goal_xy = loader.get<std::vector<double>>(
      "demo.mobile_transport_nav_goal_xy", std::vector<double>{});
    const double mobile_nav_goal_yaw =
      loader.get<double>("demo.mobile_transport_nav_goal_yaw", 0.0);
    const double mobile_nav_timeout =
      loader.get<double>("demo.mobile_transport_nav_timeout", 180.0);
    const bool mobile_verify_reachable =
      loader.get<bool>("demo.mobile_transport_verify_reachable", true);
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
      if (scenario == "transport_probe") {
        // ---- 选点体检：扫一遍候选 A 点，报告 IK / 碰撞 / 奇异 ----
        // 这个场景不动机器人，只出一张表，用来在配 transport_pick_xyz 之前
        // 先知道哪些点是真能用的。判据与 transport 真跑完全一致（同一份 yaml、
        // 同一个 CollisionValidator / SingularityMonitor）。
        if (transport_size.size() != 3U) {
          RCLCPP_ERROR(logger, "跳过 transport_probe: transport_object_size_xyz 必须是 3 个数");
          exit_code = 1;
          continue;
        }
        const moveit::core::RobotModelConstPtr model = planner.getRobotModel();
        const moveit::core::JointModelGroup * jmg = model->getJointModelGroup(transport_group);
        if (jmg == nullptr) {
          RCLCPP_ERROR(
            logger, "跳过 transport_probe: 规划组 '%s' 不存在", transport_group.c_str());
          exit_code = 1;
          continue;
        }
        const std::string probe_tcp_link = transport_tcp_link.empty() ?
          (transport_group == params.closed_chain.leader_group ?
          params.closed_chain.leader_tcp_link :
          params.closed_chain.follower_tcp_link) :
          transport_tcp_link;

        moveit::core::RobotStatePtr seed;
        std::string seed_error;
        if (!planner.getCurrentState(seed, seed_error)) {
          RCLCPP_ERROR(logger, "跳过 transport_probe: 取当前状态失败: %s", seed_error.c_str());
          exit_code = 1;
          continue;
        }

        SingularityMonitor probe_singularity;
        std::string probe_error;
        if (!probe_singularity.configure(params.singularity, probe_error)) {
          RCLCPP_ERROR(logger, "跳过 transport_probe: 奇异监视器配置失败: %s", probe_error.c_str());
          exit_code = 1;
          continue;
        }

        RCLCPP_INFO(
          logger,
          "[probe] 组=%s TCP=%s 判据: sigma_min>=%.4f 条件数<=%.1f；"
          "物体尺寸=(%.3f,%.3f,%.3f) grasp_z_offset=%.3f approach=%.3f",
          transport_group.c_str(), probe_tcp_link.c_str(),
          params.singularity.min_singular_value, params.singularity.max_condition_number,
          transport_size[0], transport_size[1], transport_size[2],
          transport_grasp_z_offset, transport_approach_height);
        RCLCPP_INFO(
          logger,
          "[probe] 每个候选按「抓取点 + 接近点」成对判定，两点全过才算可用；"
          "物体中心随候选放在 抓取z - grasp_z_offset");

        std::size_t good_count = 0U;
        double best_sigma = -1.0;
        std::vector<double> best_xyz;
        for (const double px : probe_x_list) {
          for (const double py : probe_y_list) {
            for (const double pz : probe_z_list) {
              // 物体跟着候选点走：它的中心由抓取高度反推，这样每个候选测的都是
              // 它自己那套几何关系，而不是拿固定物体位置去套所有候选。
              const std::vector<double> object_center{px, py, pz - transport_grasp_z_offset};
              auto probe_scene = std::make_shared<planning_scene::PlanningScene>(model);
              const moveit_msgs::msg::CollisionObject probe_obj =
                makeBoxObject("probe_target", transport_frame, object_center, transport_size);
              if (!probe_scene->processCollisionObjectMsg(probe_obj)) {
                RCLCPP_WARN(
                  logger, "[probe] (%.3f,%.3f,%.3f) 物体注入局部场景失败，跳过", px, py, pz);
                continue;
              }
              CollisionValidator probe_collision;
              if (!probe_collision.configure(probe_scene, params.collision, probe_error)) {
                RCLCPP_WARN(
                  logger, "[probe] 碰撞校验器配置失败: %s", probe_error.c_str());
                continue;
              }

              const ProbeVerdict grasp = probeTcpPose(
                *seed, jmg, probe_tcp_link,
                makeTransportPose(transport_frame, px, py, pz, transport_quat),
                params.closed_chain.ik_timeout, probe_collision, probe_singularity);
              const ProbeVerdict approach = probeTcpPose(
                *seed, jmg, probe_tcp_link,
                makeTransportPose(
                  transport_frame, px, py, pz + transport_approach_height, transport_quat),
                params.closed_chain.ik_timeout, probe_collision, probe_singularity);

              const bool pair_ok = grasp.good() && approach.good();
              const double pair_sigma = std::min(grasp.sigma_min, approach.sigma_min);
              RCLCPP_INFO(
                logger,
                "[probe] (%.3f, %.3f, %.3f) %s | 抓取: ik=%d col=%d sing=%d σ=%.4f κ=%.1f%s%s"
                " | 接近: ik=%d col=%d sing=%d σ=%.4f κ=%.1f%s%s",
                px, py, pz, pair_ok ? "可用" : "不可用",
                grasp.ik_ok ? 1 : 0, grasp.collision ? 1 : 0, grasp.singular ? 1 : 0,
                grasp.sigma_min, grasp.condition_number,
                grasp.note.empty() ? "" : " ", grasp.note.c_str(),
                approach.ik_ok ? 1 : 0, approach.collision ? 1 : 0, approach.singular ? 1 : 0,
                approach.sigma_min, approach.condition_number,
                approach.note.empty() ? "" : " ", approach.note.c_str());

              if (pair_ok) {
                ++good_count;
                if (pair_sigma > best_sigma) {
                  best_sigma = pair_sigma;
                  best_xyz = {px, py, pz};
                }
              }
            }
          }
        }

        const std::size_t total =
          probe_x_list.size() * probe_y_list.size() * probe_z_list.size();
        if (good_count == 0U) {
          RCLCPP_ERROR(
            logger,
            "[probe] %zu 个候选全部不可用。请扩大 demo.transport_probe_* 的扫描范围，"
            "或先确认 transport_grasp_orientation_xyzw 是该臂真能做到的姿态",
            total);
          exit_code = 1;
        } else {
          RCLCPP_INFO(
            logger,
            "[probe] %zu/%zu 个候选可用；最佳(σ 最大)= (%.3f, %.3f, %.3f) σ=%.4f。"
            "把它填进 demo.transport_pick_xyz 时记得减去 grasp_z_offset=%.3f "
            "（yaml 里给的是**物体中心**，探针报的是 TCP 高度）=> 物体中心 z=%.3f",
            good_count, total, best_xyz[0], best_xyz[1], best_xyz[2], best_sigma,
            transport_grasp_z_offset, best_xyz[2] - transport_grasp_z_offset);
        }
      } else if (scenario == "transport") {
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

        // ---- 抓取高度边界校验 ----
        // 这道校验是拿实测教训换来的：TCP 只离物体顶面 0.03m 时，腕部那颗
        // r=0.05 的碰撞球已经嵌进物体，目标状态非法，规划器只会报
        // RETRIES_EXHAUSTED —— 一个完全看不出根因的错误。宁可在这里带着
        // 具体数字直接拒绝，也不要让它在第2步失败后让人去猜。
        // TCP link 的解析规则必须与 planSingleArm 内部**完全一致**，
        // 否则这里量的是一个 link、规划器用的是另一个，校验就成了摆设。
        const std::string resolved_tcp_link = transport_tcp_link.empty() ?
          (transport_group == params.closed_chain.leader_group ?
          params.closed_chain.leader_tcp_link :
          params.closed_chain.follower_tcp_link) :
          transport_tcp_link;
        std::string radius_detail;
        const double tcp_radius = tcpCoincidentCollisionRadius(
          planner.getRobotModel(), resolved_tcp_link, radius_detail);
        const double min_grasp_z_offset =
          0.5 * transport_size[2] + tcp_radius + transport_clearance_margin;
        RCLCPP_INFO(
          logger,
          "[transport] TCP link=%s 与其重合连杆的碰撞外接半径=%.4fm (%s)；"
          "物体半高=%.4fm 余量=%.4fm => grasp_z_offset 下限=%.4fm，当前=%.4fm",
          resolved_tcp_link.c_str(), tcp_radius, radius_detail.c_str(),
          0.5 * transport_size[2], transport_clearance_margin,
          min_grasp_z_offset, transport_grasp_z_offset);
        if (transport_grasp_z_offset < min_grasp_z_offset) {
          RCLCPP_ERROR(
            logger,
            "跳过 transport: demo.transport_grasp_z_offset=%.4f 小于下限 %.4f，"
            "TCP 会与物体碰撞（腕部碰撞球半径 %.4f + 物体半高 %.4f + 余量 %.4f）。"
            "请把 transport_grasp_z_offset 提到 %.4f 以上，"
            "并把 transport_pick_xyz/place_xyz 的 z 相应下调以保持 TCP 落在可达高度。",
            transport_grasp_z_offset, min_grasp_z_offset, tcp_radius,
            0.5 * transport_size[2], transport_clearance_margin, min_grasp_z_offset);
          exit_code = 1;
          continue;
        }

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
        // 逐步量化指标。做求解器横向对比时，光有"6 步全过"是不够的 ——
        // 必须能比出规划耗时、节拍、最差奇异值，否则"换个求解器也能过"
        // 说明不了它到底更好还是更差。
        double total_plan_wall = 0.0;
        double total_traj_duration = 0.0;
        double worst_sigma_all = std::numeric_limits<double>::infinity();
        int total_attempts = 0;
        std::size_t steps_done = 0U;
        for (std::size_t i = 0; i < steps.size(); ++i) {
          const TransportStep & step = steps[i];
          SingleArmPlanRequest request;
          request.group = transport_group;
          request.use_pose_target = true;
          request.pose_target = step.pose;
          request.tcp_link = transport_tcp_link;

          const auto plan_begin = std::chrono::steady_clock::now();
          const PlanResult result = planner.planSingleArm(request);
          const double plan_wall = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - plan_begin).count();
          total_plan_wall += plan_wall;
          total_attempts += result.attempts_used;
          if (result.succeeded()) {
            ++steps_done;
            total_traj_duration += result.final_metrics.duration;
            if (result.worst_singularity.valid) {
              worst_sigma_all =
                std::min(worst_sigma_all, result.worst_singularity.min_singular_value);
            }
          }
          RCLCPP_INFO(
            logger,
            "[transport] 步骤 %zu/%zu %s -> 目标(%.3f, %.3f, %.3f): %s"
            " | 规划耗时 %.3fs 尝试 %d 次 节拍 %.3fs 点数 %zu 最差σ %.4f",
            i + 1U, steps.size(), step.name.c_str(),
            step.pose.pose.position.x, step.pose.pose.position.y,
            step.pose.pose.position.z, toString(result.code),
            plan_wall, result.attempts_used, result.final_metrics.duration,
            result.trajectory.joint_trajectory.points.size(),
            result.worst_singularity.valid ?
            result.worst_singularity.min_singular_value : -1.0);

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
        // 一行汇总，专门给"换求解器再跑一遍"的横向对比用（grep 这一行即可）。
        RCLCPP_INFO(
          logger,
          "[transport][summary] planner=%s 结果=%s 成功步数=%zu/%zu "
          "规划总耗时=%.3fs 轨迹总节拍=%.3fs 总尝试=%d 全程最差σ=%.4f 执行=%s",
          params.planner_id.c_str(), all_ok ? "PASS" : "FAIL",
          steps_done, steps.size(), total_plan_wall, total_traj_duration, total_attempts,
          std::isfinite(worst_sigma_all) ? worst_sigma_all : -1.0,
          execute ? "true" : "false");
      } else if (scenario == "mobile_transport") {
        // ---- 移动作业：起始位置取货 -> 底盘导航 -> 目标位置放货 ----
        //
        // 关键设计：`buildTransportSteps()` 那六步**一行都不用改**。
        // 第 4 步"移动到 B 上方"是体系内的横移，底盘挪没挪都成立，
        // 所以整个流程就是 transport + 在第 3 步(抬起)之后插入一段导航。
        //
        // 物体坐标系为什么用体系 astribot_torso_base 就够（不需要
        // AttachedCollisionObject）：导航途中机械臂**保持抬起姿态不动**，
        // 物体与机器人的相对位姿恒定，体系坐标严格成立，物体自然跟着走。
        // !!! 如果以后要在导航途中收臂，这个前提就破了，必须改成把物体
        // 附着到 TCP link 上（AttachedCollisionObject + touch_links）。!!!
        if (transport_pick.size() != 3U || transport_place.size() != 3U ||
          transport_size.size() != 3U)
        {
          RCLCPP_ERROR(
            logger,
            "跳过 mobile_transport: transport_pick_xyz / transport_place_xyz / "
            "transport_object_size_xyz 必须各是 3 个数");
          exit_code = 1;
          continue;
        }
        if (mobile_nav_goal_xy.size() != 2U) {
          RCLCPP_ERROR(
            logger,
            "跳过 mobile_transport: demo.mobile_transport_nav_goal_xy 必须是 2 个数(map 系 x,y)，"
            "当前是 %zu 个", mobile_nav_goal_xy.size());
          exit_code = 1;
          continue;
        }

        // 抓取高度下限校验：与 transport 用同一套判据，不另立标准。
        const std::string mobile_tcp_link = transport_tcp_link.empty() ?
          (transport_group == params.closed_chain.leader_group ?
          params.closed_chain.leader_tcp_link :
          params.closed_chain.follower_tcp_link) :
          transport_tcp_link;
        std::string mobile_radius_detail;
        const double mobile_tcp_radius = tcpCoincidentCollisionRadius(
          planner.getRobotModel(), mobile_tcp_link, mobile_radius_detail);
        const double mobile_min_offset =
          0.5 * transport_size[2] + mobile_tcp_radius + transport_clearance_margin;
        if (transport_grasp_z_offset < mobile_min_offset) {
          RCLCPP_ERROR(
            logger,
            "跳过 mobile_transport: transport_grasp_z_offset=%.4f 小于下限 %.4f"
            "（腕部碰撞球 %.4f + 物体半高 %.4f + 余量 %.4f），TCP 会与物体碰撞",
            transport_grasp_z_offset, mobile_min_offset, mobile_tcp_radius,
            0.5 * transport_size[2], transport_clearance_margin);
          exit_code = 1;
          continue;
        }

        // TF：用来把物体的体系坐标换算到 map 系，证明它真的在世界里被搬走了。
        tf2_ros::Buffer tf_buffer(node->get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer, node);

        const std::string object_id = "transport_target";
        auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>(
          "/planning_scene", rclcpp::QoS(1).transient_local());
        publishSceneDiff(
          scene_pub, makeBoxObject(object_id, transport_frame, transport_pick, transport_size));
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        RCLCPP_INFO(
          logger,
          "[mobile] 物体入场景 A=(%.3f, %.3f, %.3f) frame=%s；"
          "导航目标 map=(%.3f, %.3f) yaw=%.3f；放置点 B=(%.3f, %.3f, %.3f)",
          transport_pick[0], transport_pick[1], transport_pick[2], transport_frame.c_str(),
          mobile_nav_goal_xy[0], mobile_nav_goal_xy[1], mobile_nav_goal_yaw,
          transport_place[0], transport_place[1], transport_place[2]);
        RCLCPP_WARN(
          logger,
          "[mobile] 本机无夹爪，纯运动学演示；导航途中机械臂保持抬起姿态不收臂 —— "
          "伸出的手臂与物体超出了代价地图那个 0.42m 外接足迹，nav2 看不到它");

        const std::vector<TransportStep> steps = buildTransportSteps(
          transport_frame, transport_pick, transport_place,
          transport_grasp_z_offset, transport_approach_height, transport_quat);

        double mobile_plan_wall = 0.0;
        double mobile_worst_sigma = std::numeric_limits<double>::infinity();
        std::size_t mobile_steps_done = 0U;
        bool mobile_ok = true;

        // 一步"规划 + 校验 + 执行"。失败返回 false，由调用处决定中止。
        const auto run_arm_step = [&](std::size_t index) -> bool {
            const TransportStep & step = steps[index];
            SingleArmPlanRequest request;
            request.group = transport_group;
            request.use_pose_target = true;
            request.pose_target = step.pose;
            request.tcp_link = transport_tcp_link;

            const auto begin = std::chrono::steady_clock::now();
            const PlanResult result = planner.planSingleArm(request);
            const double wall =
              std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
            mobile_plan_wall += wall;
            RCLCPP_INFO(
              logger,
              "[mobile] 步骤 %zu/%zu %s -> 目标(%.3f, %.3f, %.3f): %s"
              " | 规划 %.3fs 尝试 %d 次 节拍 %.3fs 最差σ %.4f",
              index + 1U, steps.size(), step.name.c_str(),
              step.pose.pose.position.x, step.pose.pose.position.y,
              step.pose.pose.position.z, toString(result.code), wall,
              result.attempts_used, result.final_metrics.duration,
              result.worst_singularity.valid ?
              result.worst_singularity.min_singular_value : -1.0);

            if (!result.succeeded() && !result.noActionNeeded()) {
              RCLCPP_ERROR(
                logger, "[mobile] 步骤 %s 失败(%s)，中止流程",
                step.name.c_str(), toString(result.code));
              return false;
            }
            if (result.succeeded()) {
              ++mobile_steps_done;
              if (result.worst_singularity.valid) {
                mobile_worst_sigma = std::min(
                  mobile_worst_sigma, result.worst_singularity.min_singular_value);
              }
            }
            if (result.succeeded() && execute) {
              std::string exec_message;
              const PlanErrorCode exec = planner.executeTrajectory(
                transport_group, result.trajectory, exec_message);
              if (exec != PlanErrorCode::kSuccess) {
                RCLCPP_ERROR(
                  logger, "[mobile] 步骤 %s 执行失败: %s (%s)",
                  step.name.c_str(), toString(exec), exec_message.c_str());
                return false;
              }
            }
            return true;
          };

        // ---- 阶段 1：起始位置取货（步骤 1~3）----
        RCLCPP_INFO(logger, "[mobile] ===== 阶段 1/3：起始位置取货 =====");
        for (std::size_t i = 0; i < 3U && mobile_ok; ++i) {
          mobile_ok = run_arm_step(i);
        }

        // 记录物体此刻的 map 系位置。查不到 TF 只降级为 WARN ——
        // 它只影响"报告世界位移"这一件事，不是流程本身的一环。
        std::array<double, 3> object_map_before{0.0, 0.0, 0.0};
        std::array<double, 3> base_map_before{0.0, 0.0, 0.0};
        bool have_before = false;
        if (mobile_ok) {
          std::string tf_why;
          const bool a = bodyPointToMap(
            tf_buffer, mobile_map_frame, transport_frame, transport_pick,
            object_map_before, tf_why);
          const bool b = bodyPointToMap(
            tf_buffer, mobile_map_frame, transport_frame, std::vector<double>{0.0, 0.0, 0.0},
            base_map_before, tf_why);
          have_before = a && b;
          if (have_before) {
            RCLCPP_INFO(
              logger,
              "[mobile] 取货后：物体 map=(%.3f, %.3f, %.3f) 底盘 map=(%.3f, %.3f)",
              object_map_before[0], object_map_before[1], object_map_before[2],
              base_map_before[0], base_map_before[1]);
          } else {
            RCLCPP_WARN(logger, "[mobile] 取不到 map 系位置(%s)，世界位移无法报告", tf_why.c_str());
          }
        }

        // ---- 阶段 2：底盘导航 ----
        NavOutcome nav;
        if (mobile_ok) {
          RCLCPP_INFO(logger, "[mobile] ===== 阶段 2/3：底盘导航 =====");
          const geometry_msgs::msg::PoseStamped nav_goal = makeNavGoal(
            mobile_map_frame, mobile_nav_goal_xy[0], mobile_nav_goal_xy[1],
            mobile_nav_goal_yaw);

          if (mobile_verify_reachable) {
            std::string why;
            if (!verifyGoalReachable(node, mobile_plan_action, nav_goal, 30.0, why)) {
              // 关键：不可达就**不发**导航目标。否则会表现成机器人挣扎很久后
              // ABORTED，而那看着像"局部规划器走不动"，排查方向完全错。
              RCLCPP_ERROR(
                logger, "[mobile] 导航目标预检不通过: %s —— 不下发导航目标，中止流程",
                why.c_str());
              mobile_ok = false;
            } else {
              RCLCPP_INFO(logger, "[mobile] 导航目标预检通过（全局路径存在）");
            }
          }

          if (mobile_ok) {
            nav = runNavigation(node, mobile_nav_action, nav_goal, mobile_nav_timeout);
            RCLCPP_INFO(
              logger, "[mobile] 导航结果: %s，耗时 %.1fs",
              nav.reason.c_str(), nav.elapsed_sec);
            if (!nav.succeeded) {
              // 导航失败绝不能继续放货：机器人不在预期的世界位置上，
              // 后面那三步会把物体"放"在一个错误的地方，还报成功。
              RCLCPP_ERROR(
                logger, "[mobile] 导航未成功，中止流程（不在错误的世界位置放货）");
              mobile_ok = false;
            }
          }
        }

        // ---- 阶段 3：目标位置放货（步骤 4~6）----
        if (mobile_ok) {
          RCLCPP_INFO(logger, "[mobile] ===== 阶段 3/3：目标位置放货 =====");
          // 物体在场景里更新到 B。必须在放货前做：否则后面三步仍以为物体在 A。
          publishSceneDiff(
            scene_pub,
            makeBoxObject(object_id, transport_frame, transport_place, transport_size));
          std::this_thread::sleep_for(std::chrono::milliseconds(800));
          for (std::size_t i = 3U; i < steps.size() && mobile_ok; ++i) {
            mobile_ok = run_arm_step(i);
          }
        }

        // ---- 世界位移报告：这才是"真的搬走了"的证据 ----
        if (mobile_ok && have_before) {
          std::array<double, 3> object_map_after{0.0, 0.0, 0.0};
          std::array<double, 3> base_map_after{0.0, 0.0, 0.0};
          std::string tf_why;
          const bool a = bodyPointToMap(
            tf_buffer, mobile_map_frame, transport_frame, transport_place,
            object_map_after, tf_why);
          const bool b = bodyPointToMap(
            tf_buffer, mobile_map_frame, transport_frame, std::vector<double>{0.0, 0.0, 0.0},
            base_map_after, tf_why);
          if (a && b) {
            const double obj_dx = object_map_after[0] - object_map_before[0];
            const double obj_dy = object_map_after[1] - object_map_before[1];
            const double base_dx = base_map_after[0] - base_map_before[0];
            const double base_dy = base_map_after[1] - base_map_before[1];
            RCLCPP_INFO(
              logger,
              "[mobile] 放货后：物体 map=(%.3f, %.3f, %.3f) 底盘 map=(%.3f, %.3f)",
              object_map_after[0], object_map_after[1], object_map_after[2],
              base_map_after[0], base_map_after[1]);
            RCLCPP_INFO(
              logger,
              "[mobile] 世界位移：物体 (%.3f, %.3f) 距离 %.3fm | "
              "底盘 (%.3f, %.3f) 距离 %.3fm",
              obj_dx, obj_dy, std::hypot(obj_dx, obj_dy),
              base_dx, base_dy, std::hypot(base_dx, base_dy));
          } else {
            RCLCPP_WARN(logger, "[mobile] 放货后取不到 map 系位置(%s)", tf_why.c_str());
          }
        }

        // 收尾：把物体从场景里移掉，避免污染后续场景。
        moveit_msgs::msg::CollisionObject mobile_remove;
        mobile_remove.id = object_id;
        mobile_remove.header.frame_id = transport_frame;
        mobile_remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        publishSceneDiff(scene_pub, mobile_remove);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        if (!mobile_ok) {
          exit_code = 1;
        }
        RCLCPP_INFO(
          logger,
          "[mobile_transport][summary] planner=%s 结果=%s 机械臂成功步数=%zu/%zu "
          "导航=%s(%.1fs) 规划总耗时=%.3fs 全程最差σ=%.4f 执行=%s",
          params.planner_id.c_str(), mobile_ok ? "PASS" : "FAIL",
          mobile_steps_done, steps.size(), nav.reason.empty() ? "未执行" : nav.reason.c_str(),
          nav.elapsed_sec, mobile_plan_wall,
          std::isfinite(mobile_worst_sigma) ? mobile_worst_sigma : -1.0,
          execute ? "true" : "false");
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
