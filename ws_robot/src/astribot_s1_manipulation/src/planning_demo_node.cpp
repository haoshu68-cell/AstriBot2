// Copyright 2026 Astribot

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
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "astribot_s1_manipulation/dual_arm_planner.hpp"
#include "astribot_s1_manipulation/gripper_commander.hpp"

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

  auto & col = p.collision;
  col.check_self_collision =
    loader.get<bool>("collision.check_self_collision", col.check_self_collision);
  col.check_environment_collision =
    loader.get<bool>("collision.check_environment_collision", col.check_environment_collision);
  col.collect_all_contacts =
    loader.get<bool>("collision.collect_all_contacts", col.collect_all_contacts);
  col.max_contacts = static_cast<std::size_t>(
    loader.get<int64_t>("collision.max_contacts", static_cast<int64_t>(col.max_contacts)));

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
    RCLCPP_INFO(logger, "无轨迹输出，因为当前构型已经满足目标，无需运动");
    return;
  }

  if (!result.succeeded()) {
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

  RCLCPP_INFO(logger, "优化: %s", result.optimization.note.c_str());
  if (result.optimization.optimized_accepted) {
    RCLCPP_INFO(
      logger, "  已采纳 TOTG 优化，节拍缩短 %.1f%%",
      100.0 * result.optimization.duration_reduction_ratio);
  } else if (result.optimization.fell_back) {
    RCLCPP_WARN(logger, "  已回退到 baseline（优化结果不合法或未更快）");
  }

  if (result.worst_residual.valid) {
    RCLCPP_INFO(
      logger, "闭链残差(全轨迹最差): 位置 %.6f m, 姿态 %.6f rad -> %s",
      result.worst_residual.position_error, result.worst_residual.orientation_error,
      result.worst_residual.within_tolerance ? "满足约束" : "超出阈值");
  }

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

/// 把世界里已有的物体挂到某个 link 上（真实抓取）。
///
/// 语义要点（MoveIt 的行为，不是我们自己实现的）：
///   · operation=ADD 且该 id 已存在于 world 时，MoveIt 会把它从 world 移走、
///     挂到 link_name 上，位姿保持不变。不需要我们自己算相对位姿。
///   · 之后物体随该 link 一起动，并且碰撞检测把它当作**机器人的一部分** ——
///     这正是"物体真的被夹住了"在规划层的准确表达。
///
/// touch_links 必须给：夹爪合上时指垫本来就贴着物体，不声明允许接触的话
/// attach 完的瞬间就会报"夹爪与物体碰撞"，后续每一步规划都失败。
moveit_msgs::msg::AttachedCollisionObject makeAttachObject(
  const std::string & id, const std::string & link,
  const std::vector<std::string> & touch_links)
{
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = link;
  attached.object.id = id;
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  attached.touch_links = touch_links;
  return attached;
}

/// 松爪：把物体摘下来放回世界。
///
/// MoveIt 在 REMOVE 一个 attached object 时，会按它**当前实际位姿**重新放进 world。
/// 所以放置点是"手真的把它带到了哪儿"，而不是我们以为的 B —— 这是我们想要的：
/// 若执行有偏差，物体就落在有偏差的地方，日志里能看出来。
moveit_msgs::msg::AttachedCollisionObject makeDetachObject(
  const std::string & id, const std::string & link)
{
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = link;
  attached.object.id = id;
  attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  return attached;
}

/// 末端在某个抓取方向上的几何包络（相对 TCP）。
///
/// setback —— 从 TCP 沿"背离物体"方向走多远还有碰撞几何（正值）。
///            **注意它量的是最远端**，不是"最近的那块几何"，所以不适合当净空判据：
///            TCP 背后是整条手臂，实测 link_7 一项就到 0.21m。
///            保留它只用于诊断输出（看末端几何整体占了多长一段）。
/// reach   —— 从 TCP 沿"朝向物体"方向最远还有碰撞几何（正值，即指尖伸出量）。
///            这个是真判据：物体顶面比它还远，两指就够不到。
struct EndEffectorSpan
{
  double setback{0.0};
  double reach{0.0};
  bool valid{false};
};

/// 量出末端（腕部 + 夹爪）相对 TCP 的几何包络，投影到给定的抓取方向上。
///
/// !!! 这个函数是替换掉 tcpCoincidentCollisionRadius 的，原因必须写清楚 !!!
/// 原函数的算法是"从 TCP 沿父链上溯，只要关节原点没有平移就继续，取最大外接半径"。
/// 它成立的前提是 **TCP 与腕部连杆原点重合**（当时 TCP = tool_link，而
/// tool_joint 的 xyz 是 0 0 0，确实与 link_7 重合）。
/// 2026-08-20 把 TCP 改到 tcp_link（法兰再往 -y 0.15m）之后，这个前提没了：
/// 上溯第一步 tcp_joint 的平移是 0.15，循环立刻终止，函数**恒返回 0**。
/// 于是 min_grasp_z_offset 退化成 0.5*size_z + margin，那道"跑之前就拒绝"的
/// 检查变成永远通过 —— 它不再度量任何东西，却还在报告一个看起来合理的下限。
/// 这是我把 TCP 挪走时引入的**静默失效**，不是原函数写错了。
///
/// 新算法直接量真实包络，不再依赖"原点重合"这个巧合：
///   1. 取 TCP 与末端各连杆在同一个 RobotState 下的相对变换。
///      这个相对关系与手臂 7 个关节无关 —— TCP 和夹爪都刚性挂在 link_7 上，
///      只随夹爪开合角变化。所以随便取一个状态量都对，不需要先解 IK。
///   2. 把每个连杆碰撞体的 AABB 8 个角点变换到 TCP 系。
///   3. 投影到 grasp_dir_in_tcp（"朝向物体"的单位方向，在 TCP 系下表达），
///      取 min/max 得到 setback / reach。
///
/// 为什么要投影而不是只看某个轴：抓取姿态由 yaml 里的四元数给定，夹爪的接近轴
/// （TCP 系的 -y）与世界竖直方向**一般不重合**。实测当前配置差 28.6°，
/// 直接拿 -y 方向的伸出量当竖直伸出量会高估 14%。
EndEffectorSpan endEffectorSpan(
  const moveit::core::RobotModelConstPtr & model, const std::string & tcp_link,
  const Eigen::Vector3d & grasp_dir_in_tcp, const std::vector<std::string> & links,
  std::string & detail)
{
  detail.clear();
  EndEffectorSpan span;
  if (!model || grasp_dir_in_tcp.norm() < 1.0e-9) {
    detail = "模型为空或抓取方向为零向量";
    return span;
  }
  const moveit::core::LinkModel * tcp = model->getLinkModel(tcp_link);
  if (tcp == nullptr) {
    detail = "找不到 TCP link '" + tcp_link + "'";
    return span;
  }
  const Eigen::Vector3d dir = grasp_dir_in_tcp.normalized();

  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.update();
  const Eigen::Isometry3d tcp_to_world = state.getGlobalLinkTransform(tcp).inverse();

  double lo = std::numeric_limits<double>::max();
  double hi = std::numeric_limits<double>::lowest();
  for (const std::string & name : links) {
    const moveit::core::LinkModel * link = model->getLinkModel(name);
    if (link == nullptr || link->getShapes().empty()) {
      continue;
    }
    const Eigen::Isometry3d link_in_tcp = tcp_to_world * state.getGlobalLinkTransform(link);
    const Eigen::Vector3d half = 0.5 * link->getShapeExtentsAtOrigin();
    const Eigen::Vector3d center = link->getCenteredBoundingBoxOffset();
    double link_lo = std::numeric_limits<double>::max();
    double link_hi = std::numeric_limits<double>::lowest();
    for (int sx = -1; sx <= 1; sx += 2) {
      for (int sy = -1; sy <= 1; sy += 2) {
        for (int sz = -1; sz <= 1; sz += 2) {
          const Eigen::Vector3d corner_local =
            center + Eigen::Vector3d(sx * half.x(), sy * half.y(), sz * half.z());
          const double proj = dir.dot(link_in_tcp * corner_local);
          link_lo = std::min(link_lo, proj);
          link_hi = std::max(link_hi, proj);
        }
      }
    }
    lo = std::min(lo, link_lo);
    hi = std::max(hi, link_hi);
    detail += (detail.empty() ? "" : ", ") + link->getName() + "=[" +
      std::to_string(link_lo) + "," + std::to_string(link_hi) + "]";
  }
  if (hi < lo) {
    detail = "末端连杆列表里没有任何带碰撞几何的连杆";
    return span;
  }
  span.setback = (lo < 0.0) ? -lo : 0.0;
  span.reach = (hi > 0.0) ? hi : 0.0;
  span.valid = true;
  return span;
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

/// attach / detach 同样走场景差分话题，理由与上面完全一致。
///
/// 注意 attached_collision_objects 挂在 robot_state 下面，不在 world 下面 ——
/// 挂上之后它就是机器人状态的一部分了。robot_state.is_diff 必须是 true，
/// 否则这条消息会被当成一个完整的机器人状态，把当前关节值也一并覆盖掉。
void publishAttachDiff(
  const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr & pub,
  const moveit_msgs::msg::AttachedCollisionObject & attached)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  scene.robot_state.attached_collision_objects.push_back(attached);
  pub->publish(scene);
}

/// 一个轴对齐长方体沿任意方向的支撑宽度。
///
/// 为什么要算这个而不是直接取 size_xyz[0]：夹爪张合方向由抓取姿态四元数决定，
/// 直接取 x 尺寸只在"张合轴刚好映射到本体系 x"时才对。姿态一改就静默错，
/// 而错了的表现是夹爪合到错误角度 —— 不会报任何错。
double boxWidthAlongDirection(
  const std::vector<double> & size_xyz, const Eigen::Vector3d & dir_in_object_frame)
{
  const Eigen::Vector3d d = dir_in_object_frame.cwiseAbs();
  return d.x() * size_xyz[0] + d.y() * size_xyz[1] + d.z() * size_xyz[2];
}

/// 在取货点合爪并把物体挂到 TCP 上。
///
/// execute=false（纯规划模式）时不下发夹爪动作 —— 机器人本来就没动，
/// 驱动真实夹爪没有意义。但 attach **照做**：接下来三步的规划必须知道
/// 手上带着东西，否则规划出的轨迹在真跑时会拿物体去撞环境。
/// 这种情况下 attach 是从"没动过的当前位姿"发生的，日志里必须说清楚，
/// 不能让人以为干跑的搬运段几何是可信的。
bool graspAndAttach(
  const rclcpp::Logger & logger,
  GripperCommander & gripper,
  const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr & scene_pub,
  const std::string & object_id, const std::string & tcp_link,
  const std::vector<std::string> & touch_links,
  double grasp_width, bool execute, const char * tag)
{
  if (execute) {
    const GripperOutcome closing = gripper.closeToWidth(grasp_width);
    RCLCPP_INFO(
      logger,
      "%s 合爪: 物体宽=%.4fm -> 目标角=%.4frad(理论张口 %.4fm) 实测角=%.4frad "
      "耗时 %.2fs 结果=%s%s%s",
      tag, grasp_width, closing.target_rad, closing.target_width_m, closing.measured_rad,
      closing.elapsed_sec, toString(closing.code),
      closing.detail.empty() ? "" : " | ", closing.detail.c_str());
    if (!closing.ok()) {
      RCLCPP_ERROR(
        logger, "%s 合爪失败(%s)，不做 attach —— 否则会得到\"物体跟着走但其实没夹住\"的假成功",
        tag, toString(closing.code));
      return false;
    }
  } else {
    RCLCPP_WARN(
      logger,
      "%s execute=false：不下发夹爪动作（机器人没动，驱动夹爪没意义）。"
      "但仍然把物体 attach 到 %s，让后续规划知道手上带着东西 —— "
      "注意此时 attach 是从未移动的当前位姿发生的，搬运段的几何不可当真",
      tag, tcp_link.c_str());
  }

  publishAttachDiff(scene_pub, makeAttachObject(object_id, tcp_link, touch_links));
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  RCLCPP_INFO(
    logger, "%s 物体已 attach 到 %s（碰撞检测起把它当机器人的一部分），touch_links %zu 个",
    tag, tcp_link.c_str(), touch_links.size());
  return true;
}

/// 在放置点松爪并把物体摘回世界。
bool releaseAndDetach(
  const rclcpp::Logger & logger,
  GripperCommander & gripper,
  const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr & scene_pub,
  const std::string & object_id, const std::string & tcp_link,
  bool execute, const char * tag)
{
  bool ok = true;
  if (execute) {
    const GripperOutcome opening = gripper.open();
    RCLCPP_INFO(
      logger, "%s 松爪: 目标角=%.4frad(理论张口 %.4fm) 实测角=%.4frad 耗时 %.2fs 结果=%s%s%s",
      tag, opening.target_rad, opening.target_width_m, opening.measured_rad,
      opening.elapsed_sec, toString(opening.code),
      opening.detail.empty() ? "" : " | ", opening.detail.c_str());
    if (!opening.ok()) {
      RCLCPP_ERROR(logger, "%s 松爪失败(%s)，仍执行 detach 以免污染场景", tag,
        toString(opening.code));
      ok = false;
    }
  } else {
    RCLCPP_WARN(logger, "%s execute=false：不下发夹爪动作，仅在场景里 detach", tag);
  }

  publishAttachDiff(scene_pub, makeDetachObject(object_id, tcp_link));
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  RCLCPP_INFO(
    logger, "%s 物体已 detach，MoveIt 按其**当前实际位姿**放回世界", tag);
  return ok;
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
/// 单次 IK 尝试。由 probeTcpPose 重试调用，不要直接用（单次结果是随机的）。
ProbeVerdict probeTcpPoseOnce(
  const moveit::core::RobotState & seed,
  const moveit::core::JointModelGroup * jmg,
  const std::string & tcp_link,
  const geometry_msgs::msg::PoseStamped & pose,
  double ik_timeout,
  const CollisionValidator & collision,
  const SingularityMonitor & singularity);

ProbeVerdict probeTcpPose(
  const moveit::core::RobotState & seed,
  const moveit::core::JointModelGroup * jmg,
  const std::string & tcp_link,
  const geometry_msgs::msg::PoseStamped & pose,
  double ik_timeout,
  int ik_attempts,
  const CollisionValidator & collision,
  const SingularityMonitor & singularity)
{
  ProbeVerdict best;
  bool have_any = false;
  const int attempts = std::max(1, ik_attempts);
  for (int attempt = 0; attempt < attempts; ++attempt) {
    ProbeVerdict trial = probeTcpPoseOnce(
      seed, jmg, tcp_link, pose, ik_timeout, collision, singularity);
    if (trial.good()) {
      return trial;
    }
    if (!have_any || (trial.ik_ok && !best.ik_ok) ||
      (trial.ik_ok == best.ik_ok && trial.sigma_min > best.sigma_min))
    {
      best = trial;
      have_any = true;
    }
  }
  return best;
}

ProbeVerdict probeTcpPoseOnce(
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
      RCLCPP_ERROR(logger, "规划器初始化失败，节点退出: %s", error.c_str());
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
      rclcpp::shutdown();
      return 1;
    }

    ParameterLoader loader(node);
    const std::vector<std::string> end_effector_links =
      loader.get<std::vector<std::string>>(
      "demo.end_effector_links", std::vector<std::string>{});
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
    const std::vector<double> probe_x_list = loader.get<std::vector<double>>(
      "demo.transport_probe_x_list", std::vector<double>{});
    const std::vector<double> probe_y_list = loader.get<std::vector<double>>(
      "demo.transport_probe_y_list", std::vector<double>{});
    const std::vector<double> probe_z_list = loader.get<std::vector<double>>(
      "demo.transport_probe_z_list", std::vector<double>{});
    const double probe_ik_timeout =
      loader.get<double>("demo.transport_probe_ik_timeout", 0.05);
    const int probe_ik_attempts = static_cast<int>(
      loader.get<int64_t>("demo.transport_probe_ik_attempts", 20));
    GripperConfig gripper_config;
    gripper_config.group_name =
      loader.get<std::string>("demo.gripper_group", std::string("gripper_left"));
    gripper_config.action_name = loader.get<std::string>(
      "demo.gripper_action", std::string("/gripper_left_controller/follow_joint_trajectory"));
    gripper_config.open_state_name =
      loader.get<std::string>("demo.gripper_open_state", std::string("open"));
    gripper_config.closed_state_name =
      loader.get<std::string>("demo.gripper_closed_state", std::string("closed"));
    gripper_config.left_pad_link =
      loader.get<std::string>("demo.gripper_left_pad_link", std::string(""));
    gripper_config.right_pad_link =
      loader.get<std::string>("demo.gripper_right_pad_link", std::string(""));
    gripper_config.jaw_axis_in_tcp = loader.get<std::vector<double>>(
      "demo.gripper_jaw_axis_in_tcp", std::vector<double>{1.0, 0.0, 0.0});
    gripper_config.move_time_sec = loader.get<double>("demo.gripper_move_time", 1.2);
    gripper_config.settle_time_sec = loader.get<double>("demo.gripper_settle_time", 0.8);
    gripper_config.converge_tolerance_rad =
      loader.get<double>("demo.gripper_converge_tolerance", 0.02);
    gripper_config.grasp_preload_m = loader.get<double>("demo.gripper_grasp_preload", 0.004);
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
    const std::string mobile_carry_pose =
      loader.get<std::string>("demo.mobile_transport_carry_pose", std::string("ready"));
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

    GripperCommander gripper(node);
    auto configure_gripper =
      [&](const std::string & tcp_link, const char * tag) -> bool {
        GripperConfig cfg = gripper_config;
        cfg.tcp_link = tcp_link;
        std::string detail;
        const PlanErrorCode code = gripper.configure(planner.getRobotModel(), cfg, detail);
        if (code != PlanErrorCode::kSuccess) {
          RCLCPP_ERROR(
            logger, "%s 夹爪配置失败(%s): %s", tag, toString(code), detail.c_str());
          return false;
        }
        RCLCPP_INFO(
          logger,
          "%s 夹爪就绪: 组=%s 主动关节=%s 张开角=%.4frad(张口 %.4fm) "
          "闭合角=%.4frad(张口 %.4fm) 预紧=%.4fm action=%s",
          tag, cfg.group_name.c_str(), gripper.jointName().c_str(),
          gripper.openAngle(), gripper.jawWidthAtAngle(gripper.openAngle()),
          gripper.closedAngle(), gripper.jawWidthAtAngle(gripper.closedAngle()),
          cfg.grasp_preload_m, cfg.action_name.c_str());
        return true;
      };

    for (const std::string & scenario : scenarios) {
      if (scenario == "transport_probe") {
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
          "物体尺寸=(%.3f,%.3f,%.3f) grasp_z_offset=%.3f approach=%.3f；"
          "IK 预算 %.3fs × %d 次(取任一成功)",
          transport_group.c_str(), probe_tcp_link.c_str(),
          params.singularity.min_singular_value, params.singularity.max_condition_number,
          transport_size[0], transport_size[1], transport_size[2],
          transport_grasp_z_offset, transport_approach_height,
          probe_ik_timeout, probe_ik_attempts);
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
                probe_ik_timeout, probe_ik_attempts, probe_collision, probe_singularity);
              const ProbeVerdict approach = probeTcpPose(
                *seed, jmg, probe_tcp_link,
                makeTransportPose(
                  transport_frame, px, py, pz + transport_approach_height, transport_quat),
                probe_ik_timeout, probe_ik_attempts, probe_collision, probe_singularity);

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

        const std::string resolved_tcp_link = transport_tcp_link.empty() ?
          (transport_group == params.closed_chain.leader_group ?
          params.closed_chain.leader_tcp_link :
          params.closed_chain.follower_tcp_link) :
          transport_tcp_link;
        std::string span_detail;
        const Eigen::Quaterniond grasp_q(
          transport_quat[3], transport_quat[0], transport_quat[1], transport_quat[2]);
        const Eigen::Vector3d grasp_dir_in_tcp =
          grasp_q.normalized().conjugate() * Eigen::Vector3d(0.0, 0.0, -1.0);
        const EndEffectorSpan span = endEffectorSpan(
          planner.getRobotModel(), resolved_tcp_link, grasp_dir_in_tcp,
          end_effector_links, span_detail);
        if (!span.valid) {
          RCLCPP_ERROR(
            logger, "跳过 transport: 量不出末端包络（%s）。"
            "demo.end_effector_links 是否配对了？", span_detail.c_str());
          exit_code = 1;
          continue;
        }
        const double max_grasp_z_offset = 0.5 * transport_size[2] + span.reach;
        RCLCPP_INFO(
          logger,
          "[transport] TCP=%s 抓取方向(TCP系)=[%.3f %.3f %.3f]（与竖直方向夹角 %.1f°）；"
          "指尖沿该方向伸出 reach=%.4fm；物体半高=%.4fm "
          "=> grasp_z_offset 上限=%.4fm，当前=%.4fm。"
          "下限由碰撞校验逐点把关，不在这里算。末端各连杆投影区间: %s",
          resolved_tcp_link.c_str(), grasp_dir_in_tcp.x(), grasp_dir_in_tcp.y(),
          grasp_dir_in_tcp.z(),
          std::acos(std::min(1.0, std::max(-1.0, grasp_dir_in_tcp.normalized().dot(
              Eigen::Vector3d(0.0, -1.0, 0.0))))) * 180.0 / M_PI,
          span.reach, 0.5 * transport_size[2], max_grasp_z_offset,
          transport_grasp_z_offset, span_detail.c_str());
        if (transport_grasp_z_offset > max_grasp_z_offset) {
          RCLCPP_ERROR(
            logger,
            "跳过 transport: grasp_z_offset=%.4f 高于上限 %.4f —— "
            "夹爪指尖沿抓取方向只伸出 %.4fm，而物体顶面在 TCP 前方 %.4fm 处，"
            "两指够不到物体：规划和执行都会\"成功\"，但夹爪是在物体上方闭合到空气里。"
            "请降到 %.4f 以下（越接近 %.4f 越是只夹到物体顶边，取窗口中段更稳）。",
            transport_grasp_z_offset, max_grasp_z_offset, span.reach,
            transport_grasp_z_offset - 0.5 * transport_size[2],
            max_grasp_z_offset, max_grasp_z_offset);
          exit_code = 1;
          continue;
        }

        auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>(
          "/planning_scene", rclcpp::QoS(1).transient_local());

        if (!configure_gripper(resolved_tcp_link, "[transport]")) {
          exit_code = 1;
          continue;
        }
        const Eigen::Vector3d jaw_axis_tcp(
          gripper_config.jaw_axis_in_tcp.size() == 3U ?
          gripper_config.jaw_axis_in_tcp[0] : 1.0,
          gripper_config.jaw_axis_in_tcp.size() == 3U ?
          gripper_config.jaw_axis_in_tcp[1] : 0.0,
          gripper_config.jaw_axis_in_tcp.size() == 3U ?
          gripper_config.jaw_axis_in_tcp[2] : 0.0);
        const Eigen::Vector3d jaw_axis_in_frame = grasp_q.normalized() * jaw_axis_tcp;
        const double grasp_width = boxWidthAlongDirection(transport_size, jaw_axis_in_frame);
        {
          double probe_angle = 0.0;
          std::string why;
          const PlanErrorCode width_code =
            gripper.graspAngleForWidth(grasp_width, probe_angle, why);
          if (width_code != PlanErrorCode::kSuccess) {
            RCLCPP_ERROR(
              logger, "跳过 transport: 张合方向上物体宽 %.4fm 夹不了(%s): %s",
              grasp_width, toString(width_code), why.c_str());
            exit_code = 1;
            continue;
          }
          RCLCPP_INFO(
            logger, "[transport] 张合方向 frame=(%.3f, %.3f, %.3f) 上物体宽 %.4fm "
            "-> 抓取角 %.4frad",
            jaw_axis_in_frame.x(), jaw_axis_in_frame.y(), jaw_axis_in_frame.z(),
            grasp_width, probe_angle);
        }

        publishSceneDiff(
          scene_pub, makeBoxObject(object_id, transport_frame, transport_pick, transport_size));
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        RCLCPP_INFO(
          logger,
          "[transport] 物体已加入规划场景: A=(%.3f, %.3f, %.3f) 尺寸=(%.3f, %.3f, %.3f) "
          "frame=%s；目标位置 B=(%.3f, %.3f, %.3f)",
          transport_pick[0], transport_pick[1], transport_pick[2],
          transport_size[0], transport_size[1], transport_size[2],
          transport_frame.c_str(),
          transport_place[0], transport_place[1], transport_place[2]);
        RCLCPP_WARN(
          logger,
          "[transport] 抓取语义边界：夹爪**真的**按指令开合，物体**真的**被 attach 到 TCP "
          "并参与碰撞检测（规划器必须带着它绕障）；但 Gazebo 里没有这个物体的刚体，"
          "不产生夹持力 —— 物理夹持受 mimic 从动关节 7.79° 稳态误差与未标定摩擦影响，"
          "是独立课题");

        if (execute) {
          const GripperOutcome pre_open = gripper.open();
          RCLCPP_INFO(
            logger, "[transport] 预张开: 目标角=%.4frad 实测角=%.4frad 结果=%s%s%s",
            pre_open.target_rad, pre_open.measured_rad, toString(pre_open.code),
            pre_open.detail.empty() ? "" : " | ", pre_open.detail.c_str());
          if (!pre_open.ok()) {
            RCLCPP_ERROR(
              logger, "[transport] 预张开失败(%s)，中止：合着爪去抓必然撞物体",
              toString(pre_open.code));
            exit_code = 1;
            continue;
          }
        }

        const std::vector<TransportStep> steps = buildTransportSteps(
          transport_frame, transport_pick, transport_place,
          transport_grasp_z_offset, transport_approach_height, transport_quat);

        bool all_ok = true;
        bool object_attached = false;
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

          if (step.name == "2-下降到A") {
            if (!graspAndAttach(
                logger, gripper, scene_pub, object_id, resolved_tcp_link,
                end_effector_links, grasp_width, execute, "[transport]"))
            {
              all_ok = false;
              exit_code = 1;
              break;
            }
            object_attached = true;
          } else if (step.name == "5-下降到B") {
            const bool released = releaseAndDetach(
              logger, gripper, scene_pub, object_id, resolved_tcp_link,
              execute, "[transport]");
            object_attached = false;
            if (!released) {
              all_ok = false;
              exit_code = 1;
              break;
            }
          }
        }

        if (object_attached) {
          publishAttachDiff(scene_pub, makeDetachObject(object_id, resolved_tcp_link));
          std::this_thread::sleep_for(std::chrono::milliseconds(300));
          RCLCPP_INFO(logger, "[transport] 收尾：物体仍挂在手上，已补一次 detach");
        }
        moveit_msgs::msg::CollisionObject remove;
        remove.id = object_id;
        remove.header.frame_id = transport_frame;
        remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        publishSceneDiff(scene_pub, remove);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        RCLCPP_INFO(
          logger, "[transport] 搬运序列%s，物体已从规划场景移除",
          all_ok ? "全部完成" : "中止");
        RCLCPP_INFO(
          logger,
          "[transport][summary] planner=%s 结果=%s 成功步数=%zu/%zu "
          "规划总耗时=%.3fs 轨迹总节拍=%.3fs 总尝试=%d 全程最差σ=%.4f 执行=%s",
          params.planner_id.c_str(), all_ok ? "PASS" : "FAIL",
          steps_done, steps.size(), total_plan_wall, total_traj_duration, total_attempts,
          std::isfinite(worst_sigma_all) ? worst_sigma_all : -1.0,
          execute ? "true" : "false");
      } else if (scenario == "mobile_transport") {
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

        const std::string mobile_tcp_link = transport_tcp_link.empty() ?
          (transport_group == params.closed_chain.leader_group ?
          params.closed_chain.leader_tcp_link :
          params.closed_chain.follower_tcp_link) :
          transport_tcp_link;
        std::string mobile_span_detail;
        const Eigen::Quaterniond mobile_grasp_q(
          transport_quat[3], transport_quat[0], transport_quat[1], transport_quat[2]);
        const Eigen::Vector3d mobile_grasp_dir =
          mobile_grasp_q.normalized().conjugate() * Eigen::Vector3d(0.0, 0.0, -1.0);
        const EndEffectorSpan mobile_span = endEffectorSpan(
          planner.getRobotModel(), mobile_tcp_link, mobile_grasp_dir,
          end_effector_links, mobile_span_detail);
        if (!mobile_span.valid) {
          RCLCPP_ERROR(
            logger, "跳过 mobile_transport: 量不出末端包络（%s）",
            mobile_span_detail.c_str());
          exit_code = 1;
          continue;
        }
        const double mobile_max_offset = 0.5 * transport_size[2] + mobile_span.reach;
        if (transport_grasp_z_offset > mobile_max_offset) {
          RCLCPP_ERROR(
            logger,
            "跳过 mobile_transport: transport_grasp_z_offset=%.4f 高于上限 %.4f"
            "（夹爪指尖沿抓取方向只伸出 %.4f），两指够不到物体 —— 会静默空抓。"
            "下限由碰撞校验逐点把关，同 transport 场景。",
            transport_grasp_z_offset, mobile_max_offset, mobile_span.reach);
          exit_code = 1;
          continue;
        }

        tf2_ros::Buffer tf_buffer(node->get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer, node);

        const std::string object_id = "transport_target";
        auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>(
          "/planning_scene", rclcpp::QoS(1).transient_local());

        if (!configure_gripper(mobile_tcp_link, "[mobile]")) {
          exit_code = 1;
          continue;
        }
        const Eigen::Vector3d mobile_jaw_axis_tcp(
          gripper_config.jaw_axis_in_tcp.size() == 3U ?
          gripper_config.jaw_axis_in_tcp[0] : 1.0,
          gripper_config.jaw_axis_in_tcp.size() == 3U ?
          gripper_config.jaw_axis_in_tcp[1] : 0.0,
          gripper_config.jaw_axis_in_tcp.size() == 3U ?
          gripper_config.jaw_axis_in_tcp[2] : 0.0);
        const double mobile_grasp_width = boxWidthAlongDirection(
          transport_size, mobile_grasp_q.normalized() * mobile_jaw_axis_tcp);
        {
          double probe_angle = 0.0;
          std::string why;
          const PlanErrorCode width_code =
            gripper.graspAngleForWidth(mobile_grasp_width, probe_angle, why);
          if (width_code != PlanErrorCode::kSuccess) {
            RCLCPP_ERROR(
              logger, "跳过 mobile_transport: 张合方向上物体宽 %.4fm 夹不了(%s): %s",
              mobile_grasp_width, toString(width_code), why.c_str());
            exit_code = 1;
            continue;
          }
        }

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
          "[mobile] 抓取语义边界：夹爪真的开合、物体真的 attach 到 TCP 并参与碰撞检测；"
          "但 Gazebo 里没有该物体的刚体，不产生夹持力。"
          "另外导航途中伸出的手臂与手上的物体都超出了代价地图那个 0.42m 外接足迹，"
          "nav2 看不到它们 —— 这一条没有因为改用 attach 而改变");

        if (execute) {
          const GripperOutcome pre_open = gripper.open();
          RCLCPP_INFO(
            logger, "[mobile] 预张开: 目标角=%.4frad 实测角=%.4frad 结果=%s%s%s",
            pre_open.target_rad, pre_open.measured_rad, toString(pre_open.code),
            pre_open.detail.empty() ? "" : " | ", pre_open.detail.c_str());
          if (!pre_open.ok()) {
            RCLCPP_ERROR(
              logger, "[mobile] 预张开失败(%s)，中止", toString(pre_open.code));
            exit_code = 1;
            continue;
          }
        }

        const std::vector<TransportStep> steps = buildTransportSteps(
          transport_frame, transport_pick, transport_place,
          transport_grasp_z_offset, transport_approach_height, transport_quat);

        double mobile_plan_wall = 0.0;
        double mobile_worst_sigma = std::numeric_limits<double>::infinity();
        std::size_t mobile_steps_done = 0U;
        bool mobile_ok = true;
        bool mobile_object_attached = false;

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

        RCLCPP_INFO(logger, "[mobile] ===== 阶段 1/3：起始位置取货 =====");
        for (std::size_t i = 0; i < 3U && mobile_ok; ++i) {
          mobile_ok = run_arm_step(i);
          if (mobile_ok && i == 1U) {
            mobile_ok = graspAndAttach(
              logger, gripper, scene_pub, object_id, mobile_tcp_link,
              end_effector_links, mobile_grasp_width, execute, "[mobile]");
            mobile_object_attached = mobile_ok;
          }
        }

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

        NavOutcome nav;
        if (mobile_ok) {
          RCLCPP_INFO(logger, "[mobile] ===== 阶段 2/3：底盘导航 =====");

          if (!mobile_carry_pose.empty()) {
            SingleArmPlanRequest carry_request;
            carry_request.group = transport_group;
            carry_request.named_target = mobile_carry_pose;
            const PlanResult carry = planner.planSingleArm(carry_request);
            RCLCPP_INFO(
              logger, "[mobile] 导航前收臂到 '%s': %s | 节拍 %.3fs 最差σ %.4f",
              mobile_carry_pose.c_str(), toString(carry.code),
              carry.final_metrics.duration,
              carry.worst_singularity.valid ?
              carry.worst_singularity.min_singular_value : -1.0);
            if (!carry.succeeded() && !carry.noActionNeeded()) {
              RCLCPP_ERROR(
                logger,
                "[mobile] 收臂失败(%s)，中止 —— 伸着手臂导航实测必然 ABORTED。"
                "若是手上物体与本体碰撞导致规划失败，说明这个搬运姿态不可行，"
                "要换 carry_pose 或换抓取点",
                toString(carry.code));
              mobile_ok = false;
            } else if (carry.succeeded() && execute) {
              std::string carry_message;
              const PlanErrorCode carry_exec = planner.executeTrajectory(
                transport_group, carry.trajectory, carry_message);
              if (carry_exec != PlanErrorCode::kSuccess) {
                RCLCPP_ERROR(
                  logger, "[mobile] 收臂执行失败: %s (%s)",
                  toString(carry_exec), carry_message.c_str());
                mobile_ok = false;
              }
            }
          } else {
            RCLCPP_WARN(
              logger,
              "[mobile] carry_pose 为空：导航途中不收臂。实测这样会因臂-底盘耦合"
              "限速到 0.15 而走不满进度检查器阈值，导航大概率 ABORTED");
          }
        }
        if (mobile_ok) {
          const geometry_msgs::msg::PoseStamped nav_goal = makeNavGoal(
            mobile_map_frame, mobile_nav_goal_xy[0], mobile_nav_goal_xy[1],
            mobile_nav_goal_yaw);

          if (mobile_verify_reachable) {
            std::string why;
            if (!verifyGoalReachable(node, mobile_plan_action, nav_goal, 30.0, why)) {
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
              RCLCPP_ERROR(
                logger, "[mobile] 导航未成功，中止流程（不在错误的世界位置放货）");
              mobile_ok = false;
            }
          }
        }

        if (mobile_ok) {
          RCLCPP_INFO(logger, "[mobile] ===== 阶段 3/3：目标位置放货 =====");
          for (std::size_t i = 3U; i < steps.size() && mobile_ok; ++i) {
            mobile_ok = run_arm_step(i);
            if (mobile_ok && i == 4U) {
              mobile_ok = releaseAndDetach(
                logger, gripper, scene_pub, object_id, mobile_tcp_link, execute, "[mobile]");
              mobile_object_attached = false;
            }
          }
        }

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

        if (mobile_object_attached) {
          publishAttachDiff(scene_pub, makeDetachObject(object_id, mobile_tcp_link));
          std::this_thread::sleep_for(std::chrono::milliseconds(300));
          RCLCPP_INFO(logger, "[mobile] 收尾：物体仍挂在手上，已补一次 detach");
        }
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
        if (!result.succeeded() && !result.noActionNeeded()) {
          exit_code = 1;
        }
      } else if (scenario == "planner_comparison") {
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
