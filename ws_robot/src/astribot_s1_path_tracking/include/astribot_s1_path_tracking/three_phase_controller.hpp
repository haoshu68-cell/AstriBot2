// Copyright 2026 Astribot
//
// 三段式路径跟踪控制器（nav2_core::Controller 插件）。
//
// 相位：ALIGN_START（原地对齐路径起始方向）-> FOLLOW（沿路径行驶）
//       -> ALIGN_GOAL（原地对齐目标姿态，可关闭）-> DONE
//
// 设计取舍，逐条都有原因：
//
// 1) **跟踪段不自己实现**，用 pluginlib 嵌套加载一个内层 nav2 控制器
//    （RPP 或 MPPI）。理由：路径跟踪本身 nav2 已经做得比我们好，本包要加的
//    只是"相位调度 + 原地旋转"。重写跟踪算法等于凭空引入一堆已被解决的问题。
//
// 2) **位置容差不自带**，每拍从 goal_checker->getTolerances() 取。
//    两处各存一份阈值必然漂开，而漂开后的现象是"控制器认为到了、
//    GoalChecker 认为没到"，两边日志都正常。
//
// 3) **每段都有超时**，超时抛 nav2_core::PlannerException 让 action 明确失败。
//    绝不允许静默停在某一段 —— 那会表现为"机器人不动且无任何报错"。
//
// 4) **zero_vy_in_follow** 控制跟踪段是否允许全向横移：
//    true  = 前向为主（差速风格），三段式语义最清晰；
//    false = 保留 X 型全向轮的横移能力。
//    默认 true；旧行为通过配置一键回退（仓库规范：不删旧逻辑）。
//
// 5) **align_goal_enabled=false 用于探索场景**（按需求：探索目标到位后
//    不再原地转朝向）。通过配置两个插件实例 + FollowPath 的 controller_id
//    按场景选择，不在代码里 if 场景。

#ifndef ASTRIBOT_S1_PATH_TRACKING__THREE_PHASE_CONTROLLER_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__THREE_PHASE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "astribot_s1_path_tracking/align_math.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"

namespace astribot_s1_path_tracking
{

class ThreePhaseController : public nav2_core::Controller
{
public:
  ThreePhaseController() = default;
  ~ThreePhaseController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /// 供测试/诊断读取当前相位。
  Phase phase() const {return phase_;}

private:
  /// 读取参数，越界即抛（禁止静默回落到"看起来正常"的默认值）。
  void declareAndLoadParams();

  /// 加载内层控制器。失败即抛 —— 没有内层控制器就没有跟踪能力，
  /// 不能降级成"只会原地转"。
  void loadInnerController(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros);

  /// 从 goal_checker 取位置/朝向容差；取不到则用配置的兜底值并告警一次。
  void resolveTolerances(nav2_core::GoalChecker * goal_checker, double & xy_tol, double & yaw_tol);

  /// 纯旋转指令：只出 wz，vx/vy 严格为 0。
  geometry_msgs::msg::TwistStamped rotateOnly(
    double error_rad, const std_msgs::msg::Header & header) const;

  /// 相位超时检查，超时抛异常。
  void checkPhaseTimeout(const rclcpp::Time & now);

  void enterPhase(Phase p, const rclcpp::Time & now, const char * why);

  // ---- ROS 句柄 ----
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("ThreePhaseController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // ---- 内层控制器 ----
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> inner_loader_;
  nav2_core::Controller::Ptr inner_;

  // ---- 状态 ----
  nav_msgs::msg::Path plan_;
  Phase phase_{Phase::kDone};
  rclcpp::Time phase_started_;
  double start_heading_{0.0};
  bool start_heading_valid_{false};
  bool warned_tolerance_fallback_{false};

  // ---- 参数 ----
  std::string inner_plugin_;
  bool align_start_enabled_{true};
  bool align_goal_enabled_{true};
  bool zero_vy_in_follow_{true};
  double align_kp_{1.0};
  double align_max_vel_{0.6};
  double align_floor_vel_{0.05};
  double align_tol_rad_{0.05};
  double start_min_angle_rad_{0.20};
  double lookahead_m_{0.5};
  double align_timeout_sec_{15.0};
  double follow_timeout_sec_{0.0};      // 0 = 不限（交给 nav2 的 progress_checker）
  double fallback_xy_tol_{0.18};
  double fallback_yaw_tol_{0.20};
};

}  // namespace astribot_s1_path_tracking

#endif  // ASTRIBOT_S1_PATH_TRACKING__THREE_PHASE_CONTROLLER_HPP_
