// Copyright 2026 Astribot

#ifndef ASTRIBOT_S1_PATH_TRACKING__THREE_PHASE_CONTROLLER_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__THREE_PHASE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cmath>

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

protected:
  virtual bool hasTerminalRefinement() const {return false;}
  void accountPolicyPause(double seconds, const rclcpp::Time & now) {
    phase_started_ = phase_started_ + rclcpp::Duration::from_seconds(seconds);
    last_tick_time_ = now;
  }

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
  ///
  /// wz_now 是**实测**角速度，用于惯性补偿（align_inertia_enabled_=false 时忽略）。
  /// 本函数**不是 const**：它维护滑行闩锁与标定统计，两者都必须跨拍。
  geometry_msgs::msg::TwistStamped rotateOnly(
    double error_rad, double wz_now, const std_msgs::msg::Header & header);

  /// 接近段限速：就地按距终点距离夹住 cmd 的线速度模长。
  ///
  /// 只在 kFollow 且**内层已出指令之后**调用。两个对齐段是纯旋转，
  /// 没有线速度可夹，天然不经过这里。
  void applyApproachCap(geometry_msgs::msg::TwistStamped & cmd, double dist_to_goal_m);

  /// 若 ALIGN_GOAL 段实际拿不到执行机会，告警一次（只在真实取到容差后调用）。
  /// 存在的理由：`align_goal_enabled: true` 会让人以为终点对齐在工作，
  /// 而它取决于 GoalChecker 是否卡朝向 —— 一个看着开着其实关着的开关，
  /// 必须显式说出来，不能只写在 yaml 注释里。
  void warnIfGoalAlignInert(double xy_tol, double yaw_tol);

  /// 若 GoalChecker 的朝向容差**紧到本层物理上到不了**，告警一次。
  void warnIfYawToleranceUnreachable(double yaw_tol);

  /// 一次性说明"终点航向的交付精度是哪个数"，并在两个数**离得太远**时告警。
  void noteDeliveredYawAccuracy(double yaw_tol);

  /// 本层能保证的最好朝向落点 [rad] = align_tolerance + 松手后的余转。
  double bestAchievableYawLanding() const;

  /// 相位超时检查，超时抛异常。
  void checkPhaseTimeout(const rclcpp::Time & now);

  /// 进入某个相位。
  ///
  /// restart_timer=true 表示"这是一个新目标"：即使相位值没变也必须重置
  /// phase_started_。漏掉它会永久锁死导航 —— 上一个目标在 ALIGN_START 被中止时，
  /// 新目标进同一相位会继承旧计时器，第一拍就判超时（实测读到 1108s），
  /// 之后每个目标都瞬间失败且永不恢复。
  void enterPhase(
    Phase p, const rclcpp::Time & now, const char * why, bool restart_timer = false);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("ThreePhaseController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> inner_loader_;
  nav2_core::Controller::Ptr inner_;

  nav_msgs::msg::Path plan_;
  Phase phase_{Phase::kDone};
  rclcpp::Time phase_started_{0, 0, RCL_ROS_TIME};   // 时钟源必须显式给，默认是 SYSTEM_TIME、与 clock_->now() 相减即抛
  double start_heading_{0.0};
  bool start_heading_valid_{false};
  bool start_alignment_engaged_{false};
  /// 上一条路径的终点，用于区分「新目标」与「同一目标的周期性重规划」。
  PlanarPoint last_goal_end_{};
  bool has_last_goal_{false};
  bool warned_tolerance_fallback_{false};
  /// ALIGN_GOAL 段实际不生效的告警只打一次（20Hz 刷屏会把真问题淹掉）。
  bool warned_goal_align_inert_{false};
  /// 朝向容差紧到到不了的告警同样只打一次。
  bool warned_yaw_unreachable_{false};
  /// "交付精度是哪个数"的一次性说明（含两数拉开过大的告警）。
  bool noted_delivered_accuracy_{false};
  /// 上一次 computeVelocityCommands 的时刻，用来区分「同一目标的周期重规划」
  /// 与「同一目标的**新一次** FollowPath 下发」。见 isFreshFollowAttempt 的
  /// 头注释：这两者路径内容几乎一样，只有时间空档能分开。
  /// **必须显式 RCL_ROS_TIME**：默认构造是 SYSTEM_TIME，与 clock_->now() 相减即抛。
  rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};
  bool has_tick_{false};

  std::string inner_plugin_;
  bool align_start_enabled_{true};
  bool align_goal_enabled_{true};
  bool zero_vy_in_follow_{true};
  double align_kp_{1.0};
  double align_max_vel_{0.6};
  double align_floor_vel_{0.05};
  double align_tol_rad_{0.05};
  double start_min_angle_rad_{0.20};  double lookahead_m_{0.5};
  double align_timeout_sec_{15.0};
  double follow_timeout_sec_{0.0};      // 0 = 不限（交给 nav2 的 progress_checker）
  double fallback_xy_tol_{0.18};
  double fallback_yaw_tol_{0.20};
  /// 终点位移小于此值即视为同一目标（默认行为树 1Hz 重规划会反复调 setPlan）。
  double new_goal_epsilon_m_{0.25};
  /// tick 空档超过此值 => 上一个 FollowPath action 已结束，这次 setPlan 是新一次
  /// 尝试。默认 0.5s = 20Hz 下 10 拍：远大于单拍抖动，又远小于 align_timeout(15s)。
  double new_attempt_gap_sec_{0.5};

  /// 总开关。false = 一键回退到接近段不限速（今天的行为）。
  bool approach_enabled_{true};
  /// 收敛区长度 D(m)：离终点 d < D 时线速度模长上限按 d/D 线性收敛。
  /// 治的是 **0.55s 未建模死时间**（实测 vel_track_best_lag_s p50=0.55、增益 0.977），
  /// 终段过冲 ≈ v_接近·τ，而 nav2 MPPI 没有 dead-time 参数、改权重动不了这个乘积。
  double approach_dist_m_{1.50};
  /// 速度下限(m/s)：保证终段还能动（实测底盘 0.02 m/s 即可平动）。
  double approach_v_min_{0.05};

  /// 总开关。false = 一键回退到不补偿（= 改动前的行为，逐字等价）。
  /// 关掉它的同时**必须**把 yaw_goal_tolerance 放回 0.20，否则就是
  /// "容差 0.05 + 未补偿余转最坏 0.033" ⇒ 对齐段转不到 ⇒ 超时 -> abort -> PAUSED。
  bool align_inertia_enabled_{true};
  /// 指令→实际的角速度死时间 [s]。
  double align_coast_lag_{0.03};
  /// 松手后的等效角减速度 [rad/s^2]。
  double align_coast_decel_{7.0};
  /// 判"已停住"的角速度阈值 [rad/s]。**必须小于 floor 速度下的实测角速度**
  /// （floor 0.05 × 跟踪比 0.73~0.92 = 0.037~0.046），否则发着 floor 速度就算
  /// 静止 ⇒ 闩锁提前释放 ⇒ 边界自激。0.02 相对 0.037 有 1.85 倍余量。
  double align_settled_wz_{0.02};

  /// 已因"预测到位"而发零速、正在等滑停。置位期间**只发零速**，不重新判 e_pred
  /// —— 少了这个闩就是单阈值门在边界上以 20Hz 自激（"停→余转过头→反向→再停"）。
  /// 在 enterPhase 里复位：相位一换，下面记的误差就换了物理含义。
  bool align_coasting_{false};
  /// 发零速那一拍的有向误差与实测角速度，以及按模型预测的余转角。
  /// 存在的唯一目的是**标定** lag/decel：滑停后打一条"预测 %.4f / 实测 %.4f"，
  /// 那是这两个常数唯一的实测数据来源（当初的 0.006~0.033 没记 wz）。
  double coast_err_at_stop_{0.0};
  double coast_wz_at_stop_{0.0};
  double coast_predicted_{0.0};

  /// 本目标是否已打过到达误差。**必须有这个闩**：判据在 20Hz 的 tick 里求值，
  /// 不闩住就是每拍一条，几秒钟把关心的那一行冲出屏幕。
  /// 在 setPlan 认定"新目标"时复位（不是在周期重规划那条路径上复位）。
  bool arrival_logged_{false};
  /// 跨目标累计量。用户要的是"统计"，单条读数不够：单个目标的误差落在容差内
  /// 说明不了系统性偏置，n 条的均值/最大值才能。进程内累计，重启即清零。
  int arrival_count_{0};
  double arrival_xy_sum_{0.0};
  double arrival_xy_max_{0.0};
  double arrival_yaw_sum_{0.0};
  double arrival_yaw_max_{0.0};
};

}  // namespace astribot_s1_path_tracking

#endif  // ASTRIBOT_S1_PATH_TRACKING__THREE_PHASE_CONTROLLER_HPP_
