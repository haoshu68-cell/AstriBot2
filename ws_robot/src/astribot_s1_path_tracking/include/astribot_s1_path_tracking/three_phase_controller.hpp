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
#include "astribot_s1_path_tracking/narrow_math.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
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

  /// 供诊断读取窄通道是否已接管。
  bool narrowEngaged() const {return narrow_engaged_;}

private:
  /// 一拍窄通道判定的结果。take_over=true 时 cmd 就是本拍要下发的指令，
  /// 内层控制器**这一拍不调用**。
  struct NarrowDecision
  {
    bool take_over{false};
    geometry_msgs::msg::TwistStamped cmd;
  };

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

  /// 进入某个相位。
  ///
  /// restart_timer=true 表示"这是一个新目标"：即使相位值没变也必须重置
  /// phase_started_。漏掉它会永久锁死导航 —— 上一个目标在 ALIGN_START 被中止时，
  /// 新目标进同一相位会继承旧计时器，第一拍就判超时（实测读到 1108s），
  /// 之后每个目标都瞬间失败且永不恢复。
  void enterPhase(
    Phase p, const rclcpp::Time & now, const char * why, bool restart_timer = false);

  // ================= 窄通道贴边通行（状态 3） =================
  //
  // 「窄通道」按用户口径定义：**局部地图代价值全覆盖足迹的通道**，
  // 即足迹最大代价 >= 253。本图实测该档占可行域 34.68%，其中
  //   < 0.388m（内切直径 0.776 除以 2）31.26% —— 物理放不进去，不是本层职责；
  //   0.388 ~ 0.42m         3.42%  —— **只有朝向有利时才过得去，本层目标域**；
  //   >= 0.42m             65.32%  —— MPPI 自己能过。
  //
  // 为什么用「接管」而不是「临时改 MPPI 权重」：
  //   1) 用户红线要求「脱困结束后所有改过的权重必须复原」。接管天然满足 ——
  //      一个参数都没改，退出即恢复，不存在复原漏项。
  //   2) 实测这一档里足迹代价恒为 253、梯度为 0（占可行域 35%）。
  //      调权重改变的是一个**零梯度量**的系数，乘多少都还是零梯度。
  //      2026-08 已实测 consider_footprint:false 无收益并回退。
  //
  // 退出后不需要恢复任何东西，这是本设计最重要的安全性质。

  /// 把 plan_ 的点变换到局部代价地图的 global frame。
  ///
  /// 必须变换：plan_ 来自 ComputePathToPose，是 **map** 系；而
  /// computeVelocityCommands 收到的 pose 与局部代价地图都是 **odom** 系
  /// （见 nav2_params 里 local_costmap.global_frame: odom）。
  /// 不变换就是拿 map 系的路径去查 odom 系的代价，SLAM 一做回环修正就错开。
  ///
  /// @return false = 变换拿不到。此时**不接管**（缺数据不等于安全）。
  bool planInCostmapFrame(std::vector<PlanarPoint> & out) const;

  /// 读机器人中心格代价与足迹最大代价。
  ///
  /// yaw 显式传入而不是从 pose 里取：红线判据要查的是「最有利朝向下」的
  /// 足迹代价，那个朝向不等于当前朝向。
  /// @return false = 代价地图不可用或机器人出图。
  bool readCosts(
    const geometry_msgs::msg::PoseStamped & pose,
    double yaw,
    double & center_cost, double & footprint_cost);

  /// 窄通道一拍判定 + 指令生成。红线违反时抛异常，绝不静默继续。
  NarrowDecision evaluateNarrow(
    const geometry_msgs::msg::PoseStamped & pose, const rclcpp::Time & now);

  /// 退出窄通道接管，复位所有计数器。
  void disengageNarrow(const char * why);

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
  /// 上一条路径的终点，用于区分「新目标」与「同一目标的周期性重规划」。
  PlanarPoint last_goal_end_{};
  bool has_last_goal_{false};
  bool warned_tolerance_fallback_{false};

  // ---- 窄通道接管状态 ----
  /// 足迹碰致命带的连续拍数（进入用）。
  int narrow_hits_{0};
  /// 足迹已脱离致命带的连续拍数（退出用）。
  int narrow_clear_hits_{0};
  bool narrow_engaged_{false};
  rclcpp::Time narrow_started_;
  /// 本次 setPlan 以来**连续**几次贴边都没穿过去。
  ///
  /// 语义是「连续失败次数」，不是「贴过几次边」——穿成功一次就清零。
  /// 起初写成后者，把 10 次成功穿越也算进上限，于是一条沿途有 4 个窄处、
  /// 但完全走得通的路径被判成「不可行」。长路径经过多个窄处是常态。
  int narrow_engage_count_{0};
  /// 已就当前目标放弃过一次（抛过「路径不可行」）。防止 20Hz 重复抛同一异常：
  /// 抛异常到 FollowPath 真的 abort 之间有延迟，实测每次放弃刷 15 条同样的 WARN。
  bool narrow_gave_up_{false};
  /// 已就**当前这一处**窄通道抛过异常（红线 / 中心致命）。
  ///
  /// 与 narrow_gave_up_ 的区别：那个按目标生效、直到换目标才复位；
  /// 这个在机器人脱离窄通道（足迹代价回落）时就复位，所以同一个目标里
  /// 后面遇到**新的**窄处仍然能如实上报。
  ///
  /// 需要它的理由与 narrow_gave_up_ 相同：抛异常到 FollowPath 真的 abort
  /// 之间有延迟，期间 controller_server 仍以 20Hz 调进来。实测红线一次触发
  /// 刷了 4~5 条同样的 WARN。
  bool narrow_threw_here_{false};
  bool warned_narrow_frame_{false};
  bool warned_narrow_heading_{false};
  /// 足迹多边形。每拍从 costmap_ros_ 取（可能被 /footprint 动态改），
  /// 存成员只为省一次分配。
  nav2_costmap_2d::Footprint footprint_cache_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;

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
  /// 终点位移小于此值即视为同一目标（默认行为树 1Hz 重规划会反复调 setPlan）。
  double new_goal_epsilon_m_{0.25};

  // ---- 窄通道参数 ----
  /// 总开关。false = 完全走旧行为（只调内层），一行代价查询都不做。
  bool narrow_enabled_{true};
  NarrowTriggerConfig narrow_trigger_{};
  NarrowLimits narrow_limits_{};
  /// 退出需要的连续「已脱离」拍数。
  int narrow_clear_ticks_{3};
  /// 横向扫描单侧范围(m)。同时是**离参考路径的横向偏移上限** ——
  /// 用户红线「脱困不能脱离全局参考路径太远」由这一项和
  /// narrow_max_path_deviation_m_ 共同保证。
  double narrow_scan_half_width_m_{0.30};
  /// 横向扫描步长(m)。必须远小于栅格 0.05 —— 目标区间只有 0.032m 宽。
  double narrow_scan_step_m_{0.01};
  /// 有利朝向周期(rad)。正八边形足迹 = pi/4。
  double narrow_favorable_period_rad_{0.7853981634};
  /// 估通道方向的前视弧长(m)。
  double narrow_heading_lookahead_m_{0.40};
  /// 卡住判据：这么久没有**弧长进展**就算原地蹭(s)。
  ///
  /// 注意这里判的是进展，不是时长。旧的「接管超过 N 秒即失败」等于给窄通道
  /// 设了 v_along x N 的长度上限（实测 25s x 0.10 = 2.5m，一条 2.5m 的健康
  /// 通行被连砍 3 次）。「无限循环脱困」的真实特征是时间在走而弧长不涨。
  double narrow_stall_timeout_sec_{6.0};
  /// 小于此弧长增量不算进展(m)。置 0 会让栅格噪声冒充进展、判据形同虚设。
  double narrow_stall_min_gain_m_{0.05};
  /// 绝对上限兜底(s)。刻意设在「长通道正常通行」够用的量级之外，
  /// 只防弧长判据本身失效时的无限贴边。
  double narrow_hard_timeout_sec_{120.0};
  /// 进展判据的跨拍状态。每次进入接管重置，**换路径时也重置**。
  NarrowProgressState narrow_progress_{};
  /// 上一拍看到的路径终点与点数，用来识别"路径被重规划换掉了"。
  ///
  /// 必须有：进展高水位是跨拍保留的，而换目标时剩余弧长会整体跳变
  /// （新目标更远 ⇒ 剩余变大），不重开窗口就永远超不过旧高水位 ⇒ 误判卡住。
  PlanarPoint narrow_last_path_end_{};
  std::size_t narrow_last_path_size_{0U};
  /// 同一条路径最多接管次数。超限抛异常，交给上层换路径。
  int narrow_max_engagements_{3};
  /// 机器人离参考路径的最大距离(m)。超出抛异常 ——
  /// 用户红线「脱困不能脱离全局参考路径太远，防止机器人乱跑」。
  double narrow_max_path_deviation_m_{0.50};
};

}  // namespace astribot_s1_path_tracking

#endif  // ASTRIBOT_S1_PATH_TRACKING__THREE_PHASE_CONTROLLER_HPP_
