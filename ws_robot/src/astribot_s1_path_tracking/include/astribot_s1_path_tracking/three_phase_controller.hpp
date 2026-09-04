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
#include <atomic>
#include <cmath>

#include "astribot_s1_path_tracking/align_math.hpp"
#include "astribot_s1_path_tracking/narrow_math.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
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
  rclcpp::Time phase_started_{0, 0, RCL_ROS_TIME};   // 时钟源必须显式给，默认是 SYSTEM_TIME、与 clock_->now() 相减即抛
  double start_heading_{0.0};
  bool start_heading_valid_{false};
  /// 上一条路径的终点，用于区分「新目标」与「同一目标的周期性重规划」。
  PlanarPoint last_goal_end_{};
  bool has_last_goal_{false};
  bool warned_tolerance_fallback_{false};
  /// 上一次 computeVelocityCommands 的时刻，用来区分「同一目标的周期重规划」
  /// 与「同一目标的**新一次** FollowPath 下发」。见 isFreshFollowAttempt 的
  /// 头注释：这两者路径内容几乎一样，只有时间空档能分开。
  /// **必须显式 RCL_ROS_TIME**：默认构造是 SYSTEM_TIME，与 clock_->now() 相减即抛。
  rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};
  bool has_tick_{false};

  // ---- 窄通道接管状态 ----
  /// 足迹碰致命带的连续拍数（进入用）。
  int narrow_hits_{0};
  /// 足迹已脱离致命带的连续拍数（退出用）。
  int narrow_clear_hits_{0};
  bool narrow_engaged_{false};
  /// 本次接管的起始时刻。**必须显式指定 RCL_ROS_TIME**：默认构造是
  /// RCL_SYSTEM_TIME，与 clock_->now() 相减会抛 "different time sources
  /// [1 != 2]"。原先只有接管中才相减所以从未暴露；2026-09-03 加了每拍都问的
  /// 最短驻留判据后，controller 逐拍抛异常、follow_path 逐拍 Aborting，
  /// 机器人一步没走。时钟源不匹配是编译期看不出、运行期必炸的一类错。
  rclcpp::Time narrow_started_{0, 0, RCL_ROS_TIME};
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
  /// tick 空档超过此值 => 上一个 FollowPath action 已结束，这次 setPlan 是新一次
  /// 尝试。默认 0.5s = 20Hz 下 10 拍：远大于单拍抖动，又远小于 align_timeout(15s)。
  double new_attempt_gap_sec_{0.5};

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
  /// 朝向等价周期(rad)。**必须 pi/2**，两种足迹统一。语义与不能取 pi/4 的
  /// 理由见 narrow_math.hpp 的 PrealignConfig::favorable_period_rad。
  /// 启动时逐个足迹用 periodPreservesLateralExtent() 校验，不成立即拒绝启动。
  double narrow_favorable_period_rad_{M_PI / 2.0};
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
  /// 「代价场饱和 + 内层无进展」检测器状态。跑在早退**之前**。
  NarrowStallState narrow_stall_{};
  /// 本次接管是否由「饱和无进展」这条第二入口进来的。
  /// 🔴 迟滞两侧必须同源：由 253 进来的，脱离判据也用 253（不是 254），
  ///    否则接管的同一拍就满足「足迹 < 254」的脱离条件，保证振荡。
  bool narrow_via_saturation_{false};
  /// 由第二入口接管的累计次数。上报在统计行里，用于判这条入口有没有在乱开。
  int narrow_saturation_engagements_{0};
  /// 由第二入口接管后的**最短驻留**。代价掉下 253 在这段时间内不足以退出。
  ///
  /// 🔴 为什么必须有（2026-09-03 实测）：只把脱离阈值从 254 改成 253 仍然振荡，
  /// 因为这一类通道的代价场不是平的 253，而是 229~253 的纹理（贴边层自己的
  /// 横向扫描当场报回 229/233/249/253）。机器人一边原地转，当前位姿的八边形
  /// 代价就会自己掉到 233 < 253 ⇒ 3 拍脱离命中 ⇒ 立刻交回 MPPI。
  /// 实测 18 次接管里 13 次时长只有 0.35~1.80s，而接管起手的朝向误差是
  /// 0.42~0.77rad、闸门 0.12rad、wz 上限 0.20rad/s ⇒ **转正就需要 1.5~3.3s**，
  /// 于是 37 个接管拍里 27 拍是纯原地转(vx=0)、一次都没转完就被赶出去，
  /// 出去后 MPPI 又转回它自己的朝向、再卡 3s、再从同样的 0.44rad 重来。
  ///
  /// 这就是「迟滞同源」的另一半：上一轮只对齐了**阈值**(253 对 253)，
  /// 没对齐**变量** —— 入口问的是「有没有推进」，出口问的却是「代价高不高」。
  /// 最短驻留把出口重新压回入口那个变量上：先给足能转正的时间，
  /// 再谈代价掉没掉。这个下界不是调出来的，是算出来的：
  ///     朝向误差到最近有利朝向 <= narrow_favorable_period/2
  ///     转正耗时 <= (narrow_favorable_period/2) / narrow_wz_max = 0.785/0.20 = 3.93s
  /// 故默认 4.0s，且启动守卫强制 >= 该算术下界、< 卡住判据(6s)。
  double narrow_saturation_min_dwell_sec_{4.0};
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

  // ---------------- 进入窄通道前的朝向预对齐 ----------------
  //
  // 原来本层只在「足迹已经碰到致命带」之后才接管并转朝向，也就是机器人总是
  // **先以不利朝向撞进窄处**再在里面转。narrow_math.hpp:118 记录的实测时序
  // （中心代价 0→218→229→致命，被推进膨胀带深处，最后自己变成 ESCAPE 的对象）
  // 就是这个缺陷。预对齐趁还在开阔处（原地转零风险）先把车转正。
  //
  // ⚠️ 它**不改变全局规划器的任何判据**：SmacPlanner2D 读栅格代价值、
  //    膨胀层只用内切半径，整条链旋转无关。收益全在 MPPI 侧与
  //    「不被推进膨胀带深处」这条链上。

  /// 一键回退到预对齐之前的行为（仓库规范：不删旧逻辑）。
  bool narrow_prealign_enabled_{true};
  PrealignConfig narrow_prealign_{};
  /// 旋转扫掠校验的角步长(rad)。
  double narrow_prealign_sweep_step_rad_{0.20};
  /// 单次预对齐的超时(s)。超时只放弃并告警，**不抛异常** ——
  /// 预对齐失败属于「没能开始」，计入放弃上限会误杀整条可行路径
  /// （见 three_phase_controller.cpp:725 那段实测教训）。
  double narrow_prealign_timeout_sec_{5.0};
  /// 每个目标最多预对齐几次，防振荡。
  int narrow_prealign_max_per_goal_{5};

  /// 是否正在预对齐（原地转）。
  bool prealign_active_{false};
  double prealign_target_yaw_{0.0};
  rclcpp::Time prealign_started_{0, 0, RCL_ROS_TIME};   // 时钟源必须显式给，默认是 SYSTEM_TIME、与 clock_->now() 相减即抛
  /// 本目标已用掉的预对齐次数。
  int prealign_used_{0};

  // 逐项计数，全部上报。合成一个总数就无法区分「从不触发」与「一路开阔」。
  int prealign_triggered_{0};
  int prealign_succeeded_{0};
  int prealign_timeout_{0};
  int prealign_sweep_blocked_{0};
  int prealign_capped_{0};
  int prealign_blocked_even_favorable_{0};
  bool warned_prealign_cap_{false};

  /// 预对齐的一拍决策，与 NarrowDecision 同构。
  struct PrealignDecision
  {
    bool take_over{false};
    geometry_msgs::msg::TwistStamped cmd;
  };

  /// 预对齐状态机。开阔处才会走到这里。
  PrealignDecision evaluatePrealign(
    const geometry_msgs::msg::PoseStamped & pose, const rclcpp::Time & now);

  /// 在**一次持锁**内把两种足迹的代价查询能力交给 fn。
  ///
  /// 🔴 绝不能在持锁期间调用内层控制器 —— MPPI 也要拿这把同一把锁，
  ///    那是必死的死锁（同 readCosts 的注释）。所以 fn 里只做查询，
  ///    出了这个作用域调用方才决定动作。
  /// @param fn  (大足迹取值, 小足迹取值)。缩足迹未启用时第二个为空。
  /// @return false = 代价地图/足迹不可用，fn 未被调用
  bool withCostQueries(
    const std::function<void(const FootprintCostFn &, const FootprintCostFn &)> & fn);

  /// 一次持锁跑完策略前视 + 扫掠校验 + 当前位姿的大足迹代价。
  bool runPrealignQueries(
    const std::vector<PlanarPoint> & path,
    const PlanarPoint & robot,
    double robot_yaw,
    StrategyPreview & preview,
    bool & sweep_clear,
    double & sweep_worst_cost,
    double & big_footprint_cost);

  // ---------------- 窄通道内临时缩小足迹（八边形 -> 正方形）----------------
  //
  // 依据全部在 narrow_math.hpp 那一节。这里只强调实现纪律：
  //
  // 🔴 顺序不可颠倒：**先对齐、后换足迹**。膨胀层只用内切半径、旋转无关，
  //    小足迹把代价地图的朝向盲区从 0.035 放大到 0.133（nav2 自己的
  //    calculateMinAndMaxDistances 实测），所以小足迹生效期间朝向必须锁住。
  //
  // 🔴 切换是**跨进程**的：planner 用的是 global costmap（另一个进程），
  //    只能发 Polygon 到它的 footprint 话题，是 fire-and-forget。
  //    所以必须**回读验证**（published_footprint），验证不过就复原并放弃，
  //    绝不"假定切成功"继续走。
  //
  // 🔴 防锁存：设了小足迹的进程若挂掉，代价地图会一直按小足迹算。
  //    本层负责 deactivate/cleanup/异常/新目标时复原，并按周期续租；
  //    租约过期由协调器侧看门狗兜底。

  /// 一键回退：false 时完全不碰足迹（退回只做预对齐的行为）。
  bool narrow_square_enabled_{false};
  /// 默认(大)足迹与窄通道(小)足迹，均从 yaml 解析。
  std::vector<geometry_msgs::msg::Point> footprint_default_{};
  std::vector<geometry_msgs::msg::Point> footprint_narrow_{};
  /// 两者的内切/外接半径，由 nav2 的 calculateMinAndMaxDistances 算出（不自己实现）。
  double footprint_default_inscribed_{0.0};
  double footprint_default_circumscribed_{0.0};
  double footprint_narrow_inscribed_{0.0};
  double footprint_narrow_circumscribed_{0.0};
  /// 对齐到通道方向后（delta=0）两种足迹的**侧向半宽**。
  /// 🔴 通道能不能过只取决于这个量，不是内切/外接半径。
  ///    切换的收益判据、以及"切了是否真的更窄"的运行期断言都用它。
  double footprint_default_lateral_{0.0};
  double footprint_narrow_lateral_{0.0};
  /// 启动守卫：小足迹内切半径不得小于底盘物理包络，防打错字缩到比躯干还小。
  double chassis_min_envelope_radius_{0.30};
  /// 小足迹最长生效时长(s)。红线：所有受限动作必须带超时。
  double narrow_square_hard_timeout_sec_{30.0};
  /// 续租周期(s)与回读验证超时(s)。
  double narrow_square_lease_period_sec_{0.5};
  double narrow_square_verify_timeout_sec_{2.0};

  // ---- 切换迟滞 ----
  //
  // 🔴 为什么必须有：第一轮 A/B 实测 123 次切入 / 121 次"装得下"复原 /
  //    单次驻留中位 0.15s，小足迹只生效 5.7%。0.15s 恰好 = narrow_clear_ticks(3)
  //    / controller_frequency(20Hz)，也就是说复原**每次都在法律允许的最早那一拍**触发。
  //
  //    根因不是参数没调好，而是**切入与切出问的是两个不同的几何问题**：
  //      · 切入：沿路径**前视 preview_m** 有没有大足迹过不去的点
  //      · 切出（旧）：**只看机器人当前位姿**大足迹压不压致命带
  //    而当前位姿的大足迹代价必然低于阈值 —— 否则 evaluateNarrow 早就接管了，
  //    根本走不到预对齐。于是切出判据在小足迹生效那一瞬间就已成立。
  //
  //    所以修法是让两边问同一个问题、且切出的窗口更长：
  //      切入：preview_m(1.00m) 内有麻烦        ⇒ 切
  //      切出：exit_preview_m(1.20m) 内全无麻烦 ⇒ 复原
  //    0.20m 的迟滞带 = 4 个栅格(0.05m)，是代价地图**表达得出**的量。
  //    （对比：八边形各向异性 0.032m < 一个栅格，所以纯预对齐测不出效果。）

  /// 切出用的前视窗口(m)。启动守卫：必须 >= 切入窗口 narrow_prealign_.preview_m。
  double narrow_square_exit_preview_m_{1.20};
  /// 切出需要连续几拍成立。与 narrow_clear_ticks_ 分开：那个是接管层的退出判据。
  int narrow_square_exit_ticks_{6};
  /// 最短驻留(s)：切入后这段时间内**不许**因"装得下"复原。
  /// ⚠️ 只压这一条复原路径；硬超时/新目标/deactivate/看门狗都是安全通路，不受它限制。
  double narrow_square_min_dwell_sec_{2.0};
  /// 复原后的冷却期(s)：这段时间内不许再切入，防"复原-立刻再切"自激。
  double narrow_square_cooldown_sec_{3.0};

  /// 小足迹当前是否**已验证生效**（不是"已请求"）。
  bool square_active_{false};
  /// 已请求但还没回读验证通过。
  bool square_pending_{false};
  rclcpp::Time square_requested_{0, 0, RCL_ROS_TIME};   // 时钟源必须显式给，默认是 SYSTEM_TIME、与 clock_->now() 相减即抛
  rclcpp::Time square_activated_{0, 0, RCL_ROS_TIME};   // 时钟源必须显式给，默认是 SYSTEM_TIME、与 clock_->now() 相减即抛
  rclcpp::Time square_last_lease_{0, 0, RCL_ROS_TIME};   // 时钟源必须显式给，默认是 SYSTEM_TIME、与 clock_->now() 相减即抛
  /// 请求缩足迹时锁定的目标朝向（失去对齐要转回它，而不是复原足迹）。
  double square_locked_yaw_{0.0};
  /// 复原时刻（冷却期起点）。⚠️ 必须配 valid 标志：默认构造的 rclcpp::Time 是
  /// RCL_SYSTEM_TIME，与节点时钟(RCL_ROS_TIME)相减会**抛异常**，不是返回大值。
  bool square_cooldown_valid_{false};
  rclcpp::Time square_cooldown_started_{0, 0, RCL_ROS_TIME};   // 时钟源必须显式给，默认是 SYSTEM_TIME、与 clock_->now() 相减即抛
  /// 切出判据连续成立的拍数。
  int square_exit_clear_hits_{0};

  int square_switch_count_{0};
  int square_verify_fail_count_{0};
  int square_revert_cleared_{0};      ///< 因大足迹重新装得下而复原
  int square_revert_timeout_{0};      ///< 因硬超时而复原
  /// 是否正处于小足迹生效状态（含"已请求待回读确认"）。
  ///
  /// 供 evaluateNarrow 的"便宜早退"开例外用：缩足迹成功后 footprint_cost
  /// 正好从 254 掉到 253，早退条件命中，贴边通行层就不接管了，控制权落回
  /// 内层 MPPI —— 而那一段代价恒 253、零梯度，MPPI 只会来回蹭到硬超时。
  [[nodiscard]] bool squareActive() const {return square_active_ || square_pending_;}

  int square_realign_count_{0};       ///< 生效期间失去对齐 -> 保持小足迹、重新对齐
  /// 是否正处于"接管旋转、等对齐到 yaw_resume_rad"的状态（朝向层迟滞的锁存位）。
  ///
  /// 没有这个锁存位就没有迟滞：切入切出同一个阈值 ⇒ 在闸门上自激。
  /// 实测 749 次重新对齐、30s 硬超时窗口内一步没前进 ⇒ "能进不能出"。
  bool square_realigning_{false};
  int square_suppressed_dwell_{0};    ///< 切出判据已成立但被最短驻留压住的拍数
  int square_suppressed_cooldown_{0}; ///< 想切入但被冷却期挡掉的拍数
  int square_exit_degraded_{0};       ///< 拿不到路径、退化成只看当前位姿判切出的拍数
  /// 第二入口接管期间，「足迹已脱离致命带」但被最短驻留压住的拍数。
  /// 与 square_suppressed_dwell_ 刻意分开：那是方形足迹层的切出计数，
  /// 两层的驻留阈值不同(4.0s vs 2.0s)，混用会把两个现象记成一个数。
  int narrow_suppressed_dwell_{0};
  /// 前视判定为「缩足迹能过」的拍数 —— 即**本功能有用武之地的拍数**。
  ///
  /// 🔴 为什么必须单独计：它的对照量 prealign_blocked_even_favorable_
  ///    （连小足迹转正都过不去）第一轮实测 790，而"需缩足迹"那行是**节流 INFO
  ///    且没有累计值**，只能数到 1 行 —— 两个数根本不可比，而"790 : 1"这个
  ///    比值恰恰是决定这功能该不该留下的唯一依据。节流日志不能当事件计数器。
  int square_applicable_ticks_{0};
  /// 因「切了反而更宽」而被运行期断言拒绝的次数。>0 就说明周期/足迹配错了。
  int square_refused_wider_{0};

  /// ⚠️ 必须是 LifecyclePublisher 而不是 rclcpp::Publisher：后者能编译
  /// （LifecyclePublisher 继承自它），但拿不到 on_activate()，而**未激活的
  /// lifecycle 发布者会静默丢弃所有消息** —— 那正是本功能最不能出的错。
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Polygon>::SharedPtr
    global_footprint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr global_footprint_sub_;
  /// 最近一次从 global costmap 回读到的顶点数（0 = 还没收到）。
  std::atomic<int> global_footprint_vertices_{0};

  /// 解析两个足迹、算半径、跑启动守卫。非法即抛。
  void loadFootprints(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & p);

  /// 请求把两个代价地图切到指定足迹。local 直接调（同进程），global 发话题。
  void requestFootprint(const std::vector<geometry_msgs::msg::Point> & fp);

  /// 回读 global costmap 的足迹顶点数是否已等于期望值。
  bool footprintVerified(std::size_t want_vertices) const;

  /// 校验「按 narrow_favorable_period 取模」对该足迹是否保侧向包络。不成立即抛。
  void checkPeriodPreservesLateral(
    const std::vector<geometry_msgs::msg::Point> & fp, const char * what) const;

  /// 复原到默认足迹并清状态。why 会进日志。**幂等**，随时可调。
  /// 副作用：开启冷却期（防"复原-立刻再切"自激）。
  void revertFootprint(const char * why);

  /// 无条件续租。
  ///
  /// 🔴 必须在**每一拍、任何相位、任何一层接管**的情况下都被调用。
  ///    第一轮实测的教训：续租原先写在 evaluateSquare 里，而 evaluateNarrow
  ///    接管时 kFollow 会 early-return，根本走不到那行 —— 于是恰好在窄通道
  ///    接管期间（最需要小足迹的时候）停止续租，协调器侧看门狗在 2.1s 时
  ///    误判"设它的进程死了"，把足迹从中途抢回默认值。
  ///    续租是**存活信号**，与"本拍是哪一层在控制"完全无关。
  void renewFootprintLease(const rclcpp::Time & now);

  /// 切出判据的一次求值。
  struct SquareExitProbe
  {
    bool clear{false};        ///< 本拍判定"大足迹已经装得下"
    bool from_path{false};    ///< true=用了前视窗口；false=退化成只看当前位姿
  };

  /// 小足迹状态机。返回是否本拍接管（接管时 cmd 有效）。
  PrealignDecision evaluateSquare(
    const geometry_msgs::msg::PoseStamped & pose,
    const rclcpp::Time & now,
    const StrategyPreview & preview,
    double robot_yaw,
    const SquareExitProbe & exit);
};

}  // namespace astribot_s1_path_tracking

#endif  // ASTRIBOT_S1_PATH_TRACKING__THREE_PHASE_CONTROLLER_HPP_
