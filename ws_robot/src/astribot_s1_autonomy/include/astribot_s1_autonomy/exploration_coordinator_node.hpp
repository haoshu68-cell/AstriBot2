// Copyright 2026 Astribot.
//
// 探索协调器：严格时序探索调度 + 未知区域禁行强校验。
//
// ============================ 它解决什么问题 ============================
// 本包已有的 frontier_explorer_node 是「候选点建议流」——每个规划周期都发一次
// /explore/goal_pose，不管上一个目标走到哪了。直接把那个流接到 Nav2 会出现
// 需求点名禁止的问题：提前下发、重叠下发、连续跳点。
// （实测过：临时 Python 桥接 20s 内向 Nav2 下发 18 次目标、反复抢占、成功 0 次。）
//
// 本节点是「调度器」：
//   · 严格单点推进 —— 必须完全抵达并稳定驻留当前目标，才允许生成下一个
//   · 双层校验 —— 目标点本身 + 从当前位置到目标的全局路径，都不许碰未知栅格
//   · 异常闭环 —— 导航失败/定位丢失/候选不合法都有明确的状态与限次重试
//
// 两类栅格图的分工（实测踩坑后定下的，混用会让探索一个目标都发不出去，
// 完整机制见 costmap_adapter.hpp 文件头）：
//   /map                       —— 只用于前沿搜索（前沿的定义依赖「未知」状态）
//   /global_costmap/costmap_raw —— 只用于下发前校验（与 planner_server 同一张图）
//
// 与 Nav2 的关系：**只用官方接口，不改 Nav2 任何源码**。
//   NavigateToPose      —— 执行导航
//   ComputePathToPose   —— 只为拿到全局路径做校验，不用它来控制
// =====================================================================
#ifndef ASTRIBOT_S1_AUTONOMY__EXPLORATION_COORDINATOR_NODE_HPP_
#define ASTRIBOT_S1_AUTONOMY__EXPLORATION_COORDINATOR_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "astribot_s1_autonomy/escape_logic.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "astribot_s1_autonomy/costmap_adapter.hpp"
#include "astribot_s1_autonomy/exploration_state.hpp"
#include "astribot_s1_autonomy/failure_budget.hpp"
#include "astribot_s1_autonomy/frontier_search.hpp"
#include "astribot_s1_autonomy/path_validator.hpp"

namespace astribot_s1_autonomy
{

class ExplorationCoordinatorNode : public rclcpp::Node
{
public:
  explicit ExplorationCoordinatorNode(const rclcpp::NodeOptions & options);
  ~ExplorationCoordinatorNode() override;

  ExplorationCoordinatorNode(const ExplorationCoordinatorNode &) = delete;
  ExplorationCoordinatorNode & operator=(const ExplorationCoordinatorNode &) = delete;
  ExplorationCoordinatorNode(ExplorationCoordinatorNode &&) = delete;
  ExplorationCoordinatorNode & operator=(ExplorationCoordinatorNode &&) = delete;

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using FollowPath = nav2_msgs::action::FollowPath;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using PlanGoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  /// 下发方式。
  ///   kNavigateToPose —— 只发目标点，路径由 bt_navigator 内部重新规划。
  ///                      保留 BT 的 1Hz 重规划与整套恢复行为。
  ///   kFollowPath     —— 直接把**已通过双层校验的那条路径**交给控制器跟踪。
  ///                      省掉一次重规划，且跟踪的就是被校验过的路径；
  ///                      代价是 BT 的重规划与恢复行为全部拿不到，
  ///                      必须由本节点自己承担（见 replan_period_sec）。
  enum class DispatchMode
  {
    kNavigateToPose,
    kFollowPath,
  };

  /// 跟踪期的重规划策略。
  ///
  ///   kOnInvalid（默认）—— 只在「当前路径已经不能用」时才重规划：剩余段被新观测
  ///       判成不可通行、机器人已偏离路径、控制器中止、或路径超龄。
  ///       这是需求「未跟踪到位不得开始规划下一条路径」的实现。
  ///   kPeriodic       —— 旧行为：无条件按 replan_period_sec 周期重规划。
  ///       只保留作一键回退对比用。实测数据：36 个目标下发了 393 次 FollowPath，
  ///       平均每个目标换 10.9 条路径、节拍 ~1.5s，没有一条被跟踪到位。
  enum class ReplanPolicy
  {
    kOnInvalid,
    kPeriodic,
  };

  /// 冷启动自举的动作方式。
  ///
  ///   kDisabled —— 不自举（出问题时一键关掉）。
  ///   kRotate   —— 只原地旋转。**刻意不提供平移**：底盘足迹是外接半径 0.42 的
  ///       正八边形（内切半径 0.388），原地旋转最多扫过 3.2cm 的环带，
  ///       几何上几乎不进入新区域；而平移是开环积分推进，风险面完全不同。
  ///       0.2rad 的旋转已经足够触发 slam_toolbox 的 minimum_travel_heading(0.2)
  ///       插入首批扫描，这就够破环了。
  enum class BootstrapMode
  {
    kDisabled,
    kRotate,
  };

  /// 一个待校验/待执行的目标。
  struct GoalCandidatePose
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double cost{0.0};
  };

  // ---------------- 参数 ----------------
  void declareParameters();
  bool loadParameters(std::string & error);

  // ---------------- 状态机 ----------------
  //
  // !!! 线程约定（整份实现的地基，改动前务必先读）!!!
  // 下面这一整组「状态机 / 生成校验 / 下发监控」的私有函数，**一律要求调用方
  // 已持有 state_mutex_**。锁不在函数内部拿，而是由三类入口统一拿：
  //     controlTick()          —— 定时器入口
  //     onPlanResult() / onNavGoalResponse() / onNavResult()  —— action 回调入口
  //     onPauseService() / onResumeService()                  —— 服务入口
  // 这样做的原因：需求要求「每次目标下发必须有状态锁与时序锁，禁止多线程重复下发」。
  // 如果每个小函数各自加锁，状态判断和状态修改之间就会出现锁间隙，
  // 两个线程能各自通过「现在是 kValidating」的检查、然后双双下发目标。
  // 把锁提到入口，一次 tick / 一次回调就是一个原子的状态推进。

  /// 集中式状态转换。非法转换会被拒绝并回退到 kIdle（需求：状态机异常自动回退安全态）。
  void transitionTo(ExplorationState next, const std::string & why);
  /// 主循环，由定时器驱动。
  void controlTick();

  void tickIdle();
  void tickBootstrap();
  void tickGenNextPoint();
  void tickValidating();
  void tickNavigating();
  void tickArrived();
  void tickPaused();
  void tickCompleted();

  // ---------------- 冷启动自举 ----------------
  /// 是否应当进入自举，并给出原因。只在 IDLE / GEN_NEXT_POINT 里调用。
  ///
  /// 刻意**不**从 PAUSED 触发：PAUSED 的语义是需求写死的「保持当前状态、输出告警、
  /// 等待人工重置」，在那个状态下自己动起来与需求直接冲突。冷启动死锁本来就发生在
  /// 预算耗尽之前的 IDLE/GEN_NEXT_POINT，从这两处触发已经够破环。
  /// context 是**调用方**的触发理由（地图内容不足 / 采不到候选 / 地图未就绪）。
  /// 必须由调用方传进来：本函数只知道"停滞了多久"，不知道停在哪一步 ——
  /// 早先把理由写死成"地图内容不足"，结果采不到候选那条路径也报这句，
  /// 日志直接指错方向（实测：地图明明有 71 个前沿格，日志却说地图内容不足）。
  bool shouldBootstrap(const std::string & context, std::string & why);
  /// 进入自举：记录起始朝向、清零计时。
  void beginBootstrap(const std::string & why);
  /// 自举安全门。返回 false 时必须立刻停车并显式告警（绝不静默继续转）。
  bool bootstrapSafe(std::string & why);
  /// 向底盘发一帧速度指令。zero=true 时发全零（用于停车兜底）。
  void publishBootstrapCmd(bool zero);
  /// 自举专用定时器回调：以 bootstrap_cmd_rate_hz 持续重发速度。
  ///
  /// 为什么不能靠 controlTick 发：底盘 cmd_vel_timeout_sec=0.5，而控制节拍是 0.5s，
  /// 正好卡在超时边界上，速度会被反复归零 —— 必须独立高频重发。
  void bootstrapCmdTick();

  // ---------------- 膨胀带脱困（ESCAPE）----------------
  //
  // 与 BOOTSTRAP 是两个不同的死锁：BOOTSTRAP 治「地图还没长出来」，
  // ESCAPE 治「机器人站在膨胀带里、全局规划器拒绝从这儿起步」。
  // 判据是单个中心格 cost>=253，所以**原地旋转无效**，必须平移。

  /// 读机器人所在格在 costmap(规划器视角) 与 /map(物理真值) 上的三态值。
  CellReading readRobotCell();

  /// 判断是否该进 ESCAPE。返回结论并填原因；调用方负责状态跃迁与告警。
  EscapeVerdict checkEscapeTrigger(std::string & why);

  /// 选一个脱困目标：先来路、再最近可规划格。失败返回 false。
  bool pickEscapeTarget(PlanarPoint & target, std::string & why);

  /// ESCAPE 状态每拍。
  void tickEscape();

  /// 脱困专用速度发布（车体系，走 bootstrap_cmd_pub_ 同一个出口）。
  void publishEscapeCmd(const EscapeCommand & cmd);

  /// 记一个面包屑（低频采样机器人位姿），并裁掉过期的。
  void recordBreadcrumb();
  /// 地图是否「已经能用来做探索决策」。
  ///
  /// 与 mapReady() 的区别（这是两回事，混用会造出假的 COMPLETED）：
  ///   mapReady()  —— 消息层面：收到了、自洽、没超时。
  ///   mapUsable() —— 内容层面：已知格数量够不够支撑一次前沿判定。
  /// 冷启动时地图是「收到了但全是未知」，mapReady 为真而 mapUsable 为假；
  /// 此时 raw_frontier_cells==0，只看前沿格数就会把它判成「探索完成」。
  bool mapUsable(const GridMap & map, std::size_t & known_cells) const;

  // ---------------- 数据回调 ----------------
  void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  /// 自举安全门用的激光回调。**必须用 BEST_EFFORT 订阅**：
  /// 本项目已经在探针脚本上踩过 —— RELIABLE 订阅 BEST_EFFORT 发布方一帧都收不到，
  /// 而当时的代码把「没收到数据」当成了「前方无障碍」，结果盲走 3m。
  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg);
  /// 全局代价地图回调。这是**下发前校验**所用的栅格图，
  /// 与 planner_server 规划时所用的是同一张，避免两层判据互相锁死。
  /// 详细机制见 costmap_adapter.hpp 文件头。
  void costmapCallback(const nav2_msgs::msg::Costmap::ConstSharedPtr & msg);

  // ---------------- 前置条件 ----------------
  /// 地图是否可用（非空、自洽、未超时）。不可用时拦截生成逻辑。
  /// 整栈是否已就绪：地图 + 定位 + 里程计 + **下发前校验用的栅格图**。
  ///
  /// 为什么要它：三个失败计数器（导航失败/采样失败/自动恢复）原先在启动瞬态里
  /// 就开始累加，而那时 TF 还没铺开、代价地图还没发布。实测两次死锁都是这样来的：
  ///   · 3~4 次 TF extrapolation 瞬态失败烧光 3 次自动恢复预算 -> 永久停住；
  ///   · 图还没长起来时采不到候选，同样烧光预算 -> 永久停住。
  /// 就绪之前的失败**只记日志、不计数**；就绪之后的失败照常计数。
  bool stackReady(std::string & why);

  bool mapReady(std::string & why);
  /// 里程计是否新鲜。不新鲜视为定位/里程计丢失，必须冻结目标发布。
  bool odomReady(std::string & why);
  /// 机器人位姿是否可用。取不到即视为定位丢失（查的是 map -> base）。
  bool robotPose(double & x, double & y, double & yaw, std::string & why);
  /// 在**指定** frame 下查机器人位姿。
  ///
  /// 为什么要能指定：自举必须在 odom 系里量转角。map -> odom 由 SLAM 发布，
  /// 而真正的冷启动死锁下 SLAM 还没出图，用 map 系会让自举被自己的前置条件挡死
  /// （实测连续 71 次「需要自举但取不到当前朝向」）—— 那正是它要破的死锁。
  bool poseInFrame(
    const std::string & frame, double & x, double & y, double & yaw, std::string & why);
  /// 取「下发前校验」应当使用的栅格图快照。
  ///
  /// use_costmap_for_validation_ 为真时返回代价地图快照（不可用/过期则返回
  /// nullptr 并填 why —— 绝不静默退回 /map，否则又变成两张图校验）；
  /// 为假时返回 /map 快照（保留旧行为，供出问题时一键回退对比）。
  std::shared_ptr<GridMap> validationGrid(std::string & why);

  // ---------------- 目标生成与校验 ----------------
  /// 跑前沿搜索，产出按代价升序排列的候选点。
  std::vector<GoalCandidatePose> generateCandidates(
    const GridMap & map, double rx, double ry, std::size_t & raw_frontier_cells);
  /// 对当前候选队列的队首发起路径校验（异步）。
  void requestPlanForCurrentCandidate();
  void onPlanGoalResponse(const PlanGoalHandle::SharedPtr & handle);
  /// ComputePathToPose 结果回调：做逐点未知区校验，通过则下发导航目标。
  void onPlanResult(const PlanGoalHandle::WrappedResult & result);
  /// 丢弃当前候选并推进到下一个；候选耗尽时按需转 PAUSED。
  void rejectCurrentCandidate(const std::string & reason);
  /// 本轮候选全部被拒后的收尾：按限次决定重新采样还是转 PAUSED。
  void onCandidatesExhausted(const std::string & reason);
  /// 清空本轮候选队列与游标。
  void resetCycleState();
  /// 记一次「去过/试过」，让代价函数后续避开这个位置。
  void recordVisit(double x, double y);

  // ---------------- 导航下发与监控 ----------------
  /// 下发导航目标。**这是全节点唯一一处下发点**，且只允许从 kValidating 调用。
  void dispatchNavGoal(const GoalCandidatePose & goal);
  void onNavGoalResponse(const NavGoalHandle::SharedPtr & handle);
  void onNavResult(const NavGoalHandle::WrappedResult & result);
  /// 把已校验路径交给 controller_server 跟踪（kFollowPath 模式）。
  void dispatchFollowPath(const GoalCandidatePose & goal, const nav_msgs::msg::Path & path);
  void onFollowGoalResponse(const FollowGoalHandle::SharedPtr & handle);
  void onFollowResult(const FollowGoalHandle::WrappedResult & result);
  /// kFollowPath 模式下的周期性重规划：BT 的 1Hz 重规划拿不到了，得自己发。
  void maybeRequestReplan(const rclcpp::Time & now);
  /// 当前在跟踪的这条路径是否已经不能用了（kOnInvalid 策略的判据）。
  ///
  /// 这是需求「没跟踪到位就开始规划下一条路径」的正面实现：默认答案是 false，
  /// 也就是**默认让控制器把当前路径跟踪完**，只有下面这几条硬条件之一成立才换路径：
  ///   1) replan_forced_ —— 控制器已经中止，当前路径事实上已经作废
  ///   2) 没有在途路径（异常兜底）
  ///   3) 剩余段被最新观测判成穿未知/占据区 —— 再跟下去就是往障碍里开
  ///   4) 机器人已偏离当前路径超过 path_deviation_limit_m —— 这条路径不再描述它的处境
  ///   5) 路径超龄超过 path_max_age_sec（>0 才启用，纯兜底）
  /// 每一条都会把原因写进 why，日志里能直接看出是哪一条触发的。
  bool needsReplan(const rclcpp::Time & now, std::string & why);
  /// 真正发出一次「跟踪期重规划」请求。调用方负责已经判定需要重规划。
  void requestReplan(const rclcpp::Time & now, const std::string & why);
  /// 「重规划没换成新路径」的统一收口。
  ///
  /// 当前路径还能走时只记日志继续跟；当前路径已经判死时累计次数，
  /// 超过 max_invalid_replan_attempts 就撤掉目标交给状态机另选一个 ——
  /// 绝不继续跟踪一条已知不可通行的路径。
  void onReplanProducedNoUsablePath(const std::string & detail);
  /// 取消在途导航目标（若有）。用于超时、定位丢失、人工暂停。
  void cancelActiveNavGoal(const std::string & reason);
  /// 记录一次导航失败，达上限则转 PAUSED。
  void registerNavFailure(const std::string & reason);

  // ---------------- 输出 ----------------
  void publishState();
  void publishComplete(bool done);

  // ---------------- 服务 ----------------
  void onPauseService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onResumeService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // ================= 成员 =================
  // ---- 参数：话题/坐标系 ----
  std::string map_topic_;
  std::string costmap_topic_;
  std::string odom_topic_;
  std::string state_topic_;
  std::string complete_topic_;
  std::string current_goal_topic_;
  std::string nav_action_name_;
  /// 下发目标时指定的行为树 xml 路径；空=用 bt_navigator 默认树。
  /// 探索场景用它切到三段式控制器（终点不转朝向）。
  std::string nav_behavior_tree_;
  std::string plan_action_name_;
  std::string map_frame_;
  std::string robot_base_frame_;
  std::string planner_id_;

  // ---- 参数：节拍与超时 ----
  double control_period_sec_{0.0};
  double tf_timeout_sec_{0.0};
  double map_timeout_sec_{0.0};
  double costmap_timeout_sec_{0.0};
  double odom_timeout_sec_{0.0};
  double plan_timeout_sec_{0.0};
  double nav_timeout_sec_{0.0};

  // 未知净空半径的副本。校验本身在 PathValidator 里，但「它是否与地图分辨率相容」
  // 只有收到真实地图才能判断，所以 mapCallback 需要能读到它。
  double unknown_clearance_radius_{0.0};

  // ---- 参数：抵达收敛判定 ----
  double arrival_xy_tolerance_{0.0};
  double arrival_yaw_tolerance_{0.0};
  bool check_yaw_{true};
  double dwell_time_sec_{0.0};
  double settle_speed_{0.0};

  // ---- 参数：重试与限次 ----
  int max_candidates_per_cycle_{0};
  // 两个失败上限的**唯一真值**在 failure_budget_ 里（max_sample_failures /
  // max_validation_failures），不要在这里再存一份镜像 —— 两份就会不同步。
  int max_consecutive_nav_failures_{0};
  double pause_cooldown_sec_{0.0};
  int max_auto_resume_attempts_{0};

  // ---- 算法 ----
  FrontierSearch search_;
  FrontierSearchParams search_params_;
  PathValidator validator_;
  CostmapAdapterParams costmap_params_;
  /// 下发前校验是否使用全局代价地图（默认 true）。
  /// 置 false 会退回「用 /map 校验」的旧行为——那是已知会锁死探索的配置，
  /// 只保留作对比排查用，正常运行不要关。
  bool use_costmap_for_validation_{true};

  // ---- ROS 句柄 ----
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr plan_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // ---------------- 足迹锁存看门狗 ----------------
  //
  // 要防的是什么：控制器可以在窄通道里临时把代价地图的足迹缩小
  //（见 three_phase_controller.hpp 那一节）。**设了小足迹的进程一旦挂掉/卡住，
  // 代价地图会一直按小足迹算**，机器人从此被系统性低估，而日志上一切正常。
  // 控制器自己负责 deactivate/cleanup/换目标时复原，但它死了就没人复原了。
  //
  // 机制：控制器按周期把小足迹重发到写话题（续租）。本看门狗同时订阅
  //   · 写话题   —— 知道"设它的人还活着"（最后一次请求的时刻）
  //   · 回读话题 —— 知道代价地图**当前**到底是什么足迹
  // 回读到非默认足迹、且租约已过期 ⇒ 把默认足迹发回去。
  //
  // ⚠️ 用独立的回调组：与 controlTick 共用互斥组会让这两个订阅被饿死
  //    （本项目实测过 cmd_vel 订阅一次都执行不到、且全程无告警）。
  //
  // 🔴 判据必须带**龄期**，不能只看"最后一次读到的顶点数"。第一轮实测的教训：
  //    · 回读来自 global costmap 的 publish_frequency = 1.0Hz（周期 1.0s），
  //      而本看门狗 tick 是 0.5s、控制器侧切换一度快到 ~1.7Hz。
  //    · 旧租约超时 2.0s < 连续 3 次回读所需的 3.0s ——
  //      也就是说它可能在"3 次连续读数还凑不齐"时就已经动手了。
  //    · 更根本的：回读一旦停更（costmap 挂了/话题断了），旧读数会被当成
  //      **当前值**继续用（本项目一天内犯过三次同类错）。
  //   所以现在三条同时成立才动手：
  //      ① 最新回读的龄期 <= readback_stale_sec（读数是活的）
  //      ② 连续 consecutive_reads 次**回读**都是非默认足迹
  //         （在回读回调里数，不在 tick 里数 —— 在 tick 里数会把同一个 1Hz
  //          采样重复计两次，是典型的采样别名）
  //      ③ 租约龄期 > lease_timeout_sec
  bool fp_watchdog_enabled_{false};
  std::string fp_watchdog_write_topic_;
  std::string fp_watchdog_readback_topic_;
  std::string fp_watchdog_default_footprint_;
  double fp_watchdog_lease_timeout_sec_{4.0};
  /// 回读周期(s)，仅用于启动守卫的算术（必须与 costmap 的 publish_frequency 一致）。
  double fp_watchdog_readback_period_sec_{1.0};
  /// 回读龄期上限(s)：超过这个就认为"读数不是当前值"，本看门狗**不动手**。
  double fp_watchdog_readback_stale_sec_{3.0};
  /// 需要连续多少次回读都是非默认足迹才允许动手。
  int fp_watchdog_consecutive_reads_{3};
  std::size_t fp_watchdog_default_vertices_{0U};
  /// 代价地图当前足迹的顶点数（回读）。0 = 还没收到。
  std::atomic<int> fp_current_vertices_{0};
  /// 最近一次回读到达的时刻。0 = 从没收到过。
  std::atomic<int64_t> fp_last_readback_ns_{0};
  /// 连续读到"非默认足迹"的**回读次数**（在回读回调里累加，读到默认即归零）。
  std::atomic<int> fp_nondefault_streak_{0};
  /// 最后一次在写话题上看到"非默认足迹"请求的时刻（= 续租时刻）。
  std::atomic<int64_t> fp_last_request_ns_{0};
  int fp_watchdog_reverts_{0};
  /// 因回读陈旧而**放弃判定**的次数。必须上报 —— 否则"看门狗没动手"会被
  /// 误读成"一切正常"，而真相可能是它已经瞎了。
  int fp_watchdog_stale_skips_{0};
  rclcpp::CallbackGroup::SharedPtr fp_watchdog_cb_group_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr fp_write_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr fp_readback_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr fp_revert_pub_;
  rclcpp::TimerBase::SharedPtr fp_watchdog_timer_;

  /// 建立看门狗的订阅/发布/定时器。未启用时什么都不做。
  void setupFootprintWatchdog();
  /// 一拍看门狗检查。
  void footprintWatchdogTick();
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// 定时器单独一个互斥回调组：前沿搜索有几十毫秒级开销，
  /// 放进独立组后即使用多线程执行器，也不会把 action 结果回调挤在后面排队。
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  /// 订阅/服务/action 客户端共用一个组，保证数据快照与状态查询彼此串行。
  rclcpp::CallbackGroup::SharedPtr io_cb_group_;

  // ---- 状态 ----
  /// 状态机主锁：保护 state_ 及其相关计数器。
  /// 所有 action 回调、服务回调、定时器都要先拿这把锁再动状态。
  std::mutex state_mutex_;
  ExplorationState state_{ExplorationState::kIdle};
  rclcpp::Time state_entered_time_;

  /// 「严格单点推进」的硬保险：任意时刻最多一个导航目标在途。
  /// 下发前 compare_exchange，失败即说明有并发下发企图，直接拒绝并 ERROR。
  std::atomic<bool> nav_goal_in_flight_{false};
  /// 路径校验请求在途标志，防止同一候选被重复提交校验。
  std::atomic<bool> plan_request_in_flight_{false};

  NavGoalHandle::SharedPtr nav_goal_handle_;
  FollowGoalHandle::SharedPtr follow_goal_handle_;
  /// 当前"有效"的 FollowPath 目标 id。
  ///
  /// 为什么必须有它：周期性重规划要换路径，只能再发一个 FollowPath 目标，
  /// 而 nav2 的 action server 是单目标语义 —— 新目标会 terminate_current()，
  /// 旧目标以 ABORTED 回到客户端。若把这种"被自己取代"当成控制器失败，
  /// 就会：重规划 -> 旧目标 ABORTED -> 判失败 -> 重试 -> 再发 -> 再抢占…
  /// 实测该自激循环产生 66 个终止结果，而 controller_server 真正 abort 只有 2 次。
  /// 所以结果回调必须先比对 goal_id，只认当前这一个。
  rclcpp_action::GoalUUID current_follow_goal_id_{};
  bool has_current_follow_goal_id_{false};
  DispatchMode dispatch_mode_{DispatchMode::kFollowPath};
  std::string follow_action_name_;
  std::string follow_controller_id_;
  std::string follow_goal_checker_id_;
  double replan_period_sec_{0.0};
  /// 跟踪期重规划策略，见 ReplanPolicy。
  ReplanPolicy replan_policy_{ReplanPolicy::kOnInvalid};
  /// kOnInvalid 策略下「检查当前路径是否还能用」的节拍(s)。
  /// 注意这只是**本地几何+栅格校验**，不发任何 action，代价是几百个采样点。
  double replan_check_period_sec_{0.0};
  /// 两次重规划请求之间的最小间隔(s)。防止判据在阈值附近抖动时连续换路径。
  double replan_min_interval_sec_{0.0};
  /// 当前路径的最大寿命(s)，>0 才启用。纯兜底：正常情况下路径不该因为「旧」而被换掉。
  double path_max_age_sec_{0.0};
  /// 机器人偏离当前路径多远就认为这条路径已经不描述它的处境(m)。
  double path_deviation_limit_m_{0.0};
  /// 当前在跟踪的那条路径（世界坐标顶点），用于剩余段校验与偏离度计算。
  std::vector<PlanarPoint> active_path_;
  rclcpp::Time active_path_time_;
  /// 上一次做「路径是否还能用」检查的时刻。
  rclcpp::Time last_replan_check_time_;
  /// 强制重规划标志：控制器中止后由 onFollowResult 置位，下一次检查必定触发换路径。
  /// 用显式标志而不是「把 last_replan_time_ 推到过去」——后者在 kOnInvalid 策略下
  /// 什么都不会发生，是个静默失效的写法。
  bool replan_forced_{false};
  /// 当前这条路径是否**已被判定不可通行**（剩余段穿占据/未知区）。
  ///
  /// 有它才能区分两种"重规划没换成"：
  ///   · 只是这次规划请求失败/超时 —— 当前路径还能走，沿用是对的；
  ///   · 当前路径已经判死、替代路径也不合法 —— 沿用就是明知走不通还往里顶。
  /// 实测后者的代价：机器人在原地顶了 17s，直到 progress checker 才救回来。
  bool active_path_impassable_{false};
  /// 连续「路径判死 + 重规划无有效替代」的次数。超限即放弃该目标、另选一个。
  int invalid_replan_count_{0};
  int max_invalid_replan_attempts_{0};
  /// FollowPath 中止后对**同一个目标**的重试次数上限。
  /// 对应默认行为树里 RecoveryNode number_of_retries="1" 所提供的能力：
  /// 进度停滞(Failed to make progress)在 BT 模式下会被吸收一次再算失败，
  /// follow_path 模式没有 BT，必须由本节点补上，否则每次停滞都直接判失败
  /// （实测：12 次下发全部因 Failed to make progress 判死、0 次收敛）。
  int follow_max_retries_{1};
  /// 当前目标已重试次数。
  int follow_retry_count_{0};
  /// 上次为「跟踪中重规划」发出规划请求的时刻。
  rclcpp::Time last_replan_time_;
  /// true 表示当前在途的规划请求是「跟踪中重规划」，而不是候选校验。
  /// 两者共用 plan_client_，靠这个标志区分 onPlanResult 该走哪条分支。
  bool plan_request_is_replan_{false};
  PlanGoalHandle::SharedPtr plan_goal_handle_;
  GoalCandidatePose active_goal_;
  bool has_active_goal_{false};
  rclcpp::Time nav_started_time_;
  rclcpp::Time plan_requested_time_;

  /// 本轮的候选队列与游标。
  std::vector<GoalCandidatePose> candidates_;
  std::size_t candidate_index_{0U};

  /// 抵达驻留计时起点（首次满足位置/朝向条件的时刻）。
  rclcpp::Time dwell_started_time_;
  bool dwell_active_{false};

  // ---- 计数器 ----
  /// 两个连续失败预算。**必须**用这个结构体而不是两个裸 int：
  /// 清零条件的归属是这里唯一出过 bug 的地方（共用一个计数器时上限结构上
  /// 不可达，实测空转 400s），已由 test_failure_budget 钉住，别再拆回裸 int。
  ExplorationFailureBudget failure_budget_;
  int nav_failure_count_{0};
  int auto_resume_count_{0};
  uint64_t goals_dispatched_{0U};
  uint64_t goals_succeeded_{0U};
  uint64_t candidates_rejected_{0U};

  // ---- 数据快照 ----
  /// SLAM 原始占据栅格。**只用于前沿搜索**：前沿的定义依赖「未知」这个状态，
  /// 代价地图被 obstacle_layer 清障刷过之后未知区不完整，拿它找前沿会漏区域。
  std::shared_ptr<GridMap> latest_map_;
  rclcpp::Time latest_map_time_;
  std::mutex map_mutex_;
  /// 全局代价地图（已转成三态 GridMap）。**只用于下发前校验**。
  std::shared_ptr<GridMap> latest_costmap_;
  rclcpp::Time latest_costmap_time_;
  std::mutex costmap_mutex_;
  double odom_speed_{0.0};
  rclcpp::Time latest_odom_time_;
  std::mutex odom_mutex_;

  /// 历史访问记录：被拒的候选点也记进来，让代价函数后续避开。
  std::vector<VisitRecord> visit_history_;
  std::size_t visit_history_limit_{50U};

  bool exploration_complete_{false};
  /// pause 服务请求的冻结标志，与异常导致的 kPaused 区分开。
  bool manually_paused_{false};

  // ---- 冷启动自举 ----
  BootstrapMode bootstrap_mode_{BootstrapMode::kRotate};
  std::string bootstrap_cmd_vel_topic_;
  std::string bootstrap_scan_topic_;
  /// 自举量转角所用的 frame。默认 odom（不依赖 SLAM），见 poseInFrame 说明。
  std::string bootstrap_yaw_frame_;
  double bootstrap_angular_vel_{0.0};
  double bootstrap_duration_sec_{0.0};
  double bootstrap_cmd_rate_hz_{0.0};
  double bootstrap_trigger_wait_sec_{0.0};
  double bootstrap_scan_timeout_sec_{0.0};
  double bootstrap_min_clearance_m_{0.0};
  /// 一次自举至少要转出多少角度才算「真的动了」(rad)。
  /// 必须 >= slam_toolbox 的 minimum_travel_heading（本项目 0.2），
  /// 否则转了也不会插入新扫描，地图照样不长。
  double bootstrap_min_yaw_delta_{0.0};
  int bootstrap_max_attempts_{0};
  /// 判定「地图内容足以做探索决策」的已知格数下限。
  std::size_t min_known_cells_for_decision_{0U};

  int bootstrap_count_{0};
  rclcpp::Time bootstrap_started_time_;
  double bootstrap_start_yaw_{0.0};
  bool bootstrap_start_yaw_valid_{false};
  /// 地图内容持续不可用/采不到候选的起始时刻，用于 bootstrap_trigger_wait_sec 计时。
  rclcpp::Time bootstrap_stall_since_;
  bool bootstrap_stall_active_{false};
  /// 最近一次自举的结论，进状态话题（禁止静默失败：被安全门拦下也必须能被看到）。
  std::string bootstrap_last_result_{"未执行"};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr bootstrap_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr bootstrap_cmd_timer_;

  // ---- 脱困参数 ----
  bool escape_enabled_{true};
  int escape_trigger_failures_{5};
  double escape_search_radius_m_{1.5};
  double escape_heading_tol_rad_{1.5708};
  double escape_linear_vel_{0.08};
  double escape_angular_vel_{0.20};
  double escape_arrive_tol_m_{0.05};
  double escape_align_tol_rad_{0.35};
  double escape_timeout_sec_{20.0};
  int escape_max_attempts_{3};
  int escape_clear_ticks_{5};
  double breadcrumb_window_sec_{60.0};
  double breadcrumb_sample_hz_{2.0};

  // ---- 脱困运行期状态 ----
  /// 连续「起点致命」计数。与 consecutive_invalid_ 分开：后者混了目标侧失败。
  int start_lethal_failures_{0};
  int escape_count_{0};
  rclcpp::Time escape_started_time_;
  PlanarPoint escape_target_{};
  bool escape_target_valid_{false};
  int escape_clear_streak_{0};
  EscapeCommand escape_cmd_{};
  std::string escape_last_result_{"未执行"};
  /// 来路轨迹。低频采样，按 breadcrumb_window_sec_ 裁剪。
  std::vector<Breadcrumb> breadcrumbs_;
  rclcpp::Time last_breadcrumb_time_;
  double latest_scan_min_range_{-1.0};
  rclcpp::Time latest_scan_time_;
  bool has_scan_{false};
  std::mutex scan_mutex_;
};

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__EXPLORATION_COORDINATOR_NODE_HPP_
