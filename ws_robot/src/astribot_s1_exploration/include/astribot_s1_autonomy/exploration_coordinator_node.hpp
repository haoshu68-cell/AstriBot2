// Copyright 2026 Astribot.
#ifndef ASTRIBOT_S1_AUTONOMY__EXPLORATION_COORDINATOR_NODE_HPP_
#define ASTRIBOT_S1_AUTONOMY__EXPLORATION_COORDINATOR_NODE_HPP_

#include <atomic>
#include <chrono>
#include "astribot_s1_autonomy/exploration_progress.hpp"
#include "astribot_s1_autonomy/exploration_candidates.hpp"
#include <future>
#include <deque>
#include <chrono>
#include "astribot_navigation_msgs/msg/navigation_execution_status.hpp"
#include "astribot_navigation_msgs/msg/navigation_policy_status.hpp"
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "astribot_s1_autonomy/escape_logic.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
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
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using PlanGoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

  /// 下发方式。
  enum class DispatchMode
  {
    kNavigateToPose,
    kFollowPath,
  };

  /// 跟踪期的重规划策略。
  enum class ReplanPolicy
  {
    kOnInvalid,
    kPeriodic,
  };

  /// 冷启动自举的动作方式。
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
    double euclidean_distance{0.0};
    std::size_t cluster_index{0};
    double cost{0.0};
  };

  void declareParameters();
  struct EvaluatedCandidate {GoalCandidatePose goal; std::vector<PlanarPoint> path;};
  std::vector<EvaluatedCandidate> evaluated_candidates_;
  bool dispatchBestValidatedGoal();
  void recordCandidateFailure(double x, double y, const std::string & reason, bool map_dependent);
  CandidateFailureMemory candidate_failures_;
  double failure_cooldown_sec_{20.0}, failure_radius_m_{0.35};
  double selection_budget_sec_{8.0}, path_turn_weight_{0.2};
  std::chrono::steady_clock::time_point selection_started_;
  std::size_t cooldown_filtered_{0};
  std::future<FrontierSearch::Result> search_future_;
  std::shared_ptr<std::atomic<bool>> search_cancel_;
  std::shared_ptr<GridMap> search_map_;
  uint64_t map_revision_{0};  // protected by map_mutex_, increments on content changes only
  uint64_t completed_map_revision_{0};
  std::chrono::steady_clock::time_point search_started_;
  double search_start_x_{0.0}, search_start_y_{0.0};
  double search_max_age_sec_{2.0}, search_max_displacement_m_{0.25};
  CompletionTracker completion_tracker_;
  unsigned int completion_observations_{3};
  double completion_stable_sec_{2.0};
  std::string progress_detail_{"WAITING_INPUT"};
  std::size_t raw_frontiers_{0}, reachable_frontiers_{0}, unresolved_unknown_{0};
  uint64_t search_discarded_{0}, search_revalidated_{0};
  uint64_t search_epoch_{0};
  uint64_t search_request_epoch_{0};
  uint64_t plan_epoch_{0};
  uint64_t nav_epoch_{0};
  void applyTaskPreemption(const std::string & task_id);
  std::deque<std::pair<std::string,std::string>> task_events_;
  std::string current_task_id_;
  std::string policy_failure_reason_;
  bool pending_nav_failure_{false};
  std::chrono::steady_clock::time_point failure_classification_at_;
  rclcpp::Subscription<astribot_navigation_msgs::msg::NavigationExecutionStatus>::SharedPtr execution_status_sub_;
  rclcpp::Subscription<astribot_navigation_msgs::msg::NavigationPolicyStatus>::SharedPtr policy_status_sub_;

  bool loadParameters(std::string & error);


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

  /// 是否应当进入自举，并给出原因。只在 IDLE / GEN_NEXT_POINT 里调用。
  bool shouldBootstrap(const std::string & context, std::string & why);
  /// 进入自举：记录起始朝向、清零计时。
  void beginBootstrap(const std::string & why);
  /// 自举安全门。返回 false 时必须立刻停车并显式告警（绝不静默继续转）。
  /// 向底盘发一帧速度指令。zero=true 时发全零（用于停车兜底）。
  void publishBootstrapCmd(bool zero);
  /// 自举专用定时器回调：以 bootstrap_cmd_rate_hz 持续重发速度。
  ///
  /// 为什么不能靠 controlTick 发：底盘 cmd_vel_timeout_sec=0.5，而控制节拍是 0.5s，
  /// 正好卡在超时边界上，速度会被反复归零 —— 必须独立高频重发。


  /// 读机器人所在格在 costmap(规划器视角) 与 /map(物理真值) 上的三态值。

  /// 判断是否该进 ESCAPE。返回结论并填原因；调用方负责状态跃迁与告警。

  /// 选一个脱困目标：先来路、再最近可规划格。失败返回 false。

  /// ESCAPE 状态每拍。
  void tickEscape();

  /// 脱困专用速度发布（车体系，走 bootstrap_cmd_pub_ 同一个出口）。

  /// 记一个面包屑（低频采样机器人位姿），并裁掉过期的。
  /// 地图是否「已经能用来做探索决策」。
  bool mapUsable(const GridMap & map, std::size_t & known_cells) const;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  /// 自举安全门用的激光回调。**必须用 BEST_EFFORT 订阅**：
  /// 本项目已经在探针脚本上踩过 —— RELIABLE 订阅 BEST_EFFORT 发布方一帧都收不到，
  /// 而当时的代码把「没收到数据」当成了「前方无障碍」，结果盲走 3m。
  /// 全局代价地图回调。这是**下发前校验**所用的栅格图，
  /// 与 planner_server 规划时所用的是同一张，避免两层判据互相锁死。
  /// 详细机制见 costmap_adapter.hpp 文件头。
  void costmapCallback(const nav2_msgs::msg::Costmap::ConstSharedPtr & msg);

  /// 地图是否可用（非空、自洽、未超时）。不可用时拦截生成逻辑。
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

  /// 跑前沿搜索，产出按代价升序排列的候选点。
  std::vector<GoalCandidatePose> generateCandidates(
    const FrontierSearch::Result & result);
  /// 对当前候选队列的队首发起路径校验（异步）。
  void requestPlanForCurrentCandidate();
  void onPlanGoalResponse(uint64_t epoch, const PlanGoalHandle::SharedPtr & handle);
  /// ComputePathToPose 结果回调：做逐点未知区校验，通过则下发导航目标。
  void onPlanResult(uint64_t epoch, const PlanGoalHandle::WrappedResult & result);
  /// 丢弃当前候选并推进到下一个；候选耗尽时按需转 PAUSED。
  void rejectCurrentCandidate(const std::string & reason, bool map_dependent = false);
  /// 本轮候选全部被拒后的收尾：按限次决定重新采样还是转 PAUSED。
  void onCandidatesExhausted(const std::string & reason);
  /// 清空本轮候选队列与游标。
  void resetCycleState();
  /// 记一次「去过/试过」，让代价函数后续避开这个位置。
  void recordVisit(double x, double y);

  /// 下发导航目标。**这是全节点唯一一处下发点**，且只允许从 kValidating 调用。
  void dispatchNavGoal(const GoalCandidatePose & goal);
  void onNavGoalResponse(uint64_t epoch, const NavGoalHandle::SharedPtr & handle);
  void onNavResult(uint64_t epoch, const NavGoalHandle::WrappedResult & result);
  /// 取消在途导航目标（若有）。用于超时、定位丢失、人工暂停。
  void cancelActiveNavGoal(const std::string & reason);
  /// 记录一次导航失败，达上限则转 PAUSED。
  void registerNavFailure(const std::string & reason);

  void publishState();
  void publishComplete(bool done);

  void onPauseService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onResumeService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::string map_topic_;
  bool map_transient_local_{true};
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

  double control_period_sec_{0.0};
  double tf_timeout_sec_{0.0};
  double map_timeout_sec_{0.0};
  double costmap_timeout_sec_{0.0};
  double odom_timeout_sec_{0.0};
  double plan_timeout_sec_{0.0};
  double nav_timeout_sec_{0.0};

  double unknown_clearance_radius_{0.0};

  double arrival_xy_tolerance_{0.0};
  double arrival_yaw_tolerance_{0.0};
  bool check_yaw_{true};
  double dwell_time_sec_{0.0};
  double settle_speed_{0.0};

  int max_candidates_per_cycle_{0};
  int max_consecutive_nav_failures_{0};
  double pause_cooldown_sec_{0.0};
  int max_auto_resume_attempts_{0};

  FrontierSearch search_;
  FrontierSearchParams search_params_;
  PathValidator validator_;
  CostmapAdapterParams costmap_params_;
  /// 下发前校验是否使用全局代价地图（默认 true）。
  /// 置 false 会退回「用 /map 校验」的旧行为——那是已知会锁死探索的配置，
  /// 只保留作对比排查用，正常运行不要关。
  bool use_costmap_for_validation_{true};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr plan_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// 定时器单独一个互斥回调组：前沿搜索有几十毫秒级开销，
  /// 放进独立组后即使用多线程执行器，也不会把 action 结果回调挤在后面排队。
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  /// 订阅/服务/action 客户端共用一个组，保证数据快照与状态查询彼此串行。
  rclcpp::CallbackGroup::SharedPtr io_cb_group_;
  rclcpp::CallbackGroup::SharedPtr command_cb_group_;

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
  /// 当前"有效"的 FollowPath 目标 id。
  rclcpp_action::GoalUUID current_follow_goal_id_{};
  bool has_current_follow_goal_id_{false};
  DispatchMode dispatch_mode_{DispatchMode::kNavigateToPose};
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

  /// 两个连续失败预算。**必须**用这个结构体而不是两个裸 int：
  /// 清零条件的归属是这里唯一出过 bug 的地方（共用一个计数器时上限结构上
  /// 不可达，实测空转 400s），已由 test_failure_budget 钉住，别再拆回裸 int。
  ExplorationFailureBudget failure_budget_;
  int nav_failure_count_{0};
  int auto_resume_count_{0};
  uint64_t goals_dispatched_{0U};
  uint64_t goals_succeeded_{0U};
  uint64_t candidates_rejected_{0U};

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

  BootstrapMode bootstrap_mode_{BootstrapMode::kDisabled};
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

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  bool escape_enabled_{false};
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

  /// 连续「起点致命」计数。与 consecutive_invalid_ 分开：后者混了目标侧失败。
  int start_lethal_failures_{0};
  /// **连续**失败的脱困次数，不是累计尝试次数。与 escape_max_attempts_ 比较。
  /// 必须在「成功出带」和「人工 ~/resume」两处清零，否则 escape_max_attempts_
  /// 变成整个运行期的总配额：沿途多个窄处但完全走得通的长路径会被误判成
  /// 「不可行」，且第 N+1 次起永久拒绝，提示操作员去调一个救不了的 ~/resume。
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
