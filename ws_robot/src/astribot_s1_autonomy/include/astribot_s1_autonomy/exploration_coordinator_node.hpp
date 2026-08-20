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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "astribot_s1_autonomy/exploration_state.hpp"
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
  void tickGenNextPoint();
  void tickValidating();
  void tickNavigating();
  void tickArrived();
  void tickPaused();
  void tickCompleted();

  // ---------------- 数据回调 ----------------
  void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

  // ---------------- 前置条件 ----------------
  /// 地图是否可用（非空、自洽、未超时）。不可用时拦截生成逻辑。
  bool mapReady(std::string & why);
  /// 里程计是否新鲜。不新鲜视为定位/里程计丢失，必须冻结目标发布。
  bool odomReady(std::string & why);
  /// 机器人位姿是否可用。取不到即视为定位丢失。
  bool robotPose(double & x, double & y, double & yaw, std::string & why);

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
  std::string odom_topic_;
  std::string state_topic_;
  std::string complete_topic_;
  std::string current_goal_topic_;
  std::string nav_action_name_;
  std::string plan_action_name_;
  std::string map_frame_;
  std::string robot_base_frame_;
  std::string planner_id_;

  // ---- 参数：节拍与超时 ----
  double control_period_sec_{0.0};
  double tf_timeout_sec_{0.0};
  double map_timeout_sec_{0.0};
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
  int max_sample_failures_{0};
  int max_consecutive_nav_failures_{0};
  double pause_cooldown_sec_{0.0};
  int max_auto_resume_attempts_{0};

  // ---- 算法 ----
  FrontierSearch search_;
  FrontierSearchParams search_params_;
  PathValidator validator_;

  // ---- ROS 句柄 ----
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
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
  int sample_failure_count_{0};
  int nav_failure_count_{0};
  int auto_resume_count_{0};
  uint64_t goals_dispatched_{0U};
  uint64_t goals_succeeded_{0U};
  uint64_t candidates_rejected_{0U};

  // ---- 数据快照 ----
  std::shared_ptr<GridMap> latest_map_;
  rclcpp::Time latest_map_time_;
  std::mutex map_mutex_;
  double odom_speed_{0.0};
  rclcpp::Time latest_odom_time_;
  std::mutex odom_mutex_;

  /// 历史访问记录：被拒的候选点也记进来，让代价函数后续避开。
  std::vector<VisitRecord> visit_history_;
  std::size_t visit_history_limit_{50U};

  bool exploration_complete_{false};
  /// pause 服务请求的冻结标志，与异常导致的 kPaused 区分开。
  bool manually_paused_{false};
};

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__EXPLORATION_COORDINATOR_NODE_HPP_
