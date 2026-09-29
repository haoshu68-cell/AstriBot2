#include "astribot_navigation_zones/client.hpp"
// Copyright 2026 Astribot.
#ifndef ASTRIBOT_S1_AUTONOMY__EXPLORATION_COORDINATOR_NODE_HPP_
#define ASTRIBOT_S1_AUTONOMY__EXPLORATION_COORDINATOR_NODE_HPP_

#include <astribot_operator_msgs/srv/exploration_command.hpp>
#include <nlohmann/json.hpp>
#include <map>

#include <atomic>
#include <chrono>
#include "astribot_s1_autonomy/exploration_progress.hpp"
#include "astribot_s1_autonomy/exploration_candidates.hpp"
#include <future>
#include <deque>
#include <chrono>
#include "astribot_navigation_msgs/msg/navigation_execution_status.hpp"
#include "astribot_navigation_msgs/msg/navigation_policy_status.hpp"
#include "astribot_s1_autonomy/navigation_readiness.hpp"
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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

  /// 一个待校验/待执行的目标。
  struct GoalCandidatePose
  {
    double frontier_x{0.0};
    double frontier_y{0.0};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double euclidean_distance{0.0};
    std::size_t cluster_index{0};
    double cost{0.0};
  };

  bool require_zones_{false};astribot_navigation_zones::Gate zones_;
  std::shared_ptr<GridMap> raw_zone_map_,filtered_zone_source_;
  std::string zone_token_,filtered_zone_token_;bool zones_present_{false};
  bool refreshZoneMap();
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
  bool require_fixed_envelope_{false};
  astribot_navigation_msgs::msg::NavigationEnvelopeV2::ConstSharedPtr fixed_envelope_;
  rclcpp::Subscription<astribot_navigation_msgs::msg::NavigationEnvelopeV2>::SharedPtr fixed_envelope_sub_;

  bool loadParameters(std::string & error);


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

  /// 地图是否「已经能用来做探索决策」。
  bool mapUsable(const GridMap & map, std::size_t & known_cells) const;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
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
  using OperatorCommand = astribot_operator_msgs::srv::ExplorationCommand;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operator_status_pub_, operator_events_pub_;
  rclcpp::Service<OperatorCommand>::SharedPtr operator_command_;
  std::string operator_boot_, operator_signature_, transition_reason_;
  uint64_t operator_revision_{0};
  std::map<std::string, std::pair<std::string, OperatorCommand::Response>> operator_requests_;
  nlohmann::json operatorStatus();
  void applyOperatorOperation(const std::string &, std_srvs::srv::Trigger::Response &);

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr plan_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr finalize_client_;
  bool finalize_on_completion_{true};
  bool session_ending_{false};
  bool finalize_requested_{false};
  std::string session_end_reason_;
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

  /// 分别累计采样与校验失败；重采样不得清除校验失败预算。
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
  /// 仅在显式恢复/暂停或真正进入目标生成后置为 true。
  /// 防止尚未建立探索会话时错误开放“取消并保存”。
  bool session_started_{false};

  /// 地图内容足以做探索决策的已知格数下限。
  std::size_t min_known_cells_for_decision_{0U};
};

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__EXPLORATION_COORDINATOR_NODE_HPP_
